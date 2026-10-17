/*
 * ESP32-P4 high-speed DWC2 hardware passthrough.
 *
 * Linux owns the controller registers and all USB protocol state.  ESP-IDF
 * only keeps the UTMI PHY clocked and reflects the physical interrupt into
 * the guest PLIC.  Transfer descriptors and payloads contain identity-mapped
 * PSRAM addresses, so the DWC2 payload path never passes through this file.
 * Only controller register accesses and interrupt delivery cross the emulator.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include "rv32_diagnostics.h"

#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_rom_sys.h"
#include "esp_cache.h"
#include "esp_memory_utils.h"
#include <string.h>
#include "esp_timer.h"
#include "esp_private/usb_phy.h"
#include "hal/usb_dwc_hal.h"
#include "soc/interrupts.h"
#include "soc/usb_dwc_struct.h"

#include "dwc2_passthrough.h"
#include "rv32_host_wake.h"

#if defined(CONFIG_RV32_USB_FULL_SPEED_ONLY) && \
	CONFIG_RV32_USB_FULL_SPEED_ONLY
#define DWC2_PHY_SPEED_LIMIT USB_PHY_SPEED_FULL
#define DWC2_PHY_SPEED_LIMIT_NAME "full-speed"
#else
#define DWC2_PHY_SPEED_LIMIT USB_PHY_SPEED_HIGH
#define DWC2_PHY_SPEED_LIMIT_NAME "high-speed"
#endif

static usb_phy_handle_t usb_phy;
static usb_dwc_hal_context_t usb_hal;
static intr_handle_t usb_interrupt;
static atomic_bool guest_irq_pending;
static atomic_uint irq_physical;
static atomic_uint irq_forwarded;
static atomic_uint irq_completed;
static int64_t last_sof_forwarded_us;

/* Host-channel tracking.  Guest RAM lives in PSRAM behind the host cache and
 * the controller's DMA writes bypass it.  The guest only issues cache ops for
 * streaming mappings, so coherent buffers (USB HID input) would otherwise be
 * read stale.  Invalidate the received span whenever an IN channel completes,
 * and keep counters for the stall reporter. */
#define DWC2_CHANNEL_BASE   0x500u
#define DWC2_CHANNEL_STRIDE 0x20u
#define DWC2_CHANNELS       16u
#define DWC2_HCINT_XFERCOMPL (1u << 0)
#define DWC2_HCINT_STALL     (1u << 3)
#define DWC2_HCINT_NAK       (1u << 4)
#define DWC2_HCINT_XACTERR   (1u << 7)
#define DWC2_HCINT_BBLERR    (1u << 8)
#define DWC2_HCINT_FRMOVRUN  (1u << 9)
#define DWC2_HCINT_DATATGL   (1u << 10)
#define DWC2_CACHE_LINE      CONFIG_CACHE_L2_CACHE_LINE_SIZE

struct dwc2_channel_track {
	uint32_t hcchar;
	uint32_t hctsiz;
	uint32_t hcdma;
};
static struct dwc2_channel_track channel_track[DWC2_CHANNELS];
static struct dwc2_passthrough_channel_stats channel_stats;

static void dwc2_track_hcint(uint32_t channel, uint32_t hcint)
{
	struct dwc2_channel_track *track = &channel_track[channel];
	uint32_t programmed = track->hctsiz & 0x7ffffu;
	uint32_t remaining = USB_DWC_HS.host_chans[channel].hctsiz_reg.val &
			     0x7ffffu;
	bool in = (track->hcchar >> 15) & 1u;

	if (hcint & DWC2_HCINT_XFERCOMPL) {
		channel_stats.completions++;
		if (in && programmed > remaining) {
			uint32_t received = programmed - remaining;
			uintptr_t start = track->hcdma & ~(uintptr_t)(DWC2_CACHE_LINE - 1u);
			uintptr_t end = (track->hcdma + received + DWC2_CACHE_LINE - 1u) &
					~(uintptr_t)(DWC2_CACHE_LINE - 1u);

			channel_stats.in_bytes += received;
			channel_stats.in_completions++;
			if (esp_ptr_external_ram((void *)track->hcdma) &&
			    esp_cache_msync((void *)start, end - start,
					    ESP_CACHE_MSYNC_FLAG_DIR_M2C |
					    ESP_CACHE_MSYNC_FLAG_INVALIDATE) == ESP_OK)
				channel_stats.invalidations++;
			channel_stats.last_in_ep = (track->hcchar >> 11) & 0xfu;
			channel_stats.last_in_len = received;
			memcpy(channel_stats.last_in_data, (const void *)track->hcdma,
			       received < sizeof(channel_stats.last_in_data) ?
				       received : sizeof(channel_stats.last_in_data));
		}
	}
	if (hcint & DWC2_HCINT_NAK)
		channel_stats.naks++;
	if (hcint & DWC2_HCINT_STALL)
		channel_stats.stalls++;
	if (hcint & (DWC2_HCINT_XACTERR | DWC2_HCINT_BBLERR))
		channel_stats.errors++;
	if (hcint & DWC2_HCINT_DATATGL)
		channel_stats.toggle_errors++;
	if (hcint & DWC2_HCINT_FRMOVRUN)
		channel_stats.frame_overruns++;
}

void dwc2_passthrough_channel_stats(struct dwc2_passthrough_channel_stats *stats)
{
	*stats = channel_stats;
}
#if CONFIG_RV32_DEBUG_USB
static atomic_uint physical_irq_count;
static bool guest_read_seen;
static bool guest_write_seen;
static bool status_valid;
static bool no_link_warning_printed;
static unsigned int powered_no_link_polls;
static uint32_t last_hprt;
static uint32_t last_gahbcfg;
static uint32_t last_gintmsk;
static bool last_guest_read_seen;
static bool last_guest_write_seen;
#endif
static bool initialized;

#if CONFIG_RV32_DEBUG_USB
static const char *dwc2_port_speed_name(uint32_t speed)
{
	switch (speed) {
	case USB_DWC_SPEED_HIGH:
		return "high-speed";
	case USB_DWC_SPEED_FULL:
		return "full-speed";
	case USB_DWC_SPEED_LOW:
		return "low-speed";
	default:
		return "unknown";
	}
}

static const char *dwc2_line_state_name(uint32_t line_state)
{
	switch (line_state) {
	case 0:
		return "SE0";
	case 1:
		return "D+";
	case 2:
		return "D-";
	case 3:
		return "SE1-invalid";
	default:
		return "unknown";
	}
}

#endif

/* GINTSTS bit 3; written back as 1 to clear. */
#define DWC2_GINTSTS_SOF (1u << 3)
/* Linux's dwc2 host enables the start-of-frame interrupt while any periodic
 * endpoint (a keyboard or mouse) exists and uses it only to queue transfers
 * whose frame has come.  Every frame, 1000 times a second at full speed, is
 * far too many emulated interrupts.  Each forwarded interrupt costs the guest
 * about 1.2 ms of kernel-mode emulation, so forward one in sixteen and poll
 * HID devices every 16 ms (usbhid.kbpoll and usbhid.mousepoll in the
 * bootargs); an idle mouse then takes a sixth of the CPU instead of a third. */
#define DWC2_SOF_FORWARD_INTERVAL_US 16000

static void IRAM_ATTR dwc2_physical_isr(void *argument)
{
	(void)argument;
	atomic_fetch_add_explicit(&irq_physical, 1u, memory_order_relaxed);
#if CONFIG_RV32_DEBUG_USB
	atomic_fetch_add_explicit(&physical_irq_count, 1,
				 memory_order_relaxed);
#endif
	if ((USB_DWC_HS.gintsts_reg.val & USB_DWC_HS.gintmsk_reg.val) ==
	    DWC2_GINTSTS_SOF) {
		int64_t now = esp_timer_get_time();

		if (now - last_sof_forwarded_us < DWC2_SOF_FORWARD_INTERVAL_US) {
			USB_DWC_HS.gintsts_reg.val = DWC2_GINTSTS_SOF;
			return;
		}
		last_sof_forwarded_us = now;
	}
	atomic_fetch_add_explicit(&irq_forwarded, 1u, memory_order_relaxed);
	atomic_store_explicit(&guest_irq_pending, true, memory_order_release);
	rv32_host_wake_request();

	/*
	 * DWC2 uses a level interrupt.  Leave the source disconnected until
	 * Linux clears GINTSTS/HCINT/HPRT and completes its PLIC claim; otherwise
	 * the ESP CPU would spin in the same physical ISR.
	 */
	esp_intr_disable(usb_interrupt);
}

int dwc2_passthrough_init(void)
{
	esp_err_t error;

	if (initialized)
		return 0;

	if ((uintptr_t)&USB_DWC_HS != DWC2_GUEST_BASE) {
		printf("ERROR: DWC2 HS register base is %p, expected 0x%08x\n",
		       &USB_DWC_HS, DWC2_GUEST_BASE);
		return -1;
	}

	error = esp_intr_alloc(ETS_USB_OTG_INTR_SOURCE,
			       ESP_INTR_FLAG_LEVEL1 |
			       ESP_INTR_FLAG_INTRDISABLED,
			       dwc2_physical_isr, NULL, &usb_interrupt);
	if (error != ESP_OK) {
		printf("ERROR: DWC2 interrupt allocation failed: %s\n",
		       esp_err_to_name(error));
		return -1;
	}

	const usb_phy_config_t phy_config = {
		.controller = USB_PHY_CTRL_OTG,
		.target = USB_PHY_TARGET_UTMI,
		.otg_mode = USB_OTG_MODE_HOST,
		.otg_speed = DWC2_PHY_SPEED_LIMIT,
		.ext_io_conf = NULL,
		.otg_io_conf = NULL,
	};

	error = usb_new_phy(&phy_config, &usb_phy);
	if (error != ESP_OK) {
		printf("ERROR: USB UTMI PHY initialization failed: %s\n",
		       esp_err_to_name(error));
		esp_intr_free(usb_interrupt);
		usb_interrupt = NULL;
		return -1;
	}

	/* Apply the ESP32-P4 UTMI/host-mode quirks before Linux takes over. */
	usb_dwc_hal_init(&usb_hal, 0);
	atomic_store_explicit(&guest_irq_pending, false, memory_order_relaxed);
#if CONFIG_RV32_DEBUG_USB
	atomic_store_explicit(&physical_irq_count, 0, memory_order_relaxed);
#endif

	error = esp_intr_enable(usb_interrupt);
	if (error != ESP_OK) {
		printf("ERROR: DWC2 interrupt enable failed: %s\n",
		       esp_err_to_name(error));
		usb_dwc_hal_deinit(&usb_hal);
		usb_del_phy(usb_phy);
		usb_phy = NULL;
		esp_intr_free(usb_interrupt);
		usb_interrupt = NULL;
		return -1;
	}

	initialized = true;
	RV32_DIAG("DWC2 passthrough ready: registers 0x%08x, core 0x%08lx, "
	       "limit=%s, port=%s\n",
	       DWC2_GUEST_BASE, (unsigned long)USB_DWC_HS.gsnpsid_reg.val,
	       DWC2_PHY_SPEED_LIMIT_NAME,
	       USB_DWC_HS.hprt_reg.prtconnsts ? "connected" : "disconnected");
	RV32_DIAG("USB ownership: Linux DWC2 (ESP-IDF USB Host and TinyUSB are "
	       "not installed)\n");
	RV32_DIAG("USB 2.0 UTMI PHY: dedicated DM=49 DP=50; connector role and "
	       "VBUS sourcing are provided by board hardware\n");
	return 0;
}

void dwc2_passthrough_service(void)
{
#if CONFIG_RV32_DEBUG_USB
	uint32_t hprt;
	uint32_t gahbcfg;
	uint32_t gintmsk;
	uint32_t gintsts;
	bool changed;

	if (!initialized)
		return;

	hprt = USB_DWC_HS.hprt_reg.val;
	gahbcfg = USB_DWC_HS.gahbcfg_reg.val;
	gintmsk = USB_DWC_HS.gintmsk_reg.val;
	gintsts = USB_DWC_HS.gintsts_reg.val;
	changed = !status_valid || hprt != last_hprt ||
		gahbcfg != last_gahbcfg || gintmsk != last_gintmsk ||
		guest_read_seen != last_guest_read_seen ||
		guest_write_seen != last_guest_write_seen;

	if (changed) {
		printf("USB DWC2 state: linux=%s%s mode=%s power=%u "
		       "connected=%u enabled=%u line=%s(%lu) speed=%s(%lu) "
		       "hprt=%08lx gahbcfg=%08lx gintmsk=%08lx "
		       "gintsts=%08lx irq=%u\n",
		       guest_read_seen ? "probed" : "not-probed",
		       guest_write_seen ? "/configured" : "",
		       USB_DWC_HS.gintsts_reg.curmod ? "host" : "device",
		       USB_DWC_HS.hprt_reg.prtpwr,
		       USB_DWC_HS.hprt_reg.prtconnsts,
		       USB_DWC_HS.hprt_reg.prtena,
		       dwc2_line_state_name(USB_DWC_HS.hprt_reg.prtlnsts),
		       (unsigned long)USB_DWC_HS.hprt_reg.prtlnsts,
		       dwc2_port_speed_name(USB_DWC_HS.hprt_reg.prtspd),
		       (unsigned long)USB_DWC_HS.hprt_reg.prtspd,
		       (unsigned long)hprt, (unsigned long)gahbcfg,
		       (unsigned long)gintmsk, (unsigned long)gintsts,
		       atomic_load_explicit(&physical_irq_count,
					    memory_order_relaxed));

		last_hprt = hprt;
		last_gahbcfg = gahbcfg;
		last_gintmsk = gintmsk;
		last_guest_read_seen = guest_read_seen;
		last_guest_write_seen = guest_write_seen;
		status_valid = true;
	}

	if (guest_write_seen && USB_DWC_HS.hprt_reg.prtpwr &&
	    !USB_DWC_HS.hprt_reg.prtconnsts) {
		if (powered_no_link_polls < 10)
			powered_no_link_polls++;
		if (powered_no_link_polls == 10 && !no_link_warning_printed) {
			printf("USB DWC2 has enabled root-port power, but the PHY "
			       "still sees no device pull-up; check connector VBUS, "
			       "CC role resistors, and the dedicated DM/DP path\n");
			no_link_warning_printed = true;
		}
	} else {
		powered_no_link_polls = 0;
		if (USB_DWC_HS.hprt_reg.prtconnsts)
			no_link_warning_printed = false;
	}
#endif
}

bool dwc2_passthrough_contains(uint32_t address, size_t width)
{
	return width != 0 && width <= sizeof(uint32_t) &&
		address >= DWC2_GUEST_BASE &&
		address - DWC2_GUEST_BASE <= DWC2_GUEST_SIZE - width;
}

uint32_t dwc2_passthrough_load(uint32_t address, size_t width)
{
	uint32_t aligned_address = address & ~3u;
	uint32_t shift = (address & 3u) * 8u;
	uint32_t value;

	if (!dwc2_passthrough_contains(address, width))
		return 0;
#if CONFIG_RV32_DEBUG_USB
	guest_read_seen = true;
#endif

	value = *(volatile uint32_t *)(uintptr_t)aligned_address;
	{
		uint32_t offset = aligned_address - DWC2_GUEST_BASE;

		if (offset >= DWC2_CHANNEL_BASE &&
		    offset < DWC2_CHANNEL_BASE + DWC2_CHANNELS * DWC2_CHANNEL_STRIDE &&
		    (offset & (DWC2_CHANNEL_STRIDE - 1u)) == 0x8u)
			dwc2_track_hcint((offset - DWC2_CHANNEL_BASE) /
						 DWC2_CHANNEL_STRIDE, value);
	}
	if (width == 1)
		return (value >> shift) & 0xffu;
	if (width == 2)
		return (value >> shift) & 0xffffu;
	return value;
}

void dwc2_passthrough_store(uint32_t address, uint32_t value, size_t width)
{
	uint32_t aligned_address = address & ~3u;
	volatile uint32_t *reg;
	uint32_t shift;
	uint32_t mask;

	if (!dwc2_passthrough_contains(address, width))
		return;
#if CONFIG_RV32_DEBUG_USB
	guest_write_seen = true;
#endif

	reg = (volatile uint32_t *)(uintptr_t)aligned_address;
	{
		uint32_t offset = aligned_address - DWC2_GUEST_BASE;

		if (offset >= DWC2_CHANNEL_BASE &&
		    offset < DWC2_CHANNEL_BASE + DWC2_CHANNELS * DWC2_CHANNEL_STRIDE &&
		    width == sizeof(uint32_t)) {
			struct dwc2_channel_track *track =
				&channel_track[(offset - DWC2_CHANNEL_BASE) /
					       DWC2_CHANNEL_STRIDE];

			switch (offset & (DWC2_CHANNEL_STRIDE - 1u)) {
			case 0x0u: track->hcchar = value; break;
			case 0x10u: track->hctsiz = value; break;
			case 0x14u: track->hcdma = value; break;
			default: break;
			}
		}
	}
	if (width == sizeof(uint32_t)) {
		*reg = value;
		return;
	}

	/* DWC2 is a 32-bit register interface; retain defensive subword support. */
	shift = (address & 3u) * 8u;
	mask = (width == 1 ? 0xffu : 0xffffu) << shift;
	*reg = (*reg & ~mask) | ((value << shift) & mask);
}

bool dwc2_passthrough_irq_pending(void)
{
	return atomic_load_explicit(&guest_irq_pending, memory_order_acquire);
}

void dwc2_passthrough_irq_complete(void)
{
	if (!initialized)
		return;

	if (atomic_exchange_explicit(&guest_irq_pending, false,
				     memory_order_acq_rel)) {
		atomic_fetch_add_explicit(&irq_completed, 1u,
					  memory_order_relaxed);
		esp_intr_enable(usb_interrupt);
	}
}

void dwc2_passthrough_dump(void)
{
	uint32_t channels;

	if (!initialized)
		return;
	channels = USB_DWC_HS.ghwcfg2_reg.numhstchnl + 1u;
	if (channels > 16u)
		channels = 16u;
	esp_rom_printf("[USB] gintsts=%08x gintmsk=%08x gahbcfg=%08x hprt=%08x "
		       "hfnum=%08x haint=%08x haintmsk=%08x\r\n",
		       (unsigned)USB_DWC_HS.gintsts_reg.val,
		       (unsigned)USB_DWC_HS.gintmsk_reg.val,
		       (unsigned)USB_DWC_HS.gahbcfg_reg.val,
		       (unsigned)USB_DWC_HS.hprt_reg.val,
		       (unsigned)USB_DWC_HS.hfnum_reg.val,
		       (unsigned)USB_DWC_HS.haint_reg.val,
		       (unsigned)USB_DWC_HS.haintmsk_reg.val);
	for (uint32_t i = 0; i < channels; i++) {
		uint32_t hcchar = USB_DWC_HS.host_chans[i].hcchar_reg.val;
		uint32_t hcint = USB_DWC_HS.host_chans[i].hcint_reg.val;

		if (hcchar == 0u && hcint == 0u)
			continue;
		esp_rom_printf("[USB] ch%u hcchar=%08x hcint=%08x hcintmsk=%08x "
			       "hctsiz=%08x hcdma=%08x\r\n", (unsigned)i,
			       (unsigned)hcchar, (unsigned)hcint,
			       (unsigned)USB_DWC_HS.host_chans[i].hcintmsk_reg.val,
			       (unsigned)USB_DWC_HS.host_chans[i].hctsiz_reg.val,
			       (unsigned)USB_DWC_HS.host_chans[i].hcdma_reg.val);
	}
}

void dwc2_passthrough_stats(struct dwc2_passthrough_stats *stats)
{
	stats->physical = atomic_load_explicit(&irq_physical,
					       memory_order_relaxed);
	stats->forwarded = atomic_load_explicit(&irq_forwarded,
						memory_order_relaxed);
	stats->completed = atomic_load_explicit(&irq_completed,
						memory_order_relaxed);
	stats->pending = atomic_load_explicit(&guest_irq_pending,
					      memory_order_relaxed);
	stats->gintsts = initialized ? USB_DWC_HS.gintsts_reg.val : 0u;
	stats->gintmsk = initialized ? USB_DWC_HS.gintmsk_reg.val : 0u;
	stats->hprt = initialized ? USB_DWC_HS.hprt_reg.val : 0u;
}
