/*
 * NOT USED!! 
 *
 * ESP32-P4 high-speed DWC2 hardware passthrough.
 *
 * Linux owns the controller registers and all USB protocol state.  ESP-IDF
 * only keeps the UTMI PHY clocked and reflects the physical interrupt into
 * the guest PLIC.  Transfer descriptors and payloads contain identity-mapped
 * PSRAM addresses, so the DWC2 data path never passes through this file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_private/usb_phy.h"
#include "hal/usb_dwc_hal.h"
#include "soc/interrupts.h"
#include "soc/usb_dwc_struct.h"

#include "dwc2_passthrough.h"

static usb_phy_handle_t usb_phy;
static usb_dwc_hal_context_t usb_hal;
static intr_handle_t usb_interrupt;
static atomic_bool guest_irq_pending;
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
static bool initialized;

static void dwc2_physical_isr(void *argument)
{
	(void)argument;
	atomic_fetch_add_explicit(&physical_irq_count, 1,
				 memory_order_relaxed);
	atomic_store_explicit(&guest_irq_pending, true, memory_order_release);

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
		.otg_speed = USB_PHY_SPEED_HIGH,
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
	atomic_store_explicit(&physical_irq_count, 0, memory_order_relaxed);

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
	printf("DWC2 HS passthrough ready: registers 0x%08x, core 0x%08lx, "
	       "port=%s\n",
	       DWC2_GUEST_BASE, (unsigned long)USB_DWC_HS.gsnpsid_reg.val,
	       USB_DWC_HS.hprt_reg.prtconnsts ? "connected" : "disconnected");
	printf("USB ownership: Linux DWC2 (ESP-IDF USB Host and TinyUSB are "
	       "not installed)\n");
	printf("WARNING: board H2 is wired as a Type-C sink (CC1/CC2 Rd); "
	       "native host needs DFP/Rp and a protected VBUS source\n");
	return 0;
}

void dwc2_passthrough_service(void)
{
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
		       "connected=%u enabled=%u line=%lu speed=%lu "
		       "hprt=%08lx gahbcfg=%08lx gintmsk=%08lx "
		       "gintsts=%08lx irq=%u\n",
		       guest_read_seen ? "probed" : "not-probed",
		       guest_write_seen ? "/configured" : "",
		       USB_DWC_HS.gintsts_reg.curmod ? "host" : "device",
		       USB_DWC_HS.hprt_reg.prtpwr,
		       USB_DWC_HS.hprt_reg.prtconnsts,
		       USB_DWC_HS.hprt_reg.prtena,
		       (unsigned long)USB_DWC_HS.hprt_reg.prtlnsts,
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
			       "still sees no device pull-up; check H2 VBUS and "
			       "Type-C DFP/Rp hardware\n");
			no_link_warning_printed = true;
		}
	} else {
		powered_no_link_polls = 0;
		if (USB_DWC_HS.hprt_reg.prtconnsts)
			no_link_warning_printed = false;
	}
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
	guest_read_seen = true;

	value = *(volatile uint32_t *)(uintptr_t)aligned_address;
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
	guest_write_seen = true;

	reg = (volatile uint32_t *)(uintptr_t)aligned_address;
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
				     memory_order_acq_rel))
		esp_intr_enable(usb_interrupt);
}
