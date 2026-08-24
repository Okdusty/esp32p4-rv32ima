/*
 * Copyright (c) 2023, Jisheng Zhang <jszhang@kernel.org>. All rights reserved.
 *
 * Use some code of mini-rv32ima.c from https://github.com/cnlohr/mini-rv32ima
 * Copyright 2022 Charles Lohr
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "port.h"
#include "cache.h"
#include "display_bridge.h"
#include "dwc2_passthrough.h"
#include "psram.h"
#include "virtio_net_bridge.h"

static uint32_t ram_amt = GUEST_RAM_SIZE;
static uint8_t *guest_ram;

#define EMULATOR_BATCH_INSTRUCTIONS 16384
#define DISPLAY_COMMIT_INTERVAL_US 5000u
#define USB_STATUS_INTERVAL_US 500000u
#define EMULATOR_DEBUG_CSR 0

struct MiniRV32IMAState;
void DumpState(struct MiniRV32IMAState *core);
static uint32_t HandleControlStore(
	uint32_t addy, uint32_t val, size_t width);
static uint32_t HandleControlLoad(uint32_t addy, size_t width);
static void HandleCacheBlockOp(uint32_t physical_address,
			       uint32_t operation);
static void UpdatePlatformInterrupts(struct MiniRV32IMAState *state);
static void GuestUartFlush(void);
static void GuestUartFlushIfDue(uint64_t current_time);
static void HandleOtherCSRWrite(uint8_t *image, uint16_t csrno, uint32_t value);
static int32_t HandleOtherCSRRead(uint8_t *image, uint16_t csrno);
static void MiniSleep();

#define MINIRV32WARN(x...) printf(x);
#define MINI_RV32_RAM_SIZE ram_amt
#define MINIRV32_IMPLEMENTATION
/* Per-instruction trap tracing is prohibitively expensive on the emulator. */
#define MINIRV32_POSTEXEC(...) do { } while (0)

#define MINIRV32_HANDLE_MEM_STORE_CONTROL(addy, val) \
	do { \
		size_t control_width = 1u << ((ir >> 12) & 3); \
		if ((uint32_t)((addy) - DISPLAY_FB_GUEST_BASE) <= \
		    DISPLAY_FB_SIZE - control_width) { \
			display_bridge_store((addy), (val), control_width); \
		} else if (HandleControlStore((addy), (val), control_width)) { \
			return (val); \
		} \
	} while (0)
#define MINIRV32_HANDLE_MEM_LOAD_CONTROL(addy, rval) \
	do { \
		size_t control_width = 1u << ((ir >> 12) & 3); \
		if ((uint32_t)((addy) - DISPLAY_FB_GUEST_BASE) <= \
		    DISPLAY_FB_SIZE - control_width) \
			rval = display_bridge_load((addy), control_width); \
		else \
			rval = HandleControlLoad((addy), control_width); \
		switch ((ir >> 12) & 7) { \
		case 0: rval = (int8_t)rval; break; \
		case 1: rval = (int16_t)rval; break; \
		default: break; \
		} \
	} while (0)
#define MINIRV32_OTHERCSR_WRITE(csrno, value) HandleOtherCSRWrite(image, csrno, value);
#define MINIRV32_OTHERCSR_READ(csrno, value) value = HandleOtherCSRRead(image, csrno);
#define MINIRV32_HANDLE_CACHE_OP(physical_address, operation) \
	HandleCacheBlockOp(physical_address, operation)
#define MINIRV32_LOAD4_UNCACHED(ofs) MiniRV32Load4Uncached(ofs)
#define MINIRV32_LOAD2_UNCACHED(ofs) MiniRV32Load2Uncached(ofs)
#define MINIRV32_LOAD1_UNCACHED(ofs) MiniRV32Load1Uncached(ofs)
#define MINIRV32_STORE4_UNCACHED(ofs, val) MiniRV32Store4Uncached(ofs, val)
#define MINIRV32_STORE2_UNCACHED(ofs, val) MiniRV32Store2Uncached(ofs, val)
#define MINIRV32_STORE1_UNCACHED(ofs, val) MiniRV32Store1Uncached(ofs, val)
/* Keep rdtime monotonic inside a long instruction batch. */
#define MINIRV32_HOST_TIME_US() GetTimeMicroseconds()

#define MINIRV32_CUSTOM_MEMORY_BUS
static inline __attribute__((always_inline)) void MINIRV32_STORE4(
	uint32_t ofs, uint32_t val)
{
	memcpy(guest_ram + ofs, &val, sizeof(val));
}

static inline __attribute__((always_inline)) void MINIRV32_STORE2(
	uint32_t ofs, uint16_t val)
{
	memcpy(guest_ram + ofs, &val, sizeof(val));
}

static inline __attribute__((always_inline)) void MINIRV32_STORE1(
	uint32_t ofs, uint8_t val)
{
	guest_ram[ofs] = val;
}

static inline __attribute__((always_inline)) uint32_t MINIRV32_LOAD4(
	uint32_t ofs)
{
	uint32_t value;

	memcpy(&value, guest_ram + ofs, sizeof(value));
	return value;
}

static inline __attribute__((always_inline)) uint16_t MINIRV32_LOAD2(
	uint32_t ofs)
{
	uint16_t value;

	memcpy(&value, guest_ram + ofs, sizeof(value));
	return value;
}

static inline __attribute__((always_inline)) uint8_t MINIRV32_LOAD1(
	uint32_t ofs)
{
	return guest_ram[ofs];
}

static uint32_t MiniRV32Load4Uncached(uint32_t ofs)
{
	uint32_t value;

	cache_uncached_read(ofs, &value, sizeof(value));
	return value;
}

static uint16_t MiniRV32Load2Uncached(uint32_t ofs)
{
	uint16_t value;

	cache_uncached_read(ofs, &value, sizeof(value));
	return value;
}

static uint8_t MiniRV32Load1Uncached(uint32_t ofs)
{
	uint8_t value;

	cache_uncached_read(ofs, &value, sizeof(value));
	return value;
}

static void MiniRV32Store4Uncached(uint32_t ofs, uint32_t value)
{
	cache_uncached_write(ofs, &value, sizeof(value));
}

static void MiniRV32Store2Uncached(uint32_t ofs, uint16_t value)
{
	cache_uncached_write(ofs, &value, sizeof(value));
}

static void MiniRV32Store1Uncached(uint32_t ofs, uint8_t value)
{
	cache_uncached_write(ofs, &value, sizeof(value));
}

#include "mini-rv32ima.h"

void DumpState(struct MiniRV32IMAState *core)
{
	unsigned int pc = core->pc;
	unsigned int *regs = (unsigned int *)core->regs;
	uint64_t thit, taccessed;

	cache_get_stat(&thit, &taccessed);
	printf("hit: %" PRIu64 " accessed: %" PRIu64 "\n", thit, taccessed);
	printf("timer=%08" PRIx32 "%08" PRIx32
	       " match=%08" PRIx32 "%08" PRIx32
	       " sie=%08" PRIx32 " sip=%08" PRIx32
	       " sstatus=%08" PRIx32 " flags=%08" PRIx32 "\n",
	       core->timerh, core->timerl,
	       core->timermatchh, core->timermatchl,
	       core->sie, core->sip, core->mstatus, core->extraflags);
	printf("PC: %08x ", pc);
	printf("Z:%08x ra:%08x sp:%08x gp:%08x tp:%08x t0:%08x t1:%08x t2:%08x s0:%08x s1:%08x a0:%08x a1:%08x a2:%08x a3:%08x a4:%08x a5:%08x ",
		   regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7],
		regs[8], regs[9], regs[10], regs[11], regs[12], regs[13], regs[14], regs[15] );
	printf("a6:%08x a7:%08x s2:%08x s3:%08x s4:%08x s5:%08x s6:%08x s7:%08x s8:%08x s9:%08x s10:%08x s11:%08x t3:%08x t4:%08x t5:%08x t6:%08x\n",
		   regs[16], regs[17], regs[18], regs[19], regs[20], regs[21], regs[22], regs[23],
		regs[24], regs[25], regs[26], regs[27], regs[28], regs[29], regs[30], regs[31] );
}

struct MiniRV32IMAState core;

void app_main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	printf("psram init\n");

	if (psram_init() < 0) {
		printf("failed to init psram\n");
		return;
	}
	guest_ram = psram_get_base();
	if (guest_ram == NULL) {
		printf("failed to map guest RAM\n");
		return;
	}

	/* Reserve guest RAM first, then allocate the LCD driver's framebuffer. */
	display_bridge_init();
	if (HostInputInit() < 0)
		return;
	if (virtio_net_bridge_init() < 0)
		printf("WARNING: C6/virtio-net bridge initialization failed\n");
	if (HostConsoleInit() < 0)
		printf("WARNING: Asynchronous UART console unavailable\n");

	printf("\nLoading kernel from flash...\n");

	restart:

	if (load_images(ram_amt, NULL) < 0)
		return;

  memset(&core, 0, sizeof(core));

	core.pc = MINIRV32_RAM_IMAGE_OFFSET + KERNEL_LOAD_OFFSET;
	core.regs[10] = 0; /* a0 = hart ID. */
	core.regs[11] = MINIRV32_RAM_IMAGE_OFFSET + DTB_LOAD_OFFSET;
	core.extraflags = 1; /* Supervisor mode. */
	core.satp = 0;       /* Linux enables Sv32 itself. */

	uint64_t lastTime = GetTimeMicroseconds();
	uint64_t last_display_commit = lastTime;
	uint64_t last_usb_status = lastTime;
	int instrs_per_flip = EMULATOR_BATCH_INSTRUCTIONS;
	printf("RV32IMA starting\n");
	printf("Initial PC: 0x%08" PRIx32 "\n", core.pc);

	while (1) {
		int ret;
		uint64_t currentTime = GetTimeMicroseconds();
		uint32_t elapsedUs = (uint32_t)(currentTime - lastTime);

		if (elapsedUs > 100000) elapsedUs = 100000;
		lastTime = currentTime;
		GuestUartFlushIfDue(currentTime);
		UpdatePlatformInterrupts(&core);
		if (currentTime - last_usb_status >= USB_STATUS_INTERVAL_US) {
			dwc2_passthrough_service();
			last_usb_status = currentTime;
		}

		ret = MiniRV32IMAStep(&core, NULL, 0, elapsedUs, instrs_per_flip);
		if (ret == 1 ||
		    currentTime - last_display_commit >= DISPLAY_COMMIT_INTERVAL_US) {
			display_bridge_commit();
			last_display_commit = currentTime;
		}

		switch (ret) {
			case 0:
				break;
			case 1:
				MiniSleep();
				break;
			case 3:
				break;
			case 0x7777:
				goto restart;
			case 0x5555:
				GuestUartFlush();
				printf("POWEROFF@0x%" PRIu32 "%"PRIu32"\n", core.cycleh, core.cyclel);
				DumpState(&core);
				return;
			default:
				printf("Unknown failure: ret=%d\n", ret);
				DumpState(&core);
				return;
		}
	}


	DumpState(&core);
}

static void MiniSleep(void)
{
	usleep(10);
}

static void HandleCacheBlockOp(uint32_t physical_address,
			       uint32_t operation)
{
	switch (operation) {
	case 0: /* cbo.inval */
		cache_invalidate_guest_line(physical_address);
		break;
	case 1: /* cbo.clean */
		cache_clean_guest_line(physical_address);
		break;
	case 2: /* cbo.flush */
		cache_flush_guest_line(physical_address);
		break;
	default:
		break;
	}
}

static uint8_t uart_scratch = 0;
static uint8_t uart_ier = 0;
static uint16_t uart_divisor = 0;
static uint8_t uart_lcr = 0;
static uint8_t uart_mcr = 0;
static uint8_t uart_fcr = 0;
static bool uart_thre_irq_pending = true;

#define GUEST_UART_TX_BUFFER_SIZE 256u
#define GUEST_UART_TX_FLUSH_US    10000u

static uint8_t guest_uart_tx_buffer[GUEST_UART_TX_BUFFER_SIZE];
static size_t guest_uart_tx_length;
static uint64_t guest_uart_last_flush;

static void GuestUartFlush(void)
{
	if (!guest_uart_tx_length)
		return;

	HostConsoleWrite(guest_uart_tx_buffer, guest_uart_tx_length);
	guest_uart_tx_length = 0;
	guest_uart_last_flush = GetTimeMicroseconds();
}

static void GuestUartWrite(uint8_t value)
{
	guest_uart_tx_buffer[guest_uart_tx_length++] = value;

	if (value == '\n' ||
	    guest_uart_tx_length == GUEST_UART_TX_BUFFER_SIZE)
		GuestUartFlush();
}

static void GuestUartFlushIfDue(uint64_t current_time)
{
	if (guest_uart_tx_length &&
	    current_time - guest_uart_last_flush >= GUEST_UART_TX_FLUSH_US)
		GuestUartFlush();
}

#define UART_BASE              0x10000000u
#define UART_IER_RDI           (1u << 0)
#define UART_IER_THRI          (1u << 1)
#define PLIC_BASE              0x0c000000u
#define PLIC_SIZE              0x00400000u
#define PLIC_UART_SOURCE       1u
#define PLIC_USB_SOURCE        2u
#define PLIC_NET_SOURCE        VIRTIO_NET_PLIC_SOURCE
#define PLIC_SOURCE_COUNT      3u
#define PLIC_PENDING_BASE      0x00001000u
#define PLIC_ENABLE_BASE       0x00002000u
#define PLIC_CONTEXT_BASE      0x00200000u
#define PLIC_CONTEXT_THRESHOLD 0x00000000u
#define PLIC_CONTEXT_CLAIM     0x00000004u
#define SIP_SEIP               (1u << 9)

static uint32_t plic_priority[PLIC_SOURCE_COUNT + 1u] = { 0, 1, 1, 1 };
static uint32_t plic_enable;
static uint32_t plic_threshold;

static bool uart_interrupt_pending(void)
{
	if ((uart_ier & UART_IER_RDI) && IsKBHit())
		return true;

	return (uart_ier & UART_IER_THRI) && uart_thre_irq_pending;
}

static bool plic_source_pending(uint32_t source)
{
	if (source == PLIC_UART_SOURCE)
		return uart_interrupt_pending();
	if (source == PLIC_USB_SOURCE)
		return dwc2_passthrough_irq_pending();
	if (source == PLIC_NET_SOURCE)
		return virtio_net_bridge_irq_pending();
	return false;
}

static uint32_t plic_claim(void)
{
	uint32_t best_source = 0;
	uint32_t best_priority = plic_threshold;

	for (uint32_t source = 1; source <= PLIC_SOURCE_COUNT; source++) {
		if ((plic_enable & (1u << source)) &&
		    plic_source_pending(source) &&
		    plic_priority[source] > best_priority) {
			best_source = source;
			best_priority = plic_priority[source];
		}
	}
	return best_source;
}

static bool plic_interrupt_pending(void)
{
	return plic_claim() != 0;
}

static uint32_t PlicLoad(uint32_t addy)
{
	uint32_t offset = addy - PLIC_BASE;

	if (offset >= sizeof(uint32_t) &&
	    offset <= PLIC_SOURCE_COUNT * sizeof(uint32_t))
		return plic_priority[offset / sizeof(uint32_t)];

	if (offset == PLIC_PENDING_BASE) {
		uint32_t pending = 0;

		for (uint32_t source = 1; source <= PLIC_SOURCE_COUNT; source++) {
			if (plic_source_pending(source))
				pending |= 1u << source;
		}
		return pending;
	}

	if (offset == PLIC_ENABLE_BASE)
		return plic_enable;

	if (offset == PLIC_CONTEXT_BASE + PLIC_CONTEXT_THRESHOLD)
		return plic_threshold;

	if (offset == PLIC_CONTEXT_BASE + PLIC_CONTEXT_CLAIM)
		/* Both emulated devices are level-triggered. */
		return plic_claim();

	return 0;
}

static void PlicStore(uint32_t addy, uint32_t val)
{
	uint32_t offset = addy - PLIC_BASE;

	if (offset >= sizeof(uint32_t) &&
	    offset <= PLIC_SOURCE_COUNT * sizeof(uint32_t)) {
		plic_priority[offset / sizeof(uint32_t)] = val;
	} else if (offset == PLIC_ENABLE_BASE) {
		plic_enable = val;
	} else if (offset == PLIC_CONTEXT_BASE + PLIC_CONTEXT_THRESHOLD) {
		plic_threshold = val;
	} else if (offset == PLIC_CONTEXT_BASE + PLIC_CONTEXT_CLAIM &&
		   val == PLIC_USB_SOURCE) {
		dwc2_passthrough_irq_complete();
	}
}

static void UpdatePlatformInterrupts(struct MiniRV32IMAState *state)
{
	if (plic_interrupt_pending()) {
		state->sip |= SIP_SEIP;
		state->extraflags &= ~4u; /* Wake a guest blocked in WFI. */
	} else {
		state->sip &= ~SIP_SEIP;
	}
}

static uint32_t HandleControlStore(
	uint32_t addy, uint32_t val, size_t width)
{
	if (display_bridge_contains(addy, width)) {
		display_bridge_store(addy, val, width);
		return 0;
	}

	if (dwc2_passthrough_contains(addy, width)) {
		dwc2_passthrough_store(addy, val, width);
		return 0;
	}

	if (virtio_net_bridge_contains(addy, width)) {
		virtio_net_bridge_store(addy, val, width);
		return 0;
	}

	if (addy >= PLIC_BASE && addy < PLIC_BASE + PLIC_SIZE) {
		PlicStore(addy, val);
		return 0;
	}

    switch (addy) {
        case 0x10000000:
            if (uart_lcr & 0x80) uart_divisor = (uart_divisor & 0xFF00) | (val & 0xFF);
            else {
			GuestUartWrite(val);
			uart_thre_irq_pending = true;
		}
            break;
        case 0x10000001:
            if (uart_lcr & 0x80) uart_divisor = (uart_divisor & 0x00FF) | ((val & 0xFF) << 8);
            else {
			uint8_t old_ier = uart_ier;
			uart_ier = val & 0x0f;
			if ((uart_ier & UART_IER_THRI) &&
			    !(old_ier & UART_IER_THRI))
				uart_thre_irq_pending = true;
			if (!(uart_ier & UART_IER_THRI))
				uart_thre_irq_pending = false;
		}
            break;
        case 0x10000002: uart_fcr = val; break;
        case 0x10000003: uart_lcr = val; break;
        case 0x10000004: uart_mcr = val; break;
        case 0x10000007: uart_scratch = val; break;
    }
    return 0;
}



static uint32_t HandleControlLoad(uint32_t addy, size_t width)
{
	if (display_bridge_contains(addy, width))
		return display_bridge_load(addy, width);

	if (dwc2_passthrough_contains(addy, width))
		return dwc2_passthrough_load(addy, width);

	if (virtio_net_bridge_contains(addy, width))
		return virtio_net_bridge_load(addy, width);

	if (addy >= PLIC_BASE && addy < PLIC_BASE + PLIC_SIZE)
		return PlicLoad(addy);

    switch (addy) {
        case 0x10000000:
			if (uart_lcr & 0x80)
				return uart_divisor & 0xff;
			return IsKBHit() ? ReadKBByte() : 0;
        case 0x10000001:
            if (uart_lcr & 0x80) return (uart_divisor >> 8) & 0xFF;
            return uart_ier;
		case 0x10000002:
			if ((uart_ier & UART_IER_RDI) && IsKBHit())
				return 0xC4; /* FIFO enabled, received data available. */
			if ((uart_ier & UART_IER_THRI) && uart_thre_irq_pending) {
				uart_thre_irq_pending = false;
				return 0xC2; /* FIFO enabled, THR empty. */
			}
			return 0xC1; /* FIFO enabled, no interrupt pending. */
        case 0x10000003: return uart_lcr;
        case 0x10000004: return uart_mcr;
        case 0x10000005:
          return 0x60 | (IsKBHit() ? 1 : 0);
        case 0x10000006: return 0x00;
        case 0x10000007: return uart_scratch;
    }
    return 0;
}
static void HandleOtherCSRWrite(uint8_t *image, uint16_t csrno, uint32_t value)
{
	uint32_t ptrstart, ptrend;

	#if EMULATOR_DEBUG_CSR
	if (csrno != 0x136 && csrno != 0x137 && csrno != 0x138 &&
		csrno != 0x139 && csrno != 0x3a0 && csrno != 0x3b0 && csrno != 0xf14) {
		static int csr_write_count = 0;
		if (csr_write_count++ < 20)
			printf("[CSR_WRITE] csr=0x%03x value=0x%08" PRIx32 "\n",
			       csrno, value);
		}
	#endif

		switch (csrno) {
			case 0x136:
				printf("%d", (int)value);
				fflush(stdout);
				break;
			case 0x137:
				printf("%08" PRIx32, value);
				fflush(stdout);
				break;
			case 0x138:
				ptrstart = value - MINIRV32_RAM_IMAGE_OFFSET;
				ptrend = ptrstart;
				if (ptrstart >= ram_amt)
					printf("DEBUG PASSED INVALID PTR (%"PRIu32")\n", value);
			while (ptrend < ram_amt) {
				uint8_t c = MINIRV32_LOAD1(ptrend);
				if (c == 0)
					break;
				fwrite(&c, 1, 1, stdout);
				ptrend++;
			}
			break;
			case 0x139:
				putchar(value);
				fflush(stdout);
				break;
			default:
				break;
		}
}

static int32_t HandleOtherCSRRead(uint8_t *image, uint16_t csrno)
{
	int32_t result = 0;

	if (csrno == 0x140) {
		if (!IsKBHit())
			result = -1;
		else
			result = ReadKBByte();
	}

	#if EMULATOR_DEBUG_CSR
	if (csrno != 0x140 && csrno != 0xC00 && csrno != 0x3a0 &&
		csrno != 0x3b0 && csrno != 0xf14 && result == 0) {
		static int csr_warn_count = 0;
		if (csr_warn_count++ < 20)
			printf("[CSR_READ] Unhandled csr=0x%03x -> 0x%08" PRIx32 "\n",
			       csrno, (uint32_t)result);
		}
	#endif

		return result;
}
