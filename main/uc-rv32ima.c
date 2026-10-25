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

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"
#include "esp_rom_serial_output.h"
#include "esp_system.h"
#include "esp_log.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if CONFIG_RV32_HOST_PERF_STATS
#include "freertos/idf_additions.h"
#endif

#include "port.h"
#ifdef ESP_PLATFORM
#include "esp_hosted.h"
#endif
#include "rv32_diagnostics.h"
#include "rv32_memory_placement.h"
#include "reset_protocol.h"
#if CONFIG_RV32_CACHE_PERF_STATS
#include "host_cache_profile.h"
#endif
#if CONFIG_RV32_HOST_PC_SAMPLER
#include "host_pc_sampler.h"
#endif
#if CONFIG_RV32_HOST_MICROBENCH
#include "host_microbench.h"
#endif
#if CONFIG_RV32_HOST_LANE_REPORT
#include "host_lane_report.h"
#endif
#include "cache.h"
#include "display_bridge.h"
#include "dwc2_passthrough.h"
#include "rv32_host_wake.h"
#include "psram.h"
#include "pv_console_protocol.h"
#if CONFIG_RV32_NATIVE_JIT
#include "rv32_native_jit.h"
#include "guest_hle.h"
#include "guest_lz4.h"
#endif
#include "virtio_console_bridge.h"
#include "virtio_blk_bridge.h"
#include "virtio_flash_bridge.h"
#include "virtio_gpu_bridge.h"
#include "virtio_input_bridge.h"
#include "virtio_net_bridge.h"

static uint32_t ram_amt = GUEST_RAM_SIZE;
static uint8_t *guest_ram;
static uint8_t *linear_mmio_host;
static uint32_t guest_reset_mode;

#define EMULATOR_BATCH_INSTRUCTIONS 32768
#define USB_STATUS_INTERVAL_US 500000u

#if CONFIG_RV32_EMULATOR_PERF_STATS
#define MINIRV32_PERF_STATS
#define EMULATOR_PERF_INTERVAL_US \
	((uint64_t)CONFIG_RV32_EMULATOR_PERF_INTERVAL_MS * 1000u)
#endif

#if CONFIG_RV32_HOST_PERF_STATS
#define HOST_PERF_INTERVAL_US 5000000u
#endif

struct MiniRV32IMAState;
void DumpState(struct MiniRV32IMAState *core);
#if CONFIG_RV32_NATIVE_JIT
static uint32_t RV32JitMemoryLoad(
	struct rv32_native_jit_runtime *runtime,
	uint32_t virtual_address, uint32_t descriptor);
static uint32_t RV32JitMemoryStore(
	struct rv32_native_jit_runtime *runtime,
	uint32_t virtual_address, uint32_t value, uint32_t descriptor);
#endif
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
static void MiniSleep(void);

volatile uint32_t rv32_host_wake;
#define MINIRV32_HOST_WAKE_PENDING() (rv32_host_wake != 0u)

#define MINIRV32WARN(x...) printf(x);
#define MINI_RV32_RAM_SIZE ram_amt
#include "guest_profile.h"
#if CONFIG_RV32_GUEST_PROFILE
#define MINIRV32_PROFILE_INSTRUCTION(pc, offset, privilege) \
	rv32_profile_interpreted((pc), (offset), (privilege))
#define MINIRV32_PROFILE_REASON(reason) rv32_profile_set_reason(reason)
#define MINIRV32_PROFILE_FALLTHROUGH() rv32_profile_fallthrough()
#endif
#define MINIRV32_IMPLEMENTATION
/* The bundled kernel and supported replacement images use aligned, 32-bit
 * RV32IMA instructions.  Skip redundant fetch checks in the hot loop; ports
 * that include mini-rv32ima.h without this define retain strict validation. */
#define MINIRV32_TRUSTED_32BIT_FETCH
/* System calls and page faults from user mode enter the kernel without
 * ending the batch. */
#define MINIRV32_CONTINUE_AFTER_TRAP
#if CONFIG_RV32_DECODED_BLOCK_CACHE
#define MINIRV32_DECODED_BLOCK_CACHE
#endif
#if CONFIG_RV32_NATIVE_JIT
/* Most probes land on heads that are still cold.  Skip RV32_JIT_PROBE_SKIP
 * visits after each failed probe and let the next one count them all, so a
 * head compiles after the same number of guest visits at a thirty-second of
 * the probe cost; only loop heads, hot at 16 visits, compile one window later
 * than with 15 skips.  A fresh region is entered at most that many visits
 * late.  Measured against 15 skips: `find` −4 %, the fork loop −3 %, the
 * desktop start within noise. */
#define RV32_JIT_PROBE_SKIP 31u
/* Indexed by PC bits 2-11; an entry holds the head's PC bits 12-31, a valid
 * bit and the visits still to skip.  256 entries of PC and count let kernel
 * heads evict each other before their skips ran out. */
#define RV32_JIT_PROBE_FILTER_ENTRIES 1024u
#define RV32_JIT_PROBE_FILTER_TAG (~(RV32_JIT_PROBE_FILTER_ENTRIES * 4u - 1u))
#define RV32_JIT_PROBE_FILTER_VALID 32u
#define RV32_JIT_PROBE_FILTER_COUNT (RV32_JIT_PROBE_FILTER_VALID - 1u)
_Static_assert((RV32_JIT_PROBE_FILTER_ENTRIES &
		(RV32_JIT_PROBE_FILTER_ENTRIES - 1u)) == 0u &&
	       RV32_JIT_PROBE_SKIP < RV32_JIT_PROBE_FILTER_VALID &&
	       (RV32_JIT_PROBE_FILTER_VALID & RV32_JIT_PROBE_FILTER_TAG) == 0u,
	       "the skip count and valid bit sit below the PC tag");
static uint32_t rv32_jit_probe_filter[RV32_JIT_PROBE_FILTER_ENTRIES];

static inline const struct rv32_native_jit_result *RV32JitTryFiltered(
	uint32_t physical_offset, uint32_t guest_pc, uint32_t limit,
	uint32_t context)
{
	uint32_t *entry = &rv32_jit_probe_filter[(guest_pc >> 2) &
		(RV32_JIT_PROBE_FILTER_ENTRIES - 1u)];
	uint32_t tag = (guest_pc & RV32_JIT_PROBE_FILTER_TAG) |
		RV32_JIT_PROBE_FILTER_VALID;
	/* The tag's low bits are clear, so the entry minus the tag is this
	 * head's skip count exactly when it is at most the count mask: one
	 * unsigned compare tests the tag and a nonzero count. */
	uint32_t skips = *entry - tag;
	const struct rv32_native_jit_result *result;

	if (skips - 1u < RV32_JIT_PROBE_FILTER_COUNT) {
		*entry = tag + skips - 1u;
		rv32_profile_set_reason(RV32_PROFILE_FILTERED);
		return NULL;
	}
#if CONFIG_RV32_HOST_PERF_STATS
	result = rv32_native_jit_try_profiled(physical_offset, guest_pc, limit,
		context, skips == 0u ? RV32_JIT_PROBE_SKIP + 1u : 1u);
#else
	result = rv32_native_jit_try_weighted(physical_offset, guest_pc, limit,
		context, skips == 0u ? RV32_JIT_PROBE_SKIP + 1u : 1u);
#endif
	if (result == NULL)
		*entry = tag | RV32_JIT_PROBE_SKIP;
	else if (skips == 0u)
		*entry = 0u;
	return result;
}
#define MINIRV32_NATIVE_JIT_TRY(physical_offset, guest_pc, limit, context) \
	RV32JitTryFiltered((physical_offset), (guest_pc), (limit), (context))
#define MINIRV32_NATIVE_JIT_FLUSH() rv32_native_jit_fence()
#define MINIRV32_NOTE_WRITE_PAGE(offset) rv32_native_jit_note_write_page(offset)
#define MINIRV32_NATIVE_JIT_MEMORY_FLUSH() \
	rv32_native_jit_memory_cache_flush()
#define MINIRV32_NATIVE_JIT_MEMORY_FLUSH_PAGE(virtual_address) \
	rv32_native_jit_memory_cache_flush_page((virtual_address))
#define MINIRV32_NATIVE_JIT_CACHE_RAM_PAGE(offset, va, context, write) \
	rv32_native_jit_seed_ram_page((offset), (va), (context), (write))
#if CONFIG_RV32_NATIVE_JIT_SUPERVISOR
#define MINIRV32_NATIVE_JIT_MAX_PRIVILEGE 1u
#endif
#endif
/* Per-instruction trap tracing is prohibitively expensive on the emulator. */
#define MINIRV32_POSTEXEC(...) do { } while (0)

#define MINIRV32_HANDLE_MEM_STORE_CONTROL(addy, val) \
	do { \
		size_t control_width = 1u << (MINIRV32_DECODE_FUNCT3() & 3u); \
		if (HandleControlStore((addy), (val), control_width)) \
			return (val); \
	} while (0)
#define MINIRV32_HANDLE_MEM_LOAD_CONTROL(addy, rval) \
	do { \
		size_t control_width = 1u << (MINIRV32_DECODE_FUNCT3() & 3u); \
		rval = HandleControlLoad((addy), control_width); \
		switch (MINIRV32_DECODE_FUNCT3()) { \
		case 0: rval = (int8_t)rval; break; \
		case 1: rval = (int16_t)rval; break; \
		default: break; \
		} \
	} while (0)
#define MINIRV32_OTHERCSR_WRITE(csrno, value) HandleOtherCSRWrite(image, csrno, value);
#define MINIRV32_OTHERCSR_READ(csrno, value) value = HandleOtherCSRRead(image, csrno);
#define MINIRV32_HANDLE_CACHE_OP(physical_address, operation) \
	HandleCacheBlockOp(physical_address, operation)
#if defined(CONFIG_RV32_DISPLAY_ACCEL) && CONFIG_RV32_DISPLAY_ACCEL
/* mini-rv32ima exposes this as a generic host-backed MMIO aperture.  The
 * platform maps the current display surface into it, but the interpreter
 * itself has no pixel-format or display-command knowledge. */
#define MINIRV32_LINEAR_MMIO_APERTURE
#define MINIRV32_LINEAR_MMIO_BASE DISPLAY_FB_GUEST_BASE
#define MINIRV32_LINEAR_MMIO_SIZE DISPLAY_FB_SIZE
#define MINIRV32_LINEAR_MMIO_HOST linear_mmio_host
#if CONFIG_RV32_DISPLAY_ASYNC_GUEST
#define MINIRV32_LINEAR_MMIO_ACCESS() display_bridge_guest_framebuffer_access()
#endif
#define MINIRV32_LINEAR_MMIO_PUBLISH(start, end, calls, bytes) \
	display_bridge_mark_framebuffer_dirty((start), (end), (calls), (bytes))
#endif
#if CONFIG_RV32_FLASH_XIP
/* The "xip" flash partition, mapped once by psram_map_xip_partition(), is
 * read-only guest physical memory at GUEST_XIP_BASE (uc.dts "mtd-rom"). */
#define MINIRV32_ROM_APERTURE
#define MINIRV32_ROM_BASE GUEST_XIP_BASE
#define MINIRV32_ROM_SIZE rv32_xip_size
#define MINIRV32_ROM_HOST rv32_xip_host
#endif
/* The USB controller and the PLIC first: interrupt handlers hit them most. */
#define MINIRV32_IS_MMIO_ADDRESS(address) \
	(((address) >= 0x50000000u && (address) < 0x50040000u) || \
	 ((address) >= 0x0c000000u && (address) < 0x0c400000u) || \
	 ((address) >= 0x10000000u && (address) < 0x12000200u) || \
	 ((address) >= VIRTIO_CONSOLE_GUEST_BASE && \
	  (address) < VIRTIO_CONSOLE_GUEST_BASE + VIRTIO_CONSOLE_GUEST_SIZE) || \
	 ((address) >= VIRTIO_BLK_GUEST_BASE && \
	  (address) < VIRTIO_BLK_GUEST_BASE + VIRTIO_BLK_GUEST_SIZE) || \
	 ((address) >= VIRTIO_FLASH_GUEST_BASE && \
	  (address) < VIRTIO_FLASH_GUEST_BASE + VIRTIO_FLASH_GUEST_SIZE) || \
	 ((address) >= VIRTIO_GPU_GUEST_BASE && \
	  (address) < VIRTIO_GPU_GUEST_BASE + VIRTIO_GPU_GUEST_SIZE) || \
	 ((address) >= VIRTIO_INPUT_GUEST_BASE && \
	  (address) < VIRTIO_INPUT_GUEST_BASE + VIRTIO_INPUT_GUEST_SIZE))
#define MINIRV32_LOAD4_UNCACHED(ofs) MiniRV32Load4Uncached(ofs)
#define MINIRV32_LOAD2_UNCACHED(ofs) MiniRV32Load2Uncached(ofs)
#define MINIRV32_LOAD1_UNCACHED(ofs) MiniRV32Load1Uncached(ofs)
#define MINIRV32_STORE4_UNCACHED(ofs, val) MiniRV32Store4Uncached(ofs, val)
#define MINIRV32_STORE2_UNCACHED(ofs, val) MiniRV32Store2Uncached(ofs, val)
#define MINIRV32_STORE1_UNCACHED(ofs, val) MiniRV32Store1Uncached(ofs, val)
/* Keep rdtime monotonic inside a long instruction batch without calling the
 * 64-bit ESP timer for every guest CSR read.  CPU0 stays at the configured
 * fixed frequency while the emulator runs. */
#define MINIRV32_HOST_CYCLE_COUNT() esp_cpu_get_cycle_count()
#define MINIRV32_HOST_CYCLES_PER_US CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ

#define MINIRV32_CUSTOM_MEMORY_BUS
static inline __attribute__((always_inline)) void MiniRV32Store4(
	uint8_t *image, uint32_t ofs, uint32_t val)
{
	memcpy(image + ofs, &val, sizeof(val));
}

static inline __attribute__((always_inline)) void MiniRV32Store2(
	uint8_t *image, uint32_t ofs, uint16_t val)
{
	memcpy(image + ofs, &val, sizeof(val));
}

static inline __attribute__((always_inline)) void MiniRV32Store1(
	uint8_t *image, uint32_t ofs, uint8_t val)
{
	image[ofs] = val;
}

static inline __attribute__((always_inline)) uint32_t MiniRV32Load4(
	const uint8_t *image, uint32_t ofs)
{
	uint32_t value;

	memcpy(&value, image + ofs, sizeof(value));
	return value;
}

static inline __attribute__((always_inline)) uint16_t MiniRV32Load2(
	const uint8_t *image, uint32_t ofs)
{
	uint16_t value;

	memcpy(&value, image + ofs, sizeof(value));
	return value;
}

static inline __attribute__((always_inline)) uint8_t MiniRV32Load1(
	const uint8_t *image, uint32_t ofs)
{
	return image[ofs];
}

/* mini-rv32ima's implementation functions all carry `image`.  Expanding the
 * memory bus through that argument lets GCC retain the PSRAM base in a host
 * register instead of loading the global guest_ram pointer for every fetch. */
#define MINIRV32_STORE4(ofs, val) MiniRV32Store4(image, (ofs), (val))
#define MINIRV32_STORE2(ofs, val) MiniRV32Store2(image, (ofs), (val))
#define MINIRV32_STORE1(ofs, val) MiniRV32Store1(image, (ofs), (val))
#define MINIRV32_LOAD4(ofs) MiniRV32Load4(image, (ofs))
#define MINIRV32_LOAD2(ofs) MiniRV32Load2(image, (ofs))
#define MINIRV32_LOAD1(ofs) MiniRV32Load1(image, (ofs))

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

RV32_HOT_STATE_BSS struct MiniRV32IMAState core __attribute__((aligned(128)));

/* Stall reporter.  It runs on CPU1, so it still reports when the emulator
 * core is stuck, and prints through ROM output, which polls the UART FIFO
 * instead of depending on the UART task.  Three conditions: the emulator
 * loop stops iterating; a USB interrupt was forwarded to the guest and never
 * completed (the guest spins inside its handler while the loop keeps
 * running); or heavy forwarded USB traffic.  After the second report of a
 * hang it restarts the chip, since nothing else recovers the guest. */
#define EMULATOR_WATCH_STALL_S    2u
#define EMULATOR_WATCH_RESTART_S  12u
#define EMULATOR_WATCH_USB_PER_S  100u
#define EMULATOR_WATCH_USB_EVERY  5u
#define EMULATOR_WATCH_PC_SAMPLES 8u

static volatile uint32_t emulator_loops;
static uint32_t plic_enable;
static uint32_t plic_threshold;
static uint32_t plic_priority[];
static volatile uint32_t plic_claims[];
static volatile uint32_t plic_completes[];
static volatile uint32_t plic_last_claimed;
static inline uint32_t plic_pending_sources(void);
static uint32_t plic_claim(void);

static void emulator_watch_dump(const char *what, uint32_t loops,
				uint32_t instructions,
				const struct dwc2_passthrough_stats *now,
				const struct dwc2_passthrough_stats *last)
{
	uint32_t samples[EMULATOR_WATCH_PC_SAMPLES];

	esp_rom_printf("[Host] %s loops/s=%u insn/s=%u pc=%08x priv=%u wfi=%u "
		       "usb/s phys=%u fwd=%u done=%u pend=%u\r\n", what,
		       (unsigned)loops, (unsigned)instructions,
		       (unsigned)core.pc, (unsigned)(core.extraflags & 3u),
		       (unsigned)((core.extraflags >> 2) & 1u),
		       (unsigned)(now->physical - last->physical),
		       (unsigned)(now->forwarded - last->forwarded),
		       (unsigned)(now->completed - last->completed),
		       (unsigned)now->pending);
	{
		struct dwc2_passthrough_channel_stats ch;

		dwc2_passthrough_channel_stats(&ch);
		esp_rom_printf("[USB] xfers=%u in=%u bytes=%u inval=%u nak=%u stall=%u "
			       "err=%u tgl=%u ovr=%u last ep%u len%u "
			       "%02x%02x%02x%02x%02x%02x%02x%02x\r\n",
			       (unsigned)ch.completions, (unsigned)ch.in_completions,
			       (unsigned)ch.in_bytes, (unsigned)ch.invalidations,
			       (unsigned)ch.naks, (unsigned)ch.stalls,
			       (unsigned)ch.errors, (unsigned)ch.toggle_errors,
			       (unsigned)ch.frame_overruns, (unsigned)ch.last_in_ep,
			       (unsigned)ch.last_in_len, ch.last_in_data[0],
			       ch.last_in_data[1], ch.last_in_data[2],
			       ch.last_in_data[3], ch.last_in_data[4],
			       ch.last_in_data[5], ch.last_in_data[6],
			       ch.last_in_data[7]);
	}
	{
		/* Quiet-device diagnostics: endpoints still polled (NAKs) while no
		 * data arrives, plus the controller's channel registers, at most
		 * once a minute. */
		static struct dwc2_passthrough_channel_stats quiet_ref;
		static uint32_t quiet_reports;
		struct dwc2_passthrough_channel_stats ch;

		dwc2_passthrough_stats_channels(&ch);
		if (ch.in_completions == quiet_ref.in_completions &&
		    ch.naks - quiet_ref.naks > 200u)
			quiet_reports++;
		else
			quiet_reports = 0;
		if (quiet_reports != 0u && quiet_reports % 12u == 6u) {
			esp_rom_printf("[USB] polled without data for %us:", (unsigned)(quiet_reports * 5u));
			for (uint32_t ep = 1; ep < 16; ep++)
				if (ch.ep_nak[ep] != 0u || ch.ep_in[ep] != 0u)
					esp_rom_printf(" ep%u in=%u nak=%u ovr=%u", (unsigned)ep,
						       (unsigned)ch.ep_in[ep], (unsigned)ch.ep_nak[ep],
						       (unsigned)ch.ep_ovr[ep]);
			esp_rom_printf("\r\n");
			dwc2_passthrough_dump();
		}
		quiet_ref = ch;
	}
	if (strcmp(what, "USB load") == 0)
		return;
	esp_rom_printf("[Host] mstatus=%08x sie=%08x sip=%08x stvec=%08x "
		       "sepc=%08x scause=%08x stval=%08x satp=%08x\r\n",
		       (unsigned)core.mstatus, (unsigned)core.sie,
		       (unsigned)core.sip, (unsigned)core.stvec,
		       (unsigned)core.sepc, (unsigned)core.scause,
		       (unsigned)core.stval, (unsigned)core.satp);
	for (uint32_t i = 0; i < EMULATOR_WATCH_PC_SAMPLES; i++) {
		samples[i] = core.pc;
		esp_rom_delay_us(3000);
	}
	esp_rom_printf("[Host] pc samples %08x %08x %08x %08x %08x %08x %08x "
		       "%08x ra=%08x sp=%08x\r\n", (unsigned)samples[0],
		       (unsigned)samples[1], (unsigned)samples[2],
		       (unsigned)samples[3], (unsigned)samples[4],
		       (unsigned)samples[5], (unsigned)samples[6],
		       (unsigned)samples[7], (unsigned)core.regs[1],
		       (unsigned)core.regs[2]);
	esp_rom_printf("[PLIC] enable=%08x threshold=%u pending=%08x claim-now=%u "
		       "last=%u prio=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
		       (unsigned)plic_enable, (unsigned)plic_threshold,
		       (unsigned)plic_pending_sources(), (unsigned)plic_claim(),
		       (unsigned)plic_last_claimed, (unsigned)plic_priority[1],
		       (unsigned)plic_priority[2], (unsigned)plic_priority[3],
		       (unsigned)plic_priority[4], (unsigned)plic_priority[5],
		       (unsigned)plic_priority[6], (unsigned)plic_priority[7],
		       (unsigned)plic_priority[8]);
	esp_rom_printf("[PLIC] claims none=%u src1..8=%u,%u,%u,%u,%u,%u,%u,%u "
		       "completes=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
		       (unsigned)plic_claims[0], (unsigned)plic_claims[1],
		       (unsigned)plic_claims[2], (unsigned)plic_claims[3],
		       (unsigned)plic_claims[4], (unsigned)plic_claims[5],
		       (unsigned)plic_claims[6], (unsigned)plic_claims[7],
		       (unsigned)plic_claims[8],
		       (unsigned)plic_completes[1], (unsigned)plic_completes[2],
		       (unsigned)plic_completes[3], (unsigned)plic_completes[4],
		       (unsigned)plic_completes[5], (unsigned)plic_completes[6],
		       (unsigned)plic_completes[7], (unsigned)plic_completes[8]);
	dwc2_passthrough_dump();
}

/* The lane report's esp_intr_dump() overflowed the stall reporter's 3 KiB
 * stack (CPU1 stack protection fault 25 s after boot). */
#if CONFIG_RV32_HOST_LANE_REPORT
#define EMULATOR_WATCH_STACK_SIZE 8192
#else
#define EMULATOR_WATCH_STACK_SIZE 3072
#endif

static void emulator_watch_task(void *argument)
{
	struct dwc2_passthrough_stats last, now;
	uint32_t last_loops = emulator_loops;
	uint32_t last_instructions = core.cyclel;
	uint32_t stalled = 0;
	uint32_t stuck = 0;
	uint32_t busy = 0;

	(void)argument;
	dwc2_passthrough_stats(&last);
#if CONFIG_RV32_HOST_LANE_REPORT
	uint32_t seconds = 0u;
#endif
	for (;;) {
		uint32_t loops;
		uint32_t instructions;
		uint32_t hung;
		const char *what = NULL;

		vTaskDelay(pdMS_TO_TICKS(1000));
#if CONFIG_RV32_HOST_LANE_REPORT
		HostLaneReportTick(++seconds);
#endif
		loops = emulator_loops;
		instructions = core.cyclel;
		dwc2_passthrough_stats(&now);
		stalled = loops == last_loops ? stalled + 1u : 0u;
		stuck = now.pending && now.completed == last.completed ?
				stuck + 1u : 0u;
		busy = now.forwarded - last.forwarded >= EMULATOR_WATCH_USB_PER_S ?
				busy + 1u : 0u;
		hung = stalled > stuck ? stalled : stuck;
		if (stalled >= EMULATOR_WATCH_STALL_S)
			what = "EMULATOR STALLED";
		else if (stuck >= EMULATOR_WATCH_STALL_S)
			what = "GUEST STUCK IN USB IRQ";
		else if (busy != 0u && busy % EMULATOR_WATCH_USB_EVERY == 1u)
			what = "USB load";
		if (what != NULL &&
		    (hung < EMULATOR_WATCH_STALL_S ||
		     (hung - EMULATOR_WATCH_STALL_S) % EMULATOR_WATCH_USB_EVERY == 0u))
			emulator_watch_dump(what, loops - last_loops,
					    instructions - last_instructions,
					    &now, &last);
		if (hung >= EMULATOR_WATCH_RESTART_S) {
			esp_rom_printf("[Host] %s for %us; restarting\r\n", what,
				       (unsigned)hung);
			esp_rom_output_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
			esp_restart();
		}
		last_loops = loops;
		last_instructions = instructions;
		last = now;
	}
}

#if CONFIG_RV32_NATIVE_JIT
static uint32_t RV32JitMemoryWidth(uint32_t funct3, bool load)
{
	if (funct3 == 0u || (load && funct3 == 4u))
		return 1u;
	if (funct3 == 1u || (load && funct3 == 5u))
		return 2u;
	if (funct3 == 2u)
		return 4u;
	return 0u;
}

/* A page may be published only into the bank of the live memory context.
 * Native code stops right after an inline CSR write that changes SUM/MXR,
 * but a stale bank must never learn a mapping that depends on them. */
static inline bool RV32JitBankIsCurrent(
	const struct rv32_native_jit_runtime *runtime)
{
	return runtime->memory_context == MiniRV32NativeMemoryContext(
		(const struct MiniRV32IMAState *)runtime->opaque);
}

static inline void RV32JitCacheReadPage(
	struct rv32_native_jit_runtime *runtime, uint32_t virtual_address,
	const void *host_address)
{
	struct rv32_native_jit_read_page *page =
		&runtime->read_pages[(virtual_address >> 12) &
			(RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u)];

	if (!RV32JitBankIsCurrent(runtime))
		return;
	/* The bias converts any address in this translated 4 KiB guest page to
	 * its corresponding host address with one native ADD.  Publish the tag
	 * last so generated code can never observe a new tag with an old bias. */
	page->host_bias =
		(uint32_t)(uintptr_t)host_address - virtual_address;
	__atomic_store_n(&page->virtual_page, virtual_address >> 12,
			 __ATOMIC_RELEASE);
}

static inline void RV32JitCacheWritePage(
	struct rv32_native_jit_runtime *runtime, uint32_t virtual_address,
	void *host_address, uint32_t linear_page_start,
	uint32_t linear_page_end)
{
	struct rv32_native_jit_write_page *page =
		&runtime->write_pages[(virtual_address >> 12) &
			(RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u)];

	if (!RV32JitBankIsCurrent(runtime))
		return;
	page->host_bias =
		(uint32_t)(uintptr_t)host_address - virtual_address;
	page->linear_page_start = linear_page_start;
	page->linear_page_end = linear_page_end;
	__atomic_store_n(&page->virtual_page, virtual_address >> 12,
			 __ATOMIC_RELEASE);
}

#if CONFIG_RV32_NATIVE_JIT_RAM_REFILL
static bool IRAM_ATTR RV32JitRefillRam(
	struct rv32_native_jit_runtime *runtime, uint32_t address, uint32_t access)
{
	struct MiniRV32IMAState *state = runtime->opaque;
	bool write = (access & 8u) != 0u;
	uint32_t width = RV32JitMemoryWidth(access & 7u, !write);
	int fault = 0;
	int uncached = 0;

	if (width == 0u || (address & (width - 1u)) != 0u ||
	    runtime->memory_context != MiniRV32NativeMemoryContext(state))
		return false;
	uint32_t physical = MiniRV32Translate(state, runtime->image, address,
		write ? ACCESS_WRITE : ACCESS_READ, &fault, &uncached);
	if (fault)
		return false;
#ifdef MINIRV32_ROM_APERTURE
	uint32_t rom_offset;

	/* Read-only flash: publish reads only, so a store never goes native. */
	if (MiniRV32RomOffset(runtime->image, physical, &rom_offset)) {
		if (write)
			return false;
		RV32JitCacheReadPage(runtime, address, runtime->image + rom_offset);
		return true;
	}
#endif
	if (uncached || physical < MINIRV32_RAM_IMAGE_OFFSET)
		return false;
	uint32_t offset = physical - MINIRV32_RAM_IMAGE_OFFSET;
	if (ram_amt < 0x1000u || (offset & ~0xfffu) > ram_amt - 0x1000u)
		return false;
	/* Only publish the mapping; generated code retries the access itself. */
	if (write) {
		rv32_native_jit_note_write_page(offset);
		RV32JitCacheWritePage(runtime, address, runtime->image + offset,
				     UINT32_MAX, 0u);
	}
	else
		RV32JitCacheReadPage(runtime, address, runtime->image + offset);
	return true;
}
#endif

#if CONFIG_RV32_GUEST_HLE
/*
 * Host versions of guest library routines (guest_hle.h).  Each one does the
 * whole call or declines, leaving it to the guest's own code.  Declining
 * part way through a copy or fill is safe: the guest code redoes it from the
 * start and rewrites the same bytes, because the buffers are disjoint.  Only
 * ordinary cached RAM is touched; device memory, uncached (DMA-coherent)
 * pages and faults go to the guest code, which handles them as before.
 */
static uint8_t *RV32HleMap(uint32_t virtual_address, int access,
			   uint32_t *span)
{
	struct MiniRV32IMAState *state = &core;
	int fault;
	int uncached = 0;
	uint32_t physical = MiniRV32Translate(state, guest_ram, virtual_address,
					      access, &fault, &uncached);
	uint32_t offset = physical - MINIRV32_RAM_IMAGE_OFFSET;

#ifdef MINIRV32_ROM_APERTURE
	if (!fault && access == ACCESS_READ &&
	    MiniRV32RomOffset(guest_ram, physical, &offset)) {
		/* ROM pages are whole: the rest of this page is readable. */
		*span = 0x1000u - (virtual_address & 0xfffu);
		return guest_ram + offset;
	}
#endif
	if (fault || uncached || physical < MINIRV32_RAM_IMAGE_OFFSET ||
	    offset >= ram_amt)
		return NULL;
	if (access == ACCESS_WRITE)
		rv32_native_jit_note_write_page(offset);
	*span = 0x1000u - (virtual_address & 0xfffu);
	if (*span > ram_amt - offset)
		*span = ram_amt - offset;
	return guest_ram + offset;
}

/* The kernel's linear map is one contiguous block of guest RAM. */
static uint8_t *RV32HleLinear(uint32_t virtual_address, uint32_t size)
{
	struct MiniRV32IMAState *state = &core;
	uint32_t linear = virtual_address - MINIRV32_KERNEL_LINEAR_BASE;
	uint32_t window = ram_amt - MINIRV32_KERNEL_LINEAR_PHYS_OFFSET;

	if ((CSR(extraflags) & 3u) != 1u || !(CSR(satp) & 0x80000000u) ||
	    linear >= window || size > window - linear)
		return NULL;
	return guest_ram + MINIRV32_KERNEL_LINEAR_PHYS_OFFSET + linear;
}

static bool RV32HleCopy(uint32_t destination, uint32_t source, uint32_t size)
{
	while (size != 0u) {
		uint32_t source_span;
		uint32_t destination_span;
		uint32_t chunk;
		const uint8_t *from = RV32HleMap(source, ACCESS_READ, &source_span);
		uint8_t *to = from != NULL ?
			RV32HleMap(destination, ACCESS_WRITE, &destination_span) :
			NULL;

		if (to == NULL)
			return false;
		chunk = size < source_span ? size : source_span;
		if (chunk > destination_span)
			chunk = destination_span;
		memcpy(to, from, chunk);
		source += chunk;
		destination += chunk;
		size -= chunk;
	}
	return true;
}

static bool RV32HleFill(uint32_t destination, uint8_t value, uint32_t size)
{
	while (size != 0u) {
		uint32_t span;
		uint32_t chunk;
		uint8_t *to = RV32HleMap(destination, ACCESS_WRITE, &span);

		if (to == NULL)
			return false;
		chunk = size < span ? size : span;
		memset(to, value, chunk);
		destination += chunk;
		size -= chunk;
	}
	return true;
}

/* EROFS decompresses up to a few hundred KiB at once; larger calls stay
 * with the guest decoder. */
#define RV32_HLE_LZ4_PAGES 65u
static uint8_t *rv32_hle_lz4_in[RV32_HLE_LZ4_PAGES];
static uint8_t *rv32_hle_lz4_out[RV32_HLE_LZ4_PAGES];

/* Host address of every guest page of [address, address + size). */
static bool RV32HleMapPages(uint32_t address, uint32_t size, int access,
			    uint8_t **pages, uint32_t *count)
{
	uint32_t first = address & 0xfffu;

	*count = (first + size + 0xfffu) >> 12;
	if (size == 0u || size > UINT32_MAX - address ||
	    *count > RV32_HLE_LZ4_PAGES)
		return false;
	for (uint32_t i = 0; i < *count; i++) {
		uint32_t span;

		pages[i] = RV32HleMap((address & ~0xfffu) + (i << 12), access,
				      &span);
		if (pages[i] == NULL || span != 0x1000u)
			return false;
	}
	return true;
}

/* LZ4_decompress_safe{,_partial}: the result is the output size, or the
 * call is left to the guest decoder (see guest_lz4.h). */
static bool RV32HleLz4(uint32_t source, uint32_t destination,
		       int32_t source_size, int32_t capacity, bool partial)
{
	struct guest_lz4_buffer in;
	struct guest_lz4_buffer out;
	uint32_t in_pages;
	uint32_t out_pages;
	uint32_t source_in_destination = UINT32_MAX;

	if (source_size <= 0 || capacity <= 0 ||
	    !RV32HleMapPages(source, (uint32_t)source_size, ACCESS_READ,
			     rv32_hle_lz4_in, &in_pages) ||
	    !RV32HleMapPages(destination, (uint32_t)capacity, ACCESS_WRITE,
			     rv32_hle_lz4_out, &out_pages))
		return false;
	if (source - destination < (uint32_t)capacity) {
		/* EROFS in-place: the source starts inside the output mapping. */
		source_in_destination = source - destination;
	} else if (destination - source < (uint32_t)source_size) {
		return false;
	} else {
		/* Disjoint addresses may share a page only as the same virtual
		 * page (EROFS input right after the output); another mapping of
		 * the same RAM could overlap bytes. */
		for (uint32_t i = 0; i < in_pages; i++)
			for (uint32_t j = 0; j < out_pages; j++)
				if (rv32_hle_lz4_in[i] == rv32_hle_lz4_out[j] &&
				    (source >> 12) + i != (destination >> 12) + j)
					return false;
	}
	in = (struct guest_lz4_buffer) {
		rv32_hle_lz4_in, source & 0xfffu, (uint32_t)source_size };
	out = (struct guest_lz4_buffer) {
		rv32_hle_lz4_out, destination & 0xfffu, (uint32_t)capacity };
	if (!guest_lz4_decode(&in, &out, partial, source_in_destination))
		return false;
	core.regs[10] = (uint32_t)capacity;
	rv32_profile_jit_add(RV32_PROFILE_HLE_BYTES, (uint32_t)capacity);
	return true;
}

/* LZ4_compress_fast() for zram: a page from the kernel's linear map into a
 * buffer there.  The block is the host encoder's (any valid block decodes
 * the same); 0 when it does not fit, as the kernel reports that. */
static bool RV32HleLz4Compress(uint32_t source, uint32_t destination,
			       int32_t source_size, int32_t capacity)
{
	const uint8_t *from;
	uint8_t *to;
	uint32_t size;

	if (source_size <= 0 || capacity <= 0 ||
	    (uint32_t)source_size > GUEST_LZ4_ENCODE_MAX)
		return false;
	from = RV32HleLinear(source, (uint32_t)source_size);
	to = RV32HleLinear(destination, (uint32_t)capacity);
	if (from == NULL || to == NULL ||
	    destination - source < (uint32_t)source_size ||
	    source - destination < (uint32_t)capacity)
		return false;
	for (uint32_t page = destination >> 12;
	     page <= (destination + (uint32_t)capacity - 1u) >> 12; page++)
		rv32_native_jit_note_write_page(
			(uint32_t)(to - guest_ram) + (page << 12) - (destination & ~0xfffu));
	size = guest_lz4_encode(from, (uint32_t)source_size, to, (uint32_t)capacity);
	core.regs[10] = size;
	rv32_profile_jit_add(RV32_PROFILE_HLE_BYTES, (uint32_t)source_size);
	return true;
}

/* rv32_native_jit_hle_fn: a0..a4 are the call's arguments, ra its return. */
static bool RV32HleRun(uint32_t kind)
{
	uint32_t *regs = core.regs;
	uint32_t a0 = regs[10];
	uint32_t a1 = regs[11];
	uint32_t a2 = regs[12];

	switch (kind) {
	case GUEST_HLE_MEMCPY:
	case GUEST_HLE_MEMMOVE:
		if (a2 == 0u)
			return true;
		if (a0 - a1 >= a2 && a1 - a0 >= a2) {
			if (!RV32HleCopy(a0, a1, a2))
				return false;
		} else {
			/* Overlap: memcpy's result is the guest code's own business.
			 * A move is redone only when it happens in one piece. */
			uint8_t *to = RV32HleLinear(a0, a2);
			uint8_t *from = RV32HleLinear(a1, a2);

			if (kind != GUEST_HLE_MEMMOVE || to == NULL || from == NULL)
				return false;
			memmove(to, from, a2);
		}
		rv32_profile_jit_add(RV32_PROFILE_HLE_BYTES, a2);
		return true;
	case GUEST_HLE_MEMSET:
		if (!RV32HleFill(a0, (uint8_t)a1, a2))
			return false;
		rv32_profile_jit_add(RV32_PROFILE_HLE_BYTES, a2);
		return true;
	case GUEST_HLE_DIV64_32: {
		/* *n /= base; return the remainder.  The guest code reads and
		 * writes *n as two aligned words. */
		uint32_t span;
		uint8_t *n;
		uint64_t dividend;
		uint64_t quotient;

		if (a1 == 0u || (a0 & 3u) != 0u)
			return false;
		n = RV32HleMap(a0, ACCESS_WRITE, &span);
		if (n == NULL || span < sizeof(dividend))
			return false;
		memcpy(&dividend, n, sizeof(dividend));
		quotient = dividend / a1;
		memcpy(n, &quotient, sizeof(quotient));
		regs[10] = (uint32_t)(dividend - quotient * a1);
		return true;
	}
	case GUEST_HLE_LZ4_SAFE:
		return RV32HleLz4(a0, a1, (int32_t)a2, (int32_t)regs[13], false);
	case GUEST_HLE_LZ4_SAFE_PARTIAL: {
		int32_t target = (int32_t)regs[13];
		int32_t capacity = (int32_t)regs[14];

		return RV32HleLz4(a0, a1, (int32_t)a2,
				  target < capacity ? target : capacity, true);
	}
	case GUEST_HLE_LZ4_COMPRESS:
		return RV32HleLz4Compress(a0, a1, (int32_t)a2, (int32_t)regs[13]);
	default:
		return false;
	}
}
#endif

static uint32_t IRAM_ATTR RV32JitMemoryLoad(
	struct rv32_native_jit_runtime *runtime,
	uint32_t virtual_address, uint32_t descriptor)
{
	struct MiniRV32IMAState *state = runtime->opaque;
	uint8_t *image = runtime->image;
	uint32_t funct3 = descriptor & 7u;
	uint32_t width = RV32JitMemoryWidth(funct3, true);
	uint32_t physical_address;
	uint32_t offset;
	uint32_t value = 0u;
	int fault;
	int uncached = 0;

	runtime->memory_calls++;
	if (width == 0u) {
		rv32_native_jit_memory_exit(runtime, descriptor, 3u, 0u);
		return 1u;
	}
	physical_address = MiniRV32Translate(
		state, image, virtual_address, ACCESS_READ, &fault, &uncached);
	if (fault) {
		rv32_native_jit_memory_exit(runtime, descriptor, 14u,
					    virtual_address);
		return 1u;
	}

	if (physical_address >= MINIRV32_RAM_IMAGE_OFFSET) {
		offset = physical_address - MINIRV32_RAM_IMAGE_OFFSET;
		if (offset < ram_amt && width <= ram_amt - offset) {
			RV32_JIT_TRACE_MEMORY(runtime, uncached ?
				RV32_JIT_TRACE_MEMORY_UNCACHED : RV32_JIT_TRACE_MEMORY_RAM);
			if (uncached) {
				if (width == 4u)
					value = MiniRV32Load4Uncached(offset);
				else if (width == 2u)
					value = MiniRV32Load2Uncached(offset);
				else
					value = MiniRV32Load1Uncached(offset);
			} else if (width == 4u) {
				value = MiniRV32Load4(image, offset);
			} else if (width == 2u) {
				value = MiniRV32Load2(image, offset);
			} else {
				value = MiniRV32Load1(image, offset);
			}
			if (funct3 == 0u)
				value = (uint32_t)(int32_t)(int8_t)value;
			else if (funct3 == 1u)
				value = (uint32_t)(int32_t)(int16_t)value;
			runtime->memory_value = value;
			if (!uncached && ram_amt >= 0x1000u &&
			    (offset & ~0xfffu) <= ram_amt - 0x1000u)
				RV32JitCacheReadPage(runtime, virtual_address,
						       image + offset);
			return 0u;
		}
	}

#ifdef MINIRV32_ROM_APERTURE
	/* Read-only flash: an ordinary cached load whatever the PTE's uncached
	 * bit says, and the page joins the native read cache. */
	if (MiniRV32RomOffset(image, physical_address, &offset) &&
	    physical_address - MINIRV32_ROM_BASE <= MINIRV32_ROM_SIZE - width) {
		if (width == 4u)
			value = MiniRV32Load4(image, offset);
		else if (width == 2u)
			value = MiniRV32Load2(image, offset);
		else
			value = MiniRV32Load1(image, offset);
		if (funct3 == 0u)
			value = (uint32_t)(int32_t)(int8_t)value;
		else if (funct3 == 1u)
			value = (uint32_t)(int32_t)(int16_t)value;
		runtime->memory_value = value;
		RV32JitCacheReadPage(runtime, virtual_address, image + offset);
		return 0u;
	}
#endif

#if defined(CONFIG_RV32_DISPLAY_ACCEL) && CONFIG_RV32_DISPLAY_ACCEL
	offset = physical_address - MINIRV32_LINEAR_MMIO_BASE;
	if (linear_mmio_host != NULL &&
	    offset < MINIRV32_LINEAR_MMIO_SIZE &&
	    width <= MINIRV32_LINEAR_MMIO_SIZE - offset) {
		const uint8_t *source = linear_mmio_host + offset;

		RV32_JIT_TRACE_MEMORY(runtime, RV32_JIT_TRACE_MEMORY_FRAMEBUFFER);
		MINIRV32_LINEAR_MMIO_ACCESS();
		if (width == 4u)
			memcpy(&value, source, sizeof(value));
		else if (width == 2u) {
			uint16_t half;

			memcpy(&half, source, sizeof(half));
			value = half;
		} else {
			value = source[0];
		}
		if (funct3 == 0u)
			value = (uint32_t)(int32_t)(int8_t)value;
		else if (funct3 == 1u)
			value = (uint32_t)(int32_t)(int16_t)value;
		runtime->memory_value = value;
		if (MINIRV32_LINEAR_MMIO_SIZE >= 0x1000u &&
		    (offset & ~0xfffu) <=
			MINIRV32_LINEAR_MMIO_SIZE - 0x1000u)
			RV32JitCacheReadPage(runtime, virtual_address, source);
		return 0u;
	}
#endif

	if (MINIRV32_IS_MMIO_ADDRESS(physical_address)) {
		RV32_JIT_TRACE_MEMORY(runtime, RV32_JIT_TRACE_MEMORY_DEVICE);
		/* Let the interpreter perform side-effecting device accesses exactly
		 * once.  Earlier native instructions remain committed. */
		rv32_native_jit_memory_exit(runtime, descriptor, 0u, 0u);
		return 1u;
	}
	rv32_native_jit_memory_exit(runtime, descriptor, 6u,
				    virtual_address);
	return 1u;
}

static uint32_t IRAM_ATTR RV32JitMemoryStore(
	struct rv32_native_jit_runtime *runtime,
	uint32_t virtual_address, uint32_t value, uint32_t descriptor)
{
	struct MiniRV32IMAState *state = runtime->opaque;
	uint8_t *image = runtime->image;
	uint32_t funct3 = descriptor & 7u;
	uint32_t width = RV32JitMemoryWidth(funct3, false);
	uint32_t physical_address;
	uint32_t offset;
	int fault;
	int uncached = 0;

	runtime->memory_calls++;
	if (width == 0u) {
		rv32_native_jit_memory_exit(runtime, descriptor, 3u, 0u);
		return 1u;
	}
	physical_address = MiniRV32Translate(
		state, image, virtual_address, ACCESS_WRITE, &fault, &uncached);
	if (fault) {
		rv32_native_jit_memory_exit(runtime, descriptor, 16u,
					    virtual_address);
		return 1u;
	}

	if (physical_address >= MINIRV32_RAM_IMAGE_OFFSET) {
		offset = physical_address - MINIRV32_RAM_IMAGE_OFFSET;
		if (offset < ram_amt && width <= ram_amt - offset) {
			rv32_native_jit_note_write_page(offset);
			RV32_JIT_TRACE_MEMORY(runtime, uncached ?
				RV32_JIT_TRACE_MEMORY_UNCACHED : RV32_JIT_TRACE_MEMORY_RAM);
			if (uncached) {
				if (width == 4u)
					MiniRV32Store4Uncached(offset, value);
				else if (width == 2u)
					MiniRV32Store2Uncached(offset, value);
				else
					MiniRV32Store1Uncached(offset, value);
			} else if (width == 4u) {
				MiniRV32Store4(image, offset, value);
			} else if (width == 2u) {
				MiniRV32Store2(image, offset, value);
			} else {
				MiniRV32Store1(image, offset, value);
			}
			if (!uncached && ram_amt >= 0x1000u &&
			    (offset & ~0xfffu) <= ram_amt - 0x1000u)
				RV32JitCacheWritePage(runtime, virtual_address,
							image + offset,
							UINT32_MAX, 0u);
			return 0u;
		}
	}

#if defined(CONFIG_RV32_DISPLAY_ACCEL) && CONFIG_RV32_DISPLAY_ACCEL
	offset = physical_address - MINIRV32_LINEAR_MMIO_BASE;
	if (linear_mmio_host != NULL &&
	    offset < MINIRV32_LINEAR_MMIO_SIZE &&
	    width <= MINIRV32_LINEAR_MMIO_SIZE - offset) {
		uint8_t *destination = linear_mmio_host + offset;

		RV32_JIT_TRACE_MEMORY(runtime, RV32_JIT_TRACE_MEMORY_FRAMEBUFFER);
		MINIRV32_LINEAR_MMIO_ACCESS();
		if (width == 4u)
			memcpy(destination, &value, sizeof(value));
		else if (width == 2u)
			memcpy(destination, &value, sizeof(uint16_t));
		else
			destination[0] = (uint8_t)value;
		if (offset < runtime->result.linear_dirty_start)
			runtime->result.linear_dirty_start = offset;
		if (offset + width > runtime->result.linear_dirty_end)
		runtime->result.linear_dirty_end = offset + width;
		runtime->result.linear_store_calls++;
		runtime->result.linear_store_bytes += width;
		if (MINIRV32_LINEAR_MMIO_SIZE >= 0x1000u &&
		    (offset & ~0xfffu) <=
			MINIRV32_LINEAR_MMIO_SIZE - 0x1000u) {
			uint32_t page_start = offset & ~0xfffu;

			RV32JitCacheWritePage(runtime, virtual_address,
						    destination, page_start,
						    page_start + 0x1000u);
		}
		return 0u;
	}
#endif

	if (MINIRV32_IS_MMIO_ADDRESS(physical_address)) {
		RV32_JIT_TRACE_MEMORY(runtime, RV32_JIT_TRACE_MEMORY_DEVICE);
		rv32_native_jit_memory_exit(runtime, descriptor, 0u, 0u);
		return 1u;
	}
	rv32_native_jit_memory_exit(runtime, descriptor, 8u,
				    virtual_address);
	return 1u;
}
#endif

#if CONFIG_RV32_JIT_EXIT_TRACE
static uint32_t JitTraceAddressSpace(void) { return core.satp; }
static uint64_t jit_trace_last_time;
static RV32_STATE_BSS struct rv32_jit_trace_snapshot jit_trace_snapshot;

static __attribute__((noinline, noclone)) void JitTraceReport(uint64_t now)
{
	uint64_t elapsed = now - jit_trace_last_time;
	if (elapsed < (uint64_t)CONFIG_RV32_JIT_EXIT_TRACE_INTERVAL_MS * 1000u)
		return;
	jit_trace_last_time = now;
	struct rv32_jit_trace_snapshot *s = &jit_trace_snapshot;
	rv32_jit_trace_take(s);
	uint64_t exits = 0u, cycles = 0u, candidates = 0u;
	for (uint32_t i = 0; i < RV32_JIT_TRACE_REASON_COUNT; i++) {
		if (i < RV32_JIT_TRACE_UNSUPPORTED) {
			exits += s->reasons[i].count;
			cycles += s->reasons[i].cycles;
		} else candidates += s->reasons[i].count;
	}
	if (!exits && !candidates && !s->refills.count) return;
	printf("JIT-EXIT at-us=%" PRIu64 " us=%" PRIu64 " checked=%" PRIu64 " callback-cycles=%" PRIu64
	       " candidates=%" PRIu64 " refill=%" PRIu64 "/%" PRIu64
	       " refill-cycles=%" PRIu64 " dropped=%" PRIu64 "/%" PRIu64 "\n",
	       now, elapsed, exits, cycles, candidates, s->refill_resumes,
	       s->refills.count, s->refills.cycles, s->dropped.count, s->dropped.cycles);
	for (uint32_t i = 0; i < RV32_JIT_TRACE_REASON_COUNT; i++) {
		if (s->reasons[i].count)
			printf("JIT-REASON %s count=%" PRIu64 " cycles=%" PRIu64 "\n",
			       rv32_jit_trace_reason_name(i), s->reasons[i].count,
			       s->reasons[i].cycles);
	}
	for (uint32_t i = 0; i < RV32_JIT_TRACE_TOP; i++) {
		const struct rv32_jit_trace_record *r = &s->top[i];
		if (!r->total.count) break;
		printf("JIT-SITE %s pc=%08" PRIx32 " phys-off=%08" PRIx32
		       " insn=%08" PRIx32 " satp=%08" PRIx32
		       " va=%08" PRIx32 "..%08" PRIx32 " trap=%" PRIu32
		       " count=%" PRIu64 " cycles=%" PRIu64 "\n",
		       rv32_jit_trace_reason_name(r->reason), r->pc, r->physical_offset,
		       r->instruction, r->satp, r->first_address, r->last_address,
		       r->trap, r->total.count, r->total.cycles);
	}
}
#endif

#if CONFIG_RV32_EMULATOR_PERF_STATS
static uint64_t emulator_perf_last_time;
static uint64_t emulator_perf_last_cycle;
static struct virtio_net_bridge_perf_stats emulator_perf_last_net;
static struct virtio_blk_bridge_perf_stats emulator_perf_last_blk;

static void EmulatorPerfStart(uint64_t current_time)
{
	emulator_perf_last_time = current_time;
	emulator_perf_last_cycle =
		((uint64_t)core.cycleh << 32) | core.cyclel;
	memset(&core.perf, 0, sizeof(core.perf));
	HostCacheStatsReset();
	virtio_net_bridge_perf_snapshot(&emulator_perf_last_net);
	virtio_blk_bridge_perf_snapshot(&emulator_perf_last_blk);
}
static __attribute__((noinline, noclone)) void EmulatorPerfReport(uint64_t current_time)
{
	uint64_t elapsed_us = current_time - emulator_perf_last_time;
	uint64_t guest_cycle = ((uint64_t)core.cycleh << 32) | core.cyclel;
	uint64_t guest_delta = guest_cycle - emulator_perf_last_cycle;
	uint64_t mips_x100;
	uint64_t l1_total;
	uint64_t l2_total;
	struct host_cache_stats cache_stats;
	struct virtio_net_bridge_perf_stats net;
	struct virtio_blk_bridge_perf_stats blk;
	uint32_t l1_hit_permille;
	uint32_t l2_hit_permille;
	uint64_t opcode_total = 0;

	if (elapsed_us < EMULATOR_PERF_INTERVAL_US)
		return;

	HostCacheStatsReadAndReset(&cache_stats);
	virtio_net_bridge_perf_snapshot(&net);
	virtio_blk_bridge_perf_snapshot(&blk);
	mips_x100 = elapsed_us ? guest_delta * 100u / elapsed_us : 0u;
	/* HIT counts every completed access and NXTLVL_RD every line fill; the
	 * MISS register counts stall time, not misses (ESP32-P4 TRM, cache
	 * performance counters), so it has no place in a hit rate. */
	l1_total = cache_stats.l1_hits;
	l2_total = cache_stats.l2_hits;
	l1_hit_permille = l1_total > cache_stats.l1_next_reads ?
		(uint32_t)((l1_total - cache_stats.l1_next_reads) * 1000u /
			   l1_total) : 0u;
	l2_hit_permille = l2_total > cache_stats.l2_next_reads ?
		(uint32_t)((l2_total - cache_stats.l2_next_reads) * 1000u /
			   l2_total) : 0u;
	for (unsigned int i = 0; i < 32; i++)
		opcode_total += core.perf.opcode[i];

	printf("\n[RV32 perf] %" PRIu64 ".%02" PRIu64 " MIPS; "
	       "L1D %" PRIu32 ".%01" PRIu32 "%%; "
	       "L2 %" PRIu32 ".%01" PRIu32 "%%\n",
	       mips_x100 / 100u, mips_x100 % 100u,
	       l1_hit_permille / 10u, l1_hit_permille % 10u,
	       l2_hit_permille / 10u, l2_hit_permille % 10u);
	printf("[RV32 perf] cache L1 hit=%" PRIu32 " miss=%" PRIu32
	       " conflict=%" PRIu32 " next-r=%" PRIu32 " next-w=%" PRIu32
	       "; L2 hit=%" PRIu32 " miss=%" PRIu32 " conflict=%" PRIu32
	       " PSRAM-r=%" PRIu32 " PSRAM-w=%" PRIu32 "\n",
	       cache_stats.l1_hits, cache_stats.l1_misses,
	       cache_stats.l1_conflicts, cache_stats.l1_next_reads,
	       cache_stats.l1_next_writes, cache_stats.l2_hits,
	       cache_stats.l2_misses, cache_stats.l2_conflicts,
	       cache_stats.l2_next_reads, cache_stats.l2_next_writes);
	printf("[RV32 perf] TLB fast=%" PRIu32 " full=%" PRIu32
	       " walk=%" PRIu32 " flush=%" PRIu32
	       " (satp=%" PRIu32 " sfence=%" PRIu32
	       ": global=%" PRIu32 " page=%" PRIu32 ")"
	       "; page exec=%" PRIu32 " read=%" PRIu32
	       " (RAM=%" PRIu32 ") write=%" PRIu32
	       " (RAM=%" PRIu32 ") linear=%" PRIu32 "\n",
	       core.perf.fast_tlb_hits, core.perf.tlb_hits,
	       core.perf.tlb_walks, core.perf.tlb_flushes,
	       core.perf.satp_writes, core.perf.sfence_vma,
	       core.perf.sfence_global, core.perf.sfence_page,
	       core.perf.exec_page_hits, core.perf.read_page_hits,
	       core.perf.read_ram_page_hits, core.perf.write_page_hits,
	       core.perf.write_ram_page_hits, core.perf.kernel_linear_hits);
	#define OPCODE_PERCENT(index) \
		(opcode_total ? \
		 (uint32_t)((uint64_t)core.perf.opcode[(index)] * 1000u / \
		 opcode_total) : 0u)
	printf("[RV32 perf] op load=%" PRIu32 ".%01" PRIu32
	       "%% store=%" PRIu32 ".%01" PRIu32
	       "%% opimm=%" PRIu32 ".%01" PRIu32
	       "%% op=%" PRIu32 ".%01" PRIu32
	       "%% branch=%" PRIu32 ".%01" PRIu32 "%%\n",
	       OPCODE_PERCENT(0) / 10u, OPCODE_PERCENT(0) % 10u,
	       OPCODE_PERCENT(8) / 10u, OPCODE_PERCENT(8) % 10u,
	       OPCODE_PERCENT(4) / 10u, OPCODE_PERCENT(4) % 10u,
	       OPCODE_PERCENT(12) / 10u, OPCODE_PERCENT(12) % 10u,
	       OPCODE_PERCENT(24) / 10u, OPCODE_PERCENT(24) % 10u);
	printf("[RV32 perf] op jal=%" PRIu32 ".%01" PRIu32
	       "%% jalr=%" PRIu32 ".%01" PRIu32
	       "%% lui=%" PRIu32 ".%01" PRIu32
	       "%% auipc=%" PRIu32 ".%01" PRIu32
	       "%% system=%" PRIu32 ".%01" PRIu32
	       "%% atomic=%" PRIu32 ".%01" PRIu32 "%%\n",
	       OPCODE_PERCENT(27) / 10u, OPCODE_PERCENT(27) % 10u,
	       OPCODE_PERCENT(25) / 10u, OPCODE_PERCENT(25) % 10u,
	       OPCODE_PERCENT(13) / 10u, OPCODE_PERCENT(13) % 10u,
	       OPCODE_PERCENT(5) / 10u, OPCODE_PERCENT(5) % 10u,
	       OPCODE_PERCENT(28) / 10u, OPCODE_PERCENT(28) % 10u,
	       OPCODE_PERCENT(11) / 10u, OPCODE_PERCENT(11) % 10u);
	printf("[RV32 perf] load lb=%" PRIu32 " lh=%" PRIu32
	       " lw=%" PRIu32 " lbu=%" PRIu32 " lhu=%" PRIu32
	       "; store sb=%" PRIu32 " sh=%" PRIu32 " sw=%" PRIu32 "\n",
	       core.perf.load_funct3[0], core.perf.load_funct3[1],
	       core.perf.load_funct3[2], core.perf.load_funct3[4],
	       core.perf.load_funct3[5], core.perf.store_funct3[0],
	       core.perf.store_funct3[1], core.perf.store_funct3[2]);
	printf("[RV32 perf] opimm add=%" PRIu32 " shift-l=%" PRIu32
	       " slt=%" PRIu32 "/%" PRIu32 " xor=%" PRIu32
	       " shift-r=%" PRIu32 " or=%" PRIu32 " and=%" PRIu32 "\n",
	       core.perf.opimm_funct3[0], core.perf.opimm_funct3[1],
	       core.perf.opimm_funct3[2], core.perf.opimm_funct3[3],
	       core.perf.opimm_funct3[4], core.perf.opimm_funct3[5],
	       core.perf.opimm_funct3[6], core.perf.opimm_funct3[7]);
	printf("[RV32 perf] op add/sub=%" PRIu32 " shift-l=%" PRIu32
	       " slt=%" PRIu32 "/%" PRIu32 " xor=%" PRIu32
	       " shift-r=%" PRIu32 " or=%" PRIu32 " and=%" PRIu32 "\n",
	       core.perf.op_funct3[0], core.perf.op_funct3[1],
	       core.perf.op_funct3[2], core.perf.op_funct3[3],
	       core.perf.op_funct3[4], core.perf.op_funct3[5],
	       core.perf.op_funct3[6], core.perf.op_funct3[7]);
	printf("[RV32 perf] branch eq=%" PRIu32 " ne=%" PRIu32
	       " lt=%" PRIu32 " ge=%" PRIu32 " ltu=%" PRIu32
	       " geu=%" PRIu32 "; mul/div=%" PRIu32 "\n",
	       core.perf.branch_funct3[0], core.perf.branch_funct3[1],
	       core.perf.branch_funct3[4], core.perf.branch_funct3[5],
	       core.perf.branch_funct3[6], core.perf.branch_funct3[7],
	       core.perf.muldiv);
	printf("[I/O perf] net RX=%" PRIu32 "/%" PRIu32 " KiB drop=%" PRIu32
	       " TX=%" PRIu32 "/%" PRIu32 " KiB err=%" PRIu32
	       "; complete=%" PRIu32 "/%" PRIu32
	       " irq=%" PRIu32 " wake=%" PRIu32 " resched=%" PRIu32 "\n",
	       net.rx_frames - emulator_perf_last_net.rx_frames,
	       (net.rx_bytes - emulator_perf_last_net.rx_bytes) / 1024u,
	       net.rx_drops - emulator_perf_last_net.rx_drops,
	       net.tx_frames - emulator_perf_last_net.tx_frames,
	       (net.tx_bytes - emulator_perf_last_net.tx_bytes) / 1024u,
	       net.tx_errors - emulator_perf_last_net.tx_errors,
	       net.rx_completions - emulator_perf_last_net.rx_completions,
	       net.tx_completions - emulator_perf_last_net.tx_completions,
	       net.guest_interrupts - emulator_perf_last_net.guest_interrupts,
	       net.service_wakes - emulator_perf_last_net.service_wakes,
	       net.budget_reschedules -
		       emulator_perf_last_net.budget_reschedules);
	printf("[I/O perf] block request=%" PRIu32 " read=%" PRIu32
	       " KiB write=%" PRIu32 " KiB cache hit=%" PRIu32
	       " miss=%" PRIu32 "\n",
	       blk.requests - emulator_perf_last_blk.requests,
	       (blk.read_bytes - emulator_perf_last_blk.read_bytes) / 1024u,
	       (blk.write_bytes - emulator_perf_last_blk.write_bytes) / 1024u,
	       blk.cache_hits - emulator_perf_last_blk.cache_hits,
	       blk.cache_misses - emulator_perf_last_blk.cache_misses);
	#undef OPCODE_PERCENT

	emulator_perf_last_time = current_time;
	emulator_perf_last_cycle = guest_cycle;
	memset(&core.perf, 0, sizeof(core.perf));
	emulator_perf_last_net = net;
	emulator_perf_last_blk = blk;
}
#endif

#if CONFIG_RV32_HOST_PERF_STATS
static uint64_t host_perf_last_time;
static uint64_t host_perf_last_cycle;
static configRUN_TIME_COUNTER_TYPE host_perf_last_idle[2];
/* HostPerfReport runs underneath app_main's persistent frame and several
 * printf frames.  Keep the comparatively large display/JIT snapshots out of
 * the main-task stack so adding generic counters cannot make the next
 * interrupt's 128-byte context push cross the hardware stack-guard bound.
 * The reporter is called only by the emulator task, so static storage is
 * sufficient and does not weaken synchronization in either stats producer. */
static struct display_bridge_perf_stats host_perf_display;
#if CONFIG_RV32_NATIVE_JIT
static struct rv32_native_jit_stats host_perf_last_jit;
static struct rv32_native_jit_stats host_perf_current_jit;
#endif

static uint32_t HostPerfBusyPermille(
	configRUN_TIME_COUNTER_TYPE current_idle,
	configRUN_TIME_COUNTER_TYPE previous_idle,
	uint64_t elapsed_us)
{
	uint64_t idle_us = current_idle - previous_idle;

	if (idle_us >= elapsed_us)
		return 0;
	return 1000u - (uint32_t)(idle_us * 1000u / elapsed_us);
}

/* The emulator core's own counters, read where this report runs: cycles and
 * retired instructions, and the ESP32-P4's two branch events (TRM: mhpmevent8
 * = 6, conditional-branch mispredictions; mhpmevent9 = 7, conditional
 * branches; counters 0xB08 and 0xB09 with their high halves at 0xB88 and
 * 0xB89, counting once their mcountinhibit bits are cleared). */
#define HOST_PERF_COUNTER64(name, low, high) \
static inline uint64_t name(void) \
{ \
	uint32_t lo, hi, hi2; \
\
	do { \
		__asm__ volatile("csrr %0, " #high : "=r"(hi)); \
		__asm__ volatile("csrr %0, " #low : "=r"(lo)); \
		__asm__ volatile("csrr %0, " #high : "=r"(hi2)); \
	} while (hi != hi2); \
	return ((uint64_t)hi << 32) | lo; \
}
HOST_PERF_COUNTER64(HostPerfCpuCycles, 0xB00, 0xB80)
HOST_PERF_COUNTER64(HostPerfCpuInstructions, 0xB02, 0xB82)
HOST_PERF_COUNTER64(HostPerfCpuMispredicts, 0xB08, 0xB88)
HOST_PERF_COUNTER64(HostPerfCpuBranches, 0xB09, 0xB89)
#undef HOST_PERF_COUNTER64
enum {
	HOST_PERF_CPU_CYCLES,
	HOST_PERF_CPU_INSNS,
	HOST_PERF_CPU_MISPREDICTS,
	HOST_PERF_CPU_BRANCHES,
	HOST_PERF_CPU_COUNTERS
};
static uint64_t host_perf_last_cpu[HOST_PERF_CPU_COUNTERS];

static void HostPerfCpuCountersRead(uint64_t *values)
{
	values[HOST_PERF_CPU_CYCLES] = HostPerfCpuCycles();
	values[HOST_PERF_CPU_INSNS] = HostPerfCpuInstructions();
	values[HOST_PERF_CPU_MISPREDICTS] = HostPerfCpuMispredicts();
	values[HOST_PERF_CPU_BRANCHES] = HostPerfCpuBranches();
}

static void HostPerfCpuCountersEnable(void)
{
	static bool enabled;

	if (enabled)
		return;
	enabled = true;
	__asm__ volatile("csrw 0x328, %0" : : "r"(6u));
	__asm__ volatile("csrw 0x329, %0" : : "r"(7u));
	__asm__ volatile("csrw 0xB08, zero\n\tcsrw 0xB88, zero\n\t"
			 "csrw 0xB09, zero\n\tcsrw 0xB89, zero");
	/* CY, IR, HPM8 and HPM9 count. */
	__asm__ volatile("csrc 0x320, %0" : : "r"(0x305u));
}

static void HostPerfStart(uint64_t current_time)
{
	host_perf_last_time = current_time;
	HostPerfCpuCountersEnable();
	HostPerfCpuCountersRead(host_perf_last_cpu);
	host_perf_last_cycle =
		((uint64_t)core.cycleh << 32) | core.cyclel;
	for (BaseType_t cpu = 0; cpu < 2; cpu++)
		host_perf_last_idle[cpu] =
			ulTaskGetIdleRunTimeCounterForCore(cpu);
	display_bridge_perf_read_and_reset(&host_perf_display);
#if CONFIG_RV32_NATIVE_JIT
	rv32_native_jit_get_stats(&host_perf_last_jit);
#endif
}

static __attribute__((noinline, noclone)) void HostPerfReport(uint64_t current_time)
{
	uint64_t elapsed_us = current_time - host_perf_last_time;
	uint64_t guest_cycle;
	uint64_t guest_delta;
	uint64_t mips_x100;
	configRUN_TIME_COUNTER_TYPE idle[2];
	uint32_t busy[2];
	uint32_t display_busy_permille;
#if CONFIG_RV32_NATIVE_JIT
	struct rv32_native_jit_stats *jit = &host_perf_current_jit;
#endif
	struct display_bridge_perf_stats *display = &host_perf_display;

	if (elapsed_us < HOST_PERF_INTERVAL_US)
		return;

	guest_cycle = ((uint64_t)core.cycleh << 32) | core.cyclel;
	guest_delta = guest_cycle - host_perf_last_cycle;
	mips_x100 = elapsed_us ? guest_delta * 100u / elapsed_us : 0u;
	for (BaseType_t cpu = 0; cpu < 2; cpu++) {
		idle[cpu] = ulTaskGetIdleRunTimeCounterForCore(cpu);
		busy[cpu] = HostPerfBusyPermille(
			idle[cpu], host_perf_last_idle[cpu], elapsed_us);
	}
	display_bridge_perf_read_and_reset(display);
	display_busy_permille = elapsed_us
		? (uint32_t)((uint64_t)display->service_us * 1000u / elapsed_us)
		: 0u;

	printf("\n[Host perf] CPU0 %" PRIu32 ".%" PRIu32
	       "%% busy, CPU1 %" PRIu32 ".%" PRIu32
	       "%% busy; RV32 %" PRIu64 ".%02" PRIu64 " MIPS\n",
	       busy[0] / 10u, busy[0] % 10u,
	       busy[1] / 10u, busy[1] % 10u,
	       mips_x100 / 100u, mips_x100 % 100u);
	{
		uint64_t cpu[HOST_PERF_CPU_COUNTERS];
		uint64_t cycles, insns, branches, mispredicts;

		HostPerfCpuCountersRead(cpu);
		cycles = cpu[HOST_PERF_CPU_CYCLES] -
			host_perf_last_cpu[HOST_PERF_CPU_CYCLES];
		insns = cpu[HOST_PERF_CPU_INSNS] -
			host_perf_last_cpu[HOST_PERF_CPU_INSNS];
		mispredicts = cpu[HOST_PERF_CPU_MISPREDICTS] -
			host_perf_last_cpu[HOST_PERF_CPU_MISPREDICTS];
		branches = cpu[HOST_PERF_CPU_BRANCHES] -
			host_perf_last_cpu[HOST_PERF_CPU_BRANCHES];
		printf("[CPU0 perf] cycles=%" PRIu64 " insns=%" PRIu64
		       " cpi=%" PRIu64 ".%02" PRIu64 " branches=%" PRIu64
		       " mispredicts=%" PRIu64 " (%" PRIu64 ".%" PRIu64
		       "%% of branches)\n",
		       cycles, insns, insns ? cycles / insns : 0u,
		       insns ? cycles * 100u / insns % 100u : 0u,
		       branches, mispredicts,
		       branches ? mispredicts * 100u / branches : 0u,
		       branches ? mispredicts * 1000u / branches % 10u : 0u);
	}
	printf("[Display perf] worker %" PRIu32 ".%" PRIu32
	       "%%/%" PRIu32 " us, wake=%" PRIu32 " vsync=%" PRIu32
	       " cmd=%" PRIu32 " (fill=%" PRIu32 " copy=%" PRIu32
	       " write=%" PRIu32 " tile=%" PRIu32 ": set=%" PRIu32 " fill=%" PRIu32
	       " blit=%" PRIu32 " cursor=%" PRIu32 "), FIFO high=%" PRIu32
	       " busy=%" PRIu32 " slices=%" PRIu32
	       " deferred=%" PRIu32 "\n",
	       display_busy_permille / 10u, display_busy_permille % 10u,
	       display->service_us, display->service_wakes, display->vsyncs,
	       display->commands,
	       display->fill_commands, display->copy_commands,
	       display->bulk_write_commands,
	       display->tile_commands, display->tile_set_commands,
	       display->tile_fill_commands, display->tile_blit_commands,
	       display->tile_cursor_commands, display->fifo_high_water,
	       display->fifo_busy, display->fifo_slices,
	       display->fifo_deferred);
	printf("[Display perf] cache=%" PRIu32 " calls/%" PRIu32
	       " KiB/%" PRIu32 " us; PPA=%" PRIu32
	       " fills + %" PRIu32 " blits/%" PRIu32
	       " us; pixels PPA=%" PRIu32 " CPU-fill=%" PRIu32
	       " copy=%" PRIu32 " tile=%" PRIu32 "\n",
	       display->cache_syncs, display->cache_bytes / 1024u,
	       display->cache_us, display->ppa_fills, display->ppa_blits,
	       display->ppa_us, display->ppa_fill_pixels,
	       display->cpu_fill_pixels, display->copy_pixels, display->tile_pixels);
	printf("[Display flow] fb-store=%" PRIu32 " calls/%" PRIu32
	       " KiB; commit=%" PRIu32 "/%" PRIu32
	       " KiB; pv-write=%" PRIu32 "/%" PRIu32
	       " KiB; publish=%" PRIu32 "/%" PRIu32
	       " KiB; present=%" PRIu32 "/%" PRIu32
	       " KiB/%" PRIu32 " us (CPU=%" PRIu32
	       " deferred=%" PRIu32 ")\n",
	       display->framebuffer_store_calls,
	       display->framebuffer_store_bytes / 1024u,
	       display->framebuffer_commit_events,
	       display->framebuffer_commit_bytes / 1024u,
	       display->bulk_write_commands,
	       display->bulk_write_bytes / 1024u,
	       display->framebuffer_publish_events,
	       display->framebuffer_publish_bytes / 1024u,
	       display->framebuffer_present_events,
	       display->framebuffer_present_bytes / 1024u,
		       display->framebuffer_present_us,
		       display->framebuffer_present_cpu_copies,
		       display->framebuffer_present_deferred);
	printf("[Display lock] guest=%" PRIu32 " slices/%" PRIu32
	       " us wait/%" PRIu32 " us max; snapshot=%" PRIu32
	       " us wait/%" PRIu32 " us max\n",
	       display->framebuffer_guest_slices,
	       display->framebuffer_guest_wait_us,
	       display->framebuffer_guest_wait_max_us,
	       display->framebuffer_snapshot_wait_us,
	       display->framebuffer_snapshot_wait_max_us);
	printf("[Display async] compute-slices=%" PRIu32
	       " pixel-access-waits=%" PRIu32 "\n",
	       display->framebuffer_async_slices,
	       display->framebuffer_access_waits);
	#define DISPLAY_AVG_CYCLES(total, count) \
		((count) ? (total) / (count) : 0u)
	printf("[Display cycles] dispatch=%" PRIu32 " total/%" PRIu32
	       " avg/%" PRIu32 " max; avg/op fill=%" PRIu32
	       " copy=%" PRIu32 " image1=%" PRIu32 " write=%" PRIu32
	       " tile-set=%" PRIu32
	       " tile-fill=%" PRIu32 " tile-blit=%" PRIu32
	       " cursor=%" PRIu32 "\n",
	       display->command_cycles,
	       DISPLAY_AVG_CYCLES(display->command_cycles, display->commands),
	       display->command_max_cycles,
	       DISPLAY_AVG_CYCLES(display->fill_cycles, display->fill_commands),
	       DISPLAY_AVG_CYCLES(display->copy_cycles, display->copy_commands),
	       DISPLAY_AVG_CYCLES(display->image1_cycles,
				  display->image1_commands),
	       DISPLAY_AVG_CYCLES(display->bulk_write_cycles,
				  display->bulk_write_commands),
	       DISPLAY_AVG_CYCLES(display->tile_set_cycles,
				  display->tile_set_commands),
	       DISPLAY_AVG_CYCLES(display->tile_fill_cycles,
				  display->tile_fill_commands),
	       DISPLAY_AVG_CYCLES(display->tile_blit_cycles,
				  display->tile_blit_commands),
	       DISPLAY_AVG_CYCLES(display->tile_cursor_cycles,
				  display->tile_cursor_commands));
	printf("[Display cursor] toggles=%" PRIu32 " cycles=%" PRIu32
	       " total/%" PRIu32 " avg/%" PRIu32 " max\n",
	       display->cursor_toggles, display->cursor_toggle_cycles,
	       DISPLAY_AVG_CYCLES(display->cursor_toggle_cycles,
				  display->cursor_toggles),
	       display->cursor_toggle_max_cycles);
	#undef DISPLAY_AVG_CYCLES
	printf("[Display frame] samples=%" PRIu32 " total=%" PRIu32
	       " us avg=%" PRIu32 " us max=%" PRIu32
	       " us; cache overhead=%" PRIu32 " us\n",
	       display->frame_samples, display->frame_total_us,
	       display->frame_samples
		? display->frame_total_us / display->frame_samples : 0u,
	       display->frame_max_us, display->cache_us);
	printf("[Display producer] submit=%" PRIu32 "/%" PRIu32
	       " us, payload=%" PRIu32 " KiB (inline=%" PRIu32
	       " pool=%" PRIu32 "), shared-result=%" PRIu32
	       "; cross-core wake=%" PRIu32
	       "/%" PRIu32 " us\n",
	       display->producer_submissions, display->producer_us,
	       display->producer_payload_bytes / 1024u,
	       display->producer_inline_payloads,
	       display->producer_external_payloads,
	       display->producer_shared_results,
	       display->producer_wakes, display->producer_wake_us);
	printf("[Display batch] outer=%" PRIu32 " records=%" PRIu32
	       " avg=%" PRIu32 " fallback=%" PRIu32
	       " cycles=%" PRIu32 "\n",
	       display->tile_batch_commands, display->tile_batch_records,
	       display->tile_batch_commands
		? display->tile_batch_records / display->tile_batch_commands : 0u,
	       display->tile_batch_fallbacks, display->tile_batch_cycles);
#if CONFIG_RV32_NATIVE_JIT
	rv32_native_jit_get_stats(jit);
	{
		uint64_t lookups = jit->lookups - host_perf_last_jit.lookups;
		uint64_t calls = jit->calls - host_perf_last_jit.calls;
		uint64_t native_guest = jit->guest_instructions -
			host_perf_last_jit.guest_instructions;
		uint64_t memory_calls = jit->memory_calls -
			host_perf_last_jit.memory_calls;
		uint64_t native_links = jit->native_link_transitions -
			host_perf_last_jit.native_link_transitions;
		uint32_t call_permille = lookups ?
			(uint32_t)(calls * 1000u / lookups) : 0u;
		uint32_t coverage_permille = guest_delta ?
			(uint32_t)(native_guest * 1000u / guest_delta) : 0u;

		printf("[JIT perf] lookup=%" PRIu64 " call=%" PRIu64
		       " (%" PRIu32 ".%" PRIu32 "%%), guest=%" PRIu64
		       " (%" PRIu32 ".%" PRIu32 "%%) avg=%" PRIu64
		       " mem=%" PRIu64 " bailout=%" PRIu32
		       " compiled=%" PRIu32 " rejected=%" PRIu32
		       " replaced=%" PRIu32 "\n",
		       lookups, calls, call_permille / 10u, call_permille % 10u,
		       native_guest, coverage_permille / 10u,
		       coverage_permille % 10u,
		       calls ? native_guest / calls : 0u, memory_calls,
		       jit->memory_bailouts - host_perf_last_jit.memory_bailouts,
		       jit->compiled_blocks - host_perf_last_jit.compiled_blocks,
		       jit->rejected_blocks - host_perf_last_jit.rejected_blocks,
		       jit->replacements - host_perf_last_jit.replacements);
		printf("[JIT boundaries] window=%" PRIu64 " us cpu=%u MHz; "
		       "C-try=%" PRIu64 " C-enter=%" PRIu64
		       " blocks=%" PRIu64 " links=%" PRIu64
		       "; inclusive-cycles=%" PRIu64 " native-cycles=%" PRIu64
		       " compile-cycles=%" PRIu64 " resolve-cycles=%" PRIu64
		       " probes=%" PRIu64 "/%" PRIu64 " cycles\n",
		       elapsed_us, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
		       jit->dispatch_attempts - host_perf_last_jit.dispatch_attempts,
		       jit->native_entries - host_perf_last_jit.native_entries,
		       calls, native_links,
		       jit->dispatch_cycles - host_perf_last_jit.dispatch_cycles,
		       jit->native_cycles - host_perf_last_jit.native_cycles,
		       jit->compile_cycles - host_perf_last_jit.compile_cycles,
		       jit->resolve_cycles - host_perf_last_jit.resolve_cycles,
		       jit->probe_count - host_perf_last_jit.probe_count,
		       jit->probe_cycles - host_perf_last_jit.probe_cycles);
		{
			static const char *const classes[RV32_NATIVE_JIT_UNSUPPORTED_CLASSES] = {
				"ecall", "xret", "wfi", "sfence", "fencei", "cbo",
				"time", "satp", "csr", "lrsc", "amo", "other",
			};

			printf("[JIT unsupported] probes:");
			for (uint32_t c = 0; c < RV32_NATIVE_JIT_UNSUPPORTED_CLASSES; c++)
				printf(" %s=%" PRIu64, classes[c],
				       jit->unsupported_probes[c] -
					       host_perf_last_jit.unsupported_probes[c]);
			printf(" | cuts:");
			for (uint32_t c = 0; c < RV32_NATIVE_JIT_UNSUPPORTED_CLASSES; c++)
				printf(" %s=%" PRIu64, classes[c],
				       jit->unsupported_cuts[c] -
					       host_perf_last_jit.unsupported_cuts[c]);
			printf("\n");
		}
		printf("[JIT RAM refill] attempts=%" PRIu64 " resumed=%" PRIu64
		       " cycles=%" PRIu64 "\n",
		       jit->ram_refill_attempts - host_perf_last_jit.ram_refill_attempts,
		       jit->ram_refill_resumes - host_perf_last_jit.ram_refill_resumes,
		       jit->ram_refill_cycles - host_perf_last_jit.ram_refill_cycles);
		printf("[JIT compile] guest=%" PRIu64 " native=%" PRIu64
		       " words; max=%" PRIu32 "/%" PRIu32
		       "; reject collect=%" PRIu32 " size=%" PRIu32
		       " codegen=%" PRIu32 " publish=%" PRIu32 "\n",
		       jit->compiled_guest_instructions -
			       host_perf_last_jit.compiled_guest_instructions,
		       jit->compiled_native_words -
			       host_perf_last_jit.compiled_native_words,
		       jit->max_compiled_guest_instructions,
		       jit->max_compiled_native_words,
		       jit->rejected_collect - host_perf_last_jit.rejected_collect,
		       jit->rejected_code_size -
			       host_perf_last_jit.rejected_code_size,
		       jit->rejected_codegen -
			       host_perf_last_jit.rejected_codegen,
			       jit->rejected_publish -
				       host_perf_last_jit.rejected_publish);
		printf("[JIT async] submitted=%" PRIu32 " installed=%" PRIu32
		       " stale=%" PRIu32 " duplicate=%" PRIu32
		       " set-full=%" PRIu32 " queue-full=%" PRIu32
		       " local=%" PRIu32 " worker=%" PRIu64 " cycles\n",
		       jit->async_submitted - host_perf_last_jit.async_submitted,
		       jit->async_installed - host_perf_last_jit.async_installed,
		       jit->async_stale - host_perf_last_jit.async_stale,
		       jit->async_duplicates -
			       host_perf_last_jit.async_duplicates,
		       jit->async_set_full - host_perf_last_jit.async_set_full,
		       jit->async_queue_full -
			       host_perf_last_jit.async_queue_full,
		       jit->async_local - host_perf_last_jit.async_local,
		       jit->async_worker_cycles -
			       host_perf_last_jit.async_worker_cycles);
		printf("[JIT install] installs=%" PRIu32 " cycles=%" PRIu64
		       " forgets=%" PRIu32 " cycles=%" PRIu64
		       " publish=%" PRIu64 " cycles\n",
		       jit->installs - host_perf_last_jit.installs,
		       jit->install_cycles - host_perf_last_jit.install_cycles,
		       jit->forgets - host_perf_last_jit.forgets,
		       jit->forget_cycles - host_perf_last_jit.forget_cycles,
		       jit->publish_cycles - host_perf_last_jit.publish_cycles);
		{
			static uint32_t last_fast_dispatches;
			uint32_t fast_dispatches =
				rv32_native_jit_fast_dispatch_count();

			printf("[JIT trace] joined=%" PRIu64 " jumps in %" PRIu32
			       " regions; max=%" PRIu32 "; cross-page=%" PRIu32
			       " regions/%" PRIu64 " chains; native-link=%" PRIu64
			       " fast-dispatch=%" PRIu32 "\n",
			       jit->traced_jumps - host_perf_last_jit.traced_jumps,
			       jit->traced_regions - host_perf_last_jit.traced_regions,
			       jit->max_traced_jumps,
			       jit->cross_page_regions -
				       host_perf_last_jit.cross_page_regions,
			       jit->cross_page_chains -
				       host_perf_last_jit.cross_page_chains,
			       native_links,
			       fast_dispatches - last_fast_dispatches);
			last_fast_dispatches = fast_dispatches;
		}
		printf("[JIT cache] resident=%" PRIu32 "/%" PRIu32
		       " KiB head=%" PRIu32 "/%" PRIu32
		       " KiB; evict arena=%" PRIu32 " tag=%" PRIu32
		       " wrap=%" PRIu32 " spared=%" PRIu32
		       "; flushes tlb=%" PRIu32
		       " fence=%" PRIu32 "\n",
		       jit->resident_regions, jit->resident_bytes / 1024u,
		       jit->arena_head_bytes / 1024u,
		       jit->arena_capacity_bytes / 1024u,
		       jit->arena_evictions - host_perf_last_jit.arena_evictions,
		       jit->tag_evictions - host_perf_last_jit.tag_evictions,
		       jit->arena_wraps - host_perf_last_jit.arena_wraps,
		       jit->arena_spares - host_perf_last_jit.arena_spares,
		       jit->translation_flushes -
			       host_perf_last_jit.translation_flushes,
		       jit->fences - host_perf_last_jit.fences);
		printf("[JIT hot] resident=%" PRIu32 "/%" PRIu32
		       " KiB head=%" PRIu32 "/%" PRIu32
		       " KiB; promote=%" PRIu32 " (early=%" PRIu32
		       " pressure=%" PRIu32 ") defer=%" PRIu32
		       " evict=%" PRIu32 " wrap=%" PRIu32 " spared=%" PRIu32
		       " hand=%" PRIu32 " bar=%" PRIu32 "\n",
		       jit->hot_resident_regions,
		       jit->hot_resident_bytes / 1024u,
		       jit->hot_arena_head_bytes / 1024u,
		       jit->hot_arena_capacity_bytes / 1024u,
		       jit->hot_promotions - host_perf_last_jit.hot_promotions,
		       jit->hot_early_promotions -
			       host_perf_last_jit.hot_early_promotions,
		       jit->hot_pressure_promotions -
			       host_perf_last_jit.hot_pressure_promotions,
		       jit->hot_admission_deferrals -
			       host_perf_last_jit.hot_admission_deferrals,
		       jit->hot_evictions - host_perf_last_jit.hot_evictions,
		       jit->hot_arena_wraps -
			       host_perf_last_jit.hot_arena_wraps,
		       jit->hot_spares - host_perf_last_jit.hot_spares,
		       jit->hot_hand_promotions -
			       host_perf_last_jit.hot_hand_promotions,
		       jit->hot_promote_bar);
		{
			uint32_t accounted_permille = native_guest ?
				(uint32_t)((uint64_t)
					jit->profile_interval_guest_instructions * 1000u /
					native_guest) : 0u;

			printf("[JIT chain entries] accounted=%" PRIu32
			       " (%" PRIu32 ".%" PRIu32 "%%), discarded=%" PRIu32
			       " guest instructions\n",
			       jit->profile_interval_guest_instructions,
			       accounted_permille / 10u,
			       accounted_permille % 10u,
			       jit->profile_discarded_guest_instructions);
			for (uint32_t rank = 0;
			     rank < RV32_NATIVE_JIT_PROFILE_TOP_REGIONS; rank++) {
				const struct rv32_native_jit_region_stats *region =
					&jit->profile_top_regions[rank];

				if (region->guest_instructions == 0u)
					break;
				printf("  #%-2" PRIu32 " pc=%08" PRIx32
				       " phys=%08" PRIx32 " chain-guest=%" PRIu32
				       " region=%u mem=%u code=%" PRIu32
				       "B loop=%u hot=%u\n",
				       rank + 1u, region->guest_pc,
				       region->physical_offset,
				       region->guest_instructions,
				       region->region_instructions,
				       region->memory_sites, region->code_bytes,
				       region->loops_to_start,
				       region->internal_hot);
			}
		}
	}
	host_perf_last_jit = *jit;
#endif
	host_perf_last_time = current_time;
	host_perf_last_cycle = guest_cycle;
	HostPerfCpuCountersRead(host_perf_last_cpu);
	for (BaseType_t cpu = 0; cpu < 2; cpu++)
		host_perf_last_idle[cpu] = idle[cpu];
}
#endif

void app_main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
#ifdef ESP_PLATFORM
	if (!CONFIG_RV32_DIAGNOSTIC_LOGS)
		esp_log_level_set("*", ESP_LOG_WARN);
	/* The ROM banner precedes application UART setup and can be unreadable
	 * at our console baud. Report the SDK's saved reset classification plus
	 * raw core reasons once, without changing watchdog/reset behavior. */
	/* Always visible: an unexplained restart is otherwise only classified in
	 * the ROM banner, which prints at the ROM baud rate. */
	printf("Reset reason: idf=%u raw_cpu0=0x%x",
	       (unsigned int)esp_reset_reason(),
	       (unsigned int)esp_rom_get_reset_reason(0));
#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
	printf(" raw_cpu1=0x%x", (unsigned int)esp_rom_get_reset_reason(1));
#endif
	printf("\n");
#endif
#ifdef ESP_PLATFORM
	/* First, as its constructor used to: ESP-Hosted's SDIO transport allocates
	 * the SDMMC interrupt (shared with the microSD) on the CPU that starts it. */
	if (HostRunOnCpu1(esp_hosted_init) != ESP_OK)
		printf("WARNING: ESP-Hosted initialization failed\n");
#endif
	RV32_DIAG("psram init\n");

	if (psram_init() < 0) {
		printf("failed to init psram\n");
		return;
	}
	guest_ram = psram_get_base();
	if (guest_ram == NULL) {
		printf("failed to map guest RAM\n");
		return;
	}
#if CONFIG_RV32_GUEST_PROFILE
	/* Before the JIT self-test: generated code embeds the counter addresses. */
	if (rv32_profile_init(guest_ram, ram_amt) < 0)
		printf("WARNING: guest profile tables unavailable\n");
#endif
	/* Reserve guest RAM first, then allocate the LCD driver's framebuffer. */
	display_bridge_set_guest_memory(guest_ram, GUEST_PHYS_BASE, ram_amt);
	display_bridge_init();
	linear_mmio_host = display_bridge_framebuffer_host();
	/* The USB OTG interrupt stays off the emulator's CPU. */
	if (HostRunOnCpu1(HostInputInit) < 0)
		return;
	if (virtio_net_bridge_init() < 0)
		printf("WARNING: C6/virtio-net bridge initialization failed\n");
	if (HostConsoleInit() < 0)
		printf("WARNING: Asynchronous UART console unavailable\n");
	if (HostBrownoutInit() < 0)
		printf("WARNING: Brownout ride-through unavailable\n");
	if (xTaskCreatePinnedToCore(emulator_watch_task, "emu_watch",
				    EMULATOR_WATCH_STACK_SIZE,
				    NULL, tskIDLE_PRIORITY + 3u, NULL,
				    CONFIG_FREERTOS_NUMBER_OF_CORES > 1 ? 1 : 0) !=
	    pdPASS)
		printf("WARNING: Emulator stall reporter unavailable\n");
#if CONFIG_RV32_GUEST_PROFILE
	if (rv32_profile_start_reports() < 0)
		printf("WARNING: guest profile reports unavailable\n");
#endif
	if (virtio_console_bridge_init() < 0)
		printf("WARNING: virtio-console bridge initialization failed\n");
	/* On CPU1: the first sdmmc_host_init() places the SDMMC interrupt, which
	 * the microSD and the C6 Wi-Fi SDIO link share. */
	if (HostRunOnCpu1(virtio_blk_bridge_init) < 0)
		printf("WARNING: microSD/virtio-blk bridge unavailable\n");
	if (virtio_flash_bridge_init() < 0)
		printf("WARNING: read-only flash disk unavailable\n");
	if (virtio_gpu_bridge_init() < 0)
		printf("WARNING: virtio-gpu bridge unavailable\n");
	if (virtio_input_bridge_init() < 0)
		printf("WARNING: virtio-input bridge unavailable\n");
#if CONFIG_RV32_NATIVE_JIT
	if (!rv32_native_jit_init()) {
		printf("ERROR: Native RV32 JIT initialization/self-test failed (group %" PRIu32 ")\n",
		       rv32_native_jit_self_test_failure);
#ifdef ESP_PLATFORM
		/* Stay flashable: a halted emulator cannot run rv32-reboot. */
		printf("Entering ROM download mode for recovery\n");
		GuestUartFlush();
		HostRestart(true);
#endif
		return;
	}
	{
		/* ~1 KiB: keep it off the 3.5 KiB main-task stack. */
		static struct rv32_native_jit_stats stats;

		rv32_native_jit_get_stats(&stats);
		RV32_DIAG("Native RV32 JIT ready: self-tested, %u four-way region tags in a %" PRIu32 " KiB adaptive executable L2 arena + %" PRIu32 " KiB internal hot tier; ready-region chaining, one shared ABI frame\n",
		       RV32_NATIVE_JIT_CACHE_SLOTS,
		       stats.arena_capacity_bytes / 1024u,
		       stats.hot_arena_capacity_bytes / 1024u);
	}
#endif

	RV32_DIAG("\nLoading kernel from flash...\n");

#ifndef ESP_PLATFORM
	restart:
#endif
	guest_reset_mode = RV32_RESET_NORMAL;

	if (load_images(ram_amt, NULL) < 0)
		return;
#if CONFIG_RV32_NATIVE_JIT
	/* A guest reboot reloads the image at the same physical addresses. */
	rv32_native_jit_flush();
#endif
	HostEnableGuestRamPrefetch(guest_ram + KERNEL_LOAD_OFFSET,
				   ram_amt - KERNEL_LOAD_OFFSET);

  memset(&core, 0, sizeof(core));

	core.pc = MINIRV32_RAM_IMAGE_OFFSET + KERNEL_LOAD_OFFSET;
	core.regs[10] = 0; /* a0 = hart ID. */
	core.regs[11] = MINIRV32_RAM_IMAGE_OFFSET + DTB_LOAD_OFFSET;
	/* Supervisor mode; every reservation bit set means no LR reservation. */
	core.extraflags = 1u | ~7u;
	core.satp = 0;       /* Linux enables Sv32 itself. */
#if CONFIG_RV32_NATIVE_JIT
	rv32_native_jit_bind(core.regs, guest_ram, &core,
			     RV32JitMemoryLoad, RV32JitMemoryStore);
#if CONFIG_RV32_GUEST_HLE
	rv32_native_jit_set_hle(RV32HleRun, ram_amt);
#endif
#if CONFIG_RV32_NATIVE_JIT_RAM_REFILL
	rv32_native_jit_set_ram_refill(RV32JitRefillRam);
#endif
	{
#define RV32_CSR_OFFSET(field) \
	((int32_t)offsetof(struct MiniRV32IMAState, field) - \
	 (int32_t)offsetof(struct MiniRV32IMAState, regs))
		const struct rv32_native_jit_csr_layout csr_layout = {
			.sstatus = RV32_CSR_OFFSET(mstatus),
			.sie = RV32_CSR_OFFSET(sie),
			.stvec = RV32_CSR_OFFSET(stvec),
			.sscratch = RV32_CSR_OFFSET(sscratch),
			.sepc = RV32_CSR_OFFSET(sepc),
			.scause = RV32_CSR_OFFSET(scause),
			.stval = RV32_CSR_OFFSET(stval),
			.sip = RV32_CSR_OFFSET(sip),
			.satp = RV32_CSR_OFFSET(satp),
			.extraflags = RV32_CSR_OFFSET(extraflags),
		};
#undef RV32_CSR_OFFSET
		rv32_native_jit_set_csr_layout(&csr_layout);
	}
#if CONFIG_RV32_NATIVE_JIT_SUPERVISOR
	rv32_native_jit_set_kernel_linear_map(MINIRV32_KERNEL_LINEAR_BASE,
		MINI_RV32_RAM_SIZE - MINIRV32_KERNEL_LINEAR_PHYS_OFFSET);
#if CONFIG_RV32_NATIVE_JIT_KERNEL_LINEAR
	/* After the CSR layout: the JIT reads satp from it. */
	rv32_native_jit_set_kernel_linear_data(MINIRV32_KERNEL_LINEAR_PHYS_OFFSET);
#endif
#endif
	rv32_native_jit_start_async_compiler();
#endif

	uint64_t lastTime = GetTimeMicroseconds();
#if CONFIG_RV32_JIT_EXIT_TRACE
	rv32_jit_trace_start(JitTraceAddressSpace);
	jit_trace_last_time = lastTime;
#endif
#if CONFIG_RV32_DEBUG_USB
	uint64_t last_usb_status = lastTime;
#endif
	int instrs_per_flip = EMULATOR_BATCH_INSTRUCTIONS;
#if CONFIG_RV32_EMULATOR_PERF_STATS
	EmulatorPerfStart(lastTime);
#endif
#if CONFIG_RV32_HOST_PERF_STATS
	HostPerfStart(lastTime);
#endif
#if CONFIG_RV32_CACHE_PERF_STATS
	uint64_t last_cache_profile = lastTime;
	HostCacheProfileStart();
#endif
#if CONFIG_RV32_HOST_PC_SAMPLER
	HostPcSamplerStart();
#endif
#if CONFIG_RV32_HOST_MICROBENCH
	HostMicrobench();
#endif
	/* Outrank every unpinned host task (TCP/IP, the timer service), so none
	 * of them is ever scheduled on the emulator core; the system tasks
	 * pinned to CPU0 (IPC, the default event loop) still outrank this. */
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 6u);
	printf("RV32IMA starting\n");
	RV32_DIAG("Initial PC: 0x%08" PRIx32 "\n", core.pc);

	while (1) {
		int ret;
		uint64_t currentTime = GetTimeMicroseconds();
		uint32_t elapsedUs = (uint32_t)(currentTime - lastTime);

		if (elapsedUs > 100000) elapsedUs = 100000;
		lastTime = currentTime;
		emulator_loops = emulator_loops + 1u;
		rv32_profile_service(core.cyclel);
		rv32_host_wake = 0u;
		GuestUartFlushIfDue(currentTime);
		virtio_console_bridge_poll();
		UpdatePlatformInterrupts(&core);
#if CONFIG_RV32_DEBUG_USB
		if (currentTime - last_usb_status >= USB_STATUS_INTERVAL_US) {
			dwc2_passthrough_service();
			last_usb_status = currentTime;
		}
#endif

		if (!display_bridge_guest_slice_begin()) {
#if CONFIG_RV32_NATIVE_JIT && CONFIG_RV32_DISPLAY_ACCEL
			/* The next pixel access reacquires the surface through the
			 * checked helper before a native mapping returns. */
			rv32_native_jit_memory_cache_flush_host_range(
				linear_mmio_host, DISPLAY_FB_SIZE);
#endif
		}
		ret = MiniRV32IMAStep(&core, guest_ram, 0, elapsedUs,
				       instrs_per_flip);
		/* A fenced writer publishes explicitly. Legacy mmap writers publish
		 * after their configured quiet period, so timer-interrupt slices inside
		 * a scanline copy cannot expose a partially completed frame. */
		display_bridge_guest_slice_end();
#if CONFIG_RV32_JIT_EXIT_TRACE
		JitTraceReport(GetTimeMicroseconds());
#endif
#if CONFIG_RV32_EMULATOR_PERF_STATS
		EmulatorPerfReport(currentTime);
#endif
#if CONFIG_RV32_HOST_PERF_STATS
		HostPerfReport(currentTime);
#endif
#if CONFIG_RV32_CACHE_PERF_STATS
		/* Check only at the existing guest-batch boundary. The sampler's
		 * cached code is entered once per second, never per instruction. */
		if (currentTime - last_cache_profile >= 1000000u) {
			last_cache_profile = currentTime;
			HostCacheProfileReport();
		}
#endif
		switch (ret) {
			case 0:
				break;
			case 1:
				MiniSleep();
				break;
			case 3:
				break;
			case 0x7777:
				GuestUartFlush();
#ifdef ESP_PLATFORM
				HostRestart(guest_reset_mode == RV32_RESET_DOWNLOAD);
#else
				goto restart;
#endif
				break;
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
	/* WFI must not hammer the outer device-poll loop.  This sub-tick pause is
	 * a ROM busy wait (not a 100 Hz FreeRTOS delay), so host interrupts remain
	 * responsive while idle cache traffic stays bounded. */
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
static uint32_t pv_console_pointer;

#define GUEST_UART_TX_BUFFER_SIZE 256u
#define GUEST_UART_TX_FLUSH_US    1000u

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

static void GuestParavirtualConsoleWrite(uint32_t virtual_address,
					 size_t length)
{
	if (length == 0 || length > RV32_PV_CONSOLE_MAX_LENGTH)
		return;

	while (length) {
		int fault = 0;
		int uncached = 0;
		uint32_t physical_address = MiniRV32Translate(
			&core, guest_ram, virtual_address, ACCESS_READ,
			&fault, &uncached);
		size_t chunk = 0x1000u - (virtual_address & 0xfffu);

		(void)uncached;
		if (fault || physical_address < MINIRV32_RAM_IMAGE_OFFSET ||
		    physical_address - MINIRV32_RAM_IMAGE_OFFSET >= ram_amt)
			return;
		if (chunk > length)
			chunk = length;
		if (chunk > ram_amt -
			    (physical_address - MINIRV32_RAM_IMAGE_OFFSET))
			return;

		HostConsoleWrite(
			guest_ram + physical_address - MINIRV32_RAM_IMAGE_OFFSET,
			chunk);
		virtual_address += (uint32_t)chunk;
		length -= chunk;
	}
}

#define UART_BASE              0x10000000u
#define UART_IER_RDI           (1u << 0)
#define UART_IER_THRI          (1u << 1)
#define PLIC_BASE              0x0c000000u
#define PLIC_SIZE              0x00400000u
#define PLIC_UART_SOURCE       1u
#define PLIC_USB_SOURCE        2u
#define PLIC_NET_SOURCE        VIRTIO_NET_PLIC_SOURCE
#define PLIC_CONSOLE_SOURCE    VIRTIO_CONSOLE_PLIC_SOURCE
#define PLIC_BLOCK_SOURCE      VIRTIO_BLK_PLIC_SOURCE
#define PLIC_GPU_SOURCE        VIRTIO_GPU_PLIC_SOURCE
#define PLIC_FLASH_SOURCE      VIRTIO_FLASH_PLIC_SOURCE
#define PLIC_INPUT_SOURCE      VIRTIO_INPUT_PLIC_SOURCE
#define PLIC_SOURCE_COUNT      8u
#define PLIC_PENDING_BASE      0x00001000u
#define PLIC_ENABLE_BASE       0x00002000u
#define PLIC_CONTEXT_BASE      0x00200000u
#define PLIC_CONTEXT_THRESHOLD 0x00000000u
#define PLIC_CONTEXT_CLAIM     0x00000004u
#define SIP_SEIP               (1u << 9)

static uint32_t plic_priority[PLIC_SOURCE_COUNT + 1u] =
	{ 0, 1, 1, 1, 1, 1, 1, 1, 1 };
static uint32_t plic_enable;
/* Diagnostics for the stall reporter: claims and completions per source,
 * claims that found nothing, and the last source handed out. */
static volatile uint32_t plic_claims[PLIC_SOURCE_COUNT + 1u];
static volatile uint32_t plic_completes[PLIC_SOURCE_COUNT + 1u];
static volatile uint32_t plic_last_claimed;
static uint32_t plic_threshold;

/* Once the virtio console's input queue is live it owns host UART RX, so
 * ttyS0 must neither report nor consume those bytes.  Transmit stays with
 * ttyS0: /dev/console writes drain only through THRE interrupts. */
static inline bool uart_rx_ready(void)
{
	return !virtio_console_bridge_owns_uart_rx() && IsKBHit();
}

static bool uart_interrupt_pending(void)
{
	if ((uart_ier & UART_IER_RDI) && uart_rx_ready())
		return true;

	return (uart_ier & UART_IER_THRI) && uart_thre_irq_pending;
}

/* Bit n set while source n is pending.  The device getters are inline reads
 * of each bridge's interrupt status; the UART check, which may consult the
 * host UART driver, comes last. */
static inline uint32_t plic_pending_sources(void)
{
	uint32_t pending = 0;

	if (dwc2_passthrough_irq_pending())
		pending |= 1u << PLIC_USB_SOURCE;
	if (virtio_net_bridge_irq_pending())
		pending |= 1u << PLIC_NET_SOURCE;
	if (virtio_console_bridge_irq_pending())
		pending |= 1u << PLIC_CONSOLE_SOURCE;
	if (virtio_blk_bridge_irq_pending())
		pending |= 1u << PLIC_BLOCK_SOURCE;
	if (virtio_gpu_bridge_irq_pending())
		pending |= 1u << PLIC_GPU_SOURCE;
	if (virtio_flash_bridge_irq_pending())
		pending |= 1u << PLIC_FLASH_SOURCE;
	if (virtio_input_bridge_irq_pending())
		pending |= 1u << PLIC_INPUT_SOURCE;
	if (uart_interrupt_pending())
		pending |= 1u << PLIC_UART_SOURCE;
	return pending;
}

static uint32_t plic_claim(void)
{
	uint32_t candidates = plic_pending_sources() & plic_enable;
	uint32_t best_source = 0;
	uint32_t best_priority = plic_threshold;

	while (candidates != 0u) {
		uint32_t source = (uint32_t)__builtin_ctz(candidates);

		if (plic_priority[source] > best_priority) {
			best_source = source;
			best_priority = plic_priority[source];
		}
		candidates &= candidates - 1u;
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

	if (offset == PLIC_PENDING_BASE)
		return plic_pending_sources();

	if (offset == PLIC_ENABLE_BASE)
		return plic_enable;

	if (offset == PLIC_CONTEXT_BASE + PLIC_CONTEXT_THRESHOLD)
		return plic_threshold;

	if (offset == PLIC_CONTEXT_BASE + PLIC_CONTEXT_CLAIM) {
		/* Every emulated source is level-triggered. */
		uint32_t source = plic_claim();

		plic_claims[source] = plic_claims[source] + 1u;
		plic_last_claimed = source;
		return source;
	}

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
	} else if (offset == PLIC_CONTEXT_BASE + PLIC_CONTEXT_CLAIM) {
		if (val <= PLIC_SOURCE_COUNT)
			plic_completes[val] = plic_completes[val] + 1u;
		if (val == PLIC_USB_SOURCE)
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

/* Same test as every bridge's contains(): the whole access lies inside. */
static inline bool MmioRangeContains(uint32_t addy, size_t width,
				     uint32_t base, uint32_t size)
{
	return width != 0 && width <= sizeof(uint32_t) && addy >= base &&
	       addy - base <= size - width;
}

static uint32_t HandleControlStore(
	uint32_t addy, uint32_t val, size_t width)
{
	if (addy == RV32_RESET_BASE + RV32_RESET_MODE_OFFSET &&
	    width == sizeof(uint32_t)) {
		/* Invalid requests disarm a previously selected download boot. */
		guest_reset_mode = val == RV32_RESET_DOWNLOAD ? val : RV32_RESET_NORMAL;
		return 0;
	}
	if (addy == RV32_PV_CONSOLE_BASE + RV32_PV_CONSOLE_POINTER_OFFSET &&
	    width == sizeof(uint32_t)) {
		pv_console_pointer = val;
		return 0;
	}
	if (addy == RV32_PV_CONSOLE_BASE + RV32_PV_CONSOLE_LENGTH_OFFSET &&
	    width == sizeof(uint32_t)) {
		GuestParavirtualConsoleWrite(pv_console_pointer, val);
		return 0;
	}

	/* Interrupt handlers touch the USB controller and the PLIC far more often
	 * than anything else; the virtio range tests are inline. */
	if (MmioRangeContains(addy, width, DWC2_GUEST_BASE, DWC2_GUEST_SIZE)) {
		dwc2_passthrough_store(addy, val, width);
		return 0;
	}
	if (addy >= PLIC_BASE && addy < PLIC_BASE + PLIC_SIZE) {
		PlicStore(addy, val);
		return 0;
	}
	if (MmioRangeContains(addy, width, VIRTIO_NET_GUEST_BASE,
			      VIRTIO_NET_GUEST_SIZE)) {
		virtio_net_bridge_store(addy, val, width);
		return 0;
	}
	if (MmioRangeContains(addy, width, VIRTIO_CONSOLE_GUEST_BASE,
			      VIRTIO_CONSOLE_GUEST_SIZE)) {
		virtio_console_bridge_store(addy, val, width);
		return 0;
	}
	if (MmioRangeContains(addy, width, VIRTIO_BLK_GUEST_BASE,
			      VIRTIO_BLK_GUEST_SIZE)) {
		virtio_blk_bridge_store(addy, val, width);
		return 0;
	}
	if (MmioRangeContains(addy, width, VIRTIO_FLASH_GUEST_BASE,
			      VIRTIO_FLASH_GUEST_SIZE)) {
		virtio_flash_bridge_store(addy, val, width);
		return 0;
	}
	if (MmioRangeContains(addy, width, VIRTIO_GPU_GUEST_BASE,
			      VIRTIO_GPU_GUEST_SIZE)) {
		virtio_gpu_bridge_store(addy, val, width);
		return 0;
	}
	if (MmioRangeContains(addy, width, VIRTIO_INPUT_GUEST_BASE,
			      VIRTIO_INPUT_GUEST_SIZE)) {
		virtio_input_bridge_store(addy, val, width);
		return 0;
	}
	if (display_bridge_contains(addy, width)) {
		display_bridge_store(addy, val, width);
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
	if (width == sizeof(uint32_t)) {
		if (addy == RV32_RESET_BASE + RV32_RESET_MAGIC_OFFSET)
			return RV32_RESET_MAGIC;
		if (addy == RV32_RESET_BASE + RV32_RESET_MODE_OFFSET)
			return guest_reset_mode;
	}
	if (width == sizeof(uint32_t) &&
	    addy == RV32_PV_CONSOLE_BASE + RV32_PV_CONSOLE_MAGIC_OFFSET)
		return RV32_PV_CONSOLE_MAGIC;
	if (width == sizeof(uint32_t) &&
	    addy == RV32_PV_CONSOLE_BASE + RV32_PV_CONSOLE_MAX_LENGTH_OFFSET)
		return RV32_PV_CONSOLE_MAX_LENGTH;

	if (MmioRangeContains(addy, width, DWC2_GUEST_BASE, DWC2_GUEST_SIZE))
		return dwc2_passthrough_load(addy, width);
	if (addy >= PLIC_BASE && addy < PLIC_BASE + PLIC_SIZE)
		return PlicLoad(addy);
	if (MmioRangeContains(addy, width, VIRTIO_NET_GUEST_BASE,
			      VIRTIO_NET_GUEST_SIZE))
		return virtio_net_bridge_load(addy, width);
	if (MmioRangeContains(addy, width, VIRTIO_CONSOLE_GUEST_BASE,
			      VIRTIO_CONSOLE_GUEST_SIZE))
		return virtio_console_bridge_load(addy, width);
	if (MmioRangeContains(addy, width, VIRTIO_BLK_GUEST_BASE,
			      VIRTIO_BLK_GUEST_SIZE))
		return virtio_blk_bridge_load(addy, width);
	if (MmioRangeContains(addy, width, VIRTIO_FLASH_GUEST_BASE,
			      VIRTIO_FLASH_GUEST_SIZE))
		return virtio_flash_bridge_load(addy, width);
	if (MmioRangeContains(addy, width, VIRTIO_GPU_GUEST_BASE,
			      VIRTIO_GPU_GUEST_SIZE))
		return virtio_gpu_bridge_load(addy, width);
	if (MmioRangeContains(addy, width, VIRTIO_INPUT_GUEST_BASE,
			      VIRTIO_INPUT_GUEST_SIZE))
		return virtio_input_bridge_load(addy, width);
	if (display_bridge_contains(addy, width))
		return display_bridge_load(addy, width);

    switch (addy) {
        case 0x10000000:
			if (uart_lcr & 0x80)
				return uart_divisor & 0xff;
			return uart_rx_ready() ? ReadKBByte() : 0;
        case 0x10000001:
            if (uart_lcr & 0x80) return (uart_divisor >> 8) & 0xFF;
            return uart_ier;
		case 0x10000002:
			if ((uart_ier & UART_IER_RDI) && uart_rx_ready())
				return 0xC4; /* FIFO enabled, received data available. */
			if ((uart_ier & UART_IER_THRI) && uart_thre_irq_pending) {
				uart_thre_irq_pending = false;
				return 0xC2; /* FIFO enabled, THR empty. */
			}
			return 0xC1; /* FIFO enabled, no interrupt pending. */
        case 0x10000003: return uart_lcr;
        case 0x10000004: return uart_mcr;
        case 0x10000005:
          return 0x60 | (uart_rx_ready() ? 1 : 0);
        case 0x10000006: return 0x00;
        case 0x10000007: return uart_scratch;
    }
    return 0;
}
static void HandleOtherCSRWrite(uint8_t *image, uint16_t csrno, uint32_t value)
{
	uint32_t ptrstart, ptrend;

	#if CONFIG_RV32_DEBUG_CSR
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
				uint8_t c = image[ptrend];
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

	#if CONFIG_RV32_DEBUG_CSR
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
