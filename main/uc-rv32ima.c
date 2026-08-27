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
#endif

#if CONFIG_RV32_HOST_PERF_STATS
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#endif

#include "port.h"
#include "cache.h"
#include "display_bridge.h"
#include "dwc2_passthrough.h"
#include "psram.h"
#include "pv_console_protocol.h"
#include "virtio_console_bridge.h"
#include "virtio_net_bridge.h"

static uint32_t ram_amt = GUEST_RAM_SIZE;
static uint8_t *guest_ram;

#define EMULATOR_BATCH_INSTRUCTIONS 32768
#define USB_STATUS_INTERVAL_US 500000u
#define EMULATOR_DEBUG_CSR 0

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

#define MINIRV32WARN(x...) printf(x);
#define MINI_RV32_RAM_SIZE ram_amt
#define MINIRV32_IMPLEMENTATION
/* The bundled kernel and supported replacement images use aligned, 32-bit
 * RV32IMA instructions.  Skip redundant fetch checks in the hot loop; ports
 * that include mini-rv32ima.h without this define retain strict validation. */
#define MINIRV32_TRUSTED_32BIT_FETCH
#if CONFIG_RV32_DECODED_BLOCK_CACHE
#define MINIRV32_DECODED_BLOCK_CACHE
#endif
/* Per-instruction trap tracing is prohibitively expensive on the emulator. */
#define MINIRV32_POSTEXEC(...) do { } while (0)

#define MINIRV32_HANDLE_MEM_STORE_CONTROL(addy, val) \
	do { \
		size_t control_width = 1u << (MINIRV32_DECODE_FUNCT3() & 3u); \
		if (display_bridge_contains((addy), control_width)) { \
			display_bridge_store((addy), (val), control_width); \
		} else if (HandleControlStore((addy), (val), control_width)) { \
			return (val); \
		} \
	} while (0)
#define MINIRV32_HANDLE_MEM_LOAD_CONTROL(addy, rval) \
	do { \
		size_t control_width = 1u << (MINIRV32_DECODE_FUNCT3() & 3u); \
		if (display_bridge_contains((addy), control_width)) \
			rval = display_bridge_load((addy), control_width); \
		else \
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
#define MINIRV32_IS_MMIO_ADDRESS(address) \
	(((address) >= 0x10000000u && (address) < 0x12000200u) || \
	 ((address) >= VIRTIO_CONSOLE_GUEST_BASE && \
	  (address) < VIRTIO_CONSOLE_GUEST_BASE + VIRTIO_CONSOLE_GUEST_SIZE) || \
	 ((address) >= 0x0c000000u && (address) < 0x0c400000u) || \
	 ((address) >= 0x50000000u && (address) < 0x50040000u))
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

struct MiniRV32IMAState core __attribute__((aligned(128)));

#if CONFIG_RV32_EMULATOR_PERF_STATS
static uint64_t emulator_perf_last_time;
static uint64_t emulator_perf_last_cycle;

static void EmulatorPerfStart(uint64_t current_time)
{
	emulator_perf_last_time = current_time;
	emulator_perf_last_cycle =
		((uint64_t)core.cycleh << 32) | core.cyclel;
	memset(&core.perf, 0, sizeof(core.perf));
	HostCacheStatsReset();
}

static void EmulatorPerfReport(uint64_t current_time)
{
	uint64_t elapsed_us = current_time - emulator_perf_last_time;
	uint64_t guest_cycle = ((uint64_t)core.cycleh << 32) | core.cyclel;
	uint64_t guest_delta = guest_cycle - emulator_perf_last_cycle;
	uint64_t mips_x100;
	uint64_t l1_total;
	uint64_t l2_total;
	struct host_cache_stats cache_stats;
	uint32_t l1_hit_permille;
	uint32_t l2_hit_permille;
	uint64_t opcode_total = 0;

	if (elapsed_us < EMULATOR_PERF_INTERVAL_US)
		return;

	HostCacheStatsReadAndReset(&cache_stats);
	mips_x100 = elapsed_us ? guest_delta * 100u / elapsed_us : 0u;
	l1_total = (uint64_t)cache_stats.l1_hits + cache_stats.l1_misses;
	l2_total = (uint64_t)cache_stats.l2_hits + cache_stats.l2_misses;
	l1_hit_permille = l1_total ?
		(uint32_t)((uint64_t)cache_stats.l1_hits * 1000u / l1_total) : 0u;
	l2_hit_permille = l2_total ?
		(uint32_t)((uint64_t)cache_stats.l2_hits * 1000u / l2_total) : 0u;
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
	#undef OPCODE_PERCENT

	emulator_perf_last_time = current_time;
	emulator_perf_last_cycle = guest_cycle;
	memset(&core.perf, 0, sizeof(core.perf));
}
#endif

#if CONFIG_RV32_HOST_PERF_STATS
static uint64_t host_perf_last_time;
static uint64_t host_perf_last_cycle;
static configRUN_TIME_COUNTER_TYPE host_perf_last_idle[2];

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

static void HostPerfStart(uint64_t current_time)
{
	struct display_bridge_perf_stats ignored;

	host_perf_last_time = current_time;
	host_perf_last_cycle =
		((uint64_t)core.cycleh << 32) | core.cyclel;
	for (BaseType_t cpu = 0; cpu < 2; cpu++)
		host_perf_last_idle[cpu] =
			ulTaskGetIdleRunTimeCounterForCore(cpu);
	display_bridge_perf_read_and_reset(&ignored);
}

static void HostPerfReport(uint64_t current_time)
{
	uint64_t elapsed_us = current_time - host_perf_last_time;
	uint64_t guest_cycle;
	uint64_t guest_delta;
	uint64_t mips_x100;
	configRUN_TIME_COUNTER_TYPE idle[2];
	uint32_t busy[2];
	uint32_t display_busy_permille;
	struct display_bridge_perf_stats display;

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
	display_bridge_perf_read_and_reset(&display);
	display_busy_permille = elapsed_us
		? (uint32_t)((uint64_t)display.service_us * 1000u / elapsed_us)
		: 0u;

	printf("\n[Host perf] CPU0 %" PRIu32 ".%" PRIu32
	       "%% busy, CPU1 %" PRIu32 ".%" PRIu32
	       "%% busy; RV32 %" PRIu64 ".%02" PRIu64 " MIPS\n",
	       busy[0] / 10u, busy[0] % 10u,
	       busy[1] / 10u, busy[1] % 10u,
	       mips_x100 / 100u, mips_x100 % 100u);
	printf("[Display perf] worker %" PRIu32 ".%" PRIu32
	       "%%/%" PRIu32 " us, wake=%" PRIu32 " vsync=%" PRIu32
	       " cmd=%" PRIu32 " (fill=%" PRIu32 " copy=%" PRIu32
	       " tile=%" PRIu32 ": set=%" PRIu32 " fill=%" PRIu32
	       " blit=%" PRIu32 " cursor=%" PRIu32 "), FIFO high=%" PRIu32
	       " busy=%" PRIu32 " slices=%" PRIu32
	       " deferred=%" PRIu32 "\n",
	       display_busy_permille / 10u, display_busy_permille % 10u,
	       display.service_us, display.service_wakes, display.vsyncs,
	       display.commands,
	       display.fill_commands, display.copy_commands,
	       display.tile_commands, display.tile_set_commands,
	       display.tile_fill_commands, display.tile_blit_commands,
	       display.tile_cursor_commands, display.fifo_high_water,
	       display.fifo_busy, display.fifo_slices,
	       display.fifo_deferred);
	printf("[Display perf] cache=%" PRIu32 " calls/%" PRIu32
	       " KiB/%" PRIu32 " us; PPA=%" PRIu32
	       " fills + %" PRIu32 " blits/%" PRIu32
	       " us; pixels PPA=%" PRIu32 " CPU-fill=%" PRIu32
	       " copy=%" PRIu32 " tile=%" PRIu32 "\n",
	       display.cache_syncs, display.cache_bytes / 1024u,
	       display.cache_us, display.ppa_fills, display.ppa_blits,
	       display.ppa_us, display.ppa_fill_pixels,
	       display.cpu_fill_pixels, display.copy_pixels, display.tile_pixels);
	#define DISPLAY_AVG_CYCLES(total, count) \
		((count) ? (total) / (count) : 0u)
	printf("[Display cycles] dispatch=%" PRIu32 " total/%" PRIu32
	       " avg/%" PRIu32 " max; avg/op fill=%" PRIu32
	       " copy=%" PRIu32 " image1=%" PRIu32 " tile-set=%" PRIu32
	       " tile-fill=%" PRIu32 " tile-blit=%" PRIu32
	       " cursor=%" PRIu32 "\n",
	       display.command_cycles,
	       DISPLAY_AVG_CYCLES(display.command_cycles, display.commands),
	       display.command_max_cycles,
	       DISPLAY_AVG_CYCLES(display.fill_cycles, display.fill_commands),
	       DISPLAY_AVG_CYCLES(display.copy_cycles, display.copy_commands),
	       DISPLAY_AVG_CYCLES(display.image1_cycles,
				  display.image1_commands),
	       DISPLAY_AVG_CYCLES(display.tile_set_cycles,
				  display.tile_set_commands),
	       DISPLAY_AVG_CYCLES(display.tile_fill_cycles,
				  display.tile_fill_commands),
	       DISPLAY_AVG_CYCLES(display.tile_blit_cycles,
				  display.tile_blit_commands),
	       DISPLAY_AVG_CYCLES(display.tile_cursor_cycles,
				  display.tile_cursor_commands));
	printf("[Display cursor] toggles=%" PRIu32 " cycles=%" PRIu32
	       " total/%" PRIu32 " avg/%" PRIu32 " max\n",
	       display.cursor_toggles, display.cursor_toggle_cycles,
	       DISPLAY_AVG_CYCLES(display.cursor_toggle_cycles,
				  display.cursor_toggles),
	       display.cursor_toggle_max_cycles);
	#undef DISPLAY_AVG_CYCLES
	printf("[Display frame] samples=%" PRIu32 " total=%" PRIu32
	       " us avg=%" PRIu32 " us max=%" PRIu32
	       " us; cache overhead=%" PRIu32 " us\n",
	       display.frame_samples, display.frame_total_us,
	       display.frame_samples
		? display.frame_total_us / display.frame_samples : 0u,
	       display.frame_max_us, display.cache_us);
	printf("[Display producer] submit=%" PRIu32 "/%" PRIu32
	       " us, payload=%" PRIu32 " KiB (inline=%" PRIu32
	       " pool=%" PRIu32 "), shared-result=%" PRIu32
	       "; cross-core wake=%" PRIu32
	       "/%" PRIu32 " us\n",
	       display.producer_submissions, display.producer_us,
	       display.producer_payload_bytes / 1024u,
	       display.producer_inline_payloads,
	       display.producer_external_payloads,
	       display.producer_shared_results,
	       display.producer_wakes, display.producer_wake_us);
	printf("[Display batch] outer=%" PRIu32 " records=%" PRIu32
	       " avg=%" PRIu32 " fallback=%" PRIu32
	       " cycles=%" PRIu32 "\n",
	       display.tile_batch_commands, display.tile_batch_records,
	       display.tile_batch_commands
		? display.tile_batch_records / display.tile_batch_commands : 0u,
	       display.tile_batch_fallbacks, display.tile_batch_cycles);
	host_perf_last_time = current_time;
	host_perf_last_cycle = guest_cycle;
	for (BaseType_t cpu = 0; cpu < 2; cpu++)
		host_perf_last_idle[cpu] = idle[cpu];
}
#endif

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
	display_bridge_set_guest_memory(guest_ram, GUEST_PHYS_BASE, ram_amt);
	display_bridge_init();
	if (HostInputInit() < 0)
		return;
	if (virtio_net_bridge_init() < 0)
		printf("WARNING: C6/virtio-net bridge initialization failed\n");
	if (HostConsoleInit() < 0)
		printf("WARNING: Asynchronous UART console unavailable\n");
	if (virtio_console_bridge_init() < 0)
		printf("WARNING: virtio-console bridge initialization failed\n");

	printf("\nLoading kernel from flash...\n");

	restart:

	if (load_images(ram_amt, NULL) < 0)
		return;
	HostEnableGuestRamPrefetch(guest_ram + KERNEL_LOAD_OFFSET,
				   ram_amt - KERNEL_LOAD_OFFSET);

  memset(&core, 0, sizeof(core));

	core.pc = MINIRV32_RAM_IMAGE_OFFSET + KERNEL_LOAD_OFFSET;
	core.regs[10] = 0; /* a0 = hart ID. */
	core.regs[11] = MINIRV32_RAM_IMAGE_OFFSET + DTB_LOAD_OFFSET;
	core.extraflags = 1; /* Supervisor mode. */
	core.satp = 0;       /* Linux enables Sv32 itself. */

	uint64_t lastTime = GetTimeMicroseconds();
	uint64_t last_usb_status = lastTime;
	int instrs_per_flip = EMULATOR_BATCH_INSTRUCTIONS;
#if CONFIG_RV32_EMULATOR_PERF_STATS
	EmulatorPerfStart(lastTime);
#endif
#if CONFIG_RV32_HOST_PERF_STATS
	HostPerfStart(lastTime);
#endif
	printf("RV32IMA starting\n");
	printf("Initial PC: 0x%08" PRIx32 "\n", core.pc);

	while (1) {
		int ret;
		uint64_t currentTime = GetTimeMicroseconds();
		uint32_t elapsedUs = (uint32_t)(currentTime - lastTime);

		if (elapsedUs > 100000) elapsedUs = 100000;
		lastTime = currentTime;
		GuestUartFlushIfDue(currentTime);
		virtio_console_bridge_poll();
		UpdatePlatformInterrupts(&core);
		if (currentTime - last_usb_status >= USB_STATUS_INTERVAL_US) {
			dwc2_passthrough_service();
			last_usb_status = currentTime;
		}

		ret = MiniRV32IMAStep(&core, guest_ram, 0, elapsedUs,
				       instrs_per_flip);
#if CONFIG_RV32_EMULATOR_PERF_STATS
		EmulatorPerfReport(currentTime);
#endif
#if CONFIG_RV32_HOST_PERF_STATS
		HostPerfReport(currentTime);
#endif
		/* simplefb has no end-of-frame callback.  Explicit full-frame
		 * writers use the SYNC doorbell; this cheap batch-boundary handoff
		 * keeps unmodified fbcon responsive without a wall-clock timer. */
		display_bridge_commit();

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
#define PLIC_SOURCE_COUNT      4u
#define PLIC_PENDING_BASE      0x00001000u
#define PLIC_ENABLE_BASE       0x00002000u
#define PLIC_CONTEXT_BASE      0x00200000u
#define PLIC_CONTEXT_THRESHOLD 0x00000000u
#define PLIC_CONTEXT_CLAIM     0x00000004u
#define SIP_SEIP               (1u << 9)

static uint32_t plic_priority[PLIC_SOURCE_COUNT + 1u] = { 0, 1, 1, 1, 1 };
static uint32_t plic_enable;
static uint32_t plic_threshold;

static bool uart_interrupt_pending(void)
{
	if (virtio_console_bridge_owns_uart_rx())
		return false;
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
	if (source == PLIC_CONSOLE_SOURCE)
		return virtio_console_bridge_irq_pending();
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

	if (virtio_console_bridge_contains(addy, width)) {
		virtio_console_bridge_store(addy, val, width);
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
	if (width == sizeof(uint32_t) &&
	    addy == RV32_PV_CONSOLE_BASE + RV32_PV_CONSOLE_MAGIC_OFFSET)
		return RV32_PV_CONSOLE_MAGIC;
	if (width == sizeof(uint32_t) &&
	    addy == RV32_PV_CONSOLE_BASE + RV32_PV_CONSOLE_MAX_LENGTH_OFFSET)
		return RV32_PV_CONSOLE_MAX_LENGTH;

	if (display_bridge_contains(addy, width))
		return display_bridge_load(addy, width);

	if (dwc2_passthrough_contains(addy, width))
		return dwc2_passthrough_load(addy, width);

	if (virtio_net_bridge_contains(addy, width))
		return virtio_net_bridge_load(addy, width);

	if (virtio_console_bridge_contains(addy, width))
		return virtio_console_bridge_load(addy, width);

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
