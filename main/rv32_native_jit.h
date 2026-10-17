#ifndef RV32_NATIVE_JIT_H
#define RV32_NATIVE_JIT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "rv32_jit_trace.h"

#define RV32_NATIVE_JIT_CACHE_SLOTS 2048u
#define RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES 64u
#define RV32_NATIVE_JIT_PROFILE_TOP_REGIONS 8u

struct rv32_native_jit_memory_site {
	uint32_t guest_pc;
	uint8_t executed_before;
	uint8_t descriptor;
	uint8_t destination;
	uint8_t store;
};

struct rv32_native_jit_result {
	uint32_t next_pc;
	uint32_t executed;
	uint32_t trap;
	uint32_t trap_value;
	/* True only when next_pc is a completed region's control-flow successor.
	 * A cold memory resume is architectural progress, but not a new hot head. */
	uint32_t native_successor;
	/* Directory slot of the last generated region that returned to C.  Native
	 * successor links can traverse several regions without involving the C
	 * dispatcher, so this identifies the final region for checked lookup and
	 * cold-memory recovery. */
	uint32_t native_slot;
	uint32_t linear_dirty_start;
	uint32_t linear_dirty_end;
	uint32_t linear_store_calls;
	uint32_t linear_store_bytes;
};

struct rv32_native_jit_read_page {
	uint32_t virtual_page;
	uint32_t host_bias;
};

struct rv32_native_jit_write_page {
	uint32_t virtual_page;
	uint32_t host_bias;
	uint32_t linear_page_start;
	uint32_t linear_page_end;
	/* A framebuffer page is reported at most once per top-level native
	 * dispatch, even when a generated inner loop stores every pixel. */
	uint32_t dirty_epoch;
	uint32_t reserved[3];
};

struct rv32_native_jit_runtime {
	/* Keep the returned fields first so try() can return them without copying. */
	struct rv32_native_jit_result result;
	uint32_t *regs;
	uint8_t *image;
	void *opaque;
	uint32_t memory_address;
	uint32_t memory_value;
	uint32_t memory_calls;
	uint32_t memory_context;
	uint32_t loop_iterations;
	uint32_t dirty_epoch;
	/* A top-level dispatcher call owns this bounded native-chain budget. */
	uint32_t chain_instruction_limit;
	uint32_t chain_executed;
	uint32_t chain_blocks;
	uint32_t chain_links;
	struct rv32_native_jit_read_page
		read_pages[RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES];
	struct rv32_native_jit_write_page
		write_pages[RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES];
	const struct rv32_native_jit_memory_site *sites;
	uint32_t site_count;
	uint32_t (*load)(struct rv32_native_jit_runtime *runtime,
			 uint32_t virtual_address, uint32_t descriptor);
	uint32_t (*store)(struct rv32_native_jit_runtime *runtime,
			  uint32_t virtual_address, uint32_t value,
			  uint32_t descriptor);
	/* Publishes one RAM page mapping without performing the access.  access
	 * is funct3 in bits 0..2 and the store flag in bit 3. */
	bool (*refill_ram)(struct rv32_native_jit_runtime *runtime,
			   uint32_t virtual_address, uint32_t access);
#if CONFIG_RV32_JIT_EXIT_TRACE
	/* Set only by the checked callback, never by generated code. */
	uint32_t trace_memory_kind;
#endif
};

/* Interval-only chain-entry diagnostic record. guest_instructions includes
 * all work reached from this top-level entry, including native successors;
 * it is not an exclusive per-region instruction count or a time sample.
 * No process identity or known application address is involved. */
struct rv32_native_jit_region_stats {
	uint32_t guest_pc;
	uint32_t physical_offset;
	uint32_t guest_instructions;
	uint32_t code_bytes;
	uint8_t region_instructions;
	uint8_t memory_sites;
	uint8_t loops_to_start;
	uint8_t internal_hot;
};

struct rv32_native_jit_stats {
	/* calls counts generated blocks, including blocks reached by native links.
	 * These separate counters measure the C API and assembly-entry boundaries. */
	uint64_t dispatch_attempts;
	uint64_t native_entries;
	uint64_t dispatch_cycles;
	uint64_t native_cycles;
	/* Cycles inside dispatch spent compiling and resolving memory bailouts. */
	uint64_t compile_cycles;
	uint64_t resolve_cycles;
	/* Dispatch attempts that entered no native code and compiled nothing. */
	uint64_t probe_count;
	uint64_t probe_cycles;
	/* Why native code stopped: probes landing on an instruction the JIT does
	 * not compile, and region collections cut by one, by instruction class. */
#define RV32_NATIVE_JIT_UNSUPPORTED_CLASSES 12u
	uint64_t unsupported_probes[RV32_NATIVE_JIT_UNSUPPORTED_CLASSES];
	uint64_t unsupported_cuts[RV32_NATIVE_JIT_UNSUPPORTED_CLASSES];
	uint64_t lookups;
	uint64_t calls;
	uint64_t guest_instructions;
	uint64_t memory_calls;
	uint64_t ram_refill_attempts;
	uint64_t ram_refill_resumes;
	uint64_t ram_refill_cycles;
	uint64_t compiled_guest_instructions;
	uint64_t compiled_native_words;
	uint64_t traced_jumps;
	uint64_t cross_page_chains;
	uint64_t native_link_transitions;
	uint32_t compiled_blocks;
	uint32_t traced_regions;
	uint32_t cross_page_regions;
	uint32_t rejected_blocks;
	uint32_t replacements;
	uint32_t memory_bailouts;
	uint32_t max_compiled_guest_instructions;
	uint32_t max_compiled_native_words;
	uint32_t max_traced_jumps;
	uint32_t rejected_collect;
	uint32_t rejected_code_size;
	uint32_t rejected_codegen;
	uint32_t rejected_publish;
	uint32_t arena_evictions;
	uint32_t tag_evictions;
	uint32_t arena_wraps;
	uint32_t hot_promotions;
	uint32_t hot_evictions;
	uint32_t hot_arena_wraps;
	uint32_t hot_early_promotions;
	uint32_t hot_pressure_promotions;
	uint32_t hot_admission_deferrals;
	uint32_t resident_regions;
	uint32_t resident_bytes;
	uint32_t arena_head_bytes;
	uint32_t arena_capacity_bytes;
	uint32_t hot_resident_regions;
	uint32_t hot_resident_bytes;
	uint32_t hot_arena_head_bytes;
	uint32_t hot_arena_capacity_bytes;
	uint32_t profile_interval_guest_instructions;
	uint32_t profile_discarded_guest_instructions;
	struct rv32_native_jit_region_stats
		profile_top_regions[RV32_NATIVE_JIT_PROFILE_TOP_REGIONS];
};

/*
 * Cache hot RV32 user and supervisor integer regions as native ESP32-P4 code.
 * A checked callback validates each new data page; a small tagged cache per
 * privilege/SUM bank then permits aligned native accesses without bypassing
 * Sv32, uncached DMA or MMIO rules.  Machine-mode contexts are never native.
 */
bool rv32_native_jit_init(void);
void rv32_native_jit_bind(
	uint32_t *guest_regs, uint8_t *guest_ram, void *opaque,
	uint32_t (*load)(struct rv32_native_jit_runtime *runtime,
			 uint32_t virtual_address, uint32_t descriptor),
	uint32_t (*store)(struct rv32_native_jit_runtime *runtime,
			  uint32_t virtual_address, uint32_t value,
			  uint32_t descriptor));
/* Install after bind and before execution. Changing the callback flushes
 * compiled regions, which choose whether to emit resumable miss stubs. */
void rv32_native_jit_set_ram_refill(bool (*refill)(
	struct rv32_native_jit_runtime *, uint32_t, uint32_t));
/* The interpreter's fixed S-mode linear map, which fences never change. */
/* Byte offsets of the guest CSR words relative to regs[0], from the
 * interpreter's state structure; generated code reads and writes them
 * directly.  Unset (or out of range) leaves CSR instructions interpreted. */
struct rv32_native_jit_csr_layout {
	int32_t sstatus;
	int32_t sie;
	int32_t stvec;
	int32_t sscratch;
	int32_t sepc;
	int32_t scause;
	int32_t stval;
	int32_t sip;
	int32_t satp;
	int32_t extraflags;
};
void rv32_native_jit_set_csr_layout(
	const struct rv32_native_jit_csr_layout *layout);

void rv32_native_jit_set_kernel_linear_map(uint32_t virtual_base,
					   uint32_t size);
const struct rv32_native_jit_result *rv32_native_jit_try(
	uint32_t physical_offset, uint32_t guest_pc,
	uint32_t instruction_limit, uint32_t memory_context);
/* As rv32_native_jit_try, but counts warm_weight visits toward compiling a
 * cold head; a caller that skips probes between calls passes the skipped
 * count plus one so warm-up timing in guest visits is unchanged. */
const struct rv32_native_jit_result *rv32_native_jit_try_weighted(
	uint32_t physical_offset, uint32_t guest_pc,
	uint32_t instruction_limit, uint32_t memory_context,
	uint32_t warm_weight);
const struct rv32_native_jit_result *rv32_native_jit_try_profiled(
	uint32_t physical_offset, uint32_t guest_pc,
	uint32_t instruction_limit, uint32_t memory_context,
	uint32_t warm_weight);
void rv32_native_jit_flush(void);
void rv32_native_jit_memory_cache_flush(void);
void rv32_native_jit_memory_cache_flush_page(uint32_t virtual_address);
/* Call only between native executions. Revoke all cached data translations
 * whose host pages overlap the supplied surface, including virtual aliases. */
void rv32_native_jit_memory_cache_flush_host_range(const void *host, size_t size);
/* Region exits resolved natively by rv32_native_jit_dispatch since boot. */
uint32_t rv32_native_jit_fast_dispatch_count(void);
/* Nonzero after a failed rv32_native_jit_init: 1 chain results, 2 native
 * dispatch, 3 interior loops, 4 general self-test. */
extern uint32_t rv32_native_jit_self_test_failure;
/* Publish one whole, aligned ordinary-RAM page after the interpreter has
 * checked its bounds and the indicated access permission. MMIO/uncached
 * mappings must not be passed here. Read validation never grants writes. */
void rv32_native_jit_seed_ram_page(uint32_t physical_offset,
	uint32_t virtual_address, uint32_t memory_context, bool write);
void rv32_native_jit_get_stats(struct rv32_native_jit_stats *stats);

static inline void rv32_native_jit_memory_exit(
	struct rv32_native_jit_runtime *runtime, uint32_t descriptor,
	uint32_t trap, uint32_t trap_value)
{
	uint32_t site = descriptor >> 3;

	if (site < runtime->site_count) {
		runtime->result.next_pc = runtime->sites[site].guest_pc;
		runtime->result.executed += runtime->sites[site].executed_before;
	} else {
		runtime->result.next_pc = 0;
	}
	runtime->result.trap = trap;
	runtime->result.trap_value = trap_value;
}

#endif
