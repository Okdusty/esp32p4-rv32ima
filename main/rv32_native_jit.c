#include "rv32_native_jit.h"
#include "rv32_memory_placement.h"
#include "guest_profile.h"
#include "guest_hle.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#ifdef ESP_PLATFORM
#include <stdio.h>
#include "esp_private/esp_cache_private.h"
#include "hal/cache_ll.h"
#include "psram.h"
#include "esp_cpu.h"
#include "driver/gptimer.h"
#endif

#if defined(ESP_PLATFORM) && CONFIG_RV32_NATIVE_JIT_ASYNC_COMPILE
#define JIT_ASYNC_COMPILE 1
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
#define JIT_ASYNC_COMPILE 0
#endif

#if CONFIG_RV32_HOST_PERF_STATS
static inline uint32_t jit_profile_cycle_count(void)
{
#ifdef ESP_PLATFORM
	return esp_cpu_get_cycle_count();
#else
	return 0u;
#endif
}
#endif

/* Generated regions occupy complete 128-byte cache lines in a bounded ring
 * arena.  This keeps publication deterministic on the ESP32-P4's separate
 * data/instruction paths without wasting a fixed 1 KiB on every short region. */
#define JIT_CACHE_SLOTS RV32_NATIVE_JIT_CACHE_SLOTS
#define JIT_CACHE_WAYS 4u
#define JIT_CACHE_SETS (JIT_CACHE_SLOTS / JIT_CACHE_WAYS)
#define JIT_WARM_SLOTS JIT_CACHE_SLOTS
#define JIT_MAX_GUEST_INSTRUCTIONS 32u
#define JIT_MAX_MAPPED_REGISTERS 8u
/* A memory-site descriptor stores its index above three funct3 bits, so its
 * table remains limited to 32 entries even when a region spans more ordinary
 * integer/control instructions.  Memory-heavier candidates simply stop at
 * the existing checked code-generation bound. */
#define JIT_MAX_MEMORY_SITES 32u
#define JIT_MAX_LOOP_READ_CACHES 2u
#define JIT_MAX_LOOP_WRITE_CACHES 1u
#define JIT_MAX_BAILOUT_PATCHES JIT_MAX_MEMORY_SITES
#define JIT_MAX_CONTROL_EXIT_PATCHES JIT_MAX_GUEST_INSTRUCTIONS
#define JIT_MAX_INTERNAL_BRANCH_PATCHES JIT_MAX_GUEST_INSTRUCTIONS
#define JIT_MAX_TRACE_CALL_DEPTH 8u
#define JIT_EXEC_PAGE_ENTRIES 64u
#define JIT_EXEC_HISTORY_BITS 4096u
#define JIT_CODE_WORDS 512u
#define JIT_CODE_BYTES (JIT_CODE_WORDS * 4u)
#define JIT_CODE_ALIGNMENT 128u
#define JIT_CODE_ARENA_MIN_BYTES (64u * 1024u)
#define JIT_INTERNAL_CODE_ARENA_MAX_BYTES (128u * 1024u)
#define JIT_CODE_ARENA_MAX_BYTES (2048u * 1024u)
#define JIT_INTERNAL_RESERVE_BYTES (32u * 1024u)
#define JIT_HOT_CODE_ARENA_MIN_BYTES (16u * 1024u)
#define JIT_HOT_CODE_ARENA_MAX_BYTES (96u * 1024u)
#define JIT_HOT_CODE_ARENA_STEP_BYTES (16u * 1024u)
#define JIT_CODE_UNIT_WORDS (JIT_CODE_ALIGNMENT / sizeof(uint32_t))
#define JIT_LOOP_HOT_THRESHOLD 16u
#define JIT_GENERAL_HOT_THRESHOLD 64u
/* Fork, exec and exit run most kernel paths a few times per process, so at
 * 64 visits much of that code would still be interpreted after dozens of
 * processes; supervisor heads qualify at half the count. */
#define JIT_KERNEL_GENERAL_HOT_THRESHOLD 32u
/* Fill unused internal executable RAM promptly, but require much stronger
 * evidence before a PSRAM-backed region may displace an existing promotion.
 * The two-stage policy avoids the medium-hot ring ping-pong seen with globally
 * lower thresholds while making short-lived render/copy loops benefit from an
 * otherwise idle tier. */
#define JIT_HOT_LOOP_ADMIT_CALLS 256u
#define JIT_HOT_GENERAL_ADMIT_CALLS 512u
#define JIT_HOT_LOOP_REPLACE_CALLS 1024u
#define JIT_HOT_GENERAL_REPLACE_CALLS 2048u
#define JIT_HOT_LOCATION_UNIT_BITS 10u
#define JIT_HOT_LOCATION_UNIT_MASK \
	((1u << JIT_HOT_LOCATION_UNIT_BITS) - 1u)
#define JIT_HOT_OCCUPANCY_WORDS \
	((JIT_HOT_CODE_ARENA_MAX_BYTES / JIT_CODE_ALIGNMENT + 31u) / 32u)
#define JIT_LINK_TARGET_COUNT_MASK 0x3fu
#define JIT_LINK_TARGET_LOOP_FLAG 0x40u
#define JIT_LINK_TARGET_FLAG_MASK (JIT_CODE_ALIGNMENT - 1u)
#define JIT_LINK_CONFIRMATIONS 3u
#define JIT_NATIVE_CHAIN_ENTRY_BYTES (5u * sizeof(uint32_t))
#define JIT_MIN_GUEST_INSTRUCTIONS 3u
#define JIT_SELF_TEST_LOOP_ITERATIONS 32u
/* Guest instructions a native loop may run before the chain checks for a
 * pending host interrupt; one C round trip per this many instructions. */
#define JIT_LOOP_WAKE_BUDGET 8192u
/* Start offsets known not to compile: an unsupported first instruction or
 * fewer than JIT_MIN_GUEST_INSTRUCTIONS collectable ones before an unsupported
 * instruction.  Both depend only on the code bytes, so entries live until the
 * next code flush. */
#define JIT_NEGATIVE_CACHE_ENTRIES 512u
/* Jump cache entries per privilege; see struct jit_jump_entry. */
#define JIT_JUMP_CACHE_ENTRIES 256u
/* memory_site.store value for an inline word atomic. */
#define JIT_SITE_AMO 2u

#ifdef ESP_PLATFORM
#include "rv32_host_wake.h"
#define JIT_HOST_WAKE_PENDING() (rv32_host_wake != 0u)
#define JIT_HOST_WAKE_WORD (&rv32_host_wake)
#else
#define JIT_HOST_WAKE_PENDING() 0
static const uint32_t jit_no_host_wake;
#define JIT_HOST_WAKE_WORD (&jit_no_host_wake)
#endif

/* MiniRV32NativeMemoryContext layout: TLB generation in bits 0..7,
 * privilege in bits 8..9, SUM in bit 10 and MXR in bit 11. */
#define JIT_CONTEXT_EPOCH_MASK 0xffu
#define JIT_CONTEXT_PRIVILEGE_SHIFT 8u
#define JIT_CONTEXT_SUM (1u << 10)
/* Machine mode always stays in the interpreter. */
#if CONFIG_RV32_NATIVE_JIT_SUPERVISOR
#define JIT_MAX_PRIVILEGE 1u
#else
#define JIT_MAX_PRIVILEGE 0u
#endif

/* Generated transition counters are diagnostic only.  Standalone tests keep
 * them by default so they can prove whether a native tail link was taken;
 * production firmware omits their per-block stores.  Tests may explicitly
 * set this to zero to exercise the exact counterless code shape. */
#ifndef RV32_NATIVE_JIT_RUNTIME_COUNTERS
#if !defined(ESP_PLATFORM) || CONFIG_RV32_HOST_PERF_STATS || \
	CONFIG_RV32_EMULATOR_PERF_STATS
#define RV32_NATIVE_JIT_RUNTIME_COUNTERS 1
#else
#define RV32_NATIVE_JIT_RUNTIME_COUNTERS 0
#endif
#endif

#define JIT_WARM_CLASSIFIED 0x01u
#define JIT_WARM_LOOPS_TO_START 0x02u
/* Queued for the compile worker. */
#define JIT_WARM_PENDING 0x04u
/* The head's first instruction has not been examined yet: most heads are
 * visited once, so a first visit only counts and the second one checks. */
#define JIT_WARM_UNCHECKED 0x08u

#if (JIT_CACHE_SLOTS & (JIT_CACHE_SLOTS - 1u)) != 0
#error "JIT_CACHE_SLOTS must be a power of two"
#endif
#if (JIT_CACHE_SETS & (JIT_CACHE_SETS - 1u)) != 0
#error "JIT_CACHE_SETS must be a power of two"
#endif
#if (JIT_WARM_SLOTS & (JIT_WARM_SLOTS - 1u)) != 0
#error "JIT_WARM_SLOTS must be a power of two"
#endif
#if JIT_CODE_BYTES % JIT_CODE_ALIGNMENT != 0
#error "The maximum native region must occupy complete L2 cache lines"
#endif
#if JIT_CODE_ARENA_MIN_BYTES % JIT_CODE_ALIGNMENT != 0 || \
    JIT_CODE_ARENA_MAX_BYTES % JIT_CODE_ALIGNMENT != 0
#error "The native JIT arena must occupy complete L2 cache lines"
#endif
#if (RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES & \
     (RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u)) != 0
#error "RV32 native JIT page-cache size must be a power of two"
#endif

enum jit_entry_state {
	JIT_ENTRY_EMPTY,
	JIT_ENTRY_READY,
	JIT_ENTRY_REJECTED,
};

enum jit_instruction_kind {
	JIT_INSN_UNSUPPORTED,
	JIT_INSN_INTEGER,
	JIT_INSN_LOAD,
	JIT_INSN_STORE,
	JIT_INSN_BRANCH,
	JIT_INSN_JAL,
	JIT_INSN_JALR,
	JIT_INSN_AMO,
	JIT_INSN_CSR,
	JIT_INSN_NOP,
};

typedef uint32_t (*jit_block_fn)(struct rv32_native_jit_runtime *runtime);

/* Assembly wrapper preserving the host ABI once around a complete linked
 * native chain.  Native successors tail-jump without crossing this boundary. */
extern uint32_t rv32_native_jit_enter(
	jit_block_fn block, struct rv32_native_jit_runtime *runtime);
extern void rv32_native_jit_refill_trampoline(void);

struct jit_cache_entry {
	uint32_t physical_offset;
	uint32_t guest_pc;
	uint8_t guest_instruction_count;
	uint8_t state;
	uint8_t memory_site_count;
	uint8_t loops_to_start;
	uint16_t code_unit;
	uint8_t code_units;
	uint8_t spans_pages;
};

struct jit_exec_page {
	uint32_t virtual_page;
	uint32_t physical_page_offset;
};

struct jit_warm_entry {
	uint32_t physical_offset;
	uint32_t guest_pc;
	uint16_t hits;
	uint8_t valid;
	uint8_t reserved;
};

/* A promoted region retains its authoritative copy and metadata in the main
 * PSRAM arena.  This compact per-directory-slot record merely redirects hot
 * execution to an internal-SRAM copy, so eviction never forces recompilation. */
struct jit_hot_entry {
	/* Zero means not promoted. Otherwise the low bits hold start + 1 and
	 * the high bits hold length - 1, both in cache-line units. */
	uint16_t location;
	uint16_t calls;
};

/* Native exits to a target the region's own link does not predict (returns,
 * indirect calls, the other side of a branch) look up their guest PC here
 * before the full directory search in rv32_native_jit_dispatch.  packed_target
 * has the jit_links format.  Entries depend on the current translations and
 * go with the links; guest_pc UINT32_MAX (never a 4-byte aligned PC) is
 * empty. */
struct jit_jump_entry {
	uint32_t guest_pc;
	uint32_t packed_target;
};

/* Read by rv32_native_jit_dispatch (rv32_native_jit_enter.S), which hard-codes
 * these byte offsets and the entry/hot-entry layouts asserted below. */
struct rv32_native_jit_dispatch_table {
	struct jit_cache_entry *entries;	/* 0 */
	struct jit_hot_entry *hot_entries;	/* 4 */
	uint32_t *code_arena;			/* 8 */
	uint32_t *hot_code_arena;		/* 12 */
	const volatile uint32_t *host_wake;	/* 16 */
	uint32_t fast_dispatches;		/* 20 */
	uint32_t loop_iterations_offset;	/* 24 */
	uint32_t set_mask;			/* 28 */
	uint32_t loop_wake_budget;		/* 32 */
	uint32_t hot_loop_admit_calls;		/* 36 */
	uint32_t hot_general_admit_calls;	/* 40 */
	uint32_t hot_loop_replace_calls;	/* 44 */
	uint32_t hot_general_replace_calls;	/* 48 */
	uint8_t *link_learning_budget;		/* 52 */
	struct jit_exec_page *exec_pages;	/* 56 */
	const uint32_t *exec_privilege_tag;	/* 60 */
	struct jit_jump_entry *jump_cache;	/* 64: this privilege's */
	struct jit_link_entry *links;		/* 68 */
	uint8_t *link_targets;			/* 72 */
	uint32_t *jump_misses;			/* 76 */
};
struct rv32_native_jit_dispatch_table rv32_native_jit_dispatch_table;
/* Which init self-test group failed, for the firmware's diagnostic. */
uint32_t rv32_native_jit_self_test_failure;
/* A freshly compiled region sends its first JIT_LINK_CONFIRMATIONS same-page
 * exits through C so jit_observe_link can still publish a native link; the
 * stub then resolves the remaining (polymorphic) exits itself. */
static uint8_t jit_link_learning_budget[JIT_CACHE_SLOTS];
extern void rv32_native_jit_dispatch(void);

_Static_assert(sizeof(struct jit_cache_entry) == 16u &&
	       offsetof(struct jit_cache_entry, guest_instruction_count) == 8u &&
	       offsetof(struct jit_cache_entry, state) == 9u &&
	       offsetof(struct jit_cache_entry, loops_to_start) == 11u &&
	       offsetof(struct jit_cache_entry, code_unit) == 12u,
	       "rv32_native_jit_dispatch hard-codes the cache entry layout");
_Static_assert(sizeof(struct jit_hot_entry) == 4u &&
	       offsetof(struct jit_hot_entry, calls) == 2u,
	       "rv32_native_jit_dispatch hard-codes the hot entry layout");
_Static_assert(offsetof(struct rv32_native_jit_dispatch_table,
			exec_privilege_tag) == 60u &&
	       offsetof(struct rv32_native_jit_dispatch_table,
			jump_cache) == 64u &&
	       offsetof(struct rv32_native_jit_dispatch_table, links) == 68u &&
	       offsetof(struct rv32_native_jit_dispatch_table,
			link_targets) == 72u &&
	       sizeof(struct jit_jump_entry) == 8u &&
	       JIT_JUMP_CACHE_ENTRIES == 256u &&
	       sizeof(struct jit_exec_page) == 8u &&
	       JIT_EXEC_PAGE_ENTRIES == 64u,
	       "rv32_native_jit_dispatch hard-codes the table layout");
_Static_assert(JIT_ENTRY_READY == 1u && JIT_CACHE_WAYS == 4u &&
	       JIT_CODE_ALIGNMENT == 128u && JIT_HOT_LOCATION_UNIT_BITS == 10u &&
	       JIT_NATIVE_CHAIN_ENTRY_BYTES == 20u,
	       "rv32_native_jit_dispatch hard-codes these constants");

/* Generated tails read this compact directory on every native-region edge.
 * Keep it in internal RAM: the Doom profile traverses roughly 1.5 million
 * links per second, so fetching these two words from the old 40 KiB PSRAM
 * records needlessly competed with guest data and generated code in L2. */
struct jit_link_entry {
	uint32_t packed_target;
	uint32_t expected_next_pc;
};

/* Link training runs only after returning through the C dispatcher.  Keep its
 * colder candidate state in reserved PSRAM instead of spending internal RAM
 * on fields that generated code never reads. */
struct jit_link_candidate {
	uint32_t candidate_target;
	uint32_t candidate_next_pc;
	uint8_t candidate_confirmations;
	uint8_t reserved[3];
};

struct jit_register_use {
	uint8_t reads;
	uint8_t writes;
	uint8_t needs_initial;
	uint8_t first_access_seen;
};

struct jit_register_map {
	int8_t host_for_guest[32];
	uint8_t guest_for_host[JIT_MAX_MAPPED_REGISTERS];
	uint8_t host_register[JIT_MAX_MAPPED_REGISTERS];
	uint8_t needs_initial[JIT_MAX_MAPPED_REGISTERS];
	uint8_t dirty[JIT_MAX_MAPPED_REGISTERS];
	uint8_t count;
};

struct jit_deferred_guard {
	uint32_t *instruction;
	uint8_t funct3;
	uint8_t source1;
	uint8_t source2;
	bool refill;
};

struct jit_memory_stub {
	struct jit_deferred_guard guards[3];
	uint8_t guard_count;
	uint8_t site;
	uint8_t store;
	uint8_t grouped;
	int16_t group_immediate;
	uint8_t group_value_guest;
	uint8_t padding;
};

struct jit_loop_read_cache {
	uint8_t instruction_index;
	uint8_t tag_register;
	uint8_t bias_register;
	uint8_t valid;
	uint8_t alignment_invariant;
};

struct jit_loop_read_cache_stub {
	uint32_t *miss_instruction;
	uint32_t *continuation;
	uint8_t memory_stub;
	uint8_t tag_register;
	uint8_t bias_register;
	uint8_t alignment_mask;
};

struct jit_loop_write_cache {
	uint8_t instruction_index;
	uint8_t valid;
	uint8_t alignment_invariant;
};

struct jit_loop_write_cache_stub {
	uint32_t *miss_instruction;
	uint32_t *continuation;
	uint8_t memory_stub;
	uint8_t funct3;
	uint8_t width;
	uint8_t alignment_mask;
};

struct jit_two_page_store_stub {
	uint32_t *range_miss;
	uint32_t *continuation;
	const uint32_t *instructions;
	int16_t min_offset;
	int16_t max_end;
	uint8_t memory_stub;
	uint8_t count;
	uint8_t alignment_mask;
};

/* A kernel-region load, store, group or persistent-stream guard that tried
 * the linear map inline.  Its miss branch runs the ordinary page-cache
 * emitter's code out of line, which then resumes after the inline access. */
enum jit_linear_kind {
	JIT_LINEAR_LOAD,
	JIT_LINEAR_STORE,
	JIT_LINEAR_LOAD_GROUP,
	JIT_LINEAR_STORE_GROUP,
	JIT_LINEAR_PERSISTENT,
};

struct jit_linear_slow_path {
	uint32_t *miss;
	uint32_t *continuation;
	/* A group's members in the trace; otherwise the one instruction. */
	const uint32_t *instructions;
	uint32_t instruction;
	uint32_t guest_pc;
	int16_t group_min_offset;
	int16_t group_max_end;
	uint8_t group_count;
	uint8_t group_alignment_mask;
	uint8_t kind;
	uint8_t executed_before;
};

struct jit_builder {
	uint32_t *start;
	uint32_t *cursor;
	uint32_t *end;
	uint32_t *bailout_patches[JIT_MAX_BAILOUT_PATCHES];
	uint32_t *control_exit_patches[JIT_MAX_CONTROL_EXIT_PATCHES];
	uint32_t *hard_exit_patches[JIT_MAX_CONTROL_EXIT_PATCHES];
	uint32_t *dirty_call_patches[JIT_MAX_MEMORY_SITES];
	struct jit_memory_stub *memory_stubs;
	struct jit_loop_read_cache_stub *loop_read_cache_stubs;
	struct jit_loop_write_cache_stub *loop_write_cache_stubs;
	struct jit_two_page_store_stub *two_page_store_stubs;
	struct jit_linear_slow_path *linear_slow_paths;
	uint8_t bailout_patch_count;
	uint8_t control_exit_patch_count;
	uint8_t hard_exit_patch_count;
	uint8_t dirty_call_patch_count;
	uint8_t memory_stub_count;
	uint8_t loop_read_cache_stub_count;
	uint8_t loop_write_cache_stub_count;
	uint8_t two_page_store_stub_count;
	uint8_t linear_slow_path_count;
	bool instruction_counter_active;
	bool failed;
	/* Deferred memory stubs may retry through the RAM refill path. */
	bool refill;
	/* A kernel region whose loads and stores try the linear map first.  The
	 * preamble loads the limit and host registers; the negated window start
	 * is a third register, or RV_X0 to rebuild it from linear_offset at
	 * each access. */
	bool linear;
	uint8_t linear_limit_register;
	uint8_t linear_host_register;
	uint8_t linear_offset_register;
	uint32_t linear_offset;
};

struct jit_internal_branch_patch {
	uint32_t *instruction;
	uint8_t target_index;
	uint8_t funct3;
	uint8_t source1;
	uint8_t source2;
};

struct jit_memory_group {
	uint32_t count;
	int32_t min_offset;
	int32_t max_end;
	uint32_t alignment_mask;
};

/* Entry values only steer grouping; generated checks must not assume they
 * still hold on a later entry. */
struct jit_group_hint {
	const uint32_t *entry_values;
	uint32_t invariant_registers;
};

struct jit_persistent_memory_group {
	struct jit_memory_group memory;
	uint32_t first;
	uint32_t last;
	uint32_t base;
	uint32_t total_bytes;
	bool store;
	bool valid;
};

struct jit_group_guards {
	uint32_t *range_miss;
	uint32_t *alignment_miss;
	uint32_t *page_miss;
};

/* Compilation is synchronous on the emulator core.  Keep its large bounded
 * analysis tables out of the 4 KiB FreeRTOS main-task stack; generated code
 * and the hot dispatcher never touch this cold workspace. */
struct jit_compile_workspace {
	uint32_t instructions[JIT_MAX_GUEST_INSTRUCTIONS];
	uint32_t guest_pcs[JIT_MAX_GUEST_INSTRUCTIONS];
	enum jit_instruction_kind kinds[JIT_MAX_GUEST_INSTRUCTIONS];
	bool leaders[JIT_MAX_GUEST_INSTRUCTIONS];
	uint32_t *native_labels[JIT_MAX_GUEST_INSTRUCTIONS];
	struct jit_internal_branch_patch
		internal_branch_patches[JIT_MAX_INTERNAL_BRANCH_PATCHES];
	struct jit_memory_stub memory_stubs[JIT_MAX_MEMORY_SITES];
	struct jit_loop_read_cache_stub
		loop_read_cache_stubs[JIT_MAX_LOOP_READ_CACHES];
	struct jit_loop_write_cache_stub
		loop_write_cache_stubs[JIT_MAX_LOOP_WRITE_CACHES];
	struct jit_two_page_store_stub two_page_store_stubs[JIT_MAX_MEMORY_SITES];
	struct jit_linear_slow_path linear_slow_paths[JIT_MAX_MEMORY_SITES];
	uint32_t trace_return_pcs[JIT_MAX_TRACE_CALL_DEPTH];
	uint8_t trace_link_registers[JIT_MAX_TRACE_CALL_DEPTH];
};

static RV32_HOT_STATE_BSS struct jit_cache_entry jit_entries[JIT_CACHE_SLOTS];
static struct jit_warm_entry *jit_warm;
/* Every failing dispatch probe touches one warm entry, so the table stays in
 * internal RAM even though the rest of the cold directory lives in PSRAM. */
static struct jit_warm_entry jit_warm_storage[JIT_WARM_SLOTS];
static RV32_HOT_STATE_BSS _Alignas(JIT_CODE_ALIGNMENT) struct jit_link_entry
	jit_link_storage[JIT_CACHE_SLOTS];
static struct jit_link_entry *jit_links = jit_link_storage;
/* Nonzero once a link or jump-cache entry may hold the slot's code pointer:
 * set wherever one is published (jit_observe_link, the assembly dispatcher),
 * cleared with the last such pointer.  Evicting or moving a region that
 * nothing reached natively then skips the scan of every link. */
static RV32_HOT_STATE_BSS uint8_t jit_link_targets[JIT_CACHE_SLOTS];
/* How often the region ran since the arena's replacement hand last passed
 * it: the C dispatcher counts its entries and a 1 kHz timer on the emulator
 * core counts samples of the PC inside its code (jit_sample_pc), so native
 * links and jumps cost nothing.  The hand clears it and spares the region
 * once (second chance), promotes a region above the bar to the hot tier,
 * and the directory prefers ways without it. */
static RV32_HOT_STATE_BSS uint16_t jit_entry_entered[JIT_CACHE_SLOTS];
/* Regions the hand moves past per install before it evicts regardless. */
#define JIT_ARENA_SPARES 32u
/* Hot copies the tier's hand moves past per promotion (each a directory scan). */
#define JIT_HOT_SPARES 4u
/* The promotion bar, in entries and samples since the hand last passed: once
 * a second it doubles when more than JIT_PROMOTE_RATE_HIGH regions were
 * promoted in that second and halves below JIT_PROMOTE_RATE_LOW, so the
 * tier's few slots turn over at a steady, cheap rate. */
#define JIT_PROMOTE_BAR_MIN 16u
#define JIT_PROMOTE_BAR_MAX 8192u
#define JIT_PROMOTE_RATE_HIGH 30u
#define JIT_PROMOTE_RATE_LOW 10u
static uint32_t jit_promote_bar = 64u;
static uint32_t jit_promotions_this_second;
#ifdef ESP_PLATFORM
static uint32_t jit_promote_bar_tick_at;
#endif
/* Owner map of the hot tier's units (slot + 1), for the sampler. */
static uint16_t jit_hot_unit_owner[JIT_HOT_CODE_ARENA_MAX_BYTES /
				   JIT_CODE_ALIGNMENT];
static RV32_HOT_STATE_BSS struct jit_jump_entry
	jit_jump_cache[JIT_MAX_PRIVILEGE + 1u][JIT_JUMP_CACHE_ENTRIES];
/* Exit targets the dispatch stub searched the directory for and did not
 * find: a guest PC, UINT32_MAX when empty.  Most chains end at such a target
 * (code not compiled yet, never compiled, or a host routine), and every exit
 * there repeated the page and hash work and the four-way set search before
 * returning to C; a recorded target returns after one load.  The stub checks
 * the table once a jump-cache miss has passed the learning budget, so links
 * are learned as before, and records a target only after a full search.
 * Like the jump cache, entries hold under the current translations and go
 * with the links (jit_clear_all_links: translation and executed-page
 * flushes, fences, jit_flush).  The only way a target gains a ready region
 * is jit_install_generated_body, which clears its entry, so an entry never
 * hides one.  One table serves both privileges: a target is only searched
 * on a page its privilege executes (the exiting region's page, or one the
 * exec-page cache validated for that privilege), and U and S never execute
 * the same page, so an entry recorded in one privilege can never stand for
 * a region of the other; at worst it sends an exit to C, which is always
 * safe.  Indexed by ((pc >> 8) ^ pc) & 0x7fc as a byte offset, which mixes
 * the page into the low PC bits in three instructions. */
#define JIT_JUMP_MISS_ENTRIES 512u
static RV32_HOT_STATE_BSS uint32_t jit_jump_misses[JIT_JUMP_MISS_ENTRIES];
_Static_assert(offsetof(struct rv32_native_jit_dispatch_table,
			jump_misses) == 76u && JIT_JUMP_MISS_ENTRIES == 512u,
	       "rv32_native_jit_dispatch hard-codes the recorded-miss table");

static inline uint32_t *jit_jump_miss_slot(uint32_t guest_pc)
{
	return &jit_jump_misses[(((guest_pc >> 8) ^ guest_pc) >> 2) &
				(JIT_JUMP_MISS_ENTRIES - 1u)];
}

/* guest_pc gains a ready region: the table may no longer record it. */
static void jit_forget_jump_miss(uint32_t guest_pc)
{
	uint32_t *miss = jit_jump_miss_slot(guest_pc);

	if (*miss == guest_pc)
		*miss = UINT32_MAX;
}
static struct jit_link_candidate *jit_link_candidates;
#ifndef ESP_PLATFORM
static struct jit_link_candidate jit_link_candidate_storage[JIT_CACHE_SLOTS];
#endif
static uint8_t jit_replace_way[JIT_CACHE_SETS];
static RV32_HOT_STATE_BSS struct jit_hot_entry jit_hot_entries[JIT_CACHE_SLOTS];
static struct jit_exec_page jit_exec_pages[JIT_EXEC_PAGE_ENTRIES];
static RV32_HOT_STATE_BSS uint32_t jit_negative_cache[JIT_NEGATIVE_CACHE_ENTRIES];

static inline uint32_t *jit_negative_slot(uint32_t physical_offset)
{
	return &jit_negative_cache[(physical_offset >> 2) &
				   (JIT_NEGATIVE_CACHE_ENTRIES - 1u)];
}

/* Offsets are word aligned, so the low bit marks an occupied slot. */
static inline void jit_remember_negative(uint32_t physical_offset)
{
	*jit_negative_slot(physical_offset) = physical_offset | 1u;
}
/* Pages executed under the current translations.  Bits survive eviction and
 * fences: a false positive costs a link flush, a false negative would keep
 * stale links after a remap. */
static uint32_t jit_exec_history[JIT_EXEC_HISTORY_BITS / 32u];
static RV32_STATE_BSS struct jit_compile_workspace jit_compile_workspace;
/* The mapping cache still permits safe C-side chaining across validated Sv32
 * pages.  Keep native trace construction within its starting page in normal
 * operation: profiling showed that the much larger multi-page regions caused
 * enough arena and tag churn to lose throughput.  The correctness self-test
 * enables this briefly so the dormant generic mechanism remains covered. */
static bool jit_cross_page_trace_self_test;
static bool jit_has_cross_page_entries;
static uint32_t *jit_code_arena;
static uint32_t *jit_hot_code_arena;
#ifndef ESP_PLATFORM
static _Alignas(JIT_CODE_ALIGNMENT) uint32_t
	jit_hot_code_storage[JIT_HOT_CODE_ARENA_MIN_BYTES / sizeof(uint32_t)];
#endif
static RV32_STATE_BSS _Alignas(JIT_CODE_ALIGNMENT) uint32_t jit_staging[JIT_CODE_WORDS];
static uint16_t jit_loop_code_head;
static uint16_t jit_regular_code_head;
/* One ring for every region (JIT_ARENA_SINGLE_RING): second chance keeps
 * the hot loops and kernel code, so fixed partitions only shrink the space
 * each class may use. */
#define JIT_ARENA_SINGLE_RING 1
static uint16_t jit_code_head;
/* With JIT_KEEP_KERNEL, kernel regions take the first half of each
 * partition and user regions the second half (the heads above). */
static uint16_t jit_kernel_loop_code_head;
static uint16_t jit_kernel_regular_code_head;
/* The longest region, in arena units: a region overlapping a new allocation
 * at start therefore begins at or after start - (JIT_MAX_REGION_UNITS - 1).
 * A region's memory-site table fills the end of its last units. */
#define JIT_MAX_REGION_UNITS ((JIT_CODE_BYTES + JIT_MAX_MEMORY_SITES * \
	sizeof(struct rv32_native_jit_memory_site) + JIT_CODE_ALIGNMENT - 1u) / \
	JIT_CODE_ALIGNMENT)
/* Directory slot + 1 of the region that begins at each arena unit, or 0.
 * Checked against the entry on use, so values left by evictions, rejections
 * and flushes are harmless.  NULL scans the whole directory instead. */
static uint16_t *jit_unit_owner;
#ifndef ESP_PLATFORM
static uint16_t
	jit_unit_owner_storage[JIT_CODE_ARENA_MAX_BYTES / JIT_CODE_ALIGNMENT];
#endif
static uint16_t jit_loop_code_units;
static uint16_t jit_code_arena_units;
static uint16_t jit_hot_code_head;
static uint16_t jit_hot_code_arena_units;
static uint32_t jit_hot_code_occupancy[JIT_HOT_OCCUPANCY_WORDS];
/* Data-page caches for U, S and S with SUM, so traps and uaccess windows
 * switch banks instead of flushing.  Generated code uses the runtime it was
 * entered with; jit_rt is the last adopted bank. */
enum jit_memory_bank {
	JIT_BANK_USER,
	JIT_BANK_SUPERVISOR,
	JIT_BANK_SUPERVISOR_SUM,
	JIT_MEMORY_BANKS,
};
static struct rv32_native_jit_runtime jit_runtimes[JIT_MEMORY_BANKS];
static struct rv32_native_jit_runtime *jit_rt = &jit_runtimes[JIT_BANK_USER];
/* TLB epoch of the shared exec mappings and links; UINT32_MAX when empty. */
static uint32_t jit_memory_epoch = UINT32_MAX;
/* U and S never execute the same Sv32 page, so exec mappings carry the
 * privilege above the 20-bit VPN. */
static uint32_t jit_exec_privilege_tag;

static void jit_set_exec_privilege(uint32_t privilege)
{
	jit_exec_privilege_tag = privilege << 20;
	rv32_native_jit_dispatch_table.jump_cache = jit_jump_cache[privilege];
}

/* S-mode fetches here use the fixed kernel linear map; fences never change it. */
static uint32_t jit_kernel_linear_base;
static uint32_t jit_kernel_linear_size;

/*
 * Kernel linear-map data accesses (rv32_native_jit_set_kernel_linear_data).
 * The interpreter maps an S-mode address va with va - base < size, paging
 * on, straight to guest RAM offset phys + (va - base), with no page walk,
 * permission check or write-page report.  Generated kernel regions do the
 * same before their page-cache lookup, in four host instructions:
 *
 *	add	t1, rs1, (-base - LOW)	register or lui
 *	bgeu	t1, limit, slow		slow: the unchanged page-cache code
 *	add	t1, t1, host		host = RAM + phys + LOW
 *	lw	rd, imm(t1)
 *
 * The check is on the base register, so its 12-bit displacement must stay in
 * the window too: a base offset below JIT_LINEAR_LOW_MARGIN (4 KiB >= 2048)
 * or within JIT_LINEAR_HIGH_MARGIN (4 KiB >= 2047 + 3) of the end takes the
 * slow path, and limit = size - LOW - HIGH.  Every fast access, misaligned
 * or not, then lies inside the window and so inside guest RAM; one check
 * covers a whole group of displacements off one base.  The slow path also
 * handles the window's first and last 4 KiB, which on this guest are kernel
 * head text and the reserved display/DTB area.
 */
#define JIT_LINEAR_LOW_MARGIN 4096u
#define JIT_LINEAR_HIGH_MARGIN 4096u
static uint32_t jit_kernel_linear_phys;
static bool jit_kernel_linear_data;
/* Callee-saved registers a region may give the check (two or three).  The
 * self-test lowers it to cover the form that rebuilds the window offset. */
static uint8_t jit_linear_register_budget = 3u;

#if defined(CONFIG_RV32_NATIVE_JIT_KEEP_KERNEL) && \
	CONFIG_RV32_NATIVE_JIT_KEEP_KERNEL
#define JIT_KEEP_KERNEL 1
#else
#define JIT_KEEP_KERNEL 0
#endif
#if JIT_KEEP_KERNEL
/* Guest RAM pages that resident kernel regions' instructions came from, one
 * bit each.  Linux rewrites kernel text only through a fresh write mapping
 * (the text-poke fixmap, for static keys), which reaches
 * rv32_native_jit_note_write_page; the next FENCE.I then discards everything.
 * Any other FENCE.I, after mapping a new user page, keeps kernel regions. */
#define JIT_KERNEL_CODE_PAGES (64u * 1024u * 1024u / 4096u)
static uint32_t jit_kernel_code_pages[JIT_KERNEL_CODE_PAGES / 32u];
static bool jit_kernel_code_written;
/* Advances on full flushes only: queued kernel regions outlive user fences. */
static uint32_t jit_kernel_generation;

static inline bool jit_kernel_pc(uint32_t guest_pc)
{
	return guest_pc - jit_kernel_linear_base < jit_kernel_linear_size;
}

/* Slots that took a user region since the last FENCE.I, so the next one
 * visits only those instead of the whole directory; a count past the end
 * means the list overflowed and the fence scans every slot. */
#define JIT_USER_SLOT_LIST 1024u
static uint16_t jit_user_slots[JIT_USER_SLOT_LIST];
static uint32_t jit_user_slot_count;

static inline void jit_note_user_slot(uint32_t slot, uint32_t guest_pc)
{
	if (jit_kernel_pc(guest_pc) || jit_user_slot_count > JIT_USER_SLOT_LIST)
		return;
	if (jit_user_slot_count < JIT_USER_SLOT_LIST)
		jit_user_slots[jit_user_slot_count] = (uint16_t)slot;
	jit_user_slot_count++;
}
#endif
static void jit_invalidate_translations(void);
static struct rv32_native_jit_stats jit_stats;
/* Advanced by every code flush; asynchronous compiles queued before the
 * latest flush are dropped. */
static uint32_t jit_code_generation;
#if CONFIG_RV32_HOST_PERF_STATS
/* One internal-RAM add per top-level native dispatch is substantially less
 * invasive than reading the cycle counter around every generated region.
 * Slots are cleared when their metadata is replaced so a counter can never be
 * reported under a different guest PC. */
static uint32_t jit_profile_guest_instructions[JIT_CACHE_SLOTS];
static uint32_t jit_profile_discarded_guest_instructions;
#endif

static void jit_reset_code_heads(void)
{
	jit_loop_code_head = 0u;
	jit_regular_code_head = jit_loop_code_units;
	jit_kernel_loop_code_head = 0u;
	jit_kernel_regular_code_head = jit_loop_code_units;
	jit_code_head = 0u;
	jit_hot_code_head = 0u;
	memset(jit_hot_code_occupancy, 0, sizeof(jit_hot_code_occupancy));
}

_Static_assert(sizeof(struct jit_cache_entry) == 16u,
	       "JIT hot lookup entry must remain a power-of-two size");
_Static_assert(sizeof(struct jit_link_entry) == 8u,
	       "JIT native-link entries must remain two words");
_Static_assert((JIT_CACHE_WAYS & (JIT_CACHE_WAYS - 1u)) == 0u &&
	       JIT_CACHE_WAYS <= UINT8_MAX,
	       "The replacement way wraps with a mask and fits a byte");
_Static_assert((JIT_EXEC_PAGE_ENTRIES & (JIT_EXEC_PAGE_ENTRIES - 1u)) == 0u,
	       "The executable-page cache size must be a power of two");
_Static_assert(JIT_CODE_ARENA_MAX_BYTES / JIT_CODE_ALIGNMENT <= UINT16_MAX,
	       "JIT arena offsets must fit their metadata field");
_Static_assert(JIT_MAX_REGION_UNITS <= UINT8_MAX,
	       "JIT region lengths must fit their metadata field");
_Static_assert(JIT_HOT_CODE_ARENA_MIN_BYTES % JIT_CODE_ALIGNMENT == 0u &&
	       JIT_HOT_CODE_ARENA_MAX_BYTES % JIT_CODE_ALIGNMENT == 0u,
	       "JIT hot-code arena must occupy complete cache lines");
_Static_assert(JIT_HOT_GENERAL_REPLACE_CALLS <= UINT16_MAX,
	       "JIT hot-code admission counter must fit its metadata field");
_Static_assert(JIT_HOT_LOOP_ADMIT_CALLS < JIT_HOT_LOOP_REPLACE_CALLS &&
	       JIT_HOT_GENERAL_ADMIT_CALLS < JIT_HOT_GENERAL_REPLACE_CALLS,
	       "JIT hot-code free-space admission must precede replacement");
_Static_assert(JIT_HOT_CODE_ARENA_MAX_BYTES / JIT_CODE_ALIGNMENT <=
	       JIT_HOT_LOCATION_UNIT_MASK,
	       "JIT hot-code arena offset must fit its packed metadata field");
_Static_assert(JIT_MAX_REGION_UNITS <=
	       (1u << (16u - JIT_HOT_LOCATION_UNIT_BITS)),
	       "JIT hot-code region length must fit its packed metadata field");
_Static_assert(JIT_MAX_GUEST_INSTRUCTIONS <= JIT_LINK_TARGET_COUNT_MASK,
	       "JIT link target count must fit its packed pointer");
_Static_assert((JIT_CODE_ALIGNMENT & (JIT_CODE_ALIGNMENT - 1u)) == 0u &&
	       JIT_CODE_ALIGNMENT > JIT_LINK_TARGET_LOOP_FLAG,
	       "JIT link flags require aligned generated-code pointers");
_Static_assert(sizeof(struct rv32_native_jit_read_page) == 8u,
	       "JIT read-page entries are indexed with a three-bit shift");
_Static_assert(sizeof(struct rv32_native_jit_write_page) == 32u,
	       "JIT write-page entries are indexed with a five-bit shift");
_Static_assert(offsetof(struct rv32_native_jit_runtime, dispatch_miss_pc) == 92u,
	       "rv32_native_jit_dispatch stores dispatch_miss_pc at 92(t6)");
_Static_assert(offsetof(struct rv32_native_jit_runtime, linear_host) <= 2047u,
	       "the linear-map preamble loads its words with 12-bit offsets");
#ifdef ESP_PLATFORM
_Static_assert(JIT_CACHE_SLOTS * sizeof(struct jit_link_candidate) <=
	       HOST_JIT_METADATA_SIZE,
	       "The reserved host JIT metadata region is too small");
#endif

#define JIT_WARM_BYTES (JIT_WARM_SLOTS * sizeof(*jit_warm))

#define JIT_RESULT_OFFSET(field) \
	(offsetof(struct rv32_native_jit_runtime, result) + \
	 offsetof(struct rv32_native_jit_result, field))

enum {
	RV_X0 = 0,
	RV_RA = 1,
	RV_SP = 2,
	RV_T0 = 5,
	RV_T1 = 6,
	RV_T2 = 7,
	RV_S0 = 8,
	RV_S1 = 9,
	RV_A0 = 10,
	RV_A1 = 11,
	RV_A2 = 12,
	RV_A3 = 13,
	RV_A4 = 14,
	RV_A5 = 15,
	RV_A6 = 16,
	RV_A7 = 17,
	RV_S2 = 18,
	RV_S9 = 25,
	RV_S10 = 26,
	RV_S11 = 27,
	RV_T3 = 28,
	RV_T4 = 29,
	RV_T5 = 30,
	RV_T6 = 31,
};

/* Generated code executes inside rv32_native_jit_enter's shared ABI frame.
 * The out-of-line linear-dirty helper links through t4 rather than ra, leaving
 * the wrapper return address untouched.  Repeated regions never map enough
 * guest registers to reach s9, so it remains available for the write-page tag. */
#define JIT_RUNTIME_REGISTER RV_T6
#define JIT_GUEST_REGS_REGISTER RV_T5
#define JIT_DIRTY_LINK_REGISTER RV_T4
#define JIT_LOOP_REGISTER RV_A6
#define JIT_EXECUTED_REGISTER RV_A7
#define JIT_LOOP_WRITE_TAG_REGISTER RV_S9
#define JIT_LOOP_WRITE_BIAS_REGISTER RV_A5
#define JIT_CHAIN_EXECUTED_REGISTER RV_S10
#define JIT_CHAIN_LIMIT_REGISTER RV_S11

_Static_assert(JIT_MAX_MAPPED_REGISTERS <= 8u,
	       "s9 is reserved for the native loop write-page tag");

static inline uint32_t rv_r(uint32_t opcode, uint32_t rd, uint32_t funct3,
			    uint32_t rs1, uint32_t rs2, uint32_t funct7)
{
	return (funct7 << 25) | (rs2 << 20) | (rs1 << 15) |
	       (funct3 << 12) | (rd << 7) | opcode;
}

static inline uint32_t rv_i(uint32_t opcode, uint32_t rd, uint32_t funct3,
			    uint32_t rs1, int32_t immediate)
{
	return (((uint32_t)immediate & 0xfffu) << 20) | (rs1 << 15) |
	       (funct3 << 12) | (rd << 7) | opcode;
}

static inline uint32_t rv_s(uint32_t opcode, uint32_t funct3, uint32_t rs1,
			    uint32_t rs2, int32_t immediate)
{
	uint32_t imm = (uint32_t)immediate & 0xfffu;

	return ((imm >> 5) << 25) | (rs2 << 20) | (rs1 << 15) |
	       (funct3 << 12) | ((imm & 0x1fu) << 7) | opcode;
}

static inline uint32_t rv_b(uint32_t funct3, uint32_t rs1, uint32_t rs2,
			    int32_t offset)
{
	uint32_t imm = (uint32_t)offset & 0x1fffu;

	return (((imm >> 12) & 1u) << 31) |
	       (((imm >> 5) & 0x3fu) << 25) |
	       (rs2 << 20) | (rs1 << 15) | (funct3 << 12) |
	       (((imm >> 1) & 0x0fu) << 8) |
	       (((imm >> 11) & 1u) << 7) | 0x63u;
}

static inline uint32_t rv_j(uint32_t rd, int32_t offset)
{
	uint32_t imm = (uint32_t)offset & 0x1fffffu;

	return (((imm >> 20) & 1u) << 31) |
	       (((imm >> 1) & 0x3ffu) << 21) |
	       (((imm >> 11) & 1u) << 20) |
	       (((imm >> 12) & 0xffu) << 12) |
	       (rd << 7) | 0x6fu;
}

static inline void jit_emit(struct jit_builder *builder, uint32_t instruction)
{
	if (builder->cursor >= builder->end) {
		builder->failed = true;
		return;
	}
	*builder->cursor++ = instruction;
}

static void jit_emit_constant(struct jit_builder *builder, uint32_t host_reg,
			      uint32_t value)
{
	int32_t low = (int32_t)(value << 20) >> 20;
	uint32_t upper = value - (uint32_t)low;

	if (upper == 0u) {
		jit_emit(builder, rv_i(0x13u, host_reg, 0u, RV_X0, low));
		return;
	}

	jit_emit(builder, (upper & 0xfffff000u) |
			  (host_reg << 7) | 0x37u);
	if (low != 0)
		jit_emit(builder, rv_i(0x13u, host_reg, 0u,
				       host_reg, low));
}

static bool jit_patch_branch(uint32_t *instruction, const uint32_t *target,
			     uint32_t funct3, uint32_t rs1, uint32_t rs2)
{
	ptrdiff_t byte_offset = (const uint8_t *)target -
		(const uint8_t *)instruction;

	if ((byte_offset & 1) != 0 || byte_offset < -4096 || byte_offset > 4094)
		return false;
	*instruction = rv_b(funct3, rs1, rs2, (int32_t)byte_offset);
	return true;
}

static bool jit_patch_jal(uint32_t *instruction, const uint32_t *target,
			  uint32_t rd)
{
	ptrdiff_t byte_offset = (const uint8_t *)target -
		(const uint8_t *)instruction;

	if ((byte_offset & 1) != 0 || byte_offset < -1048576 ||
	    byte_offset > 1048574)
		return false;
	*instruction = rv_j(rd, (int32_t)byte_offset);
	return true;
}

static void jit_emit_load_host(struct jit_builder *builder, uint32_t host_reg,
			       uint32_t base, size_t offset)
{
	if (offset > 2047u) {
		builder->failed = true;
		return;
	}
	jit_emit(builder, rv_i(0x03u, host_reg, 2u, base, (int32_t)offset));
}

static void jit_emit_store_host(struct jit_builder *builder, uint32_t base,
				uint32_t host_reg, size_t offset)
{
	if (offset > 2047u) {
		builder->failed = true;
		return;
	}
	jit_emit(builder, rv_s(0x23u, 2u, base, host_reg, (int32_t)offset));
}

static enum jit_instruction_kind jit_instruction_kind(uint32_t instruction);

#ifdef ESP_PLATFORM
/* Instruction and data fetches share the ESP32-P4's L2 cache, so new code
 * only has to leave L1D for L2 and old lines L1I; esp_cache_msync also writes
 * L2 back to PSRAM and evicts the code from L2, so its first run refetches
 * it.  Enabled once jit_check_light_publish has seen it work. */
static bool jit_light_publish;
#endif

static bool jit_sync_code(const uint32_t *start, size_t published_bytes)
{
	esp_err_t result;
#if CONFIG_RV32_HOST_PERF_STATS
	uint32_t cycle_start = jit_profile_cycle_count();
#endif

#ifdef ESP_PLATFORM
	if (jit_light_publish) {
		esp_cache_sync_ops_enter_critical_section();
		cache_ll_l1_writeback_dcache_addr(CACHE_LL_ID_ALL,
			(uint32_t)(uintptr_t)start, published_bytes);
		cache_ll_l1_invalidate_icache_addr(CACHE_LL_ID_ALL,
			(uint32_t)(uintptr_t)start, published_bytes);
		esp_cache_sync_ops_exit_critical_section();
		__atomic_thread_fence(__ATOMIC_SEQ_CST);
		__asm__ volatile("fence.i" ::: "memory");
#if CONFIG_RV32_HOST_PERF_STATS
		jit_stats.publish_cycles +=
			(uint32_t)(jit_profile_cycle_count() - cycle_start);
#endif
		return true;
	}
#endif
	/* fence.i orders fetches, but the ESP32-P4 also needs the dirty data line
	 * published and any old instruction line invalidated explicitly. */
	result = esp_cache_msync((void *)start, published_bytes,
				 ESP_CACHE_MSYNC_FLAG_DIR_C2M |
				 ESP_CACHE_MSYNC_FLAG_TYPE_DATA |
				 ESP_CACHE_MSYNC_FLAG_INVALIDATE);
	if (result != ESP_OK)
		return false;

	result = esp_cache_msync((void *)start, published_bytes,
				 ESP_CACHE_MSYNC_FLAG_DIR_M2C |
				 ESP_CACHE_MSYNC_FLAG_TYPE_INST);
	if (result != ESP_OK)
		return false;

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
#ifndef RV32_NATIVE_JIT_TEST_NO_FENCE_I
	__asm__ volatile("fence.i" ::: "memory");
#endif
#if CONFIG_RV32_HOST_PERF_STATS
	jit_stats.publish_cycles += (uint32_t)(jit_profile_cycle_count() - cycle_start);
#endif
	return true;
}

/* Every zero in generated code is an unpatched placeholder and would raise
 * an illegal-instruction exception.  jit_codegen rejects such a region
 * before it can be installed; the slot patches applied at install only
 * produce valid instructions. */
static bool jit_code_valid(const uint32_t *code, size_t used_words)
{
	for (size_t i = 0; i < used_words; i++) {
		if (code[i] == 0u ||
		    jit_instruction_kind(code[i]) == JIT_INSN_UNSUPPORTED)
			return false;
	}
	return true;
}

static uint32_t *jit_entry_code(uint32_t slot)
{
	return jit_code_arena +
		(uint32_t)jit_entries[slot].code_unit * JIT_CODE_UNIT_WORDS;
}

/* The memory-site table at the end of slot's arena units (jit_install_code),
 * so it is replaced together with the code that indexes it. */
static const struct rv32_native_jit_memory_site *jit_slot_sites(uint32_t slot)
{
	const struct jit_cache_entry *entry = &jit_entries[slot];

	return (const struct rv32_native_jit_memory_site *)(jit_code_arena +
		((uint32_t)entry->code_unit + entry->code_units) *
		JIT_CODE_UNIT_WORDS) - entry->memory_site_count;
}

static bool jit_code_ranges_overlap(uint32_t first_start, uint32_t first_units,
				    uint32_t second_start,
				    uint32_t second_units)
{
	return first_start < second_start + second_units &&
		second_start < first_start + first_units;
}

static uint32_t jit_hot_code_unit(const struct jit_hot_entry *hot)
{
	return ((uint32_t)hot->location & JIT_HOT_LOCATION_UNIT_MASK) - 1u;
}

static uint32_t jit_hot_code_units(const struct jit_hot_entry *hot)
{
	return ((uint32_t)hot->location >> JIT_HOT_LOCATION_UNIT_BITS) + 1u;
}

static uint16_t jit_hot_location(uint32_t code_unit, uint32_t code_units)
{
	return (uint16_t)(((code_units - 1u) << JIT_HOT_LOCATION_UNIT_BITS) |
			  (code_unit + 1u));
}

static bool jit_hot_range_is_free(uint32_t start, uint32_t units)
{
	if (units == 0u || start + units > jit_hot_code_arena_units)
		return false;
	for (uint32_t unit = start; unit < start + units; unit++) {
		if (jit_hot_code_occupancy[unit >> 5] &
		    (UINT32_C(1) << (unit & 31u)))
			return false;
	}
	return true;
}

static void jit_hot_mark_range(uint32_t start, uint32_t units, bool occupied)
{
	for (uint32_t unit = start; unit < start + units; unit++) {
		uint32_t *word = &jit_hot_code_occupancy[unit >> 5];
		uint32_t bit = UINT32_C(1) << (unit & 31u);

		if (occupied)
			*word |= bit;
		else
			*word &= ~bit;
	}
}

/* Promotion is a cold event, so a bounded line-granularity scan is cheaper
 * than burdening every native dispatch with allocator metadata. Search from
 * the ring head first to preserve locality, then reuse holes before evicting. */
static bool jit_hot_find_free_range(uint32_t units, uint32_t *start)
{
	uint32_t head = jit_hot_code_head;

	if (units == 0u || units > jit_hot_code_arena_units)
		return false;
	for (uint32_t candidate = head;
	     candidate + units <= jit_hot_code_arena_units; candidate++) {
		if (jit_hot_range_is_free(candidate, units)) {
			*start = candidate;
			return true;
		}
	}
	for (uint32_t candidate = 0u;
	     candidate < head && candidate + units <= jit_hot_code_arena_units;
	     candidate++) {
		if (jit_hot_range_is_free(candidate, units)) {
			*start = candidate;
			return true;
		}
	}
	return false;
}

static uint32_t *jit_hot_entry_code(uint32_t slot)
{
	return jit_hot_code_arena +
		jit_hot_code_unit(&jit_hot_entries[slot]) * JIT_CODE_UNIT_WORDS;
}

static uint32_t *jit_current_entry_code(uint32_t slot)
{
	return jit_hot_entries[slot].location != 0u ?
		jit_hot_entry_code(slot) : jit_entry_code(slot);
}

static uint32_t jit_link_target_pointer(uint32_t packed_target)
{
	return packed_target & ~JIT_LINK_TARGET_FLAG_MASK;
}

static uint32_t jit_pack_link_target(
	uint32_t slot, const struct jit_cache_entry *entry)
{
	uintptr_t code = (uintptr_t)jit_current_entry_code(slot);

	/* Generated code jumps to the chain entry of this address; the target
	 * checks the budget and sets up its own loop. */
	if (entry->state != JIT_ENTRY_READY ||
	    entry->guest_instruction_count == 0u ||
	    (code & JIT_LINK_TARGET_FLAG_MASK) != 0u || code > UINT32_MAX)
		return 0u;
	return (uint32_t)code;
}

static void jit_clear_link(struct jit_link_entry *link)
{
	link->packed_target = 0u;
	link->expected_next_pc = UINT32_MAX;
}

static void jit_retarget_links(uint32_t slot, const uint32_t *old_code,
			       const uint32_t *new_code)
{
	uint32_t old_pointer = (uint32_t)(uintptr_t)old_code;
	uint32_t new_pointer = (uint32_t)(uintptr_t)new_code;

	if (old_code == new_code || jit_link_targets[slot] == 0u)
		return;
	for (uint32_t privilege = 0; privilege <= JIT_MAX_PRIVILEGE; privilege++) {
		for (uint32_t index = 0; index < JIT_JUMP_CACHE_ENTRIES; index++) {
			struct jit_jump_entry *jump = &jit_jump_cache[privilege][index];

			if (jit_link_target_pointer(jump->packed_target) == old_pointer)
				jump->packed_target = new_pointer |
					(jump->packed_target & JIT_LINK_TARGET_FLAG_MASK);
		}
	}
	/* Link candidates are left alone, as in jit_clear_all_links. */
	for (uint32_t index = 0; index < JIT_CACHE_SLOTS; index++) {
		struct jit_link_entry *link = &jit_links[index];

		if (jit_link_target_pointer(link->packed_target) == old_pointer)
			link->packed_target = new_pointer |
				(link->packed_target & JIT_LINK_TARGET_FLAG_MASK);
	}
}

static void jit_clear_entry_links(uint32_t slot)
{
	uint32_t cold_pointer;
	uint32_t hot_pointer = 0u;

	if (jit_entries[slot].state != JIT_ENTRY_READY ||
	    jit_entries[slot].code_units == 0u) {
		jit_clear_link(&jit_links[slot]);
		if (jit_link_candidates != NULL)
			memset(&jit_link_candidates[slot], 0,
			       sizeof(jit_link_candidates[slot]));
		return;
	}
	cold_pointer = (uint32_t)(uintptr_t)jit_entry_code(slot);
	if (jit_hot_entries[slot].location != 0u)
		hot_pointer = (uint32_t)(uintptr_t)jit_hot_entry_code(slot);
	jit_clear_link(&jit_links[slot]);
	if (jit_link_candidates != NULL)
		memset(&jit_link_candidates[slot], 0,
		       sizeof(jit_link_candidates[slot]));
	if (jit_link_targets[slot] == 0u)
		return;
	jit_link_targets[slot] = 0u;
	for (uint32_t privilege = 0; privilege <= JIT_MAX_PRIVILEGE; privilege++) {
		for (uint32_t index = 0; index < JIT_JUMP_CACHE_ENTRIES; index++) {
			struct jit_jump_entry *jump = &jit_jump_cache[privilege][index];
			uint32_t target = jit_link_target_pointer(jump->packed_target);

			if (target == cold_pointer ||
			    (hot_pointer != 0u && target == hot_pointer)) {
				jump->guest_pc = UINT32_MAX;
				jump->packed_target = 0u;
			}
		}
	}
	/* Other slots' link candidates may still hold this code's pointer: as
	 * in jit_clear_all_links they only count confirmations, and a link is
	 * published with the target found at that moment, so the 24 KiB PSRAM
	 * scan per eviction is not needed. */
	for (uint32_t index = 0; index < JIT_CACHE_SLOTS; index++) {
		struct jit_link_entry *link = &jit_links[index];
		uint32_t target = jit_link_target_pointer(link->packed_target);

		if (target == cold_pointer || (hot_pointer != 0u &&
					       target == hot_pointer))
			jit_clear_link(link);
	}
}

/* Translation changes clear the links and the jump cache.  Link candidates
 * only count confirmations: a publication always uses the target found at
 * that moment, so they stay (rv32_native_jit_flush clears them), sparing
 * 24 KiB of PSRAM writes per context switch. */
static void jit_clear_all_links(void)
{
	memset(jit_jump_cache, 0xff, sizeof(jit_jump_cache));
	memset(jit_jump_misses, 0xff, sizeof(jit_jump_misses));
	memset(jit_link_targets, 0, sizeof(jit_link_targets));
	for (uint32_t index = 0; index < JIT_CACHE_SLOTS; index++)
		jit_clear_link(&jit_links[index]);
}

static void jit_forget_hot_code(uint32_t slot)
{
	struct jit_hot_entry *hot = &jit_hot_entries[slot];

	if (hot->location != 0u) {
		jit_hot_mark_range(jit_hot_code_unit(hot),
				   jit_hot_code_units(hot), false);
		if (jit_entries[slot].state == JIT_ENTRY_READY)
			jit_retarget_links(slot, jit_hot_entry_code(slot),
					   jit_entry_code(slot));
	}
	hot->location = 0u;
	hot->calls = 0u;
}

#if CONFIG_RV32_GUEST_PROFILE
static void jit_profile_fold_slot(uint32_t slot)
{
	uint32_t entries;

	if (rv32_profile_region_entries == NULL)
		return;
	entries = rv32_profile_region_entries[slot];
	if (entries == 0u)
		return;
	rv32_profile_region_entries[slot] = 0u;
	rv32_profile_fold_region(jit_entries[slot].physical_offset,
				 jit_entries[slot].guest_pc,
				 jit_entries[slot].guest_instruction_count, entries);
}

void rv32_native_jit_profile_fold_live(void)
{
	for (uint32_t slot = 0; slot < JIT_CACHE_SLOTS; slot++)
		jit_profile_fold_slot(slot);
}
#endif

static void jit_forget_entry_body(uint32_t slot);

static void jit_forget_entry(uint32_t slot)
{
#if CONFIG_RV32_HOST_PERF_STATS
	uint32_t start = jit_profile_cycle_count();

	jit_forget_entry_body(slot);
	jit_stats.forget_cycles += (uint32_t)(jit_profile_cycle_count() - start);
	jit_stats.forgets++;
#else
	jit_forget_entry_body(slot);
#endif
}

static void jit_forget_entry_body(uint32_t slot)
{
#if CONFIG_RV32_GUEST_PROFILE
	jit_profile_fold_slot(slot);
#endif
#if CONFIG_RV32_HOST_PERF_STATS
	jit_profile_discarded_guest_instructions +=
		jit_profile_guest_instructions[slot];
	jit_profile_guest_instructions[slot] = 0u;
#endif
	jit_clear_entry_links(slot);
	jit_forget_hot_code(slot);
	jit_entry_entered[slot] = 0u;
}

static void jit_observe_link(uint32_t source_slot, uint32_t next_pc,
			     uint32_t target_slot,
			     const struct jit_cache_entry *target)
{
	struct jit_link_entry *link;
	struct jit_link_candidate *candidate;
	uint32_t packed_target;

	if (jit_link_candidates == NULL || source_slot >= JIT_CACHE_SLOTS ||
	    source_slot == target_slot)
		return;
	packed_target = jit_pack_link_target(target_slot, target);
	if (packed_target == 0u)
		return;
	link = &jit_links[source_slot];
	candidate = &jit_link_candidates[source_slot];
	if (link->packed_target != 0u &&
	    link->expected_next_pc == next_pc) {
		/* Promotion and demotion can change only the host pointer. */
		jit_link_targets[target_slot] = 1u;
		link->packed_target = packed_target;
		memset(candidate, 0, sizeof(*candidate));
		return;
	}
	if (candidate->candidate_target == 0u ||
	    candidate->candidate_next_pc != next_pc) {
		candidate->candidate_target = packed_target;
		candidate->candidate_next_pc = next_pc;
		candidate->candidate_confirmations = 1u;
		return;
	}
	candidate->candidate_target = packed_target;
	if (candidate->candidate_confirmations < UINT8_MAX)
		candidate->candidate_confirmations++;
	if (candidate->candidate_confirmations < JIT_LINK_CONFIRMATIONS)
		return;
	/* Publish the target before its nonzero packed word.  Generated code runs
	 * on this same CPU, but release ordering also makes the invariant explicit. */
	link->expected_next_pc = next_pc;
	jit_link_targets[target_slot] = 1u;
	__atomic_store_n(&link->packed_target, packed_target, __ATOMIC_RELEASE);
	memset(candidate, 0, sizeof(*candidate));
}

/* Copy only repeatedly executed regions into scarce internal executable RAM.
 * Their authoritative PSRAM copy remains resident, so this small variable-size
 * ring can evict a promotion without invalidating or recompiling the region. */
static __attribute__((noinline)) bool jit_promote_hot_code(
	uint32_t slot, const struct jit_cache_entry *entry, bool allow_eviction)
{
	struct jit_hot_entry *hot = &jit_hot_entries[slot];
	uint32_t units = entry->code_units;
	uint32_t previous_head = jit_hot_code_head;
	uint32_t start;
	uint32_t *destination;
	bool wrapped;

	if (jit_hot_code_arena == NULL || units == 0u ||
	    units > jit_hot_code_arena_units)
		return false;
	if (!jit_hot_find_free_range(units, &start)) {
		uint32_t spared = 0u;

		if (!allow_eviction)
			return false;
		start = previous_head;
		if (start + units > jit_hot_code_arena_units)
			start = 0u;
		/* The tier replaces oldest first: it must follow the regions hot
		 * now, and a protected tier froze (clock6-clock8, 2026-10-20/22). */
		(void)spared;
	}
	wrapped = start < previous_head;

	for (uint32_t index = 0; index < JIT_CACHE_SLOTS; index++) {
		struct jit_hot_entry *resident = &jit_hot_entries[index];

		if (resident->location == 0u ||
		    !jit_code_ranges_overlap(start, units,
					     jit_hot_code_unit(resident),
					     jit_hot_code_units(resident)))
			continue;
		jit_forget_hot_code(index);
		jit_stats.hot_evictions++;
	}

	destination = jit_hot_code_arena + start * JIT_CODE_UNIT_WORDS;
	memcpy(destination, jit_entry_code(slot),
	       units * JIT_CODE_ALIGNMENT);
	for (uint32_t unit = start; unit < start + units; unit++)
		jit_hot_unit_owner[unit] = (uint16_t)(slot + 1u);
	if (!jit_sync_code(destination, units * JIT_CODE_ALIGNMENT))
		return false;

	hot->location = jit_hot_location(start, units);
	jit_hot_mark_range(start, units, true);
	jit_retarget_links(slot, jit_entry_code(slot), destination);
	jit_hot_code_head = (uint16_t)(start + units);
	if (jit_hot_code_head == jit_hot_code_arena_units) {
		jit_hot_code_head = 0u;
		wrapped = true;
	}
	if (wrapped)
		jit_stats.hot_arena_wraps++;
	jit_stats.hot_promotions++;
	jit_promotions_this_second++;
	return true;
}

static inline uint32_t *jit_dispatch_code(
	uint32_t slot, const struct jit_cache_entry *entry)
{
	struct jit_hot_entry *hot;
	uint32_t admit_threshold;
	uint32_t replace_threshold;

	if (jit_entry_entered[slot] != UINT16_MAX)
		jit_entry_entered[slot]++;
	if (__builtin_expect(jit_hot_code_arena == NULL, 0))
		return jit_entry_code(slot);
	hot = &jit_hot_entries[slot];
	if (__builtin_expect(hot->location != 0u, 1))
		return jit_hot_entry_code(slot);
	if (hot->calls == UINT16_MAX)
		return jit_entry_code(slot);

	admit_threshold = entry->loops_to_start ? JIT_HOT_LOOP_ADMIT_CALLS :
		JIT_HOT_GENERAL_ADMIT_CALLS;
	replace_threshold = entry->loops_to_start ? JIT_HOT_LOOP_REPLACE_CALLS :
		JIT_HOT_GENERAL_REPLACE_CALLS;
	hot->calls++;
	if (hot->calls == admit_threshold) {
		if (jit_promote_hot_code(slot, entry, false)) {
			jit_stats.hot_early_promotions++;
			return jit_hot_entry_code(slot);
		}
		jit_stats.hot_admission_deferrals++;
	}
	if (hot->calls < replace_threshold)
		return jit_entry_code(slot);
	if (jit_promote_hot_code(slot, entry, true)) {
		jit_stats.hot_pressure_promotions++;
		return jit_hot_entry_code(slot);
	}
	/* A permanent size/publication failure must not retry on every dispatch. */
	hot->calls = UINT16_MAX;
	return jit_entry_code(slot);
}

/* Install a relocatable region into one of two cache-line rings.  Backward
 * loops receive the larger protected partition: transient straight-line and
 * call/return regions can churn in their probationary partition without
 * overwriting reusable array, copy, paint, or decode loops.  Any metadata
 * whose bytes are about to be overwritten is invalidated before publication.
 * Compilation is cold, so a bounded directory scan is preferable to adding
 * allocator state to the hot lookup path. */
#ifdef ESP_PLATFORM
/* The emulator core's 1 kHz sampler: a PC inside a region's code, in the
 * arena or its hot copy, counts as an entry of that region. */
static bool IRAM_ATTR jit_sample_pc(gptimer_handle_t timer,
				    const gptimer_alarm_event_data_t *event,
				    void *context)
{
	uint32_t pc;
	uint32_t offset;
	uint16_t owner = 0u;

	(void)timer;
	(void)event;
	(void)context;
	__asm__ volatile("csrr %0, mepc" : "=r"(pc));
	offset = pc - (uint32_t)(uintptr_t)jit_code_arena;
	if (jit_unit_owner != NULL &&
	    offset < (uint32_t)jit_code_arena_units * JIT_CODE_ALIGNMENT) {
		owner = jit_unit_owner[offset / JIT_CODE_ALIGNMENT];
	} else {
		offset = pc - (uint32_t)(uintptr_t)jit_hot_code_arena;
		if (jit_hot_code_arena != NULL &&
		    offset < (uint32_t)jit_hot_code_arena_units * JIT_CODE_ALIGNMENT)
			owner = jit_hot_unit_owner[offset / JIT_CODE_ALIGNMENT];
	}
	if (owner != 0u && owner <= JIT_CACHE_SLOTS &&
	    jit_entry_entered[owner - 1u] != UINT16_MAX)
		jit_entry_entered[owner - 1u]++;
	return false;
}

static void jit_start_sampler(void)
{
	gptimer_handle_t timer = NULL;
	const gptimer_config_t config = {
		.clk_src = GPTIMER_CLK_SRC_DEFAULT,
		.direction = GPTIMER_COUNT_UP,
		.resolution_hz = 1000000u,
	};
	const gptimer_event_callbacks_t callbacks = {
		.on_alarm = jit_sample_pc,
	};
	const gptimer_alarm_config_t alarm = {
		.alarm_count = 1000u,
		.reload_count = 0u,
		.flags.auto_reload_on_alarm = true,
	};

	if (gptimer_new_timer(&config, &timer) != ESP_OK ||
	    gptimer_register_event_callbacks(timer, &callbacks, NULL) != ESP_OK ||
	    gptimer_enable(timer) != ESP_OK ||
	    gptimer_set_alarm_action(timer, &alarm) != ESP_OK ||
	    gptimer_start(timer) != ESP_OK)
		printf("WARNING: JIT sampler timer unavailable\n");
}
#endif

/* Once a second of host time, last second's promotion rate sets the bar. */
static void jit_promote_bar_tick(void)
{
#ifdef ESP_PLATFORM
	uint32_t now = esp_cpu_get_cycle_count();

	if ((int32_t)(now - jit_promote_bar_tick_at) < 0)
		return;
	jit_promote_bar_tick_at = now + 360u * 1000u * 1000u;
	if (jit_promotions_this_second > JIT_PROMOTE_RATE_HIGH &&
	    jit_promote_bar < JIT_PROMOTE_BAR_MAX)
		jit_promote_bar *= 2u;
	else if (jit_promotions_this_second < JIT_PROMOTE_RATE_LOW &&
		 jit_promote_bar > JIT_PROMOTE_BAR_MIN)
		jit_promote_bar /= 2u;
	jit_promotions_this_second = 0u;
#endif
}

/* Drops a resident region whose arena space new code is taking. */
static void jit_evict_resident(uint32_t index)
{
	struct jit_cache_entry *resident = &jit_entries[index];

	jit_forget_entry(index);
	resident->state = JIT_ENTRY_EMPTY;
	resident->code_units = 0u;
	resident->spans_pages = 0u;
	jit_stats.replacements++;
	jit_stats.arena_evictions++;
}

static bool jit_install_code(uint32_t slot, const uint32_t *code,
			     size_t used_words,
			     const struct rv32_native_jit_memory_site *sites,
			     uint32_t site_count, bool loops_to_start, bool kernel)
{
	struct jit_cache_entry *entry = &jit_entries[slot];
	uint32_t site_bytes = site_count * (uint32_t)sizeof(*sites);
	uint32_t units = ((uint32_t)used_words * sizeof(uint32_t) + site_bytes +
		JIT_CODE_ALIGNMENT - 1u) / JIT_CODE_ALIGNMENT;
	uint32_t partition_start = loops_to_start ? 0u : jit_loop_code_units;
	uint32_t partition_end = loops_to_start ? jit_loop_code_units :
		jit_code_arena_units;
	uint16_t *head = loops_to_start ? &jit_loop_code_head :
		&jit_regular_code_head;
	uint32_t start;
	uint32_t *destination;

#if JIT_KEEP_KERNEL
	/* User code is recompiled after every FENCE.I; in its own half of the
	 * ring it no longer pushes resident kernel code out. */
	{
		uint32_t middle = partition_start +
			(partition_end - partition_start) / 2u;

		if (kernel) {
			partition_end = middle;
			head = loops_to_start ? &jit_kernel_loop_code_head :
				&jit_kernel_regular_code_head;
		} else {
			partition_start = middle;
		}
	}
#else
	(void)kernel;
#endif
#if JIT_ARENA_SINGLE_RING
	partition_start = 0u;
	partition_end = jit_code_arena_units;
	head = &jit_code_head;
#endif
	jit_promote_bar_tick();
	start = *head;

	if (units == 0u || units > partition_end - partition_start)
		return false;
	if (start < partition_start || start + units > partition_end) {
		start = partition_start;
		jit_stats.arena_wraps++;
	}

	if (jit_unit_owner != NULL) {
		uint32_t spared = 0u;
		bool moved = true;

		/* Second chance: a resident region entered since the hand last
		 * passed keeps its place once and the hand moves past it; after
		 * JIT_ARENA_SPARES of them the next ones go regardless. */
		while (moved && spared < JIT_ARENA_SPARES) {
			uint32_t first = start - partition_start >=
				JIT_MAX_REGION_UNITS - 1u ?
				start - (JIT_MAX_REGION_UNITS - 1u) : partition_start;

			moved = false;
			for (uint32_t unit = first; unit < start + units; unit++) {
				uint32_t owner = jit_unit_owner[unit];
				const struct jit_cache_entry *resident;

				if (owner == 0u)
					continue;
				resident = &jit_entries[owner - 1u];
				if (owner - 1u == slot ||
				    resident->state != JIT_ENTRY_READY ||
				    resident->code_unit != unit ||
				    jit_entry_entered[owner - 1u] == 0u ||
				    !jit_code_ranges_overlap(start, units,
							     resident->code_unit,
							     resident->code_units))
					continue;
				/* A region this hot keeps running from PSRAM unless
				 * the hand promotes it: link entries never reach the
				 * dispatch counts of jit_dispatch_code. */
				if (jit_entry_entered[owner - 1u] >= jit_promote_bar &&
				    jit_hot_entries[owner - 1u].location == 0u &&
				    jit_promote_hot_code(owner - 1u, resident, true))
					jit_stats.hot_hand_promotions++;
				jit_entry_entered[owner - 1u] = 0u;
				jit_stats.arena_spares++;
				spared++;
				start = unit + resident->code_units;
				if (start + units > partition_end) {
					start = partition_start;
					jit_stats.arena_wraps++;
				}
				moved = true;
				break;
			}
		}
		{
		uint32_t first = start - partition_start >= JIT_MAX_REGION_UNITS - 1u ?
			start - (JIT_MAX_REGION_UNITS - 1u) : partition_start;

		for (uint32_t unit = first; unit < start + units; unit++) {
			uint32_t owner = jit_unit_owner[unit];
			const struct jit_cache_entry *resident;

			if (owner == 0u)
				continue;
			resident = &jit_entries[owner - 1u];
			if (owner - 1u == slot ||
			    resident->state != JIT_ENTRY_READY ||
			    resident->code_unit != unit ||
			    !jit_code_ranges_overlap(start, units,
						     resident->code_unit,
						     resident->code_units))
				continue;
			jit_evict_resident(owner - 1u);
		}
		}
	} else {
		for (uint32_t index = 0; index < JIT_CACHE_SLOTS; index++) {
			const struct jit_cache_entry *resident = &jit_entries[index];

			if (index == slot || resident->state != JIT_ENTRY_READY ||
			    !jit_code_ranges_overlap(start, units,
						     resident->code_unit,
						     resident->code_units))
				continue;
			jit_evict_resident(index);
		}
	}

	destination = jit_code_arena + start * JIT_CODE_UNIT_WORDS;
	memset(destination, 0, units * JIT_CODE_ALIGNMENT);
	memcpy(destination, code, used_words * sizeof(uint32_t));
	memcpy((uint8_t *)(destination + units * JIT_CODE_UNIT_WORDS) -
	       site_bytes, sites, site_bytes);
	*head = (uint16_t)(start + units);
	if (*head == partition_end) {
		*head = (uint16_t)partition_start;
		jit_stats.arena_wraps++;
	}
	if (!jit_sync_code(destination, units * JIT_CODE_ALIGNMENT))
		return false;

	entry->code_unit = (uint16_t)start;
	entry->code_units = (uint8_t)units;
	entry->memory_site_count = (uint8_t)site_count;
	jit_entry_entered[slot] = 0u;
	if (jit_unit_owner != NULL) {
		/* Every unit, so the sampler maps any PC inside the region. */
		for (uint32_t unit = start; unit < start + units; unit++)
			jit_unit_owner[unit] = (uint16_t)(slot + 1u);
	}
	return true;
}

static bool jit_valid_op_imm(uint32_t instruction)
{
	uint32_t funct3 = (instruction >> 12) & 7u;
	uint32_t funct7 = instruction >> 25;

	if (funct3 == 1u)
		return funct7 == 0u;
	if (funct3 == 5u)
		return funct7 == 0u || funct7 == 0x20u;
	return true;
}

static bool jit_valid_op(uint32_t instruction)
{
	uint32_t funct3 = (instruction >> 12) & 7u;
	uint32_t funct7 = instruction >> 25;

	if (funct7 == 0u || funct7 == 1u)
		return true;
	return funct7 == 0x20u && (funct3 == 0u || funct3 == 5u);
}

static struct rv32_native_jit_csr_layout jit_csr_layout;
static bool jit_csr_layout_valid;

void rv32_native_jit_set_csr_layout(
	const struct rv32_native_jit_csr_layout *layout)
{
	const int32_t *offsets = (const int32_t *)layout;

	jit_csr_layout_valid = false;
	if (layout == NULL)
		return;
	for (size_t i = 0; i < sizeof(*layout) / sizeof(int32_t); i++) {
		if (offsets[i] < -2048 || offsets[i] > 2047 || (offsets[i] & 3))
			return;
	}
	jit_csr_layout = *layout;
	jit_csr_layout_valid = true;
}

/* Guest state offset of a supervisor CSR the generated code may access, or
 * INT32_MIN.  satp writes change translations and stay interpreted. */
static int32_t jit_csr_offset(uint32_t csrno, bool write)
{
	if (!jit_csr_layout_valid)
		return INT32_MIN;
	switch (csrno) {
	case 0x100u: return jit_csr_layout.sstatus;
	case 0x104u: return jit_csr_layout.sie;
	case 0x105u: return jit_csr_layout.stvec;
	case 0x140u: return jit_csr_layout.sscratch;
	case 0x141u: return jit_csr_layout.sepc;
	case 0x142u: return jit_csr_layout.scause;
	case 0x143u: return jit_csr_layout.stval;
	case 0x144u: return jit_csr_layout.sip;
	case 0x180u: return write ? INT32_MIN : jit_csr_layout.satp;
	default: return INT32_MIN;
	}
}

/* CSRRW/CSRRWI always write; the set/clear forms write only with a nonzero
 * source, exactly as the interpreter's write-back is a no-op otherwise. */
static bool jit_csr_writes(uint32_t instruction)
{
	uint32_t funct3 = (instruction >> 12) & 7u;
	uint32_t rs1 = (instruction >> 15) & 0x1fu;

	return funct3 == 1u || funct3 == 5u || rs1 != 0u;
}

/* A write that can change the memory context (SUM/MXR via a register
 * operand to sstatus) or newly deliver a supervisor interrupt (set bits in
 * sstatus, sie or sip) may leave the region after the write. */
static bool jit_csr_may_exit(uint32_t instruction)
{
	uint32_t funct3 = (instruction >> 12) & 7u;
	uint32_t csrno = instruction >> 20;

	if (!jit_csr_writes(instruction))
		return false;
	if (csrno == 0x100u && funct3 <= 3u)
		return true;
	if (csrno == 0x100u || csrno == 0x104u || csrno == 0x144u)
		return funct3 == 1u || funct3 == 2u || funct3 == 5u ||
			funct3 == 6u;
	return false;
}

/* RV32A word operations.  LR/SC keep their reservation in the guest state
 * word the CSR layout names, so they compile only once that is known. */
static bool jit_valid_amo(uint32_t instruction)
{
	switch (instruction >> 27) {
	case 0x00u: case 0x01u: case 0x04u: case 0x08u: case 0x0cu:
	case 0x10u: case 0x14u: case 0x18u: case 0x1cu:
		return true;
	case 0x02u: /* LR.W */
		return jit_csr_layout_valid && ((instruction >> 20) & 0x1fu) == 0u;
	case 0x03u: /* SC.W */
		return jit_csr_layout_valid;
	default:
		return false;
	}
}

static enum jit_instruction_kind jit_instruction_kind(uint32_t instruction)
{
	uint32_t opcode = instruction & 0x7fu;
	uint32_t funct3 = (instruction >> 12) & 7u;

	if ((instruction & 3u) != 3u)
		return JIT_INSN_UNSUPPORTED;

	switch (opcode) {
	case 0x17u: /* AUIPC */
	case 0x37u: /* LUI */
		return JIT_INSN_INTEGER;
	case 0x13u: /* OP-IMM */
		return jit_valid_op_imm(instruction) ? JIT_INSN_INTEGER :
			JIT_INSN_UNSUPPORTED;
	case 0x33u: /* OP / RV32M */
		return jit_valid_op(instruction) ? JIT_INSN_INTEGER :
			JIT_INSN_UNSUPPORTED;
	case 0x03u: /* LOAD */
		return (funct3 <= 2u || funct3 == 4u || funct3 == 5u) ?
			JIT_INSN_LOAD : JIT_INSN_UNSUPPORTED;
	case 0x23u: /* STORE */
		return funct3 <= 2u ? JIT_INSN_STORE : JIT_INSN_UNSUPPORTED;
	case 0x63u: /* BRANCH */
		return (funct3 <= 1u || (funct3 >= 4u && funct3 <= 7u)) ?
			JIT_INSN_BRANCH : JIT_INSN_UNSUPPORTED;
	case 0x6fu:
		return JIT_INSN_JAL;
	case 0x67u:
		return funct3 == 0u ? JIT_INSN_JALR : JIT_INSN_UNSUPPORTED;
	case 0x2fu: /* RV32A word operations */
		return (funct3 == 2u && jit_valid_amo(instruction)) ?
			JIT_INSN_AMO : JIT_INSN_UNSUPPORTED;
	case 0x0fu: /* Plain FENCE needs no host action; FENCE.I and CBOs do. */
		return funct3 == 0u ? JIT_INSN_NOP : JIT_INSN_UNSUPPORTED;
	case 0x73u: /* Zicsr on the supervisor CSRs; ecall/xRET/WFI/fences stay. */
		return (funct3 != 0u && funct3 != 4u &&
			jit_csr_offset(instruction >> 20,
				       jit_csr_writes(instruction)) != INT32_MIN) ?
			JIT_INSN_CSR : JIT_INSN_UNSUPPORTED;
	default:
		return JIT_INSN_UNSUPPORTED;
	}
}

#if CONFIG_RV32_HOST_PERF_STATS
/* Instruction class of an unsupported region start/cut for the perf report:
 * 0 ecall/ebreak, 1 xRET, 2 WFI, 3 SFENCE.VMA, 4 FENCE.I, 5 CBO, 6 time/cycle
 * CSR, 7 satp write, 8 other CSR, 9 LR/SC, 10 other RV32A, 11 anything else. */
static uint32_t jit_unsupported_class(uint32_t instruction)
{
	uint32_t opcode = instruction & 0x7fu;
	uint32_t funct3 = (instruction >> 12) & 7u;
	uint32_t csrno = instruction >> 20;

	if (opcode == 0x73u) {
		if (funct3 == 0u) {
			if (csrno == 0u || csrno == 1u)
				return 0u;
			if (csrno == 0x102u || csrno == 0x302u)
				return 1u;
			if (csrno == 0x105u)
				return 2u;
			if ((csrno & 0xfe0u) == 0x120u)
				return 3u;
			return 11u;
		}
		if (csrno >= 0xc00u && csrno <= 0xc81u)
			return 6u;
		if (csrno == 0x180u)
			return 7u;
		return 8u;
	}
	if (opcode == 0x0fu)
		return funct3 == 1u ? 4u : 5u;
	if (opcode == 0x2fu) {
		uint32_t funct5 = instruction >> 27;

		return (funct5 == 2u || funct5 == 3u) ? 9u : 10u;
	}
	return 11u;
}
#endif

static bool jit_instruction_terminal(enum jit_instruction_kind kind)
{
	return kind == JIT_INSN_BRANCH || kind == JIT_INSN_JAL ||
	       kind == JIT_INSN_JALR;
}

static int32_t jit_i_immediate(uint32_t instruction)
{
	return (int32_t)instruction >> 20;
}

static int32_t jit_s_immediate(uint32_t instruction)
{
	uint32_t immediate = ((instruction >> 7) & 0x1fu) |
		((instruction >> 25) << 5);

	return (int32_t)(immediate << 20) >> 20;
}

static int32_t jit_b_immediate(uint32_t instruction)
{
	uint32_t immediate = ((instruction >> 31) << 12) |
		(((instruction >> 7) & 1u) << 11) |
		(((instruction >> 25) & 0x3fu) << 5) |
		(((instruction >> 8) & 0x0fu) << 1);

	return (int32_t)(immediate << 19) >> 19;
}

static int32_t jit_j_immediate(uint32_t instruction)
{
	uint32_t immediate = ((instruction >> 31) << 20) |
		(((instruction >> 12) & 0xffu) << 12) |
		(((instruction >> 20) & 1u) << 11) |
		(((instruction >> 21) & 0x3ffu) << 1);

	return (int32_t)(immediate << 11) >> 11;
}

static uint32_t jit_memory_width(uint32_t funct3, bool load)
{
	if (funct3 == 0u || (load && funct3 == 4u))
		return 1u;
	if (funct3 == 1u || (load && funct3 == 5u))
		return 2u;
	return funct3 == 2u ? 4u : 0u;
}

static bool jit_group_crosses_hint_page(const struct jit_group_hint *hint,
	uint32_t base, int32_t min_offset, int32_t max_end)
{
	if (hint->entry_values == NULL ||
	    (hint->invariant_registers & (1u << base)) == 0u)
		return false;
	uint32_t value = base == 0u ? 0u : hint->entry_values[base];
	return ((value + (uint32_t)min_offset) >> 12) !=
	       ((value + (uint32_t)max_end) >> 12);
}

/* Find adjacent accesses whose effective addresses are all base + immediate.
 * A single page/alignment guard can cover the whole group, but only while no
 * load overwrites the shared base register. */
static struct jit_memory_group jit_find_memory_group(
	const uint32_t *instructions, const enum jit_instruction_kind *kinds,
	const bool *leaders, uint32_t total, uint32_t first, bool store,
	const struct jit_group_hint *hint)
{
	struct jit_memory_group group = {
		.min_offset = INT32_MAX,
		.max_end = INT32_MIN,
	};
	enum jit_instruction_kind wanted = store ? JIT_INSN_STORE :
		JIT_INSN_LOAD;
	uint32_t base = (instructions[first] >> 15) & 0x1fu;
	bool base_overwritten = false;

	for (uint32_t i = first; i < total; i++) {
		uint32_t instruction = instructions[i];
		uint32_t funct3;
		uint32_t width;
		int32_t offset;
		int32_t end;

		/* A native branch may enter any region leader.  Do not combine that
		 * instruction with its lexical predecessor, because it needs a distinct
		 * native landing address and dynamic-instruction checkpoint. */
		if ((i != first && leaders[i]) || kinds[i] != wanted ||
		    ((instruction >> 15) & 0x1fu) != base || base_overwritten)
			break;
		funct3 = (instruction >> 12) & 7u;
		width = jit_memory_width(funct3, !store);
		offset = store ? jit_s_immediate(instruction) :
			jit_i_immediate(instruction);
		end = offset + (int32_t)width - 1;
		/* Keeping both range endpoints in ADDI immediates makes the guard
		 * smaller than repeating a page-cache lookup. */
		if (width == 0u || end > 2047 ||
		    (((uint32_t)offset & (width - 1u)) != 0u))
			break;
		/* Loads stop at a page crossing; stores have a two-page path. */
		if (!store && group.count != 0u &&
		    jit_group_crosses_hint_page(hint, base,
			offset < group.min_offset ? offset : group.min_offset,
			end > group.max_end ? end : group.max_end))
			break;

		if (offset < group.min_offset)
			group.min_offset = offset;
		if (end > group.max_end)
			group.max_end = end;
		group.alignment_mask |= width - 1u;
		group.count++;
		if (!store && ((instruction >> 7) & 0x1fu) == base)
			base_overwritten = true;
	}

	if (group.count < 2u ||
	    group.max_end - group.min_offset >= 4096) {
		group.count = 1u;
		group.min_offset = 0;
		group.max_end = 0;
		group.alignment_mask = 0u;
	}
	return group;
}

static bool jit_instruction_writes_register(
	uint32_t instruction, enum jit_instruction_kind kind, uint32_t reg)
{
	if (reg == 0u)
		return false;
	switch (kind) {
	case JIT_INSN_INTEGER:
	case JIT_INSN_LOAD:
	case JIT_INSN_JAL:
	case JIT_INSN_JALR:
	case JIT_INSN_AMO:
	case JIT_INSN_CSR:
		return ((instruction >> 7) & 0x1fu) == reg;
	default:
		return false;
	}
}

static bool jit_region_writes_register(
	const uint32_t *instructions, const enum jit_instruction_kind *kinds,
	uint32_t count, uint32_t reg)
{
	for (uint32_t i = 0; i < count; i++) {
		if (jit_instruction_writes_register(instructions[i], kinds[i], reg))
			return true;
	}
	return false;
}

/* A local page tag starts invalid on every native entry. Once its refill
 * validates alignment, a fixed-offset access stays aligned across back edges
 * if every write to its base preserves the low address bits. Only accept
 * self-ADDI by a multiple of the access width (or an unchanged base); arbitrary
 * arithmetic, loads into the base and changes on any branch path retain the
 * per-access check. This proof does not validate translation or permissions. */
static bool jit_loop_preserves_alignment(
	const uint32_t *instructions, const enum jit_instruction_kind *kinds,
	uint32_t loop_start, uint32_t count, uint32_t base, uint32_t width)
{
	if (loop_start >= count || width <= 1u)
		return false;
	for (uint32_t i = loop_start; i < count; i++) {
		uint32_t instruction = instructions[i];

		if (!jit_instruction_writes_register(instruction, kinds[i], base))
			continue;
		if ((instruction & 0x707fu) != 0x13u ||
		    ((instruction >> 15) & 0x1fu) != base ||
		    ((uint32_t)jit_i_immediate(instruction) & (width - 1u)) != 0u)
			return false;
	}
	return true;
}

/* Keep one high-value, invariant-base memory stream translated in a4 for the
 * duration of a straight-line region segment.  Other integer and single
 * memory operations do not use a4, so accesses separated by useful work can
 * share the same checked page guard without being reordered. */
static struct jit_persistent_memory_group jit_find_persistent_memory_group(
	const uint32_t *instructions, const enum jit_instruction_kind *kinds,
	const bool *leaders, uint32_t total, const struct jit_group_hint *hint)
{
	struct jit_persistent_memory_group best = { 0 };

	for (uint32_t first = 0; first < total; first++) {
		enum jit_instruction_kind wanted = kinds[first];
		struct jit_persistent_memory_group candidate = {
			.memory = {
				.min_offset = INT32_MAX,
				.max_end = INT32_MIN,
			},
			.first = first,
			.last = first,
			.base = (instructions[first] >> 15) & 0x1fu,
			.store = wanted == JIT_INSN_STORE,
		};

		if (wanted != JIT_INSN_LOAD && wanted != JIT_INSN_STORE)
			continue;
		for (uint32_t i = first; i < total; i++) {
			uint32_t instruction = instructions[i];
			enum jit_instruction_kind kind = kinds[i];

			if ((i != first && leaders[i]) ||
			    kind == JIT_INSN_BRANCH || kind == JIT_INSN_JAL ||
			    kind == JIT_INSN_JALR)
				break;
			if (kind == wanted &&
			    ((instruction >> 15) & 0x1fu) == candidate.base) {
				uint32_t funct3 = (instruction >> 12) & 7u;
				uint32_t width = jit_memory_width(
					funct3, wanted == JIT_INSN_LOAD);
				int32_t offset = candidate.store ?
					jit_s_immediate(instruction) :
					jit_i_immediate(instruction);
				int32_t end = offset + (int32_t)width - 1;

				if (width != 0u && end <= 2047 &&
				    ((uint32_t)offset & (width - 1u)) == 0u) {
					/* End before a page-crossing load: members are
					 * the contiguous first..last range. */
					if (!candidate.store && candidate.memory.count != 0u &&
					    jit_group_crosses_hint_page(hint, candidate.base,
						offset < candidate.memory.min_offset ?
							offset : candidate.memory.min_offset,
						end > candidate.memory.max_end ?
							end : candidate.memory.max_end))
						break;
					if (offset < candidate.memory.min_offset)
						candidate.memory.min_offset = offset;
					if (end > candidate.memory.max_end)
						candidate.memory.max_end = end;
					candidate.memory.alignment_mask |= width - 1u;
					candidate.memory.count++;
					candidate.total_bytes += width;
					candidate.last = i;
				} else if (i == first) {
					/* The guard is emitted at the first member;
					 * without one at first, later members would
					 * use a stale a4. */
					break;
				}
			}
			if (jit_instruction_writes_register(
				    instruction, kind, candidate.base))
				break;
		}
		if (candidate.memory.count < 2u ||
		    candidate.last - candidate.first + 1u ==
			    candidate.memory.count ||
		    candidate.memory.max_end - candidate.memory.min_offset >= 4096)
			continue;
		candidate.valid = true;
		if (!best.valid || candidate.memory.count > best.memory.count ||
		    (candidate.memory.count == best.memory.count &&
		     candidate.last - candidate.first < best.last - best.first))
			best = candidate;
	}
	return best;
}

static bool jit_persistent_memory_access(
	const struct jit_persistent_memory_group *group, uint32_t index,
	uint32_t instruction, enum jit_instruction_kind kind)
{
	uint32_t width;
	int32_t offset;
	int32_t end;

	if (!group->valid || index < group->first || index > group->last ||
	    kind != (group->store ? JIT_INSN_STORE : JIT_INSN_LOAD) ||
	    ((instruction >> 15) & 0x1fu) != group->base)
		return false;
	width = jit_memory_width((instruction >> 12) & 7u, !group->store);
	offset = group->store ? jit_s_immediate(instruction) :
		jit_i_immediate(instruction);
	end = offset + (int32_t)width - 1;
	return width != 0u && end <= 2047 &&
			((uint32_t)offset & (width - 1u)) == 0u;
}

/* Repeated native regions can retain up to two useful read translations in
 * ABI-preserved registers.  The first access refreshes each pair through the
 * ordinary shared page cache; subsequent iterations need only a page
 * comparison and host-bias add.  Consecutive and persistent groups already
 * amortize their lookup, so leave those paths unchanged. */
static uint32_t jit_find_loop_read_caches(
	const uint32_t *instructions, const enum jit_instruction_kind *kinds,
	const bool *leaders, uint32_t count,
	const struct jit_persistent_memory_group *persistent,
	bool preserve_persistent_base,
	struct jit_loop_read_cache caches[JIT_MAX_LOOP_READ_CACHES],
	const struct jit_group_hint *hint)
{
	static const uint8_t tag_registers[JIT_MAX_LOOP_READ_CACHES] = {
		RV_S0, RV_S10,
	};
	static const uint8_t bias_registers[JIT_MAX_LOOP_READ_CACHES] = {
		RV_S1, RV_S11,
	};
	uint32_t found = 0u;

	memset(caches, 0,
	       sizeof(struct jit_loop_read_cache) * JIT_MAX_LOOP_READ_CACHES);
	for (uint32_t i = 0; i < count && found < JIT_MAX_LOOP_READ_CACHES;
	     i++) {
		struct jit_memory_group group;

		if (kinds[i] != JIT_INSN_LOAD ||
		    jit_persistent_memory_access(
			    persistent, i, instructions[i], kinds[i]))
			continue;
		if (!preserve_persistent_base) {
			group = jit_find_memory_group(
				instructions, kinds, leaders, count, i, false, hint);
			if (group.count > 1u) {
				i += group.count - 1u;
				continue;
			}
		}
		caches[found].instruction_index = (uint8_t)i;
		caches[found].tag_register = tag_registers[found];
		caches[found].bias_register = bias_registers[found];
		caches[found].valid = 1u;
		found++;
	}
	return found;
}

static const struct jit_loop_read_cache *jit_loop_read_cache_for(
	const struct jit_loop_read_cache caches[JIT_MAX_LOOP_READ_CACHES],
	uint32_t count, uint32_t instruction_index)
{
	for (uint32_t i = 0; i < count; i++) {
		if (caches[i].valid &&
		    caches[i].instruction_index == instruction_index)
			return &caches[i];
	}
	return NULL;
}

static struct jit_loop_write_cache jit_find_loop_write_cache(
	const uint32_t *instructions, const enum jit_instruction_kind *kinds,
	const bool *leaders, uint32_t count,
	const struct jit_persistent_memory_group *persistent,
	bool persistent_guard_hoisted, const struct jit_group_hint *hint)
{
	struct jit_loop_write_cache cache = { 0 };

	/* A non-hoisted persistent stream reloads a5 inside the loop.  Otherwise
	 * reserve it as the local write bias, but do not dismantle an already
	 * efficient consecutive store group merely to cache its first operation. */
	if (persistent->valid && !persistent_guard_hoisted)
		return cache;
	if (!persistent_guard_hoisted) {
		for (uint32_t i = 0; i < count; i++) {
			bool store = kinds[i] == JIT_INSN_STORE;
			struct jit_memory_group group;

			if (!store && kinds[i] != JIT_INSN_LOAD)
				continue;
			group = jit_find_memory_group(
				instructions, kinds, leaders, count, i, store, hint);
			if (group.count > 1u)
				return cache;
		}
	}
	for (uint32_t i = 0; i < count; i++) {
		struct jit_memory_group group;

		if (kinds[i] != JIT_INSN_STORE ||
		    jit_persistent_memory_access(
			    persistent, i, instructions[i], kinds[i]))
			continue;
		group = jit_find_memory_group(
			instructions, kinds, leaders, count, i, true, hint);
		if (group.count > 1u) {
			i += group.count - 1u;
			continue;
		}
		cache.instruction_index = (uint8_t)i;
		cache.valid = 1u;
		break;
	}
	return cache;
}

static const struct jit_loop_write_cache *jit_loop_write_cache_for(
	const struct jit_loop_write_cache *cache, uint32_t instruction_index)
{
	return cache->valid && cache->instruction_index == instruction_index ?
		cache : NULL;
}

static void jit_note_read(struct jit_register_use use[32], uint32_t reg)
{
	if (reg == 0u)
		return;
	if (!use[reg].first_access_seen) {
		use[reg].needs_initial = 1u;
		use[reg].first_access_seen = 1u;
	}
	if (use[reg].reads != UINT8_MAX)
		use[reg].reads++;
}

static void jit_note_write(struct jit_register_use use[32], uint32_t reg)
{
	if (reg == 0u)
		return;
	if (!use[reg].first_access_seen)
		use[reg].first_access_seen = 1u;
	if (use[reg].writes != UINT8_MAX)
		use[reg].writes++;
}

static void jit_note_instruction_registers(
	struct jit_register_use use[32], uint32_t instruction,
	enum jit_instruction_kind kind)
{
	uint32_t opcode = instruction & 0x7fu;
	uint32_t rd = (instruction >> 7) & 0x1fu;
	uint32_t rs1 = (instruction >> 15) & 0x1fu;
	uint32_t rs2 = (instruction >> 20) & 0x1fu;

	switch (kind) {
	case JIT_INSN_INTEGER:
		if (opcode == 0x13u)
			jit_note_read(use, rs1);
		else if (opcode == 0x33u) {
			jit_note_read(use, rs1);
			jit_note_read(use, rs2);
		}
		jit_note_write(use, rd);
		break;
	case JIT_INSN_LOAD:
		jit_note_read(use, rs1);
		jit_note_write(use, rd);
		break;
	case JIT_INSN_STORE:
		jit_note_read(use, rs1);
		jit_note_read(use, rs2);
		break;
	case JIT_INSN_AMO:
		jit_note_read(use, rs1);
		jit_note_read(use, rs2);
		jit_note_write(use, rd);
		break;
	case JIT_INSN_CSR:
		if (((instruction >> 12) & 4u) == 0u)
			jit_note_read(use, rs1);
		jit_note_write(use, rd);
		break;
	case JIT_INSN_BRANCH:
		jit_note_read(use, rs1);
		jit_note_read(use, rs2);
		break;
	case JIT_INSN_JAL:
		jit_note_write(use, rd);
		break;
	case JIT_INSN_JALR:
		/* Read before write is important when rd aliases rs1. */
		jit_note_read(use, rs1);
		jit_note_write(use, rd);
		break;
	default:
		break;
	}
}

static void jit_build_register_map(struct jit_register_map *map,
				   const uint32_t *instructions,
				   const enum jit_instruction_kind *kinds,
				   uint32_t count, bool repeated,
				   bool dynamic_counter)
{
	struct jit_register_use use[32] = { 0 };
	bool selected[32] = { false };
	uint32_t prefix_writes = 0u;

	memset(map, 0, sizeof(*map));
	memset(map->host_for_guest, -1, sizeof(map->host_for_guest));
	map->host_for_guest[0] = RV_X0;

	for (uint32_t i = 0; i < count; i++)
		jit_note_instruction_registers(use, instructions[i], kinds[i]);
	/* Writes in the leading integer run dominate every exit, so those
	 * registers need no initial load.  Anything that can bail out ends it. */
	for (uint32_t i = 0; i < count && kinds[i] == JIT_INSN_INTEGER; i++)
		prefix_writes |= UINT32_C(1) << ((instructions[i] >> 7) & 0x1fu);
	for (uint32_t reg = 1u; reg < 32u; reg++)
		use[reg].needs_initial |= !(prefix_writes & (UINT32_C(1) << reg));

	for (uint32_t slot = 0; slot < JIT_MAX_MAPPED_REGISTERS; slot++) {
		uint32_t host_reg;
		uint32_t best_reg = 0;
		int best_benefit = 0;
		uint32_t caller_slot = slot;

		/* t3 is always free for a mapped guest register.  a6 is the loop
		 * counter only for repeated regions, and a7 is reserved only when an
		 * exact dynamic instruction count is required.  Spill additional high
		 * value mappings into ABI-preserved s2..s9. */
		if (caller_slot == 0u) {
			host_reg = RV_T3;
		} else {
			caller_slot--;
			if (!repeated && caller_slot == 0u) {
				host_reg = RV_A6;
			} else {
				if (!repeated)
					caller_slot--;
				if (!dynamic_counter && caller_slot == 0u) {
					host_reg = RV_A7;
				} else {
					if (!dynamic_counter)
						caller_slot--;
					host_reg = RV_S2 + caller_slot;
				}
			}
		}
		for (uint32_t reg = 1; reg < 32u; reg++) {
			int ordinary_cost;
			int mapped_cost;
			int benefit;

			if (selected[reg])
				continue;
			ordinary_cost = use[reg].reads + use[reg].writes;
			/* A tail back-edge executes the generated body many times per
			 * call.  Price its per-instruction guest-register traffic over a
			 * conservative four iterations, while the save/load/commit cost is
			 * paid only once.  This keeps transient load results and loop limits
			 * resident instead of spilling them on every native iteration. */
			if (repeated)
				ordinary_cost *= 4;
			/* rv32_native_jit_enter saves callee-saved registers once per
			 * chain, so only the guest load and store count here. */
			mapped_cost = use[reg].needs_initial + !!use[reg].writes;
			benefit = ordinary_cost - mapped_cost;
			if (benefit > best_benefit) {
				best_benefit = benefit;
				best_reg = reg;
			}
		}

		if (best_reg == 0u)
			break;
		selected[best_reg] = true;
		map->host_for_guest[best_reg] = (int8_t)host_reg;
		map->guest_for_host[map->count] = (uint8_t)best_reg;
		map->host_register[map->count] = (uint8_t)host_reg;
		map->needs_initial[map->count] = use[best_reg].needs_initial;
		map->dirty[map->count] = use[best_reg].writes != 0u;
		map->count++;
	}
}

static uint32_t jit_guest_source(struct jit_builder *builder,
				 const struct jit_register_map *map,
				 uint32_t guest_reg, uint32_t temporary)
{
	int host_reg = map->host_for_guest[guest_reg];

	if (host_reg >= 0)
		return (uint32_t)host_reg;
	jit_emit_load_host(builder, temporary, JIT_GUEST_REGS_REGISTER,
			   guest_reg * sizeof(uint32_t));
	return temporary;
}

static uint32_t jit_guest_destination(const struct jit_register_map *map,
				      uint32_t guest_reg,
				      uint32_t temporary)
{
	int host_reg = map->host_for_guest[guest_reg];

	return host_reg >= 0 ? (uint32_t)host_reg : temporary;
}

static void jit_commit_unmapped_destination(
	struct jit_builder *builder, const struct jit_register_map *map,
	uint32_t guest_reg, uint32_t host_reg)
{
	if (guest_reg != 0u && map->host_for_guest[guest_reg] < 0)
		jit_emit_store_host(builder, JIT_GUEST_REGS_REGISTER, host_reg,
				    guest_reg * sizeof(uint32_t));
}

/* Add this region's executed instructions to the chain count in s10.  A
 * region using the second loop-read cache has borrowed s10/s11, so update its
 * saved count and restore both chain registers instead. */
static void jit_emit_chain_count(struct jit_builder *builder,
				 bool chain_registers_clobbered,
				 bool dynamic_counter_active, uint32_t count)
{
	if (chain_registers_clobbered) {
		jit_emit_load_host(builder, RV_T1, JIT_RUNTIME_REGISTER,
				   offsetof(struct rv32_native_jit_runtime,
					    chain_executed));
		if (dynamic_counter_active)
			jit_emit(builder, rv_r(0x33u, RV_T1, 0u, RV_T1,
					       JIT_EXECUTED_REGISTER, 0u));
		else
			jit_emit(builder, rv_i(0x13u, RV_T1, 0u, RV_T1,
					       (int32_t)count));
		jit_emit_store_host(builder, JIT_RUNTIME_REGISTER, RV_T1,
				    offsetof(struct rv32_native_jit_runtime,
					     chain_executed));
		jit_emit(builder, rv_i(0x13u, JIT_CHAIN_EXECUTED_REGISTER,
				       0u, RV_T1, 0));
		jit_emit_load_host(builder, JIT_CHAIN_LIMIT_REGISTER,
				   JIT_RUNTIME_REGISTER,
				   offsetof(struct rv32_native_jit_runtime,
					    chain_instruction_limit));
	} else if (dynamic_counter_active) {
		jit_emit(builder, rv_r(0x33u, JIT_CHAIN_EXECUTED_REGISTER, 0u,
				       JIT_CHAIN_EXECUTED_REGISTER,
				       JIT_EXECUTED_REGISTER, 0u));
	} else {
		jit_emit(builder, rv_i(0x13u, JIT_CHAIN_EXECUTED_REGISTER, 0u,
				       JIT_CHAIN_EXECUTED_REGISTER,
				       (int32_t)count));
	}
}

/* Write every dirty mapped guest register back to the guest register file. */
static void jit_emit_register_commit(struct jit_builder *builder,
				     const struct jit_register_map *map)
{
	for (uint32_t i = 0; i < map->count; i++) {
		if (map->dirty[i])
			jit_emit_store_host(builder, JIT_GUEST_REGS_REGISTER,
					    map->host_register[i],
					    map->guest_for_host[i] *
					    sizeof(uint32_t));
	}
}

static void jit_emit_link_value(struct jit_builder *builder,
				const struct jit_register_map *map,
				uint32_t guest_rd, uint32_t value)
{
	uint32_t destination;

	if (guest_rd == 0u)
		return;
	destination = jit_guest_destination(map, guest_rd, RV_T0);
	jit_emit_constant(builder, destination, value);
	jit_commit_unmapped_destination(builder, map, guest_rd, destination);
}

static bool jit_emit_integer(struct jit_builder *builder,
			     const struct jit_register_map *map,
			     uint32_t instruction, uint32_t guest_pc)
{
	uint32_t opcode = instruction & 0x7fu;
	uint32_t rd = (instruction >> 7) & 0x1fu;
	uint32_t rs1 = (instruction >> 15) & 0x1fu;
	uint32_t rs2 = (instruction >> 20) & 0x1fu;
	uint32_t source1;
	uint32_t source2;
	uint32_t destination;

	if (rd == 0u)
		return true;
	destination = jit_guest_destination(map, rd, RV_T2);

	switch (opcode) {
	case 0x37u: /* LUI */
		jit_emit(builder, (instruction & 0xfffff000u) |
				  (destination << 7) | 0x37u);
		break;
	case 0x17u: /* AUIPC uses the guest PC, not the native code PC. */
		jit_emit_constant(builder, destination,
				  guest_pc + (instruction & 0xfffff000u));
		break;
	case 0x13u:
		source1 = jit_guest_source(builder, map, rs1, RV_T0);
		jit_emit(builder, (instruction & 0xfff07000u) |
				  (source1 << 15) | (destination << 7) | 0x13u);
		break;
	case 0x33u:
		source1 = jit_guest_source(builder, map, rs1, RV_T0);
		if (rs2 == rs1)
			source2 = source1;
		else
			source2 = jit_guest_source(builder, map, rs2, RV_T1);
		jit_emit(builder, rv_r(0x33u, destination,
				     (instruction >> 12) & 7u,
				     source1, source2, instruction >> 25));
		break;
	default:
		return false;
	}

	jit_commit_unmapped_destination(builder, map, rd, destination);
	return !builder->failed;
}

static void jit_record_bailout(struct jit_builder *builder)
{
	uint32_t *patch;

	if (builder->bailout_patch_count >= JIT_MAX_BAILOUT_PATCHES) {
		builder->failed = true;
		return;
	}
	patch = builder->cursor;
	jit_emit(builder, 0u);
	if (!builder->failed)
		builder->bailout_patches[builder->bailout_patch_count++] = patch;
}

static void jit_record_control_exit(struct jit_builder *builder)
{
	uint32_t *patch;

	if (builder->control_exit_patch_count >=
	    JIT_MAX_CONTROL_EXIT_PATCHES) {
		builder->failed = true;
		return;
	}
	patch = builder->cursor;
	jit_emit(builder, 0u);
	if (!builder->failed)
		builder->control_exit_patches[
			builder->control_exit_patch_count++] = patch;
}

/* An exit that must reach the interpreter: it publishes the result and
 * returns to C without following a link or the dispatch stub. */
static void jit_record_hard_exit(struct jit_builder *builder)
{
	uint32_t *patch;

	if (builder->hard_exit_patch_count >= JIT_MAX_CONTROL_EXIT_PATCHES) {
		builder->failed = true;
		return;
	}
	patch = builder->cursor;
	jit_emit(builder, 0u);
	if (!builder->failed)
		builder->hard_exit_patches[builder->hard_exit_patch_count++] = patch;
}

static bool jit_emit_executed_add(struct jit_builder *builder,
				  uint32_t instructions)
{
	if (instructions == 0u)
		return true;
	if (instructions > 2047u)
		return false;
	jit_emit(builder, rv_i(0x13u, JIT_EXECUTED_REGISTER, 0u,
			       JIT_EXECUTED_REGISTER,
			       (int32_t)instructions));
	return !builder->failed;
}

/* A page miss is uncommon after the first access.  Return its compact site
 * number to C instead of duplicating the checked translation callback at
 * every generated load/store.  The resolver completes the one memory
 * operation, seeds the page cache, and resumes the guest in the interpreter.
 * Keeping these cold stubs to a few words is what lets memory-heavy loops fit
 * in the executable L2 cache. */
static void jit_emit_memory_bailout(struct jit_builder *builder,
				    uint32_t site, bool store)
{
	jit_emit_store_host(builder, JIT_RUNTIME_REGISTER, RV_A1,
			    offsetof(struct rv32_native_jit_runtime,
				     memory_address));
	if (store)
		jit_emit_store_host(builder, JIT_RUNTIME_REGISTER, RV_A2,
				    offsetof(struct rv32_native_jit_runtime,
					     memory_value));
	/* The result is published only on return to C, not at each native link.
	 * Seed the local count here even for straight-line regions: the resolver
	 * adds this site's static prefix, never a preceding region's result. */
	jit_emit_store_host(builder, JIT_RUNTIME_REGISTER,
			    builder->instruction_counter_active ?
				    JIT_EXECUTED_REGISTER : RV_X0,
			    JIT_RESULT_OFFSET(executed));
	jit_emit_constant(builder, RV_A0, site + 1u);
	jit_record_bailout(builder);
}

static struct jit_memory_stub *jit_defer_memory_stub(
	struct jit_builder *builder, uint32_t site, bool store, bool grouped,
	int32_t group_immediate, uint32_t group_value_guest)
{
	struct jit_memory_stub *stub;

	if (builder->memory_stub_count >= JIT_MAX_MEMORY_SITES) {
		builder->failed = true;
		return NULL;
	}
	stub = &builder->memory_stubs[builder->memory_stub_count++];
	memset(stub, 0, sizeof(*stub));
	stub->site = (uint8_t)site;
	stub->store = store;
	stub->grouped = grouped;
	stub->group_immediate = (int16_t)group_immediate;
	stub->group_value_guest = (uint8_t)group_value_guest;
	return stub;
}

static bool jit_defer_memory_guard(struct jit_builder *builder,
					   struct jit_memory_stub *stub,
				   uint32_t *instruction, uint32_t funct3,
				   uint32_t source1, uint32_t source2)
{
	struct jit_deferred_guard *guard;

	if (stub == NULL || stub->guard_count >= 3u) {
		builder->failed = true;
		return false;
	}
	guard = &stub->guards[stub->guard_count++];
	guard->instruction = instruction;
	guard->funct3 = (uint8_t)funct3;
	guard->source1 = (uint8_t)source1;
	guard->source2 = (uint8_t)source2;
	guard->refill = false;
	return true;
}

/* A page-tag guard directly follows lw t2, tag(t1), which a successful refill
 * retries.  Range and alignment guards never retry. */
static bool jit_defer_page_guard(struct jit_builder *builder,
	struct jit_memory_stub *stub, uint32_t *instruction,
	uint32_t funct3, uint32_t source1, uint32_t source2)
{
	if (!jit_defer_memory_guard(builder, stub, instruction,
				    funct3, source1, source2))
		return false;
	stub->guards[stub->guard_count - 1u].refill = true;
	return true;
}

static bool jit_defer_loop_read_cache(
	struct jit_builder *builder, const struct jit_memory_stub *memory_stub,
	uint32_t *miss_instruction, uint32_t *continuation,
	const struct jit_loop_read_cache *cache, uint32_t width)
{
	struct jit_loop_read_cache_stub *stub;
	ptrdiff_t memory_stub_index;

	if (cache == NULL || !cache->valid ||
	    builder->loop_read_cache_stub_count >= JIT_MAX_LOOP_READ_CACHES)
		return false;
	memory_stub_index = memory_stub - builder->memory_stubs;
	if (memory_stub_index < 0 ||
	    memory_stub_index >= (ptrdiff_t)JIT_MAX_MEMORY_SITES)
		return false;
	stub = &builder->loop_read_cache_stubs[
		builder->loop_read_cache_stub_count++];
	stub->miss_instruction = miss_instruction;
	stub->continuation = continuation;
	stub->memory_stub = (uint8_t)memory_stub_index;
	stub->tag_register = cache->tag_register;
	stub->bias_register = cache->bias_register;
	stub->alignment_mask = cache->alignment_invariant ? (uint8_t)(width - 1u) : 0u;
	return true;
}

static bool jit_defer_loop_write_cache(
	struct jit_builder *builder, const struct jit_memory_stub *memory_stub,
	uint32_t *miss_instruction, uint32_t *continuation, uint32_t funct3,
	uint32_t width, bool alignment_invariant)
{
	struct jit_loop_write_cache_stub *stub;
	ptrdiff_t memory_stub_index;

	if (builder->loop_write_cache_stub_count >=
	    JIT_MAX_LOOP_WRITE_CACHES)
		return false;
	memory_stub_index = memory_stub - builder->memory_stubs;
	if (memory_stub_index < 0 ||
	    memory_stub_index >= (ptrdiff_t)JIT_MAX_MEMORY_SITES)
		return false;
	stub = &builder->loop_write_cache_stubs[
		builder->loop_write_cache_stub_count++];
	stub->miss_instruction = miss_instruction;
	stub->continuation = continuation;
	stub->memory_stub = (uint8_t)memory_stub_index;
	stub->funct3 = (uint8_t)funct3;
	stub->width = (uint8_t)width;
	stub->alignment_mask = alignment_invariant ? (uint8_t)(width - 1u) : 0u;
	return true;
}

static bool jit_emit_refill_alignment_guard(
	struct jit_builder *builder, struct jit_memory_stub *memory_stub,
	uint32_t mask)
{
	uint32_t *miss;

	if (mask == 0u)
		return true;
	/* Keep t0 (page), a1 (address) and a2 (store value) intact. The local tag
	 * must not become valid until this and the shared translation check pass. */
	jit_emit(builder, rv_i(0x13u, RV_T2, 7u, RV_A1, (int32_t)mask));
	miss = builder->cursor;
	jit_emit(builder, 0u);
	return jit_defer_memory_guard(builder, memory_stub, miss,
				      1u, RV_T2, RV_X0);
}

/* A local tag miss need not leave native execution when the shared checked
 * page cache already knows the mapping.  Keep this refresh path out of the
 * repeated loop body; only a true shared-cache miss reaches the resolver. */
static bool jit_emit_loop_read_cache_stubs(struct jit_builder *builder)
{
	const size_t pages_offset =
		offsetof(struct rv32_native_jit_runtime, read_pages);

	for (uint32_t index = 0;
	     index < builder->loop_read_cache_stub_count; index++) {
		const struct jit_loop_read_cache_stub *stub =
			&builder->loop_read_cache_stubs[index];
		struct jit_memory_stub *memory_stub =
			&builder->memory_stubs[stub->memory_stub];
		uint32_t *refresh = builder->cursor;
		uint32_t *shared_miss;
		uint32_t *resume;

		if (!jit_patch_branch(stub->miss_instruction, refresh, 1u,
				      RV_T0, stub->tag_register))
			return false;
		if (!jit_emit_refill_alignment_guard(builder, memory_stub,
						     stub->alignment_mask))
			return false;
		/* t0 still contains the current virtual page from the hot guard. */
		jit_emit(builder, rv_i(0x13u, RV_T1, 7u, RV_T0,
				       RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u));
		jit_emit(builder, rv_i(0x13u, RV_T1, 1u, RV_T1, 3));
		jit_emit(builder, rv_r(0x33u, RV_T1, 0u,
				       JIT_RUNTIME_REGISTER, RV_T1, 0u));
		jit_emit_load_host(builder, RV_T2, RV_T1,
				   pages_offset +
				   offsetof(struct rv32_native_jit_read_page,
					    virtual_page));
		shared_miss = builder->cursor;
		jit_emit(builder, 0u);
		jit_emit_load_host(builder, stub->bias_register, RV_T1,
				   pages_offset +
				   offsetof(struct rv32_native_jit_read_page,
					    host_bias));
		jit_emit(builder, rv_i(0x13u, stub->tag_register, 0u, RV_T0, 0));
		resume = builder->cursor;
		jit_emit(builder, 0u);
		if (builder->failed ||
		    !jit_defer_page_guard(builder, memory_stub, shared_miss,
					     1u, RV_T0, RV_T2) ||
		    !jit_patch_branch(resume, stub->continuation, 0u,
				      RV_X0, RV_X0))
			return false;
	}
	return !builder->failed;
}

static bool jit_emit_deferred_memory_stubs(
	struct jit_builder *builder, const struct jit_register_map *map,
	const struct rv32_native_jit_memory_site *sites)
{
	for (uint32_t index = 0; index < builder->memory_stub_count; index++) {
		const struct jit_memory_stub *stub = &builder->memory_stubs[index];
		const struct jit_deferred_guard *refill = NULL;

		if (builder->refill) {
			for (uint32_t g = 0; g < stub->guard_count; g++) {
				if (!stub->guards[g].refill)
					continue;
				if (refill != NULL)
					return false;
				refill = &stub->guards[g];
			}
		}
		if (refill != NULL) {
			/* The retry target must be the tag load, before any guest
			 * access. */
			if ((refill->instruction[-1] & 0x000fffffu) !=
			    rv_i(0x03u, RV_T2, 2u, RV_T1, 0) ||
			    !jit_patch_branch(refill->instruction, builder->cursor,
				refill->funct3, refill->source1, refill->source2))
				return false;
			if (stub->grouped)
				jit_emit(builder, rv_i(0x13u, RV_A1, 0u, RV_A4,
						       stub->group_immediate));
			jit_emit_constant(builder, RV_A0,
				(sites[stub->site].descriptor & 7u) |
				(stub->store ? 8u : 0u));
			jit_emit_constant(builder, RV_T2,
				(uint32_t)(uintptr_t)rv32_native_jit_refill_trampoline);
			jit_emit(builder, rv_i(0x67u, RV_T4, 0u, RV_T2, 0));
			uint32_t *retry = builder->cursor;
			jit_emit(builder, 0u);
			if (!jit_patch_branch(retry, refill->instruction - 1,
					      1u, RV_T2, RV_X0))
				return false;
		}
		uint32_t *slow_path = builder->cursor;

		for (uint32_t guard = 0; guard < stub->guard_count; guard++) {
			const struct jit_deferred_guard *patch =
				&stub->guards[guard];
			if (patch == refill)
				continue;

			if (!jit_patch_branch(patch->instruction, slow_path,
					      patch->funct3, patch->source1,
					      patch->source2))
				return false;
		}
		if (stub->grouped) {
			jit_emit(builder, rv_i(0x13u, RV_A1, 0u, RV_A4,
					       stub->group_immediate));
			if (stub->store) {
				uint32_t value = jit_guest_source(
					builder, map, stub->group_value_guest, RV_T2);

				jit_emit(builder, rv_i(0x13u, RV_A2, 0u,
						       value, 0));
			}
		}
		jit_emit_memory_bailout(builder, stub->site, stub->store);
	}
	return !builder->failed;
}

static bool jit_patch_unconditional(uint32_t *instruction,
				    const uint32_t *target)
{
	return jit_patch_branch(instruction, target, 0u, RV_X0, RV_X0);
}

/* A fast framebuffer store still has to notify the backend.  Ordinary RAM
 * sees only the load and predictable negative branch below.  Framebuffer
 * stores call one shared cold helper per generated block, avoiding a copy of
 * the min/max dirty-range sequence at every store instruction. */
static bool jit_emit_linear_dirty_page(struct jit_builder *builder,
				       uint32_t calls, uint32_t bytes)
{
	uint32_t *not_linear_patch;
	uint32_t *already_dirty_patch;
	uint32_t *call_patch;

	if (builder->dirty_call_patch_count >= JIT_MAX_MEMORY_SITES) {
		builder->failed = true;
		return false;
	}

	/* t1 retains runtime + (write-cache index * entry size). */
	jit_emit_load_host(builder, RV_T0, RV_T1,
			   offsetof(struct rv32_native_jit_runtime, write_pages) +
			   offsetof(struct rv32_native_jit_write_page,
				    linear_page_start));
	not_linear_patch = builder->cursor;
	jit_emit(builder, 0u); /* blt t0, x0, done */
	jit_emit_load_host(builder, RV_T2, RV_T1,
			   offsetof(struct rv32_native_jit_runtime, write_pages) +
			   offsetof(struct rv32_native_jit_write_page, dirty_epoch));
	jit_emit_load_host(builder, RV_A3, JIT_RUNTIME_REGISTER,
			   offsetof(struct rv32_native_jit_runtime, dirty_epoch));
	already_dirty_patch = builder->cursor;
	jit_emit(builder, 0u); /* beq page epoch, dispatch epoch, done */
	jit_emit_store_host(builder, RV_T1, RV_A3,
			    offsetof(struct rv32_native_jit_runtime, write_pages) +
			    offsetof(struct rv32_native_jit_write_page, dirty_epoch));

#if CONFIG_RV32_HOST_PERF_STATS
	jit_emit_constant(builder, RV_A2, calls);
	jit_emit_constant(builder, RV_A3, bytes);
#else
	(void)calls;
	(void)bytes;
#endif
	call_patch = builder->cursor;
	jit_emit(builder, 0u); /* jal ra, shared dirty-range helper */
	if (!builder->failed)
		builder->dirty_call_patches[builder->dirty_call_patch_count++] =
			call_patch;

	if (builder->failed ||
	    !jit_patch_branch(not_linear_patch, builder->cursor,
			      4u, RV_T0, RV_X0) ||
	    !jit_patch_branch(already_dirty_patch, builder->cursor,
			      0u, RV_T2, RV_A3))
		return false;
	return true;
}

/* Repeated writes usually stay within one translated page.  A local hit does
 * only the store; a page change refreshes from the shared checked cache and
 * performs dirty-page accounting once on this cold path. */
static bool jit_emit_loop_write_cache_stubs(struct jit_builder *builder)
{
	const size_t pages_offset =
		offsetof(struct rv32_native_jit_runtime, write_pages);

	for (uint32_t index = 0;
	     index < builder->loop_write_cache_stub_count; index++) {
		const struct jit_loop_write_cache_stub *stub =
			&builder->loop_write_cache_stubs[index];
		struct jit_memory_stub *memory_stub =
			&builder->memory_stubs[stub->memory_stub];
		uint32_t *refresh = builder->cursor;
		uint32_t *shared_miss;
		uint32_t *resume;

		if (!jit_patch_branch(stub->miss_instruction, refresh, 1u,
				      RV_T0, JIT_LOOP_WRITE_TAG_REGISTER))
			return false;
		if (!jit_emit_refill_alignment_guard(builder, memory_stub,
						     stub->alignment_mask))
			return false;
		/* t0 is the current virtual page; a1/a2 retain address and value. */
		jit_emit(builder, rv_i(0x13u, RV_T1, 7u, RV_T0,
				       RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u));
		jit_emit(builder, rv_i(0x13u, RV_T1, 1u, RV_T1, 5));
		jit_emit(builder, rv_r(0x33u, RV_T1, 0u,
				       JIT_RUNTIME_REGISTER, RV_T1, 0u));
		jit_emit_load_host(builder, RV_T2, RV_T1,
				   pages_offset +
				   offsetof(struct rv32_native_jit_write_page,
					    virtual_page));
		shared_miss = builder->cursor;
		jit_emit(builder, 0u);
		jit_emit_load_host(builder, JIT_LOOP_WRITE_BIAS_REGISTER, RV_T1,
				   pages_offset +
				   offsetof(struct rv32_native_jit_write_page,
					    host_bias));
		jit_emit(builder, rv_i(0x13u, JIT_LOOP_WRITE_TAG_REGISTER,
				       0u, RV_T0, 0));
		jit_emit(builder, rv_r(0x33u, RV_T0, 0u, RV_A1,
				       JIT_LOOP_WRITE_BIAS_REGISTER, 0u));
		jit_emit(builder, rv_s(0x23u, stub->funct3, RV_T0, RV_A2, 0));
		if (!jit_emit_linear_dirty_page(builder, 1u, stub->width))
			return false;
		resume = builder->cursor;
		jit_emit(builder, 0u);
		if (builder->failed ||
		    !jit_defer_page_guard(builder, memory_stub, shared_miss,
					     1u, RV_T0, RV_T2) ||
		    !jit_patch_unconditional(resume, stub->continuation))
			return false;
	}
	return !builder->failed;
}

static bool jit_emit_linear_dirty_helper(struct jit_builder *builder)
{
	uint32_t *helper;
	uint32_t *start_covered_patch;
	uint32_t *end_covered_patch;

	if (builder->dirty_call_patch_count == 0u)
		return true;

	/* The compiler's auxiliary-section guard keeps normal fall-through out;
	 * every entry here is an explicit t4-linked dirty-page call. */
	helper = builder->cursor;
	for (uint32_t i = 0; i < builder->dirty_call_patch_count; i++) {
		if (!jit_patch_jal(builder->dirty_call_patches[i], helper,
				   JIT_DIRTY_LINK_REGISTER))
			return false;
	}

	/* t0 contains linear_page_start and t1 identifies the write-page entry. */
	jit_emit_load_host(builder, RV_T2, JIT_RUNTIME_REGISTER,
			   JIT_RESULT_OFFSET(linear_dirty_start));
	start_covered_patch = builder->cursor;
	jit_emit(builder, 0u); /* bgeu t0, t2, start_done */
	jit_emit_store_host(builder, JIT_RUNTIME_REGISTER, RV_T0,
			    JIT_RESULT_OFFSET(linear_dirty_start));
	if (builder->failed ||
	    !jit_patch_branch(start_covered_patch, builder->cursor,
			      7u, RV_T0, RV_T2))
		return false;

	jit_emit_load_host(builder, RV_T0, RV_T1,
			   offsetof(struct rv32_native_jit_runtime, write_pages) +
			   offsetof(struct rv32_native_jit_write_page,
				    linear_page_end));
	jit_emit_load_host(builder, RV_T2, JIT_RUNTIME_REGISTER,
			   JIT_RESULT_OFFSET(linear_dirty_end));
	end_covered_patch = builder->cursor;
	jit_emit(builder, 0u); /* bgeu t2, t0, end_done */
	jit_emit_store_host(builder, JIT_RUNTIME_REGISTER, RV_T0,
			    JIT_RESULT_OFFSET(linear_dirty_end));
	if (builder->failed ||
	    !jit_patch_branch(end_covered_patch, builder->cursor,
			      7u, RV_T2, RV_T0))
		return false;

#if CONFIG_RV32_HOST_PERF_STATS
	jit_emit_load_host(builder, RV_T0, JIT_RUNTIME_REGISTER,
			   JIT_RESULT_OFFSET(linear_store_calls));
	jit_emit(builder, rv_r(0x33u, RV_T0, 0u, RV_T0, RV_A2, 0u));
	jit_emit_store_host(builder, JIT_RUNTIME_REGISTER, RV_T0,
			    JIT_RESULT_OFFSET(linear_store_calls));
	jit_emit_load_host(builder, RV_T0, JIT_RUNTIME_REGISTER,
			   JIT_RESULT_OFFSET(linear_store_bytes));
	jit_emit(builder, rv_r(0x33u, RV_T0, 0u, RV_T0, RV_A3, 0u));
	jit_emit_store_host(builder, JIT_RUNTIME_REGISTER, RV_T0,
			    JIT_RESULT_OFFSET(linear_store_bytes));
#endif
	jit_emit(builder, rv_i(0x67u, RV_X0, 0u,
			       JIT_DIRTY_LINK_REGISTER, 0));
	return !builder->failed;
}

/* Emit one guarded host-base calculation shared by a consecutive memory
 * group. A4 preserves the guest base and A5 the translated host bias while
 * t0 becomes the host address corresponding to that guest base. */
static bool jit_emit_group_fast_address(
	struct jit_builder *builder, const struct jit_register_map *map,
	uint32_t guest_base, const struct jit_memory_group *group, bool store,
	struct jit_group_guards *guards)
{
	uint32_t source = jit_guest_source(builder, map, guest_base, RV_T0);
	size_t pages_offset = store ?
		offsetof(struct rv32_native_jit_runtime, write_pages) :
		offsetof(struct rv32_native_jit_runtime, read_pages);
	size_t tag_offset = pages_offset + (store ?
		offsetof(struct rv32_native_jit_write_page, virtual_page) :
		offsetof(struct rv32_native_jit_read_page, virtual_page));
	size_t bias_offset = pages_offset + (store ?
		offsetof(struct rv32_native_jit_write_page, host_bias) :
		offsetof(struct rv32_native_jit_read_page, host_bias));
	uint32_t entry_shift = store ? 5u : 3u;

	memset(guards, 0, sizeof(*guards));
	jit_emit(builder, rv_i(0x13u, RV_A4, 0u, source, 0));
	jit_emit(builder, rv_i(0x13u, RV_T0, 0u, RV_A4,
			       group->min_offset));
	jit_emit(builder, rv_i(0x13u, RV_T0, 5u, RV_T0, 12));
	jit_emit(builder, rv_i(0x13u, RV_T2, 0u, RV_A4,
			       group->max_end));
	jit_emit(builder, rv_i(0x13u, RV_T2, 5u, RV_T2, 12));
	guards->range_miss = builder->cursor;
	jit_emit(builder, 0u); /* bne first_page, last_page, slow */
	if (group->alignment_mask != 0u) {
		jit_emit(builder, rv_i(0x13u, RV_T2, 7u, RV_A4,
				       (int32_t)group->alignment_mask));
		guards->alignment_miss = builder->cursor;
		jit_emit(builder, 0u); /* bne alignment, x0, slow */
	}

	jit_emit(builder, rv_i(0x13u, RV_T1, 7u, RV_T0,
			       RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u));
	jit_emit(builder, rv_i(0x13u, RV_T1, 1u, RV_T1,
			       (int32_t)entry_shift));
	jit_emit(builder, rv_r(0x33u, RV_T1, 0u,
			       JIT_RUNTIME_REGISTER, RV_T1, 0u));
	jit_emit_load_host(builder, RV_T2, RV_T1, tag_offset);
	guards->page_miss = builder->cursor;
	jit_emit(builder, 0u); /* bne virtual_page, cached_tag, slow */
	jit_emit_load_host(builder, RV_A5, RV_T1, bias_offset);
	jit_emit(builder, rv_r(0x33u, RV_T0, 0u, RV_A4, RV_A5, 0u));
	return !builder->failed;
}

static bool jit_emit_load(struct jit_builder *builder,
			  const struct jit_register_map *map,
			  struct jit_cache_entry *entry,
			  struct rv32_native_jit_memory_site *memory_sites,
			  uint32_t instruction, uint32_t guest_pc,
			  uint32_t executed_before,
			  const struct jit_loop_read_cache *loop_cache)
{
	uint32_t rd = (instruction >> 7) & 0x1fu;
	uint32_t rs1 = (instruction >> 15) & 0x1fu;
	uint32_t funct3 = (instruction >> 12) & 7u;
	uint32_t source = jit_guest_source(builder, map, rs1, RV_T0);
	uint32_t width = jit_memory_width(funct3, true);
	uint32_t site = entry->memory_site_count;
	uint32_t descriptor;
	uint32_t destination;
	uint32_t *page_miss_patch;
	uint32_t *alignment_miss_patch = NULL;
	struct jit_memory_stub *stub;

	if (site >= JIT_MAX_MEMORY_SITES)
		return false;
	memory_sites[site].guest_pc = guest_pc;
	memory_sites[site].executed_before = (uint8_t)executed_before;
	descriptor = (site << 3) | funct3;
	memory_sites[site].descriptor = (uint8_t)descriptor;
	memory_sites[site].destination = (uint8_t)rd;
	memory_sites[site].store = 0u;
	entry->memory_site_count++;
	destination = jit_guest_destination(map, rd, RV_T2);
	stub = jit_defer_memory_stub(builder, site, false, false, 0, 0u);
	if (stub == NULL)
		return false;

	jit_emit(builder, rv_i(0x13u, RV_A1, 0u, source,
			       jit_i_immediate(instruction)));
	/* A successful checked callback remembers one host-contiguous page.  Most
	 * subsequent accesses can then perform the native load directly, while a
	 * page change, unaligned access or MMIO access returns to the checked path. */
	if (loop_cache != NULL) {
		uint32_t *local_miss_patch;
		uint32_t *continuation;

		if (width > 1u && !loop_cache->alignment_invariant) {
			jit_emit(builder, rv_i(0x13u, RV_T2, 7u, RV_A1,
					       (int32_t)(width - 1u)));
			alignment_miss_patch = builder->cursor;
			jit_emit(builder, 0u); /* bne alignment, x0, slow */
		}
		jit_emit(builder, rv_i(0x13u, RV_T0, 5u, RV_A1, 12));
		local_miss_patch = builder->cursor;
		jit_emit(builder, 0u); /* bne page, local tag, refresh */
		continuation = builder->cursor;
		jit_emit(builder, rv_r(0x33u, RV_T0, 0u, RV_A1,
				       loop_cache->bias_register, 0u));
		jit_emit(builder, rv_i(0x03u, destination, funct3, RV_T0, 0));
		if (!jit_defer_loop_read_cache(builder, stub,
					       local_miss_patch, continuation,
					       loop_cache, width) ||
		    (alignment_miss_patch != NULL &&
		     !jit_defer_memory_guard(builder, stub,
					     alignment_miss_patch, 1u,
					     RV_T2, RV_X0)))
			return false;
		jit_commit_unmapped_destination(builder, map, rd, destination);
		return !builder->failed;
	}

	jit_emit(builder, rv_i(0x13u, RV_T0, 5u, RV_A1, 12)); /* srli */
	jit_emit(builder, rv_i(0x13u, RV_T1, 7u, RV_T0,
			       RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u));
	jit_emit(builder, rv_i(0x13u, RV_T1, 1u, RV_T1, 3));
	jit_emit(builder, rv_r(0x33u, RV_T1, 0u,
			       JIT_RUNTIME_REGISTER, RV_T1, 0u));
	jit_emit_load_host(builder, RV_T2, RV_T1,
			   offsetof(struct rv32_native_jit_runtime, read_pages) +
			   offsetof(struct rv32_native_jit_read_page,
				    virtual_page));
	page_miss_patch = builder->cursor;
	jit_emit(builder, 0u); /* bne t0, t2, slow */
	if (width > 1u) {
		jit_emit(builder, rv_i(0x13u, RV_T0, 7u, RV_A1,
				       (int32_t)(width - 1u)));
		alignment_miss_patch = builder->cursor;
		jit_emit(builder, 0u); /* bne t0, x0, slow */
	}
	jit_emit_load_host(builder, RV_T0, RV_T1,
			   offsetof(struct rv32_native_jit_runtime, read_pages) +
			   offsetof(struct rv32_native_jit_read_page, host_bias));
	jit_emit(builder, rv_r(0x33u, RV_T0, 0u, RV_A1, RV_T0, 0u));
	jit_emit(builder, rv_i(0x03u, destination, funct3, RV_T0, 0));
	if (!jit_defer_page_guard(builder, stub, page_miss_patch,
				    1u, RV_T0, RV_T2) ||
	    (alignment_miss_patch != NULL &&
	     !jit_defer_memory_guard(builder, stub, alignment_miss_patch,
				     1u, RV_T0, RV_X0)))
		return false;
	jit_commit_unmapped_destination(builder, map, rd, destination);
	return !builder->failed;
}

static bool jit_emit_store(struct jit_builder *builder,
			   const struct jit_register_map *map,
			   struct jit_cache_entry *entry,
			   struct rv32_native_jit_memory_site *memory_sites,
			   uint32_t instruction, uint32_t guest_pc,
			   uint32_t executed_before,
			   const struct jit_loop_write_cache *loop_cache)
{
	uint32_t rs1 = (instruction >> 15) & 0x1fu;
	uint32_t rs2 = (instruction >> 20) & 0x1fu;
	uint32_t funct3 = (instruction >> 12) & 7u;
	uint32_t address_source = jit_guest_source(builder, map, rs1, RV_T0);
	uint32_t width = jit_memory_width(funct3, false);
	uint32_t value_source;
	uint32_t site = entry->memory_site_count;
	uint32_t descriptor;
	uint32_t *page_miss_patch;
	uint32_t *alignment_miss_patch = NULL;
	struct jit_memory_stub *stub;

	if (site >= JIT_MAX_MEMORY_SITES)
		return false;
	memory_sites[site].guest_pc = guest_pc;
	memory_sites[site].executed_before = (uint8_t)executed_before;
	descriptor = (site << 3) | funct3;
	memory_sites[site].descriptor = (uint8_t)descriptor;
	memory_sites[site].destination = 0u;
	memory_sites[site].store = 1u;
	entry->memory_site_count++;
	stub = jit_defer_memory_stub(builder, site, true, false, 0, 0u);
	if (stub == NULL)
		return false;

	jit_emit(builder, rv_i(0x13u, RV_A1, 0u, address_source,
			       jit_s_immediate(instruction)));
	/* Resolve rs2 after the address so two unmapped operands do not contend
	 * for t0.  a1 is already the final virtual address. */
	value_source = jit_guest_source(builder, map, rs2, RV_T0);
	jit_emit(builder, rv_i(0x13u, RV_A2, 0u, value_source, 0));
	if (loop_cache != NULL) {
		uint32_t *local_miss_patch;
		uint32_t *continuation;

		if (width > 1u && !loop_cache->alignment_invariant) {
			jit_emit(builder, rv_i(0x13u, RV_T2, 7u, RV_A1,
					       (int32_t)(width - 1u)));
			alignment_miss_patch = builder->cursor;
			jit_emit(builder, 0u); /* bne alignment, x0, slow */
		}
		jit_emit(builder, rv_i(0x13u, RV_T0, 5u, RV_A1, 12));
		local_miss_patch = builder->cursor;
		jit_emit(builder, 0u); /* bne page, local tag, refresh */
		jit_emit(builder, rv_r(0x33u, RV_T0, 0u, RV_A1,
				       JIT_LOOP_WRITE_BIAS_REGISTER, 0u));
		jit_emit(builder, rv_s(0x23u, funct3, RV_T0, RV_A2, 0));
		continuation = builder->cursor;
		if (!jit_defer_loop_write_cache(
			    builder, stub, local_miss_patch, continuation,
			    funct3, width, loop_cache->alignment_invariant) ||
		    (alignment_miss_patch != NULL &&
		     !jit_defer_memory_guard(builder, stub,
					     alignment_miss_patch, 1u,
					     RV_T2, RV_X0)))
			return false;
		return !builder->failed;
	}

	jit_emit(builder, rv_i(0x13u, RV_T0, 5u, RV_A1, 12)); /* srli */
	jit_emit(builder, rv_i(0x13u, RV_T1, 7u, RV_T0,
			       RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u));
	jit_emit(builder, rv_i(0x13u, RV_T1, 1u, RV_T1, 5));
	jit_emit(builder, rv_r(0x33u, RV_T1, 0u,
			       JIT_RUNTIME_REGISTER, RV_T1, 0u));
	jit_emit_load_host(builder, RV_T2, RV_T1,
			   offsetof(struct rv32_native_jit_runtime, write_pages) +
			   offsetof(struct rv32_native_jit_write_page,
				    virtual_page));
	page_miss_patch = builder->cursor;
	jit_emit(builder, 0u); /* bne t0, t2, slow */
	if (width > 1u) {
		jit_emit(builder, rv_i(0x13u, RV_T0, 7u, RV_A1,
				       (int32_t)(width - 1u)));
		alignment_miss_patch = builder->cursor;
		jit_emit(builder, 0u); /* bne t0, x0, slow */
	}
	jit_emit_load_host(builder, RV_T0, RV_T1,
			   offsetof(struct rv32_native_jit_runtime, write_pages) +
			   offsetof(struct rv32_native_jit_write_page, host_bias));
	jit_emit(builder, rv_r(0x33u, RV_T0, 0u, RV_A1, RV_T0, 0u));
	jit_emit(builder, rv_s(0x23u, funct3, RV_T0, RV_A2, 0));
	if (!jit_emit_linear_dirty_page(builder, 1u, width))
		return false;
	if (!jit_defer_page_guard(builder, stub, page_miss_patch,
				    1u, RV_T0, RV_T2) ||
	    (alignment_miss_patch != NULL &&
	     !jit_defer_memory_guard(builder, stub, alignment_miss_patch,
				     1u, RV_T0, RV_X0)))
		return false;
	return !builder->failed;
}

/* Word atomics on a single hart are a checked read-modify-write through the
 * write-page cache.  A miss re-executes the instruction in the interpreter
 * instead of resolving it in C, so no half-applied update can escape. */
static bool jit_emit_amo(struct jit_builder *builder,
			 const struct jit_register_map *map,
			 struct jit_cache_entry *entry,
			 struct rv32_native_jit_memory_site *memory_sites,
			 uint32_t instruction, uint32_t guest_pc,
			 uint32_t executed_before)
{
	uint32_t rd = (instruction >> 7) & 0x1fu;
	uint32_t rs1 = (instruction >> 15) & 0x1fu;
	uint32_t rs2 = (instruction >> 20) & 0x1fu;
	uint32_t funct5 = instruction >> 27;
	uint32_t address_source = jit_guest_source(builder, map, rs1, RV_T0);
	uint32_t value_source;
	uint32_t site = entry->memory_site_count;
	uint32_t *page_miss_patch;
	uint32_t *alignment_miss_patch;
	struct jit_memory_stub *stub;

	if (site >= JIT_MAX_MEMORY_SITES)
		return false;
	memory_sites[site].guest_pc = guest_pc;
	memory_sites[site].executed_before = (uint8_t)executed_before;
	memory_sites[site].descriptor = (uint8_t)((site << 3) | 2u);
	memory_sites[site].destination = 0u;
	memory_sites[site].store = JIT_SITE_AMO;
	entry->memory_site_count++;
	stub = jit_defer_memory_stub(builder, site, true, false, 0, 0u);
	if (stub == NULL)
		return false;

	jit_emit(builder, rv_i(0x13u, RV_A1, 0u, address_source, 0));
	value_source = jit_guest_source(builder, map, rs2, RV_T0);
	jit_emit(builder, rv_i(0x13u, RV_A2, 0u, value_source, 0));
	jit_emit(builder, rv_i(0x13u, RV_T0, 5u, RV_A1, 12)); /* srli */
	jit_emit(builder, rv_i(0x13u, RV_T1, 7u, RV_T0,
			       RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u));
	jit_emit(builder, rv_i(0x13u, RV_T1, 1u, RV_T1, 5));
	jit_emit(builder, rv_r(0x33u, RV_T1, 0u,
			       JIT_RUNTIME_REGISTER, RV_T1, 0u));
	jit_emit_load_host(builder, RV_T2, RV_T1,
			   offsetof(struct rv32_native_jit_runtime, write_pages) +
			   offsetof(struct rv32_native_jit_write_page,
				    virtual_page));
	page_miss_patch = builder->cursor;
	jit_emit(builder, 0u); /* bne t0, t2, slow */
	jit_emit(builder, rv_i(0x13u, RV_T0, 7u, RV_A1, 3));
	alignment_miss_patch = builder->cursor;
	jit_emit(builder, 0u); /* bne t0, x0, slow */
	jit_emit_load_host(builder, RV_T0, RV_T1,
			   offsetof(struct rv32_native_jit_runtime, write_pages) +
			   offsetof(struct rv32_native_jit_write_page, host_bias));
	jit_emit(builder, rv_r(0x33u, RV_T0, 0u, RV_A1, RV_T0, 0u));
	if (funct5 == 0x02u || funct5 == 0x03u) {
		/* The reservation is the physical offset (host address minus the
		 * image base) shifted into extraflags, exactly as interpreted.
		 * t2 = this access's offset; a3 = reservation word. */
		jit_emit_load_host(builder, RV_T2, JIT_RUNTIME_REGISTER,
				   offsetof(struct rv32_native_jit_runtime, image));
		jit_emit(builder, rv_r(0x33u, RV_T2, 0u, RV_T0, RV_T2, 0x20u));
		jit_emit(builder, rv_i(0x03u, RV_A3, 2u, JIT_GUEST_REGS_REGISTER,
				       jit_csr_layout.extraflags));
		if (funct5 == 0x02u) {
			jit_emit(builder, rv_i(0x13u, RV_A3, 7u, RV_A3, 7));
			jit_emit(builder, rv_i(0x13u, RV_T2, 1u, RV_T2, 3));
			jit_emit(builder, rv_r(0x33u, RV_A3, 6u, RV_A3, RV_T2, 0u));
			jit_emit(builder, rv_s(0x23u, 2u, JIT_GUEST_REGS_REGISTER,
					       RV_A3, jit_csr_layout.extraflags));
			jit_emit(builder, rv_i(0x03u, RV_T2, 2u, RV_T0, 0)); /* value */
			if (rd != 0u) {
				uint32_t destination =
					jit_guest_destination(map, rd, RV_T2);

				if (destination != RV_T2)
					jit_emit(builder, rv_i(0x13u, destination, 0u,
							       RV_T2, 0));
				jit_commit_unmapped_destination(builder, map, rd,
								destination);
			}
		} else {
			uint32_t *mismatch_patch;
			uint32_t *done_patch;
			uint32_t destination = rd != 0u ?
				jit_guest_destination(map, rd, RV_T2) : RV_X0;

			jit_emit(builder, rv_i(0x13u, RV_A3, 5u, RV_A3, 3));
			mismatch_patch = builder->cursor;
			jit_emit(builder, 0u); /* bne a3, t2, failed */
			jit_emit(builder, rv_s(0x23u, 2u, RV_T0, RV_A2, 0));
			if (rd != 0u)
				jit_emit(builder, rv_i(0x13u, destination, 0u, RV_X0, 0));
			if (!jit_emit_linear_dirty_page(builder, 1u, 4u))
				return false;
			done_patch = builder->cursor;
			jit_emit(builder, 0u); /* j done */
			if (!jit_patch_branch(mismatch_patch, builder->cursor, 1u,
					      RV_A3, RV_T2))
				return false;
			if (rd != 0u)
				jit_emit(builder, rv_i(0x13u, destination, 0u, RV_X0, 1));
			if (!jit_patch_unconditional(done_patch, builder->cursor))
				return false;
			/* Any SC drops the reservation, as in the interpreter:
			 * every offset bit set matches no RAM offset. */
			jit_emit(builder, rv_i(0x03u, RV_A3, 2u, JIT_GUEST_REGS_REGISTER,
					       jit_csr_layout.extraflags));
			jit_emit(builder, rv_i(0x13u, RV_A3, 6u, RV_A3, -8));
			jit_emit(builder, rv_s(0x23u, 2u, JIT_GUEST_REGS_REGISTER,
					       RV_A3, jit_csr_layout.extraflags));
			if (rd != 0u)
				jit_commit_unmapped_destination(builder, map, rd,
								destination);
		}
		if (!jit_defer_page_guard(builder, stub, page_miss_patch,
					  1u, RV_T0, RV_T2) ||
		    !jit_defer_memory_guard(builder, stub, alignment_miss_patch,
					    1u, RV_T0, RV_X0))
			return false;
		return !builder->failed;
	}
	/* t1 must keep the write-page entry pointer for the dirty-page check
	 * below; t2 (the page tag) is free once the guard has passed. */
	jit_emit(builder, rv_i(0x03u, RV_T2, 2u, RV_T0, 0)); /* old value */
	switch (funct5) {
	case 0x00u: /* AMOADD */
		jit_emit(builder, rv_r(0x33u, RV_A2, 0u, RV_T2, RV_A2, 0u));
		break;
	case 0x01u: /* AMOSWAP */
		break;
	case 0x04u: /* AMOXOR */
		jit_emit(builder, rv_r(0x33u, RV_A2, 4u, RV_T2, RV_A2, 0u));
		break;
	case 0x08u: /* AMOOR */
		jit_emit(builder, rv_r(0x33u, RV_A2, 6u, RV_T2, RV_A2, 0u));
		break;
	case 0x0cu: /* AMOAND */
		jit_emit(builder, rv_r(0x33u, RV_A2, 7u, RV_T2, RV_A2, 0u));
		break;
	case 0x10u: /* AMOMIN: keep rs2 when rs2 < old */
		jit_emit(builder, rv_b(4u, RV_A2, RV_T2, 8));
		jit_emit(builder, rv_i(0x13u, RV_A2, 0u, RV_T2, 0));
		break;
	case 0x14u: /* AMOMAX: keep rs2 when rs2 >= old */
		jit_emit(builder, rv_b(5u, RV_A2, RV_T2, 8));
		jit_emit(builder, rv_i(0x13u, RV_A2, 0u, RV_T2, 0));
		break;
	case 0x18u: /* AMOMINU */
		jit_emit(builder, rv_b(6u, RV_A2, RV_T2, 8));
		jit_emit(builder, rv_i(0x13u, RV_A2, 0u, RV_T2, 0));
		break;
	case 0x1cu: /* AMOMAXU */
		jit_emit(builder, rv_b(7u, RV_A2, RV_T2, 8));
		jit_emit(builder, rv_i(0x13u, RV_A2, 0u, RV_T2, 0));
		break;
	default:
		return false;
	}
	jit_emit(builder, rv_s(0x23u, 2u, RV_T0, RV_A2, 0));
	/* The access has succeeded: publish rd before the dirty-page check,
	 * which clobbers t0, t2, a2 and a3. */
	if (rd != 0u) {
		uint32_t destination = jit_guest_destination(map, rd, RV_T2);

		if (destination != RV_T2)
			jit_emit(builder, rv_i(0x13u, destination, 0u, RV_T2, 0));
		jit_commit_unmapped_destination(builder, map, rd, destination);
	}
	if (!jit_emit_linear_dirty_page(builder, 1u, 4u))
		return false;
	if (!jit_defer_page_guard(builder, stub, page_miss_patch,
				  1u, RV_T0, RV_T2) ||
	    !jit_defer_memory_guard(builder, stub, alignment_miss_patch,
				    1u, RV_T0, RV_X0))
		return false;
	return !builder->failed;
}

/* Supervisor CSR accesses stay native.  Reads and side-effect-free writes
 * touch the guest state word directly.  A write that may change the memory
 * context (SUM/MXR) or make a supervisor interrupt deliverable leaves the
 * region after the write, so the interpreter re-evaluates both exactly as it
 * would after executing the instruction itself. */
static bool jit_emit_csr(struct jit_builder *builder,
			 const struct jit_register_map *map,
			 uint32_t instruction, uint32_t guest_pc)
{
	uint32_t rd = (instruction >> 7) & 0x1fu;
	uint32_t rs1 = (instruction >> 15) & 0x1fu;
	uint32_t funct3 = (instruction >> 12) & 7u;
	uint32_t csrno = instruction >> 20;
	bool writes = jit_csr_writes(instruction);
	int32_t offset = jit_csr_offset(csrno, writes);
	bool sum_check = writes && csrno == 0x100u && funct3 <= 3u;
	bool irq_check = writes &&
		(csrno == 0x100u || csrno == 0x104u || csrno == 0x144u) &&
		(funct3 == 1u || funct3 == 2u || funct3 == 5u || funct3 == 6u);
	struct jit_deferred_guard exits[2];
	struct jit_deferred_guard continues[3];
	uint32_t exit_count = 0u;
	uint32_t continue_count = 0u;
	uint32_t *skip_patch = NULL;
	uint32_t new_register = RV_T2;

	if (offset == INT32_MIN)
		return false;
	jit_emit(builder, rv_i(0x03u, RV_T1, 2u, JIT_GUEST_REGS_REGISTER,
			       offset)); /* old value */
	if (writes) {
		uint32_t source;

		switch (funct3) {
		case 1u: /* CSRRW */
			new_register = jit_guest_source(builder, map, rs1, RV_T2);
			break;
		case 2u: /* CSRRS */
			source = jit_guest_source(builder, map, rs1, RV_T2);
			jit_emit(builder, rv_r(0x33u, RV_T2, 6u, RV_T1, source, 0u));
			break;
		case 3u: /* CSRRC */
			source = jit_guest_source(builder, map, rs1, RV_T2);
			jit_emit(builder, rv_i(0x13u, RV_T0, 4u, source, -1));
			jit_emit(builder, rv_r(0x33u, RV_T2, 7u, RV_T1, RV_T0, 0u));
			break;
		case 5u: /* CSRRWI */
			jit_emit(builder, rv_i(0x13u, RV_T2, 0u, RV_X0, (int32_t)rs1));
			break;
		case 6u: /* CSRRSI */
			jit_emit(builder, rv_i(0x13u, RV_T2, 6u, RV_T1, (int32_t)rs1));
			break;
		case 7u: /* CSRRCI */
			jit_emit(builder, rv_i(0x13u, RV_T2, 7u, RV_T1, ~(int32_t)rs1));
			break;
		default:
			return false;
		}
		jit_emit(builder, rv_s(0x23u, 2u, JIT_GUEST_REGS_REGISTER,
				       new_register, offset));
		if (sum_check) {
			/* SUM (bit 18) or MXR (bit 19) changed: the memory context is
			 * now different, so finish in C. */
			jit_emit(builder, rv_r(0x33u, RV_T0, 4u, RV_T1, new_register, 0u));
			jit_emit(builder, rv_i(0x13u, RV_T0, 5u, RV_T0, 18));
			jit_emit(builder, rv_i(0x13u, RV_T0, 7u, RV_T0, 3));
			exits[exit_count++] = (struct jit_deferred_guard) {
				.instruction = builder->cursor, .funct3 = 1u,
				.source1 = RV_T0, .source2 = RV_X0,
			};
			jit_emit(builder, 0u); /* bne t0, x0, exit */
		}
	}
	if (rd != 0u) {
		uint32_t destination = jit_guest_destination(map, rd, RV_T0);

		jit_emit(builder, rv_i(0x13u, destination, 0u, RV_T1, 0));
		jit_commit_unmapped_destination(builder, map, rd, destination);
	}
	if (irq_check) {
		/* Mirror MiniRV32SupervisorInterruptCause: a pending enabled
		 * supervisor interrupt is deliverable in U-mode, or in S-mode with
		 * sstatus.SIE set.  Falling through these checks is the exit. */
		jit_emit(builder, rv_i(0x03u, RV_T0, 2u, JIT_GUEST_REGS_REGISTER,
				       jit_csr_layout.sip));
		jit_emit(builder, rv_i(0x03u, RV_T1, 2u, JIT_GUEST_REGS_REGISTER,
				       jit_csr_layout.sie));
		jit_emit(builder, rv_r(0x33u, RV_T0, 7u, RV_T0, RV_T1, 0u));
		jit_emit(builder, rv_i(0x13u, RV_T0, 7u, RV_T0, 0x220));
		continues[continue_count++] = (struct jit_deferred_guard) {
			.instruction = builder->cursor, .funct3 = 0u,
			.source1 = RV_T0, .source2 = RV_X0,
		};
		jit_emit(builder, 0u); /* beq t0, x0, continue: nothing pending */
		jit_emit(builder, rv_i(0x03u, RV_T1, 2u, JIT_GUEST_REGS_REGISTER,
				       jit_csr_layout.extraflags));
		jit_emit(builder, rv_i(0x13u, RV_T1, 7u, RV_T1, 3));
		exits[exit_count++] = (struct jit_deferred_guard) {
			.instruction = builder->cursor, .funct3 = 0u,
			.source1 = RV_T1, .source2 = RV_X0,
		};
		jit_emit(builder, 0u); /* beq t1, x0, exit: U-mode delivers */
		jit_emit(builder, rv_i(0x13u, RV_T1, 0u, RV_T1, -1));
		continues[continue_count++] = (struct jit_deferred_guard) {
			.instruction = builder->cursor, .funct3 = 1u,
			.source1 = RV_T1, .source2 = RV_X0,
		};
		jit_emit(builder, 0u); /* bne t1, x0, continue: M-mode */
		jit_emit(builder, rv_i(0x03u, RV_T1, 2u, JIT_GUEST_REGS_REGISTER,
				       jit_csr_layout.sstatus));
		jit_emit(builder, rv_i(0x13u, RV_T1, 7u, RV_T1, 2));
		continues[continue_count++] = (struct jit_deferred_guard) {
			.instruction = builder->cursor, .funct3 = 0u,
			.source1 = RV_T1, .source2 = RV_X0,
		};
		jit_emit(builder, 0u); /* beq t1, x0, continue: SIE clear */
	} else if (sum_check) {
		skip_patch = builder->cursor;
		jit_emit(builder, 0u); /* j continue */
	}
	if (sum_check || irq_check) {
		uint32_t *exit_label = builder->cursor;
		uint32_t *continue_label;

		/* The interpreter must see this exit before the guest can mask the
		 * interrupt again or touch memory under the new SUM/MXR: no link
		 * or dispatch stub may carry on natively. */
		jit_emit_constant(builder, RV_A1, guest_pc + 4u);
		jit_record_hard_exit(builder);
		continue_label = builder->cursor;
		if (builder->failed)
			return false;
		for (uint32_t i = 0; i < exit_count; i++) {
			if (!jit_patch_branch(exits[i].instruction, exit_label,
					      exits[i].funct3, exits[i].source1,
					      exits[i].source2))
				return false;
		}
		for (uint32_t i = 0; i < continue_count; i++) {
			if (!jit_patch_branch(continues[i].instruction, continue_label,
					      continues[i].funct3, continues[i].source1,
					      continues[i].source2))
				return false;
		}
		if (skip_patch != NULL &&
		    !jit_patch_unconditional(skip_patch, continue_label))
			return false;
	}
	return !builder->failed;
}

static bool jit_emit_load_group(
	struct jit_builder *builder, const struct jit_register_map *map,
	struct jit_cache_entry *entry,
	struct rv32_native_jit_memory_site *memory_sites,
	const uint32_t *instructions, uint32_t guest_pc,
	uint32_t executed_before, const struct jit_memory_group *group)
{
	uint32_t first = instructions[0];
	uint32_t first_rd = (first >> 7) & 0x1fu;
	uint32_t first_funct3 = (first >> 12) & 7u;
	uint32_t guest_base = (first >> 15) & 0x1fu;
	uint32_t site = entry->memory_site_count;
	struct jit_group_guards guards;
	struct jit_memory_stub *stub;

	if (site >= JIT_MAX_MEMORY_SITES ||
	    !jit_emit_group_fast_address(builder, map, guest_base, group,
					 false, &guards))
		return false;
	memory_sites[site].guest_pc = guest_pc;
	memory_sites[site].executed_before = (uint8_t)executed_before;
	memory_sites[site].descriptor = (uint8_t)((site << 3) | first_funct3);
	memory_sites[site].destination = (uint8_t)first_rd;
	memory_sites[site].store = 0u;
	entry->memory_site_count++;

	for (uint32_t i = 0; i < group->count; i++) {
		uint32_t instruction = instructions[i];
		uint32_t rd = (instruction >> 7) & 0x1fu;
		uint32_t destination = jit_guest_destination(map, rd, RV_T2);

		jit_emit(builder, rv_i(0x03u, destination,
				       (instruction >> 12) & 7u, RV_T0,
				       jit_i_immediate(instruction)));
		jit_commit_unmapped_destination(builder, map, rd, destination);
	}
	stub = jit_defer_memory_stub(builder, site, false, true,
				     jit_i_immediate(first), 0u);
	if (!jit_defer_memory_guard(builder, stub, guards.range_miss,
				    1u, RV_T0, RV_T2) ||
	    (guards.alignment_miss != NULL &&
	     !jit_defer_memory_guard(builder, stub, guards.alignment_miss,
				     1u, RV_T2, RV_X0)) ||
	    !jit_defer_page_guard(builder, stub, guards.page_miss,
				    1u, RV_T0, RV_T2))
		return false;
	return !builder->failed;
}

static bool jit_emit_store_group(
	struct jit_builder *builder, const struct jit_register_map *map,
	struct jit_cache_entry *entry,
	struct rv32_native_jit_memory_site *memory_sites,
	const uint32_t *instructions, uint32_t guest_pc,
	uint32_t executed_before, const struct jit_memory_group *group)
{
	uint32_t first = instructions[0];
	uint32_t first_rs2 = (first >> 20) & 0x1fu;
	uint32_t first_funct3 = (first >> 12) & 7u;
	uint32_t guest_base = (first >> 15) & 0x1fu;
	uint32_t site = entry->memory_site_count;
	uint32_t total_bytes = 0u;
	struct jit_group_guards guards;
	struct jit_memory_stub *stub;

	if (site >= JIT_MAX_MEMORY_SITES ||
	    !jit_emit_group_fast_address(builder, map, guest_base, group,
					 true, &guards))
		return false;
	memory_sites[site].guest_pc = guest_pc;
	memory_sites[site].executed_before = (uint8_t)executed_before;
	memory_sites[site].descriptor = (uint8_t)((site << 3) | first_funct3);
	memory_sites[site].destination = 0u;
	memory_sites[site].store = 1u;
	entry->memory_site_count++;

	for (uint32_t i = 0; i < group->count; i++) {
		uint32_t instruction = instructions[i];
		uint32_t rs2 = (instruction >> 20) & 0x1fu;
		uint32_t value = jit_guest_source(builder, map, rs2, RV_T2);

		jit_emit(builder, rv_s(0x23u, (instruction >> 12) & 7u,
				       RV_T0, value,
				       jit_s_immediate(instruction)));
		total_bytes += jit_memory_width((instruction >> 12) & 7u,
						false);
	}
	if (!jit_emit_linear_dirty_page(builder, group->count, total_bytes))
		return false;
	stub = jit_defer_memory_stub(builder, site, true, true,
				     jit_s_immediate(first), first_rs2);
	if (stub == NULL ||
	    builder->two_page_store_stub_count >= JIT_MAX_MEMORY_SITES)
		return false;
	struct jit_two_page_store_stub *two_page =
		&builder->two_page_store_stubs[builder->two_page_store_stub_count++];
	*two_page = (struct jit_two_page_store_stub) {
		.range_miss = guards.range_miss,
		.continuation = builder->cursor,
		.instructions = instructions,
		.min_offset = (int16_t)group->min_offset,
		.max_end = (int16_t)group->max_end,
		.memory_stub = (uint8_t)(stub - builder->memory_stubs),
		.count = (uint8_t)group->count,
		.alignment_mask = (uint8_t)group->alignment_mask,
	};
	if ((guards.alignment_miss != NULL &&
	     !jit_defer_memory_guard(builder, stub, guards.alignment_miss,
				     1u, RV_T2, RV_X0)) ||
	    !jit_defer_page_guard(builder, stub, guards.page_miss,
				    1u, RV_T0, RV_T2))
		return false;
	return !builder->failed;
}

/* A store group spans under a page of offsets, so at most two pages (with
 * 32-bit wraparound).  Both write mappings are checked before any store;
 * MMIO and dirty-tracked pages go through the resolver. */
static bool jit_emit_two_page_store_stubs(struct jit_builder *builder,
					 const struct jit_register_map *map)
{
	const size_t pages = offsetof(struct rv32_native_jit_runtime, write_pages);
	for (uint32_t n = 0; n < builder->two_page_store_stub_count; ++n) {
		const struct jit_two_page_store_stub *stub = &builder->two_page_store_stubs[n];
		struct jit_memory_stub *memory = &builder->memory_stubs[stub->memory_stub];
		struct jit_deferred_guard failures[5];
		uint32_t failure_count = 0u;
		if (!jit_patch_branch(stub->range_miss, builder->cursor, 1u, RV_T0, RV_T2))
			return false;
		/* Preserve the first/last VPNs and the original guest base (a4). */
		jit_emit(builder, rv_i(0x13u, RV_A2, 0u, RV_T0, 0));
		jit_emit(builder, rv_i(0x13u, RV_A1, 0u, RV_T2, 0));
		if (stub->alignment_mask != 0u) {
			jit_emit(builder, rv_i(0x13u, RV_T0, 7u, RV_A4, stub->alignment_mask));
			failures[failure_count++] = (struct jit_deferred_guard) {
				.instruction = builder->cursor, .funct3 = 1u,
				.source1 = RV_T0, .source2 = RV_X0,
			};
			jit_emit(builder, 0u);
		}
		for (uint32_t page = 0; page < 2u; ++page) {
			uint32_t tag = page == 0u ? RV_A2 : RV_A1;
			uint32_t bias = page == 0u ? RV_A5 : RV_A3;
			jit_emit(builder, rv_i(0x13u, RV_T1, 7u, tag,
					       RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u));
			jit_emit(builder, rv_i(0x13u, RV_T1, 1u, RV_T1, 5));
			jit_emit(builder, rv_r(0x33u, RV_T1, 0u, RV_T1, JIT_RUNTIME_REGISTER, 0u));
			jit_emit_load_host(builder, RV_T0, RV_T1,
				pages + offsetof(struct rv32_native_jit_write_page, virtual_page));
			failures[failure_count++] = (struct jit_deferred_guard) {
				.instruction = builder->cursor, .funct3 = 1u,
				.source1 = RV_T0, .source2 = (uint8_t)tag,
			};
			jit_emit(builder, 0u);
			jit_emit_load_host(builder, RV_T0, RV_T1,
				pages + offsetof(struct rv32_native_jit_write_page, linear_page_start));
			failures[failure_count++] = (struct jit_deferred_guard) {
				.instruction = builder->cursor, .funct3 = 5u,
				.source1 = RV_T0, .source2 = RV_X0,
			};
			jit_emit(builder, 0u); /* bge: nonnegative start means tracked/MMIO. */
			jit_emit_load_host(builder, bias, RV_T1,
				pages + offsetof(struct rv32_native_jit_write_page, host_bias));
		}
		for (uint32_t i = 0; i < stub->count; ++i) {
			uint32_t instruction = stub->instructions[i];
			int32_t offset = jit_s_immediate(instruction);
			uint32_t width = jit_memory_width((instruction >> 12) & 7u, false);
			jit_emit(builder, rv_i(0x13u, RV_T0, 0u, RV_A4, offset));
			if (offset == stub->min_offset) {
				jit_emit(builder, rv_r(0x33u, RV_T0, 0u, RV_T0, RV_A5, 0u));
			} else if (offset + (int32_t)width - 1 == stub->max_end) {
				jit_emit(builder, rv_r(0x33u, RV_T0, 0u, RV_T0, RV_A3, 0u));
			} else {
				jit_emit(builder, rv_i(0x13u, RV_T2, 5u, RV_T0, 12));
				uint32_t *first_page = builder->cursor;
				jit_emit(builder, 0u);
				jit_emit(builder, rv_r(0x33u, RV_T0, 0u, RV_T0, RV_A3, 0u));
				uint32_t *selected = builder->cursor;
				jit_emit(builder, 0u);
				if (!jit_patch_branch(first_page, builder->cursor, 0u, RV_T2, RV_A2))
					return false;
				jit_emit(builder, rv_r(0x33u, RV_T0, 0u, RV_T0, RV_A5, 0u));
				if (!jit_patch_unconditional(selected, builder->cursor))
					return false;
			}
			uint32_t value = jit_guest_source(builder, map,
						 (instruction >> 20) & 31u, RV_T2);
			jit_emit(builder, rv_s(0x23u, (instruction >> 12) & 7u, RV_T0, value, 0));
		}
		uint32_t *resume = builder->cursor;
		jit_emit(builder, 0u);
		if (!jit_patch_unconditional(resume, stub->continuation))
			return false;
		for (uint32_t i = 0; i < failure_count; ++i) {
			const struct jit_deferred_guard *f = &failures[i];
			if (!jit_patch_branch(f->instruction, builder->cursor,
					      f->funct3, f->source1, f->source2))
				return false;
		}
		if (!jit_defer_memory_guard(builder, memory, builder->cursor, 0u, RV_X0, RV_X0))
			return false;
		jit_emit(builder, 0u);
	}
	return !builder->failed;
}

static bool jit_emit_persistent_memory_guard(
	struct jit_builder *builder, const struct jit_register_map *map,
	struct jit_cache_entry *entry,
	struct rv32_native_jit_memory_site *memory_sites,
	const struct jit_persistent_memory_group *group, uint32_t instruction,
	uint32_t guest_pc, uint32_t executed_before)
{
	uint32_t funct3 = (instruction >> 12) & 7u;
	uint32_t site = entry->memory_site_count;
	uint32_t guest_rd = (instruction >> 7) & 0x1fu;
	uint32_t guest_rs2 = (instruction >> 20) & 0x1fu;
	int32_t immediate = group->store ?
		jit_s_immediate(instruction) : jit_i_immediate(instruction);
	struct jit_group_guards guards;
	struct jit_memory_stub *stub;

	if (site >= JIT_MAX_MEMORY_SITES ||
	    !jit_emit_group_fast_address(builder, map, group->base,
					 &group->memory, group->store, &guards))
		return false;
	memory_sites[site].guest_pc = guest_pc;
	memory_sites[site].executed_before = (uint8_t)executed_before;
	memory_sites[site].descriptor = (uint8_t)((site << 3) | funct3);
	memory_sites[site].destination = (uint8_t)guest_rd;
	memory_sites[site].store = group->store;
	entry->memory_site_count++;
	stub = jit_defer_memory_stub(builder, site, group->store, true,
				     immediate, guest_rs2);
	if (!jit_defer_memory_guard(builder, stub, guards.range_miss,
				    1u, RV_T0, RV_T2) ||
	    (guards.alignment_miss != NULL &&
	     !jit_defer_memory_guard(builder, stub, guards.alignment_miss,
				     1u, RV_T2, RV_X0)) ||
	    !jit_defer_page_guard(builder, stub, guards.page_miss,
				    1u, RV_T0, RV_T2))
		return false;
	/* Guard misses branch away while a4 still holds the guest base.  The fast
	 * path retains the corresponding host base across the selected accesses. */
	jit_emit(builder, rv_r(0x33u, RV_A4, 0u, RV_A4, RV_A5, 0u));
	return !builder->failed;
}

/* The kernel linear-map check (see JIT_LINEAR_LOW_MARGIN): t1 = source -
 * (window start + margin), then destination = host address of the guest base
 * value in source.  The returned miss word becomes bgeu t1, limit, slow path;
 * nothing guest-visible has changed when it is taken. */
static uint32_t *jit_emit_linear_check(struct jit_builder *builder,
				       uint32_t source, uint32_t destination)
{
	uint32_t offset = builder->linear_offset_register;
	uint32_t *miss;

	if (source == RV_T1) {
		builder->failed = true;
		return NULL;
	}
	if (offset == RV_X0) {
		jit_emit_constant(builder, RV_T1, builder->linear_offset);
		offset = RV_T1;
	}
	jit_emit(builder, rv_r(0x33u, RV_T1, 0u, source, offset, 0u));
	miss = builder->cursor;
	jit_emit(builder, 0u); /* bgeu t1, limit, slow path */
	jit_emit(builder, rv_r(0x33u, destination, 0u, RV_T1,
			       builder->linear_host_register, 0u));
	return builder->failed ? NULL : miss;
}

/* Call after the inline access: the slow path resumes at the cursor. */
static bool jit_defer_linear_slow_path(struct jit_builder *builder,
	enum jit_linear_kind kind, uint32_t *miss,
	const uint32_t *instructions, uint32_t instruction, uint32_t guest_pc,
	uint32_t executed_before, const struct jit_memory_group *group)
{
	struct jit_linear_slow_path *path;

	/* Each slow path takes one memory site; more sites than that split the
	 * region, as an overfull stub table does. */
	if (miss == NULL || builder->failed ||
	    builder->linear_slow_path_count >= JIT_MAX_MEMORY_SITES) {
		builder->failed = true;
		return false;
	}
	path = &builder->linear_slow_paths[builder->linear_slow_path_count++];
	*path = (struct jit_linear_slow_path) {
		.miss = miss,
		.continuation = builder->cursor,
		.instructions = instructions,
		.instruction = instruction,
		.guest_pc = guest_pc,
		.kind = (uint8_t)kind,
		.executed_before = (uint8_t)executed_before,
	};
	if (group != NULL) {
		path->group_min_offset = (int16_t)group->min_offset;
		path->group_max_end = (int16_t)group->max_end;
		path->group_count = (uint8_t)group->count;
		path->group_alignment_mask = (uint8_t)group->alignment_mask;
	}
	return true;
}

static bool jit_emit_linear_load(struct jit_builder *builder,
				 const struct jit_register_map *map,
				 uint32_t instruction, uint32_t guest_pc,
				 uint32_t executed_before)
{
	uint32_t rd = (instruction >> 7) & 0x1fu;
	uint32_t source = jit_guest_source(builder, map,
					   (instruction >> 15) & 0x1fu, RV_T0);
	uint32_t destination = jit_guest_destination(map, rd, RV_T2);
	uint32_t *miss = jit_emit_linear_check(builder, source, RV_T1);

	jit_emit(builder, rv_i(0x03u, destination, (instruction >> 12) & 7u,
			       RV_T1, jit_i_immediate(instruction)));
	jit_commit_unmapped_destination(builder, map, rd, destination);
	return jit_defer_linear_slow_path(builder, JIT_LINEAR_LOAD, miss,
					  NULL, instruction, guest_pc,
					  executed_before, NULL);
}

/* A linear-map store is ordinary RAM: it needs neither the framebuffer
 * dirty report (the aperture is device memory, never in the window) nor
 * rv32_native_jit_note_write_page.  The interpreter's own linear stores skip
 * that report too; Linux rewrites kernel text only through the text-poke
 * fixmap, outside the window, whose page-cache store still reports it. */
static bool jit_emit_linear_store(struct jit_builder *builder,
				  const struct jit_register_map *map,
				  uint32_t instruction, uint32_t guest_pc,
				  uint32_t executed_before)
{
	uint32_t source = jit_guest_source(builder, map,
					   (instruction >> 15) & 0x1fu, RV_T0);
	uint32_t *miss = jit_emit_linear_check(builder, source, RV_T1);
	/* t0 is free once the check passed; the slow path reloads both. */
	uint32_t value = jit_guest_source(builder, map,
					  (instruction >> 20) & 0x1fu, RV_T0);

	jit_emit(builder, rv_s(0x23u, (instruction >> 12) & 7u, RV_T1, value,
			       jit_s_immediate(instruction)));
	return jit_defer_linear_slow_path(builder, JIT_LINEAR_STORE, miss,
					  NULL, instruction, guest_pc,
					  executed_before, NULL);
}

/* One check covers the whole group: every member's displacement is a
 * 12-bit immediate.  t0 holds the host base, as after the page-cache guard. */
static bool jit_emit_linear_load_group(
	struct jit_builder *builder, const struct jit_register_map *map,
	const uint32_t *instructions, uint32_t guest_pc,
	uint32_t executed_before, const struct jit_memory_group *group)
{
	uint32_t source = jit_guest_source(builder, map,
					   (instructions[0] >> 15) & 0x1fu, RV_T0);
	uint32_t *miss = jit_emit_linear_check(builder, source, RV_T0);

	for (uint32_t i = 0; i < group->count; i++) {
		uint32_t instruction = instructions[i];
		uint32_t rd = (instruction >> 7) & 0x1fu;
		uint32_t destination = jit_guest_destination(map, rd, RV_T2);

		jit_emit(builder, rv_i(0x03u, destination,
				       (instruction >> 12) & 7u, RV_T0,
				       jit_i_immediate(instruction)));
		jit_commit_unmapped_destination(builder, map, rd, destination);
	}
	return jit_defer_linear_slow_path(builder, JIT_LINEAR_LOAD_GROUP, miss,
					  instructions, instructions[0],
					  guest_pc, executed_before, group);
}

static bool jit_emit_linear_store_group(
	struct jit_builder *builder, const struct jit_register_map *map,
	const uint32_t *instructions, uint32_t guest_pc,
	uint32_t executed_before, const struct jit_memory_group *group)
{
	uint32_t source = jit_guest_source(builder, map,
					   (instructions[0] >> 15) & 0x1fu, RV_T0);
	uint32_t *miss = jit_emit_linear_check(builder, source, RV_T0);

	for (uint32_t i = 0; i < group->count; i++) {
		uint32_t instruction = instructions[i];
		uint32_t value = jit_guest_source(builder, map,
						  (instruction >> 20) & 0x1fu,
						  RV_T2);

		jit_emit(builder, rv_s(0x23u, (instruction >> 12) & 7u, RV_T0,
				       value, jit_s_immediate(instruction)));
	}
	return jit_defer_linear_slow_path(builder, JIT_LINEAR_STORE_GROUP, miss,
					  instructions, instructions[0],
					  guest_pc, executed_before, group);
}

/* a4 becomes the stream's host base.  The slow path is the page-cache guard
 * and, for stores, its dirty report, which the page-cache form makes after
 * the first store: nothing reads the report before the region returns, and
 * a passed guard means every selected store can complete. */
static bool jit_emit_linear_persistent_guard(
	struct jit_builder *builder, const struct jit_register_map *map,
	const struct jit_persistent_memory_group *group,
	uint32_t instruction, uint32_t guest_pc, uint32_t executed_before)
{
	uint32_t source = jit_guest_source(builder, map, group->base, RV_T0);
	uint32_t *miss = jit_emit_linear_check(builder, source, RV_A4);

	return jit_defer_linear_slow_path(builder, JIT_LINEAR_PERSISTENT, miss,
					  NULL, instruction, guest_pc,
					  executed_before, NULL);
}

/* Out of line, after the region's exits: each linear miss runs the code the
 * page-cache emitter would have placed inline, from the guest registers the
 * check left untouched, then jumps back.  The emitters allocate their memory
 * sites and stubs here, before the stub tables are emitted. */
static bool jit_emit_linear_slow_paths(struct jit_builder *builder,
	const struct jit_register_map *map, struct jit_cache_entry *entry,
	struct rv32_native_jit_memory_site *memory_sites,
	const struct jit_persistent_memory_group *persistent)
{
	for (uint32_t index = 0; index < builder->linear_slow_path_count;
	     index++) {
		const struct jit_linear_slow_path *path =
			&builder->linear_slow_paths[index];
		const struct jit_memory_group group = {
			.count = path->group_count,
			.min_offset = path->group_min_offset,
			.max_end = path->group_max_end,
			.alignment_mask = path->group_alignment_mask,
		};
		uint32_t *resume;
		bool okay;

		if (entry->memory_site_count >= JIT_MAX_MEMORY_SITES) {
			builder->failed = true;
			return false;
		}
		if (!jit_patch_branch(path->miss, builder->cursor, 7u, RV_T1,
				      builder->linear_limit_register))
			return false;
		switch (path->kind) {
		case JIT_LINEAR_LOAD:
			okay = jit_emit_load(builder, map, entry, memory_sites,
					     path->instruction, path->guest_pc,
					     path->executed_before, NULL);
			break;
		case JIT_LINEAR_STORE:
			okay = jit_emit_store(builder, map, entry, memory_sites,
					      path->instruction, path->guest_pc,
					      path->executed_before, NULL);
			break;
		case JIT_LINEAR_LOAD_GROUP:
			okay = jit_emit_load_group(builder, map, entry,
				memory_sites, path->instructions, path->guest_pc,
				path->executed_before, &group);
			break;
		case JIT_LINEAR_STORE_GROUP:
			okay = jit_emit_store_group(builder, map, entry,
				memory_sites, path->instructions, path->guest_pc,
				path->executed_before, &group);
			break;
		case JIT_LINEAR_PERSISTENT:
			okay = jit_emit_persistent_memory_guard(builder, map,
				entry, memory_sites, persistent, path->instruction,
				path->guest_pc, path->executed_before) &&
				(!persistent->store ||
				 jit_emit_linear_dirty_page(builder,
					persistent->memory.count,
					persistent->total_bytes));
			break;
		default:
			okay = false;
			break;
		}
		if (!okay)
			return false;
		resume = builder->cursor;
		jit_emit(builder, 0u); /* j back after the inline access */
		if (builder->failed ||
		    !jit_patch_unconditional(resume, path->continuation))
			return false;
	}
	return !builder->failed;
}

static bool jit_emit_persistent_memory_access(
	struct jit_builder *builder, const struct jit_register_map *map,
	struct jit_cache_entry *entry,
	struct rv32_native_jit_memory_site *memory_sites,
	const struct jit_persistent_memory_group *group, uint32_t index,
	uint32_t instruction, uint32_t guest_pc, uint32_t executed_before,
	bool guard_hoisted, bool linear)
{
	uint32_t funct3 = (instruction >> 12) & 7u;

	if (index == group->first && !guard_hoisted &&
	    !(linear ?
	      jit_emit_linear_persistent_guard(builder, map, group,
					      instruction, guest_pc,
					      executed_before) :
	      jit_emit_persistent_memory_guard(
		    builder, map, entry, memory_sites, group, instruction,
		    guest_pc, executed_before)))
		return false;

	if (!group->store) {
		uint32_t rd = (instruction >> 7) & 0x1fu;
		uint32_t destination = jit_guest_destination(map, rd, RV_T2);

		jit_emit(builder, rv_i(0x03u, destination, funct3, RV_A4,
				       jit_i_immediate(instruction)));
		jit_commit_unmapped_destination(builder, map, rd, destination);
	} else {
		uint32_t rs2 = (instruction >> 20) & 0x1fu;
		uint32_t value = jit_guest_source(builder, map, rs2, RV_T2);

		jit_emit(builder, rv_s(0x23u, funct3, RV_A4, value,
				       jit_s_immediate(instruction)));
		/* The aggregate guard proves every selected store touches this one
		 * translated page, so one report covers them all.  A hoisted guard
		 * reports in the preamble, as t1 is clobbered inside the loop, and a
		 * linear-map guard in its slow path. */
		if (index == group->first && !guard_hoisted && !linear &&
		    !jit_emit_linear_dirty_page(builder, group->memory.count,
						group->total_bytes))
			return false;
	}
	return !builder->failed;
}

static uint32_t jit_inverse_branch_funct3(uint32_t funct3)
{
	switch (funct3) {
	case 0u: return 1u; /* BEQ -> BNE */
	case 1u: return 0u; /* BNE -> BEQ */
	case 4u: return 5u; /* BLT -> BGE */
	case 5u: return 4u; /* BGE -> BLT */
	case 6u: return 7u; /* BLTU -> BGEU */
	case 7u: return 6u; /* BGEU -> BLTU */
	default: return UINT32_MAX;
	}
}

static bool jit_emit_branch_exit(struct jit_builder *builder,
				 const struct jit_register_map *map,
				 uint32_t instruction, uint32_t guest_pc)
{
	uint32_t rs1 = (instruction >> 15) & 0x1fu;
	uint32_t rs2 = (instruction >> 20) & 0x1fu;
	uint32_t funct3 = (instruction >> 12) & 7u;
	uint32_t inverse = jit_inverse_branch_funct3(funct3);
	uint32_t source1;
	uint32_t source2;
	uint32_t *not_taken_patch;

	if (inverse == UINT32_MAX)
		return false;
	source1 = jit_guest_source(builder, map, rs1, RV_T0);
	if (rs2 == rs1)
		source2 = source1;
	else
		source2 = jit_guest_source(builder, map, rs2, RV_T1);

	/* Default to the fall-through PC.  The inverse condition skips replacing
	 * it with the taken target. */
	jit_emit_constant(builder, RV_A1, guest_pc + 4u);
	not_taken_patch = builder->cursor;
	jit_emit(builder, 0u);
	jit_emit_constant(builder, RV_A1,
			  guest_pc + (uint32_t)jit_b_immediate(instruction));
	if (builder->failed ||
	    !jit_patch_branch(not_taken_patch, builder->cursor,
			      inverse, source1, source2))
		return false;
	return true;
}

static bool jit_emit_internal_branch(
	struct jit_builder *builder, const struct jit_register_map *map,
	uint32_t instruction, uint32_t target_index,
	struct jit_internal_branch_patch
		patches[JIT_MAX_INTERNAL_BRANCH_PATCHES],
	uint32_t *patch_count)
{
	uint32_t rs1 = (instruction >> 15) & 0x1fu;
	uint32_t rs2 = (instruction >> 20) & 0x1fu;
	uint32_t source1;
	uint32_t source2;

	if (*patch_count >= JIT_MAX_INTERNAL_BRANCH_PATCHES)
		return false;
	source1 = jit_guest_source(builder, map, rs1, RV_T0);
	if (rs2 == rs1)
		source2 = source1;
	else
		source2 = jit_guest_source(builder, map, rs2, RV_T1);
	if (builder->failed)
		return false;

	patches[*patch_count].instruction = builder->cursor;
	patches[*patch_count].target_index = (uint8_t)target_index;
	patches[*patch_count].funct3 =
		(uint8_t)((instruction >> 12) & 7u);
	patches[*patch_count].source1 = (uint8_t)source1;
	patches[*patch_count].source2 = (uint8_t)source2;
	jit_emit(builder, 0u);
	if (builder->failed)
		return false;
	(*patch_count)++;
	return true;
}

/* A taken branch whose target lies beyond the bounded region exits through
 * the shared result/commit path.  Its fall-through stays native, allowing a
 * region to span useful work after a rarely-taken guard. */
static bool jit_emit_branch_side_exit(
	struct jit_builder *builder, const struct jit_register_map *map,
	uint32_t instruction, uint32_t guest_pc)
{
	uint32_t rs1 = (instruction >> 15) & 0x1fu;
	uint32_t rs2 = (instruction >> 20) & 0x1fu;
	uint32_t inverse = jit_inverse_branch_funct3(
		(instruction >> 12) & 7u);
	uint32_t source1;
	uint32_t source2;
	uint32_t *not_taken_patch;

	if (inverse == UINT32_MAX)
		return false;
	source1 = jit_guest_source(builder, map, rs1, RV_T0);
	if (rs2 == rs1)
		source2 = source1;
	else
		source2 = jit_guest_source(builder, map, rs2, RV_T1);

	not_taken_patch = builder->cursor;
	jit_emit(builder, 0u); /* inverse condition skips the taken exit */
	jit_emit_constant(builder, RV_A1,
			  guest_pc + (uint32_t)jit_b_immediate(instruction));
	jit_record_control_exit(builder);
	if (builder->failed ||
	    !jit_patch_branch(not_taken_patch, builder->cursor,
			      inverse, source1, source2))
		return false;
	return true;
}

/* Execute a conditional back-edge to the beginning of this native region
 * without returning through C on every iteration.  The dedicated loop and
 * executed-count registers bound latency and preserve architectural cycle
 * accounting.  No guest address or application identity is involved. */
static bool jit_emit_branch_to_start(struct jit_builder *builder,
				     const struct jit_register_map *map,
				     uint32_t instruction, uint32_t guest_pc,
				     const uint32_t *loop_start)
{
	uint32_t rs1 = (instruction >> 15) & 0x1fu;
	uint32_t rs2 = (instruction >> 20) & 0x1fu;
	uint32_t inverse = jit_inverse_branch_funct3(
		(instruction >> 12) & 7u);
	uint32_t source1;
	uint32_t source2;
	uint32_t *not_taken_patch;
	uint32_t *repeat_patch;
	uint32_t *completed_patch;
	uint32_t *not_taken_target;

	if (inverse == UINT32_MAX)
		return false;
	source1 = jit_guest_source(builder, map, rs1, RV_T0);
	if (rs2 == rs1)
		source2 = source1;
	else
		source2 = jit_guest_source(builder, map, rs2, RV_T1);

	/* Neither exit PC is needed while the loop repeats. Materialize it only
	 * on exit, and branch directly back while the same budget permits it. */
	not_taken_patch = builder->cursor;
	jit_emit(builder, 0u); /* inverse condition -> fall-through exit */

	jit_emit(builder, rv_i(0x13u, JIT_LOOP_REGISTER, 0u,
			       JIT_LOOP_REGISTER, -1));
	repeat_patch = builder->cursor;
	jit_emit(builder, 0u); /* remaining budget -> loop_start */

	jit_emit_constant(builder, RV_A1,
			  guest_pc + (uint32_t)jit_b_immediate(instruction));
	completed_patch = builder->cursor;
	jit_emit(builder, 0u); /* budget exit skips the fall-through PC */
	not_taken_target = builder->cursor;
	jit_emit_constant(builder, RV_A1, guest_pc + 4u);
	if (builder->failed ||
	    !jit_patch_branch(not_taken_patch, not_taken_target,
			      inverse, source1, source2) ||
	    !jit_patch_branch(repeat_patch, loop_start,
			      1u, JIT_LOOP_REGISTER, RV_X0) ||
	    !jit_patch_unconditional(completed_patch, builder->cursor))
		return false;
	return true;
}

/* GCC commonly lowers a loop with an early forward exit to a conditional
 * side exit followed by an unconditional JAL x0 back to the header.  Treat
 * that exact back-edge like the conditional form above: retain mapped guest
 * registers and translated data pages until either the side exit is taken or
 * the architectural service budget is exhausted. */
static bool jit_emit_jal_to_start(struct jit_builder *builder,
				  uint32_t instruction, uint32_t guest_pc,
				  const uint32_t *loop_start)
{
	uint32_t *repeat_patch;

	if (((instruction >> 7) & 0x1fu) != RV_X0)
		return false;
	jit_emit(builder, rv_i(0x13u, JIT_LOOP_REGISTER, 0u,
			       JIT_LOOP_REGISTER, -1));
	repeat_patch = builder->cursor;
	jit_emit(builder, 0u);

	jit_emit_constant(builder, RV_A1,
			  guest_pc + (uint32_t)jit_j_immediate(instruction));
	if (builder->failed ||
	    !jit_patch_branch(repeat_patch, loop_start,
			      1u, JIT_LOOP_REGISTER, RV_X0))
		return false;
	return true;
}

/* A terminal edge can repeat a suffix of the region without rerunning its
 * preheader or committing the register map. a6 holds this entry's remaining
 * architectural budget, captured before read caches may borrow s10/s11.
 * Charge the maximum suffix length before repeating; a forward side exit
 * can only execute fewer instructions. The ordinary dynamic checkpoints
 * still report the exact work performed, including on a memory fault. */
static bool jit_emit_interior_backedge(struct jit_builder *builder,
				      const struct jit_register_map *map,
				      uint32_t instruction, uint32_t guest_pc,
				      const uint32_t *target,
				      uint32_t suffix_instructions)
{
	bool conditional = (instruction & 0x7fu) == 0x63u;
	uint32_t source1 = RV_X0;
	uint32_t source2 = RV_X0;
	uint32_t inverse = 0u;
	uint32_t *not_taken_patch = NULL;
	uint32_t *repeat_patch;
	uint32_t *completed_patch = NULL;
	uint32_t *not_taken_target = NULL;
	uint32_t target_pc;

	if (conditional) {
		uint32_t rs1 = (instruction >> 15) & 0x1fu;
		uint32_t rs2 = (instruction >> 20) & 0x1fu;

		inverse = jit_inverse_branch_funct3((instruction >> 12) & 7u);
		if (inverse == UINT32_MAX)
			return false;
		source1 = jit_guest_source(builder, map, rs1, RV_T0);
		source2 = rs2 == rs1 ? source1 :
			jit_guest_source(builder, map, rs2, RV_T1);
		not_taken_patch = builder->cursor;
		jit_emit(builder, 0u);
		target_pc = guest_pc + (uint32_t)jit_b_immediate(instruction);
	} else {
		if ((instruction & 0x7fu) != 0x6fu ||
		    ((instruction >> 7) & 0x1fu) != RV_X0)
			return false;
		target_pc = guest_pc + (uint32_t)jit_j_immediate(instruction);
	}
	jit_emit(builder, rv_i(0x13u, RV_T0, 0u, JIT_EXECUTED_REGISTER,
			       (int32_t)suffix_instructions));
	repeat_patch = builder->cursor;
	jit_emit(builder, 0u);
	jit_emit_constant(builder, RV_A1, target_pc);
	if (conditional) {
		completed_patch = builder->cursor;
		jit_emit(builder, 0u);
		not_taken_target = builder->cursor;
		jit_emit_constant(builder, RV_A1, guest_pc + 4u);
	}
	return !builder->failed &&
		(!conditional || jit_patch_branch(not_taken_patch, not_taken_target,
						 inverse, source1, source2)) &&
		jit_patch_branch(repeat_patch, target,
				 7u, JIT_LOOP_REGISTER, RV_T0) &&
		(!conditional ||
		 jit_patch_unconditional(completed_patch, builder->cursor));
}

static bool jit_emit_jalr_exit(struct jit_builder *builder,
			       const struct jit_register_map *map,
			       uint32_t instruction, uint32_t guest_pc)
{
	uint32_t rd = (instruction >> 7) & 0x1fu;
	uint32_t rs1 = (instruction >> 15) & 0x1fu;
	uint32_t source = jit_guest_source(builder, map, rs1, RV_T0);

	/* Compute the target before writing rd in case rd aliases rs1. */
	jit_emit(builder, rv_i(0x13u, RV_A1, 0u, source,
			       jit_i_immediate(instruction)));
	jit_emit(builder, rv_i(0x13u, RV_A1, 7u, RV_A1, -2));
	jit_emit_link_value(builder, map, rd, guest_pc + 4u);
	return !builder->failed;
}

/* A statically paired direct call and canonical return can remain in one
 * bounded native trace.  The guest is still free to alter the link register:
 * guard the computed JALR target and use the ordinary committed side exit if
 * it no longer matches the collected return PC. */
static bool jit_emit_jalr_trace(struct jit_builder *builder,
				const struct jit_register_map *map,
				uint32_t instruction, uint32_t guest_pc,
				uint32_t expected_pc)
{
	uint32_t rd = (instruction >> 7) & 0x1fu;
	uint32_t rs1 = (instruction >> 15) & 0x1fu;
	uint32_t source = jit_guest_source(builder, map, rs1, RV_T0);
	uint32_t *matched_patch;

	/* JALR computes its target before writing rd when the registers alias. */
	jit_emit(builder, rv_i(0x13u, RV_A1, 0u, source,
			       jit_i_immediate(instruction)));
	jit_emit(builder, rv_i(0x13u, RV_A1, 7u, RV_A1, -2));
	jit_emit_link_value(builder, map, rd, guest_pc + 4u);
	jit_emit_constant(builder, RV_T0, expected_pc);
	matched_patch = builder->cursor;
	jit_emit(builder, 0u); /* beq computed target, expected target, continue */
	jit_record_control_exit(builder);
	if (builder->failed ||
	    !jit_patch_branch(matched_patch, builder->cursor,
			      0u, RV_A1, RV_T0))
		return false;
	return true;
}

/* Unmasked directory and warm-table hash.  The directory set is the low bits
 * of the warm index, so a probe computes it once for both tables. */
static inline uint32_t jit_hash_key(uint32_t physical_offset,
				    uint32_t guest_pc)
{
	return (physical_offset >> 2) ^ (physical_offset >> 9) ^
		(guest_pc >> 12) ^ (guest_pc >> 20);
}

_Static_assert(JIT_WARM_SLOTS >= JIT_CACHE_SETS,
	       "the directory set must be the low bits of the warm index");

static uint32_t jit_hash(uint32_t physical_offset, uint32_t guest_pc,
			 uint32_t mask)
{
	return jit_hash_key(physical_offset, guest_pc) & mask;
}

static uint32_t jit_exec_page_index(uint32_t guest_pc)
{
	uint32_t virtual_page = guest_pc >> 12;

	return (virtual_page ^ (virtual_page >> 8)) &
		(JIT_EXEC_PAGE_ENTRIES - 1u);
}

static void jit_invalidate_exec_pages(void)
{
	for (uint32_t i = 0; i < JIT_EXEC_PAGE_ENTRIES; i++)
		jit_exec_pages[i].virtual_page = UINT32_MAX;
	/* Callers have already discarded all links and multi-page regions. */
	memset(jit_exec_history, 0, sizeof(jit_exec_history));
}

static uint32_t jit_exec_history_index(uint32_t virtual_page)
{
	return (virtual_page ^ (virtual_page >> 12)) &
		(JIT_EXEC_HISTORY_BITS - 1u);
}

/* Remember only mappings that have already passed MiniRV32's ordinary
 * ACCESS_EXEC translation.  A cross-page trace may consume such a mapping,
 * but it never guesses physical contiguity or performs its own page walk. */
static void jit_cache_exec_page(uint32_t physical_offset, uint32_t guest_pc)
{
	uint32_t page_offset = guest_pc & 0xfffu;
	uint32_t physical_page_offset;
	uint32_t virtual_page = guest_pc >> 12;
	uint32_t tag = virtual_page | jit_exec_privilege_tag;
	struct jit_exec_page *page;

	if (physical_offset < page_offset)
		return;
	physical_page_offset = physical_offset - page_offset;
	page = &jit_exec_pages[jit_exec_page_index(guest_pc)];
	/* The emulator and its native dispatcher own this cache on one hart. Avoid
	 * a release fence and two stores when the overwhelmingly common mapping is
	 * already current; no CPU1 worker reads or writes these entries. */
	if (page->virtual_page == tag &&
	    page->physical_page_offset == physical_page_offset)
		return;
	/* Linear-map kernel text is never fenced; leaving it out of the history
	 * avoids link flushes from hash collisions with user pages. */
	if (jit_exec_privilege_tag == 0u ||
	    guest_pc - jit_kernel_linear_base >= jit_kernel_linear_size) {
		uint32_t history_index = jit_exec_history_index(virtual_page);

		jit_exec_history[history_index >> 5] |=
			UINT32_C(1) << (history_index & 31u);
	}
	page->physical_page_offset = physical_page_offset;
	page->virtual_page = tag;
}

/* Whether the cached execute mapping of guest_pc's page is already this one. */
static inline bool jit_exec_page_current(uint32_t physical_offset,
					 uint32_t guest_pc)
{
	const struct jit_exec_page *page =
		&jit_exec_pages[jit_exec_page_index(guest_pc)];

	return page->virtual_page ==
		((guest_pc >> 12) | jit_exec_privilege_tag) &&
	       page->physical_page_offset ==
		physical_offset - (guest_pc & 0xfffu);
}

/* jit_cache_exec_page, with the common already-current mapping checked
 * inline. */
static inline void jit_note_exec_page(uint32_t physical_offset,
				      uint32_t guest_pc)
{
	if (!jit_exec_page_current(physical_offset, guest_pc))
		jit_cache_exec_page(physical_offset, guest_pc);
}

static bool jit_resolve_exec_offset(uint32_t guest_pc,
				    uint32_t *physical_offset)
{
	const struct jit_exec_page *page =
		&jit_exec_pages[jit_exec_page_index(guest_pc)];

	if (page->virtual_page != ((guest_pc >> 12) | jit_exec_privilege_tag))
		return false;
	*physical_offset = page->physical_page_offset + (guest_pc & 0xfffu);
	return true;
}

/* Same-page regions survive TLB and permission changes because lookup uses
 * the freshly translated physical address.  Links and multi-page regions
 * depend on cached mappings and go at each TLB epoch.  A privilege change
 * needs neither: U and S never execute the same page. */
static void jit_invalidate_cross_page_entries(void)
{
	jit_clear_all_links();
	if (!jit_has_cross_page_entries)
		return;
	for (uint32_t i = 0; i < JIT_CACHE_SLOTS; i++) {
		if (!jit_entries[i].spans_pages)
			continue;
		jit_forget_entry(i);
		jit_entries[i].state = JIT_ENTRY_EMPTY;
		jit_entries[i].code_units = 0u;
		jit_entries[i].spans_pages = 0u;
	}
	jit_has_cross_page_entries = false;
}

static uint32_t jit_find_collected_pc(const uint32_t *guest_pcs,
				      uint32_t count, uint32_t guest_pc)
{
	for (uint32_t i = 0; i < count; i++) {
		if (guest_pcs[i] == guest_pc)
			return i;
	}
	return UINT32_MAX;
}

#if CONFIG_RV32_GUEST_HLE
static bool jit_hle_callee(uint32_t guest_pc, uint32_t physical_offset,
			   uint32_t instruction, const uint8_t *guest_ram);
#else
#define jit_hle_callee(guest_pc, physical_offset, instruction, guest_ram) false
#endif

static bool jit_collect_region(const uint8_t *guest_ram,
			      uint32_t physical_offset,
			      uint32_t guest_pc,
			      uint32_t instruction_limit,
			      uint32_t instructions[JIT_MAX_GUEST_INSTRUCTIONS],
			      uint32_t guest_pcs[JIT_MAX_GUEST_INSTRUCTIONS],
			      enum jit_instruction_kind
				kinds[JIT_MAX_GUEST_INSTRUCTIONS],
			      uint32_t *collected)
{
	uint32_t limit = instruction_limit;
	uint32_t count = 0;
	uint32_t cursor_pc = guest_pc;
	uint32_t start_physical_page =
		physical_offset - (guest_pc & 0xfffu);
	uint32_t start_guest_page = guest_pc & ~0xfffu;
	uint32_t call_depth = 0u;
	bool page_edge = false;
	bool following_call = false;
	uint32_t *return_pcs = jit_compile_workspace.trace_return_pcs;
	uint8_t *link_registers =
		jit_compile_workspace.trace_link_registers;

	if (limit > JIT_MAX_GUEST_INSTRUCTIONS)
		limit = JIT_MAX_GUEST_INSTRUCTIONS;
	while (count < limit) {
		uint32_t instruction;
		uint32_t cursor_offset;
		enum jit_instruction_kind kind;

		if (jit_find_collected_pc(guest_pcs, count, cursor_pc) != UINT32_MAX)
			break;
		if ((cursor_pc & ~0xfffu) == start_guest_page) {
			cursor_offset = start_physical_page + (cursor_pc & 0xfffu);
		} else if (!jit_cross_page_trace_self_test ||
			   !jit_resolve_exec_offset(cursor_pc, &cursor_offset)) {
			page_edge = true;
			break;
		}
		memcpy(&instruction,
		       guest_ram + cursor_offset,
		       sizeof(instruction));
		/* A jump into a host routine ends the trace at the jump, so the
		 * region's exit lands on the routine's entry and the host runs it
		 * instead of the guest's own code inlined here.  The callee is
		 * identified here as well: one reached only from native code is never
		 * probed. */
		if (following_call) {
			following_call = false;
			if (jit_hle_callee(cursor_pc, cursor_offset, instruction,
					   guest_ram))
				break;
		}
		kind = jit_instruction_kind(instruction);
		if (kind == JIT_INSN_UNSUPPORTED) {
#if CONFIG_RV32_HOST_PERF_STATS
			jit_stats.unsupported_cuts[
				jit_unsupported_class(instruction)]++;
#endif
			break;
		}
		instructions[count] = instruction;
		guest_pcs[count] = cursor_pc;
		kinds[count] = kind;
		count++;
		/* A JAL has a statically known target. Follow both ordinary jumps and
		 * bounded direct calls when the target page was already execute-validated;
		 * a nonzero link register records the architectural return PC for a later
		 * guarded canonical JALR.  A missing mapping stops collection on the next
		 * iteration and leaves this JAL as the ordinary terminal instruction. */
		if (kind == JIT_INSN_JAL && count < limit) {
			uint32_t rd = (instruction >> 7) & 0x1fu;
			uint32_t target = cursor_pc +
				(uint32_t)jit_j_immediate(instruction);

			if ((target & 3u) == 0u &&
			    jit_find_collected_pc(guest_pcs, count, target) ==
				UINT32_MAX &&
			    (rd == RV_X0 || call_depth < JIT_MAX_TRACE_CALL_DEPTH)) {
				if (rd != RV_X0) {
					return_pcs[call_depth] = cursor_pc + 4u;
					link_registers[call_depth] = (uint8_t)rd;
					call_depth++;
				}
				following_call = true;
				cursor_pc = target;
				continue;
			}
		}
		/* Pair only the canonical indirect return form with the most recent
		 * direct call.  Code generation checks the live target before following
		 * this edge, so a modified/restored link register remains correct. */
		if (kind == JIT_INSN_JALR && count < limit && call_depth != 0u &&
		    ((instruction >> 7) & 0x1fu) == RV_X0 &&
		    ((instruction >> 15) & 0x1fu) ==
			link_registers[call_depth - 1u] &&
		    jit_i_immediate(instruction) == 0) {
			uint32_t target = return_pcs[call_depth - 1u];

			if ((target & 3u) == 0u &&
			    jit_find_collected_pc(guest_pcs, count, target) ==
				UINT32_MAX) {
				call_depth--;
				cursor_pc = target;
				continue;
			}
		}
		/* Keep collecting the lexical fall-through after a forward conditional
		 * branch. Code generation either links its target inside this trace or
		 * emits a side exit. Unfollowed transfers and backward conditions remain
		 * terminal. */
		if (kind == JIT_INSN_JAL || kind == JIT_INSN_JALR ||
		    (kind == JIT_INSN_BRANCH && jit_b_immediate(instruction) <= 0))
			break;
		cursor_pc += sizeof(uint32_t);
	}
	*collected = count;
	/* Shorter code still compiles when it ends in a jump or branch or at a
	 * page edge: call sites, return landings, two-instruction leaves, switch
	 * cases and the last words of a page sit between native regions, and
	 * interpreting them costs a round trip out of native code on every
	 * visit.  Code cut short by an unsupported instruction would go straight
	 * back to the interpreter. */
	return count >= JIT_MIN_GUEST_INSTRUCTIONS ||
	       (count != 0u && (page_edge ||
				jit_instruction_terminal(kinds[count - 1u])));
}

static bool jit_candidate_loops_to_start(const uint8_t *guest_ram,
					 uint32_t physical_offset,
					 uint32_t guest_pc,
					 uint32_t instruction_limit)
{
	uint32_t count;
	uint32_t branch_pc;

	if (!jit_collect_region(guest_ram, physical_offset, guest_pc,
				instruction_limit,
				jit_compile_workspace.instructions,
				jit_compile_workspace.guest_pcs,
				jit_compile_workspace.kinds, &count))
		return false;
	branch_pc = jit_compile_workspace.guest_pcs[count - 1u];
	if (jit_compile_workspace.kinds[count - 1u] == JIT_INSN_BRANCH)
		return branch_pc + (uint32_t)jit_b_immediate(
			jit_compile_workspace.instructions[count - 1u]) == guest_pc;
	if (jit_compile_workspace.kinds[count - 1u] == JIT_INSN_JAL &&
	    ((jit_compile_workspace.instructions[count - 1u] >> 7) & 0x1fu) ==
		RV_X0)
		return branch_pc + (uint32_t)jit_j_immediate(
			jit_compile_workspace.instructions[count - 1u]) == guest_pc;
	return false;
}

/* Compile-failure stages.  jit_last_rejection keeps the latest one so the
 * caller can retry a trace that failed only for its length. */
enum jit_reject_reason {
	JIT_REJECT_COLLECT,
	JIT_REJECT_CODE_SIZE,
	JIT_REJECT_CODEGEN,
	JIT_REJECT_PUBLISH,
};
static enum jit_reject_reason jit_last_rejection;

/* A collected trace, the input to code generation.  Collection reads guest
 * RAM and the exec mappings, so it runs on the emulator core; generation
 * works on this copy alone. */
struct jit_trace {
	uint32_t physical_offset;
	uint32_t guest_pc;
	uint32_t count;
	/* jit_collect_region's verdict for all count instructions. */
	bool collectable;
	uint32_t instructions[JIT_MAX_GUEST_INSTRUCTIONS];
	uint32_t guest_pcs[JIT_MAX_GUEST_INSTRUCTIONS];
	enum jit_instruction_kind kinds[JIT_MAX_GUEST_INSTRUCTIONS];
};

/* Generated words that depend on the directory slot, which is chosen when
 * the region is installed.  Generation emits them for slot 0. */
enum jit_slot_patch_kind {
	JIT_SLOT_PATCH_NUMBER,		/* lui rd, %hi(slot); addi rd, rd, %lo(slot) */
	JIT_SLOT_PATCH_LINK_UPPER,	/* lui t0, %hi(&jit_links[slot]) */
	JIT_SLOT_PATCH_LINK_LOWER,	/* lw rd, %lo(&jit_links[slot])+field(t0) */
	JIT_SLOT_PATCH_PROFILE_UPPER,	/* lui t0, %hi(profile counter) */
	JIT_SLOT_PATCH_PROFILE_LOWER,	/* addi t0, t0, %lo(profile counter) */
	JIT_SLOT_PATCH_ENTERED_UPPER,	/* lui t0, %hi(&jit_entry_entered[slot]) */
	JIT_SLOT_PATCH_ENTERED_STORE,	/* sb t2, %lo(&jit_entry_entered[slot])(t0) */
};

struct jit_slot_patch {
	uint16_t word;
	uint8_t kind;
	uint8_t field;
};

#define JIT_MAX_SLOT_PATCHES 10u
/* jit_unit_owner holds slot + 1 in 16 bits. */
_Static_assert(JIT_CACHE_SLOTS <= 65535u,
	       "a slot number must fit the arena owner map");

/* One region compilation.  The caller fills the inputs and buffers;
 * jit_codegen fills the results, and jit_install_generated publishes them. */
struct jit_codegen {
	const struct jit_trace *trace;
	uint32_t instruction_limit;
	const uint32_t *entry_values;
	bool refill;
	struct jit_compile_workspace *workspace;
	uint32_t *code;
	struct rv32_native_jit_memory_site *memory_sites;
	/* Only memory_site_count is used: the emitters count sites in it. */
	struct jit_cache_entry scratch;
	uint32_t used_words;
	uint32_t count;
	uint32_t traced_jump_count;
	bool loops_to_start;
	bool spans_pages;
	/* Generated at a halved length after a code-size rejection. */
	bool split;
	enum jit_reject_reason rejection;
	uint8_t slot_patch_count;
	struct jit_slot_patch slot_patches[JIT_MAX_SLOT_PATCHES];
};

static void jit_record_slot_patch(struct jit_codegen *gen,
				  struct jit_builder *builder,
				  enum jit_slot_patch_kind kind, uint32_t field)
{
	if (gen->slot_patch_count >= JIT_MAX_SLOT_PATCHES) {
		builder->failed = true;
		return;
	}
	gen->slot_patches[gen->slot_patch_count++] = (struct jit_slot_patch) {
		.word = (uint16_t)(builder->cursor - builder->start),
		.kind = (uint8_t)kind,
		.field = (uint8_t)field,
	};
}

/* lui host_reg, %hi(slot); addi host_reg, host_reg, %lo(slot): any
 * directory size, the same two words whatever slot the region gets. */
static void jit_emit_slot_number(struct jit_codegen *gen,
				 struct jit_builder *builder, uint32_t host_reg)
{
	jit_record_slot_patch(gen, builder, JIT_SLOT_PATCH_NUMBER, 0u);
	jit_emit(builder, (host_reg << 7) | 0x37u);
	jit_emit(builder, rv_i(0x13u, host_reg, 0u, host_reg, 0));
}

static void jit_patch_slot_words(struct jit_codegen *gen, uint32_t slot)
{
	uint32_t link = (uint32_t)(uintptr_t)&jit_links[slot];
	int32_t link_low = (int32_t)(link << 20) >> 20;
	uint32_t entered = (uint32_t)(uintptr_t)&jit_entry_entered[slot];
	int32_t entered_low = (int32_t)(entered << 20) >> 20;
#if CONFIG_RV32_GUEST_PROFILE
	uint32_t counter = (uint32_t)(uintptr_t)&rv32_profile_region_entries[slot];
	int32_t counter_low = (int32_t)(counter << 20) >> 20;
#endif

	for (uint32_t i = 0; i < gen->slot_patch_count; i++) {
		const struct jit_slot_patch *patch = &gen->slot_patches[i];
		uint32_t *word = &gen->code[patch->word];

		switch (patch->kind) {
		case JIT_SLOT_PATCH_NUMBER: {
			int32_t slot_low = (int32_t)(slot << 20) >> 20;

			word[0] = (word[0] & 0x00000fffu) |
				((slot - (uint32_t)slot_low) & 0xfffff000u);
			word[1] = (word[1] & 0x000fffffu) |
				((uint32_t)slot_low << 20);
			break;
		}
		case JIT_SLOT_PATCH_LINK_UPPER:
			*word = (*word & 0x00000fffu) |
				((link - (uint32_t)link_low) & 0xfffff000u);
			break;
		case JIT_SLOT_PATCH_LINK_LOWER:
			*word = (*word & 0x000fffffu) |
				((uint32_t)(link_low + (int32_t)patch->field) << 20);
			break;
		case JIT_SLOT_PATCH_ENTERED_UPPER:
			*word = (*word & 0x00000fffu) |
				((entered - (uint32_t)entered_low) & 0xfffff000u);
			break;
		case JIT_SLOT_PATCH_ENTERED_STORE:
			/* S-type immediate: bits 11:5 above rs2, bits 4:0 above the opcode. */
			*word = (*word & 0x01fff07fu) |
				(((uint32_t)entered_low & 0xfe0u) << 20) |
				(((uint32_t)entered_low & 0x1fu) << 7);
			break;
#if CONFIG_RV32_GUEST_PROFILE
		case JIT_SLOT_PATCH_PROFILE_UPPER:
			*word = (*word & 0x00000fffu) |
				((counter - (uint32_t)counter_low) & 0xfffff000u);
			break;
		case JIT_SLOT_PATCH_PROFILE_LOWER:
			*word = (*word & 0x000fffffu) |
				((uint32_t)counter_low << 20);
			break;
#endif
		default:
			break;
		}
	}
}

/* A region whose head is a kernel linear-map PC runs in S-mode: the same
 * code entered in U-mode, or with paging off, finds a zero limit in its bank
 * and takes every slow path.  Loop-cached accesses keep their loop-cache
 * translation, which serves user memory as well.  The limit, host base and
 * window offset live in callee-saved registers this region does not
 * otherwise use (rv32_native_jit_enter saves them all); without two of them
 * the region keeps the page cache alone. */
/* Whether an access off guest register base should try the linear map
 * first.  Entry values are a hint only: a base the region never writes that
 * held an address outside the window (a user pointer for get_user, a
 * vmalloc or device mapping, or x0) keeps the inline page-cache path, as it
 * will likely miss again.  Either choice is correct for any value. */
static bool jit_linear_expected(const struct jit_group_hint *hint,
				uint32_t base)
{
	if (base == 0u)
		return false;
	if (hint->entry_values == NULL ||
	    (hint->invariant_registers & (UINT32_C(1) << base)) == 0u)
		return true;
	return hint->entry_values[base] - jit_kernel_linear_base -
		JIT_LINEAR_LOW_MARGIN < jit_kernel_linear_size -
		JIT_LINEAR_LOW_MARGIN - JIT_LINEAR_HIGH_MARGIN;
}

static void jit_choose_linear_registers(struct jit_builder *builder,
	const struct jit_register_map *map, const uint32_t *instructions,
	const enum jit_instruction_kind *kinds, uint32_t count,
	uint32_t guest_pc, const struct jit_group_hint *hint,
	const struct jit_loop_read_cache *read_caches,
	uint32_t read_cache_count, const struct jit_loop_write_cache *write_cache)
{
	static const uint8_t candidates[] = {
		RV_S0, RV_S1, RV_S9, RV_S2 + 6u, RV_S2 + 5u, RV_S2 + 4u,
		RV_S2 + 3u, RV_S2 + 2u, RV_S2 + 1u, RV_S2,
	};
	uint32_t used = (UINT32_C(1) << JIT_CHAIN_EXECUTED_REGISTER) |
		(UINT32_C(1) << JIT_CHAIN_LIMIT_REGISTER);
	uint8_t found[3];
	uint32_t found_count = 0u;
	bool accesses = false;

	builder->linear = false;
	if (!jit_kernel_linear_data || jit_linear_register_budget < 2u ||
	    guest_pc - jit_kernel_linear_base >= jit_kernel_linear_size)
		return;
	for (uint32_t i = 0; i < count && !accesses; i++) {
		accesses = ((kinds[i] == JIT_INSN_LOAD &&
			     jit_loop_read_cache_for(read_caches,
						     read_cache_count, i) == NULL) ||
			    (kinds[i] == JIT_INSN_STORE &&
			     jit_loop_write_cache_for(write_cache, i) == NULL)) &&
			jit_linear_expected(hint, (instructions[i] >> 15) & 0x1fu);
	}
	if (!accesses)
		return;
	for (uint32_t i = 0; i < map->count; i++)
		used |= UINT32_C(1) << map->host_register[i];
	for (uint32_t i = 0; i < read_cache_count; i++) {
		if (!read_caches[i].valid)
			continue;
		used |= (UINT32_C(1) << read_caches[i].tag_register) |
			(UINT32_C(1) << read_caches[i].bias_register);
	}
	if (write_cache->valid)
		used |= UINT32_C(1) << JIT_LOOP_WRITE_TAG_REGISTER;
	for (uint32_t i = 0; i < sizeof(candidates) &&
	     found_count < jit_linear_register_budget && found_count < 3u; i++) {
		if ((used & (UINT32_C(1) << candidates[i])) == 0u)
			found[found_count++] = candidates[i];
	}
	if (found_count < 2u)
		return;
	builder->linear = true;
	builder->linear_limit_register = found[0];
	builder->linear_host_register = found[1];
	builder->linear_offset_register = found_count > 2u ? found[2] : RV_X0;
	builder->linear_offset =
		0u - jit_kernel_linear_base - JIT_LINEAR_LOW_MARGIN;
}

/* jit_collect_region's verdict for the first count instructions: a smaller
 * limit collects exactly that prefix, cut by the limit, not a page edge. */
static bool jit_trace_prefix_collectable(const struct jit_trace *trace,
					 uint32_t count)
{
	if (count == trace->count)
		return trace->collectable;
	return count >= JIT_MIN_GUEST_INSTRUCTIONS ||
	       (count != 0u && jit_instruction_terminal(trace->kinds[count - 1u]));
}

/* Generates the region for gen->trace into gen->code.  It reads only the
 * trace, gen's buffers and values fixed at init (CSR layout, stub and
 * link-table addresses), so it may run on either core.  The words that
 * depend on the directory slot are left for jit_install_generated. */
static __attribute__((noinline)) bool jit_codegen(struct jit_codegen *gen)
{
	const struct jit_trace *trace = gen->trace;
	const uint32_t guest_pc = trace->guest_pc;
	const uint32_t *entry_values = gen->entry_values;
	struct jit_compile_workspace *workspace = gen->workspace;
	struct jit_cache_entry *entry = &gen->scratch;
	struct rv32_native_jit_memory_site *memory_sites = gen->memory_sites;
	const uint32_t *instructions = trace->instructions;
	const uint32_t *guest_pcs = trace->guest_pcs;
	const enum jit_instruction_kind *kinds = trace->kinds;
	bool *leaders = workspace->leaders;
	uint32_t **native_labels = workspace->native_labels;
	struct jit_internal_branch_patch *internal_branch_patches =
		workspace->internal_branch_patches;
	struct jit_register_map map;
	struct jit_group_hint group_hint = { .entry_values = entry_values };
	struct jit_persistent_memory_group persistent_memory;
	struct jit_loop_read_cache loop_read_caches[JIT_MAX_LOOP_READ_CACHES];
	struct jit_loop_write_cache loop_write_cache = { 0 };
	struct jit_builder builder = {
		.start = gen->code,
		.cursor = gen->code,
		.end = gen->code + JIT_CODE_WORDS,
		.memory_stubs = workspace->memory_stubs,
		.loop_read_cache_stubs = workspace->loop_read_cache_stubs,
		.loop_write_cache_stubs = workspace->loop_write_cache_stubs,
		.two_page_store_stubs = workspace->two_page_store_stubs,
		.linear_slow_paths = workspace->linear_slow_paths,
		.refill = gen->refill,
	};
	uint32_t count;
	uint32_t loop_read_cache_count = 0u;
	uint32_t internal_branch_patch_count = 0u;
	uint32_t traced_jump_count = 0u;
	uint32_t segment_start = 0u;
	uint32_t loop_target_index = UINT32_MAX;
	uint32_t *loop_start = NULL;
	uint32_t *normal_result_start;
	uint32_t *budget_exit_patch;
	uint32_t *wrapper_entry_patch;
	bool terminal = false;
	bool loops_to_start = false;
	bool repeated = false;
	bool hoist_persistent_memory = false;
	bool chain_registers_clobbered = false;
	bool dynamic_counter_active = false;
	bool spans_pages = false;
	bool linear_persistent = false;
	enum jit_reject_reason rejection = JIT_REJECT_COLLECT;

	entry->memory_site_count = 0;
	gen->slot_patch_count = 0u;
	memset(leaders, 0, sizeof(workspace->leaders));
	memset(native_labels, 0, sizeof(workspace->native_labels));
	count = trace->count < gen->instruction_limit ?
		trace->count : gen->instruction_limit;
	if (!jit_trace_prefix_collectable(trace, count))
		goto rejected;
	for (uint32_t i = 1u; i < count; i++) {
		if ((guest_pcs[i] & ~0xfffu) != (guest_pc & ~0xfffu)) {
			spans_pages = true;
			break;
		}
	}
	rejection = JIT_REJECT_CODEGEN;
	for (uint32_t i = 0; i + 1u < count; i++) {
		if ((kinds[i] == JIT_INSN_JAL &&
		     guest_pcs[i + 1u] == guest_pcs[i] +
			(uint32_t)jit_j_immediate(instructions[i])) ||
		    kinds[i] == JIT_INSN_JALR)
			traced_jump_count++;
	}
	terminal = jit_instruction_terminal(kinds[count - 1u]);
	if (kinds[count - 1u] == JIT_INSN_BRANCH) {
		uint32_t branch_pc = guest_pcs[count - 1u];

		loop_target_index = jit_find_collected_pc(guest_pcs, count,
			branch_pc + (uint32_t)jit_b_immediate(instructions[count - 1u]));
	} else if (kinds[count - 1u] == JIT_INSN_JAL &&
		   ((instructions[count - 1u] >> 7) & 0x1fu) == RV_X0) {
		uint32_t branch_pc = guest_pcs[count - 1u];

		loop_target_index = jit_find_collected_pc(guest_pcs, count,
			branch_pc + (uint32_t)jit_j_immediate(instructions[count - 1u]));
	}
	loops_to_start = loop_target_index == 0u;
	repeated = loop_target_index < count;
	leaders[0] = true;
	if (repeated)
		leaders[loop_target_index] = true;
	for (uint32_t i = 0; i < count; i++) {
		uint32_t instruction_pc;
		uint32_t target_pc;
		uint32_t target_index;

		if (kinds[i] == JIT_INSN_JALR && i + 1u < count) {
			dynamic_counter_active = true;
			continue;
		}
		if (kinds[i] == JIT_INSN_CSR && i + 1u < count &&
		    jit_csr_may_exit(instructions[i])) {
			dynamic_counter_active = true;
			continue;
		}
		if (kinds[i] != JIT_INSN_BRANCH || i + 1u >= count)
			continue;
		/* Any nonterminal branch makes the number of executed instructions
		 * path-dependent, even when its target ultimately leaves the region. */
		dynamic_counter_active = true;
		instruction_pc = guest_pcs[i];
		target_pc = instruction_pc +
			(uint32_t)jit_b_immediate(instructions[i]);
		if ((target_pc & 3u) != 0u)
			continue;
		target_index = jit_find_collected_pc(guest_pcs, count, target_pc);
		if (target_index > i && target_index < count)
			leaders[target_index] = true;
	}
	dynamic_counter_active |= repeated;
	builder.instruction_counter_active = dynamic_counter_active;
	if (entry_values != NULL) {
		for (uint32_t reg = 0u; reg < 32u; reg++) {
			if (!jit_region_writes_register(instructions, kinds, count, reg))
				group_hint.invariant_registers |= 1u << reg;
		}
	}
	persistent_memory = jit_find_persistent_memory_group(
		instructions, kinds, leaders, count, &group_hint);
	/* If the repeated region starts with an invariant memory stream, its guard
	 * has no preceding guest work to preserve on a cold exit.  Translate it
	 * once in the entry preamble and retain the host base in a4 across every
	 * native back edge.  A base modified later in the region is invariant only
	 * within one iteration and must be translated again after each back edge. */
	hoist_persistent_memory = loops_to_start && persistent_memory.valid &&
		persistent_memory.first == 0u &&
		!jit_region_writes_register(instructions, kinds, count,
					    persistent_memory.base);
	if (repeated) {
		loop_read_cache_count = jit_find_loop_read_caches(
			instructions, kinds, leaders, count, &persistent_memory,
			hoist_persistent_memory,
			loop_read_caches, &group_hint);
		loop_write_cache = jit_find_loop_write_cache(
			instructions, kinds, leaders, count, &persistent_memory,
			hoist_persistent_memory, &group_hint);
		for (uint32_t i = 0; i < loop_read_cache_count; i++) {
			struct jit_loop_read_cache *cache = &loop_read_caches[i];
			uint32_t instruction = instructions[cache->instruction_index];

			cache->alignment_invariant =
				cache->instruction_index >= loop_target_index &&
				jit_loop_preserves_alignment(instructions, kinds,
					loop_target_index, count,
					(instruction >> 15) & 0x1fu,
					jit_memory_width((instruction >> 12) & 7u, true));
		}
		if (loop_write_cache.valid) {
			uint32_t instruction =
				instructions[loop_write_cache.instruction_index];

			loop_write_cache.alignment_invariant =
				loop_write_cache.instruction_index >= loop_target_index &&
				jit_loop_preserves_alignment(instructions, kinds,
					loop_target_index, count,
					(instruction >> 15) & 0x1fu,
					jit_memory_width((instruction >> 12) & 7u, false));
		}
	}
	for (uint32_t i = 0; i < loop_read_cache_count; i++) {
		chain_registers_clobbered |=
			loop_read_caches[i].tag_register ==
				JIT_CHAIN_EXECUTED_REGISTER ||
			loop_read_caches[i].bias_register ==
				JIT_CHAIN_EXECUTED_REGISTER ||
			loop_read_caches[i].tag_register ==
				JIT_CHAIN_LIMIT_REGISTER ||
			loop_read_caches[i].bias_register ==
				JIT_CHAIN_LIMIT_REGISTER;
	}
	jit_build_register_map(&map, instructions, kinds, count,
			       repeated, dynamic_counter_active);
	jit_choose_linear_registers(&builder, &map, instructions, kinds, count,
				    guest_pc, &group_hint, loop_read_caches,
				    loop_read_cache_count, &loop_write_cache);
	linear_persistent = builder.linear && persistent_memory.valid &&
		jit_linear_expected(&group_hint, persistent_memory.base);
	memset(gen->code, 0, JIT_CODE_BYTES);

	/* A normal wrapper entry supplies runtime in a0 and has had its budget
	 * checked by the caller.  Native successors retain t6/t5 plus the
	 * cumulative instruction state in s10/s11 and enter after these five
	 * fixed instructions, at the budget check. */
	jit_emit(&builder, rv_i(0x13u, JIT_RUNTIME_REGISTER, 0u,
			       RV_A0, 0));
	jit_emit_load_host(&builder, JIT_GUEST_REGS_REGISTER,
			   JIT_RUNTIME_REGISTER,
			   offsetof(struct rv32_native_jit_runtime, regs));
	jit_emit_load_host(&builder, JIT_CHAIN_EXECUTED_REGISTER,
			   JIT_RUNTIME_REGISTER,
			   offsetof(struct rv32_native_jit_runtime,
				    chain_executed));
	jit_emit_load_host(&builder, JIT_CHAIN_LIMIT_REGISTER,
			   JIT_RUNTIME_REGISTER,
			   offsetof(struct rv32_native_jit_runtime,
				    chain_instruction_limit));
	wrapper_entry_patch = builder.cursor;
	jit_emit(&builder, 0u); /* j past the chain entry's budget check */
	if ((size_t)(builder.cursor - builder.start) * sizeof(uint32_t) !=
	    JIT_NATIVE_CHAIN_ENTRY_BYTES)
		goto rejected;
	/* Links, the jump cache and the dispatch stub enter here without knowing
	 * this region's length.  Stop the chain where the C dispatcher would: when
	 * a full pass could run past the instruction budget. */
	jit_emit(&builder, rv_i(0x13u, RV_T0, 0u, JIT_CHAIN_EXECUTED_REGISTER,
			       (int32_t)count));
	budget_exit_patch = builder.cursor;
	jit_emit(&builder, 0u); /* bltu limit, executed + count, budget exit */
	if (builder.failed ||
	    !jit_patch_unconditional(wrapper_entry_patch, builder.cursor))
		goto rejected;
	/* The second loop-read cache deliberately uses s10/s11. Preserve the
	 * inherited chain count before that region initializes its local tags; its
	 * normal exit restores both registers before following another native edge.
	 * One-cache and non-loop regions keep the state entirely register-resident. */
	if (chain_registers_clobbered)
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER,
				    JIT_CHAIN_EXECUTED_REGISTER,
				    offsetof(struct rv32_native_jit_runtime,
					     chain_executed));
	if (repeated && !loops_to_start)
		jit_emit(&builder, rv_r(0x33u, JIT_LOOP_REGISTER, 0u,
				       JIT_CHAIN_LIMIT_REGISTER,
				       JIT_CHAIN_EXECUTED_REGISTER, 0x20u));
	if (loops_to_start) {
		/* The C dispatcher passes the iteration count.  A native entry
		 * finds zero there and derives the same count from the remaining
		 * budget; either way zero is left for the next entry.  This runs
		 * before the read caches may take s10/s11. */
		uint32_t *have_iterations_patch;

		jit_emit_load_host(&builder, JIT_LOOP_REGISTER,
				   JIT_RUNTIME_REGISTER,
				   offsetof(struct rv32_native_jit_runtime,
					    loop_iterations));
		have_iterations_patch = builder.cursor;
		jit_emit(&builder, 0u); /* bne iterations, zero, have them */
		jit_emit(&builder, rv_r(0x33u, JIT_LOOP_REGISTER, 0u,
				       JIT_CHAIN_LIMIT_REGISTER,
				       JIT_CHAIN_EXECUTED_REGISTER, 0x20u));
		jit_emit_constant(&builder, RV_T0, JIT_LOOP_WAKE_BUDGET);
		jit_emit(&builder, rv_b(6u, JIT_LOOP_REGISTER, RV_T0, 8));
		jit_emit(&builder, rv_i(0x13u, JIT_LOOP_REGISTER, 0u, RV_T0, 0));
		jit_emit(&builder, rv_i(0x13u, RV_T0, 0u, RV_X0, (int32_t)count));
		jit_emit(&builder, rv_r(0x33u, JIT_LOOP_REGISTER, 5u,
				       JIT_LOOP_REGISTER, RV_T0, 1u));
		if (builder.failed ||
		    !jit_patch_branch(have_iterations_patch, builder.cursor,
				      1u, JIT_LOOP_REGISTER, RV_X0))
			goto rejected;
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER, RV_X0,
				    offsetof(struct rv32_native_jit_runtime,
					     loop_iterations));
	}

	/* rv32_native_jit_enter preserves every saved host register once around the
	 * full chain.  Each region only initializes the page-cache tags it uses. */
	for (uint32_t i = 0; i < loop_read_cache_count; i++) {
		/* No valid 32-bit guest address can have this page tag. */
		jit_emit(&builder, rv_i(0x13u,
				       loop_read_caches[i].tag_register,
				       0u, RV_X0, -1));
	}
	if (loop_write_cache.valid)
		jit_emit(&builder, rv_i(0x13u,
				       JIT_LOOP_WRITE_TAG_REGISTER,
				       0u, RV_X0, -1));
	/* Every entry reloads the bank's window: a linked predecessor may have
	 * used these registers for its own guest values. */
	if (builder.linear) {
		jit_emit_load_host(&builder, builder.linear_limit_register,
				   JIT_RUNTIME_REGISTER,
				   offsetof(struct rv32_native_jit_runtime,
					    linear_limit));
		jit_emit_load_host(&builder, builder.linear_host_register,
				   JIT_RUNTIME_REGISTER,
				   offsetof(struct rv32_native_jit_runtime,
					    linear_host));
		if (builder.linear_offset_register != RV_X0)
			jit_emit_constant(&builder,
					  builder.linear_offset_register,
					  builder.linear_offset);
	}
#if RV32_NATIVE_JIT_RUNTIME_COUNTERS
	/* Count every generated entry, including targets reached through a native
	 * tail link that the C dispatcher never sees. */
	jit_emit_load_host(&builder, RV_T0, JIT_RUNTIME_REGISTER,
			   offsetof(struct rv32_native_jit_runtime, chain_blocks));
	jit_emit(&builder, rv_i(0x13u, RV_T0, 0u, RV_T0, 1));
	jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER, RV_T0,
			    offsetof(struct rv32_native_jit_runtime, chain_blocks));
#endif
	for (uint32_t i = 0; i < map.count; i++) {
		if (map.needs_initial[i])
			jit_emit_load_host(&builder, map.host_register[i],
					   JIT_GUEST_REGS_REGISTER,
					   map.guest_for_host[i] *
					   sizeof(uint32_t));
	}
	if (dynamic_counter_active)
		jit_emit(&builder, rv_i(0x13u, JIT_EXECUTED_REGISTER,
				       0u, RV_X0, 0));
	if (hoist_persistent_memory && linear_persistent) {
		if (!jit_emit_linear_persistent_guard(
			    &builder, &map, &persistent_memory, instructions[0],
			    guest_pc, 0u))
			goto rejected;
	} else if (hoist_persistent_memory) {
		if (!jit_emit_persistent_memory_guard(
			    &builder, &map, entry, memory_sites,
			    &persistent_memory, instructions[0], guest_pc, 0u))
			goto rejected;
		if (persistent_memory.store &&
		    !jit_emit_linear_dirty_page(&builder,
						persistent_memory.memory.count,
						persistent_memory.total_bytes))
			goto rejected;
	}
	loop_start = builder.cursor;
#if CONFIG_RV32_GUEST_PROFILE
	/* Count region entries and loop iterations.  t0/t1 carry nothing across
	 * guest-instruction boundaries, and loop caches use s0/s1/s10/s11. */
	if (rv32_profile_region_entries != NULL) {
		jit_record_slot_patch(gen, &builder,
				      JIT_SLOT_PATCH_PROFILE_UPPER, 0u);
		jit_emit(&builder, (RV_T0 << 7) | 0x37u);
		jit_record_slot_patch(gen, &builder,
				      JIT_SLOT_PATCH_PROFILE_LOWER, 0u);
		jit_emit(&builder, rv_i(0x13u, RV_T0, 0u, RV_T0, 0));
		jit_emit_load_host(&builder, RV_T1, RV_T0, 0u);
		jit_emit(&builder, rv_i(0x13u, RV_T1, 0u, RV_T1, 1));
		jit_emit_store_host(&builder, RV_T0, RV_T1, 0u);
	}
#endif

	for (uint32_t i = 0; i < count; i++) {
		uint32_t instruction_pc = guest_pcs[i];
		uint32_t executed_before;
		bool okay;

		/* A branch target skips the lexical fall-through checkpoint and lands
		 * after it.  The dynamic counter is exact for either incoming path. */
		if (dynamic_counter_active && leaders[i] && i > segment_start) {
			if (!jit_emit_executed_add(&builder, i - segment_start))
				goto rejected;
			segment_start = i;
		}
		native_labels[i] = builder.cursor;
		executed_before = dynamic_counter_active ? i - segment_start : i;
		if (jit_persistent_memory_access(
			    &persistent_memory, i, instructions[i], kinds[i])) {
			if (!jit_emit_persistent_memory_access(
				    &builder, &map, entry, memory_sites,
				    &persistent_memory, i, instructions[i],
				    instruction_pc, executed_before,
				    hoist_persistent_memory, linear_persistent))
				goto rejected;
			continue;
		}

		switch (kinds[i]) {
		case JIT_INSN_INTEGER:
			okay = jit_emit_integer(&builder, &map, instructions[i],
						instruction_pc);
			break;
		case JIT_INSN_LOAD: {
			struct jit_memory_group group = jit_find_memory_group(
				instructions, kinds, leaders, count, i, false, &group_hint);
			const struct jit_loop_read_cache *loop_cache =
				jit_loop_read_cache_for(loop_read_caches,
							loop_read_cache_count, i);

			if (hoist_persistent_memory || loop_write_cache.valid ||
			    (persistent_memory.valid &&
			    i >= persistent_memory.first &&
			    i <= persistent_memory.last))
				group.count = 1u;
			if (group.count > 1u) {
				okay = builder.linear &&
					jit_linear_expected(&group_hint,
						(instructions[i] >> 15) & 0x1fu) ?
					jit_emit_linear_load_group(
						&builder, &map, &instructions[i],
						instruction_pc, executed_before,
						&group) :
					jit_emit_load_group(
						&builder, &map, entry, memory_sites,
						&instructions[i], instruction_pc,
						executed_before, &group);
				i += group.count - 1u;
			} else if (builder.linear && loop_cache == NULL &&
				   jit_linear_expected(&group_hint,
					(instructions[i] >> 15) & 0x1fu)) {
				okay = jit_emit_linear_load(&builder, &map,
							    instructions[i],
							    instruction_pc,
							    executed_before);
			} else {
				okay = jit_emit_load(&builder, &map, entry,
						     memory_sites, instructions[i],
						     instruction_pc, executed_before,
						     loop_cache);
			}
			break;
		}
		case JIT_INSN_STORE: {
			struct jit_memory_group group = jit_find_memory_group(
				instructions, kinds, leaders, count, i, true, &group_hint);
			const struct jit_loop_write_cache *loop_cache =
				jit_loop_write_cache_for(&loop_write_cache, i);

			if (hoist_persistent_memory || loop_write_cache.valid ||
			    (persistent_memory.valid &&
			    i >= persistent_memory.first &&
			    i <= persistent_memory.last))
				group.count = 1u;
			if (group.count > 1u) {
				okay = builder.linear &&
					jit_linear_expected(&group_hint,
						(instructions[i] >> 15) & 0x1fu) ?
					jit_emit_linear_store_group(
						&builder, &map, &instructions[i],
						instruction_pc, executed_before,
						&group) :
					jit_emit_store_group(
						&builder, &map, entry, memory_sites,
						&instructions[i], instruction_pc,
						executed_before, &group);
				i += group.count - 1u;
			} else if (builder.linear && loop_cache == NULL &&
				   jit_linear_expected(&group_hint,
					(instructions[i] >> 15) & 0x1fu)) {
				okay = jit_emit_linear_store(&builder, &map,
							     instructions[i],
							     instruction_pc,
							     executed_before);
			} else {
				okay = jit_emit_store(&builder, &map, entry,
						      memory_sites, instructions[i],
						      instruction_pc, executed_before,
						      loop_cache);
			}
			break;
		}
		case JIT_INSN_BRANCH: {
			uint32_t target_pc = instruction_pc +
				(uint32_t)jit_b_immediate(instructions[i]);
			uint32_t target_index = UINT32_MAX;

			if (dynamic_counter_active) {
				if (!jit_emit_executed_add(
					    &builder, i - segment_start + 1u))
					goto rejected;
				segment_start = i + 1u;
			}
			if (loops_to_start && i + 1u == count) {
				okay = jit_emit_branch_to_start(
					&builder, &map, instructions[i], instruction_pc,
					loop_start);
			} else if (repeated && i + 1u == count) {
				okay = jit_emit_interior_backedge(
					&builder, &map, instructions[i], instruction_pc,
					native_labels[loop_target_index],
					count - loop_target_index);
			} else if (i + 1u < count) {
				if ((target_pc & 3u) == 0u)
					target_index = jit_find_collected_pc(
						guest_pcs, count, target_pc);
				if (target_index > i && target_index < count)
					okay = jit_emit_internal_branch(
						&builder, &map, instructions[i], target_index,
						internal_branch_patches,
						&internal_branch_patch_count);
				else
					okay = jit_emit_branch_side_exit(
						&builder, &map, instructions[i], instruction_pc);
			} else {
				okay = jit_emit_branch_exit(&builder, &map,
							      instructions[i], instruction_pc);
			}
			break;
		}
		case JIT_INSN_JAL:
			if (dynamic_counter_active) {
				if (!jit_emit_executed_add(
					    &builder, i - segment_start + 1u))
					goto rejected;
				segment_start = i + 1u;
			}
			if (i + 1u < count &&
			    guest_pcs[i + 1u] == instruction_pc +
				(uint32_t)jit_j_immediate(instructions[i])) {
				/* The direct target is the next native trace instruction. */
				jit_emit_link_value(
					&builder, &map,
					(instructions[i] >> 7) & 0x1fu,
					instruction_pc + 4u);
				okay = !builder.failed;
			} else if (loops_to_start && i + 1u == count) {
				okay = jit_emit_jal_to_start(
					&builder, instructions[i], instruction_pc,
					loop_start);
			} else if (repeated && i + 1u == count) {
				okay = jit_emit_interior_backedge(
					&builder, &map, instructions[i], instruction_pc,
					native_labels[loop_target_index],
					count - loop_target_index);
			} else {
				jit_emit_link_value(&builder, &map,
						    (instructions[i] >> 7) & 0x1fu,
						    instruction_pc + 4u);
				jit_emit_constant(
					&builder, RV_A1,
					instruction_pc +
					(uint32_t)jit_j_immediate(instructions[i]));
				okay = !builder.failed;
			}
			break;
		case JIT_INSN_AMO:
			okay = jit_emit_amo(&builder, &map, entry, memory_sites,
					    instructions[i], instruction_pc,
					    executed_before);
			break;
		case JIT_INSN_NOP:
			okay = true;
			break;
		case JIT_INSN_CSR:
			if (dynamic_counter_active &&
			    jit_csr_may_exit(instructions[i])) {
				if (!jit_emit_executed_add(
					    &builder, i - segment_start + 1u))
					goto rejected;
				segment_start = i + 1u;
			}
			okay = jit_emit_csr(&builder, &map, instructions[i],
					    instruction_pc);
			break;
		case JIT_INSN_JALR:
			if (dynamic_counter_active) {
				if (!jit_emit_executed_add(
					    &builder, i - segment_start + 1u))
					goto rejected;
				segment_start = i + 1u;
			}
			if (i + 1u < count)
				okay = jit_emit_jalr_trace(
					&builder, &map, instructions[i], instruction_pc,
					guest_pcs[i + 1u]);
			else
				okay = jit_emit_jalr_exit(
					&builder, &map, instructions[i], instruction_pc);
			break;
		default:
			okay = false;
			break;
		}
		if (!okay)
			goto rejected;
	}

	if (dynamic_counter_active && count > segment_start &&
	    !jit_emit_executed_add(&builder, count - segment_start))
		goto rejected;
	for (uint32_t i = 0; i < internal_branch_patch_count; i++) {
		const struct jit_internal_branch_patch *patch =
			&internal_branch_patches[i];

		if (native_labels[patch->target_index] == NULL ||
		    !jit_patch_branch(patch->instruction,
				      native_labels[patch->target_index],
				      patch->funct3, patch->source1,
				      patch->source2))
			goto rejected;
	}
	if (!terminal)
		jit_emit_constant(&builder, RV_A1,
				  guest_pcs[count - 1u] + sizeof(uint32_t));
	normal_result_start = builder.cursor;
	for (uint32_t i = 0; i < builder.control_exit_patch_count; i++) {
		if (!jit_patch_unconditional(builder.control_exit_patches[i],
					     normal_result_start))
			goto rejected;
	}
	/* Keep the cumulative count register-resident across ordinary native
	 * links. */
	jit_emit_chain_count(&builder, chain_registers_clobbered,
			     dynamic_counter_active, count);
	jit_emit_register_commit(&builder, &map);
	/* A stable normal successor tail-jumps without publishing a C result, and
	 * checks the budget at its own entry.  Guest-register commits make this
	 * safe even when the two regions chose different local register maps;
	 * the shared wrapper frame remains.  An empty link expects UINT32_MAX,
	 * which no exit produces. */
	{
		uint32_t stub = (uint32_t)(uintptr_t)rv32_native_jit_dispatch;
		int32_t stub_low = (int32_t)(stub << 20) >> 20;
		uint32_t *pc_miss_patch;
		uint32_t *normal_return;
		uint32_t *publish_return;
		uint32_t *normal_done_patch;
		uint32_t *result_return;

		jit_record_slot_patch(gen, &builder,
				      JIT_SLOT_PATCH_LINK_UPPER, 0u);
		jit_emit(&builder, (RV_T0 << 7) | 0x37u);
		jit_record_slot_patch(gen, &builder, JIT_SLOT_PATCH_LINK_LOWER,
				      offsetof(struct jit_link_entry,
					       expected_next_pc));
		jit_emit(&builder, rv_i(0x03u, RV_T1, 2u, RV_T0, 0));
		pc_miss_patch = builder.cursor;
		jit_emit(&builder, 0u); /* bne actual PC, predicted PC, return */
		jit_record_slot_patch(gen, &builder, JIT_SLOT_PATCH_LINK_LOWER,
				      offsetof(struct jit_link_entry,
					       packed_target));
		jit_emit(&builder, rv_i(0x03u, RV_T2, 2u, RV_T0, 0));
#if RV32_NATIVE_JIT_RUNTIME_COUNTERS
		jit_emit_load_host(&builder, RV_T0, JIT_RUNTIME_REGISTER,
				   offsetof(struct rv32_native_jit_runtime, chain_links));
		jit_emit(&builder, rv_i(0x13u, RV_T0, 0u, RV_T0, 1));
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER, RV_T0,
				    offsetof(struct rv32_native_jit_runtime, chain_links));
#endif
		jit_emit(&builder, rv_i(0x67u, RV_X0, 0u, RV_T2,
				       (int32_t)JIT_NATIVE_CHAIN_ENTRY_BYTES));

		normal_return = builder.cursor;
		/* Other successors go through the shared dispatch stub, which returns
		 * here only when native code cannot continue.  t1 still holds this
		 * region's link word: an empty link is learned by the stub.  These
		 * fields are consumed only after leaving the native chain; a cold
		 * stub has already seeded its own local count and the checked
		 * resolver supplies its PC, so it skips them. */
		jit_emit_slot_number(gen, &builder, RV_A5);
		jit_emit(&builder, ((stub - (uint32_t)stub_low) & 0xfffff000u) |
				   (RV_T2 << 7) | 0x37u);
		jit_emit(&builder, rv_i(0x67u, RV_T3, 0u, RV_T2, stub_low));
		publish_return = builder.cursor;
		jit_emit_constant(&builder, RV_A0, 0u);
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER, RV_A1,
				    JIT_RESULT_OFFSET(next_pc));
		if (!dynamic_counter_active)
			jit_emit_constant(&builder, RV_T0, count);
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER,
				    dynamic_counter_active ?
					    JIT_EXECUTED_REGISTER : RV_T0,
				    JIT_RESULT_OFFSET(executed));
		normal_done_patch = builder.cursor;
		jit_emit(&builder, 0u); /* j result return */

		if (!jit_emit_linear_slow_paths(&builder, &map, entry,
						memory_sites, &persistent_memory) ||
		    !jit_emit_loop_read_cache_stubs(&builder) ||
		    !jit_emit_loop_write_cache_stubs(&builder) ||
		    !jit_emit_two_page_store_stubs(&builder, &map) ||
		    !jit_emit_deferred_memory_stubs(&builder, &map, memory_sites) ||
		    !jit_emit_linear_dirty_helper(&builder))
			goto rejected;

		/* A CSR exit that must reach the interpreter counts and commits
		 * like the normal exit, then publishes without a link or stub. */
		if (builder.hard_exit_patch_count != 0u) {
			uint32_t *publish_patch;

			for (uint32_t i = 0; i < builder.hard_exit_patch_count; i++) {
				if (!jit_patch_unconditional(
					    builder.hard_exit_patches[i],
					    builder.cursor))
					goto rejected;
			}
			jit_emit_chain_count(&builder, chain_registers_clobbered,
					     dynamic_counter_active, count);
			jit_emit_register_commit(&builder, &map);
			publish_patch = builder.cursor;
			jit_emit(&builder, 0u); /* j publish */
			if (builder.failed ||
			    !jit_patch_unconditional(publish_patch, publish_return))
				goto rejected;
		}

		/* The entry check above: nothing ran, so publish this region's PC
		 * with no work.  s10 still holds the live chain count, even in a
		 * region whose read caches would later borrow it. */
		if (!jit_patch_branch(budget_exit_patch, builder.cursor, 6u,
				      JIT_CHAIN_LIMIT_REGISTER, RV_T0))
			goto rejected;
		jit_emit_constant(&builder, RV_A1, guest_pc);
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER, RV_A1,
				    JIT_RESULT_OFFSET(next_pc));
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER, RV_X0,
				    JIT_RESULT_OFFSET(executed));
		jit_emit_constant(&builder, RV_A0, 0u);
		jit_emit_slot_number(gen, &builder, RV_T0);
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER, RV_T0,
				    JIT_RESULT_OFFSET(native_slot));
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER, RV_X0,
				    JIT_RESULT_OFFSET(native_successor));
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER,
				    JIT_CHAIN_EXECUTED_REGISTER,
				    offsetof(struct rv32_native_jit_runtime,
					     chain_executed));
		jit_emit(&builder, 0x00008067u); /* ret */

		/* Every cold memory stub jumps here after saving its site and any
		 * completed region segments.  Commit mapped registers before
		 * returning to C. */
		for (uint32_t i = 0; i < builder.bailout_patch_count; i++) {
			if (!jit_patch_unconditional(builder.bailout_patches[i],
						     builder.cursor))
				goto rejected;
		}
		if (builder.bailout_patch_count != 0u)
			jit_emit_register_commit(&builder, &map);
		result_return = builder.cursor;
		jit_emit_slot_number(gen, &builder, RV_T0);
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER, RV_T0,
				    JIT_RESULT_OFFSET(native_slot));
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER, RV_X0,
				    JIT_RESULT_OFFSET(native_successor));
		/* A cold exit can bypass a clobbering loop's normal count update.
		 * Recover its entry snapshot before publishing the cumulative count;
		 * ordinary regions keep the authoritative value in s10 throughout. */
		if (chain_registers_clobbered)
			jit_emit_load_host(&builder, JIT_CHAIN_EXECUTED_REGISTER,
					   JIT_RUNTIME_REGISTER,
					   offsetof(struct rv32_native_jit_runtime,
						    chain_executed));
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER,
				    JIT_CHAIN_EXECUTED_REGISTER,
				    offsetof(struct rv32_native_jit_runtime,
					     chain_executed));
		jit_emit(&builder, 0x00008067u); /* ret */
		if (builder.failed ||
		    !jit_patch_branch(pc_miss_patch, normal_return,
				      1u, RV_A1, RV_T1) ||
		    !jit_patch_unconditional(normal_done_patch, result_return))
			goto rejected;
	}

	if (builder.failed || builder.cursor > builder.end) {
		rejection = JIT_REJECT_CODE_SIZE;
		goto rejected;
	}
	rejection = JIT_REJECT_PUBLISH;
	if (!jit_code_valid(builder.start,
			    (size_t)(builder.cursor - builder.start)))
		goto rejected;
	gen->used_words = (uint32_t)(builder.cursor - builder.start);
	gen->count = count;
	gen->traced_jump_count = traced_jump_count;
	gen->loops_to_start = loops_to_start;
	gen->spans_pages = spans_pages;
	return true;

	rejected:
	if (builder.failed)
		rejection = JIT_REJECT_CODE_SIZE;
	gen->rejection = rejection;
	return false;
}

/* A trace that outgrows the code buffer or a stub table (thirty-odd memory
 * operations in a row) still runs natively as shorter regions that link to
 * each other, so halve it before giving up on the head. */
static bool jit_codegen_splitting(struct jit_codegen *gen)
{
	uint32_t limit = gen->instruction_limit;

	gen->split = false;
	if (jit_codegen(gen))
		return true;
	while (gen->rejection == JIT_REJECT_CODE_SIZE &&
	       limit / 2u >= JIT_MIN_GUEST_INSTRUCTIONS) {
		limit /= 2u;
		gen->instruction_limit = limit;
		if (jit_codegen(gen)) {
			gen->split = true;
			return true;
		}
	}
	return false;
}

/* Publishes gen's region in slot, replacing whatever was there.  Runs on
 * the emulator core: it changes the directory, links and arena. */
static bool jit_install_generated_body(uint32_t slot, struct jit_codegen *gen);

static bool jit_install_generated(uint32_t slot, struct jit_codegen *gen)
{
#if CONFIG_RV32_HOST_PERF_STATS
	uint32_t start = jit_profile_cycle_count();
	bool installed = jit_install_generated_body(slot, gen);

	jit_stats.install_cycles += (uint32_t)(jit_profile_cycle_count() - start);
	jit_stats.installs++;
	return installed;
#else
	return jit_install_generated_body(slot, gen);
#endif
}

#if JIT_KEEP_KERNEL
/* Marks the guest RAM pages a kernel region's instructions came from.  The
 * linear map is contiguous, so each instruction lies at the head's physical
 * offset plus its PC distance from the head. */
static void jit_mark_kernel_code(const struct jit_trace *trace, uint32_t count)
{
	if (!jit_kernel_pc(trace->guest_pc))
		return;
	for (uint32_t i = 0; i < count; i++) {
		uint32_t page = (trace->physical_offset + trace->guest_pcs[i] -
				 trace->guest_pc) >> 12;

		if (page < JIT_KERNEL_CODE_PAGES)
			jit_kernel_code_pages[page >> 5] |=
				UINT32_C(1) << (page & 31u);
	}
}
#endif

static bool jit_install_generated_body(uint32_t slot, struct jit_codegen *gen)
{
	struct jit_cache_entry *entry = &jit_entries[slot];
	const struct jit_trace *trace = gen->trace;
	uint32_t site_count = gen->scratch.memory_site_count;

	jit_forget_entry(slot);
	jit_patch_slot_words(gen, slot);
	if (!jit_install_code(slot, gen->code, gen->used_words,
			      gen->memory_sites, site_count, gen->loops_to_start,
#if JIT_KEEP_KERNEL
			      jit_kernel_pc(trace->guest_pc)
#else
			      false
#endif
			      )) {
		gen->rejection = JIT_REJECT_PUBLISH;
		return false;
	}
	if (gen->split)
		rv32_profile_jit_event(RV32_PROFILE_SPLIT_COMPILE);
	entry->physical_offset = trace->physical_offset;
	entry->guest_pc = trace->guest_pc;
	entry->guest_instruction_count = (uint8_t)gen->count;
	entry->loops_to_start = gen->loops_to_start;
	entry->spans_pages = gen->spans_pages;
	jit_link_learning_budget[slot] = JIT_LINK_CONFIRMATIONS;
	jit_forget_jump_miss(trace->guest_pc);
	entry->state = JIT_ENTRY_READY;
#if JIT_KEEP_KERNEL
	jit_mark_kernel_code(trace, gen->count);
	jit_note_user_slot(slot, trace->guest_pc);
#endif
	jit_has_cross_page_entries |= gen->spans_pages;
	jit_stats.compiled_blocks++;
	jit_stats.compiled_guest_instructions += gen->count;
	jit_stats.compiled_native_words += gen->used_words;
	jit_stats.traced_jumps += gen->traced_jump_count;
	jit_stats.traced_regions += gen->traced_jump_count != 0u;
	jit_stats.cross_page_regions += gen->spans_pages;
	if (gen->count > jit_stats.max_compiled_guest_instructions)
		jit_stats.max_compiled_guest_instructions = gen->count;
	if (gen->used_words > jit_stats.max_compiled_native_words)
		jit_stats.max_compiled_native_words = gen->used_words;
	if (gen->traced_jump_count > jit_stats.max_traced_jumps)
		jit_stats.max_traced_jumps = gen->traced_jump_count;
	return true;
}

/* Marks slot as a head that did not compile, so it stays interpreted. */
static void jit_install_rejection(uint32_t slot,
				  const struct jit_trace *trace,
				  enum jit_reject_reason rejection,
				  const uint8_t *guest_ram)
{
	struct jit_cache_entry *entry = &jit_entries[slot];

	jit_forget_entry(slot);
	jit_last_rejection = rejection;
	rv32_profile_jit_event((enum rv32_profile_jit_event)
			       (RV32_PROFILE_REJECT_COLLECT + rejection));
#if CONFIG_RV32_JIT_EXIT_TRACE
	if (rv32_jit_trace_enabled()) {
		uint32_t opcode;
		memcpy(&opcode, guest_ram + trace->physical_offset,
		       sizeof(opcode));
		rv32_jit_trace_record(RV32_JIT_TRACE_COLLECT + rejection,
			trace->guest_pc, trace->physical_offset, opcode,
			0u, 0u, 0u);
	}
#else
	(void)guest_ram;
#endif
	entry->physical_offset = trace->physical_offset;
	entry->guest_pc = trace->guest_pc;
	entry->guest_instruction_count = 0u;
	entry->memory_site_count = 0u;
	entry->loops_to_start = 0u;
	entry->code_units = 0u;
	entry->spans_pages = 0u;
	entry->state = JIT_ENTRY_REJECTED;
#if JIT_KEEP_KERNEL
	jit_note_user_slot(slot, trace->guest_pc);
#endif
	jit_stats.rejected_blocks++;
	switch (rejection) {
	case JIT_REJECT_COLLECT:
		jit_stats.rejected_collect++;
		jit_remember_negative(trace->physical_offset);
		break;
	case JIT_REJECT_CODE_SIZE:
		jit_stats.rejected_code_size++;
		break;
	case JIT_REJECT_CODEGEN:
		jit_stats.rejected_codegen++;
		break;
	case JIT_REJECT_PUBLISH:
		jit_stats.rejected_publish++;
		break;
	}
}

/* The emulator core collects into this trace and generates into
 * jit_staging, for synchronous compiles and the self-tests. */
static RV32_STATE_BSS struct jit_trace jit_sync_trace;
static RV32_STATE_BSS struct rv32_native_jit_memory_site
	jit_sync_memory_sites[JIT_MAX_MEMORY_SITES];

static void jit_collect_trace(struct jit_trace *trace,
			      const uint8_t *guest_ram,
			      uint32_t physical_offset, uint32_t guest_pc,
			      uint32_t instruction_limit)
{
	trace->physical_offset = physical_offset;
	trace->guest_pc = guest_pc;
	trace->collectable = jit_collect_region(guest_ram, physical_offset,
		guest_pc, instruction_limit, trace->instructions,
		trace->guest_pcs, trace->kinds, &trace->count);
}

/* Compiles into slot on this core. */
static bool jit_compile_now(uint32_t slot, const uint8_t *guest_ram,
	uint32_t physical_offset, uint32_t guest_pc, uint32_t instruction_limit,
	const uint32_t *entry_values, bool split)
{
	struct jit_codegen gen = {
		.trace = &jit_sync_trace,
		.instruction_limit =
			instruction_limit < JIT_MAX_GUEST_INSTRUCTIONS ?
			instruction_limit : JIT_MAX_GUEST_INSTRUCTIONS,
		.entry_values = entry_values,
		.refill = jit_runtimes[JIT_BANK_USER].refill_ram != NULL,
		.workspace = &jit_compile_workspace,
		.code = jit_staging,
		.memory_sites = jit_sync_memory_sites,
	};
	bool generated;

	jit_collect_trace(&jit_sync_trace, guest_ram, physical_offset,
			  guest_pc, instruction_limit);
	generated = split ? jit_codegen_splitting(&gen) : jit_codegen(&gen);
	if (generated && jit_install_generated(slot, &gen))
		return true;
	jit_install_rejection(slot, &jit_sync_trace, gen.rejection, guest_ram);
	return false;
}

static bool jit_compile_with_entry_values(uint32_t slot,
	const uint8_t *guest_ram, uint32_t physical_offset, uint32_t guest_pc,
	uint32_t instruction_limit, const uint32_t *entry_values)
{
	return jit_compile_now(slot, guest_ram, physical_offset, guest_pc,
			       instruction_limit, entry_values, false);
}

/* Self-tests compile synthetic regions before binding live guest registers. */
static bool jit_compile(uint32_t slot, const uint8_t *guest_ram,
	uint32_t physical_offset, uint32_t guest_pc, uint32_t instruction_limit)
{
	return jit_compile_with_entry_values(slot, guest_ram, physical_offset,
		guest_pc, instruction_limit, NULL);
}

static bool jit_compile_splitting(uint32_t slot, const uint8_t *guest_ram,
	uint32_t physical_offset, uint32_t guest_pc, uint32_t instruction_limit,
	const uint32_t *entry_values)
{
	return jit_compile_now(slot, guest_ram, physical_offset, guest_pc,
			       instruction_limit, entry_values, true);
}

struct jit_test_memory {
	uint32_t address;
	uint32_t value;
};

static uint32_t jit_test_load(struct rv32_native_jit_runtime *runtime,
			      uint32_t virtual_address, uint32_t descriptor)
{
	struct jit_test_memory *memory = runtime->opaque;
	struct rv32_native_jit_read_page *page;

	runtime->memory_calls++;
	if (virtual_address != memory->address || (descriptor & 7u) != 2u) {
		rv32_native_jit_memory_exit(runtime, descriptor, 6u,
					    virtual_address);
		return 1u;
	}
	runtime->memory_value = memory->value;
	page = &runtime->read_pages[(virtual_address >> 12) &
		(RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u)];
	page->host_bias =
		(uint32_t)(uintptr_t)&memory->value - virtual_address;
	page->virtual_page = virtual_address >> 12;
	return 0u;
}

static uint32_t jit_test_store(struct rv32_native_jit_runtime *runtime,
			       uint32_t virtual_address, uint32_t value,
			       uint32_t descriptor)
{
	struct jit_test_memory *memory = runtime->opaque;
	struct rv32_native_jit_write_page *page;

	runtime->memory_calls++;
	if (virtual_address != memory->address || (descriptor & 7u) != 2u) {
		rv32_native_jit_memory_exit(runtime, descriptor, 8u,
					    virtual_address);
		return 1u;
	}
	memory->value = value;
	page = &runtime->write_pages[(virtual_address >> 12) &
		(RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u)];
	page->host_bias =
		(uint32_t)(uintptr_t)&memory->value - virtual_address;
	page->linear_page_start = UINT32_MAX;
	page->linear_page_end = 0u;
	page->virtual_page = virtual_address >> 12;
	return 0u;
}

/* Called from the refill trampoline with guest registers live; the callback
 * may publish mappings but must not perform a guest access. */
uint32_t IRAM_ATTR rv32_native_jit_refill_ram_page(
	struct rv32_native_jit_runtime *runtime, uint32_t address, uint32_t access)
{
#if CONFIG_RV32_JIT_EXIT_TRACE
	uint32_t trace_start = rv32_jit_trace_cycles();
#endif
#if CONFIG_RV32_HOST_PERF_STATS
	uint32_t start = jit_profile_cycle_count();
#endif
	bool resumed = runtime->refill_ram != NULL &&
		runtime->refill_ram(runtime, address, access);
#if CONFIG_RV32_HOST_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
	jit_stats.ram_refill_attempts++;
	jit_stats.ram_refill_resumes += resumed;
#endif
#if CONFIG_RV32_HOST_PERF_STATS
	jit_stats.ram_refill_cycles += (uint32_t)(jit_profile_cycle_count() - start);
#endif
#if CONFIG_RV32_JIT_EXIT_TRACE
	rv32_jit_trace_refill(resumed, rv32_jit_trace_cycles() - trace_start);
#endif
	return resumed;
}

#if CONFIG_RV32_JIT_EXIT_TRACE
static enum rv32_jit_trace_reason jit_trace_memory_reason(
	const struct rv32_native_jit_runtime *runtime, uint32_t width,
	bool mapped)
{
	if (runtime->result.trap) return RV32_JIT_TRACE_FAULT;
	switch (runtime->trace_memory_kind) {
	case RV32_JIT_TRACE_MEMORY_RAM:
		if (width && (runtime->memory_address & (width - 1u)))
			return RV32_JIT_TRACE_RAM_UNALIGNED;
		return mapped ? RV32_JIT_TRACE_RAM_GUARD : RV32_JIT_TRACE_RAM_MAP;
	case RV32_JIT_TRACE_MEMORY_UNCACHED: return RV32_JIT_TRACE_UNCACHED;
	case RV32_JIT_TRACE_MEMORY_FRAMEBUFFER: return RV32_JIT_TRACE_FRAMEBUFFER;
	case RV32_JIT_TRACE_MEMORY_DEVICE: return RV32_JIT_TRACE_DEVICE;
	default: return RV32_JIT_TRACE_OTHER;
	}
}
#endif

/* Finish a cold memory exit once generated code has committed its registers.
 * A successful access completes here and seeds the page cache; faults and
 * MMIO exit to the interpreter. */
static void jit_resolve_memory_bailout(
	struct rv32_native_jit_runtime *runtime, uint32_t bailout)
{
	uint32_t site_index = bailout - 1u;
	const struct rv32_native_jit_memory_site *site;
	uint32_t failed;

	if (bailout == 0u)
		return;
	if (site_index >= runtime->site_count) {
		runtime->result.next_pc = 0u;
		runtime->result.trap = 2u;
		runtime->result.trap_value = 0u;
		return;
	}

	site = &runtime->sites[site_index];
	if (site->store == JIT_SITE_AMO) {
		/* The interpreter owns reservations, MMIO and faults: rerun the
		 * atomic there.  Nothing was applied on the native side. */
		runtime->result.next_pc = site->guest_pc;
		runtime->result.executed += site->executed_before;
		return;
	}
#if CONFIG_RV32_JIT_EXIT_TRACE
	bool trace = rv32_jit_trace_enabled();
	uint32_t physical = UINT32_MAX, opcode = 0u, start = 0u;
	uint32_t width = jit_memory_width(site->descriptor & 7u, !site->store);
	uint32_t vpn = runtime->memory_address >> 12;
	uint32_t index = vpn & (RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u);
	bool mapped = (site->store ? runtime->write_pages[index].virtual_page :
		       runtime->read_pages[index].virtual_page) == vpn;
	if (trace) {
		/* The exit may lie in a linked successor; use its page, and read
		 * the opcode before the callback can change memory. */
		uint32_t slot = runtime->result.native_slot;
		if (slot < JIT_CACHE_SLOTS &&
		    (site->guest_pc >> 12) == (jit_entries[slot].guest_pc >> 12))
			physical = (jit_entries[slot].physical_offset & ~0xfffu) |
				(site->guest_pc & 0xfffu);
		else
			(void)jit_resolve_exec_offset(site->guest_pc, &physical);
		if (physical != UINT32_MAX && runtime->image != NULL)
			memcpy(&opcode, runtime->image + physical, sizeof(opcode));
		runtime->trace_memory_kind = RV32_JIT_TRACE_MEMORY_OTHER;
		start = rv32_jit_trace_cycles();
	}
#endif
	if (site->store) {
		failed = runtime->store(runtime, runtime->memory_address,
					runtime->memory_value, site->descriptor);
	} else {
		failed = runtime->load(runtime, runtime->memory_address,
				       site->descriptor);
	}
#if CONFIG_RV32_JIT_EXIT_TRACE
	if (trace) {
		uint32_t cycles = rv32_jit_trace_cycles() - start;
		rv32_jit_trace_record(jit_trace_memory_reason(runtime, width, mapped),
			site->guest_pc, physical, opcode, runtime->memory_address,
			runtime->result.trap, cycles);
	}
#endif
	if (failed != 0u)
		return;

	if (!site->store && site->destination != 0u)
		runtime->regs[site->destination] = runtime->memory_value;
	runtime->result.next_pc = site->guest_pc + 4u;
	runtime->result.executed += (uint32_t)site->executed_before + 1u;
}

static __attribute__((noinline)) bool jit_self_test_chain_results(void)
{
	struct rv32_native_jit_runtime *runtime = jit_rt;
	const uint32_t base_pc = 0x18000u;
	uint32_t source_program[] = {
		rv_i(0x13u, 1u, 0u, 1u, 1),
		rv_i(0x13u, 1u, 0u, 1u, 2),
		rv_b(0u, 0u, 0u, 0xf8),
	};
	uint32_t middle_program[] = {
		rv_i(0x13u, 2u, 0u, 2u, 4),
		rv_i(0x13u, 2u, 0u, 2u, 8),
		rv_b(0u, 0u, 0u, 0xf8),
	};
	uint32_t target_program[] = {
		rv_i(0x13u, 3u, 0u, 3u, 5),
		rv_i(0x03u, 5u, 2u, 4u, 0),
		rv_i(0x13u, 3u, 0u, 3u, 7),
		rv_b(0u, 0u, 0u, 8),
	};
	struct jit_test_memory memory = {
		.address = 0x24000u,
		.value = 0x12345678u,
	};
	uint32_t regs[32];

	if (!jit_compile(0u, (const uint8_t *)source_program, 0u,
			 base_pc, 3u) ||
	    !jit_compile(1u, (const uint8_t *)middle_program, 0u,
			 base_pc + 0x100u, 3u) ||
	    !jit_compile(2u, (const uint8_t *)target_program, 0u,
			 base_pc + 0x200u, 4u))
		return false;
	for (uint32_t slot = 0u; slot < 2u; slot++) {
		jit_links[slot].expected_next_pc = base_pc + (slot + 1u) * 0x100u;
		jit_links[slot].packed_target =
			jit_pack_link_target(slot + 1u, &jit_entries[slot + 1u]);
		if (jit_links[slot].packed_target == 0u)
			return false;
		jit_link_targets[slot + 1u] = 1u;
	}
	/* Reused result storage must be overwritten at the actual return, not
	 * inherited from an earlier dispatch or a successfully linked prefix.
	 * Straight-line cold stubs have no dynamic counter: the site metadata
	 * alone accounts for instructions preceding the faulting operation. */
	for (uint32_t linked = 0u; linked < 2u; linked++) {
		for (uint32_t access = 0u; access < 3u; access++) {
			uint32_t prefix = linked ? 6u : 0u;
			uint32_t bailout;
			jit_block_fn block = (jit_block_fn)(uintptr_t)
				jit_entry_code(linked ? 0u : 2u);

			memset(regs, 0, sizeof(regs));
			*runtime = (struct rv32_native_jit_runtime) {
				.regs = regs,
				.opaque = &memory,
				.load = jit_test_load,
				.chain_executed = 13u,
				.chain_instruction_limit = 13u + prefix + 4u,
				.sites = jit_slot_sites(2),
				.site_count = jit_entries[2].memory_site_count,
				.result = {
					.next_pc = UINT32_MAX,
					.executed = UINT32_MAX,
					.native_successor = 1u,
					.native_slot = UINT32_MAX,
				},
			};
			for (uint32_t i = 0u;
			     i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++)
				runtime->read_pages[i].virtual_page = UINT32_MAX;
			regs[4] = memory.address + (access == 2u ? 4u : 0u);
			if (access == 0u &&
			    jit_test_load(runtime, memory.address, 2u) != 0u)
				return false;
			runtime->memory_calls = 0u;
			bailout = rv32_native_jit_enter(block, runtime);
			if (runtime->result.native_slot != 2u ||
			    runtime->result.native_successor != 0u ||
			    runtime->chain_links !=
				    (RV32_NATIVE_JIT_RUNTIME_COUNTERS ? linked * 2u : 0u) ||
			    regs[1] != linked * 3u || regs[2] != linked * 12u)
				return false;
			if (access == 0u) {
				if (bailout != 0u || runtime->result.executed != 4u ||
				    runtime->chain_executed != 13u + prefix + 4u ||
				    runtime->result.next_pc != base_pc + 0x214u ||
				    regs[3] != 12u || regs[5] != memory.value ||
				    runtime->memory_calls != 0u)
					return false;
			} else {
				if (bailout == 0u || runtime->result.executed != 0u ||
				    runtime->chain_executed != 13u + prefix ||
				    regs[3] != 5u || regs[5] != 0u)
					return false;
				jit_resolve_memory_bailout(runtime, bailout);
				if (runtime->result.executed != (access == 1u ? 2u : 1u) ||
				    runtime->result.next_pc != base_pc +
					    (access == 1u ? 0x208u : 0x204u) ||
				    runtime->result.trap != (access == 1u ? 0u : 6u) ||
				    regs[5] != (access == 1u ? memory.value : 0u) ||
				    runtime->memory_calls != 1u ||
				    (access == 2u &&
				     runtime->result.trap_value != memory.address + 4u))
					return false;
			}
		}
	}
	/* An empty prediction or a wrong PC publishes the source's results; a
	 * successor without budget publishes its own entry with no work done.
	 * Also exhaust the budget after one link. */
	for (uint32_t stop = 0u; stop < 4u; stop++) {
		uint32_t final_slot = stop == 3u ? 1u : 0u;
		uint32_t reported_slot = stop >= 2u ? final_slot + 1u : final_slot;
		uint32_t reported_executed = stop >= 2u ? 0u : 3u;
		jit_block_fn block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);

		memset(regs, 0, sizeof(regs));
		*runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.chain_executed = 13u,
			.chain_instruction_limit = 13u + (stop == 2u ? 5u : 9u),
			.result = {
				.next_pc = UINT32_MAX,
				.executed = UINT32_MAX,
				.native_successor = 1u,
				.native_slot = UINT32_MAX,
			},
		};
		if (stop == 0u) {
			jit_clear_link(&jit_links[0]);
		} else {
			jit_links[0].packed_target =
				jit_pack_link_target(1u, &jit_entries[1]);
			jit_links[0].expected_next_pc =
				base_pc + 0x100u + (stop == 1u ? 4u : 0u);
			jit_link_targets[1] = 1u;
		}
		if (rv32_native_jit_enter(block, runtime) != 0u ||
		    runtime->result.executed != reported_executed ||
		    runtime->result.next_pc != base_pc + (final_slot + 1u) * 0x100u ||
		    runtime->result.native_slot != reported_slot ||
		    runtime->result.native_successor != 0u ||
		    runtime->chain_executed != 13u + (final_slot + 1u) * 3u ||
		    regs[1] != 3u || regs[2] != final_slot * 12u || regs[3] != 0u)
			return false;
	}
	jit_clear_all_links();
	return true;
}

/* Same-page successors already in the cache continue through
 * rv32_native_jit_dispatch without returning to C: the published result names
 * the final region, the budget stops the chain exactly like the C dispatcher,
 * and a region's link-learning budget sends its first exits back to C. */
static __attribute__((noinline)) bool jit_self_test_native_dispatch(void)
{
	struct rv32_native_jit_runtime *runtime = jit_rt;
	const uint32_t base_pc = 0x18000u;
	static uint32_t program[3u * 64u];
	static const uint32_t no_wake;
	const volatile uint32_t *saved_wake =
		rv32_native_jit_dispatch_table.host_wake;
	uint32_t slots[3];
	uint32_t regs[32];
	bool okay = false;

	/* Device bridges may already have requested a host wake while the
	 * firmware initialises; the stub would then (correctly) refuse to chain
	 * and this deterministic test would fail.  Judge the stub, not timing. */
	rv32_native_jit_dispatch_table.host_wake = &no_wake;

	memset(program, 0, sizeof(program));
	for (uint32_t region = 0u; region < 3u; region++) {
		uint32_t *code = &program[region * 64u];
		uint32_t physical = region * 0x100u;
		uint32_t pc = base_pc + physical;
		uint32_t set = jit_hash(physical, pc, JIT_CACHE_SETS - 1u);

		code[0] = rv_i(0x13u, 1u, 0u, 1u, (int32_t)(1u << (2u * region)));
		code[1] = rv_i(0x13u, 1u, 0u, 1u, (int32_t)(2u << (2u * region)));
		code[2] = rv_b(0u, 0u, 0u, 0xf8); /* beq x0, x0, next region */
		/* The generated lookup hashes the key, so place each entry in its
		 * own set, after the ways earlier regions took there. */
		{
			uint32_t way = 0u;

			for (uint32_t earlier = 0u; earlier < region; earlier++)
				way += slots[earlier] / JIT_CACHE_WAYS == set;
			if (way >= JIT_CACHE_WAYS)
				goto out;
			slots[region] = set * JIT_CACHE_WAYS + way;
		}
		if (!jit_compile(slots[region], (const uint8_t *)program, physical,
				 pc, 3u))
			goto out;
		jit_link_learning_budget[slots[region]] = 0u;
	}
	jit_clear_all_links();
	/* These rounds rely on the hot tier's call counting, which keeps the stub
	 * from learning links; without a hot arena only rounds 4-6 apply. */
	for (uint32_t round = jit_hot_code_arena != NULL ? 0u : 4u; round < 4u;
	     round++) {
		/* 0: full chain; 1: budget admits only the first region; 2: one
		 * learning exit returns to C; 3: learning exhausted, chain again. */
		uint32_t chain_limit = round == 1u ? 5u : 9u;
		uint32_t final = round == 0u || round == 3u ? 2u : 0u;
		uint32_t expected_next = base_pc + (final + 1u) * 0x100u;
		jit_block_fn block = (jit_block_fn)(uintptr_t)
			jit_current_entry_code(slots[0]);

		if (round == 2u)
			jit_link_learning_budget[slots[0]] = 1u;
		memset(regs, 0, sizeof(regs));
		*runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.chain_executed = 13u,
			.chain_instruction_limit = 13u + chain_limit,
			.result = {
				.next_pc = UINT32_MAX,
				.executed = UINT32_MAX,
				.native_successor = 1u,
				.native_slot = UINT32_MAX,
			},
		};
		if (rv32_native_jit_enter(block, runtime) != 0u ||
		    runtime->result.executed != 3u ||
		    runtime->result.next_pc != expected_next ||
		    runtime->result.native_slot != slots[final] ||
		    runtime->result.native_successor != 0u ||
		    runtime->chain_executed != 13u + (final + 1u) * 3u ||
		    regs[1] != (final == 2u ? 63u : 3u) ||
		    jit_link_learning_budget[slots[0]] != 0u)
			goto out;
	}
	/* Without a hot arena nothing counts toward promotion: the stub fills
	 * the jump cache and learns each region's empty link (4), the next pass
	 * follows those links (5), and with the links cleared but the jump
	 * cache kept, jump-cache hits learn them again (6).  Region 2's exit
	 * has no successor and leaves its link empty. */
	rv32_native_jit_dispatch_table.hot_code_arena = NULL;
	jit_clear_all_links();
	for (uint32_t round = 4u; round < 7u; round++) {
		jit_block_fn block = (jit_block_fn)(uintptr_t)
			jit_entry_code(slots[0]);

		if (round == 6u) {
			for (uint32_t region = 0u; region < 3u; region++)
				jit_clear_link(&jit_links[slots[region]]);
		}
		memset(regs, 0, sizeof(regs));
		*runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.chain_executed = 13u,
			.chain_instruction_limit = 13u + 9u,
			.result = {
				.next_pc = UINT32_MAX,
				.executed = UINT32_MAX,
				.native_successor = 1u,
				.native_slot = UINT32_MAX,
			},
		};
		if (rv32_native_jit_enter(block, runtime) != 0u ||
		    runtime->result.executed != 3u ||
		    runtime->result.next_pc != base_pc + 0x300u ||
		    runtime->result.native_slot != slots[2] ||
		    runtime->chain_executed != 13u + 9u || regs[1] != 63u ||
		    jit_links[slots[0]].expected_next_pc != base_pc + 0x100u ||
		    jit_link_target_pointer(jit_links[slots[0]].packed_target) !=
			    (uint32_t)(uintptr_t)jit_entry_code(slots[1]) ||
		    jit_links[slots[1]].expected_next_pc != base_pc + 0x200u ||
		    jit_link_target_pointer(jit_links[slots[1]].packed_target) !=
			    (uint32_t)(uintptr_t)jit_entry_code(slots[2]) ||
		    jit_links[slots[2]].expected_next_pc != UINT32_MAX)
			goto out;
	}
	okay = true;
out:
	rv32_native_jit_dispatch_table.hot_code_arena = jit_hot_code_arena;
	jit_clear_all_links();
	rv32_native_jit_dispatch_table.host_wake = saved_wake;
	return okay;
}

static __attribute__((noinline)) bool jit_self_test_interior_loops(void)
{
	struct rv32_native_jit_runtime *runtime = jit_rt;
	uint32_t conditional_program[] = {
		rv_i(0x13u, 1u, 0u, 1u, 100), /* preheader runs only once */
		rv_b(0u, 6u, 0u, 12),        /* optionally skip to index 4 */
		rv_i(0x13u, 3u, 0u, 3u, 1),  /* interior loop header */
		rv_i(0x13u, 7u, 0u, 7u, 10),
		rv_i(0x13u, 1u, 0u, 1u, 1),
		rv_i(0x13u, 2u, 0u, 2u, -1),
		rv_b(1u, 2u, 0u, -16),
	};
	uint32_t jump_program[] = {
		rv_i(0x13u, 1u, 0u, 1u, 100),
		rv_i(0x13u, 1u, 0u, 1u, 1),  /* interior loop header */
		rv_i(0x13u, 2u, 0u, 2u, -1),
		rv_b(0u, 2u, 0u, 12),        /* early exit beyond the region */
		rv_i(0x13u, 3u, 0u, 3u, 1),
		rv_j(0u, -16),
	};
	uint32_t read_program[] = {
		rv_i(0x13u, 1u, 0u, 1u, 100),
		rv_i(0x03u, 3u, 2u, 4u, 0),  /* first read cache */
		rv_i(0x13u, 1u, 0u, 1u, 1),
		rv_i(0x03u, 5u, 2u, 6u, 0),  /* second cache borrows s10/s11 */
		rv_i(0x13u, 4u, 0u, 4u, 4),
		rv_i(0x13u, 2u, 0u, 2u, -1),
		rv_b(1u, 2u, 0u, -20),
	};
	uint32_t store_program[] = {
		rv_i(0x13u, 1u, 0u, 1u, 100),
		rv_s(0x23u, 1u, 4u, 3u, 0),  /* sh x3, 0(x4) */
		rv_i(0x13u, 4u, 0u, 4u, 2),
		rv_b(6u, 4u, 2u, -8),
	};
	uint32_t chain_prefix[] = {
		rv_i(0x13u, 9u, 0u, 9u, 1),
		rv_i(0x13u, 9u, 0u, 9u, 2),
		rv_b(0u, 0u, 0u, 0xf8),     /* 0x15f08 -> 0x16000 */
	};
	uint32_t regs[32];
	jit_block_fn block;

	/* Sweep every budget around the loop boundaries, with a nonzero inherited
	 * chain count and both paths into the first iteration. */
	if (!jit_compile(0u, (const uint8_t *)conditional_program, 0u,
			 0x14000u, 7u) || jit_entries[0].loops_to_start)
		return false;
	block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
	for (uint32_t skip = 0u; skip < 2u; skip++) {
		for (uint32_t budget = 7u; budget <= 20u; budget++) {
			uint32_t first = skip ? 5u : 7u;
			uint32_t iterations = 1u + (budget - first) / 5u;
			uint32_t executed;

			if (iterations > 3u)
				iterations = 3u;
			executed = first + (iterations - 1u) * 5u;
			memset(regs, 0, sizeof(regs));
			*runtime = (struct rv32_native_jit_runtime) {
				.regs = regs,
				.chain_executed = 13u,
				.chain_instruction_limit = 13u + budget,
				.result = { .executed = UINT32_MAX, .native_successor = 1u },
			};
			regs[2] = 3u;
			regs[6] = !skip;
			if (rv32_native_jit_enter(block, runtime) != 0u ||
			    runtime->result.executed != executed ||
			    runtime->result.native_successor != 0u ||
			    runtime->chain_executed != 13u + executed ||
			    runtime->result.next_pc !=
				    (iterations == 3u ? 0x1401cu : 0x14008u) ||
			    regs[1] != 100u + iterations ||
			    regs[2] != 3u - iterations ||
			    regs[3] != iterations - skip ||
			    regs[7] != 10u * (iterations - skip))
				return false;
		}
	}
	if (!jit_compile(0u, (const uint8_t *)jump_program, 0u,
			 0x15000u, 6u) || jit_entries[0].loops_to_start)
		return false;
	block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
	for (uint32_t budget = 6u; budget <= 18u; budget++) {
		uint32_t iterations = (budget - 1u) / 5u;
		uint32_t executed;

		if (iterations > 3u)
			iterations = 3u;
		executed = 1u + iterations * 5u - (iterations == 3u ? 2u : 0u);
		memset(regs, 0, sizeof(regs));
		*runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.chain_executed = 13u,
			.chain_instruction_limit = 13u + budget,
		};
		regs[2] = 3u;
		if (rv32_native_jit_enter(block, runtime) != 0u ||
		    runtime->result.executed != executed ||
		    runtime->chain_executed != 13u + executed ||
		    runtime->result.next_pc !=
			    (iterations == 3u ? 0x15018u : 0x15004u) ||
		    regs[1] != 100u + iterations || regs[2] != 3u - iterations ||
		    regs[3] != iterations - (iterations == 3u))
			return false;
	}

	/* Two read caches must not corrupt the entry budget. Also force a cold
	 * page on the second iteration and verify the exact fault PC and registers. */
	if (!jit_compile(0u, (const uint8_t *)read_program, 0u,
			 0x16000u, 7u) ||
	    !jit_compile(1u, (const uint8_t *)chain_prefix, 0u,
			 0x15f00u, 3u))
		return false;
	jit_links[1].expected_next_pc = 0x16000u;
	jit_links[1].packed_target = jit_pack_link_target(0u, &jit_entries[0]);
	jit_link_targets[0] = 1u;
	block = (jit_block_fn)(uintptr_t)jit_entry_code(1u);
	for (uint32_t cold = 0u; cold < 2u; cold++) {
		uint32_t values[3] = { 11u, 22u, 33u };
		struct jit_test_memory memory = { .address = 0u, .value = 71u };
		uint32_t address = cold ? 0x20ffcu : 0x20000u;
		uint32_t bailout;

		memset(regs, 0, sizeof(regs));
		*runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.opaque = &memory,
			.load = jit_test_load,
			.chain_executed = 13u,
			.chain_instruction_limit = 13u + 3u + 19u,
			.sites = jit_slot_sites(0),
			.site_count = jit_entries[0].memory_site_count,
			.result = { .executed = UINT32_MAX, .native_successor = 1u },
		};
		runtime->read_pages[0x20u] = (struct rv32_native_jit_read_page) {
			.virtual_page = 0x20u,
			.host_bias = (uint32_t)(uintptr_t)values - address,
		};
		runtime->read_pages[0x22u] = (struct rv32_native_jit_read_page) {
			.virtual_page = 0x22u,
			.host_bias = (uint32_t)(uintptr_t)&memory.value - 0x22000u,
		};
		regs[2] = 3u;
		regs[4] = address;
		regs[6] = 0x22000u;
		bailout = rv32_native_jit_enter(block, runtime);
		if (regs[9] != 3u || runtime->result.native_slot != 0u ||
		    runtime->result.native_successor != 0u ||
		    runtime->chain_links != (RV32_NATIVE_JIT_RUNTIME_COUNTERS ? 1u : 0u))
			return false;
		if (cold) {
			if (bailout == 0u || runtime->chain_executed != 16u)
				return false;
			jit_resolve_memory_bailout(runtime, bailout);
			if (runtime->result.trap != 6u ||
			    runtime->result.trap_value != 0x21000u ||
			    runtime->result.next_pc != 0x16004u ||
			    runtime->result.executed != 7u ||
			    regs[1] != 101u || regs[2] != 2u || regs[3] != 11u ||
			    regs[4] != 0x21000u || regs[5] != 71u)
				return false;
		} else if (bailout != 0u || runtime->result.executed != 19u ||
			   runtime->chain_executed != 35u ||
			   runtime->result.next_pc != 0x1601cu ||
			   regs[1] != 103u || regs[2] != 0u || regs[3] != 33u ||
			   regs[4] != address + 12u || regs[5] != 71u) {
			return false;
		}
	}
	/* Stores retain checked page translation and framebuffer dirty reporting
	 * while the preheader and guest register commits are paid only once. */
	{
		uint16_t pixels[4] = { 0u, 0u, 0u, 0x55aau };

		if (!jit_compile(0u, (const uint8_t *)store_program, 0u,
				 0x17000u, 4u))
			return false;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		memset(regs, 0, sizeof(regs));
		*runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.chain_instruction_limit = 10u,
			.dirty_epoch = 1u,
		};
		runtime->result.linear_dirty_start = UINT32_MAX;
		runtime->write_pages[0x23u] = (struct rv32_native_jit_write_page) {
			.virtual_page = 0x23u,
			.host_bias = (uint32_t)(uintptr_t)pixels - 0x23000u,
			.linear_page_start = 0x1000u,
			.linear_page_end = 0x2000u,
		};
		regs[2] = 0x23006u;
		regs[3] = 0x1234u;
		regs[4] = 0x23000u;
		if (rv32_native_jit_enter(block, runtime) != 0u ||
		    runtime->result.executed != 10u ||
		    runtime->chain_executed != 10u ||
		    runtime->result.next_pc != 0x17010u ||
		    runtime->result.linear_dirty_start != 0x1000u ||
		    runtime->result.linear_dirty_end != 0x2000u ||
		    regs[1] != 100u || regs[4] != 0x23006u ||
		    pixels[0] != 0x1234u || pixels[1] != 0x1234u ||
		    pixels[2] != 0x1234u || pixels[3] != 0x55aau)
			return false;
		memset(&runtime->result, 0, sizeof(runtime->result));
		runtime->chain_executed = 0u;
		regs[1] = 0u;
		regs[3] = 0xabcdu;
		regs[4] = 0x23001u;
		if (rv32_native_jit_enter(block, runtime) == 0u ||
		    runtime->memory_address != 0x23001u ||
		    runtime->result.executed != 1u ||
		    regs[1] != 100u || regs[4] != 0x23001u ||
		    pixels[0] != 0x1234u || pixels[1] != 0x1234u)
			return false;
	}
#if CONFIG_RV32_HOST_PERF_STATS
	{
		static struct rv32_native_jit_stats stats; /* off the main stack */

		jit_profile_discarded_guest_instructions = 11u;
		jit_profile_guest_instructions[0] = 17u;
		jit_profile_guest_instructions[1] = 19u;
		rv32_native_jit_flush();
		rv32_native_jit_get_stats(&stats);
		if (stats.profile_interval_guest_instructions != 0u ||
		    stats.profile_discarded_guest_instructions != 47u)
			return false;
		rv32_native_jit_get_stats(&stats);
		if (stats.profile_discarded_guest_instructions != 0u)
			return false;
	}
#endif
	return true;
}

/* Kernel linear-map loads and stores on the device itself: a load group, a
 * store group and misaligned singles through both forms of the check, then
 * a zero limit and bases just outside the window, which must leave through
 * the page-cache path without touching memory.  Runs before binding, with a
 * borrowed bank whose window covers a small local buffer. */
static __attribute__((noinline)) bool jit_self_test_kernel_linear(void)
{
	struct rv32_native_jit_runtime *runtime = jit_rt;
	const uint32_t window = 0xc0000000u;
	const uint32_t base_pc = window + 0x10000u;
	const uint32_t data_va = window + JIT_LINEAR_LOW_MARGIN;
	const uint32_t program[] = {
		rv_i(0x03u, 1u, 2u, 4u, 0),       /* lw x1, 0(x4) */
		rv_i(0x03u, 2u, 4u, 4u, 5),       /* lbu x2, 5(x4) */
		rv_i(0x03u, 3u, 1u, 4u, 6),       /* lh x3, 6(x4) */
		rv_s(0x23u, 2u, 5u, 1u, 8),       /* sw x1, 8(x5) */
		rv_s(0x23u, 0u, 5u, 2u, 13),      /* sb x2, 13(x5) */
		rv_s(0x23u, 1u, 5u, 3u, 14),      /* sh x3, 14(x5) */
		rv_i(0x03u, 6u, 2u, 7u, 1),       /* lw x6, 1(x7): misaligned */
		rv_i(0x03u, 8u, 0u, 7u, 3),       /* lb x8, 3(x7) */
		rv_s(0x23u, 1u, 9u, 6u, 2),       /* sh x6, 2(x9): misaligned */
		rv_b(0u, 0u, 0u, 8),              /* beq x0, x0, +8 */
	};
	const uint32_t count = sizeof(program) / sizeof(program[0]);
	struct jit_test_memory memory = { .address = 0u };
	_Alignas(4) uint8_t data[64];
	_Alignas(4) uint8_t expected[64];
	uint32_t regs[32];
	uint32_t saved_base = jit_kernel_linear_base;
	uint32_t saved_size = jit_kernel_linear_size;
	bool saved_data = jit_kernel_linear_data;
	bool ok = true;

	jit_kernel_linear_base = window;
	jit_kernel_linear_size = 0x01000000u;
	jit_kernel_linear_data = true;
	for (uint32_t pass = 0; pass < 5u && ok; pass++) {
		/* Passes 0-1: in the window, three then two registers; 2: zero
		 * limit; 3: base at the limit; 4: base below the window start. */
		uint32_t base = data_va + (pass == 3u ? 64u : 0u) -
			(pass == 4u ? 4u : 0u);
		uint32_t value;
		uint32_t bailout;

		jit_linear_register_budget = pass == 1u ? 2u : 3u;
		if (!jit_compile(0u, (const uint8_t *)program, 0u, base_pc,
				 count)) {
			ok = false;
			break;
		}
		for (uint32_t i = 0; i < sizeof(data); i++)
			expected[i] = data[i] = (uint8_t)(i * 37u + 0x85u);
		memset(regs, 0, sizeof(regs));
		regs[4] = base;
		regs[5] = base + 16u;
		regs[7] = base + 32u;
		regs[9] = base + 41u;
		*runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.opaque = &memory,
			.load = jit_test_load,
			.store = jit_test_store,
			.linear_limit = pass == 2u ? 0u : 64u,
			.linear_host = (uint32_t)(uintptr_t)data,
		};
		for (uint32_t i = 0; i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++) {
			runtime->read_pages[i].virtual_page = UINT32_MAX;
			runtime->write_pages[i].virtual_page = UINT32_MAX;
			runtime->write_pages[i].linear_page_start = UINT32_MAX;
		}
		bailout = rv32_native_jit_enter(
			(jit_block_fn)(uintptr_t)jit_entry_code(0u), runtime);
		if (pass >= 2u) {
			/* The first group leaves before any access or write. */
			ok = bailout == 1u && runtime->memory_calls == 0u &&
				runtime->memory_address == base && regs[1] == 0u;
			for (uint32_t i = 0; i < sizeof(data); i++)
				ok = ok && data[i] == expected[i];
			continue;
		}
		memcpy(&value, &expected[0], 4u);
		ok = bailout == 0u && runtime->result.trap == 0u &&
			runtime->result.executed == count &&
			runtime->result.next_pc == base_pc + count * 4u + 4u &&
			runtime->memory_calls == 0u && regs[1] == value &&
			regs[2] == expected[5] &&
			regs[3] == (uint32_t)(int32_t)(int16_t)
				(expected[6] | (expected[7] << 8));
		memcpy(&expected[24], &value, 4u);
		expected[29] = (uint8_t)regs[2];
		expected[30] = (uint8_t)regs[3];
		expected[31] = (uint8_t)(regs[3] >> 8);
		memcpy(&value, &expected[33], 4u);
		ok = ok && regs[6] == value &&
			regs[8] == (uint32_t)(int32_t)(int8_t)expected[35];
		expected[43] = (uint8_t)value;
		expected[44] = (uint8_t)(value >> 8);
		for (uint32_t i = 0; i < sizeof(data); i++)
			ok = ok && data[i] == expected[i];
		/* The page caches never learned a linear-map page. */
		for (uint32_t i = 0; i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++)
			ok = ok && runtime->read_pages[i].virtual_page == UINT32_MAX &&
				runtime->write_pages[i].virtual_page == UINT32_MAX;
	}
	jit_linear_register_budget = 3u;
	jit_kernel_linear_base = saved_base;
	jit_kernel_linear_size = saved_size;
	jit_kernel_linear_data = saved_data;
	jit_forget_entry(0u);
	memset(jit_entries, 0, sizeof(jit_entries));
	memset(jit_hot_entries, 0, sizeof(jit_hot_entries));
	jit_reset_code_heads();
#if JIT_KEEP_KERNEL
	memset(jit_kernel_code_pages, 0, sizeof(jit_kernel_code_pages));
	jit_user_slot_count = 0u;
#endif
	return ok;
}

static bool jit_self_test(void)
{
	/* Borrow the user bank, cleared again by init, instead of putting
	 * 2.5 KiB of page caches on the 4 KiB main-task stack. */
#define runtime (*jit_rt)
	uint32_t program[JIT_MAX_GUEST_INSTRUCTIONS] = {
		rv_i(0x13u, 1u, 0u, 0u, 7),      /* addi x1, x0, 7 */
		rv_i(0x13u, 1u, 0u, 1u, 1),      /* addi x1, x1, 1 */
		rv_i(0x13u, 1u, 0u, 1u, -1),     /* addi x1, x1, -1 */
		rv_s(0x23u, 2u, 4u, 1u, 0),      /* sw x1, 0(x4) */
		rv_i(0x03u, 2u, 2u, 4u, 0),      /* lw x2, 0(x4) */
		rv_r(0x33u, 2u, 0u, 2u, 1u, 0), /* add x2, x2, x1 */
		rv_i(0x13u, 2u, 0u, 2u, -7),     /* addi x2, x2, -7 */
		rv_b(0u, 1u, 2u, 8),              /* beq x1, x2, +8 */
	};
	struct jit_test_memory memory = {
		.address = 0x2000u,
	};
	uint32_t loop_program[5] = {
		rv_i(0x13u, 1u, 0u, 1u, 1),  /* addi x1, x1, 1 */
		rv_i(0x13u, 1u, 0u, 1u, 1),  /* addi x1, x1, 1 */
		rv_i(0x13u, 1u, 0u, 1u, 1),  /* addi x1, x1, 1 */
		rv_i(0x13u, 2u, 0u, 2u, -1), /* addi x2, x2, -1 */
		rv_b(1u, 2u, 0u, -16),        /* bne x2, x0, block start */
	};
	uint32_t long_loop_program[13] = {
		rv_i(0x13u, 1u, 0u, 1u, 1),       /* addi x1, x1, 1 */
		rv_i(0x03u, 3u, 2u, 4u, 0),       /* lw x3, 0(x4) */
		rv_r(0x33u, 3u, 0u, 3u, 1u, 0u), /* add x3, x3, x1 */
		rv_s(0x23u, 2u, 4u, 3u, 0),       /* sw x3, 0(x4) */
		rv_r(0x33u, 5u, 4u, 5u, 1u, 0u), /* xor x5, x5, x1 */
		rv_i(0x13u, 6u, 0u, 6u, 3),       /* addi x6, x6, 3 */
		rv_i(0x13u, 6u, 7u, 6u, 255),     /* andi x6, x6, 255 */
		rv_r(0x33u, 7u, 0u, 7u, 6u, 0u), /* add x7, x7, x6 */
		rv_r(0x33u, 7u, 0u, 7u, 1u, 0x20u), /* sub x7, x7, x1 */
		rv_i(0x13u, 8u, 0u, 8u, 1),       /* addi x8, x8, 1 */
		rv_i(0x13u, 9u, 0u, 9u, -1),      /* addi x9, x9, -1 */
		rv_i(0x13u, 2u, 0u, 2u, -1),      /* addi x2, x2, -1 */
		rv_b(1u, 2u, 0u, -48),            /* bne x2, x0, block start */
	};
	uint32_t four_read_loop_program[10] = {
		rv_i(0x03u, 3u, 2u, 4u, 0),       /* lw x3, 0(x4) */
		rv_i(0x03u, 5u, 2u, 7u, 0),       /* lw x5, 0(x7) */
		rv_i(0x03u, 8u, 2u, 10u, 0),      /* lw x8, 0(x10) */
		rv_i(0x03u, 11u, 2u, 13u, 0),     /* lw x11, 0(x13) */
		rv_r(0x33u, 6u, 0u, 3u, 5u, 0u), /* add x6, x3, x5 */
		rv_r(0x33u, 6u, 0u, 6u, 8u, 0u), /* add x6, x6, x8 */
		rv_r(0x33u, 6u, 0u, 6u, 11u, 0u),/* add x6, x6, x11 */
		rv_i(0x13u, 1u, 0u, 1u, 1),      /* addi x1, x1, 1 */
		rv_i(0x13u, 2u, 0u, 2u, -1),     /* addi x2, x2, -1 */
		rv_b(1u, 2u, 0u, -36),            /* bne x2, x0, block start */
	};
	uint32_t grouped_memory_program[5] = {
		rv_i(0x03u, 1u, 2u, 4u, 0),  /* lw x1, 0(x4) */
		rv_i(0x03u, 2u, 2u, 4u, 0),  /* lw x2, 0(x4) */
		rv_s(0x23u, 2u, 4u, 1u, 0),  /* sw x1, 0(x4) */
		rv_s(0x23u, 2u, 4u, 2u, 0),  /* sw x2, 0(x4) */
		rv_b(0u, 0u, 0u, 8),         /* beq x0, x0, +8 */
	};
	uint32_t grouped_range_program[3] = {
		rv_i(0x03u, 1u, 2u, 4u, 0),  /* lw x1, 0(x4) */
		rv_i(0x03u, 2u, 2u, 4u, 4),  /* lw x2, 4(x4) */
		rv_b(0u, 0u, 0u, 8),         /* beq x0, x0, +8 */
	};
	uint32_t interleaved_memory_program[4] = {
		rv_i(0x03u, 1u, 2u, 4u, 0),  /* lw x1, 0(x4) */
		rv_i(0x13u, 5u, 0u, 5u, 1),  /* addi x5, x5, 1 */
		rv_i(0x03u, 2u, 2u, 4u, 0),  /* lw x2, 0(x4) */
		rv_b(0u, 0u, 0u, 8),         /* beq x0, x0, +8 */
	};
	uint32_t forward_region_program[6] = {
		rv_i(0x13u, 1u, 0u, 1u, 1),  /* addi x1, x1, 1 */
		rv_b(0u, 2u, 0u, 12),        /* beq x2, x0, index 4 */
		rv_i(0x13u, 1u, 0u, 1u, 10), /* addi x1, x1, 10 */
		rv_i(0x13u, 3u, 0u, 3u, 1),  /* addi x3, x3, 1 */
		rv_i(0x13u, 1u, 0u, 1u, 2),  /* addi x1, x1, 2 */
		rv_b(0u, 0u, 0u, 8),         /* beq x0, x0, exit */
	};
	uint32_t side_exit_program[6] = {
		rv_i(0x13u, 1u, 0u, 1u, 1),  /* addi x1, x1, 1 */
		rv_b(0u, 2u, 0u, 64),        /* beq x2, x0, outside region */
		rv_i(0x13u, 1u, 0u, 1u, 10), /* addi x1, x1, 10 */
		rv_i(0x13u, 3u, 0u, 3u, 1),  /* addi x3, x3, 1 */
		rv_i(0x13u, 1u, 0u, 1u, 2),  /* addi x1, x1, 2 */
		rv_b(0u, 0u, 0u, 8),         /* beq x0, x0, exit */
	};
	uint32_t forward_memory_program[6] = {
		rv_i(0x13u, 1u, 0u, 1u, 1), /* addi x1, x1, 1 */
		rv_b(0u, 2u, 0u, 12),       /* beq x2, x0, index 4 */
		rv_i(0x13u, 1u, 0u, 1u, 10),/* addi x1, x1, 10 */
		rv_i(0x13u, 3u, 0u, 3u, 1), /* addi x3, x3, 1 */
		rv_i(0x03u, 4u, 2u, 5u, 0), /* lw x4, 0(x5) */
		rv_b(0u, 0u, 0u, 8),        /* beq x0, x0, exit */
	};
	uint32_t forward_loop_program[5] = {
		rv_i(0x13u, 1u, 0u, 1u, 1),  /* addi x1, x1, 1 */
		rv_b(0u, 3u, 0u, 8),         /* beq x3, x0, index 3 */
		rv_i(0x13u, 1u, 0u, 1u, 10), /* addi x1, x1, 10 */
		rv_i(0x13u, 2u, 0u, 2u, -1), /* addi x2, x2, -1 */
		rv_b(1u, 2u, 0u, -16),       /* bne x2, x0, region start */
	};
	uint32_t jump_loop_program[4] = {
		rv_i(0x13u, 1u, 0u, 1u, 1), /* addi x1, x1, 1 */
		rv_i(0x13u, 2u, 0u, 2u, -1),/* addi x2, x2, -1 */
		rv_b(0u, 2u, 0u, 8),        /* beq x2, x0, region exit */
		rv_j(0u, -12),               /* jal x0, region start */
	};
	uint32_t jump_trace_program[6] = {
		rv_i(0x13u, 1u, 0u, 1u, 1),  /* addi x1, x1, 1 */
		rv_j(0u, 8),                   /* jal x0, index 3 */
		rv_i(0x13u, 1u, 0u, 1u, 100),/* skipped */
		rv_i(0x13u, 1u, 0u, 1u, 2),  /* addi x1, x1, 2 */
		rv_i(0x13u, 2u, 0u, 2u, 3),  /* addi x2, x2, 3 */
		rv_b(1u, 0u, 0u, -4),         /* bne x0, x0, fall through */
	};
	uint32_t jump_branch_trace_program[6] = {
		rv_b(0u, 6u, 0u, 16),         /* beq x6, x0, index 4 */
		rv_i(0x13u, 1u, 0u, 1u, 1), /* addi x1, x1, 1 */
		rv_j(0u, 8),                  /* jal x0, index 4 */
		rv_i(0x13u, 1u, 0u, 1u, 100),/* skipped */
		rv_i(0x13u, 1u, 0u, 1u, 2), /* addi x1, x1, 2 */
		rv_b(1u, 0u, 0u, -4),        /* bne x0, x0, fall through */
	};
	uint32_t jump_memory_trace_program[6] = {
		rv_i(0x13u, 1u, 0u, 1u, 1), /* addi x1, x1, 1 */
		rv_j(0u, 8),                  /* jal x0, index 3 */
		rv_i(0x13u, 1u, 0u, 1u, 100),/* skipped */
		rv_i(0x03u, 2u, 2u, 4u, 0), /* lw x2, 0(x4) */
		rv_i(0x13u, 3u, 0u, 3u, 3), /* addi x3, x3, 3 */
		rv_b(1u, 0u, 0u, -4),        /* bne x0, x0, fall through */
	};
	uint32_t call_trace_program[8] = {
		rv_i(0x13u, 2u, 0u, 2u, 1), /* addi x2, x2, 1 */
		rv_j(1u, 12),                 /* jal ra, index 4 */
		rv_i(0x13u, 3u, 0u, 3u, 3), /* return: addi x3, x3, 3 */
		rv_b(0u, 0u, 0u, 20),        /* beq x0, x0, outside region */
		rv_i(0x13u, 4u, 0u, 4u, 4), /* callee: addi x4, x4, 4 */
		rv_i(0x67u, 0u, 0u, 1u, 0), /* ret */
	};
	uint32_t call_guard_program[5] = {
		rv_j(1u, 12),                 /* jal ra, index 3 */
		rv_i(0x13u, 2u, 0u, 2u, 1), /* expected return continuation */
		rv_b(0u, 0u, 0u, 16),        /* terminal */
		rv_i(0x13u, 1u, 0u, 1u, 4), /* callee changes its link register */
		rv_i(0x67u, 0u, 0u, 1u, 0), /* guarded ret */
	};
	uint32_t moving_store_loop_program[7] = {
		rv_s(0x23u, 2u, 4u, 3u, 0),  /* sw x3, 0(x4) */
		rv_i(0x13u, 5u, 0u, 5u, 1), /* addi x5, x5, 1 */
		rv_s(0x23u, 2u, 4u, 3u, 4),  /* sw x3, 4(x4) */
		rv_i(0x13u, 4u, 0u, 4u, 8), /* addi x4, x4, 8 */
		rv_i(0x13u, 2u, 0u, 2u, -1),/* addi x2, x2, -1 */
		rv_b(0u, 2u, 0u, 8),        /* beq x2, x0, region exit */
		rv_j(0u, -24),               /* jal x0, region start */
	};

	for (uint32_t pass = 0; pass < 2u; pass++) {
		for (uint32_t index = 0; index < JIT_CACHE_SLOTS; index++) {
			uint32_t slot = pass == 0u ? index :
				JIT_CACHE_SLOTS - 1u - index;
			uint32_t regs[32] = { 0 };
			runtime = (struct rv32_native_jit_runtime) {
				.regs = regs,
				.opaque = &memory,
				.load = jit_test_load,
				.store = jit_test_store,
			};
			jit_block_fn block;
			uint32_t bailout;

			for (uint32_t i = 0;
			     i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++) {
				runtime.read_pages[i].virtual_page = UINT32_MAX;
				runtime.write_pages[i].virtual_page = UINT32_MAX;
				runtime.write_pages[i].linear_page_start = UINT32_MAX;
			}

			regs[4] = memory.address;
			memory.value = 0u;
			if (!jit_compile(slot, (const uint8_t *)program, 0u,
					 0x1000u, JIT_MAX_GUEST_INSTRUCTIONS))
				return false;
			runtime.sites = jit_slot_sites(slot);
			runtime.site_count = jit_entries[slot].memory_site_count;
			block = (jit_block_fn)(uintptr_t)jit_entry_code(slot);
			bailout = rv32_native_jit_enter(block, &runtime);
			if (bailout == 0u)
				return false;
			jit_resolve_memory_bailout(&runtime, bailout);
			if (runtime.result.trap != 0u ||
			    runtime.result.executed != 4u ||
			    runtime.result.next_pc != 0x1010u || memory.value != 7u ||
			    regs[1] != 7u || regs[2] != 0u || regs[0] != 0u ||
			    runtime.memory_calls != 1u)
				return false;

			/* The store page is cached. The next pass reaches the first load,
			 * resolves it once, and seeds the independent read-page cache. */
			memory.value = 0u;
			runtime.result.next_pc = 0u;
			runtime.result.executed = 0u;
			runtime.result.trap = 0u;
			runtime.result.trap_value = 0u;
			runtime.result.linear_dirty_start = UINT32_MAX;
			runtime.result.linear_dirty_end = 0u;
			runtime.memory_calls = 0u;
			bailout = rv32_native_jit_enter(block, &runtime);
			if (bailout == 0u)
				return false;
			jit_resolve_memory_bailout(&runtime, bailout);
			if (runtime.result.trap != 0u ||
			    runtime.result.executed != 5u ||
			    runtime.result.next_pc != 0x1014u || memory.value != 7u ||
			    regs[1] != 7u || regs[2] != 7u || regs[0] != 0u ||
			    runtime.memory_calls != 1u)
				return false;

			/* Both pages are now direct native accesses. */
			memory.value = 0u;
			memset(&runtime.result, 0, sizeof(runtime.result));
			runtime.result.linear_dirty_start = UINT32_MAX;
			runtime.memory_calls = 0u;
			bailout = rv32_native_jit_enter(block, &runtime);
			if (bailout != 0u || runtime.result.trap != 0u ||
			    runtime.result.executed != 8u ||
			    runtime.result.next_pc != 0x1024u || memory.value != 7u ||
			    regs[1] != 7u || regs[2] != 7u || regs[0] != 0u ||
			    runtime.memory_calls != 0u)
				return false;

			/* A fault after mapped-register writes must commit only the
			 * instructions before the fault and leave later destinations at
			 * their incoming values. */
			memset(regs, 0, sizeof(regs));
			regs[4] = memory.address + 0x1000u;
			runtime.result.next_pc = 0u;
			runtime.result.executed = 0u;
			runtime.result.trap = 0u;
			runtime.result.trap_value = 0u;
			runtime.memory_calls = 0u;
			bailout = rv32_native_jit_enter(block, &runtime);
			if (bailout == 0u)
				return false;
			jit_resolve_memory_bailout(&runtime, bailout);
			if (runtime.result.trap != 8u ||
			    runtime.result.executed != 3u ||
			    runtime.result.next_pc != 0x100cu ||
			    runtime.result.trap_value != memory.address + 0x1000u ||
			    regs[1] != 7u || regs[2] != 0u || regs[0] != 0u ||
			    runtime.memory_calls != 1u)
				return false;
		}
	}

	/* Exercise the configured same-page trace width itself.  This catches a
	 * packed instruction-count, workspace or publication regression whenever
	 * the generic region limit is raised to reduce transition overhead. */
	{
		uint32_t regs[32] = { 0 };
		jit_block_fn block;

		for (uint32_t i = 0; i + 1u < JIT_MAX_GUEST_INSTRUCTIONS; i++)
			program[i] = rv_i(0x13u, 1u, 0u, 1u, 1);
		program[JIT_MAX_GUEST_INSTRUCTIONS - 1u] =
			rv_b(1u, RV_X0, RV_X0, 8); /* never taken */
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
		};
		if (!jit_compile(0u, (const uint8_t *)program, 0u, 0x18000u,
				 JIT_MAX_GUEST_INSTRUCTIONS) ||
		    jit_entries[0].guest_instruction_count !=
			    JIT_MAX_GUEST_INSTRUCTIONS)
			return false;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != JIT_MAX_GUEST_INSTRUCTIONS ||
		    runtime.result.next_pc !=
			    0x18000u + JIT_MAX_GUEST_INSTRUCTIONS * sizeof(uint32_t) ||
		    regs[1] != JIT_MAX_GUEST_INSTRUCTIONS - 1u)
			return false;
	}

	/* Consecutive same-page accesses share one native page lookup. Verify that
	 * each cold direction still completes exactly its first operation and that
	 * the fully warm group preserves all architectural results. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.opaque = &memory,
			.load = jit_test_load,
			.store = jit_test_store,
		};
		jit_block_fn block;
		uint32_t bailout;

		for (uint32_t i = 0;
		     i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++) {
			runtime.read_pages[i].virtual_page = UINT32_MAX;
			runtime.write_pages[i].virtual_page = UINT32_MAX;
			runtime.write_pages[i].linear_page_start = UINT32_MAX;
		}
		regs[4] = memory.address;
		memory.value = 0x11223344u;
		if (!jit_compile(0u, (const uint8_t *)grouped_memory_program, 0u,
				 0x6000u, 5u) || jit_entries[0].memory_site_count != 2u)
			return false;
		runtime.sites = jit_slot_sites(0);
		runtime.site_count = jit_entries[0].memory_site_count;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);

		bailout = rv32_native_jit_enter(block, &runtime);
		if (bailout == 0u)
			return false;
		jit_resolve_memory_bailout(&runtime, bailout);
		if (runtime.result.trap != 0u ||
		    runtime.result.executed != 1u ||
		    runtime.result.next_pc != 0x6004u ||
		    regs[1] != memory.value || regs[2] != 0u ||
		    runtime.memory_calls != 1u)
			return false;

		memset(&runtime.result, 0, sizeof(runtime.result));
		runtime.memory_calls = 0u;
		bailout = rv32_native_jit_enter(block, &runtime);
		if (bailout == 0u)
			return false;
		jit_resolve_memory_bailout(&runtime, bailout);
		if (runtime.result.trap != 0u ||
		    runtime.result.executed != 3u ||
		    runtime.result.next_pc != 0x600cu ||
		    regs[1] != memory.value || regs[2] != memory.value ||
		    runtime.memory_calls != 1u)
			return false;

		memset(&runtime.result, 0, sizeof(runtime.result));
		runtime.result.linear_dirty_start = UINT32_MAX;
		runtime.memory_calls = 0u;
		runtime.dirty_epoch = 1u;
		{
			struct rv32_native_jit_write_page *page =
				&runtime.write_pages[(memory.address >> 12) &
					(RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u)];

			page->linear_page_start = 0x1000u;
			page->linear_page_end = 0x2000u;
		}
		bailout = rv32_native_jit_enter(block, &runtime);
		if (bailout != 0u || runtime.result.trap != 0u ||
		    runtime.result.executed != 5u ||
		    runtime.result.next_pc != 0x6018u ||
		    regs[1] != memory.value || regs[2] != memory.value ||
		    runtime.result.linear_dirty_start != 0x1000u ||
		    runtime.result.linear_dirty_end != 0x2000u ||
		    runtime.memory_calls != 0u)
			return false;
#if CONFIG_RV32_HOST_PERF_STATS
		if (runtime.result.linear_store_calls != 2u ||
		    runtime.result.linear_store_bytes != 8u)
			return false;
#else
		if (runtime.result.linear_store_calls != 0u ||
		    runtime.result.linear_store_bytes != 0u)
			return false;
#endif
	}

	/* An invariant base can retain its checked host translation across integer
	 * work without moving either memory operation.  A cold guard still resolves
	 * exactly the first load; the warm path performs both in place. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.opaque = &memory,
			.load = jit_test_load,
			.store = jit_test_store,
		};
		jit_block_fn block;
		uint32_t bailout;

		for (uint32_t i = 0;
		     i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++) {
			runtime.read_pages[i].virtual_page = UINT32_MAX;
			runtime.write_pages[i].virtual_page = UINT32_MAX;
			runtime.write_pages[i].linear_page_start = UINT32_MAX;
		}
		memory.address = 0x2000u;
		memory.value = 0x89abcdefu;
		regs[4] = memory.address;
		if (!jit_compile(0u, (const uint8_t *)interleaved_memory_program,
				 0u, 0x6800u, 4u) ||
		    jit_entries[0].memory_site_count != 1u)
			return false;
		runtime.sites = jit_slot_sites(0);
		runtime.site_count = jit_entries[0].memory_site_count;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		bailout = rv32_native_jit_enter(block, &runtime);
		if (bailout == 0u)
			return false;
		jit_resolve_memory_bailout(&runtime, bailout);
		if (runtime.result.trap != 0u ||
		    runtime.result.executed != 1u ||
		    runtime.result.next_pc != 0x6804u ||
		    regs[1] != memory.value || regs[2] != 0u || regs[5] != 0u ||
		    runtime.memory_calls != 1u)
			return false;

		memset(&runtime.result, 0, sizeof(runtime.result));
		runtime.memory_calls = 0u;
		bailout = rv32_native_jit_enter(block, &runtime);
		if (bailout != 0u || runtime.result.trap != 0u ||
		    runtime.result.executed != 4u ||
		    runtime.result.next_pc != 0x6814u ||
		    regs[1] != memory.value || regs[2] != memory.value ||
		    regs[5] != 1u || runtime.memory_calls != 0u)
			return false;
	}

	/* A coalesced group that crosses a page or violates natural alignment must
	 * leave every operation after the first to the interpreter. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.opaque = &memory,
			.load = jit_test_load,
			.store = jit_test_store,
		};
		jit_block_fn block;
		uint32_t bailout;

		for (uint32_t i = 0;
		     i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++) {
			runtime.read_pages[i].virtual_page = UINT32_MAX;
			runtime.write_pages[i].virtual_page = UINT32_MAX;
			runtime.write_pages[i].linear_page_start = UINT32_MAX;
		}
		memory.address = 0x2ffcu;
		memory.value = 0xa5a55a5au;
		regs[4] = memory.address;
		if (!jit_compile(0u, (const uint8_t *)grouped_range_program, 0u,
				 0x7000u, 3u))
			return false;
		runtime.sites = jit_slot_sites(0);
		runtime.site_count = jit_entries[0].memory_site_count;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		bailout = rv32_native_jit_enter(block, &runtime);
		if (bailout == 0u)
			return false;
		jit_resolve_memory_bailout(&runtime, bailout);
		if (runtime.result.trap != 0u ||
		    runtime.result.executed != 1u ||
		    runtime.result.next_pc != 0x7004u ||
		    regs[1] != memory.value || regs[2] != 0u ||
		    runtime.memory_calls != 1u)
			return false;

		memory.address = 0x2000u;
		regs[4] = memory.address + 1u;
		memset(&runtime.result, 0, sizeof(runtime.result));
		runtime.memory_calls = 0u;
		bailout = rv32_native_jit_enter(block, &runtime);
		if (bailout == 0u)
			return false;
		jit_resolve_memory_bailout(&runtime, bailout);
		if (runtime.result.trap != 6u ||
		    runtime.result.executed != 0u ||
		    runtime.result.next_pc != 0x7000u ||
		    runtime.result.trap_value != memory.address + 1u ||
		    runtime.memory_calls != 1u)
			return false;
	}

	/* A forward branch may stay inside one native region.  Verify both paths
	 * and their exact dynamic instruction counts at the merge. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
		};
		jit_block_fn block;

		if (!jit_compile(0u, (const uint8_t *)forward_region_program, 0u,
				 0x8000u, 6u) ||
		    jit_entries[0].guest_instruction_count != 6u)
			return false;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 4u ||
		    runtime.result.next_pc != 0x801cu ||
		    regs[1] != 3u || regs[3] != 0u)
			return false;

		memset(regs, 0, sizeof(regs));
		memset(&runtime.result, 0, sizeof(runtime.result));
		regs[2] = 1u;
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 6u ||
		    runtime.result.next_pc != 0x801cu ||
		    regs[1] != 13u || regs[3] != 1u)
			return false;
	}

	/* A taken guard outside the region must commit early, while its untaken
	 * fall-through continues through the remaining native instructions. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
		};
		jit_block_fn block;

		if (!jit_compile(0u, (const uint8_t *)side_exit_program, 0u,
				 0x9000u, 6u))
			return false;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 2u ||
		    runtime.result.next_pc != 0x9044u ||
		    regs[1] != 1u || regs[3] != 0u)
			return false;

		memset(regs, 0, sizeof(regs));
		memset(&runtime.result, 0, sizeof(runtime.result));
		regs[2] = 1u;
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 6u ||
		    runtime.result.next_pc != 0x901cu ||
		    regs[1] != 13u || regs[3] != 1u)
			return false;
	}

	/* A cold memory exit after a merge combines the dynamic counter with the instruction
	 * offset inside the current segment.  Test both the taken and fall-through
	 * paths so a skipped segment can neither disappear nor be double-counted. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.opaque = &memory,
			.load = jit_test_load,
			.store = jit_test_store,
		};
		jit_block_fn block;
		uint32_t bailout;

		for (uint32_t i = 0;
		     i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++) {
			runtime.read_pages[i].virtual_page = UINT32_MAX;
			runtime.write_pages[i].virtual_page = UINT32_MAX;
			runtime.write_pages[i].linear_page_start = UINT32_MAX;
		}
		memory.address = 0x2000u;
		memory.value = 0x12345678u;
		regs[5] = memory.address;
		if (!jit_compile(0u, (const uint8_t *)forward_memory_program, 0u,
				 0xa000u, 6u))
			return false;
		runtime.sites = jit_slot_sites(0);
		runtime.site_count = jit_entries[0].memory_site_count;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		bailout = rv32_native_jit_enter(block, &runtime);
		if (bailout == 0u)
			return false;
		jit_resolve_memory_bailout(&runtime, bailout);
		if (runtime.result.trap != 0u ||
		    runtime.result.executed != 3u ||
		    runtime.result.next_pc != 0xa014u ||
		    regs[1] != 1u || regs[3] != 0u ||
		    regs[4] != memory.value || runtime.memory_calls != 1u)
			return false;

		memset(regs, 0, sizeof(regs));
		memset(&runtime.result, 0, sizeof(runtime.result));
		for (uint32_t i = 0;
		     i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++)
			runtime.read_pages[i].virtual_page = UINT32_MAX;
		regs[2] = 1u;
		regs[5] = memory.address;
		runtime.memory_calls = 0u;
		bailout = rv32_native_jit_enter(block, &runtime);
		if (bailout == 0u)
			return false;
		jit_resolve_memory_bailout(&runtime, bailout);
		if (runtime.result.trap != 0u ||
		    runtime.result.executed != 5u ||
		    runtime.result.next_pc != 0xa014u ||
		    regs[1] != 11u || regs[3] != 1u ||
		    regs[4] != memory.value || runtime.memory_calls != 1u)
			return false;
	}

	/* A loop containing an internal forward branch must accumulate a different
	 * instruction count for each path without weakening its native budget. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.loop_iterations = JIT_SELF_TEST_LOOP_ITERATIONS,
		};
		jit_block_fn block;

		if (!jit_compile(0u, (const uint8_t *)forward_loop_program, 0u,
				 0xb000u, 5u) || !jit_entries[0].loops_to_start)
			return false;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		regs[2] = 3u;
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 12u ||
		    runtime.result.next_pc != 0xb014u ||
		    regs[1] != 3u || regs[2] != 0u)
			return false;

		memset(regs, 0, sizeof(regs));
		memset(&runtime.result, 0, sizeof(runtime.result));
		regs[2] = 3u;
		regs[3] = 1u;
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 15u ||
		    runtime.result.next_pc != 0xb014u ||
		    regs[1] != 33u || regs[2] != 0u)
			return false;
	}

	/* Verify bounded native back-edge execution and exact dynamic instruction
	 * accounting independently of any application or guest address. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
		};
		jit_block_fn block;
		uint32_t bailout;

		if (!jit_compile(0u, (const uint8_t *)loop_program, 0u,
				 0x4000u, 5u) || !jit_entries[0].loops_to_start)
			return false;
		{
			uint32_t *main_code = jit_entry_code(0u);
			uint32_t *selected = main_code;

			block = (jit_block_fn)(uintptr_t)main_code;
			for (uint32_t call = 0;
			     call < JIT_HOT_LOOP_ADMIT_CALLS; call++) {
				selected = jit_dispatch_code(0u, &jit_entries[0]);

				if (call + 1u < JIT_HOT_LOOP_ADMIT_CALLS &&
				    selected != main_code)
					return false;
				block = (jit_block_fn)(uintptr_t)selected;
			}
			if (jit_hot_code_arena != NULL &&
			    (jit_hot_entries[0].location == 0u ||
			     selected == main_code))
				return false;
		}
		regs[2] = 5u;
		runtime.loop_iterations = JIT_SELF_TEST_LOOP_ITERATIONS;
		bailout = rv32_native_jit_enter(block, &runtime);
		if (bailout != 0u || runtime.result.trap != 0u ||
		    runtime.result.executed != 25u ||
		    runtime.result.next_pc != 0x4014u ||
		    regs[1] != 15u || regs[2] != 0u)
			return false;

		memset(regs, 0, sizeof(regs));
		memset(&runtime.result, 0, sizeof(runtime.result));
		regs[2] = 5u;
		runtime.loop_iterations = 2u;
		bailout = rv32_native_jit_enter(block, &runtime);
		if (bailout != 0u || runtime.result.trap != 0u ||
		    runtime.result.executed != 10u ||
		    runtime.result.next_pc != 0x4000u ||
		    regs[1] != 6u || regs[2] != 3u)
			return false;
	}

	/* A forward condition followed by JAL x0 is GCC's other common loop form.
	 * Verify both its early side exit and its bounded back-edge continuation. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.loop_iterations = JIT_SELF_TEST_LOOP_ITERATIONS,
		};
		jit_block_fn block;

		if (!jit_compile(0u, (const uint8_t *)jump_loop_program, 0u,
				 0x4800u, 4u) || !jit_entries[0].loops_to_start)
			return false;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		regs[2] = 3u;
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 11u ||
		    runtime.result.next_pc != 0x4810u ||
		    regs[1] != 3u || regs[2] != 0u)
			return false;

		memset(regs, 0, sizeof(regs));
		memset(&runtime.result, 0, sizeof(runtime.result));
		regs[2] = 5u;
		runtime.loop_iterations = 2u;
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 8u ||
		    runtime.result.next_pc != 0x4800u ||
		    regs[1] != 2u || regs[2] != 3u)
			return false;
	}

	/* Follow an address-independent same-page JAL x0 target as one native
	 * trace.  The skipped lexical word must never execute, and the terminal
	 * conditional retains its architectural fall-through PC. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
		};
		jit_block_fn block;

		if (!jit_compile(0u, (const uint8_t *)jump_trace_program, 0u,
				 0xd000u, 6u) ||
		    jit_entries[0].guest_instruction_count != 5u)
			return false;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 5u ||
		    runtime.result.next_pc != 0xd018u ||
		    regs[1] != 3u || regs[2] != 3u)
			return false;
	}

	/* A conditional edge may enter the target appended after a direct jump.
	 * Its native label must skip the lexical-path instruction checkpoint so
	 * both paths retain exact architectural counts. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
		};
		jit_block_fn block;

		if (!jit_compile(0u, (const uint8_t *)jump_branch_trace_program,
				 0u, 0xe000u, 6u) ||
		    jit_entries[0].guest_instruction_count != 5u)
			return false;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 3u ||
		    runtime.result.next_pc != 0xe018u || regs[1] != 2u)
			return false;

		memset(regs, 0, sizeof(regs));
		memset(&runtime.result, 0, sizeof(runtime.result));
		regs[6] = 1u;
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 5u ||
		    runtime.result.next_pc != 0xe018u || regs[1] != 3u)
			return false;
	}

	/* Memory exits inside an appended target report that target's real guest
	 * PC while counting the direct jump and prior trace instructions once. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.opaque = &memory,
			.load = jit_test_load,
			.store = jit_test_store,
		};
		jit_block_fn block;
		uint32_t bailout;

		for (uint32_t i = 0;
		     i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++)
			runtime.read_pages[i].virtual_page = UINT32_MAX;
		memory.address = 0x2000u;
		memory.value = 0x76543210u;
		regs[4] = memory.address;
		if (!jit_compile(0u, (const uint8_t *)jump_memory_trace_program,
				 0u, 0xf000u, 6u) ||
		    jit_entries[0].guest_instruction_count != 5u)
			return false;
		runtime.sites = jit_slot_sites(0);
		runtime.site_count = jit_entries[0].memory_site_count;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		bailout = rv32_native_jit_enter(block, &runtime);
		if (bailout == 0u)
			return false;
		jit_resolve_memory_bailout(&runtime, bailout);
		if (runtime.result.trap != 0u ||
		    runtime.result.executed != 3u ||
		    runtime.result.next_pc != 0xf010u ||
		    regs[1] != 1u || regs[2] != memory.value || regs[3] != 0u ||
		    runtime.memory_calls != 1u)
			return false;

		memset(regs, 0, sizeof(regs));
		memset(&runtime.result, 0, sizeof(runtime.result));
		regs[4] = memory.address;
		runtime.memory_calls = 0u;
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 5u ||
		    runtime.result.next_pc != 0xf018u ||
		    regs[1] != 1u || regs[2] != memory.value || regs[3] != 3u ||
		    runtime.memory_calls != 0u)
			return false;
	}

	/* A direct same-page call and its canonical return stay in one bounded
	 * trace.  The link value and non-lexical instruction order must retain the
	 * same architectural state and dynamic instruction count as RV32 code. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
		};
		jit_block_fn block;

		if (!jit_compile(0u, (const uint8_t *)call_trace_program, 0u,
				 0x11000u, 8u) ||
		    jit_entries[0].guest_instruction_count != 6u)
			return false;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 6u ||
		    runtime.result.next_pc != 0x11020u ||
		    regs[1] != 0x11008u || regs[2] != 1u ||
		    regs[3] != 3u || regs[4] != 4u)
			return false;
	}

	/* A callee may legally modify the link register.  The guarded return must
	 * then commit through the ordinary side exit without executing the
	 * statically predicted continuation. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
		};
		jit_block_fn block;

		if (!jit_compile(0u, (const uint8_t *)call_guard_program, 0u,
				 0x12000u, 5u) ||
		    jit_entries[0].guest_instruction_count != 5u)
			return false;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 3u ||
		    runtime.result.next_pc != 0x12008u ||
		    regs[1] != 0x12008u || regs[2] != 0u)
			return false;
	}

	/* A translated memory group at the loop header may be shared within one
	 * iteration only when its guest base advances on the back edge.  Exercise
	 * the common unrolled clear/copy shape so stale host addresses cannot make
	 * every iteration overwrite the first words. */
	{
		uint32_t regs[32] = { 0 };
		uint32_t words[8] = { 0 };
		const uint32_t virtual_base = 0x3000u;
		struct rv32_native_jit_write_page *page;
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.loop_iterations = JIT_SELF_TEST_LOOP_ITERATIONS,
		};
		jit_block_fn block;

		for (uint32_t i = 0;
		     i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++) {
			runtime.write_pages[i].virtual_page = UINT32_MAX;
			runtime.write_pages[i].linear_page_start = UINT32_MAX;
		}
		page = &runtime.write_pages[(virtual_base >> 12) &
			(RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u)];
		page->host_bias = (uint32_t)(uintptr_t)words - virtual_base;
		page->virtual_page = virtual_base >> 12;
		regs[2] = 4u;
		regs[3] = 0x55aa33ccu;
		regs[4] = virtual_base;
		if (!jit_compile(0u, (const uint8_t *)moving_store_loop_program,
				 0u, 0xc000u, 7u) ||
		    !jit_entries[0].loops_to_start)
			return false;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 27u ||
		    runtime.result.next_pc != 0xc01cu ||
		    regs[2] != 0u || regs[4] != virtual_base + sizeof(words) ||
		    regs[5] != 4u)
			return false;
		for (uint32_t i = 0; i < 8u; i++) {
			if (words[i] != regs[3])
				return false;
		}
	}

	/* Exercise a normal loop larger than the old eight-instruction window,
	 * including cached loads and stores. This catches code-size, back-edge and
	 * register-allocation regressions before Linux starts. */
	{
		uint32_t regs[32] = { 0 };
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.opaque = &memory,
			.load = jit_test_load,
			.store = jit_test_store,
			.loop_iterations = JIT_SELF_TEST_LOOP_ITERATIONS,
		};
		jit_block_fn block;
		uint32_t bailout;

		for (uint32_t i = 0;
		     i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++) {
			runtime.read_pages[i].virtual_page = UINT32_MAX;
			runtime.write_pages[i].virtual_page = UINT32_MAX;
			runtime.write_pages[i].linear_page_start = UINT32_MAX;
		}
		regs[2] = 3u;
		regs[4] = memory.address;
		regs[9] = 3u;
		memory.value = 1u;
		/* Seed both access directions once, as the cold resolver does at
		 * runtime, so this test exercises repeated native page-cache hits. */
		if (jit_test_load(&runtime, memory.address, 2u) != 0u ||
		    jit_test_store(&runtime, memory.address, memory.value, 2u) != 0u)
			return false;
		runtime.memory_calls = 0u;
		if (!jit_compile(0u, (const uint8_t *)long_loop_program, 0u,
				 0x5000u, 13u) || !jit_entries[0].loops_to_start ||
		    jit_entries[0].guest_instruction_count != 13u)
			return false;
		runtime.sites = jit_slot_sites(0);
		runtime.site_count = jit_entries[0].memory_site_count;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		bailout = rv32_native_jit_enter(block, &runtime);
		if (bailout != 0u || runtime.result.trap != 0u ||
		    runtime.result.executed != 39u ||
		    runtime.result.next_pc != 0x5034u ||
		    runtime.memory_calls != 0u || memory.value != 7u ||
		    regs[1] != 3u || regs[2] != 0u || regs[3] != 7u ||
		    regs[5] != 0u || regs[6] != 9u || regs[7] != 12u ||
		    regs[8] != 3u || regs[9] != 0u || regs[0] != 0u)
			return false;
	}

	/* Four independent read streams exercise every retained loop translation.
	 * The shared checked page is seeded once; all iterations must then stay in
	 * generated code without a callback or corrupting reduced guest mappings. */
	{
		uint32_t regs[32] = { 0 };
		jit_block_fn block;

		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.opaque = &memory,
			.load = jit_test_load,
			.store = jit_test_store,
			.loop_iterations = 3u,
		};
		for (uint32_t i = 0;
		     i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++)
			runtime.read_pages[i].virtual_page = UINT32_MAX;
		memory.address = 0x2000u;
		memory.value = 0x12345678u;
		regs[2] = 3u;
		regs[4] = memory.address;
		regs[7] = memory.address;
		regs[10] = memory.address;
		regs[13] = memory.address;
		if (jit_test_load(&runtime, memory.address, 2u) != 0u)
			return false;
		runtime.memory_calls = 0u;
		if (!jit_compile(0u, (const uint8_t *)four_read_loop_program,
				 0u, 0x12800u, 10u) ||
		    !jit_entries[0].loops_to_start)
			return false;
		runtime.sites = jit_slot_sites(0);
		runtime.site_count = jit_entries[0].memory_site_count;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u || runtime.result.executed != 30u ||
		    runtime.result.next_pc != 0x12828u ||
		    runtime.memory_calls != 0u || regs[1] != 3u || regs[2] != 0u ||
		    regs[3] != memory.value || regs[5] != memory.value ||
		    regs[8] != memory.value || regs[11] != memory.value ||
		    regs[6] != memory.value * 4u)
			return false;
	}

	/* The second repeated-load translation cache occupies s10/s11.  Link into
	 * such a loop with a nonzero cumulative count and prove that its local cache
	 * state cannot corrupt the register-resident chain budget.  This is the
	 * smallest form of the conflict that previously let Linux init run with an
	 * incorrect instruction count and eventually SIGSEGV. */
	{
		uint32_t dual_read_chain_program[10] = {
			rv_i(0x13u, 1u, 0u, 1u, 1),  /* source: addi x1, x1, 1 */
			rv_i(0x13u, 1u, 0u, 1u, 1),  /* addi x1, x1, 1 */
			rv_i(0x13u, 1u, 0u, 1u, 1),  /* addi x1, x1, 1 */
			rv_j(0u, 4),                  /* link to loop target */
			rv_i(0x03u, 3u, 2u, 4u, 0),  /* target: lw x3, 0(x4) */
			rv_i(0x13u, 1u, 0u, 1u, 1),  /* addi x1, x1, 1 */
			rv_i(0x03u, 5u, 2u, 7u, 0),  /* lw x5, 0(x7) */
			rv_r(0x33u, 6u, 0u, 3u, 5u, 0u), /* add x6, x3, x5 */
			rv_i(0x13u, 2u, 0u, 2u, -1), /* addi x2, x2, -1 */
			rv_b(1u, 2u, 0u, -20),       /* bne x2, x0, target */
		};
		const uint32_t source_pc = 0x13000u;
		const uint32_t target_pc = source_pc + 16u;
		uint32_t regs[32] = { 0 };
		jit_block_fn block;
		uint32_t packed_target;

		memset(jit_entries, 0, sizeof(jit_entries));
		memset(jit_hot_entries, 0, sizeof(jit_hot_entries));
		jit_clear_all_links();
		jit_reset_code_heads();
		runtime = (struct rv32_native_jit_runtime) {
			.regs = regs,
			.opaque = &memory,
			.load = jit_test_load,
			.store = jit_test_store,
			.chain_instruction_limit = 22u,
		};
		for (uint32_t i = 0;
		     i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++)
			runtime.read_pages[i].virtual_page = UINT32_MAX;
		memory.address = 0x2000u;
		memory.value = 0x12345678u;
		regs[2] = 3u;
		regs[4] = memory.address;
		regs[7] = memory.address;
		if (!jit_compile(0u, (const uint8_t *)dual_read_chain_program,
				 0u, source_pc, 4u) ||
		    !jit_compile(1u, (const uint8_t *)dual_read_chain_program,
				 16u, target_pc, 6u) ||
		    !jit_entries[1].loops_to_start)
			return false;
		packed_target = jit_pack_link_target(1u, &jit_entries[1]);
		if (packed_target == 0u)
			return false;
		jit_links[0].expected_next_pc = target_pc;
		jit_links[0].packed_target = packed_target;
		jit_link_targets[1] = 1u;
		if (jit_test_load(&runtime, memory.address, 2u) != 0u)
			return false;
		runtime.memory_calls = 0u;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		if (rv32_native_jit_enter(block, &runtime) != 0u ||
		    runtime.result.trap != 0u ||
		    runtime.result.executed != 18u ||
		    runtime.result.next_pc != target_pc + 24u ||
		    runtime.result.native_slot != 1u ||
		    runtime.chain_executed != 22u ||
		    runtime.chain_links !=
			    (RV32_NATIVE_JIT_RUNTIME_COUNTERS ? 1u : 0u) ||
		    regs[1] != 6u || regs[2] != 0u ||
		    regs[3] != memory.value || regs[5] != memory.value ||
		    regs[6] != memory.value + memory.value ||
		    runtime.memory_calls != 0u)
			return false;
	}

#undef runtime

	/* The public dispatcher must let a native back-edge consume the complete
	 * remaining MiniRV32 batch budget.  An obsolete 32-iteration inner cap made
	 * long painter and memory loops pay the native/C transition repeatedly. */
	{
		uint32_t regs[32] = { 0 };
		const struct rv32_native_jit_result *result;
		const uint32_t iterations = 300u;

		memset(jit_entries, 0, sizeof(jit_entries));
		memset(jit_warm, 0, JIT_WARM_BYTES);
		memset(jit_replace_way, 0, sizeof(jit_replace_way));
		rv32_native_jit_bind(regs, (uint8_t *)loop_program, NULL,
				     NULL, NULL);
		for (uint32_t hit = 0; hit < JIT_LOOP_HOT_THRESHOLD; hit++) {
			if (rv32_native_jit_try(0u, 0x6000u,
					iterations * 5u, 0u) != NULL)
				return false;
		}
		regs[2] = iterations;
		result = rv32_native_jit_try(0u, 0x6000u,
					     iterations * 5u, 0u);
		if (result == NULL || result->trap != 0u ||
		    result->executed != iterations * 5u ||
		    result->next_pc != 0x6014u || regs[2] != 0u ||
		    regs[1] != iterations * 3u)
			return false;
	}

	/* A checked memory-cache fill resumes after the completed access, but that
	 * continuation is inside an existing region rather than a hot control-flow
	 * head.  Only a later full-region completion may seed another compilation. */
	{
		uint32_t regs[32] = { 0 };
		const struct rv32_native_jit_result *result;

		memset(jit_entries, 0, sizeof(jit_entries));
		memset(jit_warm, 0, JIT_WARM_BYTES);
		memset(jit_replace_way, 0, sizeof(jit_replace_way));
		memory.address = 0x2000u;
		memory.value = 0x12345678u;
		regs[4] = memory.address;
		rv32_native_jit_bind(regs, (uint8_t *)grouped_memory_program,
				     &memory, jit_test_load, jit_test_store);
		for (uint32_t hit = 0; hit < JIT_GENERAL_HOT_THRESHOLD; hit++) {
			if (rv32_native_jit_try(0u, 0x7000u, 5u, 0u) != NULL)
				return false;
		}

		result = rv32_native_jit_try(0u, 0x7000u, 5u, 0u);
		if (result == NULL || result->trap != 0u ||
		    result->executed != 1u || result->next_pc != 0x7004u ||
		    result->native_successor != 0u)
			return false;
		result = rv32_native_jit_try(0u, 0x7000u, 5u, 0u);
		if (result == NULL || result->trap != 0u ||
		    result->executed != 3u || result->next_pc != 0x700cu ||
		    result->native_successor != 0u)
			return false;
		result = rv32_native_jit_try(0u, 0x7000u, 5u, 0u);
		if (result == NULL || result->trap != 0u ||
		    result->executed != 5u || result->next_pc != 0x7018u ||
		    result->native_successor != 1u)
			return false;
	}

	/* Two independently compiled regions in one translated page should consume
	 * one shared instruction budget without returning through MiniRV32 between
	 * them.  Their physical and virtual deltas deliberately match.  The target
	 * also contains a load so its native-linked cold bailout can prove that C
	 * recovers the final region's memory-site directory. */
	{
		uint32_t chain_program[8] = {
			rv_i(0x13u, 1u, 0u, 1u, 1),
			rv_i(0x13u, 2u, 0u, 2u, 2),
			rv_i(0x13u, 3u, 0u, 3u, 3),
			rv_j(0u, 4),
			rv_i(0x03u, 6u, 2u, 4u, 0),
			rv_i(0x13u, 2u, 0u, 2u, 5),
			rv_i(0x13u, 3u, 0u, 3u, 6),
			rv_b(0u, 0u, 0u, 8),
		};
		const uint32_t pcs[2] = { 0x9000u, 0x9010u };
		const uint32_t offsets[2] = { 0u, 16u };
		uint32_t regs[32] = { 0 };
		const struct rv32_native_jit_result *result;

		memset(jit_entries, 0, sizeof(jit_entries));
		memset(jit_warm, 0, JIT_WARM_BYTES);
		memset(jit_replace_way, 0, sizeof(jit_replace_way));
		memory.address = 0x2000u;
		memory.value = 0x89abcdefu;
		regs[4] = memory.address;
		rv32_native_jit_bind(regs, (uint8_t *)chain_program, &memory,
				     jit_test_load, jit_test_store);
		for (uint32_t block_index = 0; block_index < 2u;
		     block_index++) {
			for (uint32_t hit = 0; hit < JIT_GENERAL_HOT_THRESHOLD; hit++) {
				if (rv32_native_jit_try(offsets[block_index],
						       pcs[block_index], 4u, 0u) != NULL)
					return false;
			}
		}
		if (jit_test_load(jit_rt, memory.address, 2u) != 0u)
			return false;
		jit_rt->memory_calls = 0u;
		result = rv32_native_jit_try(offsets[0], pcs[0], 8u, 0u);
		if (result == NULL || result->trap != 0u ||
		    result->executed != 8u || result->next_pc != 0x9024u ||
		    result->native_successor != 1u || regs[1] != 1u ||
		    regs[2] != 7u || regs[3] != 9u ||
		    regs[6] != memory.value || jit_rt->memory_calls != 0u)
			return false;

		/* Repeatedly observe the same edge, then prove that the generated tail
		 * traverses it without a C lookup.  Diagnostic builds also verify the
		 * transition count; counterless production verifies architectural state. */
		for (uint32_t observation = 1u;
		     observation < JIT_LINK_CONFIRMATIONS; observation++) {
			memset(regs, 0, sizeof(regs));
			regs[4] = memory.address;
			result = rv32_native_jit_try(
				offsets[0], pcs[0], 8u, 0u);
			if (result == NULL || result->trap != 0u ||
			    result->executed != 8u || regs[1] != 1u ||
			    regs[2] != 7u || regs[3] != 9u ||
			    regs[6] != memory.value)
				return false;
		}
		memset(regs, 0, sizeof(regs));
		regs[4] = memory.address;
		result = rv32_native_jit_try(offsets[0], pcs[0], 8u, 0u);
		if (result == NULL || result->trap != 0u ||
		    result->executed != 8u || result->next_pc != 0x9024u ||
		    jit_rt->chain_links !=
			    (RV32_NATIVE_JIT_RUNTIME_COUNTERS ? 1u : 0u) ||
		    regs[1] != 1u ||
		    regs[2] != 7u || regs[3] != 9u ||
		    regs[6] != memory.value)
			return false;

		/* Preserve the published code link while forcing its target's load to
		 * miss.  The dispatcher must resolve target slot 1's descriptor rather
		 * than the memory-less source slot and count both regions exactly once. */
		for (uint32_t index = 0;
		     index < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; index++)
			jit_rt->read_pages[index].virtual_page = UINT32_MAX;
		memset(regs, 0, sizeof(regs));
		regs[4] = memory.address;
		jit_rt->memory_calls = 0u;
		result = rv32_native_jit_try(offsets[0], pcs[0], 8u, 0u);
		if (result == NULL || result->trap != 0u ||
		    result->executed != 5u || result->next_pc != 0x9014u ||
		    jit_rt->chain_links !=
			    (RV32_NATIVE_JIT_RUNTIME_COUNTERS ? 1u : 0u) ||
		    regs[1] != 1u ||
		    regs[2] != 2u || regs[3] != 3u ||
		    regs[6] != memory.value || jit_rt->memory_calls != 1u)
			return false;

		/* A translation-context boundary must remove the shortcut while leaving
		 * the same-page generated regions themselves reusable. */
		rv32_native_jit_memory_cache_flush();
		memset(regs, 0, sizeof(regs));
		regs[4] = memory.address;
		jit_rt->memory_calls = 0u;
		result = rv32_native_jit_try(offsets[0], pcs[0], 8u, 0u);
		if (result == NULL || result->trap != 0u ||
		    result->executed != 5u || result->next_pc != 0x9014u ||
		    jit_rt->chain_links != 0u || regs[1] != 1u ||
		    regs[2] != 2u || regs[3] != 3u ||
		    regs[6] != memory.value || jit_rt->memory_calls != 1u)
			return false;
		memset(regs, 0, sizeof(regs));
		regs[4] = memory.address;
		jit_rt->memory_calls = 0u;
		result = rv32_native_jit_try(offsets[0], pcs[0], 8u, 0u);
		if (result == NULL || result->trap != 0u ||
		    result->executed != 8u || result->next_pc != 0x9024u ||
		    jit_rt->chain_links != 0u || regs[1] != 1u ||
		    regs[2] != 7u || regs[3] != 9u ||
		    regs[6] != memory.value || jit_rt->memory_calls != 0u)
			return false;
	}

	/* Cross-page chaining and trace collection may use only page mappings that
	 * previously passed the normal execute translator. Keep the two physical
	 * blocks deliberately noncontiguous with their virtual pages so accidental
	 * virtual-to-physical arithmetic cannot satisfy this test. */
	{
		uint32_t cross_page_program[8] = {
			rv_i(0x13u, 1u, 0u, 1u, 1),
			rv_i(0x13u, 2u, 0u, 2u, 2),
			rv_i(0x13u, 3u, 0u, 3u, 3),
			rv_j(0u, 0xff4), /* 0x900c -> 0xa000 */
			rv_i(0x13u, 1u, 0u, 1u, 4),
			rv_i(0x13u, 2u, 0u, 2u, 5),
			rv_i(0x13u, 3u, 0u, 3u, 6),
			rv_b(0u, 0u, 0u, 8),
		};
		const uint32_t pcs[2] = { 0x9000u, 0xa000u };
		const uint32_t offsets[2] = { 0u, 16u };
		uint32_t regs[32] = { 0 };
		const struct rv32_native_jit_result *result;
		jit_block_fn block;
		bool compiled;
		uint32_t resolved;

		memset(jit_entries, 0, sizeof(jit_entries));
		memset(jit_warm, 0, JIT_WARM_BYTES);
		memset(jit_replace_way, 0, sizeof(jit_replace_way));
		rv32_native_jit_bind(regs, (uint8_t *)cross_page_program, NULL,
				     NULL, NULL);
		/* Compile the source before the target mapping exists so these remain
		 * independent blocks for the public-dispatch chaining check. */
		for (uint32_t hit = 0; hit < JIT_GENERAL_HOT_THRESHOLD; hit++) {
			if (rv32_native_jit_try(offsets[0], pcs[0], 4u, 0u) != NULL)
				return false;
		}
		for (uint32_t hit = 0; hit < JIT_GENERAL_HOT_THRESHOLD; hit++) {
			if (rv32_native_jit_try(offsets[1], pcs[1], 4u, 0u) != NULL)
				return false;
		}
		result = rv32_native_jit_try(offsets[0], pcs[0], 8u, 0u);
		if (result == NULL || result->trap != 0u ||
		    result->executed != 8u || result->next_pc != 0xa014u ||
		    result->native_successor != 1u || regs[1] != 5u ||
		    regs[2] != 7u || regs[3] != 9u)
			return false;

		/* With both validated mappings resident, the same control flow can be
		 * collected into one bounded region. A context flush must discard that
		 * multi-page code and both mapping assumptions. */
		memset(jit_entries, 0, sizeof(jit_entries));
		memset(regs, 0, sizeof(regs));
		jit_cross_page_trace_self_test = true;
		compiled = jit_compile(0u, (const uint8_t *)cross_page_program,
				       0u, pcs[0], 8u);
		jit_cross_page_trace_self_test = false;
		if (!compiled ||
		    jit_entries[0].guest_instruction_count != 8u ||
		    !jit_entries[0].spans_pages)
			return false;
		block = (jit_block_fn)(uintptr_t)jit_entry_code(0u);
		if (rv32_native_jit_enter(block, jit_rt) != 0u ||
		    jit_rt->result.trap != 0u ||
		    jit_rt->result.executed != 8u ||
		    jit_rt->result.next_pc != 0xa014u || regs[1] != 5u ||
		    regs[2] != 7u || regs[3] != 9u)
			return false;
		rv32_native_jit_memory_cache_flush();
		if (jit_entries[0].state != JIT_ENTRY_EMPTY ||
		    jit_resolve_exec_offset(pcs[0], &resolved) ||
		    jit_resolve_exec_offset(pcs[1], &resolved))
			return false;

		/* A page-scoped guest SFENCE.VMA must discard only the colliding data
		 * and execute mappings. An unrelated cached page remains usable. */
		{
			const uint32_t retained_pc = 0xb000u;
			uint32_t target_index =
				(pcs[1] >> 12) &
				(RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u);
			uint32_t retained_index =
				(retained_pc >> 12) &
				(RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u);

			jit_rt->read_pages[target_index].virtual_page =
				pcs[1] >> 12;
			jit_rt->write_pages[target_index].virtual_page =
				pcs[1] >> 12;
			jit_rt->read_pages[retained_index].virtual_page =
				retained_pc >> 12;
			jit_rt->write_pages[retained_index].virtual_page =
				retained_pc >> 12;
			jit_cache_exec_page(offsets[1], pcs[1]);
			jit_cache_exec_page(32u, retained_pc);
			rv32_native_jit_memory_cache_flush_page(pcs[1] + 0x123u);
			if (jit_rt->read_pages[target_index].virtual_page !=
				    UINT32_MAX ||
			    jit_rt->write_pages[target_index].virtual_page !=
				    UINT32_MAX ||
			    jit_resolve_exec_offset(pcs[1], &resolved) ||
			    jit_rt->read_pages[retained_index].virtual_page !=
				    retained_pc >> 12 ||
			    jit_rt->write_pages[retained_index].virtual_page !=
				    retained_pc >> 12 ||
			    !jit_resolve_exec_offset(retained_pc, &resolved) ||
			    resolved != 32u)
				return false;
		}
	}

	/* Exercise the public hot-dispatch path with one block more than a set
	 * has ways, all mapping to one cache set.  The first JIT_CACHE_WAYS must
	 * coexist, and touching the first must make the second (not the first)
	 * the victim for the last. */
	{
		uint32_t dispatch_program[4] = {
			rv_i(0x13u, 1u, 0u, 1u, 1), /* addi x1, x1, 1 */
			rv_i(0x13u, 2u, 0u, 2u, 2), /* addi x2, x2, 2 */
			rv_i(0x13u, 3u, 0u, 3u, 3), /* addi x3, x3, 3 */
			rv_b(0u, 0u, 0u, 8),        /* beq x0, x0, +8 */
		};
		uint32_t pcs[JIT_CACHE_WAYS + 1u] = { 0x8000u };
		uint32_t regs[32] = { 0 };
		uint32_t set = jit_hash(0u, pcs[0], JIT_CACHE_SETS - 1u);
		uint32_t pc_count = 1u;
		const struct rv32_native_jit_result *result;

		/* Derive collisions from the configured directory size instead of
		 * baking in addresses that are valid only for one hash mask. */
		for (uint32_t page = 9u;
		     page < 0x10000u && pc_count < JIT_CACHE_WAYS + 1u; page++) {
			uint32_t candidate_pc = page << 12;

			if (jit_hash(0u, candidate_pc,
				     JIT_CACHE_SETS - 1u) == set)
				pcs[pc_count++] = candidate_pc;
		}
		if (pc_count != JIT_CACHE_WAYS + 1u)
			return false;
		memset(jit_entries, 0, sizeof(jit_entries));
		memset(jit_warm, 0, JIT_WARM_BYTES);
		memset(jit_replace_way, 0, sizeof(jit_replace_way));
		rv32_native_jit_bind(regs, (uint8_t *)dispatch_program, NULL,
				     NULL, NULL);

		for (uint32_t block = 0; block < JIT_CACHE_WAYS; block++) {
			for (uint32_t hit = 0; hit < JIT_GENERAL_HOT_THRESHOLD; hit++) {
				if (rv32_native_jit_try(0u, pcs[block], 4u, 0u) != NULL)
					return false;
			}
			result = rv32_native_jit_try(0u, pcs[block], 4u, 0u);
			if (result == NULL || result->trap != 0u ||
			    result->executed != 4u ||
			    result->next_pc != pcs[block] + 20u)
				return false;
		}

		/* Make block zero most-recently used, then populate one more
		 * collision. */
		result = rv32_native_jit_try(0u, pcs[0], 4u, 0u);
		if (result == NULL || result->executed != 4u)
			return false;
		for (uint32_t hit = 0; hit < JIT_GENERAL_HOT_THRESHOLD; hit++) {
			if (rv32_native_jit_try(0u, pcs[JIT_CACHE_WAYS], 4u, 0u) !=
			    NULL)
				return false;
		}

		{
			bool found_first = false;
			bool found_second = false;
			bool found_third = false;

			for (uint32_t way = 0; way < JIT_CACHE_WAYS; way++) {
				const struct jit_cache_entry *entry =
					&jit_entries[set * JIT_CACHE_WAYS + way];

				found_first |= entry->state == JIT_ENTRY_READY &&
					entry->guest_pc == pcs[0];
				found_second |= entry->state == JIT_ENTRY_READY &&
					entry->guest_pc == pcs[1];
				found_third |= entry->state == JIT_ENTRY_READY &&
					entry->guest_pc == pcs[JIT_CACHE_WAYS];
			}
			if (!found_first || found_second || !found_third)
				return false;
		}
	}

	memset(jit_entries, 0, sizeof(jit_entries));
	memset(jit_warm, 0, JIT_WARM_BYTES);
	memset(jit_replace_way, 0, sizeof(jit_replace_way));
	memset(jit_hot_entries, 0, sizeof(jit_hot_entries));
	memset(jit_runtimes, 0, sizeof(jit_runtimes));
	jit_rt = &jit_runtimes[JIT_BANK_USER];
	jit_set_exec_privilege(0u);
	memset(&jit_stats, 0, sizeof(jit_stats));
	jit_invalidate_translations();
	jit_reset_code_heads();
	return true;
}

#ifdef ESP_PLATFORM
/* Publishes and runs "li a0, 1; ret" with esp_cache_msync, then rewrites it
 * to "li a0, 2" at the same address with the L1-only publication.  Stale
 * code still returns 1, so a cache path that bypasses L2 is caught without
 * executing garbage. */
static bool jit_check_light_publish(uint32_t *code)
{
	typedef uint32_t (*probe_fn)(void);
	bool ok;

	jit_light_publish = false;
	memset(code, 0, JIT_CODE_ALIGNMENT);
	code[0] = 0x00100513u;	/* li a0, 1 */
	code[1] = 0x00008067u;	/* ret */
	if (!jit_sync_code(code, JIT_CODE_ALIGNMENT) ||
	    ((probe_fn)(uintptr_t)code)() != 1u)
		return false;
	code[0] = 0x00200513u;	/* li a0, 2 */
	jit_light_publish = true;
	ok = jit_sync_code(code, JIT_CODE_ALIGNMENT) &&
		((probe_fn)(uintptr_t)code)() == 2u;
	jit_light_publish = false;
	memset(code, 0, JIT_CODE_ALIGNMENT);
	(void)jit_sync_code(code, JIT_CODE_ALIGNMENT);
	return ok;
}
#endif

bool rv32_native_jit_init(void)
{
	size_t arena_bytes = JIT_CODE_ARENA_MIN_BYTES;
	size_t hot_arena_bytes = 0u;

	if (jit_code_arena != NULL)
		return true;

#ifdef ESP_PLATFORM
	const unsigned int arena_caps =
		MALLOC_CAP_EXEC | MALLOC_CAP_INTERNAL | MALLOC_CAP_32BIT;
	size_t largest;
	size_t free_bytes;
	void *psram_code = NULL;
	size_t psram_code_bytes = 0u;
	void *psram_metadata = NULL;
	size_t psram_metadata_bytes = 0u;
	size_t candidate_bytes =
		JIT_CACHE_SLOTS * sizeof(*jit_link_candidates);
	bool main_code_external = false;

	if (psram_get_host_jit_metadata_region(
		    &psram_metadata, &psram_metadata_bytes) &&
	    psram_metadata_bytes >= candidate_bytes) {
		jit_link_candidates = psram_metadata;
		memset(jit_link_candidates, 0, candidate_bytes);
	} else {
		jit_link_candidates = heap_caps_calloc(
			JIT_CACHE_SLOTS, sizeof(*jit_link_candidates),
			MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	}
	jit_warm = jit_warm_storage;
	memset(jit_warm, 0, JIT_WARM_BYTES);
	if (jit_warm == NULL || jit_link_candidates == NULL)
		return false;

	/* P4's PSRAM MMU window is available on both instruction and data buses.
	 * Prefer the region reserved below Linux's first usable physical page: it
	 * is outside the guest allocator and keeps internal SRAM free for device
	 * workers.  The boot-time self-test exercises actual execution from it. */
	if (psram_get_host_jit_region(&psram_code, &psram_code_bytes) &&
	    psram_code_bytes >= JIT_CODE_ARENA_MIN_BYTES &&
	    psram_code_bytes <= JIT_CODE_ARENA_MAX_BYTES &&
	    ((uintptr_t)psram_code & (JIT_CODE_ALIGNMENT - 1u)) == 0u &&
	    (psram_code_bytes & (JIT_CODE_ALIGNMENT - 1u)) == 0u) {
		jit_code_arena = psram_code;
		arena_bytes = psram_code_bytes;
		main_code_external = true;
		memset(jit_code_arena, 0, arena_bytes);
	}

	if (jit_code_arena == NULL) {
		/* Retain the proven internal-SRAM path as a portable fallback, and
		 * leave a fixed reserve for runtime queues and DMA housekeeping. */
		largest = heap_caps_get_largest_free_block(arena_caps);
		free_bytes = heap_caps_get_free_size(arena_caps);
		for (size_t candidate = JIT_INTERNAL_CODE_ARENA_MAX_BYTES;;
		     candidate -= 16u * 1024u) {
			if (largest >= candidate &&
			    free_bytes >= candidate + JIT_INTERNAL_RESERVE_BYTES) {
				jit_code_arena = heap_caps_aligned_calloc(
					JIT_CODE_ALIGNMENT, 1u, candidate, arena_caps);
				if (jit_code_arena != NULL) {
					arena_bytes = candidate;
					break;
				}
			}
			if (candidate == JIT_CODE_ARENA_MIN_BYTES)
				break;
		}
	}
	/* A fragmented heap can make the conservative size probe disagree with
	 * the aligned allocator.  Retain the previously proven 64 KiB fallback. */
	if (jit_code_arena == NULL) {
		arena_bytes = JIT_CODE_ARENA_MIN_BYTES;
		jit_code_arena = heap_caps_aligned_calloc(
			JIT_CODE_ALIGNMENT, 1u, arena_bytes, arena_caps);
	}

	/* Once every device bridge has allocated its DMA queues, use only the
	 * surplus executable internal heap for a small hot-code tier.  The fixed
	 * reserve keeps later network, display, USB and block-I/O housekeeping from
	 * competing with JIT code.  Failure simply retains the PSRAM/L2 path. */
	if (jit_code_arena != NULL && main_code_external) {
		largest = heap_caps_get_largest_free_block(arena_caps);
		free_bytes = heap_caps_get_free_size(arena_caps);
		for (size_t candidate = JIT_HOT_CODE_ARENA_MAX_BYTES;;
		     candidate -= JIT_HOT_CODE_ARENA_STEP_BYTES) {
			if (largest >= candidate &&
			    free_bytes >= candidate + JIT_INTERNAL_RESERVE_BYTES) {
				jit_hot_code_arena = heap_caps_aligned_calloc(
					JIT_CODE_ALIGNMENT, 1u, candidate, arena_caps);
				if (jit_hot_code_arena != NULL) {
					hot_arena_bytes = candidate;
					break;
				}
			}
			if (candidate == JIT_HOT_CODE_ARENA_MIN_BYTES)
				break;
		}
	}
#else
	jit_warm = jit_warm_storage;
	jit_link_candidates = jit_link_candidate_storage;
	jit_code_arena = heap_caps_aligned_calloc(
		JIT_CODE_ALIGNMENT, 1u, arena_bytes,
		MALLOC_CAP_EXEC | MALLOC_CAP_INTERNAL | MALLOC_CAP_32BIT);
	jit_hot_code_arena = jit_hot_code_storage;
	hot_arena_bytes = sizeof(jit_hot_code_storage);
#endif

	if (jit_code_arena == NULL)
		return false;
	jit_code_arena_units = (uint16_t)(arena_bytes / JIT_CODE_ALIGNMENT);
	jit_loop_code_units = (uint16_t)(jit_code_arena_units / 2u);
#ifdef ESP_PLATFORM
	jit_light_publish = jit_check_light_publish(jit_code_arena) &&
		(jit_hot_code_arena == NULL ||
		 jit_check_light_publish(jit_hot_code_arena));
#endif
#ifdef ESP_PLATFORM
	jit_unit_owner = heap_caps_calloc(jit_code_arena_units,
		sizeof(*jit_unit_owner), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
	jit_unit_owner = jit_unit_owner_storage;
#endif
	jit_hot_code_arena_units =
		(uint16_t)(hot_arena_bytes / JIT_CODE_ALIGNMENT);
	rv32_native_jit_dispatch_table = (struct rv32_native_jit_dispatch_table){
		.entries = jit_entries,
		.hot_entries = jit_hot_entries,
		.code_arena = jit_code_arena,
		.hot_code_arena = jit_hot_code_arena,
		.host_wake = JIT_HOST_WAKE_WORD,
		.loop_iterations_offset =
			offsetof(struct rv32_native_jit_runtime, loop_iterations),
		.set_mask = JIT_CACHE_SETS - 1u,
		.loop_wake_budget = JIT_LOOP_WAKE_BUDGET,
		.hot_loop_admit_calls = JIT_HOT_LOOP_ADMIT_CALLS,
		.hot_general_admit_calls = JIT_HOT_GENERAL_ADMIT_CALLS,
		.hot_loop_replace_calls = JIT_HOT_LOOP_REPLACE_CALLS,
		.hot_general_replace_calls = JIT_HOT_GENERAL_REPLACE_CALLS,
		.link_learning_budget = jit_link_learning_budget,
		.exec_pages = jit_exec_pages,
		.exec_privilege_tag = &jit_exec_privilege_tag,
		.jump_cache = jit_jump_cache[0],
		.links = jit_links,
		.link_targets = jit_link_targets,
		.jump_misses = jit_jump_misses,
	};
#ifdef ESP_PLATFORM
	jit_start_sampler();
#endif
	jit_clear_all_links();
	memset(jit_hot_entries, 0, sizeof(jit_hot_entries));
	jit_reset_code_heads();
	jit_invalidate_exec_pages();
	if (!jit_self_test_chain_results())
		rv32_native_jit_self_test_failure = 1u;
	else if (!jit_self_test_native_dispatch())
		rv32_native_jit_self_test_failure = 2u;
	else if (!jit_self_test_interior_loops())
		rv32_native_jit_self_test_failure = 3u;
	else if (!jit_self_test_kernel_linear())
		rv32_native_jit_self_test_failure = 5u;
	else if (!jit_self_test())
		rv32_native_jit_self_test_failure = 4u;
	return rv32_native_jit_self_test_failure == 0u;
}

uint32_t rv32_native_jit_fast_dispatch_count(void)
{
	return rv32_native_jit_dispatch_table.fast_dispatches;
}

void rv32_native_jit_memory_cache_flush_host_range(const void *host, size_t size)
{
	uintptr_t first = (uintptr_t)host;
	if (!host || !size || size - 1u > UINTPTR_MAX - first)
		return;
	uintptr_t last = first + size - 1u;
	/* Every bank may map the surface, the kernel ones through the linear map.
	 * Generated linear-map accesses, like the interpreter's shortcut, treat
	 * the whole window as RAM and never consult these entries; a revoked
	 * linear-map page was likewise re-learned as RAM at its next access. */
	for (uint32_t bank = 0; bank < JIT_MEMORY_BANKS; bank++) {
		struct rv32_native_jit_runtime *runtime = &jit_runtimes[bank];

		for (uint32_t i = 0; i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++) {
			struct rv32_native_jit_read_page *read = &runtime->read_pages[i];
			struct rv32_native_jit_write_page *write =
				&runtime->write_pages[i];
			uint32_t read_host = (read->virtual_page << 12) + read->host_bias;
			uint32_t write_host =
				(write->virtual_page << 12) + write->host_bias;
			/* 64-bit end so the top host page cannot wrap. */
			if (read->virtual_page != UINT32_MAX && read_host <= last &&
			    (uint64_t)read_host + 4095u >= first)
				read->virtual_page = UINT32_MAX;
			if (write->virtual_page != UINT32_MAX && write_host <= last &&
			    (uint64_t)write_host + 4095u >= first)
				write->virtual_page = UINT32_MAX;
		}
	}
}

static void jit_invalidate_memory_pages(struct rv32_native_jit_runtime *runtime)
{
	runtime->linear_limit = 0u;
	for (uint32_t i = 0; i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++) {
		runtime->read_pages[i].virtual_page = UINT32_MAX;
		runtime->write_pages[i].virtual_page = UINT32_MAX;
		runtime->write_pages[i].linear_page_start = UINT32_MAX;
		runtime->write_pages[i].linear_page_end = 0u;
	}
}

/* Drop everything derived from Sv32 translations, in every bank. */
static void jit_invalidate_translations(void)
{
	jit_stats.translation_flushes++;
	jit_invalidate_cross_page_entries();
	for (uint32_t bank = 0; bank < JIT_MEMORY_BANKS; bank++) {
		jit_runtimes[bank].memory_context = UINT32_MAX;
		jit_invalidate_memory_pages(&jit_runtimes[bank]);
	}
	jit_invalidate_exec_pages();
	jit_memory_epoch = UINT32_MAX;
}

void rv32_native_jit_bind(
	uint32_t *guest_regs, uint8_t *guest_ram, void *opaque,
	uint32_t (*load)(struct rv32_native_jit_runtime *runtime,
			 uint32_t virtual_address, uint32_t descriptor),
	uint32_t (*store)(struct rv32_native_jit_runtime *runtime,
			  uint32_t virtual_address, uint32_t value,
			  uint32_t descriptor))
{
	memset(jit_runtimes, 0, sizeof(jit_runtimes));
	for (uint32_t bank = 0; bank < JIT_MEMORY_BANKS; bank++) {
		struct rv32_native_jit_runtime *runtime = &jit_runtimes[bank];

		runtime->regs = guest_regs;
		runtime->image = guest_ram;
		runtime->opaque = opaque;
		runtime->load = load;
		runtime->store = store;
	}
	jit_rt = &jit_runtimes[JIT_BANK_USER];
	jit_set_exec_privilege(0u);
	memset(jit_negative_cache, 0, sizeof(jit_negative_cache));
	jit_invalidate_translations();
}

void rv32_native_jit_set_ram_refill(bool (*refill)(
	struct rv32_native_jit_runtime *, uint32_t, uint32_t))
{
	if (jit_runtimes[JIT_BANK_USER].refill_ram != refill) {
		rv32_native_jit_flush();
		for (uint32_t bank = 0; bank < JIT_MEMORY_BANKS; bank++)
			jit_runtimes[bank].refill_ram = refill;
	}
}

void rv32_native_jit_set_kernel_linear_map(uint32_t virtual_base,
					   uint32_t size)
{
	/* Pages recorded under the old window may lack their history bit. */
	jit_invalidate_translations();
	/* Kernel regions embed the window start in their linear-map checks. */
	if (jit_kernel_linear_data && jit_code_arena != NULL &&
	    virtual_base != jit_kernel_linear_base)
		rv32_native_jit_flush();
	jit_kernel_linear_base = virtual_base;
	jit_kernel_linear_size = size;
}

void rv32_native_jit_set_kernel_linear_data(uint32_t physical_offset)
{
	/* Each bank takes the window up when it is next selected. */
	jit_invalidate_translations();
	jit_kernel_linear_phys = physical_offset;
	jit_kernel_linear_data = JIT_MAX_PRIVILEGE != 0u;
}

/* The interpreter's kernel_linear_limit: nonzero in S-mode with satp paging
 * on.  Any satp change flushes every bank (MiniRV32FlushTLB reaches
 * rv32_native_jit_memory_cache_flush), so the value read here holds until
 * the bank is selected again; native code never writes satp.  Without a CSR
 * layout the paging state is unknown and generated code keeps the page
 * cache. */
static void jit_select_linear_window(struct rv32_native_jit_runtime *runtime,
				     uint32_t privilege)
{
	uint32_t satp;

	runtime->linear_limit = 0u;
	if (!jit_kernel_linear_data || privilege != 1u ||
	    !jit_csr_layout_valid || runtime->regs == NULL ||
	    runtime->image == NULL || jit_kernel_linear_size <=
		JIT_LINEAR_LOW_MARGIN + JIT_LINEAR_HIGH_MARGIN)
		return;
	memcpy(&satp, (const uint8_t *)runtime->regs + jit_csr_layout.satp,
	       sizeof(satp));
	if ((satp & 0x80000000u) == 0u)
		return;
	runtime->linear_host = (uint32_t)(uintptr_t)runtime->image +
		jit_kernel_linear_phys + JIT_LINEAR_LOW_MARGIN;
	runtime->linear_limit = jit_kernel_linear_size -
		JIT_LINEAR_LOW_MARGIN - JIT_LINEAR_HIGH_MARGIN;
}

/* The ready or rejected way of set holding this head, or NULL.  The guest
 * PC is compared first: it differs in nearly every other way, so a miss
 * costs one load and branch per way.  An empty way's stale tags are harmless
 * because its state is checked once both match. */
static inline struct jit_cache_entry *jit_lookup_set(
	uint32_t physical_offset, uint32_t guest_pc, uint32_t set,
	uint32_t *slot)
{
	struct jit_cache_entry *ways = &jit_entries[set * JIT_CACHE_WAYS];

	for (uint32_t way = 0; way < JIT_CACHE_WAYS; way++) {
		struct jit_cache_entry *entry = &ways[way];

		if (entry->guest_pc != guest_pc ||
		    entry->physical_offset != physical_offset ||
		    entry->state == JIT_ENTRY_EMPTY)
			continue;
		*slot = set * JIT_CACHE_WAYS + way;
		/* The next region replaces some other way of the set. */
		if (jit_replace_way[set] == way)
			jit_replace_way[set] =
				(uint8_t)((way + 1u) & (JIT_CACHE_WAYS - 1u));
		return entry;
	}
	*slot = UINT32_MAX;
	return NULL;
}

static inline struct jit_cache_entry *jit_lookup_entry(
	uint32_t physical_offset, uint32_t guest_pc, uint32_t *slot,
	uint32_t *set)
{
	*set = jit_hash(physical_offset, guest_pc, JIT_CACHE_SETS - 1u);
	return jit_lookup_set(physical_offset, guest_pc, *set, slot);
}

static inline uint32_t jit_context_privilege(uint32_t memory_context)
{
	return (memory_context >> JIT_CONTEXT_PRIVILEGE_SHIFT) & 3u;
}

/* Slow path: privilege, SUM/MXR or TLB epoch changed. */
static __attribute__((noinline)) void jit_select_memory_context(
	uint32_t memory_context)
{
	uint32_t privilege = jit_context_privilege(memory_context);
	uint32_t epoch = memory_context & JIT_CONTEXT_EPOCH_MASK;
	struct rv32_native_jit_runtime *runtime;

	if (privilege == 0u)
		runtime = &jit_runtimes[JIT_BANK_USER];
	else if (memory_context & JIT_CONTEXT_SUM)
		runtime = &jit_runtimes[JIT_BANK_SUPERVISOR_SUM];
	else
		runtime = &jit_runtimes[JIT_BANK_SUPERVISOR];
	jit_rt = runtime;
	jit_set_exec_privilege(privilege);
	if (runtime->memory_context == memory_context)
		return;
	/* The interpreter normally flushed already, leaving UINT32_MAX. */
	if (epoch != jit_memory_epoch) {
		if (jit_memory_epoch != UINT32_MAX)
			jit_invalidate_translations();
		jit_memory_epoch = epoch;
	}
	/* Same epoch, new SUM/MXR: only this bank's data permissions change. */
	jit_invalidate_memory_pages(runtime);
	runtime->memory_context = memory_context;
	jit_select_linear_window(runtime, privilege);
}

static inline void jit_adopt_memory_context(uint32_t memory_context)
{
	if (__builtin_expect(jit_rt->memory_context != memory_context, 0))
		jit_select_memory_context(memory_context);
}

void IRAM_ATTR rv32_native_jit_seed_ram_page(uint32_t physical_offset,
	uint32_t virtual_address, uint32_t memory_context, bool write)
{
	if (jit_code_arena == NULL || jit_rt->image == NULL ||
	    ((physical_offset | virtual_address) & 0xfffu) != 0u ||
	    jit_context_privilege(memory_context) > JIT_MAX_PRIVILEGE)
		return;
	/* Adopt first so the mapping lands in the right bank and epoch. */
	jit_adopt_memory_context(memory_context);
	uint32_t vpn = virtual_address >> 12;
	uint32_t index = vpn & (RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u);
	uint32_t bias = (uint32_t)(uintptr_t)(jit_rt->image + physical_offset) -
		virtual_address;
	if (write) {
		struct rv32_native_jit_write_page *page = &jit_rt->write_pages[index];
		page->host_bias = bias;
		page->linear_page_start = UINT32_MAX;
		page->linear_page_end = 0u;
		__atomic_store_n(&page->virtual_page, vpn, __ATOMIC_RELEASE);
	} else {
		struct rv32_native_jit_read_page *page = &jit_rt->read_pages[index];
		page->host_bias = bias;
		__atomic_store_n(&page->virtual_page, vpn, __ATOMIC_RELEASE);
	}
}

#if CONFIG_RV32_GUEST_HLE
/* Guest library entry points the host runs natively (guest_hle.h): the
 * kernel virtual PC of each routine, UINT32_MAX until identified, and a
 * filter with bit (pc >> 2) & 31 set for each, so other heads are rejected
 * with one test.  A head is identified by its code on its first visit;
 * entries go with the compiled code, so a FENCE.I or reboot re-identifies
 * them.  An identified head is never compiled, so no native link reaches it
 * and every call passes through the dispatcher.  One slot per routine:
 * routines whose PCs share low bits cannot displace each other. */
static uint32_t jit_hle_pcs[GUEST_HLE_KINDS];
static uint32_t jit_hle_filter;
static rv32_native_jit_hle_fn jit_hle_run;
static uint32_t jit_hle_ram_size;

/* Routine kinds still to identify: while any remains, a kernel head's first
 * visit reads its instruction and compares it with the routines' first words,
 * so identification does not depend on the head's warm entry surviving to a
 * second probe (the EROFS LZ4 partial decoder, called a few times a second,
 * never got there and decoded in the guest). */
static uint32_t jit_hle_unidentified;
/* A routine the kernel never calls would keep that check on every first
 * visit (`find` +2.4 %): it stops after this many kernel first visits
 * without an identification, which re-arms it. */
#define JIT_HLE_FIRST_VISIT_BUDGET (1u << 20)
static uint32_t jit_hle_first_visit_budget;

/* The first-visit check tests only the budget, so the budget stays zero
 * without a host or once nothing is left to identify. */
static void jit_hle_forget(void)
{
	for (uint32_t kind = 0u; kind < GUEST_HLE_KINDS; kind++)
		jit_hle_pcs[kind] = UINT32_MAX;
	jit_hle_filter = 0u;
	jit_hle_unidentified = GUEST_HLE_KINDS - 1u;
	jit_hle_first_visit_budget =
		jit_hle_run != NULL ? JIT_HLE_FIRST_VISIT_BUDGET : 0u;
}

static void jit_hle_identified(uint32_t kind, uint32_t guest_pc)
{
	jit_hle_pcs[kind] = guest_pc;
	jit_hle_filter |= 1u << ((guest_pc >> 2) & 31u);
	if (jit_hle_unidentified != 0u)
		jit_hle_unidentified--;
	jit_hle_first_visit_budget = jit_hle_unidentified != 0u ?
		JIT_HLE_FIRST_VISIT_BUDGET : 0u;
}

void rv32_native_jit_set_hle(rv32_native_jit_hle_fn run, uint32_t ram_size)
{
	jit_hle_run = run;
	jit_hle_ram_size = ram_size;
	jit_hle_forget();
}

static inline uint32_t jit_hle_kind(uint32_t guest_pc)
{
	if (((jit_hle_filter >> ((guest_pc >> 2) & 31u)) & 1u) == 0u)
		return GUEST_HLE_NONE;
	for (uint32_t kind = GUEST_HLE_NONE + 1u; kind < GUEST_HLE_KINDS; kind++) {
		if (jit_hle_pcs[kind] == guest_pc)
			return kind;
	}
	return GUEST_HLE_NONE;
}

/* Whether the callee a trace is about to follow is a host routine.  An
 * unidentified callee in the kernel's linear window is identified now, as a
 * probe at its entry would identify it. */
static bool jit_hle_callee(uint32_t guest_pc, uint32_t physical_offset,
			   uint32_t instruction, const uint8_t *guest_ram)
{
	uint32_t kind = jit_hle_kind(guest_pc);

	if (kind != GUEST_HLE_NONE)
		return true;
	if (jit_hle_run == NULL ||
	    guest_pc - jit_kernel_linear_base >= jit_kernel_linear_size)
		return false;
	kind = guest_hle_identify(guest_ram, jit_hle_ram_size, physical_offset,
				  instruction);
	if (kind == GUEST_HLE_NONE || jit_hle_pcs[kind] != UINT32_MAX)
		return false;
	jit_hle_identified(kind, guest_pc);
	return true;
}

/* Runs the routine for the current supervisor call; on success it has
 * returned and result.next_pc is the caller's ra. */
static bool jit_hle_call(struct rv32_native_jit_runtime *runtime,
			 uint32_t memory_context, uint32_t kind)
{
	if (jit_context_privilege(memory_context) != 1u || !jit_hle_run(kind)) {
		rv32_profile_jit_event(RV32_PROFILE_HLE_DECLINED);
		return false;
	}
	rv32_profile_jit_event(RV32_PROFILE_HLE_CALL);
	runtime->result.next_pc = runtime->regs[1];
	return true;
}

static const struct rv32_native_jit_result *jit_try_hle(
	struct rv32_native_jit_runtime *runtime, uint32_t memory_context,
	uint32_t kind)
{
	if (!jit_hle_call(runtime, memory_context, kind)) {
		rv32_profile_set_reason(RV32_PROFILE_HLE_FALLBACK);
		/* The guest runs this call itself, but the next call may suit the
		 * host again: a result with nothing executed makes the interpreter
		 * go on at the head without the probe filter skipping its next
		 * visits, as a NULL result would. */
		runtime->result = (struct rv32_native_jit_result) {
			.native_slot = UINT32_MAX,
			.linear_dirty_start = UINT32_MAX,
		};
		return &runtime->result;
	}
	/* The call counts as one instruction and its return address is a
	 * native successor, like the end of a region. */
	runtime->result = (struct rv32_native_jit_result) {
		.next_pc = runtime->regs[1],
		.executed = 1u,
		.native_successor = 1u,
		.native_slot = UINT32_MAX,
		.linear_dirty_start = UINT32_MAX,
	};
	return &runtime->result;
}
#endif

/* The way for a new region in set: an unused way, else, starting at the
 * replacement way (never the most recently found one), the first ordinary
 * region, else the first loop.  A loop may displace ordinary code, but
 * ordinary code never displaces a set of loops (false); it requalifies after
 * another full general-hot interval instead.  User and kernel regions compete
 * equally: evicting user regions first (the next FENCE.I discards them
 * anyway) let kernel recompiles in a busy set evict the tail of a hot user
 * loop as soon as it was installed; curl over https took 11 % longer. */
static bool jit_choose_slot(uint32_t set, bool candidate_loop, uint32_t *slot_out)
{
	uint32_t base = set * JIT_CACHE_WAYS;
	uint32_t slot = UINT32_MAX;

	for (uint32_t way = 0; way < JIT_CACHE_WAYS; way++) {
		if (jit_entries[base + way].state == JIT_ENTRY_EMPTY) {
			slot = base + way;
			break;
		}
	}
	if (slot == UINT32_MAX) {
		uint32_t best_rank = UINT32_MAX;

		for (uint32_t i = 0; i < JIT_CACHE_WAYS; i++) {
			uint32_t way = (jit_replace_way[set] + i) &
				(JIT_CACHE_WAYS - 1u);
			const struct jit_cache_entry *entry =
				&jit_entries[base + way];
			uint32_t rank = (entry->loops_to_start ? 2u : 0u) +
				(jit_entry_entered[base + way] != 0u);

			if (rank < best_rank) {
				best_rank = rank;
				slot = base + way;
			}
		}
		if (best_rank >= 2u && !candidate_loop)
			return false;
	}
	jit_replace_way[set] = (uint8_t)((slot - base + 1u) &
					 (JIT_CACHE_WAYS - 1u));
	if (jit_entries[slot].state != JIT_ENTRY_EMPTY) {
		jit_stats.replacements++;
		jit_stats.tag_evictions++;
	}
	*slot_out = slot;
	return true;
}

#if JIT_ASYNC_COMPILE
/* Whether jit_choose_slot would find a way in set, without choosing it. */
static bool jit_set_has_room(uint32_t set, bool candidate_loop)
{
	if (candidate_loop)
		return true;
	for (uint32_t way = 0; way < JIT_CACHE_WAYS; way++) {
		const struct jit_cache_entry *entry =
			&jit_entries[set * JIT_CACHE_WAYS + way];

		if (entry->state == JIT_ENTRY_EMPTY || !entry->loops_to_start)
			return true;
	}
	return false;
}

/* Compilation on the other core.  The emulator core collects a hot head's
 * trace into a job and queues it; the worker generates the region into the
 * job; the emulator core installs finished jobs at the start of jit_try,
 * where no native code runs, so the directory, links and arena still change
 * on one core only.  Jobs pass FREE -> QUEUED (emulator core) -> DONE
 * (worker) -> FREE (emulator core) in ring order, each handover a release
 * store of state.  A flush advances jit_code_generation and the jobs queued
 * before it are dropped when they come back. */
#define JIT_ASYNC_JOBS 8u
#define JIT_ASYNC_STACK_BYTES 6144u
_Static_assert((JIT_ASYNC_JOBS & (JIT_ASYNC_JOBS - 1u)) == 0u,
	       "the job ring is indexed with a mask");

enum jit_async_state {
	JIT_ASYNC_FREE,
	JIT_ASYNC_QUEUED,
	JIT_ASYNC_DONE,
};

struct jit_async_job {
	_Alignas(JIT_CODE_ALIGNMENT) uint32_t code[JIT_CODE_WORDS];
	struct jit_trace trace;
	struct jit_codegen gen;
	struct rv32_native_jit_memory_site memory_sites[JIT_MAX_MEMORY_SITES];
	uint32_t entry_values[32];
	uint32_t generation;
	/* A kernel head is dropped only by a full flush (jit_kernel_generation). */
	uint32_t kernel_generation;
	bool kernel;
	uint32_t instruction_limit;
	uint32_t state;
	bool has_entry_values;
	bool refill;
	bool candidate_loop;
	bool generated;
};

/* Jobs and the worker's analysis tables live in PSRAM; the counters the
 * emulator core polls on every lookup stay in internal RAM. */
static struct jit_async_job *jit_async_jobs;
static struct jit_compile_workspace *jit_async_workspace;
static TaskHandle_t jit_async_worker_task;
static uint32_t jit_async_submit_index;
static uint32_t jit_async_install_index;
static uint32_t jit_async_installed;
static bool jit_async_queue_full;
static uint32_t jit_async_done;		/* written by the worker */

static bool jit_trace_spans_pages(const struct jit_trace *trace)
{
	for (uint32_t i = 1u; i < trace->count; i++) {
		if ((trace->guest_pcs[i] & ~0xfffu) != (trace->guest_pc & ~0xfffu))
			return true;
	}
	return false;
}

/* Queues the hot head; false with jit_async_queue_full set when every job is
 * busy, or false for a trace across pages, which depends on mappings that
 * may change before the job is installed. */
static bool jit_async_submit(struct rv32_native_jit_runtime *runtime,
			     uint32_t physical_offset, uint32_t guest_pc,
			     uint32_t instruction_limit, bool candidate_loop)
{
	struct jit_async_job *job = &jit_async_jobs[
		jit_async_submit_index & (JIT_ASYNC_JOBS - 1u)];

	jit_async_queue_full =
		__atomic_load_n(&job->state, __ATOMIC_ACQUIRE) != JIT_ASYNC_FREE;
	if (jit_async_queue_full)
		return false;
	jit_collect_trace(&job->trace, runtime->image, physical_offset,
			  guest_pc, instruction_limit);
	if (jit_trace_spans_pages(&job->trace))
		return false;
#if CONFIG_RV32_NATIVE_JIT_SPLIT_LOAD_PAGES
	memcpy(job->entry_values, runtime->regs, sizeof(job->entry_values));
	job->has_entry_values = true;
#else
	job->has_entry_values = false;
#endif
	job->instruction_limit =
		instruction_limit < JIT_MAX_GUEST_INSTRUCTIONS ?
		instruction_limit : JIT_MAX_GUEST_INSTRUCTIONS;
	job->refill = jit_runtimes[JIT_BANK_USER].refill_ram != NULL;
	job->candidate_loop = candidate_loop;
	job->generation = jit_code_generation;
#if JIT_KEEP_KERNEL
	job->kernel = jit_kernel_pc(guest_pc);
	job->kernel_generation = jit_kernel_generation;
#else
	job->kernel = false;
	job->kernel_generation = 0u;
#endif
	__atomic_store_n(&job->state, JIT_ASYNC_QUEUED, __ATOMIC_RELEASE);
	jit_async_submit_index++;
	jit_stats.async_submitted++;
	xTaskNotifyGive(jit_async_worker_task);
	return true;
}

static void jit_async_install_job(struct jit_async_job *job)
{
	const struct jit_trace *trace = &job->trace;
	struct jit_warm_entry *warm = &jit_warm[jit_hash(
		trace->physical_offset, trace->guest_pc, JIT_WARM_SLOTS - 1u)];
	struct jit_cache_entry *entry;
	uint32_t slot;
	uint32_t set;

	bool warm_matches = warm->valid &&
		warm->physical_offset == trace->physical_offset &&
		warm->guest_pc == trace->guest_pc;
	bool stale = job->generation != jit_code_generation;

#if JIT_KEEP_KERNEL
	if (job->kernel)
		stale = job->kernel_generation != jit_kernel_generation;
#endif
	if (stale) {
		jit_stats.async_stale++;
		/* A fence that kept the warm table left the head pending: its next
		 * visit queues it again, with a freshly collected trace. */
		if (warm_matches)
			warm->reserved &= (uint8_t)~JIT_WARM_PENDING;
		return;
	}
	entry = jit_lookup_entry(trace->physical_offset, trace->guest_pc,
				 &slot, &set);
	if (entry == NULL && !jit_choose_slot(set, job->candidate_loop, &slot)) {
		/* Filled since the head was queued: requalify, as in jit_try. */
		jit_stats.async_set_full++;
		if (warm_matches) {
			warm->hits = 0u;
			warm->reserved &= (uint8_t)~JIT_WARM_PENDING;
		}
		return;
	}
	/* As after a synchronous compile, the head leaves the warm table. */
	if (warm_matches)
		warm->valid = 0u;
	if (entry != NULL && entry->state == JIT_ENTRY_READY) {
		jit_stats.async_duplicates++;
		return;
	}
	if (job->generated && jit_install_generated(slot, &job->gen)) {
		jit_stats.async_installed++;
		return;
	}
	jit_install_rejection(slot, trace, job->gen.rejection, jit_rt->image);
}

static __attribute__((noinline)) void jit_async_install_done(void)
{
#if CONFIG_RV32_HOST_PERF_STATS
	uint32_t install_start = jit_profile_cycle_count();
#endif

	while (jit_async_installed !=
	       __atomic_load_n(&jit_async_done, __ATOMIC_ACQUIRE)) {
		struct jit_async_job *job = &jit_async_jobs[
			jit_async_install_index & (JIT_ASYNC_JOBS - 1u)];

		if (__atomic_load_n(&job->state, __ATOMIC_ACQUIRE) !=
		    JIT_ASYNC_DONE)
			break;
		jit_async_install_job(job);
		__atomic_store_n(&job->state, JIT_ASYNC_FREE, __ATOMIC_RELEASE);
		jit_async_install_index++;
		jit_async_installed++;
	}
#if CONFIG_RV32_HOST_PERF_STATS
	jit_stats.compile_cycles += (uint32_t)(
		jit_profile_cycle_count() - install_start);
#endif
}

static void jit_async_worker(void *argument)
{
	uint32_t index = 0u;

	(void)argument;
	for (;;) {
		struct jit_async_job *job =
			&jit_async_jobs[index & (JIT_ASYNC_JOBS - 1u)];
#if CONFIG_RV32_HOST_PERF_STATS
		uint32_t start;
#endif

		if (__atomic_load_n(&job->state, __ATOMIC_ACQUIRE) !=
		    JIT_ASYNC_QUEUED) {
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			continue;
		}
#if CONFIG_RV32_HOST_PERF_STATS
		start = jit_profile_cycle_count();
#endif
		job->gen = (struct jit_codegen) {
			.trace = &job->trace,
			.instruction_limit = job->instruction_limit,
			.entry_values = job->has_entry_values ?
				job->entry_values : NULL,
			.refill = job->refill,
			.workspace = jit_async_workspace,
			.code = job->code,
			.memory_sites = job->memory_sites,
		};
		job->generated = jit_codegen_splitting(&job->gen);
#if CONFIG_RV32_HOST_PERF_STATS
		jit_stats.async_worker_cycles += (uint32_t)(
			jit_profile_cycle_count() - start);
#endif
		__atomic_store_n(&job->state, JIT_ASYNC_DONE, __ATOMIC_RELEASE);
		__atomic_add_fetch(&jit_async_done, 1u, __ATOMIC_RELEASE);
		index++;
	}
}
#endif

void rv32_native_jit_start_async_compiler(void)
{
#if JIT_ASYNC_COMPILE
	struct jit_async_job *jobs;
	struct jit_compile_workspace *workspace;

	if (jit_async_worker_task != NULL || jit_code_arena == NULL)
		return;
	jobs = heap_caps_aligned_calloc(JIT_CODE_ALIGNMENT, JIT_ASYNC_JOBS,
		sizeof(*jobs), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	workspace = heap_caps_calloc(1, sizeof(*workspace),
		MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	if (jobs == NULL || workspace == NULL) {
		printf("WARNING: no PSRAM for the CPU1 JIT compiler; compiling on CPU0\n");
		heap_caps_free(jobs);
		heap_caps_free(workspace);
		return;
	}
	jit_async_workspace = workspace;
	jit_async_jobs = jobs;
	if (xTaskCreatePinnedToCore(jit_async_worker, "rv32-jit",
				    JIT_ASYNC_STACK_BYTES, NULL,
				    tskIDLE_PRIORITY + 1u,
				    &jit_async_worker_task, 1) != pdPASS) {
		printf("WARNING: CPU1 JIT compiler task unavailable; compiling on CPU0\n");
		jit_async_jobs = NULL;
		jit_async_workspace = NULL;
		heap_caps_free(jobs);
		heap_caps_free(workspace);
	}
#endif
}

/* An inline CSR write in the chain may have made a supervisor interrupt
 * deliverable or changed SUM/MXR.  The interpreter delivers the one and
 * adopts the other only after native execution stops, so the chain must end
 * before the guest can mask the interrupt again. */
static inline bool jit_interpreter_must_resume(
	const struct rv32_native_jit_runtime *runtime, uint32_t memory_context)
{
	const uint8_t *state = (const uint8_t *)runtime->regs;
	uint32_t status;
	uint32_t pending;
	uint32_t enabled;

	if (!jit_csr_layout_valid)
		return false;
	memcpy(&status, state + jit_csr_layout.sstatus, sizeof(status));
	if (((status >> 18) & 3u) != ((memory_context >> 10) & 3u))
		return true;
	memcpy(&pending, state + jit_csr_layout.sip, sizeof(pending));
	memcpy(&enabled, state + jit_csr_layout.sie, sizeof(enabled));
	if ((pending & enabled & ((1u << 9) | (1u << 5))) == 0u)
		return false;
	return jit_context_privilege(memory_context) == 0u ||
		(status & (1u << 1)) != 0u;
}

_Static_assert(offsetof(struct rv32_native_jit_runtime, dispatch_miss_pc) == 92u,
	       "rv32_native_jit_dispatch hard-codes dispatch_miss_pc");

/* Calls generated code at block with runtime in a0 and returns its bailout
 * word, as rv32_native_jit_enter does.  Regions may use every callee-saved
 * register but never sp, gp or tp, and return through ra.  Naming the
 * registers as clobbers makes jit_run's prologue save the caller's values
 * once per try, where the wrapper saved them again around every region it
 * entered; jit_run keeps its own few live values on its stack instead.  The
 * caller-saved floating-point registers are listed for C callbacks the refill
 * trampoline reaches.  A frame pointer cannot be clobbered, so such builds
 * keep the wrapper. */
#if defined(CONFIG_ESP_SYSTEM_USE_FRAME_POINTER) && \
	CONFIG_ESP_SYSTEM_USE_FRAME_POINTER
#define jit_enter_block rv32_native_jit_enter
#else
static inline __attribute__((always_inline)) uint32_t jit_enter_block(
	jit_block_fn block, struct rv32_native_jit_runtime *runtime)
{
	register uintptr_t argument __asm__("a0") = (uintptr_t)runtime;
	register uintptr_t target __asm__("t0") = (uintptr_t)block;

	__asm__ volatile("jalr ra, 0(%1)"
			 : "+r"(argument), "+r"(target)
			 :
			 : "ra", "t1", "t2", "t3", "t4", "t5", "t6",
			   "a1", "a2", "a3", "a4", "a5", "a6", "a7",
			   "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
			   "s8", "s9", "s10", "s11",
#ifdef __riscv_flen
			   "ft0", "ft1", "ft2", "ft3", "ft4", "ft5", "ft6",
			   "ft7", "ft8", "ft9", "ft10", "ft11",
			   "fa0", "fa1", "fa2", "fa3", "fa4", "fa5", "fa6",
			   "fa7",
#endif
			   "memory");
	return (uint32_t)argument;
}
#endif

/* Runs the native chain from the ready region in slot.  Out of line, so a
 * failed probe saves no registers.  The try's arguments come first, in the
 * registers jit_try received them in (noipa keeps that order). */
static __attribute__((noipa)) const struct rv32_native_jit_result *
IRAM_ATTR jit_run(uint32_t physical_offset, uint32_t guest_pc,
		  uint32_t instruction_limit, uint32_t memory_context,
		  struct rv32_native_jit_runtime *const runtime,
		  struct jit_cache_entry *entry, uint32_t slot)
{
	uint32_t set;
	uint32_t total_executed;
#if CONFIG_RV32_HOST_PERF_STATS
	const uint32_t profile_slot = slot;
#endif
#if CONFIG_RV32_HOST_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
	uint32_t block_bailouts = 0u;
	uint32_t chained_lookups = 0u;
	uint32_t cross_page_chains = 0u;
#endif

	runtime->dirty_epoch++;
	if (__builtin_expect(runtime->dirty_epoch == 0u, 0)) {
		for (uint32_t index = 0;
		     index < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; index++)
			runtime->write_pages[index].dirty_epoch = 0u;
		runtime->dirty_epoch = 1u;
	}
	/* result.native_successor is set before each region below. */
	runtime->result.linear_dirty_start = UINT32_MAX;
	runtime->result.linear_dirty_end = 0u;
	runtime->result.linear_store_calls = 0u;
	runtime->result.linear_store_bytes = 0u;
	runtime->memory_calls = 0u;
	runtime->dispatch_miss_pc = UINT32_MAX;
	runtime->chain_instruction_limit = instruction_limit;
	runtime->chain_executed = 0u;
#if RV32_NATIVE_JIT_RUNTIME_COUNTERS
	/* Only generated counter code increments these. */
	runtime->chain_blocks = 0u;
	runtime->chain_links = 0u;
#endif

	for (;;) {
		jit_block_fn block;
		struct jit_cache_entry *returned_entry;
		bool crossed_page;
		uint32_t bailout;
		uint32_t executed;
		uint32_t returned_slot;
		uint32_t remaining =
			instruction_limit - runtime->chain_executed;
		uint32_t next_pc;
#if CONFIG_RV32_GUEST_HLE
		uint32_t hle_kind;
#endif

		if (entry->guest_instruction_count > remaining) {
			if (runtime->chain_executed == 0u) {
				rv32_profile_set_reason(RV32_PROFILE_BUDGET);
				return NULL;
			}
			break;
		}
		runtime->memory_value = 0u;
		runtime->result.next_pc = guest_pc;
		runtime->result.executed = 0u;
		runtime->result.trap = 0u;
		runtime->result.trap_value = 0u;
		runtime->result.native_successor = 0u;
		runtime->result.native_slot = UINT32_MAX;
		/* runtime->sites is read only by jit_resolve_memory_bailout, which
		 * gets it from the region that returned (below). */
		/* A loop region derives its iteration count from the
		 * remaining budget, at most JIT_LOOP_WAKE_BUDGET guest
		 * instructions so a pending host interrupt is noticed. */
		runtime->loop_iterations = 0u;
		block = (jit_block_fn)(uintptr_t)jit_dispatch_code(slot,
							      entry);
	#if CONFIG_RV32_HOST_PERF_STATS
		uint32_t native_cycle_start = jit_profile_cycle_count();
		uint64_t refill_cycle_start = jit_stats.ram_refill_cycles;
	#endif
		bailout = jit_enter_block(block, runtime);
	#if CONFIG_RV32_HOST_PERF_STATS
		jit_stats.native_cycles += (uint32_t)(
			jit_profile_cycle_count() - native_cycle_start) -
			(jit_stats.ram_refill_cycles - refill_cycle_start);
		jit_stats.native_entries++;
	#endif
		returned_slot = runtime->result.native_slot;
		if (returned_slot >= JIT_CACHE_SLOTS ||
		    jit_entries[returned_slot].state != JIT_ENTRY_READY) {
			runtime->result.trap = 2u;
			runtime->result.trap_value = 0u;
			break;
		}
		returned_entry = &jit_entries[returned_slot];
		/* A tail-linked target installs no C-side site pointer. Recover it
		 * from the final slot before invoking the checked slow path. */
		if (bailout != 0u) {
#if CONFIG_RV32_HOST_PERF_STATS
			uint32_t resolve_start = jit_profile_cycle_count();
#endif
			runtime->sites = jit_slot_sites(returned_slot);
			runtime->site_count =
				returned_entry->memory_site_count;
			jit_resolve_memory_bailout(runtime, bailout);
			runtime->chain_executed +=
				runtime->result.executed;
#if CONFIG_RV32_HOST_PERF_STATS
			jit_stats.resolve_cycles += (uint32_t)(
				jit_profile_cycle_count() - resolve_start);
#endif
		}
		executed = runtime->result.executed;

#if CONFIG_RV32_HOST_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
		block_bailouts +=
			executed < returned_entry->guest_instruction_count;
#endif
		total_executed = runtime->chain_executed;
		if (__builtin_expect(total_executed > instruction_limit, 0)) {
			runtime->result.trap = 2u;
			runtime->result.trap_value = 0u;
			break;
		}
		if (executed == 0u || bailout != 0u ||
		    runtime->result.trap != 0u || JIT_HOST_WAKE_PENDING())
			break;
		next_pc = runtime->result.next_pc;
		/* Most chains end where the dispatcher has just searched the
		 * directory and found no ready region.  Unless a host routine
		 * starts there, stop before the checks below: after every
		 * native return the interpreter delivers an interrupt that an
		 * inline CSR write made pending and reads SUM/MXR afresh, which
		 * is all jit_interpreter_must_resume would decide here. */
		if (next_pc == runtime->dispatch_miss_pc
#if CONFIG_RV32_GUEST_HLE
		    && jit_hle_kind(next_pc) == GUEST_HLE_NONE
#endif
		   ) {
			runtime->result.native_successor = 1u;
#if CONFIG_RV32_HOST_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
			chained_lookups++;
#endif
			break;
		}
		if (jit_interpreter_must_resume(runtime, memory_context))
			break;

		runtime->result.native_successor = 1u;
		if (instruction_limit - total_executed <
		    JIT_MIN_GUEST_INSTRUCTIONS)
			break;
		/* Native links may have crossed several independently compiled
		 * regions. Continue from the authoritative final entry, not from
		 * the region that C originally called. */
		physical_offset = returned_entry->physical_offset;
		guest_pc = returned_entry->guest_pc;
		crossed_page = (next_pc & ~0xfffu) !=
			(guest_pc & ~0xfffu);
		/* Within one translated page the physical delta is identical. Across
		 * pages, chain only through a mapping previously validated by the
		 * ordinary Sv32 execute path in this exact memory context. */
		if (!crossed_page)
			physical_offset += next_pc - guest_pc;
		else if (!jit_resolve_exec_offset(next_pc, &physical_offset))
			break;
		guest_pc = next_pc;
#if CONFIG_RV32_HOST_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
		chained_lookups++;
#endif
#if CONFIG_RV32_GUEST_HLE
		hle_kind = jit_hle_kind(guest_pc);
		if (__builtin_expect(hle_kind != GUEST_HLE_NONE, 0)) {
			/* Native code called a host routine: run it and go on at
			 * the return address, as one more instruction. */
			if (!jit_hle_call(runtime, memory_context, hle_kind))
				break;
			runtime->chain_executed++;
			next_pc = runtime->result.next_pc;
			if (instruction_limit - runtime->chain_executed <
				JIT_MIN_GUEST_INSTRUCTIONS ||
			    !jit_resolve_exec_offset(next_pc, &physical_offset))
				break;
			guest_pc = next_pc;
		}
#endif
		/* The dispatcher has just searched the directory for this target
		 * and found no ready region. */
		if (guest_pc == runtime->dispatch_miss_pc)
			break;
		entry = jit_lookup_entry(physical_offset, guest_pc, &slot,
					 &set);
		if (entry == NULL || entry->state != JIT_ENTRY_READY)
			break;
#if CONFIG_RV32_GUEST_HLE
		/* The region did not exit to this PC; its call did. */
		if (hle_kind == GUEST_HLE_NONE)
#endif
			jit_observe_link(returned_slot, guest_pc, slot, entry);
#if CONFIG_RV32_HOST_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
		cross_page_chains += crossed_page;
#endif
	}
	total_executed = runtime->chain_executed;
#if CONFIG_RV32_HOST_PERF_STATS
	jit_profile_guest_instructions[profile_slot] += total_executed;
#endif
#if CONFIG_RV32_HOST_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
	jit_stats.lookups += chained_lookups + runtime->chain_links;
	jit_stats.calls += runtime->chain_blocks;
	jit_stats.guest_instructions += total_executed;
	jit_stats.memory_calls += runtime->memory_calls;
	jit_stats.memory_bailouts += block_bailouts;
	jit_stats.cross_page_chains += cross_page_chains;
	jit_stats.native_link_transitions += runtime->chain_links;
#endif
	runtime->result.executed = total_executed;
	if (total_executed == 0u && runtime->result.trap == 0u) {
		rv32_profile_set_reason(RV32_PROFILE_NO_PROGRESS);
		return NULL;
	}
	return &runtime->result;
}

/* A visit to a head the warm table holds: check its first instruction once,
 * count the visit and queue or compile the head once it is hot.  set is the
 * head's directory set; the try's arguments stay in their registers. */
static __attribute__((noinline)) const struct rv32_native_jit_result *
IRAM_ATTR jit_probe_warm(uint32_t physical_offset, uint32_t guest_pc,
			 uint32_t instruction_limit, uint32_t memory_context,
			 uint32_t warm_weight, struct jit_warm_entry *warm,
			 uint32_t set)
{
	struct rv32_native_jit_runtime *const runtime = jit_rt;
	uint32_t instruction;
	uint32_t slot;
	bool candidate_loop;

	if ((warm->reserved & JIT_WARM_UNCHECKED) != 0u) {
		/* The second visit loads the physical instruction once, so
		 * privileged or otherwise unsupported starts and host routines are
		 * found before the head warms further.  Later visits skip that
		 * guest-RAM read; compilation revalidates the code. */
		memcpy(&instruction, runtime->image + physical_offset,
		       sizeof(instruction));
#if CONFIG_RV32_GUEST_HLE
		/* Once every routine is identified a match would be ignored. */
		if (jit_hle_unidentified != 0u && jit_hle_run != NULL &&
		    jit_context_privilege(memory_context) == 1u) {
			uint32_t hle_kind = guest_hle_identify(runtime->image,
				jit_hle_ram_size, physical_offset, instruction);

			if (hle_kind != GUEST_HLE_NONE &&
			    jit_hle_pcs[hle_kind] == UINT32_MAX) {
				jit_hle_identified(hle_kind, guest_pc);
				warm->valid = 0u;
				return jit_try_hle(runtime, memory_context, hle_kind);
			}
		}
#endif
		if (jit_instruction_kind(instruction) == JIT_INSN_UNSUPPORTED) {
#if CONFIG_RV32_JIT_EXIT_TRACE
			rv32_jit_trace_record(RV32_JIT_TRACE_UNSUPPORTED, guest_pc,
				physical_offset, instruction, 0u, 0u, 0u);
#endif
#if CONFIG_RV32_HOST_PERF_STATS
			jit_stats.unsupported_probes[
				jit_unsupported_class(instruction)]++;
#endif
			jit_remember_negative(physical_offset);
			warm->valid = 0u;
			rv32_profile_set_reason(RV32_PROFILE_UNSUPPORTED);
			return NULL;
		}
		warm->reserved &= (uint8_t)~JIT_WARM_UNCHECKED;
	}
#if JIT_ASYNC_COMPILE
	if ((warm->reserved & JIT_WARM_PENDING) != 0u) {
		rv32_profile_set_reason(RV32_PROFILE_COMPILING);
		return NULL;
	}
#endif
	/* A caller that skipped probes for this head accounts for them here. */
	if (warm->hits < UINT16_MAX - warm_weight)
		warm->hits = (uint16_t)(warm->hits + warm_weight);
	else
		warm->hits = UINT16_MAX;
	if (warm->hits < JIT_LOOP_HOT_THRESHOLD) {
		rv32_profile_set_reason(RV32_PROFILE_WARMING);
		return NULL;
	}
	if ((warm->reserved & JIT_WARM_CLASSIFIED) == 0u) {
		warm->reserved = JIT_WARM_CLASSIFIED;
		if (jit_candidate_loops_to_start(runtime->image,
				physical_offset, guest_pc, instruction_limit))
			warm->reserved |= JIT_WARM_LOOPS_TO_START;
	}
	candidate_loop = (warm->reserved & JIT_WARM_LOOPS_TO_START) != 0u;
	if (!candidate_loop && warm->hits <
	    (jit_context_privilege(memory_context) != 0u ?
	     JIT_KERNEL_GENERAL_HOT_THRESHOLD : JIT_GENERAL_HOT_THRESHOLD)) {
		rv32_profile_set_reason(RV32_PROFILE_WARMING);
		return NULL;
	}

#if JIT_ASYNC_COMPILE
	if (jit_async_jobs != NULL) {
		if (!jit_set_has_room(set, candidate_loop)) {
			warm->hits = 0u;
			rv32_profile_set_reason(RV32_PROFILE_SET_FULL);
			return NULL;
		}
		if (jit_async_submit(runtime, physical_offset, guest_pc,
				     instruction_limit, candidate_loop)) {
			warm->reserved |= JIT_WARM_PENDING;
			rv32_profile_set_reason(RV32_PROFILE_COMPILING);
			return NULL;
		}
		if (jit_async_queue_full) {
			/* Try again on a later visit rather than stall here. */
			jit_stats.async_queue_full++;
			rv32_profile_set_reason(RV32_PROFILE_WARMING);
			return NULL;
		}
		jit_stats.async_local++;
	}
#endif
	if (!jit_choose_slot(set, candidate_loop, &slot)) {
		warm->hits = 0u;
		rv32_profile_set_reason(RV32_PROFILE_SET_FULL);
		return NULL;
	}
	{
#if CONFIG_RV32_HOST_PERF_STATS
		uint32_t compile_start = jit_profile_cycle_count();
#endif
		(void)jit_compile_splitting(slot, runtime->image,
			physical_offset, guest_pc, instruction_limit,
#if CONFIG_RV32_NATIVE_JIT_SPLIT_LOAD_PAGES
			runtime->regs
#else
			NULL
#endif
		);
#if CONFIG_RV32_HOST_PERF_STATS
		jit_stats.compile_cycles += (uint32_t)(
			jit_profile_cycle_count() - compile_start);
#endif
	}
	warm->valid = 0u;
	rv32_profile_set_reason(RV32_PROFILE_COMPILING);
	return NULL;
}

#if CONFIG_RV32_GUEST_HLE
/* A first visit to a kernel head while a host routine is still unidentified
 * and the budget lasts: identify the head by its code.  Bounded by the
 * budget, so it stays in flash with guest_hle_identify. */
static __attribute__((noinline)) const struct rv32_native_jit_result *
jit_probe_identify(struct jit_warm_entry *warm, uint32_t physical_offset,
		   uint32_t guest_pc, uint32_t memory_context)
{
	struct rv32_native_jit_runtime *const runtime = jit_rt;
	uint32_t first_word;
	uint32_t hle_kind;

	jit_hle_first_visit_budget--;
	memcpy(&first_word, runtime->image + physical_offset,
	       sizeof(first_word));
	hle_kind = guest_hle_identify(runtime->image, jit_hle_ram_size,
				      physical_offset, first_word);
	if (hle_kind == GUEST_HLE_NONE || jit_hle_pcs[hle_kind] != UINT32_MAX)
		return NULL;
	jit_hle_identified(hle_kind, guest_pc);
	warm->valid = 0u;
	return jit_try_hle(runtime, memory_context, hle_kind);
}
#endif

/* The directory way slot (UINT32_MAX for none) holding a probed head. */
static inline struct jit_cache_entry *jit_slot_entry(uint32_t slot)
{
	return slot < JIT_CACHE_SLOTS ? &jit_entries[slot] : NULL;
}

/* A try once the bank and execute page are current: run a host routine, or
 * a ready region whose bank or page had to be switched first (or that a CPU1
 * install just added), else count a first visit in the warm table and leave
 * later visits to jit_probe_warm.  Every call is a tail call, and the
 * arguments stay in jit_try's registers. */
static __attribute__((noinline)) const struct rv32_native_jit_result *
IRAM_ATTR jit_probe_adopted(uint32_t physical_offset, uint32_t guest_pc,
			    uint32_t instruction_limit,
			    uint32_t memory_context, uint32_t warm_weight,
			    uint32_t slot, uint32_t warm_index)
{
	struct jit_cache_entry *entry = jit_slot_entry(slot);
	struct jit_warm_entry *warm;

#if CONFIG_RV32_GUEST_HLE
	{
		uint32_t hle_kind = jit_hle_kind(guest_pc);

		/* Native callbacks never switch banks. */
		if (__builtin_expect(hle_kind != GUEST_HLE_NONE, 0))
			return jit_try_hle(jit_rt, memory_context, hle_kind);
	}
#endif
#if CONFIG_RV32_HOST_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
	jit_stats.lookups++;
#endif
	if (entry != NULL) {
		if (entry->state == JIT_ENTRY_READY)
			return jit_run(physical_offset, guest_pc,
				       instruction_limit, memory_context,
				       jit_rt, entry, slot);
		if (entry->state == JIT_ENTRY_REJECTED) {
			rv32_profile_set_reason(RV32_PROFILE_REJECTED);
			return NULL;
		}
	}

	/* Direct mapped: a first visit replaces whatever head shared the entry.
	 * Set-associative tables (with refusal, aging or both) left less code
	 * interpreted in the profile build, whose per-instruction hook makes
	 * interpretation look dearer than it is, but on production firmware they
	 * compiled more and were 7-9 % slower on the kernel workload (2026-10-17,
	 * docs/jit-host-routines-2026-10-17.md). */
	warm = &jit_warm[warm_index];
	if (warm->valid && warm->physical_offset == physical_offset &&
	    warm->guest_pc == guest_pc)
		return jit_probe_warm(physical_offset, guest_pc,
				      instruction_limit, memory_context,
				      warm_weight, warm,
				      warm_index & (JIT_CACHE_SETS - 1u));
	rv32_profile_set_reason(RV32_PROFILE_FIRST_VISIT);
	if (warm->valid && warm->hits >= 2u)
		rv32_profile_jit_event(RV32_PROFILE_WARM_EVICTION);
	rv32_profile_jit_event(RV32_PROFILE_WARM_INSERT);
	warm->physical_offset = physical_offset;
	warm->guest_pc = guest_pc;
	warm->hits = 1u;
	warm->valid = 1u;
	warm->reserved = JIT_WARM_UNCHECKED;
#if CONFIG_RV32_GUEST_HLE
	/* A nonzero budget implies a host and a routine to identify. */
	if (jit_hle_first_visit_budget != 0u &&
	    jit_context_privilege(memory_context) == 1u &&
	    guest_pc - jit_kernel_linear_base < jit_kernel_linear_size)
		return jit_probe_identify(warm, physical_offset, guest_pc,
					  memory_context);
#endif
	return NULL;
}

/* The detour of a try whose bank or execute page is not current, or with
 * finished CPU1 jobs: install them, switch the bank and record the execute
 * page before going on. */
static __attribute__((noinline)) const struct rv32_native_jit_result *
IRAM_ATTR jit_probe_adopting(uint32_t physical_offset, uint32_t guest_pc,
			     uint32_t instruction_limit,
			     uint32_t memory_context, uint32_t warm_weight,
			     uint32_t slot, uint32_t warm_index)
{
#if JIT_ASYNC_COMPILE
	const struct jit_cache_entry *entry = jit_slot_entry(slot);

	/* A hint only: jit_async_install_done acquires each job's state.  An
	 * install may add or evict this head's region, so look it up again.
	 * A ready region about to run leaves the installs to the next miss. */
	if ((entry == NULL || entry->state != JIT_ENTRY_READY) &&
	    jit_async_installed !=
	    __atomic_load_n(&jit_async_done, __ATOMIC_RELAXED)) {
		jit_async_install_done();
		(void)jit_lookup_set(physical_offset, guest_pc,
				     warm_index & (JIT_CACHE_SETS - 1u), &slot);
	}
#endif
	jit_adopt_memory_context(memory_context);
	jit_note_exec_page(physical_offset, guest_pc);
	return jit_probe_adopted(physical_offset, guest_pc, instruction_limit,
				 memory_context, warm_weight, slot, warm_index);
}

/* A try that cannot take jit_try's fast path: no ready region for the head
 * (slot is its way, or UINT32_MAX), a budget or privilege refusal, or a host
 * routine.  warm_index is the head's jit_hash_key under the warm-table mask;
 * the directory set is its low bits.  Failed probes outnumber entered
 * regions, so the common ones (a refusal, a negative head, a first visit)
 * save no registers: everything that calls out of line is reached by a tail
 * call. */
static __attribute__((noinline)) const struct rv32_native_jit_result *
IRAM_ATTR jit_probe_miss(uint32_t physical_offset, uint32_t guest_pc,
			 uint32_t instruction_limit, uint32_t memory_context,
			 uint32_t warm_weight, uint32_t slot,
			 uint32_t warm_index)
{
	if (jit_code_arena == NULL || jit_rt->regs == NULL ||
	    jit_rt->image == NULL || instruction_limit <
		JIT_MIN_GUEST_INSTRUCTIONS) {
		rv32_profile_set_reason(RV32_PROFILE_BUDGET);
		return NULL;
	}
	/* satp writes, LR/SC, xRET and MMIO accesses end a region, so S-mode code is
	 * generated as for U-mode; only the data bank differs. */
	if (jit_context_privilege(memory_context) > JIT_MAX_PRIVILEGE) {
		rv32_profile_set_reason(RV32_PROFILE_BUDGET);
		return NULL;
	}
	if (*jit_negative_slot(physical_offset) == (physical_offset | 1u)) {
		rv32_profile_set_reason(RV32_PROFILE_UNSUPPORTED);
		return NULL;
	}
	if (__builtin_expect(
#if JIT_ASYNC_COMPILE
		    jit_async_installed !=
		    __atomic_load_n(&jit_async_done, __ATOMIC_RELAXED) ||
#endif
		    jit_rt->memory_context != memory_context ||
		    !jit_exec_page_current(physical_offset, guest_pc), 0))
		return jit_probe_adopting(physical_offset, guest_pc,
					  instruction_limit, memory_context,
					  warm_weight, slot, warm_index);
	return jit_probe_adopted(physical_offset, guest_pc, instruction_limit,
				 memory_context, warm_weight, slot,
				 warm_index);
}

/* Most tries find a ready region.  Readiness already implies the arena, the
 * runtime and a supported start (no negative entry), so a ready region in the
 * current bank and execute page runs after the checks it still needs; after
 * a bank or page change it runs from jit_probe_adopting.  Every other case
 * continues in jit_probe_miss, where finished CPU1 jobs are installed.  Only
 * tail calls leave this path, so it saves no registers, and the callees take
 * the try's arguments in the same registers. */
static inline __attribute__((always_inline))
const struct rv32_native_jit_result *jit_try(
	uint32_t physical_offset, uint32_t guest_pc,
	uint32_t instruction_limit, uint32_t memory_context,
	uint32_t warm_weight)
{
	uint32_t warm_index = jit_hash_key(physical_offset, guest_pc) &
		(JIT_WARM_SLOTS - 1u);
	uint32_t slot;
	struct jit_cache_entry *entry = jit_lookup_set(physical_offset,
		guest_pc, warm_index & (JIT_CACHE_SETS - 1u), &slot);

	if (__builtin_expect(entry != NULL &&
			     entry->state == JIT_ENTRY_READY, 1) &&
	    instruction_limit >= JIT_MIN_GUEST_INSTRUCTIONS &&
	    jit_context_privilege(memory_context) <= JIT_MAX_PRIVILEGE
#if CONFIG_RV32_GUEST_HLE
	    && jit_hle_kind(guest_pc) == GUEST_HLE_NONE
#endif
	   ) {
		/* jit_probe_adopting switches, counts and runs it. */
		if (__builtin_expect(jit_rt->memory_context != memory_context ||
				     !jit_exec_page_current(physical_offset,
							    guest_pc), 0))
			return jit_probe_adopting(physical_offset, guest_pc,
						  instruction_limit,
						  memory_context, warm_weight,
						  slot, warm_index);
#if CONFIG_RV32_HOST_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
		jit_stats.lookups++;
#endif
		return jit_run(physical_offset, guest_pc, instruction_limit,
			       memory_context, jit_rt, entry, slot);
	}
	return jit_probe_miss(physical_offset, guest_pc, instruction_limit,
			      memory_context, warm_weight, slot, warm_index);
}

const struct rv32_native_jit_result *IRAM_ATTR rv32_native_jit_try(
	uint32_t physical_offset, uint32_t guest_pc,
	uint32_t instruction_limit, uint32_t memory_context)
{
	return jit_try(physical_offset, guest_pc, instruction_limit,
		       memory_context, 1u);
}

const struct rv32_native_jit_result *IRAM_ATTR rv32_native_jit_try_weighted(
	uint32_t physical_offset, uint32_t guest_pc,
	uint32_t instruction_limit, uint32_t memory_context,
	uint32_t warm_weight)
{
	return jit_try(physical_offset, guest_pc, instruction_limit,
		       memory_context, warm_weight);
}

const struct rv32_native_jit_result *rv32_native_jit_try_profiled(
	uint32_t physical_offset, uint32_t guest_pc,
	uint32_t instruction_limit, uint32_t memory_context,
	uint32_t warm_weight)
{
#if CONFIG_RV32_HOST_PERF_STATS
	uint32_t cycle_start = jit_profile_cycle_count();
	uint64_t compile_start = jit_stats.compile_cycles;
#endif
	const struct rv32_native_jit_result *result = jit_try(
		physical_offset, guest_pc, instruction_limit, memory_context,
		warm_weight);
#if CONFIG_RV32_HOST_PERF_STATS
	{
		uint32_t cycles = jit_profile_cycle_count() - cycle_start;

		jit_stats.dispatch_cycles += cycles;
		jit_stats.dispatch_attempts++;
		if (result == NULL && jit_stats.compile_cycles == compile_start) {
			jit_stats.probe_count++;
			jit_stats.probe_cycles += cycles;
		}
	}
#endif
	return result;
}

void rv32_native_jit_flush(void)
{
	rv32_profile_jit_event(RV32_PROFILE_JIT_FLUSH);
	jit_code_generation++;
#if JIT_KEEP_KERNEL
	jit_kernel_generation++;
	jit_kernel_code_written = false;
	jit_user_slot_count = 0u;
	memset(jit_kernel_code_pages, 0, sizeof(jit_kernel_code_pages));
#endif
#if CONFIG_RV32_GUEST_PROFILE
	rv32_native_jit_profile_fold_live();
#endif
	memset(jit_entries, 0, sizeof(jit_entries));
	memset(jit_negative_cache, 0, sizeof(jit_negative_cache));
	memset(jit_warm, 0, JIT_WARM_BYTES);
	memset(jit_replace_way, 0, sizeof(jit_replace_way));
#if CONFIG_RV32_GUEST_HLE
	jit_hle_forget();
#endif
	memset(jit_hot_entries, 0, sizeof(jit_hot_entries));
#if CONFIG_RV32_HOST_PERF_STATS
	/* Preserve interval totals when a guest instruction-cache fence discards
	 * the directory. Only the per-PC attribution is no longer available. */
	for (uint32_t i = 0; i < JIT_CACHE_SLOTS; i++)
		jit_profile_discarded_guest_instructions +=
			jit_profile_guest_instructions[i];
	memset(jit_profile_guest_instructions, 0,
	       sizeof(jit_profile_guest_instructions));
#endif
	jit_has_cross_page_entries = false;
	jit_reset_code_heads();
	if (jit_link_candidates != NULL)
		memset(jit_link_candidates, 0,
		       JIT_CACHE_SLOTS * sizeof(*jit_link_candidates));
	rv32_native_jit_memory_cache_flush();
}

void rv32_native_jit_memory_cache_flush(void)
{
	jit_invalidate_translations();
}

void IRAM_ATTR rv32_native_jit_note_write_page(uint32_t physical_offset)
{
#if JIT_KEEP_KERNEL
	uint32_t page = physical_offset >> 12;

	if (page < JIT_KERNEL_CODE_PAGES &&
	    (jit_kernel_code_pages[page >> 5] &
	     (UINT32_C(1) << (page & 31u))) != 0u)
		jit_kernel_code_written = true;
#else
	(void)physical_offset;
#endif
}

/* FENCE.I.  Linux executes it after mapping each new executable user page,
 * up to ~80 times a second while programs load, and after patching kernel
 * text.  Unless kernel code was written since the last flush, only user
 * regions go: kernel regions, every head's warm-up count and the identified
 * host routines stay.  Translations are reset as before (keeping them
 * measured no faster), which also clears links and the jump cache. */
void rv32_native_jit_fence(void)
{
	jit_stats.fences++;
#if JIT_KEEP_KERNEL
	if (jit_kernel_code_written || jit_kernel_linear_size == 0u) {
		rv32_native_jit_flush();
		return;
	}
	rv32_profile_jit_event(RV32_PROFILE_JIT_FENCE_KEEP);
	/* Queued user regions may describe replaced code; kernel ones stay. */
	jit_code_generation++;
	bool listed = jit_user_slot_count <= JIT_USER_SLOT_LIST;
	uint32_t visits = listed ? jit_user_slot_count : JIT_CACHE_SLOTS;

	for (uint32_t visit = 0; visit < visits; visit++) {
		uint32_t slot = listed ? jit_user_slots[visit] : visit;
		struct jit_cache_entry *entry = &jit_entries[slot];
		struct jit_hot_entry *hot = &jit_hot_entries[slot];

		if (entry->state == JIT_ENTRY_EMPTY ||
		    jit_kernel_pc(entry->guest_pc))
			continue;
#if CONFIG_RV32_GUEST_PROFILE
		jit_profile_fold_slot(slot);
#endif
#if CONFIG_RV32_HOST_PERF_STATS
		jit_profile_discarded_guest_instructions +=
			jit_profile_guest_instructions[slot];
		jit_profile_guest_instructions[slot] = 0u;
#endif
		if (hot->location != 0u)
			jit_hot_mark_range(jit_hot_code_unit(hot),
					   jit_hot_code_units(hot), false);
		hot->location = 0u;
		hot->calls = 0u;
		memset(entry, 0, sizeof(*entry));
	}
	jit_user_slot_count = 0u;
	memset(jit_negative_cache, 0, sizeof(jit_negative_cache));
	rv32_native_jit_memory_cache_flush();
#else
	rv32_native_jit_flush();
#endif
}

void rv32_native_jit_memory_cache_flush_page(uint32_t virtual_address)
{
	uint32_t virtual_page = virtual_address >> 12;
	uint32_t data_index =
		virtual_page & (RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u);
	uint32_t exec_index = jit_exec_page_index(virtual_address);
	uint32_t history_index = jit_exec_history_index(virtual_page);

	/* Page-scoped SFENCE.VMA: evict this VPN's slot in every bank, whatever
	 * page it holds. */
	for (uint32_t bank = 0; bank < JIT_MEMORY_BANKS; bank++) {
		struct rv32_native_jit_runtime *runtime = &jit_runtimes[bank];

		runtime->read_pages[data_index].virtual_page = UINT32_MAX;
		runtime->write_pages[data_index].virtual_page = UINT32_MAX;
		runtime->write_pages[data_index].linear_page_start = UINT32_MAX;
		runtime->write_pages[data_index].linear_page_end = 0u;
	}
	/* Links only depend on executed pages.  Keep the history bit: it may
	 * stand for another page too. */
	if (!(jit_exec_history[history_index >> 5] &
	      (UINT32_C(1) << (history_index & 31u))))
		return;
	jit_exec_pages[exec_index].virtual_page = UINT32_MAX;
	jit_invalidate_cross_page_entries();
}

void rv32_native_jit_get_stats(struct rv32_native_jit_stats *stats)
{
	if (stats == NULL)
		return;
	*stats = jit_stats;
	stats->resident_regions = 0u;
	stats->resident_bytes = 0u;
	stats->hot_resident_regions = 0u;
	stats->hot_resident_bytes = 0u;
	stats->profile_interval_guest_instructions = 0u;
	stats->profile_discarded_guest_instructions = 0u;
	memset(stats->profile_top_regions, 0,
	       sizeof(stats->profile_top_regions));
	for (uint32_t i = 0; i < JIT_CACHE_SLOTS; i++) {
		if (jit_entries[i].state != JIT_ENTRY_READY)
			continue;
		stats->resident_regions++;
		stats->resident_bytes +=
			(uint32_t)jit_entries[i].code_units * JIT_CODE_ALIGNMENT;
		if (jit_hot_entries[i].location != 0u) {
			stats->hot_resident_regions++;
			stats->hot_resident_bytes +=
				jit_hot_code_units(&jit_hot_entries[i]) *
				JIT_CODE_ALIGNMENT;
		}
#if CONFIG_RV32_HOST_PERF_STATS
		{
			uint32_t interval = jit_profile_guest_instructions[i];
			uint32_t position = RV32_NATIVE_JIT_PROFILE_TOP_REGIONS;

			stats->profile_interval_guest_instructions += interval;
			for (uint32_t rank = 0;
			     rank < RV32_NATIVE_JIT_PROFILE_TOP_REGIONS; rank++) {
				if (interval > stats->profile_top_regions[rank].
						guest_instructions) {
					position = rank;
					break;
				}
			}
			if (position < RV32_NATIVE_JIT_PROFILE_TOP_REGIONS) {
				if (position + 1u <
				    RV32_NATIVE_JIT_PROFILE_TOP_REGIONS)
					memmove(&stats->profile_top_regions[position + 1u],
						&stats->profile_top_regions[position],
						(RV32_NATIVE_JIT_PROFILE_TOP_REGIONS -
						 position - 1u) *
						 sizeof(stats->profile_top_regions[0]));
				stats->profile_top_regions[position] =
					(struct rv32_native_jit_region_stats) {
						.guest_pc = jit_entries[i].guest_pc,
						.physical_offset =
							jit_entries[i].physical_offset,
						.guest_instructions = interval,
						.code_bytes =
							(uint32_t)jit_entries[i].code_units *
							JIT_CODE_ALIGNMENT,
						.region_instructions =
							jit_entries[i].guest_instruction_count,
						.memory_sites =
							jit_entries[i].memory_site_count,
						.loops_to_start =
							jit_entries[i].loops_to_start,
						.internal_hot =
							jit_hot_entries[i].location != 0u,
					};
			}
		}
#endif
	}
#if JIT_ARENA_SINGLE_RING
	stats->arena_head_bytes = (uint32_t)jit_code_head * JIT_CODE_ALIGNMENT;
	stats->hot_promote_bar = jit_promote_bar;
#elif JIT_KEEP_KERNEL
	/* Each head's distance into its half partition. */
	stats->arena_head_bytes =
		((uint32_t)jit_kernel_loop_code_head +
		 (uint32_t)(jit_loop_code_head - jit_loop_code_units / 2u) +
		 (uint32_t)(jit_kernel_regular_code_head - jit_loop_code_units) +
		 (uint32_t)(jit_regular_code_head - jit_loop_code_units -
			    (jit_code_arena_units - jit_loop_code_units) / 2u)) *
		JIT_CODE_ALIGNMENT;
#else
	stats->arena_head_bytes =
		((uint32_t)jit_loop_code_head +
		 (uint32_t)jit_regular_code_head - jit_loop_code_units) *
		JIT_CODE_ALIGNMENT;
#endif
	stats->arena_capacity_bytes =
		(uint32_t)jit_code_arena_units * JIT_CODE_ALIGNMENT;
	stats->hot_arena_head_bytes =
		(uint32_t)jit_hot_code_head * JIT_CODE_ALIGNMENT;
	stats->hot_arena_capacity_bytes =
		(uint32_t)jit_hot_code_arena_units * JIT_CODE_ALIGNMENT;
#if CONFIG_RV32_HOST_PERF_STATS
	stats->profile_discarded_guest_instructions =
		jit_profile_discarded_guest_instructions;
	memset(jit_profile_guest_instructions, 0,
	       sizeof(jit_profile_guest_instructions));
	jit_profile_discarded_guest_instructions = 0u;
#endif
}
