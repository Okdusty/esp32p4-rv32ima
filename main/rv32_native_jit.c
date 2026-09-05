#include "rv32_native_jit.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#ifdef ESP_PLATFORM
#include "psram.h"
#endif

/* Generated regions occupy complete 128-byte cache lines in a bounded ring
 * arena.  This keeps publication deterministic on the ESP32-P4's separate
 * data/instruction paths without wasting a fixed 1 KiB on every short region. */
#define JIT_CACHE_SLOTS RV32_NATIVE_JIT_CACHE_SLOTS
#define JIT_CACHE_WAYS 2u
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
#define JIT_CODE_WORDS 512u
#define JIT_CODE_BYTES (JIT_CODE_WORDS * 4u)
#define JIT_CODE_ALIGNMENT 128u
#define JIT_CODE_ARENA_MIN_BYTES (64u * 1024u)
#define JIT_INTERNAL_CODE_ARENA_MAX_BYTES (128u * 1024u)
#define JIT_CODE_ARENA_MAX_BYTES (2048u * 1024u)
#define JIT_INTERNAL_RESERVE_BYTES (64u * 1024u)
#define JIT_HOT_CODE_ARENA_MIN_BYTES (16u * 1024u)
#define JIT_HOT_CODE_ARENA_MAX_BYTES (64u * 1024u)
#define JIT_HOT_CODE_ARENA_STEP_BYTES (16u * 1024u)
#define JIT_CODE_UNIT_WORDS (JIT_CODE_ALIGNMENT / sizeof(uint32_t))
#define JIT_LOOP_HOT_THRESHOLD 16u
#define JIT_GENERAL_HOT_THRESHOLD 64u
/* Fill unused internal executable RAM promptly, but require much stronger
 * evidence before a PSRAM-backed region may displace an existing promotion.
 * The two-stage policy avoids the medium-hot ring ping-pong seen with globally
 * lower thresholds while making short-lived render/copy loops benefit from an
 * otherwise idle tier. */
#define JIT_HOT_LOOP_ADMIT_CALLS 2048u
#define JIT_HOT_GENERAL_ADMIT_CALLS 8192u
#define JIT_HOT_LOOP_REPLACE_CALLS 8192u
#define JIT_HOT_GENERAL_REPLACE_CALLS 32768u
#define JIT_HOT_LOCATION_UNIT_BITS 10u
#define JIT_HOT_LOCATION_UNIT_MASK \
	((1u << JIT_HOT_LOCATION_UNIT_BITS) - 1u)
#define JIT_HOT_OCCUPANCY_WORDS \
	((JIT_HOT_CODE_ARENA_MAX_BYTES / JIT_CODE_ALIGNMENT + 31u) / 32u)
#define JIT_LINK_TARGET_COUNT_MASK 0x3fu
#define JIT_LINK_TARGET_LOOP_FLAG 0x40u
#define JIT_LINK_TARGET_FLAG_MASK (JIT_CODE_ALIGNMENT - 1u)
#define JIT_LINK_CONFIRMATIONS 3u
#define JIT_NATIVE_CHAIN_ENTRY_BYTES (4u * sizeof(uint32_t))
#define JIT_MIN_GUEST_INSTRUCTIONS 3u
#define JIT_SELF_TEST_LOOP_ITERATIONS 32u

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
};

typedef uint32_t (*jit_block_fn)(struct rv32_native_jit_runtime *runtime);

/* Assembly wrapper preserving the host ABI once around a complete linked
 * native chain.  Native successors tail-jump without crossing this boundary. */
extern uint32_t rv32_native_jit_enter(
	jit_block_fn block, struct rv32_native_jit_runtime *runtime);

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

struct jit_builder {
	uint32_t *start;
	uint32_t *cursor;
	uint32_t *end;
	uint32_t *bailout_patches[JIT_MAX_BAILOUT_PATCHES];
	uint32_t *control_exit_patches[JIT_MAX_CONTROL_EXIT_PATCHES];
	uint32_t *dirty_call_patches[JIT_MAX_MEMORY_SITES];
	struct jit_memory_stub *memory_stubs;
	struct jit_loop_read_cache_stub *loop_read_cache_stubs;
	struct jit_loop_write_cache_stub *loop_write_cache_stubs;
	uint8_t bailout_patch_count;
	uint8_t control_exit_patch_count;
	uint8_t dirty_call_patch_count;
	uint8_t memory_stub_count;
	uint8_t loop_read_cache_stub_count;
	uint8_t loop_write_cache_stub_count;
	bool instruction_counter_active;
	bool failed;
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
	uint32_t trace_return_pcs[JIT_MAX_TRACE_CALL_DEPTH];
	uint8_t trace_link_registers[JIT_MAX_TRACE_CALL_DEPTH];
};

static struct jit_cache_entry jit_entries[JIT_CACHE_SLOTS];
static struct rv32_native_jit_memory_site
	(*jit_memory_sites)[JIT_MAX_MEMORY_SITES];
#ifndef ESP_PLATFORM
/* The standalone correctness harness has no capability allocator.  Firmware
 * puts this cold metadata in PSRAM so a larger code directory does not take
 * internal RAM away from I/O workers. */
static struct rv32_native_jit_memory_site
	jit_memory_sites_storage[JIT_CACHE_SLOTS][JIT_MAX_MEMORY_SITES];
#endif
static struct jit_warm_entry *jit_warm;
#ifndef ESP_PLATFORM
static struct jit_warm_entry jit_warm_storage[JIT_WARM_SLOTS];
#endif
static _Alignas(JIT_CODE_ALIGNMENT) struct jit_link_entry
	jit_link_storage[JIT_CACHE_SLOTS];
static struct jit_link_entry *jit_links = jit_link_storage;
static struct jit_link_candidate *jit_link_candidates;
#ifndef ESP_PLATFORM
static struct jit_link_candidate jit_link_candidate_storage[JIT_CACHE_SLOTS];
#endif
static uint8_t jit_replace_way[JIT_CACHE_SETS];
static struct jit_hot_entry jit_hot_entries[JIT_CACHE_SLOTS];
static struct jit_exec_page jit_exec_pages[JIT_EXEC_PAGE_ENTRIES];
static struct jit_compile_workspace jit_compile_workspace;
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
static _Alignas(JIT_CODE_ALIGNMENT) uint32_t jit_staging[JIT_CODE_WORDS];
static uint16_t jit_loop_code_head;
static uint16_t jit_regular_code_head;
static uint16_t jit_loop_code_units;
static uint16_t jit_code_arena_units;
static uint16_t jit_hot_code_head;
static uint16_t jit_hot_code_arena_units;
static uint32_t jit_hot_code_occupancy[JIT_HOT_OCCUPANCY_WORDS];
static struct rv32_native_jit_runtime jit_runtime;
static struct rv32_native_jit_stats jit_stats;
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
	jit_hot_code_head = 0u;
	memset(jit_hot_code_occupancy, 0, sizeof(jit_hot_code_occupancy));
}

_Static_assert(sizeof(struct jit_cache_entry) == 16u,
	       "JIT hot lookup entry must remain a power-of-two size");
_Static_assert(sizeof(struct jit_link_entry) == 8u,
	       "JIT native-link entries must remain two words");
_Static_assert(JIT_CACHE_WAYS == 2u,
	       "The replacement bit implements exactly two cache ways");
_Static_assert((JIT_EXEC_PAGE_ENTRIES & (JIT_EXEC_PAGE_ENTRIES - 1u)) == 0u,
	       "The executable-page cache size must be a power of two");
_Static_assert(JIT_CODE_ARENA_MAX_BYTES / JIT_CODE_ALIGNMENT <= UINT16_MAX,
	       "JIT arena offsets must fit their metadata field");
_Static_assert(JIT_CODE_BYTES / JIT_CODE_ALIGNMENT <= UINT8_MAX,
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
_Static_assert(JIT_CODE_BYTES / JIT_CODE_ALIGNMENT <=
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
#ifdef ESP_PLATFORM
_Static_assert(JIT_CACHE_SLOTS * JIT_MAX_MEMORY_SITES *
	       sizeof(struct rv32_native_jit_memory_site) <=
	       HOST_JIT_METADATA_SIZE,
	       "The reserved host JIT metadata region is too small");
_Static_assert(JIT_CACHE_SLOTS * JIT_MAX_MEMORY_SITES *
	       sizeof(struct rv32_native_jit_memory_site) +
	       JIT_WARM_SLOTS * sizeof(struct jit_warm_entry) +
	       JIT_CACHE_SLOTS * sizeof(struct jit_link_candidate) <=
	       HOST_JIT_METADATA_SIZE,
	       "The reserved host JIT metadata region cannot hold cold tables");
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

static bool jit_sync_code(const uint32_t *start, size_t published_bytes)
{
	esp_err_t result;

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
	return true;
}

static bool jit_publish_code(const uint32_t *start, size_t used_words,
			     size_t published_bytes)
{
	/* Every zero in the active range is an unpatched placeholder and would
	 * raise an illegal-instruction exception.  Reject such a block before its
	 * cache entry can become visible. */
	for (size_t i = 0; i < used_words; i++) {
		if (start[i] == 0u ||
		    jit_instruction_kind(start[i]) == JIT_INSN_UNSUPPORTED)
			return false;
	}

	return jit_sync_code(start, published_bytes);
}

static uint32_t *jit_entry_code(uint32_t slot)
{
	return jit_code_arena +
		(uint32_t)jit_entries[slot].code_unit * JIT_CODE_UNIT_WORDS;
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
	uint32_t flags = entry->guest_instruction_count;

	if (entry->state != JIT_ENTRY_READY || flags == 0u ||
	    flags > JIT_LINK_TARGET_COUNT_MASK ||
	    (code & JIT_LINK_TARGET_FLAG_MASK) != 0u || code > UINT32_MAX)
		return 0u;
	if (entry->loops_to_start)
		flags |= JIT_LINK_TARGET_LOOP_FLAG;
	return (uint32_t)code | flags;
}

static void jit_retarget_links(const uint32_t *old_code,
			       const uint32_t *new_code)
{
	uint32_t old_pointer = (uint32_t)(uintptr_t)old_code;
	uint32_t new_pointer = (uint32_t)(uintptr_t)new_code;

	if (old_code == new_code)
		return;
	for (uint32_t index = 0; index < JIT_CACHE_SLOTS; index++) {
		struct jit_link_entry *link = &jit_links[index];
		struct jit_link_candidate *candidate =
			jit_link_candidates != NULL ? &jit_link_candidates[index] : NULL;

		if (jit_link_target_pointer(link->packed_target) == old_pointer)
			link->packed_target = new_pointer |
				(link->packed_target & JIT_LINK_TARGET_FLAG_MASK);
		if (candidate != NULL &&
		    jit_link_target_pointer(candidate->candidate_target) == old_pointer)
			candidate->candidate_target = new_pointer |
				(candidate->candidate_target & JIT_LINK_TARGET_FLAG_MASK);
	}
}

static void jit_clear_entry_links(uint32_t slot)
{
	uint32_t cold_pointer;
	uint32_t hot_pointer = 0u;

	if (jit_entries[slot].state != JIT_ENTRY_READY ||
	    jit_entries[slot].code_units == 0u) {
		memset(&jit_links[slot], 0, sizeof(jit_links[slot]));
		if (jit_link_candidates != NULL)
			memset(&jit_link_candidates[slot], 0,
			       sizeof(jit_link_candidates[slot]));
		return;
	}
	cold_pointer = (uint32_t)(uintptr_t)jit_entry_code(slot);
	if (jit_hot_entries[slot].location != 0u)
		hot_pointer = (uint32_t)(uintptr_t)jit_hot_entry_code(slot);
	memset(&jit_links[slot], 0, sizeof(jit_links[slot]));
	if (jit_link_candidates != NULL)
		memset(&jit_link_candidates[slot], 0,
		       sizeof(jit_link_candidates[slot]));
	for (uint32_t index = 0; index < JIT_CACHE_SLOTS; index++) {
		struct jit_link_entry *link = &jit_links[index];
		struct jit_link_candidate *observation =
			jit_link_candidates != NULL ? &jit_link_candidates[index] : NULL;
		uint32_t target = jit_link_target_pointer(link->packed_target);
		uint32_t candidate = observation != NULL ?
			jit_link_target_pointer(observation->candidate_target) : 0u;

		if (target == cold_pointer || (hot_pointer != 0u &&
					       target == hot_pointer)) {
			link->packed_target = 0u;
			link->expected_next_pc = 0u;
		}
		if (observation != NULL &&
		    (candidate == cold_pointer || (hot_pointer != 0u &&
						    candidate == hot_pointer))) {
			memset(observation, 0, sizeof(*observation));
		}
	}
}

static void jit_clear_all_links(void)
{
	memset(jit_links, 0, JIT_CACHE_SLOTS * sizeof(*jit_links));
	if (jit_link_candidates != NULL)
		memset(jit_link_candidates, 0,
		       JIT_CACHE_SLOTS * sizeof(*jit_link_candidates));
}

static void jit_forget_hot_code(uint32_t slot)
{
	struct jit_hot_entry *hot = &jit_hot_entries[slot];

	if (hot->location != 0u) {
		jit_hot_mark_range(jit_hot_code_unit(hot),
				   jit_hot_code_units(hot), false);
		if (jit_entries[slot].state == JIT_ENTRY_READY)
			jit_retarget_links(jit_hot_entry_code(slot),
					   jit_entry_code(slot));
	}
	hot->location = 0u;
	hot->calls = 0u;
}

static void jit_forget_entry(uint32_t slot)
{
#if CONFIG_RV32_HOST_PERF_STATS
	jit_profile_discarded_guest_instructions +=
		jit_profile_guest_instructions[slot];
	jit_profile_guest_instructions[slot] = 0u;
#endif
	jit_clear_entry_links(slot);
	jit_forget_hot_code(slot);
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
		if (!allow_eviction)
			return false;
		start = previous_head;
		if (start + units > jit_hot_code_arena_units)
			start = 0u;
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
	if (!jit_sync_code(destination, units * JIT_CODE_ALIGNMENT))
		return false;

	hot->location = jit_hot_location(start, units);
	jit_hot_mark_range(start, units, true);
	jit_retarget_links(jit_entry_code(slot), destination);
	jit_hot_code_head = (uint16_t)(start + units);
	if (jit_hot_code_head == jit_hot_code_arena_units) {
		jit_hot_code_head = 0u;
		wrapped = true;
	}
	if (wrapped)
		jit_stats.hot_arena_wraps++;
	jit_stats.hot_promotions++;
	return true;
}

static inline uint32_t *jit_dispatch_code(
	uint32_t slot, const struct jit_cache_entry *entry)
{
	struct jit_hot_entry *hot;
	uint32_t admit_threshold;
	uint32_t replace_threshold;

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
static bool jit_install_code(uint32_t slot, size_t used_words,
			     bool loops_to_start)
{
	struct jit_cache_entry *entry = &jit_entries[slot];
	uint32_t units = ((uint32_t)used_words * sizeof(uint32_t) +
		JIT_CODE_ALIGNMENT - 1u) / JIT_CODE_ALIGNMENT;
	uint32_t partition_start = loops_to_start ? 0u : jit_loop_code_units;
	uint32_t partition_end = loops_to_start ? jit_loop_code_units :
		jit_code_arena_units;
	uint16_t *head = loops_to_start ? &jit_loop_code_head :
		&jit_regular_code_head;
	uint32_t start = *head;
	uint32_t *destination;

	if (units == 0u || units > partition_end - partition_start)
		return false;
	if (start < partition_start || start + units > partition_end) {
		start = partition_start;
		jit_stats.arena_wraps++;
	}

	for (uint32_t index = 0; index < JIT_CACHE_SLOTS; index++) {
		struct jit_cache_entry *resident = &jit_entries[index];

		if (index == slot || resident->state != JIT_ENTRY_READY ||
		    !jit_code_ranges_overlap(start, units, resident->code_unit,
					     resident->code_units))
			continue;
		jit_forget_entry(index);
		resident->state = JIT_ENTRY_EMPTY;
		resident->code_units = 0u;
		resident->spans_pages = 0u;
		jit_stats.replacements++;
		jit_stats.arena_evictions++;
	}

	destination = jit_code_arena + start * JIT_CODE_UNIT_WORDS;
	memset(destination, 0, units * JIT_CODE_ALIGNMENT);
	memcpy(destination, jit_staging, used_words * sizeof(uint32_t));
	*head = (uint16_t)(start + units);
	if (*head == partition_end) {
		*head = (uint16_t)partition_start;
		jit_stats.arena_wraps++;
	}
	if (!jit_publish_code(destination, used_words,
			      units * JIT_CODE_ALIGNMENT))
		return false;

	entry->code_unit = (uint16_t)start;
	entry->code_units = (uint8_t)units;
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
	default:
		return JIT_INSN_UNSUPPORTED;
	}
}

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

/* Find adjacent accesses whose effective addresses are all base + immediate.
 * A single page/alignment guard can cover the whole group, but only while no
 * load overwrites the shared base register. */
static struct jit_memory_group jit_find_memory_group(
	const uint32_t *instructions, const enum jit_instruction_kind *kinds,
	const bool *leaders, uint32_t total, uint32_t first, bool store)
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
	const bool *leaders, uint32_t total)
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
					if (offset < candidate.memory.min_offset)
						candidate.memory.min_offset = offset;
					if (end > candidate.memory.max_end)
						candidate.memory.max_end = end;
					candidate.memory.alignment_mask |= width - 1u;
					candidate.memory.count++;
					candidate.total_bytes += width;
					candidate.last = i;
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
	struct jit_loop_read_cache caches[JIT_MAX_LOOP_READ_CACHES])
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
				instructions, kinds, leaders, count, i, false);
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
	bool persistent_guard_hoisted)
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
				instructions, kinds, leaders, count, i, store);
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
			instructions, kinds, leaders, count, i, true);
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

	memset(map, 0, sizeof(*map));
	memset(map->host_for_guest, -1, sizeof(map->host_for_guest));
	map->host_for_guest[0] = RV_X0;

	for (uint32_t i = 0; i < count; i++)
		jit_note_instruction_registers(use, instructions[i], kinds[i]);

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
			/* Every mapping costs its initial guest load and optional commit.
			 * rv32_native_jit_enter already saves every ABI-preserved register once
			 * around the complete linked chain, regardless of how many regions map
			 * them, so charging another save/restore here incorrectly discourages
			 * useful mappings in short regions.  Always load the old guest value: a
			 * checked memory operation may fault before a nominal first write reaches
			 * this register. */
			mapped_cost = 1 + !!use[reg].writes;
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
		map->needs_initial[map->count] = 1u;
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
		    !jit_defer_memory_guard(builder, memory_stub, shared_miss,
					     1u, RV_T0, RV_T2) ||
		    !jit_patch_branch(resume, stub->continuation, 0u,
				      RV_X0, RV_X0))
			return false;
	}
	return !builder->failed;
}

static bool jit_emit_deferred_memory_stubs(
	struct jit_builder *builder, const struct jit_register_map *map)
{
	for (uint32_t index = 0; index < builder->memory_stub_count; index++) {
		const struct jit_memory_stub *stub = &builder->memory_stubs[index];
		uint32_t *slow_path = builder->cursor;

		for (uint32_t guard = 0; guard < stub->guard_count; guard++) {
			const struct jit_deferred_guard *patch =
				&stub->guards[guard];

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
		    !jit_defer_memory_guard(builder, memory_stub, shared_miss,
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
	if (!jit_defer_memory_guard(builder, stub, page_miss_patch,
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
	if (!jit_defer_memory_guard(builder, stub, page_miss_patch,
				    1u, RV_T0, RV_T2) ||
	    (alignment_miss_patch != NULL &&
	     !jit_defer_memory_guard(builder, stub, alignment_miss_patch,
				     1u, RV_T0, RV_X0)))
		return false;
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
	    !jit_defer_memory_guard(builder, stub, guards.page_miss,
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
	if (!jit_defer_memory_guard(builder, stub, guards.range_miss,
				    1u, RV_T0, RV_T2) ||
	    (guards.alignment_miss != NULL &&
	     !jit_defer_memory_guard(builder, stub, guards.alignment_miss,
				     1u, RV_T2, RV_X0)) ||
	    !jit_defer_memory_guard(builder, stub, guards.page_miss,
				    1u, RV_T0, RV_T2))
		return false;
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
	    !jit_defer_memory_guard(builder, stub, guards.page_miss,
				    1u, RV_T0, RV_T2))
		return false;
	/* Guard misses branch away while a4 still holds the guest base.  The fast
	 * path retains the corresponding host base across the selected accesses. */
	jit_emit(builder, rv_r(0x33u, RV_A4, 0u, RV_A4, RV_A5, 0u));
	return !builder->failed;
}

static bool jit_emit_persistent_memory_access(
	struct jit_builder *builder, const struct jit_register_map *map,
	struct jit_cache_entry *entry,
	struct rv32_native_jit_memory_site *memory_sites,
	const struct jit_persistent_memory_group *group, uint32_t index,
	uint32_t instruction, uint32_t guest_pc, uint32_t executed_before,
	bool guard_hoisted)
{
	uint32_t funct3 = (instruction >> 12) & 7u;

	if (index == group->first && !guard_hoisted &&
	    !jit_emit_persistent_memory_guard(
		    builder, map, entry, memory_sites, group, instruction,
		    guest_pc, executed_before))
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
		 * translated page, so reporting it after the first store covers all
		 * later stores without per-pixel dirty-range work. */
		if (index == group->first &&
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

static uint32_t jit_hash(uint32_t physical_offset, uint32_t guest_pc,
			 uint32_t mask)
{
	uint32_t key = (physical_offset >> 2) ^ (physical_offset >> 9) ^
		(guest_pc >> 12) ^ (guest_pc >> 20);

	return key & mask;
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
}

/* Remember only mappings that have already passed MiniRV32's ordinary
 * ACCESS_EXEC translation.  A cross-page trace may consume such a mapping,
 * but it never guesses physical contiguity or performs its own page walk. */
static void jit_cache_exec_page(uint32_t physical_offset, uint32_t guest_pc)
{
	uint32_t page_offset = guest_pc & 0xfffu;
	uint32_t physical_page_offset;
	uint32_t virtual_page = guest_pc >> 12;
	struct jit_exec_page *page;

	if (physical_offset < page_offset)
		return;
	physical_page_offset = physical_offset - page_offset;
	page = &jit_exec_pages[jit_exec_page_index(guest_pc)];
	/* The emulator and its native dispatcher own this cache on one hart. Avoid
	 * a release fence and two stores when the overwhelmingly common mapping is
	 * already current; no CPU1 worker reads or writes these entries. */
	if (page->virtual_page == virtual_page &&
	    page->physical_page_offset == physical_page_offset)
		return;
	page->physical_page_offset = physical_page_offset;
	page->virtual_page = virtual_page;
}

static bool jit_resolve_exec_offset(uint32_t guest_pc,
				    uint32_t *physical_offset)
{
	const struct jit_exec_page *page =
		&jit_exec_pages[jit_exec_page_index(guest_pc)];

	if (page->virtual_page != guest_pc >> 12)
		return false;
	*physical_offset = page->physical_page_offset + (guest_pc & 0xfffu);
	return true;
}

/* Same-page regions remain valid when a TLB generation or permission context
 * changes because their lookup still includes the freshly translated physical
 * start address.  Multi-page regions also depend on cached target mappings and
 * must be discarded at that boundary. */
static void jit_invalidate_cross_page_entries(void)
{
	/* Direct successor edges bypass the ordinary physical-key lookup. Any TLB
	 * generation or page invalidation therefore discards all predictions, even
	 * though same-page generated regions themselves remain reusable. */
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
		if ((cursor_pc & ~0xfffu) == start_guest_page)
			cursor_offset = start_physical_page + (cursor_pc & 0xfffu);
		else if (!jit_cross_page_trace_self_test ||
			 !jit_resolve_exec_offset(cursor_pc, &cursor_offset))
			break;
		memcpy(&instruction,
		       guest_ram + cursor_offset,
		       sizeof(instruction));
		kind = jit_instruction_kind(instruction);
		if (kind == JIT_INSN_UNSUPPORTED)
			break;
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
	return count >= JIT_MIN_GUEST_INSTRUCTIONS;
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

static __attribute__((noinline)) bool jit_compile(
	uint32_t slot, const uint8_t *guest_ram, uint32_t physical_offset,
	uint32_t guest_pc, uint32_t instruction_limit)
{
	struct jit_cache_entry *entry = &jit_entries[slot];
	struct rv32_native_jit_memory_site *memory_sites =
		jit_memory_sites[slot];
	uint32_t *instructions = jit_compile_workspace.instructions;
	uint32_t *guest_pcs = jit_compile_workspace.guest_pcs;
	enum jit_instruction_kind *kinds = jit_compile_workspace.kinds;
	bool *leaders = jit_compile_workspace.leaders;
	uint32_t **native_labels = jit_compile_workspace.native_labels;
	struct jit_internal_branch_patch *internal_branch_patches =
		jit_compile_workspace.internal_branch_patches;
	struct jit_register_map map;
	struct jit_persistent_memory_group persistent_memory;
	struct jit_loop_read_cache loop_read_caches[JIT_MAX_LOOP_READ_CACHES];
	struct jit_loop_write_cache loop_write_cache = { 0 };
	struct jit_builder builder = {
		.start = jit_staging,
		.cursor = jit_staging,
		.end = jit_staging + JIT_CODE_WORDS,
		.memory_stubs = jit_compile_workspace.memory_stubs,
		.loop_read_cache_stubs =
			jit_compile_workspace.loop_read_cache_stubs,
		.loop_write_cache_stubs =
			jit_compile_workspace.loop_write_cache_stubs,
	};
	uint32_t count;
	uint32_t loop_read_cache_count = 0u;
	uint32_t internal_branch_patch_count = 0u;
	uint32_t traced_jump_count = 0u;
	uint32_t segment_start = 0u;
	uint32_t loop_target_index = UINT32_MAX;
	uint32_t *loop_start = NULL;
	uint32_t *normal_result_start;
	bool terminal = false;
	bool loops_to_start = false;
	bool repeated = false;
	bool hoist_persistent_memory = false;
	bool chain_registers_clobbered = false;
	bool dynamic_counter_active = false;
	bool spans_pages = false;
	enum {
		JIT_REJECT_COLLECT,
		JIT_REJECT_CODE_SIZE,
		JIT_REJECT_CODEGEN,
		JIT_REJECT_PUBLISH,
	} rejection = JIT_REJECT_COLLECT;

	jit_forget_entry(slot);
	entry->memory_site_count = 0;
	memset(leaders, 0, sizeof(jit_compile_workspace.leaders));
	memset(native_labels, 0, sizeof(jit_compile_workspace.native_labels));
	if (!jit_collect_region(guest_ram, physical_offset, guest_pc,
				instruction_limit, instructions, guest_pcs,
				kinds, &count))
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
	persistent_memory = jit_find_persistent_memory_group(
		instructions, kinds, leaders, count);
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
			loop_read_caches);
		loop_write_cache = jit_find_loop_write_cache(
			instructions, kinds, leaders, count, &persistent_memory,
			hoist_persistent_memory);
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
	memset(jit_staging, 0, JIT_CODE_BYTES);

	/* A normal wrapper entry supplies runtime in a0. Native successors retain
	 * t6/t5 plus the cumulative instruction state in s10/s11 and enter after
	 * these four fixed setup instructions. */
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
	if ((size_t)(builder.cursor - builder.start) * sizeof(uint32_t) !=
	    JIT_NATIVE_CHAIN_ENTRY_BYTES)
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
	if (loops_to_start) {
		jit_emit_load_host(&builder, JIT_LOOP_REGISTER,
				   JIT_RUNTIME_REGISTER,
				   offsetof(struct rv32_native_jit_runtime,
					    loop_iterations));
	}
	if (dynamic_counter_active)
		jit_emit(&builder, rv_i(0x13u, JIT_EXECUTED_REGISTER,
				       0u, RV_X0, 0));
	if (hoist_persistent_memory &&
	    !jit_emit_persistent_memory_guard(
		    &builder, &map, entry, memory_sites, &persistent_memory,
		    instructions[0], guest_pc, 0u))
		goto rejected;
	loop_start = builder.cursor;

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
				    hoist_persistent_memory))
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
				instructions, kinds, leaders, count, i, false);
			const struct jit_loop_read_cache *loop_cache =
				jit_loop_read_cache_for(loop_read_caches,
							loop_read_cache_count, i);

			if (hoist_persistent_memory || loop_write_cache.valid ||
			    (persistent_memory.valid &&
			    i >= persistent_memory.first &&
			    i <= persistent_memory.last))
				group.count = 1u;
			if (group.count > 1u) {
				okay = jit_emit_load_group(
					&builder, &map, entry, memory_sites,
					&instructions[i], instruction_pc,
					executed_before, &group);
				i += group.count - 1u;
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
				instructions, kinds, leaders, count, i, true);
			const struct jit_loop_write_cache *loop_cache =
				jit_loop_write_cache_for(&loop_write_cache, i);

			if (hoist_persistent_memory || loop_write_cache.valid ||
			    (persistent_memory.valid &&
			    i >= persistent_memory.first &&
			    i <= persistent_memory.last))
				group.count = 1u;
			if (group.count > 1u) {
				okay = jit_emit_store_group(
					&builder, &map, entry, memory_sites,
					&instructions[i], instruction_pc,
					executed_before, &group);
				i += group.count - 1u;
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
	/* Generated blocks return zero on their normal path.  A memory-miss stub
	 * returns site + 1 so the checked resolver can complete that operation. */
	jit_emit_constant(&builder, RV_A0, 0u);
	uint32_t completed_register;
	if (dynamic_counter_active) {
		completed_register = JIT_EXECUTED_REGISTER;
	} else {
		jit_emit_constant(&builder, RV_T0, count);
		completed_register = RV_T0;
	}
	/* Keep the cumulative count register-resident across ordinary native links.
	 * A region using the second loop-read cache has borrowed s10/s11, so update
	 * its saved count and restore both chain registers before the common tail. */
	if (chain_registers_clobbered) {
		jit_emit_load_host(&builder, RV_T1, JIT_RUNTIME_REGISTER,
				   offsetof(struct rv32_native_jit_runtime,
					    chain_executed));
		jit_emit(&builder, rv_r(0x33u, RV_T1, 0u, RV_T1,
				       completed_register, 0u));
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER, RV_T1,
				    offsetof(struct rv32_native_jit_runtime,
					     chain_executed));
		jit_emit(&builder, rv_i(0x13u, JIT_CHAIN_EXECUTED_REGISTER,
				       0u, RV_T1, 0));
		jit_emit_load_host(&builder, JIT_CHAIN_LIMIT_REGISTER,
				   JIT_RUNTIME_REGISTER,
				   offsetof(struct rv32_native_jit_runtime,
					    chain_instruction_limit));
	} else {
		jit_emit(&builder, rv_r(0x33u, JIT_CHAIN_EXECUTED_REGISTER, 0u,
				       JIT_CHAIN_EXECUTED_REGISTER,
				       completed_register, 0u));
	}
	if (builder.loop_read_cache_stub_count != 0u ||
	    builder.loop_write_cache_stub_count != 0u ||
	    builder.memory_stub_count != 0u ||
	    builder.dirty_call_patch_count != 0u) {
		uint32_t *normal_stub_skip = builder.cursor;

		jit_emit(&builder, 0u);
		if (!jit_emit_loop_read_cache_stubs(&builder) ||
		    !jit_emit_loop_write_cache_stubs(&builder) ||
		    !jit_emit_deferred_memory_stubs(&builder, &map) ||
		    !jit_emit_linear_dirty_helper(&builder) ||
		    !jit_patch_unconditional(normal_stub_skip, builder.cursor))
			goto rejected;
	}

	/* Every cold memory stub jumps here after saving its site and any completed
	 * region segments.  Commit mapped registers before returning to C. */
	for (uint32_t i = 0; i < builder.bailout_patch_count; i++) {
		if (!jit_patch_unconditional(builder.bailout_patches[i],
					     builder.cursor))
			goto rejected;
	}

	for (uint32_t i = 0; i < map.count; i++) {
		if (map.dirty[i])
			jit_emit_store_host(&builder, JIT_GUEST_REGS_REGISTER,
					    map.host_register[i],
					    map.guest_for_host[i] *
					    sizeof(uint32_t));
	}
	/* A stable normal successor can tail-jump without publishing a C result.
	 * Guest-register commits make this safe even when the two regions chose
	 * different local register maps; the shared wrapper frame remains. */
	{
		uint32_t *bailout_return_patch = builder.cursor;
		uint32_t *empty_link_patch;
		uint32_t *pc_miss_patch;
		uint32_t *budget_miss_patch;
		uint32_t *not_loop_patch;
		uint32_t *not_loop_target;
		uint32_t *normal_return;
		uint32_t *result_return;

		jit_emit(&builder, 0u); /* bne bailout, zero, return */
		jit_emit_constant(&builder, RV_T0,
				  (uint32_t)(uintptr_t)&jit_links[slot]);
		jit_emit_load_host(&builder, RV_T2, RV_T0,
				   offsetof(struct jit_link_entry, packed_target));
		empty_link_patch = builder.cursor;
		jit_emit(&builder, 0u); /* beq packed target, zero, return */
		jit_emit_load_host(&builder, RV_T1, RV_T0,
				   offsetof(struct jit_link_entry,
					    expected_next_pc));
		pc_miss_patch = builder.cursor;
		jit_emit(&builder, 0u); /* bne actual PC, predicted PC, return */

		/* The low aligned pointer bits carry the target's conservative maximum
		 * instruction count. Do not cross MiniRV32's service boundary. */
		jit_emit(&builder, rv_i(0x13u, RV_T0, 7u, RV_T2,
				       JIT_LINK_TARGET_COUNT_MASK));
		jit_emit(&builder, rv_r(0x33u, RV_T0, 0u, RV_T0,
				       JIT_CHAIN_EXECUTED_REGISTER, 0u));
		budget_miss_patch = builder.cursor;
		jit_emit(&builder, 0u); /* bltu limit, required, return */

		/* A linked loop target derives its native iteration count from the same
		 * remaining instruction budget the C dispatcher would have supplied. */
		jit_emit(&builder, rv_i(0x13u, RV_T0, 7u, RV_T2,
				       JIT_LINK_TARGET_LOOP_FLAG));
		not_loop_patch = builder.cursor;
		jit_emit(&builder, 0u); /* beq loop flag, zero, skip setup */
		jit_emit(&builder, rv_r(0x33u, RV_T1, 0u,
				       JIT_CHAIN_LIMIT_REGISTER,
				       JIT_CHAIN_EXECUTED_REGISTER, 0x20u));
		jit_emit(&builder, rv_i(0x13u, RV_T0, 7u, RV_T2,
				       JIT_LINK_TARGET_COUNT_MASK));
		jit_emit(&builder, rv_r(0x33u, RV_T1, 5u, RV_T1, RV_T0, 1u));
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER, RV_T1,
				    offsetof(struct rv32_native_jit_runtime,
					     loop_iterations));
		not_loop_target = builder.cursor;

#if RV32_NATIVE_JIT_RUNTIME_COUNTERS
		jit_emit_load_host(&builder, RV_T0, JIT_RUNTIME_REGISTER,
				   offsetof(struct rv32_native_jit_runtime, chain_links));
		jit_emit(&builder, rv_i(0x13u, RV_T0, 0u, RV_T0, 1));
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER, RV_T0,
				    offsetof(struct rv32_native_jit_runtime, chain_links));
#endif
		jit_emit(&builder, rv_i(0x13u, RV_T2, 7u, RV_T2,
				       -(int32_t)JIT_CODE_ALIGNMENT));
		jit_emit(&builder, rv_i(0x13u, RV_T2, 0u, RV_T2,
				       JIT_NATIVE_CHAIN_ENTRY_BYTES));
		jit_emit(&builder, rv_i(0x67u, RV_X0, 0u, RV_T2, 0));

		normal_return = builder.cursor;
		/* These fields are consumed only after leaving the native chain. Keep
		 * them out of the successful link path, which often executes once per
		 * short region. A cold stub has already seeded its own local count and
		 * the checked resolver supplies its PC, so it skips these two stores. */
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER, RV_A1,
				    JIT_RESULT_OFFSET(next_pc));
		if (!dynamic_counter_active)
			jit_emit_constant(&builder, RV_T0, count);
		jit_emit_store_host(&builder, JIT_RUNTIME_REGISTER,
				    dynamic_counter_active ?
					    JIT_EXECUTED_REGISTER : RV_T0,
				    JIT_RESULT_OFFSET(executed));
		result_return = builder.cursor;
		jit_emit_constant(&builder, RV_T0, slot);
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
		if (!jit_patch_branch(bailout_return_patch, result_return,
				      1u, RV_A0, RV_X0) ||
		    !jit_patch_branch(empty_link_patch, normal_return,
				      0u, RV_T2, RV_X0) ||
		    !jit_patch_branch(pc_miss_patch, normal_return,
				      1u, RV_A1, RV_T1) ||
		    !jit_patch_branch(budget_miss_patch, normal_return,
				      6u, JIT_CHAIN_LIMIT_REGISTER, RV_T0) ||
		    !jit_patch_branch(not_loop_patch, not_loop_target,
				      0u, RV_T0, RV_X0))
			goto rejected;
	}

	if (builder.failed || builder.cursor > builder.end) {
		rejection = JIT_REJECT_CODE_SIZE;
		goto rejected;
	}
	rejection = JIT_REJECT_PUBLISH;
	if (!jit_install_code(slot,
			      (size_t)(builder.cursor - builder.start),
			      loops_to_start))
		goto rejected;

	entry->physical_offset = physical_offset;
	entry->guest_pc = guest_pc;
	entry->guest_instruction_count = (uint8_t)count;
	entry->loops_to_start = loops_to_start;
	entry->spans_pages = spans_pages;
	entry->state = JIT_ENTRY_READY;
	jit_has_cross_page_entries |= spans_pages;
	jit_stats.compiled_blocks++;
	jit_stats.compiled_guest_instructions += count;
	jit_stats.compiled_native_words +=
		(uint32_t)(builder.cursor - builder.start);
	jit_stats.traced_jumps += traced_jump_count;
	jit_stats.traced_regions += traced_jump_count != 0u;
	jit_stats.cross_page_regions += spans_pages;
	if (count > jit_stats.max_compiled_guest_instructions)
		jit_stats.max_compiled_guest_instructions = count;
	if ((uint32_t)(builder.cursor - builder.start) >
	    jit_stats.max_compiled_native_words)
		jit_stats.max_compiled_native_words =
			(uint32_t)(builder.cursor - builder.start);
	if (traced_jump_count > jit_stats.max_traced_jumps)
		jit_stats.max_traced_jumps = traced_jump_count;
	return true;

	rejected:
	if (builder.failed)
		rejection = JIT_REJECT_CODE_SIZE;
	entry->physical_offset = physical_offset;
	entry->guest_pc = guest_pc;
	entry->guest_instruction_count = 0u;
	entry->memory_site_count = 0u;
	entry->loops_to_start = 0u;
	entry->code_units = 0u;
	entry->spans_pages = 0u;
	entry->state = JIT_ENTRY_REJECTED;
	jit_stats.rejected_blocks++;
	switch (rejection) {
	case JIT_REJECT_COLLECT:
		jit_stats.rejected_collect++;
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
	return false;
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

/* Resolve one cold memory exit after generated code has committed its mapped
 * guest registers.  Successful accesses are completed exactly once here and
 * seed the corresponding translated-page cache.  Faulting or MMIO accesses
 * retain the callback's architectural exit information for the interpreter. */
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
	if (site->store) {
		failed = runtime->store(runtime, runtime->memory_address,
					runtime->memory_value, site->descriptor);
	} else {
		failed = runtime->load(runtime, runtime->memory_address,
				       site->descriptor);
	}
	if (failed != 0u)
		return;

	if (!site->store && site->destination != 0u)
		runtime->regs[site->destination] = runtime->memory_value;
	runtime->result.next_pc = site->guest_pc + 4u;
	runtime->result.executed += (uint32_t)site->executed_before + 1u;
}

static __attribute__((noinline)) bool jit_self_test_chain_results(void)
{
	struct rv32_native_jit_runtime *runtime = &jit_runtime;
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
				.sites = jit_memory_sites[2],
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
	/* Empty prediction, wrong PC and insufficient successor budget must each
	 * publish the source's results. Also exhaust the budget after one link. */
	for (uint32_t stop = 0u; stop < 4u; stop++) {
		uint32_t final_slot = stop == 3u ? 1u : 0u;
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
		jit_links[0].packed_target = stop == 0u ? 0u :
			jit_pack_link_target(1u, &jit_entries[1]);
		jit_links[0].expected_next_pc =
			base_pc + 0x100u + (stop == 1u ? 4u : 0u);
		if (rv32_native_jit_enter(block, runtime) != 0u ||
		    runtime->result.executed != 3u ||
		    runtime->result.next_pc != base_pc + (final_slot + 1u) * 0x100u ||
		    runtime->result.native_slot != final_slot ||
		    runtime->result.native_successor != 0u ||
		    runtime->chain_executed != 13u + (final_slot + 1u) * 3u ||
		    regs[1] != 3u || regs[2] != final_slot * 12u || regs[3] != 0u)
			return false;
	}
	jit_clear_all_links();
	return true;
}

static __attribute__((noinline)) bool jit_self_test_interior_loops(void)
{
	struct rv32_native_jit_runtime *runtime = &jit_runtime;
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
			.sites = jit_memory_sites[0],
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
		struct rv32_native_jit_stats stats;

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

static bool jit_self_test(void)
{
	/* Initialization is single-threaded and clears jit_runtime before return.
	 * Reuse it here instead of placing its page caches (over 2.5 KiB) on the
	 * 4 KiB main-task stack underneath jit_compile's own bounded frame. */
#define runtime jit_runtime
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
			runtime.sites = jit_memory_sites[slot];
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
		runtime.sites = jit_memory_sites[0];
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
		runtime.sites = jit_memory_sites[0];
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
		runtime.sites = jit_memory_sites[0];
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
		runtime.sites = jit_memory_sites[0];
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
		runtime.sites = jit_memory_sites[0];
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
		runtime.sites = jit_memory_sites[0];
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
		runtime.sites = jit_memory_sites[0];
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
		if (jit_test_load(&jit_runtime, memory.address, 2u) != 0u)
			return false;
		jit_runtime.memory_calls = 0u;
		result = rv32_native_jit_try(offsets[0], pcs[0], 8u, 0u);
		if (result == NULL || result->trap != 0u ||
		    result->executed != 8u || result->next_pc != 0x9024u ||
		    result->native_successor != 1u || regs[1] != 1u ||
		    regs[2] != 7u || regs[3] != 9u ||
		    regs[6] != memory.value || jit_runtime.memory_calls != 0u)
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
		    jit_runtime.chain_links !=
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
			jit_runtime.read_pages[index].virtual_page = UINT32_MAX;
		memset(regs, 0, sizeof(regs));
		regs[4] = memory.address;
		jit_runtime.memory_calls = 0u;
		result = rv32_native_jit_try(offsets[0], pcs[0], 8u, 0u);
		if (result == NULL || result->trap != 0u ||
		    result->executed != 5u || result->next_pc != 0x9014u ||
		    jit_runtime.chain_links !=
			    (RV32_NATIVE_JIT_RUNTIME_COUNTERS ? 1u : 0u) ||
		    regs[1] != 1u ||
		    regs[2] != 2u || regs[3] != 3u ||
		    regs[6] != memory.value || jit_runtime.memory_calls != 1u)
			return false;

		/* A translation-context boundary must remove the shortcut while leaving
		 * the same-page generated regions themselves reusable. */
		rv32_native_jit_memory_cache_flush();
		memset(regs, 0, sizeof(regs));
		regs[4] = memory.address;
		jit_runtime.memory_calls = 0u;
		result = rv32_native_jit_try(offsets[0], pcs[0], 8u, 0u);
		if (result == NULL || result->trap != 0u ||
		    result->executed != 5u || result->next_pc != 0x9014u ||
		    jit_runtime.chain_links != 0u || regs[1] != 1u ||
		    regs[2] != 2u || regs[3] != 3u ||
		    regs[6] != memory.value || jit_runtime.memory_calls != 1u)
			return false;
		memset(regs, 0, sizeof(regs));
		regs[4] = memory.address;
		jit_runtime.memory_calls = 0u;
		result = rv32_native_jit_try(offsets[0], pcs[0], 8u, 0u);
		if (result == NULL || result->trap != 0u ||
		    result->executed != 8u || result->next_pc != 0x9024u ||
		    jit_runtime.chain_links != 0u || regs[1] != 1u ||
		    regs[2] != 7u || regs[3] != 9u ||
		    regs[6] != memory.value || jit_runtime.memory_calls != 0u)
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
		if (rv32_native_jit_enter(block, &jit_runtime) != 0u ||
		    jit_runtime.result.trap != 0u ||
		    jit_runtime.result.executed != 8u ||
		    jit_runtime.result.next_pc != 0xa014u || regs[1] != 5u ||
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

			jit_runtime.read_pages[target_index].virtual_page =
				pcs[1] >> 12;
			jit_runtime.write_pages[target_index].virtual_page =
				pcs[1] >> 12;
			jit_runtime.read_pages[retained_index].virtual_page =
				retained_pc >> 12;
			jit_runtime.write_pages[retained_index].virtual_page =
				retained_pc >> 12;
			jit_cache_exec_page(offsets[1], pcs[1]);
			jit_cache_exec_page(32u, retained_pc);
			rv32_native_jit_memory_cache_flush_page(pcs[1] + 0x123u);
			if (jit_runtime.read_pages[target_index].virtual_page !=
				    UINT32_MAX ||
			    jit_runtime.write_pages[target_index].virtual_page !=
				    UINT32_MAX ||
			    jit_resolve_exec_offset(pcs[1], &resolved) ||
			    jit_runtime.read_pages[retained_index].virtual_page !=
				    retained_pc >> 12 ||
			    jit_runtime.write_pages[retained_index].virtual_page !=
				    retained_pc >> 12 ||
			    !jit_resolve_exec_offset(retained_pc, &resolved) ||
			    resolved != 32u)
				return false;
		}
	}

	/* Exercise the public hot-dispatch path with three blocks that deliberately
	 * map to one cache set.  The first two must coexist, and touching the first
	 * must make the second (not the first) the victim for the third. */
	{
		uint32_t dispatch_program[4] = {
			rv_i(0x13u, 1u, 0u, 1u, 1), /* addi x1, x1, 1 */
			rv_i(0x13u, 2u, 0u, 2u, 2), /* addi x2, x2, 2 */
			rv_i(0x13u, 3u, 0u, 3u, 3), /* addi x3, x3, 3 */
			rv_b(0u, 0u, 0u, 8),        /* beq x0, x0, +8 */
		};
		uint32_t pcs[3] = { 0x8000u, 0u, 0u };
		uint32_t regs[32] = { 0 };
		uint32_t set = jit_hash(0u, pcs[0], JIT_CACHE_SETS - 1u);
		uint32_t pc_count = 1u;
		const struct rv32_native_jit_result *result;

		/* Derive collisions from the configured directory size instead of
		 * baking in addresses that are valid only for one hash mask. */
		for (uint32_t page = 9u; page < 0x10000u && pc_count < 3u;
		     page++) {
			uint32_t candidate_pc = page << 12;

			if (jit_hash(0u, candidate_pc,
				     JIT_CACHE_SETS - 1u) == set)
				pcs[pc_count++] = candidate_pc;
		}
		if (pc_count != 3u)
			return false;
		memset(jit_entries, 0, sizeof(jit_entries));
		memset(jit_warm, 0, JIT_WARM_BYTES);
		memset(jit_replace_way, 0, sizeof(jit_replace_way));
		rv32_native_jit_bind(regs, (uint8_t *)dispatch_program, NULL,
				     NULL, NULL);

		for (uint32_t block = 0; block < 2u; block++) {
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

		/* Make block zero most-recently used, then populate a third collision. */
		result = rv32_native_jit_try(0u, pcs[0], 4u, 0u);
		if (result == NULL || result->executed != 4u)
			return false;
		for (uint32_t hit = 0; hit < JIT_GENERAL_HOT_THRESHOLD; hit++) {
			if (rv32_native_jit_try(0u, pcs[2], 4u, 0u) != NULL)
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
					entry->guest_pc == pcs[2];
			}
			if (!found_first || found_second || !found_third)
				return false;
		}
	}

	memset(jit_entries, 0, sizeof(jit_entries));
	memset(jit_warm, 0, JIT_WARM_BYTES);
	memset(jit_replace_way, 0, sizeof(jit_replace_way));
	memset(jit_hot_entries, 0, sizeof(jit_hot_entries));
	jit_clear_all_links();
	memset(&jit_runtime, 0, sizeof(jit_runtime));
	memset(&jit_stats, 0, sizeof(jit_stats));
	jit_invalidate_exec_pages();
	jit_reset_code_heads();
	return true;
}

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
	size_t memory_site_bytes =
		JIT_CACHE_SLOTS * sizeof(*jit_memory_sites);
	size_t candidate_bytes =
		JIT_CACHE_SLOTS * sizeof(*jit_link_candidates);
	bool main_code_external = false;

	if (psram_get_host_jit_metadata_region(
		    &psram_metadata, &psram_metadata_bytes) &&
	    psram_metadata_bytes >=
		    memory_site_bytes + JIT_WARM_BYTES + candidate_bytes) {
		jit_memory_sites = psram_metadata;
		jit_warm = (struct jit_warm_entry *)
			((uint8_t *)psram_metadata + memory_site_bytes);
		jit_link_candidates = (struct jit_link_candidate *)
			((uint8_t *)jit_warm + JIT_WARM_BYTES);
		memset(jit_memory_sites, 0, memory_site_bytes);
		memset(jit_warm, 0, JIT_WARM_BYTES);
		memset(jit_link_candidates, 0, candidate_bytes);
	} else {
		jit_memory_sites = heap_caps_calloc(
			JIT_CACHE_SLOTS, sizeof(*jit_memory_sites),
			MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
		jit_warm = heap_caps_calloc(
			JIT_WARM_SLOTS, sizeof(*jit_warm),
			MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
		jit_link_candidates = heap_caps_calloc(
			JIT_CACHE_SLOTS, sizeof(*jit_link_candidates),
			MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	}
	if (jit_memory_sites == NULL || jit_warm == NULL ||
	    jit_link_candidates == NULL)
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
	jit_memory_sites = jit_memory_sites_storage;
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
	jit_hot_code_arena_units =
		(uint16_t)(hot_arena_bytes / JIT_CODE_ALIGNMENT);
	jit_clear_all_links();
	memset(jit_hot_entries, 0, sizeof(jit_hot_entries));
	jit_reset_code_heads();
	jit_invalidate_exec_pages();
	return jit_self_test_chain_results() &&
		jit_self_test_interior_loops() && jit_self_test();
}

static void jit_invalidate_memory_pages(void)
{
	for (uint32_t i = 0; i < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; i++) {
		jit_runtime.read_pages[i].virtual_page = UINT32_MAX;
		jit_runtime.write_pages[i].virtual_page = UINT32_MAX;
		jit_runtime.write_pages[i].linear_page_start = UINT32_MAX;
		jit_runtime.write_pages[i].linear_page_end = 0u;
	}
}

void rv32_native_jit_bind(
	uint32_t *guest_regs, uint8_t *guest_ram, void *opaque,
	uint32_t (*load)(struct rv32_native_jit_runtime *runtime,
			 uint32_t virtual_address, uint32_t descriptor),
	uint32_t (*store)(struct rv32_native_jit_runtime *runtime,
			  uint32_t virtual_address, uint32_t value,
			  uint32_t descriptor))
{
	memset(&jit_runtime, 0, sizeof(jit_runtime));
	jit_runtime.regs = guest_regs;
	jit_runtime.image = guest_ram;
	jit_runtime.opaque = opaque;
	jit_runtime.load = load;
	jit_runtime.store = store;
	jit_runtime.memory_context = UINT32_MAX;
	jit_clear_all_links();
	jit_invalidate_memory_pages();
	jit_invalidate_exec_pages();
}

static inline struct jit_cache_entry *jit_lookup_entry(
	uint32_t physical_offset, uint32_t guest_pc, uint32_t *slot,
	uint32_t *set)
{
	*set = jit_hash(physical_offset, guest_pc, JIT_CACHE_SETS - 1u);
	*slot = UINT32_MAX;
	for (uint32_t way = 0; way < JIT_CACHE_WAYS; way++) {
		uint32_t candidate = *set * JIT_CACHE_WAYS + way;
		struct jit_cache_entry *entry = &jit_entries[candidate];

		if (entry->state == JIT_ENTRY_EMPTY ||
		    entry->physical_offset != physical_offset ||
		    entry->guest_pc != guest_pc)
			continue;
		*slot = candidate;
		jit_replace_way[*set] = (uint8_t)(way ^ 1u);
		return entry;
	}
	return NULL;
}

const struct rv32_native_jit_result *IRAM_ATTR rv32_native_jit_try(
	uint32_t physical_offset, uint32_t guest_pc,
	uint32_t instruction_limit, uint32_t memory_context)
{
	uint32_t set;
	uint32_t slot = UINT32_MAX;
	struct jit_cache_entry *entry;
	struct jit_warm_entry *warm;
	uint32_t instruction;
	bool candidate_loop;

	if (jit_code_arena == NULL || jit_runtime.regs == NULL ||
	    jit_runtime.image == NULL || instruction_limit <
		JIT_MIN_GUEST_INSTRUCTIONS)
		return NULL;
	/* Linux boot previously exposed a supervisor-only translation fault.  Keep
	 * the experiment on ordinary user code while retaining the interpreter for
	 * kernel, exception and device paths.  This also keeps boot and I/O behavior
	 * independent from the writable-code cache. */
	if (((memory_context >> 8) & 3u) != 0u)
		return NULL;
	if (__builtin_expect(jit_runtime.memory_context != memory_context, 0)) {
		jit_runtime.memory_context = memory_context;
		jit_invalidate_cross_page_entries();
		jit_invalidate_memory_pages();
		jit_invalidate_exec_pages();
	}
	jit_cache_exec_page(physical_offset, guest_pc);

#if CONFIG_RV32_HOST_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
	jit_stats.lookups++;
#endif
	entry = jit_lookup_entry(physical_offset, guest_pc, &slot, &set);
	if (entry != NULL && entry->state == JIT_ENTRY_READY) {
		uint32_t total_executed;
	#if CONFIG_RV32_HOST_PERF_STATS
		const uint32_t profile_slot = slot;
	#endif
	#if CONFIG_RV32_HOST_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
		uint32_t block_bailouts = 0u;
		uint32_t chained_lookups = 0u;
		uint32_t cross_page_chains = 0u;
#endif

		jit_runtime.dirty_epoch++;
		if (__builtin_expect(jit_runtime.dirty_epoch == 0u, 0)) {
			for (uint32_t index = 0;
			     index < RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES; index++)
				jit_runtime.write_pages[index].dirty_epoch = 0u;
			jit_runtime.dirty_epoch = 1u;
		}
		jit_runtime.result.native_successor = 0u;
		jit_runtime.result.linear_dirty_start = UINT32_MAX;
		jit_runtime.result.linear_dirty_end = 0u;
		jit_runtime.result.linear_store_calls = 0u;
		jit_runtime.result.linear_store_bytes = 0u;
		jit_runtime.memory_calls = 0u;
		jit_runtime.chain_instruction_limit = instruction_limit;
		jit_runtime.chain_executed = 0u;
		jit_runtime.chain_blocks = 0u;
		jit_runtime.chain_links = 0u;

		for (;;) {
			jit_block_fn block;
			struct jit_cache_entry *returned_entry;
			bool crossed_page;
			uint32_t bailout;
			uint32_t executed;
			uint32_t returned_slot;
			uint32_t remaining =
				instruction_limit - jit_runtime.chain_executed;
			uint32_t next_pc;

			if (entry->guest_instruction_count > remaining) {
				if (jit_runtime.chain_executed == 0u)
					return NULL;
				break;
			}
			jit_runtime.memory_value = 0u;
			jit_runtime.result.next_pc = guest_pc;
			jit_runtime.result.executed = 0u;
			jit_runtime.result.trap = 0u;
			jit_runtime.result.trap_value = 0u;
			jit_runtime.result.native_successor = 0u;
			jit_runtime.result.native_slot = UINT32_MAX;
			jit_runtime.sites = jit_memory_sites[slot];
			jit_runtime.site_count = entry->memory_site_count;
			jit_runtime.loop_iterations = 1u;
			if (entry->loops_to_start) {
				/* The MiniRV32 batch limit is already the architectural service
				 * boundary.  Spend that remaining guest-instruction budget here so
				 * a long native loop does not return through C every 32 iterations. */
				jit_runtime.loop_iterations =
					remaining / entry->guest_instruction_count;
			}
			block = (jit_block_fn)(uintptr_t)jit_dispatch_code(slot,
								      entry);
			bailout = rv32_native_jit_enter(block, &jit_runtime);
			returned_slot = jit_runtime.result.native_slot;
			if (returned_slot >= JIT_CACHE_SLOTS ||
			    jit_entries[returned_slot].state != JIT_ENTRY_READY) {
				jit_runtime.result.trap = 2u;
				jit_runtime.result.trap_value = 0u;
				break;
			}
			returned_entry = &jit_entries[returned_slot];
			/* A tail-linked target installs no C-side site pointer. Recover it
			 * from the final slot before invoking the checked slow path. */
			if (bailout != 0u) {
				jit_runtime.sites = jit_memory_sites[returned_slot];
				jit_runtime.site_count =
					returned_entry->memory_site_count;
				jit_resolve_memory_bailout(&jit_runtime, bailout);
				jit_runtime.chain_executed +=
					jit_runtime.result.executed;
			}
			executed = jit_runtime.result.executed;

	#if CONFIG_RV32_HOST_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
			block_bailouts +=
				executed < returned_entry->guest_instruction_count;
	#endif
			total_executed = jit_runtime.chain_executed;
			if (__builtin_expect(total_executed > instruction_limit, 0)) {
				jit_runtime.result.trap = 2u;
				jit_runtime.result.trap_value = 0u;
				break;
			}
			if (executed == 0u || bailout != 0u ||
			    jit_runtime.result.trap != 0u)
				break;

			jit_runtime.result.native_successor = 1u;
			if (instruction_limit - total_executed <
			    JIT_MIN_GUEST_INSTRUCTIONS)
				break;
			next_pc = jit_runtime.result.next_pc;
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
			entry = jit_lookup_entry(physical_offset, guest_pc, &slot,
						 &set);
			if (entry == NULL || entry->state != JIT_ENTRY_READY)
				break;
			jit_observe_link(returned_slot, guest_pc, slot, entry);
#if CONFIG_RV32_HOST_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
			cross_page_chains += crossed_page;
#endif
		}
		total_executed = jit_runtime.chain_executed;
	#if CONFIG_RV32_HOST_PERF_STATS
		jit_profile_guest_instructions[profile_slot] += total_executed;
	#endif
	#if CONFIG_RV32_HOST_PERF_STATS || CONFIG_RV32_EMULATOR_PERF_STATS
		jit_stats.lookups += chained_lookups + jit_runtime.chain_links;
		jit_stats.calls += jit_runtime.chain_blocks;
		jit_stats.guest_instructions += total_executed;
		jit_stats.memory_calls += jit_runtime.memory_calls;
		jit_stats.memory_bailouts += block_bailouts;
		jit_stats.cross_page_chains += cross_page_chains;
		jit_stats.native_link_transitions += jit_runtime.chain_links;
	#endif
		jit_runtime.result.executed = total_executed;
		if (total_executed == 0u && jit_runtime.result.trap == 0u)
			return NULL;
		return &jit_runtime.result;
	}
	if (entry != NULL && entry->state == JIT_ENTRY_REJECTED)
		return NULL;

	/* Do not fill the warm table with privileged or otherwise unsupported
	 * instructions.  A single physical instruction load is cheaper than
	 * repeatedly compiling and rejecting such sites. */
	memcpy(&instruction, jit_runtime.image + physical_offset,
	       sizeof(instruction));
	if (jit_instruction_kind(instruction) == JIT_INSN_UNSUPPORTED)
		return NULL;

	warm = &jit_warm[jit_hash(physical_offset, guest_pc,
				  JIT_WARM_SLOTS - 1u)];
	if (!warm->valid || warm->physical_offset != physical_offset ||
	    warm->guest_pc != guest_pc) {
		warm->physical_offset = physical_offset;
		warm->guest_pc = guest_pc;
		warm->hits = 1u;
		warm->valid = 1u;
		warm->reserved = 0u;
		return NULL;
	}
	if (warm->hits != UINT16_MAX)
		warm->hits++;
	if (warm->hits < JIT_LOOP_HOT_THRESHOLD)
		return NULL;
	if ((warm->reserved & JIT_WARM_CLASSIFIED) == 0u) {
		warm->reserved = JIT_WARM_CLASSIFIED;
		if (jit_candidate_loops_to_start(jit_runtime.image,
				physical_offset, guest_pc, instruction_limit))
			warm->reserved |= JIT_WARM_LOOPS_TO_START;
	}
	candidate_loop = (warm->reserved & JIT_WARM_LOOPS_TO_START) != 0u;
	if (!candidate_loop && warm->hits < JIT_GENERAL_HOT_THRESHOLD)
		return NULL;

	/* Two ways retain colliding hot loop heads without lengthening the common
	 * hit search beyond two compact metadata entries.  Prefer an unused way.
	 * A loop may displace probationary ordinary code, but ordinary code never
	 * displaces a set containing two loops; it requalifies after another full
	 * general-hot interval instead. */
	for (uint32_t way = 0; way < JIT_CACHE_WAYS; way++) {
		uint32_t candidate = set * JIT_CACHE_WAYS + way;

		if (jit_entries[candidate].state == JIT_ENTRY_EMPTY) {
			slot = candidate;
			break;
		}
	}
	if (slot == UINT32_MAX) {
		slot = set * JIT_CACHE_WAYS + jit_replace_way[set];
		if (jit_entries[slot].loops_to_start) {
			uint32_t other = set * JIT_CACHE_WAYS +
				((slot - set * JIT_CACHE_WAYS) ^ 1u);

			if (!jit_entries[other].loops_to_start)
				slot = other;
			else if (!candidate_loop) {
				warm->hits = 0u;
				return NULL;
			}
		}
	}
	entry = &jit_entries[slot];
	jit_replace_way[set] = (uint8_t)((slot - set * JIT_CACHE_WAYS) ^ 1u);
	if (entry->state != JIT_ENTRY_EMPTY) {
		jit_stats.replacements++;
		jit_stats.tag_evictions++;
	}
	(void)jit_compile(slot, jit_runtime.image, physical_offset, guest_pc,
			  instruction_limit);
	warm->valid = 0u;
	return NULL;
}

void rv32_native_jit_flush(void)
{
	memset(jit_entries, 0, sizeof(jit_entries));
	memset(jit_warm, 0, JIT_WARM_BYTES);
	memset(jit_replace_way, 0, sizeof(jit_replace_way));
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
	rv32_native_jit_memory_cache_flush();
}

void rv32_native_jit_memory_cache_flush(void)
{
	jit_runtime.memory_context = UINT32_MAX;
	jit_invalidate_cross_page_entries();
	jit_invalidate_memory_pages();
	jit_invalidate_exec_pages();
}

void rv32_native_jit_memory_cache_flush_page(uint32_t virtual_address)
{
	uint32_t virtual_page = virtual_address >> 12;
	uint32_t data_index =
		virtual_page & (RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u);
	uint32_t exec_index = jit_exec_page_index(virtual_address);

	/* A page-scoped SFENCE.VMA changes neither the global permission context
	 * nor unrelated translations. Evict only the colliding direct-map slots;
	 * this is conservative even when their tags belong to another page. */
	jit_runtime.read_pages[data_index].virtual_page = UINT32_MAX;
	jit_runtime.write_pages[data_index].virtual_page = UINT32_MAX;
	jit_runtime.write_pages[data_index].linear_page_start = UINT32_MAX;
	jit_runtime.write_pages[data_index].linear_page_end = 0u;
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
	stats->arena_head_bytes =
		((uint32_t)jit_loop_code_head +
		 (uint32_t)jit_regular_code_head - jit_loop_code_units) *
		JIT_CODE_ALIGNMENT;
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
