/* Instruction-count microbenchmark, not an ESP32-P4 timing/FPS model.
 * Compile against either JIT source using RV32_NATIVE_JIT_TEST_SOURCE. */
#ifndef RV32_NATIVE_JIT_TEST_SOURCE
#define RV32_NATIVE_JIT_TEST_SOURCE "../main/rv32_native_jit.c"
#endif
#include RV32_NATIVE_JIT_TEST_SOURCE

#ifndef JIT_BENCH_STORE
#define JIT_BENCH_STORE 1
#endif
#ifndef JIT_BENCH_WIDTH
#define JIT_BENCH_WIDTH 2u
#endif
#define BENCH_ITERATIONS 8192u
#define BENCH_REPEATS 128u
#define BENCH_BASE 0x20000u
static _Alignas(4) uint32_t bench_data[BENCH_ITERATIONS];

int test_main(void)
{
	uint32_t regs[32] = { 0 };
	uint32_t funct3 = JIT_BENCH_WIDTH == 2u ? (JIT_BENCH_STORE ? 1u : 5u) : 2u;
	uint32_t program[] = {
		JIT_BENCH_STORE ? rv_s(0x23u, funct3, 4u, 3u, 0) :
			rv_i(0x03u, 3u, funct3, 4u, 0),
		rv_i(0x13u, 4u, 0u, 4u, JIT_BENCH_WIDTH),
		rv_b(6u, 4u, 2u, -8),
	};
	/* Initialize the standalone arena without counting the entire startup
	 * correctness suite as benchmark work. BSS starts cleared in this harness. */
	jit_memory_sites = jit_memory_sites_storage;
	jit_warm = jit_warm_storage;
	jit_link_candidates = jit_link_candidate_storage;
	jit_code_arena = heap_caps_aligned_calloc(JIT_CODE_ALIGNMENT, 1u,
		JIT_CODE_ARENA_MIN_BYTES, MALLOC_CAP_EXEC | MALLOC_CAP_INTERNAL);
	jit_code_arena_units = JIT_CODE_ARENA_MIN_BYTES / JIT_CODE_ALIGNMENT;
	jit_loop_code_units = jit_code_arena_units / 2u;
	jit_reset_code_heads();
	if (!jit_compile(0u, (const uint8_t *)program, 0u, 0x40000u, 3u))
		return 1;
	for (uint32_t i = 0; i < BENCH_ITERATIONS; i++)
		bench_data[i] = 0x12345678u;
	jit_runtime.regs = regs;
	jit_runtime.chain_instruction_limit = BENCH_ITERATIONS * 3u;
	jit_runtime.loop_iterations = BENCH_ITERATIONS;
	jit_invalidate_memory_pages();
	for (uint32_t page = 0; page < sizeof(bench_data) / 4096u; page++) {
		uint32_t tag = (BENCH_BASE >> 12) + page;
		uint32_t index = tag & (RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u);
		jit_runtime.read_pages[index] = (struct rv32_native_jit_read_page) {
			.virtual_page = tag,
			.host_bias = (uint32_t)(uintptr_t)bench_data - BENCH_BASE,
		};
		jit_runtime.write_pages[index] = (struct rv32_native_jit_write_page) {
			.virtual_page = tag,
			.host_bias = (uint32_t)(uintptr_t)bench_data - BENCH_BASE,
			.linear_page_start = UINT32_MAX,
		};
	}
	for (uint32_t repeat = 0; repeat < BENCH_REPEATS; repeat++) {
		jit_runtime.chain_executed = 0u;
		regs[2] = BENCH_BASE + BENCH_ITERATIONS * JIT_BENCH_WIDTH;
		regs[3] = 0x12345678u;
		regs[4] = BENCH_BASE;
		if (rv32_native_jit_enter((jit_block_fn)(uintptr_t)jit_entry_code(0u),
					 &jit_runtime) != 0u ||
		    jit_runtime.chain_executed != BENCH_ITERATIONS * 3u ||
		    regs[4] != regs[2])
			return 2;
	}
	if (JIT_BENCH_STORE) {
		uint32_t value = 0u;
		memcpy(&value, (const uint8_t *)bench_data +
			(BENCH_ITERATIONS - 1u) * JIT_BENCH_WIDTH, JIT_BENCH_WIDTH);
		if (value != (JIT_BENCH_WIDTH == 2u ? 0x5678u : 0x12345678u)) return 3;
	} else if (regs[3] != (JIT_BENCH_WIDTH == 2u ? 0x1234u : 0x12345678u)) {
		return 4;
	}
	return 0;
}
