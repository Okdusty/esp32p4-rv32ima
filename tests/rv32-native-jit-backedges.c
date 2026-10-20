/* Generic loop-tail regression: execute generated RV32 instructions and
 * compare every guest register, both instruction counts and the exit PC.
 * This is not a P4 timing or cache-coherence model. */
#ifndef RV32_NATIVE_JIT_TEST_SOURCE
#define RV32_NATIVE_JIT_TEST_SOURCE "../main/rv32_native_jit.c"
#endif
#include RV32_NATIVE_JIT_TEST_SOURCE

struct backedge_values {
	uint32_t initial;
	uint32_t limit;
	int32_t step;
};

/* Kept visible to a debugger if the freestanding runner returns a failure. */
static volatile uint32_t backedge_case;
static volatile uint32_t backedge_checked;

static bool backedge_taken(uint32_t condition, uint32_t left, uint32_t right)
{
	switch (condition) {
	case 0u: return left == right;
	case 1u: return left != right;
	case 4u: return (int32_t)left < (int32_t)right;
	case 5u: return (int32_t)left >= (int32_t)right;
	case 6u: return left < right;
	case 7u: return left >= right;
	default: return true; /* JAL x0 */
	}
}

static int backedge_check(uint32_t pc, uint32_t prefix, uint32_t condition,
			  uint32_t rs1, uint32_t rs2,
			  const struct backedge_values *values, uint32_t budget)
{
	uint32_t regs[32];
	uint32_t expected[32];
	uint32_t executed = prefix;
	uint32_t next_pc;
	struct rv32_native_jit_runtime *runtime = jit_rt;

	for (uint32_t i = 0u; i < 32u; i++)
		regs[i] = expected[i] = 0x12340000u + i;
	regs[0] = expected[0] = 0u;
	regs[3] = expected[3] = values->initial;
	regs[4] = expected[4] = values->limit;
	regs[5] = expected[5] = 0u;
	if (prefix)
		expected[6] += 7u;
	for (;;) {
		expected[3] += (uint32_t)values->step;
		expected[5]++;
		executed += 3u;
		if (!backedge_taken(condition, expected[rs1], expected[rs2])) {
			next_pc = pc + (prefix + 3u) * 4u;
			break;
		}
		if (executed + 3u > budget) {
			next_pc = pc + prefix * 4u;
			break;
		}
	}
	*runtime = (struct rv32_native_jit_runtime) {
		.regs = regs,
		.loop_iterations = budget / 3u,
		.chain_executed = 13u,
		.chain_instruction_limit = 13u + budget,
		.result = { .next_pc = UINT32_MAX, .executed = UINT32_MAX,
			.native_slot = UINT32_MAX, .native_successor = UINT32_MAX },
	};
	if (rv32_native_jit_enter((jit_block_fn)(uintptr_t)jit_entry_code(0u),
				 runtime) != 0u || runtime->result.trap != 0u)
		return 2;
	if (runtime->result.executed != executed ||
	    runtime->chain_executed != 13u + executed || executed > budget)
		return 3;
	if (runtime->result.next_pc != next_pc)
		return 4;
	if (runtime->result.native_slot != 0u ||
	    runtime->result.native_successor != 0u || runtime->chain_links != 0u)
		return 5;
	for (uint32_t i = 0u; i < 32u; i++)
		if (regs[i] != expected[i])
			return 32 + (int)i;
	backedge_checked++;
	return 0;
}

int test_main(void)
{
	static const uint32_t pcs[] = { 0x100u, 0x40000u, 0xfffff800u };
	static const uint32_t conditions[] = { 0u, 1u, 4u, 5u, 6u, 7u, 8u };
	static const uint32_t operands[][2] = { { 3u, 4u }, { 3u, 3u }, { 0u, 3u } };
	static const uint32_t budget_extras[] = { 0u, 1u, 2u, 3u, 4u, 8u, 23u };
	static const struct backedge_values values[] = {
		{ 0u, 2u, 1 }, { 1u, 2u, 1 }, { 4u, 4u, 0 },
		{ UINT32_MAX, 0u, 1 }, { 0x7fffffffu, 0x80000000u, 1 },
		{ 0x80000000u, 0u, 1 }, { 1u, 0u, -1 },
		{ UINT32_MAX, UINT32_MAX - 2u, -1 },
	};
	jit_warm = jit_warm_storage;
	jit_link_candidates = jit_link_candidate_storage;
	jit_code_arena = heap_caps_aligned_calloc(JIT_CODE_ALIGNMENT, 1u,
		JIT_CODE_ARENA_MIN_BYTES, MALLOC_CAP_EXEC | MALLOC_CAP_INTERNAL);
	if (jit_code_arena == NULL)
		return 6;
	jit_code_arena_units = JIT_CODE_ARENA_MIN_BYTES / JIT_CODE_ALIGNMENT;
	jit_loop_code_units = jit_code_arena_units / 2u;
	jit_reset_code_heads();
	for (uint32_t p = 0u; p < sizeof(pcs) / sizeof(pcs[0]); p++) {
		for (uint32_t prefix = 0u; prefix <= 1u; prefix++) {
			for (uint32_t c = 0u; c < sizeof(conditions) / sizeof(conditions[0]); c++) {
				for (uint32_t o = 0u; o < sizeof(operands) / sizeof(operands[0]); o++) {
					for (uint32_t v = 0u; v < sizeof(values) / sizeof(values[0]); v++) {
						uint32_t program[4];
						uint32_t condition = conditions[c];
						uint32_t rs1 = operands[o][0];
						uint32_t rs2 = operands[o][1];
						program[0] = rv_i(0x13u, 6u, 0u, 6u, 7);
						program[prefix] = rv_i(0x13u, 3u, 0u, 3u, values[v].step);
						program[prefix + 1u] = rv_i(0x13u, 5u, 0u, 5u, 1);
						program[prefix + 2u] = condition == 8u ? rv_j(0u, -8) :
							rv_b(condition, rs1, rs2, -8);
						if (!jit_compile(0u, (const uint8_t *)program, 0u,
								 pcs[p], prefix + 3u) ||
						    jit_entries[0].loops_to_start != (prefix == 0u))
							return 1;
						for (uint32_t b = 0u; b < sizeof(budget_extras) / sizeof(budget_extras[0]); b++) {
							backedge_case++;
							int result = backedge_check(pcs[p], prefix, condition,
								rs1, rs2, &values[v], prefix + 3u + budget_extras[b]);
							if (result != 0)
								return result;
						}
					}
				}
			}
		}
	}
	return backedge_checked == 7056u ? 0 : 7;
}
