/* Code-lifetime checks execute real generated RV32 instructions. The local
 * simulator does not model P4 caches or DMA; these test directory/branch
 * lifetime, not electrical faults or hardware cache coherence. */
#ifndef RV32_NATIVE_JIT_TEST_SOURCE
#define RV32_NATIVE_JIT_TEST_SOURCE "../main/rv32_native_jit.c"
#endif
#include RV32_NATIVE_JIT_TEST_SOURCE

#define LIFETIME_PC 0x60000u
#define LIFETIME_HOT_PAIRS 256u
#define LIFETIME_COLD_CYCLES 64u

static bool lifetime_compile_loop(uint32_t slot, uint32_t pc, uint32_t value)
{
	uint32_t program[] = {
		rv_i(0x13u, 1u, 0u, 1u, (int32_t)value),
		rv_i(0x13u, 2u, 0u, 2u, -1),
		rv_b(1u, 2u, 0u, -8),
	};
	return jit_compile(slot, (const uint8_t *)program, 0u, pc, 3u);
}

static bool lifetime_train_target(void)
{
	for (uint32_t i = 0u; i < JIT_LINK_CONFIRMATIONS; i++)
		jit_observe_link(0u, LIFETIME_PC + 0x100u, 1u, &jit_entries[1]);
	/* A second real source has an unconfirmed observation of this target. */
	jit_observe_link(3u, LIFETIME_PC + 0x100u, 1u, &jit_entries[1]);
	return jit_links[0].packed_target != 0u &&
		jit_link_candidates[3].candidate_target != 0u;
}

static bool lifetime_target_points_to(const uint32_t *code)
{
	uint32_t pointer = (uint32_t)(uintptr_t)code;
	return jit_link_target_pointer(jit_links[0].packed_target) == pointer &&
		jit_links[0].expected_next_pc == LIFETIME_PC + 0x100u &&
		jit_link_target_pointer(jit_link_candidates[3].candidate_target) == pointer &&
		jit_link_candidates[3].candidate_next_pc == LIFETIME_PC + 0x100u;
}

static bool lifetime_execute(bool linked, uint32_t value)
{
	uint32_t regs[32];
	uint32_t expected[32];
	struct rv32_native_jit_runtime *runtime = &jit_runtime;

	for (uint32_t i = 0u; i < 32u; i++)
		regs[i] = expected[i] = 0x12340000u + i;
	regs[0] = expected[0] = 0u;
	regs[1] = 0u;
	expected[1] = linked ? 3u * value : 0u;
	regs[2] = 3u;
	expected[2] = linked ? 0u : 3u;
	regs[10] = 0u;
	expected[10] = 3u;
	*runtime = (struct rv32_native_jit_runtime) {
		.regs = regs,
		.loop_iterations = 3u,
		.chain_executed = 13u,
		.chain_instruction_limit = 25u,
		.result = { .next_pc = UINT32_MAX, .executed = UINT32_MAX,
			.native_slot = UINT32_MAX },
	};
	if (rv32_native_jit_enter(
		(jit_block_fn)(uintptr_t)jit_entry_code(0u), runtime) != 0u ||
	    runtime->result.trap != 0u ||
	    runtime->result.executed != (linked ? 9u : 3u) ||
	    runtime->chain_executed != (linked ? 25u : 16u) ||
	    runtime->result.next_pc != LIFETIME_PC + 0x100u + (linked ? 12u : 0u) ||
	    runtime->result.native_slot != (linked ? 1u : 0u) ||
	    runtime->chain_links != (RV32_NATIVE_JIT_RUNTIME_COUNTERS && linked ? 1u : 0u))
		return false;
	for (uint32_t i = 0u; i < 32u; i++)
		if (regs[i] != expected[i])
			return false;
	return true;
}

static int test_lifetime(void)
{
	uint32_t source[] = {
		rv_i(0x13u, 10u, 0u, 10u, 1),
		rv_i(0x13u, 10u, 0u, 10u, 2),
		rv_b(0u, 0u, 0u, 0xf8),
	};
	rv32_native_jit_flush();
	/* Only the standalone test shrinks these rings: force wrap/reuse quickly
	 * while the ordinary partition keeps both incoming sources resident. */
	jit_code_arena_units = 64u;
	jit_loop_code_units = 32u;
	jit_reset_code_heads();
	memset(&jit_stats, 0, sizeof(jit_stats));
	if (!jit_compile(0u, (const uint8_t *)source, 0u, LIFETIME_PC, 3u))
		return 11;
	source[2] = rv_b(0u, 0u, 0u, -0x108);
	if (!jit_compile(3u, (const uint8_t *)source, 0u, LIFETIME_PC + 0x200u, 3u) ||
	    !lifetime_compile_loop(1u, LIFETIME_PC + 0x100u, 7u) ||
	    !lifetime_compile_loop(2u, LIFETIME_PC + 0x300u, 19u))
		return 12;
	if (jit_entries[1].code_units == 0u ||
	    jit_entries[1].code_units != jit_entries[2].code_units)
		return 13;
	/* One region fits exactly; promoting the other must evict it. */
	jit_hot_code_arena_units = jit_entries[1].code_units;
	if (!lifetime_train_target())
		return 14;
	for (uint32_t i = 0u; i < LIFETIME_HOT_PAIRS; i++) {
		if (!jit_promote_hot_code(1u, &jit_entries[1], true) ||
		    !lifetime_target_points_to(jit_hot_entry_code(1u)) ||
		    !lifetime_execute(true, 7u))
			return 20;
		if (!jit_promote_hot_code(2u, &jit_entries[2], true) ||
		    jit_hot_entries[1].location != 0u ||
		    !lifetime_target_points_to(jit_entry_code(1u)) ||
		    !lifetime_execute(true, 7u))
			return 21;
	}
	for (uint32_t cycle = 0u; cycle < LIFETIME_COLD_CYCLES; cycle++) {
		uint32_t value = cycle + 1u;
		if (!lifetime_compile_loop(1u, LIFETIME_PC + 0x100u, value) ||
		    !lifetime_train_target() ||
		    !jit_promote_hot_code(1u, &jit_entries[1], true) ||
		    !lifetime_target_points_to(jit_hot_entry_code(1u)) ||
		    !lifetime_execute(true, value))
			return 30;
		/* Recompile one other loop until its cold bytes overwrite the target.
		 * Removing the authoritative cold copy must invalidate its hot copy,
		 * both published and training references, before any bytes are reused. */
		for (uint32_t i = 0u;
		     i <= jit_loop_code_units && jit_entries[1].state == JIT_ENTRY_READY;
		     i++) {
			if (!lifetime_compile_loop(2u, LIFETIME_PC + 0x300u, 100u + i))
				return 31;
		}
		if (jit_entries[1].state == JIT_ENTRY_READY ||
		    jit_hot_entries[1].location != 0u ||
		    jit_links[0].packed_target != 0u ||
		    jit_link_candidates[3].candidate_target != 0u ||
		    jit_entries[0].state != JIT_ENTRY_READY ||
		    !lifetime_execute(false, 0u))
			return 32;
	}
	/* Rejected replacement must also retire existing incoming pointers. */
	uint32_t unsupported[] = { 0u, 0u, 0u };
	if (!lifetime_compile_loop(1u, LIFETIME_PC + 0x100u, 9u) ||
	    !lifetime_train_target() ||
	    !jit_promote_hot_code(1u, &jit_entries[1], true) ||
	    jit_compile(1u, (const uint8_t *)unsupported, 0u, LIFETIME_PC + 0x100u, 3u) ||
	    jit_links[0].packed_target != 0u ||
	    jit_link_candidates[3].candidate_target != 0u ||
	    jit_hot_entries[1].location != 0u || !lifetime_execute(false, 0u))
		return 40;
	if (jit_stats.hot_promotions < LIFETIME_HOT_PAIRS * 2u ||
	    jit_stats.hot_evictions < LIFETIME_HOT_PAIRS ||
	    jit_stats.arena_evictions < LIFETIME_COLD_CYCLES || jit_stats.arena_wraps == 0u)
		return 41;
	rv32_native_jit_flush();
	for (uint32_t i = 0u; i < JIT_CACHE_SLOTS; i++)
		if (jit_entries[i].state != JIT_ENTRY_EMPTY ||
		    jit_hot_entries[i].location != 0u || jit_links[i].packed_target != 0u ||
		    jit_link_candidates[i].candidate_target != 0u)
			return 42;
	for (uint32_t i = 0u; i < JIT_HOT_OCCUPANCY_WORDS; i++)
		if (jit_hot_code_occupancy[i] != 0u)
			return 43;
	return 0;
}

int test_main(void)
{
	if (!rv32_native_jit_init())
		return 10;
	return test_lifetime();
}
