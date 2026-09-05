/* Exercise real generated RV32 instructions, not a C imitation of the JIT.
 * Including the implementation also exposes cold analysis/placement state to
 * assertions; none of these fixtures are compiled into production firmware. */
#ifndef RV32_NATIVE_JIT_TEST_SOURCE
#define RV32_NATIVE_JIT_TEST_SOURCE "../main/rv32_native_jit.c"
#endif
#include RV32_NATIVE_JIT_TEST_SOURCE

/* The combined experiment can still be checked against its preserved source.
 * Production currently retains the original C-dispatch admission policy. */
#ifndef RV32_NATIVE_JIT_TEST_LINK_ENDPOINT_ADMISSION
#define RV32_NATIVE_JIT_TEST_LINK_ENDPOINT_ADMISSION 0
#endif

#define TEST_PC 0x40000u
#define TEST_DATA 0x20000u
static _Alignas(4) uint8_t data[8192];
static uint8_t expected_data[8192];

static uint32_t fault_load(struct rv32_native_jit_runtime *runtime,
	uint32_t address, uint32_t descriptor)
{
	uint32_t width = jit_memory_width(descriptor & 7u, true);
	runtime->memory_calls++;
	rv32_native_jit_memory_exit(runtime, descriptor,
		(address & (width - 1u)) ? 4u : 13u, address);
	return 1u;
}

static uint32_t fault_store(struct rv32_native_jit_runtime *runtime,
	uint32_t address, uint32_t value, uint32_t descriptor)
{
	uint32_t width = jit_memory_width(descriptor & 7u, false);
	(void)value;
	runtime->memory_calls++;
	rv32_native_jit_memory_exit(runtime, descriptor,
		(address & (width - 1u)) ? 6u : 15u, address);
	return 1u;
}

#ifndef RV32_NATIVE_JIT_TEST_BASELINE
static bool test_alignment_proof(void)
{
	uint32_t program[] = {
		rv_i(0x03u, 3u, 2u, 4u, 0),
		rv_i(0x13u, 4u, 0u, 4u, 4),
		rv_b(1u, 2u, 0u, -8),
	};
	enum jit_instruction_kind kinds[] = {
		JIT_INSN_LOAD, JIT_INSN_INTEGER, JIT_INSN_BRANCH,
	};
	if (!jit_loop_preserves_alignment(program, kinds, 0u, 3u, 4u, 4u))
		return false;
	program[1] = rv_i(0x13u, 4u, 0u, 4u, -4);
	if (!jit_loop_preserves_alignment(program, kinds, 0u, 3u, 4u, 4u))
		return false;
	program[1] = rv_i(0x13u, 4u, 0u, 4u, 2);
	if (jit_loop_preserves_alignment(program, kinds, 0u, 3u, 4u, 4u) ||
	    !jit_loop_preserves_alignment(program, kinds, 0u, 3u, 4u, 2u))
		return false;
	program[1] = rv_i(0x13u, 4u, 0u, 5u, 4);
	if (jit_loop_preserves_alignment(program, kinds, 0u, 3u, 4u, 4u))
		return false;
	program[1] = rv_r(0x33u, 4u, 0u, 4u, 5u, 0u);
	if (jit_loop_preserves_alignment(program, kinds, 0u, 3u, 4u, 4u))
		return false;
	program[0] = rv_i(0x03u, 4u, 2u, 4u, 0);
	program[1] = rv_i(0x13u, 4u, 0u, 4u, 4);
	return !jit_loop_preserves_alignment(program, kinds, 0u, 3u, 4u, 4u);
}
#endif

static bool test_access(bool store, uint32_t width, uint32_t scenario)
{
	uint32_t funct3 = width == 2u ? (store ? 1u : 5u) : 2u;
	uint32_t address = TEST_DATA;
	int32_t stride = (int32_t)width;
	int32_t offset = 0;
	uint32_t pages = 3u;
	uint32_t budget = 12u;
	uint32_t regs[32] = { 0 };
	uint32_t expected_regs[32];
	uint32_t program[4];
	uint32_t completed = 0u;
	uint32_t trap = 0u;
	uint32_t trap_address = 0u;
	uint32_t dirty_start = UINT32_MAX;
	uint32_t dirty_end = 0u;
	struct rv32_native_jit_runtime *runtime = &jit_runtime;

	switch (scenario) {
	case 1: address++; break;                 /* initially misaligned */
	case 2: stride = 1; break;                /* warm tag, now misaligned */
	case 3: address += 4096u - width; pages = 1u; break; /* missing next page */
	case 4: address += 4096u - width; break;   /* native shared-cache refill */
	case 5: address += 4096u + width; stride = -(int32_t)width; break;
	case 6: address++; offset = (int32_t)width - 1; break; /* odd base, aligned EA */
	case 7: budget = 4u; break;               /* service boundary */
	case 8: pages = 0u; break;                /* no validated mapping / MMIO */
	default: break;
	}
	program[0] = store ? rv_s(0x23u, funct3, 4u, 3u, offset) :
		rv_i(0x03u, 3u, funct3, 4u, offset);
	program[1] = rv_i(0x13u, 4u, 0u, 4u, stride);
	program[2] = rv_i(0x13u, 2u, 0u, 2u, -1);
	program[3] = rv_b(1u, 2u, 0u, -12);
	if (!jit_compile(0u, (const uint8_t *)program, 0u, TEST_PC, 4u))
		return false;
#ifndef RV32_NATIVE_JIT_TEST_BASELINE
	if (store) {
		if (jit_compile_workspace.loop_write_cache_stubs[0].alignment_mask !=
		    (scenario == 2u ? 0u : width - 1u))
			return false;
	} else if (jit_compile_workspace.loop_read_cache_stubs[0].alignment_mask !=
		   (scenario == 2u ? 0u : width - 1u)) {
		return false;
	}
#endif
	for (uint32_t i = 0; i < sizeof(data); i++)
		expected_data[i] = data[i] = (uint8_t)(i * 13u + 7u);
	regs[2] = 3u;
	regs[3] = 0x11223344u;
	regs[4] = address;
	memcpy(expected_regs, regs, sizeof(regs));
	for (uint32_t iteration = 0u; iteration < budget / 4u; iteration++) {
		uint32_t effective = expected_regs[4] + (uint32_t)offset;
		uint32_t page = (effective - TEST_DATA) >> 12;
		if ((effective & (width - 1u)) != 0u) {
			trap = store ? 6u : 4u;
		} else if (page >= 2u || (pages & (1u << page)) == 0u) {
			trap = store ? 15u : 13u;
		}
		if (trap != 0u) {
			trap_address = effective;
			break;
		}
		if (store) {
			memcpy(expected_data + effective - TEST_DATA, &expected_regs[3], width);
			uint32_t start = (page + 1u) * 4096u;
			if (start < dirty_start) dirty_start = start;
			if (start + 4096u > dirty_end) dirty_end = start + 4096u;
		} else {
			expected_regs[3] = 0u;
			memcpy(&expected_regs[3], expected_data + effective - TEST_DATA, width);
		}
		expected_regs[4] += (uint32_t)stride;
		expected_regs[2]--;
		completed += 4u;
	}
	*runtime = (struct rv32_native_jit_runtime) {
		.regs = regs, .load = fault_load, .store = fault_store,
		.loop_iterations = budget / 4u,
		.chain_executed = 13u, .chain_instruction_limit = 13u + budget,
		.dirty_epoch = 1u,
		.sites = jit_memory_sites[0], .site_count = jit_entries[0].memory_site_count,
		.result = { .executed = UINT32_MAX, .linear_dirty_start = UINT32_MAX },
	};
	jit_invalidate_memory_pages();
	for (uint32_t page = 0; page < 2u; page++) {
		if ((pages & (1u << page)) == 0u) continue;
		uint32_t tag = (TEST_DATA >> 12) + page;
		uint32_t index = tag & (RV32_NATIVE_JIT_PAGE_CACHE_ENTRIES - 1u);
		runtime->read_pages[index] = (struct rv32_native_jit_read_page) {
			.virtual_page = tag, .host_bias = (uint32_t)(uintptr_t)data - TEST_DATA,
		};
		runtime->write_pages[index] = (struct rv32_native_jit_write_page) {
			.virtual_page = tag, .host_bias = (uint32_t)(uintptr_t)data - TEST_DATA,
			.linear_page_start = (page + 1u) * 4096u,
			.linear_page_end = (page + 2u) * 4096u,
		};
	}
	uint32_t bailout = rv32_native_jit_enter(
		(jit_block_fn)(uintptr_t)jit_entry_code(0u), runtime);
	if (bailout != 0u) jit_resolve_memory_bailout(runtime, bailout);
	if ((bailout != 0u) != (trap != 0u) || runtime->result.trap != trap ||
	    runtime->result.executed != completed ||
	    runtime->chain_executed != 13u + (trap ? 0u : completed) ||
	    runtime->memory_calls != (trap ? 1u : 0u) ||
	    runtime->result.trap_value != trap_address ||
	    runtime->result.next_pc !=
		(trap || expected_regs[2] != 0u ? TEST_PC : TEST_PC + 16u) ||
	    runtime->result.linear_dirty_start != dirty_start ||
	    runtime->result.linear_dirty_end != dirty_end)
		return false;
	for (uint32_t i = 0; i < 32u; i++)
		if (regs[i] != expected_regs[i]) return false;
	for (uint32_t i = 0; i < sizeof(data); i++)
		if (data[i] != expected_data[i]) return false;
	return true;
}

#ifndef RV32_NATIVE_JIT_TEST_BASELINE
static bool test_link_endpoint_promotion(void)
{
	uint32_t program[32] = { 0 };
	uint32_t regs[32] = { 0 };
	uint32_t source_slot = jit_hash(0u, TEST_PC, JIT_CACHE_SETS - 1u) * JIT_CACHE_WAYS;
	uint32_t target_slot = jit_hash(64u, TEST_PC + 64u, JIT_CACHE_SETS - 1u) * JIT_CACHE_WAYS;
	const struct rv32_native_jit_result *result;

	if (source_slot == target_slot) target_slot++;
	rv32_native_jit_flush();
	program[0] = rv_i(0x13u, 10u, 0u, 10u, 1);
	program[1] = rv_i(0x13u, 10u, 0u, 10u, 2);
	program[2] = rv_b(0u, 0u, 0u, 56);
	program[16] = rv_i(0x13u, 1u, 0u, 1u, 1);
	program[17] = rv_i(0x13u, 2u, 0u, 2u, -1);
	program[18] = rv_b(1u, 2u, 0u, -8);
	if (!jit_compile(source_slot, (const uint8_t *)program, 0u, TEST_PC, 3u) ||
	    !jit_compile(target_slot, (const uint8_t *)program, 64u, TEST_PC + 64u, 3u))
		return false;
	rv32_native_jit_bind(regs, (uint8_t *)program, NULL, fault_load, fault_store);
	jit_runtime.memory_context = 0u;
	jit_links[source_slot].expected_next_pc = TEST_PC + 64u;
	jit_links[source_slot].packed_target = jit_pack_link_target(target_slot, &jit_entries[target_slot]);
	jit_hot_entries[target_slot].calls = JIT_HOT_LOOP_ADMIT_CALLS - 1u;
	regs[2] = 4u;
	result = rv32_native_jit_try(0u, TEST_PC, 99u, 0u);
	if (result == NULL || result->trap != 0u || result->executed != 15u ||
	    regs[1] != 4u || regs[2] != 0u || regs[10] != 3u)
		return false;
#if !RV32_NATIVE_JIT_TEST_LINK_ENDPOINT_ADMISSION
	/* Isolate alignment code generation from placement-policy changes: a
	 * native endpoint must not admit itself in the original policy. The
	 * ordinary C-dispatch path must still promote and retarget correctly. */
	if (jit_hot_entries[target_slot].location != 0u ||
	    jit_hot_entries[target_slot].calls != JIT_HOT_LOOP_ADMIT_CALLS - 1u ||
	    jit_link_target_pointer(jit_links[source_slot].packed_target) !=
		(uint32_t)(uintptr_t)jit_entry_code(target_slot))
		return false;
	(void)jit_dispatch_code(target_slot, &jit_entries[target_slot]);
#endif
	if (jit_hot_entries[target_slot].location == 0u ||
	    jit_hot_entries[target_slot].calls != JIT_HOT_LOOP_ADMIT_CALLS ||
	    jit_link_target_pointer(jit_links[source_slot].packed_target) !=
		(uint32_t)(uintptr_t)jit_hot_entry_code(target_slot))
		return false;
	/* A second traversal must use the relocated target without changing
	 * architectural state or attempting to promote it again. */
	uint32_t promotions = jit_stats.hot_promotions;
	regs[2] = 4u;
	result = rv32_native_jit_try(0u, TEST_PC, 99u, 0u);
	return result != NULL && result->trap == 0u && result->executed == 15u &&
		regs[1] == 8u && regs[2] == 0u && regs[10] == 6u &&
		jit_stats.hot_promotions == promotions;
}
#endif

int test_main(void)
{
	if (!rv32_native_jit_init()) return 10;
#ifndef RV32_NATIVE_JIT_TEST_BASELINE
	if (!test_alignment_proof()) return 20;
#endif
	for (uint32_t store = 0; store < 2u; store++)
		for (uint32_t width = 2u; width <= 4u; width += 2u)
			for (uint32_t scenario = 0; scenario < 9u; scenario++)
				if (!test_access(store != 0u, width, scenario))
					return (int)(30u + store * 18u + (width / 2u - 1u) * 9u + scenario);
#ifndef RV32_NATIVE_JIT_TEST_BASELINE
	if (!test_link_endpoint_promotion()) return 80;
#endif
	return 0;
}
