#!/bin/sh
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
native_test_dir=$(mktemp -d "${TMPDIR:-/tmp}/rv32-native-jit.XXXXXX")
native_cc=${RV32_CC:-riscv32-unknown-linux-gnu-gcc}
native_run=${RV32_RUN:-riscv32-unknown-linux-gnu-run}

# The simulator lacks fence.i/cache modeling. Only this standalone build uses
# the existing TEST_NO_FENCE_I hook; firmware keeps real publication/fences.
# Both generated-code shapes execute the existing self-tests plus the new
# alignment/fault/dirty-range and dispatch-promotion/retargeting fixtures.
# A separate lifetime fixture forces hot eviction, cold ring reuse, rejected
# replacement and whole-cache flush while checking incoming native pointers.
# The backedge fixture sweeps all branch conditions, PC encodings and budgets.
if [ "$#" -eq 0 ]; then
	set -- shortcuts lifetime backedges initial-values page-fences two-page-stores shared-pages host-range ram-refill exit-trace split-load-pages supervisor fence-keep hoisted-stores atomics csr kernel-linear hle dispatch-misses rom
fi
for native_mode in production profile; do
	if [ "$native_mode" = production ]; then native_stats=0; else native_stats=1; fi
	for native_fixture do
		if [ "$native_fixture" = exit-trace ]; then native_trace=1; else native_trace=0; fi
		"$native_cc" -march=rv32ima -mabi=ilp32 -O2 -g -Wall -Wextra -Werror \
			-mno-relax -ffreestanding -fno-builtin -fno-pic -fno-pie \
			-nostdlib -static -no-pie -Wl,--build-id=none \
			-Wl,-T,"$repo_dir/tests/native-jit/link.ld" \
			-I"$repo_dir/tests/native-jit/include" -I"$repo_dir/main" \
			-DCONFIG_RV32_HOST_PERF_STATS="$native_stats" \
			-DCONFIG_RV32_EMULATOR_PERF_STATS="$native_stats" \
			-DCONFIG_RV32_JIT_EXIT_TRACE="$native_trace" \
			-DRV32_NATIVE_JIT_RUNTIME_COUNTERS="$native_stats" \
			-DRV32_NATIVE_JIT_TEST_NO_FENCE_I=1 \
			"$repo_dir/tests/native-jit/runner.c" \
			"$repo_dir/tests/rv32-native-jit-$native_fixture.c" \
			"$repo_dir/main/rv32_jit_trace.c" \
			"$repo_dir/main/rv32_native_jit_enter.S" -lgcc \
			-o "$native_test_dir/$native_mode-$native_fixture.elf"
		"$native_run" "$native_test_dir/$native_mode-$native_fixture.elf"
		if [ "$native_fixture" = shortcuts ]; then
			printf 'Native JIT %s self-tests and 36 memory cases passed\n' "$native_mode"
		elif [ "$native_fixture" = lifetime ]; then
			printf 'Native JIT %s lifetime: 256 hot pairs, 64 cold-reuse cycles, rejection/flush passed\n' "$native_mode"
		elif [ "$native_fixture" = backedges ]; then
			printf 'Native JIT %s: 7056 branch/backedge register/PC/budget cases passed\n' "$native_mode"
		elif [ "$native_fixture" = initial-values ]; then
			printf 'Native JIT %s: 1176 entry-value/fault/branch/loop cases passed\n' "$native_mode"
		elif [ "$native_fixture" = page-fences ]; then
			printf 'Native JIT %s: 4096 data fences, code eviction/remap/filter collision and global flush passed\n' "$native_mode"
		elif [ "$native_fixture" = two-page-stores ]; then
			printf 'Native JIT %s: 1056 two-page store mapping/fault/order/wrap/budget cases passed\n' "$native_mode"
		elif [ "$native_fixture" = shared-pages ]; then
			printf 'Native JIT %s: interpreter-seeded mappings, two-page stores, permission/fence/context and exclusion cases passed\n' "$native_mode"
		elif [ "$native_fixture" = host-range ]; then
			printf 'Native JIT %s: host-range aliases/bounds, RAM preservation and generated access revocation passed\n' "$native_mode"
		elif [ "$native_fixture" = ram-refill ]; then
			printf 'Native JIT %s: RAM refill/resume, Doom column, ABI clobbers, budgets, permissions, faults and device delegation passed\n' "$native_mode"
		elif [ "$native_fixture" = exit-trace ]; then
			printf 'Native JIT %s: exit attribution, native successors, faults, flush retention, accounting and bounded overflow passed\n' "$native_mode"
		elif [ "$native_fixture" = csr ]; then
			printf 'Native JIT %s: inline supervisor CSRs, SUM/interrupt exits and satp exclusion passed\n' "$native_mode"
		elif [ "$native_fixture" = atomics ]; then
			printf 'Native JIT %s: inline word atomics and LR/SC, interpreter re-execution on misses passed\n' "$native_mode"
		elif [ "$native_fixture" = split-load-pages ]; then
			printf 'Native JIT %s: split page loads, renderer globals, six-page region, faults, remaps, widths, refills and budgets passed\n' "$native_mode"
		elif [ "$native_fixture" = supervisor ]; then
			printf 'Native JIT %s: kernel linear/page-table regions, U/S/SUM banks, links, exec tags, fences, M-mode exclusion and interpreter dispatch passed\n' "$native_mode"
		elif [ "$native_fixture" = fence-keep ]; then
			printf 'Native JIT %s: kernel regions kept across FENCE.I, user regions dropped, kernel-text writes flush passed\n' "$native_mode"
		elif [ "$native_fixture" = hoisted-stores ]; then
			printf 'Native JIT %s: hoisted store group dirty report with clobbered t1 passed\n' "$native_mode"
		elif [ "$native_fixture" = kernel-linear ]; then
			printf 'Native JIT %s: kernel linear-map widths/groups/streams, window margins and end, U/paging-off fallback, alias text writes, 600 random regions passed\n' "$native_mode"
		elif [ "$native_fixture" = dispatch-misses ]; then
			printf 'Native JIT %s: recorded dispatcher misses, learning budget, page validation, shared privilege table, C stop, install and flush invalidation passed\n' "$native_mode"
		elif [ "$native_fixture" = hle ]; then
			printf 'Native JIT %s: host-routine identification, native calls into routines and first-visit budget passed\n' "$native_mode"
		elif [ "$native_fixture" = rom ]; then
			printf 'Native JIT %s: ROM fetch/load in place, published reads, native ROM regions, store and past-end faults passed\n' "$native_mode"
		fi
	done
done
printf 'Standalone artifacts: %s\n' "$native_test_dir"
