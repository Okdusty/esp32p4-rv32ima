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
for native_mode in production profile; do
	if [ "$native_mode" = production ]; then native_stats=0; else native_stats=1; fi
	for native_fixture in shortcuts lifetime backedges; do
		"$native_cc" -march=rv32ima -mabi=ilp32 -O2 -g -Wall -Wextra -Werror \
			-mno-relax -ffreestanding -fno-builtin -fno-pic -fno-pie \
			-nostdlib -static -no-pie -Wl,--build-id=none \
			-Wl,-T,"$repo_dir/tests/native-jit/link.ld" \
			-I"$repo_dir/tests/native-jit/include" -I"$repo_dir/main" \
			-DCONFIG_RV32_HOST_PERF_STATS="$native_stats" \
			-DCONFIG_RV32_EMULATOR_PERF_STATS="$native_stats" \
			-DRV32_NATIVE_JIT_RUNTIME_COUNTERS="$native_stats" \
			-DRV32_NATIVE_JIT_TEST_NO_FENCE_I=1 \
			"$repo_dir/tests/native-jit/runner.c" \
			"$repo_dir/tests/rv32-native-jit-$native_fixture.c" \
			"$repo_dir/main/rv32_native_jit_enter.S" -lgcc \
			-o "$native_test_dir/$native_mode-$native_fixture.elf"
		"$native_run" "$native_test_dir/$native_mode-$native_fixture.elf"
		if [ "$native_fixture" = shortcuts ]; then
			printf 'Native JIT %s self-tests and 36 memory cases passed\n' "$native_mode"
		elif [ "$native_fixture" = lifetime ]; then
			printf 'Native JIT %s lifetime: 256 hot pairs, 64 cold-reuse cycles, rejection/flush passed\n' "$native_mode"
		else
			printf 'Native JIT %s: 7056 branch/backedge register/PC/budget cases passed\n' "$native_mode"
		fi
	done
done
printf 'Standalone artifacts: %s\n' "$native_test_dir"
