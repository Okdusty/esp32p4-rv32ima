# Generic JIT loop-tail shortcut — 2026-09-05

Status at about 13:23 UTC: the candidate is built and all local correctness
tests pass. It has **not been flashed**. Hardware FPS and stability remain
unmeasured for this change. The connected board still runs the unchanged
display/reset-diagnostic application. Its second post-TX-repair control
completed 100 presents in 37.457 seconds (2.6697 FPS), exit zero. The user
asked for an assessment of whether the performance goal is achievable;
pre-flash SD unmount has not started. SD/swap remain mounted, with no game
running, and UART is responsive.

## What changed

Only three emitters in `main/rv32_native_jit.c` changed from the deployed
alignment-only JIT: `jit_emit_branch_to_start()`,
`jit_emit_jal_to_start()`, and `jit_emit_interior_backedge()`.

The conditional tail used to construct the fall-through guest PC on every
iteration, even when it immediately repeated the loop. That constant is now
constructed only on the exit path. The remaining-budget test branches
directly to the native loop header, removing the extra unconditional back
branch. Unconditional JAL loops use the same direct budget branch.

For a conditional loop whose exit PC needs LUI plus ADDI, this removes three
executed host instructions per continuing iteration. For a single-instruction
PC constant it removes two. Unconditional loops lose one repeated instruction.
The unchanged instruction-count checkpoint still charges the same guest work.

This is general loop code generation, not a game-PC or application-specific
pattern. No unrolling, widened/fused stores, guest instruction substitution,
game-source edit, kernel change, relaxed alignment/page permission check,
watchdog change, or clock/cache setting change is involved. Existing memory
bailouts, exact exit PCs, dirty tracking, and native-chain budget limits remain.

The generated-code arena limits a region to 2,048 bytes. Both old and new
backward paths use the same checked +/-4 KiB branch encoding, so this shortcut
does not newly restrict branch reach.

## Measured instruction effect

The unchanged generic microbenchmark performs 128 repeats of 8,192 iterations
of SH / pointer ADDI / BLTU: 1,048,576 halfword stores.

| Measurement | Before | Candidate |
| --- | ---: | ---: |
| Host instructions in a warm continuing iteration | 14 | 11 |
| Simulator CORE instruction-fetch accesses, whole fixture | 29,905,080 | 23,614,412 |
| Read accesses | 8,251 | 8,256 |
| Write accesses | 1,130,030 | 1,130,040 |

The whole-fixture instruction-fetch count falls **21.0354%**. It includes
compiler/setup/harness work and is not an ESP32-P4 cycle count or a game FPS
measurement. The simulator fetches RV32 instructions in halfwords; the
6,290,668-access reduction is not 6,290,668 fewer host instructions.
Both runs returned zero with their register, count and final-data assertions
passing. The observed simulator execution times are not hardware benchmarks.

[Raw generated code and simulator reports](jit-backedge-instructions-2026-09-05.log)
show the exact warm path: offsets 0x28..0x5c before and 0x28..0x50 after.
The memory fast path and refill instructions in that dump are unchanged.

## Correctness checks

`scripts/test-native-jit.sh` passes in both counterless production and
profiling-generated-code shapes. Each mode runs:

- Existing in-source startup/self-tests and 36 memory/alignment/fault cases.
- 256 hot-tier eviction/retarget pairs, 64 cold-arena reuse cycles,
  rejected replacement and full-flush checks.
- 7,056 new branch-tail cases: all six conditional branches plus JAL x0,
  loops to the region start and an interior header, different/equal/zero
  operands, signed and unsigned edge/wrap values, three guest-PC constant
  encodings, and seven budgets around repeat boundaries.

The new fixture compares all 32 guest registers, exact next PC, local and
inherited chain counts, trap status, and native result identity against a
bounded reference calculation. It includes a nonzero inherited chain count.
The fixture does not model P4 caches, DMA or electrical faults.

`make test` (emulator page-cache tests) and `git diff --check` also pass.
The freestanding test's RWX-segment linker warning is expected for its
executable JIT arena. [Full test output](jit-backedge-tests-2026-09-05.log)
records the actual passes and artifact directory.

Reproduce the correctness checks with:

```sh
RV32_CC=/opt/riscv32ima/bin/riscv32-unknown-linux-gnu-gcc \
RV32_RUN=/opt/riscv32ima/bin/riscv32-unknown-linux-gnu-run \
sh scripts/test-native-jit.sh
make test
```

## Source and firmware identity

- Baseline JIT SHA256:
  `8651a1924f74edf293162b70efa619f32bd211f816708e45f1cf5348b5c9ae1d`
- Candidate JIT SHA256:
  `0760b9d7e7454187e20b365e458bb87b3d2f4076ceeaab1045e12828e1938728`
- [Bounded source diff](jit-backedge-shortcut-2026-09-05.patch);
  baseline snapshot `/tmp/rv32-jit-backedge.aHfhfQ/jit-before-backedge.c`.
- Build: `build-rv32-jit-backedge-production/uc-rv32ima.bin`
- Candidate application SHA256:
  `181d373a3b3c56cd55c5b98ab08ad5718a1cf9a5e4ef7facbf1ccebb20dae3a2`
- Candidate ELF SHA256:
  `11133a66afcbf2aa0aa785f7af2fd4f138390ee7b5e79245f97131bcdb83c1a2`
- Application bytes: 733,440, up 288 from the deployed diagnostic.
- Static DIRAM: 320,604 bytes, up 278; external text unchanged at 329,440.
- Config: `/tmp/rv32-jit-backedge.aHfhfQ/sdkconfig`, byte-identical to
  `/tmp/esp32p4-rv32-display-diagnostic-sdkconfig` after configuration.
- CPU 360 MHz, L2 256 KiB, 128-byte lines, guest-RAM prefetch on;
  host/emulator profiling counters off.
- Built kernel, DTB and partition table compare byte-identical to the
  deployed diagnostic build. No guest build was requested.
- `main/display_bridge.c` and `main/uc-rv32ima.c` retain their deployed
  diagnostic hashes. Reset-reason and rare ownership-event logging remain.

Build command:

```sh
source /home/dusty/esp-idf-v6.1-rc1/export.sh
idf.py -B build-rv32-jit-backedge-production \
  -D SDKCONFIG=/tmp/rv32-jit-backedge.aHfhfQ/sdkconfig \
  -D SDKCONFIG_DEFAULTS=/tmp/esp32p4-rv32-display-diagnostic-sdkconfig build
```

## Hardware plan and limitations

The currently installed application is still
`c4bf84192586afff21d68e96265bae10cedf98d371ed24e4e937cc6fe6075eb6`.
Its previous 100-present controls took 35.804 and 44.292 seconds on different
boots. The source shortcut must not be credited for either baseline result.

The additional unchanged-firmware control returned:

```text
control_exit=0
frames=100 elapsed_ms=37457 fps_x100=266 mode=present
```

The benchmark binary hash again matches
`66304de72f65948eaad46284cc3f0bf02ea74ed78387f5dea7ab8a398b532285`.
Release occurred at host 3,514,114 ms and terminal takeover at 3,579,076 ms
with 44 input bytes, consistent with the returning shell prompt. No
intermediate takeover or new reset was captured. Guest uptime was 3630.92 s,
no game PID remained, and the kernel panic/oops/BUG/OOM grep had no matches.
The ownership interval is 64.962 seconds, not the benchmark's 37.457-second
window. Startup and shutdown time must not be substituted for frame time.
[Raw UART control output](jit-backedge-control-uart-2026-09-05.log) also shows
that the result is in `/tmp/doom-benchmark.log`, not redirected startup stdout.

The 44.292 and 37.457-second controls on the same boot illustrate substantial
variation without a firmware change. There is no candidate hardware result
yet, and no basis for a guaranteed 5 FPS or a proven silicon ceiling. At
5 FPS, 100 presents must take at most 20 seconds; the latest control would
need about 87% more throughput. The focused loop's 21% fetch-access reduction
cannot be extrapolated to the full frame. One bounded hardware comparison
is worthwhile; continued small tweaks are not justified without a repeatable
game-level gain.

When proceeding with deployment: sync, disable the existing SD swap,
unmount SD, request the user's one-word DOWN action, then application-only
flash via the verified Espressif USB download interface at 921600 without a
pre-connection reset. Do not use the FTDI UART for this flash.

A subsequent comparison must use the unchanged SD benchmark binary and the
same demo, RGB565 mode, audio flags and 100-present window. Output is 640x400;
the installed normal game reports its existing 320x200 engine scaled 2x.
Presents are application calls, not confirmed physical VSYNC frames.

The old intermittent display symptom and whole-device reboot are still
unresolved. A successful simulator test or a quiet short replay does not
prove their cause or fix. See the
[display/reset investigation](display-reset-investigation-2026-09-05.md).
