# Native JIT instruction shortcuts — 2026-09-05

## Result and deployment state

Two generic JIT changes were implemented and tested after the user asked to
investigate candidates 2/3: linked-target hot-tier admission and repeated
memory guards. The emitted warm memory loop is smaller, but the combined
candidate has **not demonstrated a game-speed improvement**.

Three consecutive hardware runs completed with exit status zero:

| Firmware / run | 100-frame interval | Calculated FPS |
| --- | ---: | ---: |
| Previous unchanged production replay, earlier boot | 31.687 s | 3.1559 |
| Shortcuts candidate, run 1 | 32.974 s | 3.0327 |
| Shortcuts candidate, run 2 | 32.529 s | 3.0742 |
| Shortcuts candidate, run 3 | 37.664 s | 2.6551 |
| Candidate aggregate: 300 frames / 103.167 s | 343.89 ms/frame | 2.9079 |

The exact prior production binary was restored through `/dev/ttyACM0`
around 01:34 UTC, with flash hash verification and the expected boot ELF
prefix `9036b4c85`. The return controls completed in 31.847, 31.166 and 37.022 seconds,
an aggregate 2.9990 FPS. The alignment-only candidate was subsequently flashed
around 01:50 UTC, hash verified, and passed its hardware boot self-tests.
Its three intervals were 31.198, 35.900 and 30.635 seconds, an aggregate
3.0696 FPS versus 2.9990 for the return control. That small difference is
within the observed variation, not a proven gain. A later whole-device reboot
was captured around 02:09 UTC; see the fault follow-up below. The
current source now retains only the alignment shortcut; endpoint admission
was withdrawn to isolate the two changes. This alignment-only build is
tested locally and at boot, and remains experimental. No source
changes were made between either set of three hardware runs.
The user confirmed normal colors/refresh on the alignment-only candidate,
then clarified that a brief roughly once-per-minute screen glitch still
occurs. This is not a clean long-duration display result. The subsequent
whole-device reboot is also unresolved; manual intervention was asked about
but has not yet been confirmed or excluded.

The third run also slowed within the same candidate boot. Previous work
already recorded substantial variation across unchanged-firmware boots.
These samples therefore neither establish a repeatable regression nor prove
that either individual shortcut helps this workload. Both changes were in
one binary, so their effects cannot be separated from this comparison.

## 3: move an invariant alignment check out of the repeated path

Source: `main/rv32_native_jit.c`, `jit_loop_preserves_alignment()` and the
loop-cache refill emitters.

The existing local read/write page caches already skip most translation
work. Their warm halfword/word access still checked alignment every iteration.
The new compile-time proof permits moving that check into the cache-refill
path when every instruction that writes the access's base register in the
repeated suffix is a self-ADDI by a multiple of the access width, or when the
base is unchanged. All instructions in the suffix are considered, including
alternative branch paths. Unsupported writes retain the original guard.

For example, the emitted halfword-store loop changes from:

```asm
mv    a1,t3
mv    a2,s3
andi  t2,a1,1
bnez  t2,cold_bailout
srli  t0,a1,12
bne   t0,s9,refill
add   t0,a1,a5
sh    a2,0(t0)
```

to:

```asm
mv    a1,t3
mv    a2,s3
srli  t0,a1,12
bne   t0,s9,refill
add   t0,a1,a5
sh    a2,0(t0)
```

The ANDI and conditional branch now start the refill path. This removes
two host instructions per eligible warm access, not two guest instructions.
The local tag starts invalid on every native entry, so the first access still
checks alignment before installing a mapping or doing a native load/store.
A page change still checks alignment and the validated shared page mapping.
A non-invariant stride retains the per-access alignment check, even when an
earlier aligned access has primed the same-page tag.

This is deliberately narrower than a complete loop-range/page-bound proof.
The virtual-page calculation and tag comparison remain in the loop. It does
not remove translation, MMIO fallback, permissions, dirty reporting, precise
fault state, or guest-instruction service boundaries. The refill metadata
reuses an existing padding byte; compile-time cache descriptors gain the
proof flag, not the runtime per-region directory.

## 2: credit a directly linked endpoint at an existing C return

This describes the combined candidate tested first. The endpoint-admission
change has since been removed from current source for the isolated comparison
below; the combined binary, exact source snapshot and patch are preserved.

Source: `rv32_native_jit_try()` after validating `returned_slot`.

Native tail links bypass `jit_dispatch_code()`, which normally updates the
internal executable-SRAM admission counter. A frequently linked target can
therefore be undercounted. The new path credits one observed normal endpoint
when it differs from the region initially dispatched by C:

```c
if (returned_slot != slot && bailout == 0u)
    (void)jit_dispatch_code(returned_slot, returned_entry);
```

It uses the existing promotion, publication and incoming-link retargeting
mechanism after native execution has returned. The authoritative PSRAM code
copy remains available. No promotion threshold, hot-tier capacity or generated
per-link/per-pixel instrumentation was changed.

This is a bounded accounting correction, not complete per-region profiling.
It credits one endpoint, not all instructions executed by the chain; it does
not double-count the dispatched slot or credit cold-memory bailout endpoints.
Intermediate linked regions remain unsampled. The additional C-side admission
work and changed hot-tier competition could offset benefits; their actual
contribution is unmeasured in the combined production run.

## Validation

The new repeatable standalone test runs real generated RV32 instructions:

```sh
RV32_CC=/opt/riscv32ima/bin/riscv32-unknown-linux-gnu-gcc \
RV32_RUN=/opt/riscv32ima/bin/riscv32-unknown-linux-gnu-run \
sh scripts/test-native-jit.sh
```

Both production-shaped and profiling-shaped builds passed the existing JIT
self-tests, the conservative proof tests, 36 reference-checked memory cases,
and a linked-endpoint promotion/retargeting fixture. The 36 cases combine
load/store and widths 2/4 with nine scenarios: aligned access, initial
misalignment, same-page misalignment after warming, missing next page,
prevalidated next page, negative stride, odd base plus aligned effective
offset, a one-iteration service budget, and no validated mapping.

Assertions check all guest registers, the complete 8 KiB memory image,
executed counts, trap PC/value, callback counts and dirty ranges. The
no-mapping case checks fallback to the checked callback, not a physical MMIO
device transaction. The same 36 memory cases also passed against the exact
pre-change source, excluding assertions for the new proof/promotion behavior.
The linked fixture reaches the admission threshold through a native link,
checks promotion and incoming-pointer retargeting, then executes the relocated
target on a second traversal and verifies guest state again.

The standalone simulator does not implement the required instruction-cache
behavior. Initial simulator runs stopped on FENCE.I; using the existing
test-only `RV32_NATIVE_JIT_TEST_NO_FENCE_I` hook allows execution there.
Firmware retains its real cache synchronization and instruction fences.
Standalone linker RWX warnings are expected for the generated-code fixture.
Successful standalone artifacts are in `/tmp/rv32-native-jit.nguh2g/`.

`make test` and `git diff --check` also passed. `make test` is the separate
mini-rv32ima page-cache suite; it does not substitute for the new native-JIT
tests. The production firmware build passed, and the flashed board passed
its PSRAM/native-JIT boot self-tests before reaching the Linux shell.

## Instruction-count evidence, not a hardware FPS claim

`tests/rv32-native-jit-memory-bench.c` executes a generated halfword-store,
pointer-increment and branch loop for 8,192 iterations, repeated 128 times:
1,048,576 memory accesses / 3,145,728 guest instructions. Both the unchanged
and candidate JIT completed with correct final state and exit status zero.

GNU simulator `--verbose --profile-insn --profile-core` results:

| CORE access category | Before | After |
| --- | ---: | ---: |
| Read | 8,246 | 8,251 |
| Write | 1,130,035 | 1,130,030 |
| Exec | 34,097,166 | 29,905,080 |
| Total | 35,235,447 | 31,043,361 |

The approximately 12.3% reduction is in simulator code-fetch events, not
measured ESP32-P4 cycles or exact retired-instruction counts. Simulator wall
time is not an MCU performance metric. Dumps of the actual emitted code
independently confirm the two removed warm-path instructions and their new
refill location. Raw disassembly and simulator output are preserved in
`jit-shortcuts-instructions-2026-09-05.log`.

## Firmware identity and unchanged scope

| Property | Previous production | Combined candidate |
| --- | --- | --- |
| Build directory | `build-rv32-jit-results-production` | `build-rv32-jit-shortcuts-production` |
| Application bytes | 732,080 | 732,928 |
| Static DIRAM bytes | 319,363 | 320,098 |
| CPU | 360 MHz | 360 MHz |
| L2 / cache line | 256 KiB / 128 B | 256 KiB / 128 B |
| Guest-RAM prefetch | Enabled | Enabled |
| Internal hot-code tier | 16 KiB | 16 KiB |
| SD cache | 700 KiB | 700 KiB |

Previous application SHA256:
`3da2799317538fe5fe4ff31b1ae62df88f05fd1f1f60abb4a560a2684c42f8bd`.
Candidate application SHA256:
`95aa356e8221a70b3f12aec46dfcdb198080e6b714ac2f05df541c0f41826e92`.
Candidate ELF SHA256:
`7679b75f1230d1e38086f64d5bfd5caea2e4285d7ae850792bebcc5f20484394`.
Candidate JIT source SHA256:
`85d4303dda38b56437d84ee937e17967bfd099e38eb074ba123c5a121fdb3982`.
Pre-change JIT source SHA256:
`026e169649b3cae743c35d35e4262f051742de95383f0a5bbe701d75001d4bf6`.

The original source is preserved at
`/tmp/rv32-jit-selftest/jit-before-loop-alignment.c`; the bounded source diff
is also recorded as `jit-shortcuts-source-2026-09-05.patch`. The JIT was already
untracked in the accumulated worktree, so ordinary `git diff HEAD` omits it.
No commit, staging, reset, deletion, game-source change, guest-kernel change,
DTB change, display change, partition change or clock experiment was made
for this candidate. In particular, 400 MHz was not attempted.

Host/emulator/cache sampling is off. The candidate configuration is
semantically the previous production configuration with the now-defined
`CONFIG_RV32_CACHE_PERF_STATS` explicitly disabled. Static DIRAM grows by
735 bytes; the external text size is unchanged. Boot reports the same guest
RAM allocation (`0x480a0a08`) and scanout allocation (`0x49ea0a80`). Display
initialization remains 720x720 RGB565 at 59 Hz with PPA ready.

## Hardware procedure and postchecks

The application-only flash completed with hash verification through
`/dev/ttyACM0` at 921600 baud around 01:09 UTC. Console and benchmark control
used the existing FTDI `/dev/ttyUSB1` session at 921600 baud, not SSH.
Each run used the unchanged SD executable, the same demo, and a distinct
stdout/stderr log. Shell continuation lines were kept short after observed
UART input loss on long commands.

```sh
../doom-bench-640 -iwad ../freedoom1.wad \
-gfxmode rgb565 -nosound -nomusic -timedemo demo1 \
>/tmp/doom-shortcuts-3.out 2>&1; rv32_bench_status=$?
```

Exit status zero was collected before reading each benchmark result to avoid
accepting a stale log. The executable SHA256 was verified as unchanged:
`66304de72f65948eaad46284cc3f0bf02ea74ed78387f5dea7ab8a398b532285`.
The measured interval is exactly 100 application-present calls, excluding
startup/loading; it is not independent physical-panel VSYNC measurement.
No concurrent guest diagnostics ran during the timed intervals.

All startup logs report demo1 and a 600,000-byte zone. Their virtual zone
addresses differ (`0x94ffe040`, `0x94f36040`, `0x94f57040`); this is recorded
as a possible uncontrolled variable, not a demonstrated explanation.
After run 3, no benchmark process remained, SD swap was active with 6,100 KiB
used, the kernel log search found no panic/oops/BUG/OOM/underrun, three gateway
pings succeeded, and sync returned. C6/WiFi startup was successful. USB host
initialized but no attached peripheral was available for a functional test.
These checks do not replace physical-panel observation or long-duration
reliability testing.

The printable candidate boot/benchmark/postcheck capture is
`jit-shortcuts-uart-2026-09-05.log`. The exact previous production binary has
now been restored and its return-control runs completed as recorded below.
A reliable gain has not been established; 5 FPS has not been
achieved, and no absolute hardware ceiling is inferred from this experiment.

## Return control and isolation of the alignment shortcut

The exact prior production application was flashed successfully around
01:34 UTC, preserving the same clock/cache/display configuration. Its expected
ELF prefix, compile time, PSRAM/JIT self-tests, guest-RAM/scanout addresses,
16 KiB hot tier and 700 KiB SD cache were verified at boot. The same SD game
executable hash and three identical 100-present demo commands were used,
with output files `/tmp/doom-shortcuts-A1.out` through `-A3.out`.

| Run | Combined shortcut interval | Original return-control interval |
| --- | ---: | ---: |
| 1 | 32.974 s | 31.847 s |
| 2 | 32.529 s | 31.166 s |
| 3 | 37.664 s | 37.022 s |
| Aggregate FPS | 2.9079 | 2.9990 |

All six runs exited zero. The control itself slowed substantially on the
third run, so that slowdown cannot be attributed solely to the new shortcuts.
The combined candidate was about 3.0% lower in aggregate FPS than this control,
but the small cross-boot sample does not establish causality or identify
which component mattered. The combined policy is not being adopted.

Control postchecks found no remaining game process or matching kernel
panic/oops/BUG/OOM/underrun. Swap use was 6,108 KiB, all three gateway pings
succeeded, and sync returned. The three 600,000-byte zone addresses were
`0x94fb5040`, `0x94f40040` and `0x94f9a040`. Neither these virtual addresses
nor swap use establish the cause of the run-order variation. Raw boot, runs
and checks are in `jit-shortcuts-return-control-2026-09-05.log`.

To avoid discarding an emitted-instruction reduction because of an unrelated
placement-policy cost, the next candidate removes only the nine-line
endpoint-admission addition from the combined JIT. The alignment analysis
and generated memory paths are unchanged. No extra shortcut or cache policy
has been added. The combined source is preserved at
`/tmp/rv32-jit-selftest/jit-shortcuts-combined.c` with its original SHA256.
The alignment-only diff from the original source is also saved in
`jit-alignment-source-2026-09-05.patch`.

The test fixture now verifies the original admission behavior: a native
endpoint remains unpromoted at threshold-minus-one, then an ordinary explicit
C dispatch promotes it and retargets the incoming link. A second traversal
checks execution through the relocated code. The old endpoint-admission
expectation remains selectable with the test-only
`RV32_NATIVE_JIT_TEST_LINK_ENDPOINT_ADMISSION=1` when testing the preserved
combined source. Neither test selector affects production firmware.

Both production/profiling standalone modes passed again, including all 36
reference memory cases and existing self-tests. Artifacts are in
`/tmp/rv32-native-jit.Dq7JB5/`. The separate `make test` page-cache suite and
`git diff --check` also passed. The alignment-only firmware build succeeded;
its generated SDK configuration has zero semantic differences from the
combined candidate's configuration, with profiling off and CPU at 360 MHz.

Alignment-only application: `build-rv32-jit-alignment-production/uc-rv32ima.bin`

- Application SHA256: `58b76227dce9d7bf62ce6831547bba7381161644516b5d9270b32d3440c08612`
- ELF SHA256: `0bb58e8ac444d440dafa7bc28afff2cd56376e3432c1d7e424dfdea8d8a06435`
- Source SHA256: `8651a1924f74edf293162b70efa619f32bd211f816708e45f1cf5348b5c9ae1d`
- Application size: 732,688 bytes; static DIRAM: 319,974 bytes, 611 above baseline.
- Configuration: `/tmp/esp32p4-rv32-alignment-production-sdkconfig`.

At about 01:50 UTC, this candidate was flashed through `/dev/ttyACM0` at
921600 with written hash verification. The board passed PSRAM/JIT self-tests,
reported the expected ELF prefix `0bb58e8ac`, retained the 16 KiB hot tier
and 700 KiB SD cache, and reached Linux. CPU and display/guest allocations
are unchanged. The three identical 100-present runs completed as follows:

| Run | Alignment-only interval | Calculated FPS |
| --- | ---: | ---: |
| 1 | 31.198 s | 3.2053 |
| 2 | 35.900 s | 2.7855 |
| 3 | 30.635 s | 3.2642 |
| Aggregate, 300 / 97.733 s | 325.78 ms/frame | 3.0696 |

All three exit statuses were zero. The second rather than third run slowed
this time, so the variation is not a fixed third-run phenomenon. The observed
aggregate is about 2.36% above this return control, too small relative to the
spread/cross-boot uncertainty to claim a reliable improvement. No further
instruction shortcut was added after isolating alignment. The updated
host-test fixture also passed against the preserved combined source with
its endpoint-admission expectation explicitly enabled.

Pre-reboot postchecks found no remaining game process or matching kernel
fault, swap use of 6,052 KiB, three successful gateway pings and a completed
sync. Linux uptimes of 558.79 and later 967.42 seconds were observed.
The game hash was unchanged. Zone-address log collection was interrupted by
the later reboot, so those ephemeral /tmp startup logs are unavailable; no
addresses are inferred. The UART capture, including all three successful
results and the later boot, is `jit-alignment-uart-2026-09-05.log`.

## Separate display-glitch and whole-device-reboot observations

During the alignment-only runs, the user first answered yes to normal color,
refresh and absence of visible corruption/reset. They then clarified:
"screen reset/crash do occur actualy" and described a brief event roughly
once a minute, suggesting cache/buffer handling. Treat that clarification as
superseding the initial no-reset interpretation. A cache cause is a user
hypothesis, not an established diagnosis.

Read-only inspection and a host reproduction found a separate concrete
hazard: `display_terminal_process_stream()` reclaims framebuffer ownership
and marks all terminal rows dirty before checking whether its stream has any
bytes. An empty/stale terminal wake therefore requests a full terminal redraw
without parsing a byte. The production function was copied verbatim into
`display-empty-wake-repro-2026-09-05.c`, with controlled queue/render doubles.
Its result was `reclaimed=1 marked_rows=45 flush_calls=1 parsed_bytes=0`.
Exit zero means the unwanted behavior was reproduced, not that a fix passed.
This does not reproduce the physical glitch or demonstrate its timing/cause.
No display-driver or terminal-ownership fix has been applied.

Around 02:09 UTC a separate event produced a fresh ESP-IDF boot banner and
Linux shell; the user reported "device just rebooted". The subsequent uptime
was 107.18 seconds. No reset/flash command was issued by the assistant at
that time, and the existing UART reader remained live with DTR/RTS inactive.
Whether the user pressed reset/reconnected power was asked and is pending.
No readable panic/brownout/underrun message appears before the second boot.
The 921600-baud reader cannot recover the earlier ROM banner reliably and
filters non-ASCII bytes; one post-reboot output chunk was also truncated.
Do not infer a watchdog, cache error, electrical cause or spontaneous reset
class from the missing message.

A boot-only reset-reason report has now built successfully in
`build-rv32-reset-diagnostic`, adding the SDK's saved classification and raw
CPU reasons to `main/uc-rv32ima.c`. This does not change watchdog policy,
clock, cache, display or kernel configuration. It was application-only
flashed with hash verification around 02:20 UTC and reached Linux with the
expected ELF prefix `562d05c07`. Its first report was `idf=1 raw_cpu0=0x1
raw_cpu1=0x1` (SDK POWERON), from the deliberate post-flash boot, not evidence
of the earlier reboot's cause. It cannot retroactively recover that event's
reason. The JIT remains alignment-only. Details and the startup log are in the
[display/reset investigation](display-reset-investigation-2026-09-05.md).
The 5 FPS goal and fault diagnosis remain open.

The deployed reset-diagnostic image subsequently completed one unchanged
100-present replay in 31.022 seconds (3.2235 FPS), exit zero. Guest uptime
continued to 652.35 seconds with no additional reset report. This bounded
check is within the prior timing range and does not demonstrate a speedup
or long-duration reliability. The diagnostic workload/postchecks are saved
in `reset-diagnostic-workload-2026-09-05.log`.
