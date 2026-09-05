# Performance and source audit — 2026-09-04

New local candidate, 2026-09-05 around 13:19 UTC: a generic JIT loop-tail
shortcut removes exit-PC setup and an extra back branch from continuing
iterations. It reduces simulator instruction-fetch accesses by 21.0354% in
the unchanged store-loop microbenchmark, with all production/profile tests
passing. The new firmware is built but not flashed; no hardware FPS gain is
claimed. An additional unchanged-firmware control returned 37.457 seconds
for 100 presents (2.6697 FPS), exit zero, showing variation from 44.292 seconds
without a firmware change. See [the backedge audit](jit-backedge-shortcut-2026-09-05.md).

Latest diagnostic follow-up, 2026-09-05 around 12:55 UTC: after reconnecting
the FTDI TX wire restored console input, the unchanged display-diagnostic
firmware completed 100 application-present calls in 44.292 seconds (2.2577
FPS), slower than its earlier 35.804-second sample. No new speedup is claimed.
A separate three-minute bounded normal-game replay confirmed 640x400 RGB565
output (existing 320x200 Doom engine scaled 2x), with no unexpected ownership
event or reset captured before the planned SIGTERM. The physical-glitch
verdict is still pending. See [the display/reset investigation](display-reset-investigation-2026-09-05.md)
for binary hashes, complete event windows and postchecks.

Latest experiment, 2026-09-05 around 01:22 UTC: two generic JIT shortcuts
were implemented and flashed after the user's follow-up request. Three
100-frame runs returned 3.033, 3.074 and 2.655 FPS; this is not a demonstrated
game-speed improvement. The exact previous production binary was restored
around 01:34 UTC; its three return controls averaged 2.999 FPS versus 2.908
for the combined candidate, with both images slowing on their third run.
An alignment-only build, restoring the original admission policy, passed
local/boot tests and averaged 3.070 FPS in three runs, not a reliable gain
given the spread. The user reports a periodic brief screen glitch; a later
whole-device reboot was also captured around 02:09 UTC. A reset-reason-only
diagnostic build was flashed around 02:20 UTC and reached Linux; its first
report describes the known post-flash boot, not the earlier fault. Neither
the periodic glitch nor the earlier reboot is diagnosed or fixed. See
[the instruction-level audit](jit-shortcuts-2026-09-05.md) for the bounded
source patch, tests, disassembly, binary identities and raw UART logs.

Previous production verification, 2026-09-05 around 00:40 UTC: the same non-sampling
production application (`3da27993...`) completed another fixed-sequence
100-frame replay in 31.687 seconds, approximately 3.156 FPS, with exit status
zero. The user confirmed accurate colors, decent refreshing and no corruption,
while clarifying that application rendering/processing remains too slow.
No cache-capacity, line-size or prefetch-default change was retained from the
follow-up experiments. The final sections record the checks and limitations.

## Scope and Git baseline

Baseline: `db92727c33c3d7d8c7735ba98c8446fea789a997`, dated 2026-08-31,
subject `sdkconfig hardeningsss`.

This is an audit of the accumulated working tree, not a claim that every
uncommitted line was written in the most recent pass. The baseline comparison
has 39 modified tracked files, 3,040 added text lines and 331 removed text
lines. Binary changes are reported separately by Git and are not included in
those line totals. No commit, reset, cleanup, or staging was performed.

Ordinary `git diff HEAD` also misses substantial untracked source: the native
JIT and assembly entry wrapper, virtio-block and experimental virtio-GPU
bridges, framebuffer path benchmark, display shell, additional display
configuration files, USB FIFO patch, and SSH convenience script. The many
untracked build directories are experiment artifacts, not independent source
changes. The JIT source alone is over 6,000 lines, including its self-tests.

Useful inspection commands:

```sh
git log -1 --format=fuller
git diff --stat HEAD
git diff HEAD -- main/mini-rv32ima.h main/display_bridge.c guest/linux/rv32-pvfb.c
git ls-files --others --exclude-standard
git diff --no-index /dev/null main/rv32_native_jit.c
```

The final command returns status 1 when differences exist; that is normal.

## What has actually improved

The strongest demonstrated gain is in the framebuffer path. An earlier
approximately 12.45-second clear/write test dropped to approximately 0.92
seconds after batching and driver work: about 13.5 times faster for that test.
That is not a 13.5-times game-speed claim. A bulk clear and an application that
computes, converts, scales, and stores individual pixels exercise different
parts of the system.

Recorded game measurements before the new interior-loop experiment:

| Variant | Exact 100-frame duration | Reported FPS / interpretation |
| --- | --- | --- |
| 32-instruction regions, two read caches, production | 34.274 s, 35.082 s, 35.319 s | 2.91, 2.85, 2.83; aggregate about 2.866 FPS |
| 48-instruction region experiment | Recorded runs at 2.77 and 2.49 FPS | Rejected; larger regions were not a win |
| Four-read-cache experiment | 35.240 s, 33.120 s, 42.255 s, 34.985 s | Aggregate about 2.747 FPS; rejected despite one 3.01 FPS run |
| Four-cache build plus legacy-display quiet fix | 35.368 s | 2.82 FPS; presentation correctness and execution speed are separate |
| Top-entry diagnostic build, two read caches | 40.667 s | `frames=100 elapsed_ms=40667 fps_x100=245 mode=present` |
| New interior-loop production build | 35.205 s, 31.065 s, 29.670 s | 2.84, 3.21, 3.37; aggregate about 3.127 FPS |

Aggregate rates above use total frames divided by total elapsed time. The
benchmark's integer `fps_x100` field truncates; for example, 100 / 40.667 is
approximately 2.459 FPS, while the log reports 245.

These are the same SD benchmark executable and normal demo invocation, not
SSH timing or an estimated FPS from CPU utilization. They count the
application's present path; they do not independently certify 100 distinct
physical-panel VSYNC presentations. Startup/loading is outside the timed
100-frame interval. Profiling is intrusive and must not be compared with a
production binary as though the instrumentation were free.

The new three-run aggregate is approximately 9.1% above the earlier 2.866 FPS
three-run baseline. This is an observed end-to-end improvement, not a precise
isolated estimate of the compiler change: the runs show appreciable spread,
cache/guest state can vary, and the new build also restores the USB high-speed
configuration. The test is the normal demo, not a lockstep timedemo with a
fixed number of game ticks. No 5 FPS result has been demonstrated.

## Display driver and framebuffer changes

`guest/linux/rv32-pvfb.c` and `main/display_accel_protocol.h` advance the
paravirtual framebuffer protocol to version 5 and add a bounded bulk-write
operation. Ordinary fbdev `write()` calls can accumulate contiguous data in a
buffer instead of causing a long emulated MMIO store sequence. Full buffers
are submitted immediately; a partial buffer has a 16 ms delayed flush.

Ordering is preserved at drawing, synchronization, release, and mmap
boundaries. There are bounds checks, partial-copy handling, a mutex around
pending write data, shutdown cancellation, and a software fallback when the
accelerated mailbox cannot accept a command. This is generic fbdev support,
not a game-specific syscall or opcode.

The host display bridge now has a logical render surface separate from the
ST7703's continuously scanned framebuffer. The logical surface is in memory
reserved from Linux by the device tree. Dirty rectangles are published with
the ESP32-P4 PPA around VSYNC; CPU copying remains a fallback. Raw framebuffer
ownership also suppresses terminal/cursor drawing over an application's
surface. The FIFO now uses two 32 KiB payload blocks instead of four 16 KiB
blocks, retaining the same 64 KiB payload capacity with fewer submissions.

The important presentation bug was an incorrect end-of-frame assumption. A
slow mmap writer can be interrupted between scanlines. A guest execution
slice with no framebuffer stores does not mean that the frame is finished:
Linux's 100 Hz timer creates such gaps during an unfinished copy. Publishing
then exposes partially painted rows and looks like line-by-line rendering.

The revised legacy path measures elapsed time since actual framebuffer
stores and requires 30 ms of quiet before publishing. Explicit framebuffer
fences do not have that delay. The display worker and emulator also serialize
the snapshot so CPU1 cannot copy a surface while CPU0 is changing it.

This explains the reproducible scanline/presentation failure and is supported
by the reported visual improvement. It is not a proof that every possible
screen blank/reset is software. The ST7703 DMA now has a high AXI priority to
protect scanout deadlines, but electrical integrity, connectors, and power
have not been ruled out by physical measurements. Copying into a single live
scanout buffer is also not equivalent to hardware page-flipped scanout, so
absolute tear-free behavior is not established.

The quiet heuristic has another limitation: an unfenced producer that never
becomes quiet may delay presentation. It is a compatibility mechanism for
legacy mmap clients, not a universal frame-boundary protocol.

## Emulator and JIT changes

`main/mini-rv32ima.h` gained a generic directly backed MMIO-memory aperture.
Checked framebuffer loads/stores can access host memory and collect dirty
ranges once per execution batch. This removes repeated display-handler work
without treating arbitrary device registers as ordinary RAM. Exit and wait
paths still publish pending changes.

Instruction fetch now tracks the instructions remaining in the current
translated page. Sequential execution can reuse that translation without
masking and checking the virtual page on every fetch; control transfers and
translation invalidations refresh the state. Taken branches defer target
decoding until the target is actually needed.

The new native JIT is restricted to supported user-mode integer/control and
checked-memory operations. Kernel, privileged, atomic, and unsupported
instructions remain with the interpreter. It has a 2,048-slot two-way
directory, regions of at most 32 collected guest instructions, a bounded
2 MiB PSRAM code arena, and an optional internal executable hot tier.

There is a hardening tradeoff: this experimental JIT requires writable
executable host memory and is gated on disabling ESP-IDF memory protection.
Checked guest accesses remain required, but this configuration must not be
described as retaining the original host memory-protection policy.

The main mechanisms are:

- Keep useful guest registers in host registers across a region.
- Reuse checked read/write translations, including two local read caches.
- Group compatible memory accesses under shared guards.
- Retain native loops and bounded traces, with guarded control-flow exits.
- Tail-link ready successors without repeatedly crossing the C dispatcher.
- Preserve the host ABI once around the complete native chain.
- Invalidate translations and generated code at the corresponding guest
  address-space and instruction-cache boundaries.
- Keep exact executed-instruction and fault state for the interpreter.

The previous panic had a concrete JIT correctness cause: the second read-page
cache borrowed host registers also used for native-chain instruction counts
and limits. Those values now have an explicit save/restore contract, including
cold exits. Removing the panic is a correctness fix, not evidence that a
higher clock is safe. The CPU remains at 360 MHz; 400 MHz was not attempted in
this pass.

Larger blocks and more read caches were tested and rejected after actual
100-frame runs. More cached state is not automatically faster: it consumes
registers, grows generated code, and competes with guest data in the shared
cache. The current region and read-cache limits are back to 32 and two.

## The bottleneck finding and the new experiment

The diagnostic run showed CPU0 near saturation while CPU1 was lightly loaded,
high native-JIT guest-instruction coverage, short average regions, and
millions of native transitions. CPU utilization alone is not a bottleneck
proof, particularly because the emulator can remain busy while the guest is
idle; the transition and memory-path observations are the more useful part.

One dominant chain entry in the exact SD binary starts at a three-instruction
halfword-store loop: store a pixel, advance the pointer, branch. Surrounding
code performs pixel conversion and establishes a short repetition limit.
The address was used only to inspect the executable. There are no matching
game PCs, application identities, or special instruction patterns in the
optimization.

The old JIT only retained a terminal loop when its target was the first
instruction of the compiled region. A loop following setup instructions was
therefore split and paid avoidable exits, commits, and re-entry work. The new
generic compiler support permits terminal back-edges into an interior label:
the preheader executes once, the suffix repeats with the same register map,
and the remaining instruction budget is checked before another iteration.

It still performs the guest's required per-pixel computation and stores.
This does not silently replace rendering with a game-specific PPA command.

New self-tests cover both branch forms, every budget around short-loop
boundaries, an initially skipped path, early exits, entry through a native
tail link, both read caches, a page fault after a completed iteration,
misaligned stores, and framebuffer dirty-range reporting. Both production
and profiling builds passed in the RISC-V instruction-set simulator. The
ESP32-P4 also passed the new boot-time tests after flashing. `make test`
passed separately; that command covers emulator page-cache tests and should
not be misrepresented as the entire native-JIT test suite.

The profiler itself needed clarification. Its top entries attribute an
entire chain to the initial dispatcher entry; these are not exclusive
per-region cycle samples. Labels and comments now say so. A full JIT flush
also used to discard the interval's accumulated counts; it now retains those
counts in the unattributed/discarded bucket, with a regression test.

Measured PPA publication took milliseconds, while game frames took hundreds
of milliseconds. In the diagnostic full-frame case the blit was approximately
12.7 ms. This makes guest execution, conversion/stores, and native transition
overhead a more promising target than optimizing only the final copy. It does
not exclude shared-memory bandwidth effects: continuous 720 x 720 RGB565
scanout at 59 Hz alone represents about 58 MiB/s of framebuffer reads, before
other PSRAM traffic. Nor is it a rigorous proof of the hardware's maximum FPS.

## CPU1, memory, storage, and I/O

CPU1 already services display work, SD transfers, networking work, and UART
output. It does not execute a second half of the game's single guest CPU
instruction stream. Independent I/O can be offloaded, but arbitrary sequential
guest instructions cannot simply be divided between the two host cores.

The L2 default changed from 128 KiB to 256 KiB. Bulk host buffers moved into a
128 KiB reserved PSRAM pool to retain internal-RAM headroom for hot lookup
tables, stacks, DMA, and device queues. This is a memory-budget tradeoff, not
free RAM. The newly flashed production boot reported a 2 MiB main JIT arena,
a 16 KiB internal hot tier, and a 700 KiB SD read cache. The diagnostic build
had no allocated internal hot tier; that is another reason not to attribute
all production-versus-profile FPS differences to counter instructions alone.

The virtio-block bridge exposes the SD through Linux's standard block driver.
It uses a CPU1 worker, bounded descriptors/transfers, and a host-PSRAM read
cache with sequential readahead. Linux owns the filesystem; the host does not
mount it concurrently. Kernel and rootfs work also enables ordinary block,
USB mass-storage/HID, and removable-media filesystem support.

The networking bridge increased RX queue capacity and completion batching,
bounded its service bursts, and adjusted its priority to coexist with the
ESP-Hosted pipeline. Those are fairness and throughput mechanisms, not proof
that networking has no regression under every concurrent workload. The
current board boot associated with the access point and activated the packet
path; a sustained mixed network/USB/display stress test was not performed in
this pass.

USB changes include more explicit PHY/port diagnostics, a noncoherent DMA
description, and a bounded DWC2 FIFO-layout patch. An optional full-speed-only
compatibility setting existed in the older experiment configurations. The
new production build disables that cap, and boot reports `limit=high-speed`.
There was no attached USB peripheral during this benchmark, so high-speed
configuration is verified, not a 480-Mbit/s transfer rate.

The experimental virtio-GPU bridge implements a standard 2D/KMS resource
path with CPU1/PPA conversion. It remains disabled in this production test.
It is not a 3D accelerator, and the mmap/fbdev game is not automatically
converted into a virtio-GPU client.

The board reports 32 MiB physical NOR, but the image header and partition
layout currently use only the lower 16 MiB. That upper capacity has not become
Linux RAM or a swap replacement. Moving an appropriate read-only rootfs to
flash could recover several MiB currently occupied by the RAM-rootfs, but it
requires a proper storage path and filesystem/layout changes. The active
guest build has neither SquashFS nor overlay support enabled. This work was
not implemented. Previous no-swap startup testing caused substantial major
fault activity and very slow startup; the benchmark still uses the existing
16 MiB SD swap file.

The kernel Image grew from 9,201,888 to 10,861,732 bytes; device trees were
regenerated for reserved memory and new virtio devices. Build guards check
image/DTB placement, and initramfs assembly now preserves the directories
needed before virtual filesystems are mounted. SSH provisioning/helper
changes remain in the working tree from earlier work, but SSH was not used
for the measurements in this pass.

## What changed in this latest pass and deployment state

This pass added generic interior-loop code generation and its tests, corrected
chain-entry profiler interpretation and flush accounting, and generated a
production configuration without the USB full-speed diagnostic cap. It did
not change the game's source, kernel Image, device tree, or partition layout.
The benchmark executable is an instrumented/compatibility build, not a claim
of an untouched upstream binary; earlier compatibility, RGB565, and timing
work must be distinguished from these emulator-side optimizations.

Application-only flash used `/dev/ttyACM0` at 921600. Console and benchmark
commands used `/dev/ttyUSB1`. The flash write was hash-verified. No SD contents,
bootloader, kernel, DTB, partition table, or Wi-Fi settings were erased by that
application-only flash.

Build: `build-rv32-jit-interior-production/uc-rv32ima.bin`

SHA-256: `fce261575ff5933a64a429cfef1da55afe8758f81396825b8ea98c5e02e20d44`

The SD benchmark executable's SHA-256 remained
`66304de72f65948eaad46284cc3f0bf02ea74ed78387f5dea7ab8a398b532285`
across the diagnostic and production measurements.

Size: 730,448 bytes; the 1 MiB application partition has about 30% free.
Runtime configuration: 360 MHz, 256 KiB L2, native JIT on, host/emulator
profiling off, legacy quiet period 30 ms, two 32 KiB display payloads,
virtio-GPU off, USB full-speed-only off. The ordinary Kconfig default still
leaves the experimental JIT opt-in; a fresh default build is not necessarily
identical to this measured configuration.

Exact benchmark invocation, after mounting SD and enabling its existing swap:

```sh
cd /mnt/sd/opt/doom/save
../doom-bench-640 -iwad ../freedoom1.wad -gfxmode rgb565 -nosound -nomusic >/tmp/doom.out 2>&1
cat /tmp/doom-benchmark.log
```

Hardware results for the new build, each a separate completed 100-frame run:

```text
frames=100 elapsed_ms=35205 fps_x100=284 mode=present
frames=100 elapsed_ms=31065 fps_x100=321 mode=present
frames=100 elapsed_ms=29670 fps_x100=337 mode=present
```

Total: 300 frames / 95.940 seconds = approximately 3.127 FPS, or 319.8 ms per
application-presented frame. The three runs returned to the shell without an
observed panic. The fastest run alone is not used as the overall result.
The post-run kernel-log check found no matching panic/oops/BUG/OOM/underrun
messages, and no benchmark process remained. This is a bounded check, not
proof of complete driver or electrical reliability.

## Remaining limits

The display-path acceleration is real, and the new production runs show a
modest further improvement, but the 5 FPS goal remains unmet. At the new
aggregate, reaching 200 ms/frame would still require removing approximately
120 ms/frame, or 37.5% of the current frame time. A roughly 13 ms final blit
cannot explain that entire gap. Local correctness tests cannot establish a
performance gain, and a clean boot cannot exclude all guest/JIT bugs or
physical display faults. The work remains uncommitted, with important new
files still untracked. Those measurements left the tested interior-loop build
on the board; subsequent experiments and deployment are recorded below.

## Follow-up: defer native-chain result publication

After the measurements above, a separate generic JIT improvement was built.
The previous compiler wrote the next PC, local instruction count and final
slot on every region exit, then cleared the local count and successor flag
before a successful native tail link. Those five stores are unnecessary when
execution remains inside the native chain: only the eventual return to C
consumes these result fields.

The compiler now publishes them at that return. Checked memory-miss stubs
explicitly seed their own local count, including zero for a straight-line
region whose prefix is represented by site metadata. This is essential:
simply deleting the old clears could reuse a stale count and repeat the class
of budget-accounting bug that previously caused a guest crash. C still
recovers the actual final slot before resolving a cold access. Guest register
commits, translation guards, the cumulative chain budget and device-service
boundaries are unchanged.

New tests deliberately seed stale results, enter directly or through two
native links, and exercise a warm load, a cold successful load, and a faulting
load. They also check empty links, wrong successor predictions, insufficient
instruction budgets at both the first and second handoff, and reused result
fields in dynamic interior loops with two read caches. Both production-mode
and profiling-mode RISC-V simulator self-tests pass. The separate `make test`
page-cache suite and `git diff --check` pass as well.

An eight-region synthetic chain executing 1,048,576 guest instructions
reported 2,016,700 simulator core writes before this change and 1,361,599
afterwards. Those totals include harness initialization and compilation, not
just generated code. The simulator reports core memory accesses, not measured
ESP32-P4 cycles; these numbers establish reduced bookkeeping traffic, not a
game FPS improvement. The successful-link code itself eliminates five result
stores and the slot-number materialization per handoff.

Candidate build: `build-rv32-jit-results-production/uc-rv32ima.bin`

SHA-256: `3da2799317538fe5fe4ff31b1ae62df88f05fd1f1f60abb4a560a2684c42f8bd`

Size: 732,080 bytes. Its generated SDK configuration compares byte-for-byte
equal to the measured interior-loop production configuration: 360 MHz,
256 KiB L2, the same display, USB and networking settings, and profiling off.
No game, kernel, DTB or partition changes are part of this follow-up.

The application-only flash subsequently completed through `/dev/ttyACM0` at
921600 and its written hash verified. The board passed the new native self-
tests, reached Linux, and initialized display, SD, networking and high-speed
USB host mode. The USB port had no attached device, so this does not establish
peripheral transfer performance. The SD executable hash remains unchanged.

Three ordinary 100-frame demo measurements of this build were:

```text
frames=100 elapsed_ms=29921 fps_x100=334 mode=present
frames=100 elapsed_ms=35279 fps_x100=283 mode=present
frames=100 elapsed_ms=29199 fps_x100=342 mode=present
```

Aggregate: 300 / 94.399 = approximately 3.178 FPS, or 314.66 ms per frame.
This is only about 1.6% above the preceding 3.127 FPS aggregate, with a much
larger run-to-run spread. It is not convincing evidence of an end-to-end
speedup, despite the demonstrated reduction in generated bookkeeping. No
matching panic/oops/BUG/OOM/underrun message or remaining benchmark process
was found in the post-run checks. Physical-panel correctness was requested
from the user; serial logs alone do not establish it.

## Rejected experiment: L2 prefetch for native code

Code inspection found that the existing next-line prefetch window starts at
Linux's first usable RAM address, excluding the separate 2 MiB JIT arena.
The experimental `CONFIG_RV32_JIT_CODE_PREFETCH` option added only that
allocator-checked arena as the second autoload range. It defaulted off and
depended on native JIT and guest-RAM prefetch. It changed neither cache
capacity nor memory reservations, guest instructions, clocks, DMA policy or
device addresses. It was subsequently removed from the active source after
the measurements below showed no benefit.

The hardware supports instruction/data preload, with automatic fetching
triggered by configured cache hits or misses; see Section 9.3.3.3 of the
[ESP32-P4 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-p4_technical_reference_manual_en.pdf).
The local ESP-IDF ROM header exposes four L2 autoload ranges, and the chip's
`hw_ver1` register definitions provide their enables, addresses and sizes.
This is an experiment using the same low-level ROM interface as the existing
RAM prefetch; the header explicitly does not present it as a supported SDK
application API. More prefetch traffic could hurt shared-cache or DMA
behavior, so capability is not evidence of a performance benefit.

The candidate reads back the control register and second range at boot to
verify the actual hardware configuration. Its generated SDK configuration
differs from the measured result-publication build only by the new enabled
option. `make test`, whitespace checks and the ESP-IDF build pass; the host
tests do not exercise the physical prefetch engine.

Candidate: `build-rv32-jit-prefetch-production/uc-rv32ima.bin`, 732,224 bytes.

SHA-256: `f262996859ef74049b904d77c35ccd70733beda6e58a3aec0564744a5c7fa444`

Before flashing the prefetch candidate, two separate 100-frame baselines on
the result-publication build used the existing game's `-timedemo demo1`
option to keep the demo sequence fixed:

```text
frames=100 elapsed_ms=31114 fps_x100=321 mode=present
frames=100 elapsed_ms=31102 fps_x100=321 mode=present
```

Aggregate: 200 / 62.216 = approximately 3.215 FPS. These two elapsed times
differ by only 12 ms, unlike the larger spread of ordinary-demo runs. Two
runs alone do not establish a universal noise bound. They are a separate
baseline for the prefetch experiment, not additional samples to merge into
the ordinary-demo result. No game source was changed for this mode. The
benchmark exited after 100 frames in both cases, and the post-run error-log
and remaining-process checks were again clear.

Exact command (same SD directory and existing swap as above):

```sh
../doom-bench-640 -iwad ../freedoom1.wad -gfxmode rgb565 -nosound -nomusic -timedemo demo1 >/tmp/doom-timedemo.out 2>&1
cat /tmp/doom-benchmark.log
```

The prefetch candidate was then application-only flashed at 921600. The
written hash verified, native boot self-tests passed, and hardware readback
confirmed the second range was enabled rather than merely requested:

```text
L2 JIT prefetch: control=00000311 base=48200000 size=2048 KiB
```

The same timedemo invocation produced:

```text
frames=100 elapsed_ms=35161 fps_x100=284 mode=present
frames=100 elapsed_ms=39075 fps_x100=255 mode=present
```

Aggregate: 200 / 74.236 = approximately 2.694 FPS, about 16.2% lower than the
initial non-prefetch timedemo baseline. The first run followed a fresh boot;
the second remained slow after that warm-up. Different boots and cache state
limit strict causal attribution, but neither measurement supports retaining
the experiment. Extra cache/bus traffic is a plausible explanation, not a
measured exclusive cause. The optional source/configuration changes were
removed; their build artifacts remain available for reproduction.

After the second run had completed and returned to the shell at 21:01 UTC,
a full MCU boot sequence appeared around 21:03 UTC. No preceding panic was
captured. This was not established as a prefetch fault or electrical fault:
the user was asked whether they had manually reset the board. The unreadable
ROM-speed UART bytes do not supply a trustworthy decoded reset cause.

The exact previous result-publication binary was restored directly, without
rebuilding it. Its SHA-256 remains
`3da2799317538fe5fe4ff31b1ae62df88f05fd1f1f60abb4a560a2684c42f8bd`.
The application-only write hash verified; boot again passed native self-tests
and reached Linux with display, SD, networking and USB host initialization.
Only the original Linux-RAM prefetch range is configured in this build.

Two completed post-restoration fixed-sequence measurements were:

```text
frames=100 elapsed_ms=31564 fps_x100=316 mode=present
frames=100 elapsed_ms=31550 fps_x100=316 mode=present
```

Aggregate: 200 / 63.114 = approximately 3.169 FPS, or 315.57 ms per frame.
These returned close to the initial non-prefetch timedemo baseline, rather
than retaining the prefetch candidate's slowdown. The final invocation also
explicitly reported exit status zero. The SD executable hash was rechecked
and remains unchanged. No benchmark process or matching guest-kernel
panic/oops/BUG/OOM/underrun message remained after the final run. The existing
SD swap had 5,456 KiB in use; the filesystem was synchronized. A subsequent
three-packet gateway ping received three replies. That is a basic network
liveness check, not concurrent I/O throughput validation.

Some intervening attempts were interrupted by full MCU boots and are not
counted as completed measurements. A boot occurred during benchmark startup
around 21:09 UTC, and another appeared after the first successful restored
run. The user reported manually resetting the board during this phase; the
events were not precisely correlated, so these observations cannot all be
classified as spontaneous faults. No trustworthy automatic reset cause or
preceding panic was decoded. A 48-second ROM-baud observation window captured
no reset event and therefore did not resolve its cause. The final run used
one miniterm descriptor with DTR/RTS inactive and completed without an observed
reboot. This neither establishes an electrical fault nor proves long-term
stability. Future physical reset requests use the user's one-word convention:
`RESET` for normal reset, `DOWN` for download mode.

The installed application remains the exact result-publication binary above.
No 5 FPS result or rigorous hardware performance ceiling has been established.

## Cache-capacity review after the restored measurements

The user asked whether adjustable L2 capacity and other cache optimizations
remain worth considering. This follow-up is a source/configuration/map review;
no additional cache configuration was flashed for it.

The active configuration selects 256 KiB L2 with 128-byte lines. The SDK
supports 128, 256 and 512 KiB capacity; 64-byte lines are selectable at 128 or
256 KiB, while 512 KiB requires 128-byte lines in this SDK. These are startup
configuration choices, not a safe live-register resize while the reclaimed
SRAM contains running firmware. See the matching
[ESP-IDF v6.1-rc1 cache configuration](https://github.com/espressif/esp-idf/blob/v6.1-rc1/components/esp_system/port/soc/esp32p4/Kconfig.cache).

The board boot reports chip revision v1.0, and its generated configuration
selects the pre-v3 memory layout. In that layout, the SDK defines the high
SRAM region's size as 512 KiB minus the selected L2 size. Consequently:

| L2 capacity | High SRAM region | Change from current internal-SRAM capacity |
| --- | --- | --- |
| 128 KiB | 384 KiB | +128 KiB |
| 256 KiB, current | 256 KiB | baseline |
| 512 KiB | zero | -256 KiB |

This follows the matching
[ESP-IDF memory layout](https://github.com/espressif/esp-idf/blob/v6.1-rc1/components/esp_system/ld/esp32p4/memory.ld.in),
not an assumption that cache capacity is free extra memory. The current map
reports 319,363 bytes of static DIRAM usage and 441,296 bytes of linker-region
capacity. Selecting 512 KiB L2 would reduce that capacity to 179,152 bytes:
the existing static usage exceeds it by about 137 KiB, even before runtime
stacks, DMA queues and heap allocations. Linker-reported remaining space is
not the same as runtime free heap. A 512 KiB trial therefore needs a material
placement redesign first; it is not an isolated configuration toggle for
this image.

There is also a concrete placement opportunity in the current image:
`main/link.lf` maps the entire JIT C object as `noflash`. The size report shows
42,284 bytes of that object's text in internal RAM. Three emitted self-test
functions alone account for 13,914 bytes at internal addresses. Retaining
those tests but moving boot-only/cold code to the existing external-code path
could recover some internal executable space for genuinely hot native code.
That is a candidate, not an implemented change or measured speedup; cache-off
call paths and placement safety must be checked before moving anything. The
running build currently allocates only a 16 KiB internal native-code tier.

The bounded next experiments worth considering are:

- Compare 64-byte and 128-byte lines at the current 256 KiB capacity, holding
  clocks, JIT limits, display mode and I/O configuration constant. Smaller
  lines may reduce unused refill traffic for scattered accesses but increase
  refill activity on sequential pixel traffic; neither outcome is assumed.
- Compare the existing Linux-RAM prefetch enabled and disabled with the
  current JIT. Rejecting additional native-code prefetch did not separately
  establish whether the original broad guest-RAM prefetch remains beneficial.
- Audit cold-code placement and hot JIT/data locality. If comparing 128 KiB
  L2 with more internal hot-code space, explicitly pin bulk-buffer placement:
  its Kconfig default changes at that capacity and would otherwise confound
  the comparison. Record the actually allocated hot-tier size at every boot.
- Measure instruction as well as data cache traffic with low-overhead
  hardware-counter snapshots. The current cache helper reads only DBUS0
  counters and is gated together with intrusive per-instruction profiling;
  its percentages are not a complete native-code cache analysis.

Use the same unchanged SD binary and fixed-sequence 100-frame interval for
these comparisons, repeated with a return-to-baseline check. Retain a change
only when FPS and display/I/O checks support it. The failed prefetch experiment
does not rule out cache optimization, but it does rule out assuming that more
prefetch automatically helps.

## Prepared experiment: 64-byte L2 cache lines

Built `build-rv32-jit-line64-production/uc-rv32ima.bin`, 732,208 bytes.

SHA-256: `91d03dadce8d16b7d4915c04421f13eae3bf4a4810987bd8ef7510c641ef6845`

The generated configuration at
`/tmp/esp32p4-rv32-line64-production-sdkconfig` differs from the measured
result-publication configuration only in the 64/128-byte cache-line choice
and its derived line-size value. L2 remains 256 KiB; clocks, profiling,
prefetch, bulk-buffer placement, JIT limits and device settings are unchanged.
The repository's default configuration remains on 128-byte lines. No game,
emulator, kernel, driver or SDK source was changed for this candidate.

The build used the measured production configuration as its first defaults
file and `/tmp/esp32p4-rv32-line64.defaults` as a small override. The JIT's
code-publication alignment remains 128 bytes, which satisfies either cache
line size. Display/PPA allocation and alignment checks use the SDK's cache
alignment query; cache synchronization remains in place. The host page-cache
test suite and the full ESP-IDF build pass. Physical PPA/DMA behavior and FPS
are not yet verified by those checks.

The size report shows 318,266 bytes of static DIRAM usage versus 319,363 in
the 128-byte baseline. That small layout difference is a consequence of the
configuration change, not a separately introduced memory-placement change.
The actual runtime hot-tier allocation must still be recorded at boot.

At preparation time `/dev/ttyACM0` was absent and `/dev/ttyUSB1` was present.
The candidate has not been flashed; the board remains on the previously
verified result-publication application. The next action is a coordinated
download-mode transition, application-only flash at 921600, boot/self-test
checks and repeated fixed-sequence 100-frame measurements.

The download device subsequently appeared as the expected ESP32-P4 on
`/dev/ttyACM0`. The application-only write completed at 921600 and its flash
hash verified around 21:44 UTC. The installed candidate is now the 64-byte-
line image with the SHA-256 above; no bootloader, partition table, kernel,
DTB or SD data was flashed.

Before that write, the UART monitor already received a continuous
undecodable stream while the board was in download mode. The stream also
prevented normal-boot console verification after the write. A bounded raw
probe with DTR/RTS inactive and input-buffer flushing delivered the same
repeating 13-byte pattern at both requested observation rates, 921600 and
115200:

```text
fb fb ff db fb ec f0 ff f0 f0 cd f0 f2
```

Each approximately one-second probe delivered 479,232 bytes. Those delivered
byte counts do not establish the physical UART bitrate or the MCU's reset
state; there is not enough evidence to distinguish a board/UART path problem
from a host-side serial problem. No decoded panic, boot self-test outcome or
100-frame measurement was obtained for this candidate. In particular, the
pre-flash occurrence cannot be attributed to its new cache-line setting.

The noisy unbounded monitors were stopped. A bounded console reader was
started on `/dev/ttyUSB1` at 921600 with DTR/RTS inactive, suppressing
non-ASCII/control-byte floods while retaining printable diagnostic text. It
is ready to capture a coordinated normal reset. Benchmarking must wait for
an intelligible boot/shell and display initialization checks.

Sending an empty line subsequently elicited the Linux root prompt, and a
following command returned guest uptime of 303.26 seconds around 21:49:37
UTC, consistent with the post-flash boot. The shell therefore became usable
without another flash, but that did not recover the earlier host boot log.
A coordinated normal reset is still needed to record native self-test status
and the actual internal hot-tier allocation before a controlled comparison.

The requested normal reset then produced an intelligible full host boot log,
captured around 21:52 UTC. It identified the candidate ELF SHA-256 prefix
`5bb1a5ea8` (matching the built ELF), CPU frequency 360 MHz, successful PSRAM
memory testing, a 64-byte-aligned display surface, and PPA scale/fill enabled.
The native JIT reported successful self-tests, the same 2 MiB main arena and
the same 16 KiB internal hot tier as the baseline. SD exposed the expected
block device and 700 KiB read cache; C6 associated and its packet path became
active. USB host initialized with its high-speed limit and no attached
peripheral. Linux reached its root shell. These observations resolve the
missing boot/configuration evidence, not the cause of the earlier UART noise
or long-term physical-panel reliability.

The SD was mounted, its existing 16 MiB swap enabled, and the unchanged game
executable SHA-256 verified again. Fixed-sequence 100-frame measurements of
this candidate are now in progress. Visual correctness was separately
requested from the user; an absence of serial errors is not substituted for
that observation.

The two completed 64-byte-line measurements were:

```text
benchmark_exit=0
frames=100 elapsed_ms=34407 fps_x100=290 mode=present
benchmark_exit=0
frames=100 elapsed_ms=39290 fps_x100=254 mode=present
```

Aggregate: 200 / 73.697 = approximately 2.714 FPS, about 14.4% below the
preceding restored 128-byte-line aggregate of 3.169 FPS. The second invocation
remained slower despite warm-up. Two runs and different guest/cache states
do not establish an exclusive microarchitectural cause, but neither run
supports adopting the smaller line size. The candidate is rejected on the
available performance evidence; repository defaults remain on 128-byte lines.

Post-run checks found no remaining benchmark process or matching guest-kernel
panic/oops/BUG/OOM/underrun message. The existing SD swap had 5,920 KiB in use.
Three gateway pings received three replies, and `sync` completed. Physical
rendering feedback was still pending at this point; these bounded checks do
not establish universal I/O or display correctness.

The preserved result-publication application was rehashed and still matches
`3da2799317538fe5fe4ff31b1ae62df88f05fd1f1f60abb4a560a2684c42f8bd`.
At approximately 22:00 UTC the next action is to restore that exact binary
through `/dev/ttyACM0` at 921600 and repeat the fixed-sequence benchmark.
The download device was absent at this check, so restoration is pending the
user's download-mode transition. The bounded UART console remains open to
capture the restoration boot.

The expected download port appeared again, and the exact preserved
result-publication application was restored around 22:06 UTC. Its written
flash hash verified. The continuous bounded console captured an intelligible
boot without an additional reset request: 360 MHz, successful native JIT
self-tests, 2 MiB main code arena plus 16 KiB internal hot tier, 128-byte
display alignment and PPA scale/fill enabled. SD and C6 networking initialized;
USB retained its high-speed setting with no attached peripheral. Linux
returned to its root shell. The two return-to-baseline measurements were:

```text
benchmark_exit=0
frames=100 elapsed_ms=37883 fps_x100=263 mode=present
benchmark_exit=0
frames=100 elapsed_ms=38249 fps_x100=261 mode=present
```

Aggregate: 200 / 76.132 = approximately 2.627 FPS. This is substantially below
the earlier 3.169 FPS result for the exact same binary, and slightly below
the intervening 64-byte candidate's 2.714 FPS. Therefore the return check did
**not** confirm the initial apparent 14.4% cache-line regression. The final
interpretation is that 64-byte lines have not demonstrated a reliable benefit;
this experiment does not prove they are intrinsically slower. The original
128-byte default remains in place. Earlier claims in this chronology must
not be read as a causal conclusion overriding this A/B/A result.

The post-run kernel-log and remaining-process checks were clear. The SD swap
had 5,968 KiB in use, the gateway answered three of three pings, and `sync`
completed. The executable hash remained unchanged. The captured game startup
log showed the requested demo, no compatible keyboard, the existing IWAD,
and a zone-memory address of `0x94f93040` for the last run. It reported that
the separate `demo1.lmp` file could not be opened and then printed
`Playing demo demo1.lmp.`; this check did not independently trace the IWAD
demo lookup.
No `.default.cfg` existed in the save directory when checked, despite the
startup message identifying it as the save target. The IWAD was still
28,795,076 bytes with its September 1 modification date; a full on-device
IWAD content hash was not obtained in this check.

The SDK startup source explicitly calls `Cache_Set_L2_Cache_Mode` with the
application's configured capacity and line size, then invalidates L2. The
display's reported alignment comes through `esp_cache_get_alignment`, the
HAL, and `Cache_Get_L2_Cache_Line_Size`, rather than just printing a requested
constant. Dynamic frequency scaling is disabled in the restored configuration.
These checks do not support a forgotten 64-byte mode or dynamic clock change
as an explanation for the slow return runs. They also do not identify the
actual cause: guest allocation/cache state, JIT residency and background work
remain possible contributors, not established diagnoses. The apparently
repeatable first two timedemo samples were not a general noise bound.

## Prepared experiment: move boot-only JIT tests out of internal RAM

While download mode was unavailable, a separate memory-placement candidate
was prepared. `main/link.lf` retains the JIT object's existing internal-RAM
placement but assigns its three boot-only self-test functions to the SDK's
normal cached code section. In this configuration that section executes from
PSRAM. The tests are not removed, weakened or skipped. Their only call site
is initialization, after external caches and device bridges are initialized;
the application still stops before Linux if any native self-test fails.

The linked ELF confirms these placements:

| Function | Previous internal address | New cached-PSRAM address |
| --- | --- | --- |
| `jit_self_test_chain_results` | `0x4ff1f6c6` | `0x48053068` |
| `jit_self_test_interior_loops` | `0x4ff1fc98` | `0x48053652` |
| `jit_self_test` | `0x4ff20700` | `0x480505ac` |

Among the linked `jit_*` and `rv32_native_jit_*` functions, only those three
are in the external executable range. Runtime entry, native entry assembly,
initialization and invalidation functions remain internal. This uses SDK
symbol-level linker mappings; function-local/pooled data need not follow
their parent functions, so actual map sizes are used rather than assuming
every test-related byte moved.

Static DIRAM usage falls from 319,363 to 305,342 bytes: 14,021 bytes recovered.
This includes incidental linker layout/relaxation effects. Total image size
increases slightly; neither the 2 MiB native-code reservation nor the
64 KiB internal runtime reserve is changed. Whether the recovered memory
allows a larger internal hot tier must be checked on hardware, and no FPS
improvement is claimed from the map alone.

Candidate: `build-rv32-jit-coldtests-production/uc-rv32ima.bin`, 732,512 bytes.

SHA-256: `6f2fc63ec3e40fa6c722e23034e7380451e3c670d5ea146704fa6ce5a810e89e`

Its generated configuration at
`/tmp/esp32p4-rv32-coldtests-production-sdkconfig` compares byte-for-byte equal
to the preserved result-publication configuration: 256 KiB L2, 128-byte lines,
360 MHz, profiling off and unchanged I/O settings. The JIT C source remains
unchanged with SHA-256
`026e169649b3cae743c35d35e4262f051742de95383f0a5bbe701d75001d4bf6`.
Only the linker-placement rules changed for this candidate; no game, guest
kernel, device tree, driver or SDK source was modified.

The ESP-IDF build, map/ELF checks, separate emulator page-cache test suite and
whitespace checks pass. At preparation time this candidate was unflashed and
the board was running the exact older result-publication binary. Do not
rebuild that preserved restore artifact with the new linker fragment and
then treat it as the original baseline.

### Hardware deployment and memory tradeoff

The download port subsequently became available. The cold-test placement
candidate was application-only flashed through `/dev/ttyACM0` at 921600 and
its written hash verified. By approximately 22:25 UTC the continuous console
had captured a successful boot, without another manual reset. The ELF hash
prefix `bb773f24d` matched the built candidate. PSRAM testing and native JIT
self-tests passed, CPU frequency remained 360 MHz, and Linux reached its
root shell. The display retained 128-byte alignment and PPA scale/fill;
C6 associated and activated its packet path. USB initialized with its
high-speed setting and no attached peripheral.

The recovered internal space increased the actual hot-code tier from
16 KiB to **32 KiB**, with the same 2 MiB main code arena and 64 KiB internal
runtime reserve. There is an external-memory tradeoff: the host SD read
cache decreased from **700 KiB (175 lines) to 652 KiB (163 lines)**. Moving
code into cached PSRAM changed the external allocation layout: the 30 MiB
allocation starts at `0x480ac504` instead of `0x480a0a08`, leaving less space
in the prefix used by the SD cache. This is not an unchanged-I/O-memory
comparison, despite the byte-for-byte identical configuration. The Linux
physical aperture remains `0x48400000..0x49dfffff`.

The existing SD filesystem and swap were mounted/enabled, and the benchmark
executable SHA-256 was verified unchanged. Three fixed-sequence measurements
completed successfully:

```text
benchmark_exit=0
frames=100 elapsed_ms=30739 fps_x100=325 mode=present
benchmark_exit=0
frames=100 elapsed_ms=32227 fps_x100=310 mode=present
benchmark_exit=0
frames=100 elapsed_ms=31022 fps_x100=322 mode=present
```

Aggregate: 300 / 93.988 = approximately **3.192 FPS**. This is faster than the
immediately preceding slow baseline pair (2.627 FPS), but effectively matches
the earlier exact-baseline aggregates of 3.169 and 3.215 FPS. Selecting only
the slow pair would exaggerate the evidence. No reliable game-speed gain is
established; neither is a general claim that a larger hot tier cannot help
other workloads. The SD cache lost 48 KiB of capacity, which is not equivalent
to a measured 6.9% I/O throughput loss.

Post-run checks found no remaining benchmark process or matching guest-kernel
panic/oops/BUG/OOM/underrun messages. The SD swap had 6,044 KiB in use. Three
gateway pings received three replies, and `sync` completed before requesting
download mode. The retained startup logs show zone addresses `0x94f86040`,
`0x94f2c040` and `0x94f7e040` for the three runs. Different addresses confirm
some guest allocation variation; they do not establish its performance cost.
Physical rendering feedback remains pending, and no USB-transfer or sustained
mixed-I/O throughput test was performed.

The candidate is not being adopted: the five newly added linker-placement
lines have been removed, retaining the pre-existing whole-JIT internal
placement. The candidate binary and its measurement evidence are preserved.
Restoration of the exact result-publication binary is pending download mode;
at approximately 22:34 UTC `/dev/ttyACM0` was absent. The board still runs the
cold-test candidate until that restoration is actually flashed and verified.

At approximately 22:37 UTC the same missing-download-device condition had
been revalidated across three consecutive goal turns. `/dev/ttyACM0` and its
sysfs TTY entry were absent; the prior USB enumeration also contained no P4
download device. The bounded console session remained live, and Linux had
answered an uptime query during the preceding check. The preserved restore
binary still matched its recorded SHA-256. Work is blocked on the user's
physical download-mode transition, not marked complete. No flash was attempted
through the FTDI console as a substitute.

Download mode appeared after the goal was resumed. Around 22:39 UTC the exact
preserved result-publication application was restored through `/dev/ttyACM0`
at 921600. Esptool verified the written hash and exited successfully. The
continuous console captured the matching ELF prefix `9036b4c85`, 360 MHz,
successful PSRAM and JIT self-tests, the restored 16 KiB internal hot tier,
and the restored 700 KiB SD read cache. Display alignment remained 128 bytes
with PPA scale/fill, C6 associated and activated its packet path, and USB
remained high-speed configured with no attached peripheral. Linux reached
its shell. The download blocker is cleared and the cold-test firmware is no
longer installed. A return-to-baseline 100-frame check follows.

That unchanged-executable check completed with `benchmark_exit=0` and
`frames=100 elapsed_ms=31039 fps_x100=322 mode=present`: approximately
3.222 FPS. It is consistent with retaining the baseline over the cold-test
candidate's 3.192 FPS aggregate; one return run is not a tight confidence
interval. Postchecks found no remaining game process or matching kernel
panic/oops/BUG/OOM/underrun message, 5,496 KiB of swap in use, three gateway
replies from three pings, and a completed `sync`. The startup zone address was
`0x94f3e040`. Physical rendering/reset feedback was requested separately and
remains unconfirmed.

### Cache conclusions so far

Keep 256 KiB L2 and 128-byte lines as the established configuration. Neither
the smaller-line experiment nor the doubled internal hot tier demonstrated a
reliable end-to-end gain under the observed run-to-run and boot-to-boot
variation. Additional native-code prefetch was also withdrawn after its
slower measurements. These results do not prove an absolute hardware ceiling
or rule out future generic compiler/locality improvements.

Before more capacity/prefetch changes, the most useful missing evidence is
low-overhead instruction- and data-side miss/refill counters synchronized to
the same 100-frame interval, plus paging/SD activity. The existing DBUS0-only
helper combined with intrusive profiling cannot distinguish those costs
cleanly. Another firmware variant is not justified merely because a cache
setting is adjustable. A 512 KiB L2 trial would additionally require a major
internal-memory placement redesign while preserving DMA and I/O headroom.

## Hardware-only cache sampler and guest-RAM prefetch comparison

Added opt-in `CONFIG_RV32_CACHE_PERF_STATS`, default off and mutually exclusive
with the existing instruction-instrumenting emulator profiler. It samples
the named L1/L2 instruction/data bus counters for both host cores once per
second and enables the existing per-request block-cache counters. It does not
enable per-instruction, JIT-entry, or display-operation instrumentation.
`main/host_cache_profile.c` remains cached code; the existing outer guest-batch
boundary checks the reporting deadline. No game, guest kernel, DTB, partition,
JIT C source, cache capacity, line size, or CPU-frequency change is involved.

The sampler freezes only its counter-enable bits before reading, clears the
write-trigger counter-reset bits, then resumes counting after printing. It
does not disable caches or invalidate their contents. L1/L2 freezes are not
an atomic cross-level snapshot. SD fields are individually atomic, not one
transactional snapshot. Reports retain raw window start/end times, span,
late-window indication and the preceding report's wall-clock cost. Their
windows exclude printing but application FPS still includes that overhead.
The 32-bit hardware counters must not be trusted across unusually long gaps.

The matching SDK
[register definitions](https://github.com/espressif/esp-idf/blob/v6.1-rc1/components/soc/esp32p4/register/hw_ver1/soc/cache_reg.h)
distinguish L1-ICache0/1 traffic from L1 data-bus traffic at L2. Their hit/miss
labels are retained as raw hardware fields. They are not calibrated here into
unique misses, lost CPU cycles, byte bandwidth or a hardware FPS ceiling.
In particular, the observed miss and next-level-read counts differ greatly;
dividing hit by hit-plus-miss is not independently validated as a conventional
unique-access hit rate.

Full ESP-IDF builds with sampling on and off passed, as did the separate
emulator page-cache tests and whitespace checks. With sampling off, the
linked `app_main` disassembly matches the preserved baseline exactly
(SHA-256 of the extracted disassembly:
`e456459b340a4af0e9741236b18744f627cb6adafb38b16f987228a0d23d40a2`).
The sampling symbols are absent from that off build. This is a specific
code-path check, not a claim that its timestamped whole image is identical.

### Prefetch-on sampled baseline

Application: `build-rv32-cache-counters-profile/uc-rv32ima.bin`, 734,528 bytes.

SHA-256: `eb2db4af4adec1fd39fb08c463cd8d052f398bb22263f527fd21e2d5c0c817fc`

ELF SHA-256: `5877af340fb539fad1048f4c625ebba418fca27564797fcd5b67987fa1fe9a20`

Its configuration differs from the restored production configuration only by
the sampler flag. Static DIRAM rises from 319,363 to 320,118 bytes. The actual
boot retained the 16 KiB internal hot tier and 700 KiB SD cache, with unchanged
guest/scanout allocation addresses. Flash hash verification, PSRAM tests and
native self-tests passed. Linux, display, C6 and the unattached USB host
initialized. The unchanged benchmark executable hash was verified again.

```text
benchmark_exit=0
frames=100 elapsed_ms=32401 fps_x100=308 mode=present
benchmark_exit=0
frames=100 elapsed_ms=31911 fps_x100=313 mode=present
```

Aggregate: 200 / 64.312 = approximately **3.110 FPS**. Compared with the last
production return run's 3.222 FPS, this is about 3.5% lower; the known baseline
variation prevents attributing all of that difference to the sampler. Its
direct report calls typically cost about 5–5.7 ms per second. That measured
call cost is not the complete instrumentation, layout, or application cost.

Raw bounded-console output is preserved in
`docs/cache-counters-prefetch-on-2026-09-04.log`. It contains the boot, commands,
both runs and postchecks, with UART carriage returns normalized to newlines.
Physical resets/download transitions were coordinated; a suppressed-control-
byte notice is not itself a decoded panic or hardware-fault diagnosis.

For one explicitly delimited late-run subset, all 23 complete sample windows
inside host time 155–179 seconds cover 22.912314 seconds. Their counted L2
external reads are I0=495,611, D0=743,532, I1=20,339, D1=5,097. CPU0 therefore
accounts for approximately 98% of these CPU-bus reads, with instruction
fetches around 40% of its share. These counters do not include every DMA
consumer, notably continuously scanned display traffic. The subset has 61
block requests, 241,664 read bytes and 69,632 written bytes: comparatively
light but nonzero SD activity. These are interior sampling windows, not an
exactly frame-gated sum for all 100 presents. Startup and exit must not be
mixed into a rendering-only aggregate.

After both runs, no benchmark process or matching kernel error remained,
swap use was 6,028 KiB, three gateway pings received three replies, and `sync`
completed. Startup zone addresses were `0x94ffc040` and `0x94f2a040`. Physical
panel feedback remains pending; no sustained USB/mixed-I/O throughput claim
is made from these checks.

### Prefetch-off sampled candidate

Application: `build-rv32-cache-counters-noautoload/uc-rv32ima.bin`, 734,448 bytes.

SHA-256: `8a03145638631450a688c88046d152e426aa92de76b0cfbfbbbef6185dde18ff`

ELF SHA-256: `d4522fca1e52e0fe6cf9432ad9749adb9d4d80ea76eed8c4b0d46c7e3dc9004e`

Its configuration differs from the sampled baseline only by disabling
`CONFIG_RV32_GUEST_RAM_PREFETCH`. The disabled branch in `port-esp.c` now
explicitly calls `Cache_Disable_L2_Cache_Autoload()` and prints that state;
it does not merely omit the enabling/configuring call. The normal enabled
branch is unchanged. This is the broad Linux-RAM sequential prefetch, not
the earlier rejected extra native-code prefetch experiment.

The candidate was flashed through `/dev/ttyACM0` at 921600 around 22:59 UTC
and its written hash verified. Boot reported the matching ELF prefix,
360 MHz, successful PSRAM/JIT tests, 16 KiB internal hot tier, 700 KiB SD
cache, and `L2 next-line prefetch: disabled`. Guest and display allocation
addresses are unchanged. Linux reached its shell, SD and its existing swap
were mounted/enabled, and the benchmark executable hash matched. Repeated
100-frame measurements then completed:

```text
benchmark_exit=0
frames=100 elapsed_ms=31471 fps_x100=317 mode=present
benchmark_exit=0
frames=100 elapsed_ms=31669 fps_x100=315 mode=present
```

Aggregate: 200 / 63.140 = approximately **3.168 FPS**, about 1.9% above the
sampled prefetch-on pair's 3.110 FPS. The previously demonstrated baseline
variation is larger than this difference; the result does not establish a
reliable improvement or justify changing the production default. A sampled
prefetch-on return check is pending before restoring the non-sampling
production firmware.

For an illustrative counter comparison, select complete windows within
26 to 3 seconds before the last cache report preceding each first run's
successful process exit. This leaves 22 interior windows per variant,
21.915840 seconds with prefetch on and 21.913036 seconds with it off. It is
not an exact frame-aligned A/B interval. The rates are raw L2 fields per
sampled second:

| CPU0 counter field | Prefetch on | Prefetch off |
| --- | ---: | ---: |
| Instruction-bus next-level reads | 21,730 | 18,258 |
| Data-bus next-level reads | 32,512 | 80,346 |
| Data-bus next-level writebacks | 6,389 | 31,682 |

The L2 conflict fields are zero in those prefetch-off windows but nonzero
with prefetch on. Different counter activity is not automatically better
performance. In particular, these CPU-bus counters are not calibrated as an
accounting of every autoloader or DMA transaction; the table cannot be used
to claim a corresponding total PSRAM-bandwidth increase or decrease, saved
stall cycles, or an attainable FPS ceiling.

The raw prefetch-off capture is preserved in
`docs/cache-counters-prefetch-off-2026-09-04.log`. After both runs, no remaining
game process or matching kernel error was found, swap use was 6,032 KiB,
all three gateway pings replied and `sync` completed. Startup zone addresses
were `0x94f5d040` and `0x94f21040`. Physical display/reset feedback remains
pending. At approximately 23:06 UTC the next download device was absent;
the board still runs the prefetch-off diagnostic candidate, not restored
production. Both exact return binaries have been rehashed and preserved.

### Return-control result and final interpretation

Download mode became available and the exact sampled prefetch-on application
(`eb2db4af...`) was restored around 23:08 UTC. Its written hash verified;
boot again identified ELF prefix `5877af340`, CPU 360 MHz, successful PSRAM
and native self-tests, 16 KiB internal hot tier, 700 KiB SD cache, and the
enabled guest-RAM prefetch window. No extra reset was needed. After mounting
SD/swap and verifying the unchanged executable hash, the return runs were:

```text
benchmark_exit=0
frames=100 elapsed_ms=38659 fps_x100=258 mode=present
benchmark_exit=0
frames=100 elapsed_ms=38446 fps_x100=260 mode=present
```

Aggregate: 200 / 77.105 = approximately **2.594 FPS**. The same sampled
prefetch-on binary previously averaged 3.110 FPS. This reproduces substantial
boot-to-boot variation and prevents treating the intervening prefetch-off
pair's 1.9% advantage as a reliable isolated gain. Selecting only the slow
return pair would instead exaggerate that supposed benefit. The experiment
is inconclusive and does not justify changing the original prefetch default.

The first slower return run's equivalent interior subset (22 complete
windows, 21.917286 seconds) recorded about 18,816 CPU0 instruction-bus and
27,089 CPU0 data-bus external-read events per sampled second, lower than the
first faster prefetch-on run's 21,730 and 32,512. This does not support a
simple claim of increased refill rate causing the slowdown. Rates are not
work-normalized and these raw events do not establish stall cycles, retired
instructions, native-code coverage, or a hardware performance ceiling.
Allocation state, generated-code residency/placement and guest execution
mix remain hypotheses, not diagnosed causes.

The return capture is preserved in
`docs/cache-counters-prefetch-return-2026-09-04.log`, starting at the new boot;
unconsumed idle output from the preceding firmware was excluded. Postchecks
found no remaining game process or matching kernel error. Swap use was
6,040 KiB, all three gateway pings replied, and `sync` completed. Zone addresses
were `0x94fd2040` and `0x94f55040`. Physical display/reset feedback is still
pending and no sustained USB/mixed-I/O throughput test has been performed.

At approximately 23:16 UTC the sampled prefetch-on control remains installed.
Restoring the exact non-sampling production binary (`3da27993...`) is the
remaining deployment step; `/dev/ttyACM0` was absent at the last check. The
new sampler stays opt-in/default-off in source, and no cache-capacity,
cache-line, or prefetch-default change is being adopted from these tests.

At approximately 23:19 UTC, the missing download device had been revalidated
across three consecutive goal turns. Both its device node and sysfs entry
were absent, and the preceding USB enumeration had no P4 download device.
The console session remained live with advancing diagnostic windows, and
the exact production restore binary still matched its recorded hash. The
goal was then blocked on the physical download-mode transition, not complete.
The prefetch-on diagnostic firmware remained installed until the subsequent
restoration described below was actually verified.

### Final non-sampling production restoration

Download mode became available around 23:27 UTC. The preserved production
application was written through `/dev/ttyACM0` at 921600 baud; esptool exited
zero and verified the written data hash. Only the application was written,
not the bootloader, partitions, kernel, DTB or SD contents.

Application: `build-rv32-jit-results-production/uc-rv32ima.bin`, 732,080 bytes.

SHA-256: `3da2799317538fe5fe4ff31b1ae62df88f05fd1f1f60abb4a560a2684c42f8bd`

The captured boot identified the expected ELF prefix `9036b4c85` and compile
time 20:38:24, CPU 360 MHz, successful PSRAM and native-JIT self-tests, 16 KiB
internal hot tier, 700 KiB SD cache, and the enabled guest-RAM prefetch window.
The preserved build has 256 KiB L2 and 128-byte lines, with sampling off.
Linux reached its shell. Display/PPA, C6 networking and the unattached
high-speed USB host initialized. No extra physical reset was requested.

The first long setup command had no observed guest response. A subsequent
blank line elicited a prompt and the board answered two host ICMP probes;
there is no evidence here of a kernel panic. Retrying setup mounted SD/swap,
but its hash-command filename arrived truncated as `../doom-benc40`.
Repeating the short hash command verified the unchanged benchmark executable:

`66304de72f65948eaad46284cc3f0bf02ea74ed78387f5dea7ab8a398b532285`

The command-input loss was observed, but its cause is not diagnosed: it is
not proof of electrical noise, a UART overrun, or an L2 problem. The benchmark
command was then entered using short shell-continuation lines, with each
echo verified. UART baud and firmware remained unchanged. No diagnostics or
other guest commands ran during the timed invocation, and no SSH was used.

The same fixed-sequence command (`-iwad ../freedoom1.wad -gfxmode rgb565
-nosound -nomusic -timedemo demo1`) redirected output to the distinct
`/tmp/doom-final-production.out`. The shell saved the exit status immediately;
the benchmark log was read only after confirming success:

```text
benchmark_exit=0
frames=100 elapsed_ms=31351 fps_x100=318 mode=present
```

100 / 31.351 = approximately **3.190 FPS**. This is one final restoration
check, not a new multi-run average or a selected faster reboot. Loading and
startup remain outside the 100-present interval; the counter does not
independently verify 100 distinct physical-panel presentations. No 5 FPS
result or absolute hardware ceiling has been demonstrated.

Postchecks through approximately 23:38 UTC found no remaining benchmark
process and no `panic`, `oops`, `BUG:`, `out of memory` or `underrun` match in
the kernel log. Swap use was 5,672 KiB; three gateway pings all replied and
`sync` completed. Startup recorded zone address `0x94f67040` and playback of
`demo1.lmp`. These are bounded liveness checks, not sustained networking or
USB-throughput tests. No USB peripheral was attached. The full printable UART
capture, including the input anomaly rather than silently discarding it, is
preserved in `docs/cache-final-production-restore-2026-09-04.log`.

The local `make test` page-cache suite and `git diff --check` passed again.
The JIT C source hash and preserved production application hash still match
their prior records. The sampler is retained as an opt-in, default-off source
diagnostic, but is not present in the restored production binary. No further
cache variant is queued. Before attributing small FPS differences to cache
settings, the unexplained same-image variability needs controlled,
work-normalized investigation; simply enlarging L2 is not established as the
answer. Physical display/reset behavior still requires the user's observation.

At approximately 23:42–23:43 UTC, the same UART process was confirmed live
and the guest answered fresh short commands. It reported 837.05 seconds of
uptime and 5,632 KiB swap use; a repeated kernel-error search again returned
no matches. No new unsolicited fault or boot output appeared in the captured
follow-up. This supports continued guest liveness, not physical-panel health.
The observations are in `docs/cache-production-idle-followup-2026-09-04.log`.
The local backend calls the panel-reset function during initialization, while
its ordinary present function performs no panel reset. This does not rule
out a panel-only fault, power/link problem, or an incorrect rendered frame.
The missing physical observation has now persisted into a second goal turn;
production firmware is unchanged and no further speculative variant is queued.

At 23:44 UTC, a third consecutive goal turn revalidated the same missing
physical-panel observation. The existing console process was confirmed live,
no new unsolicited fault output appeared, and a fresh guest command reported
952.29 seconds of uptime. The preserved production image still matched its
recorded hash. The goal is now marked blocked awaiting the user's display
observation, not complete. This does not turn the 3.190 FPS result into a
hardware ceiling or claim that the display/reset defect has been resolved.

## 2026-09-05 replay and remaining execution-side candidates

At the user's request, the same 100-frame fixed-sequence demo was replayed
without flashing or changing any firmware/game settings. The existing UART
session and guest shell were revalidated; no game process was already running,
the working directory was `/mnt/sd/opt/doom/save`, and guest uptime was
4,166.35 seconds. SD and swap were already mounted. The command retained
`-iwad ../freedoom1.wad -gfxmode rgb565 -nosound -nomusic -timedemo demo1`,
using short verified shell-continuation lines and the new output path
`/tmp/doom-replay.out`. No SSH or concurrent guest diagnostics were used.

```text
benchmark_exit=0
frames=100 elapsed_ms=31687 fps_x100=315 mode=present
```

The unchanged executable hash was checked again after completion:
`66304de72f65948eaad46284cc3f0bf02ea74ed78387f5dea7ab8a398b532285`.
The printable capture is in `docs/production-replay-2026-09-05.log`.
100 / 31.687 is approximately 3.156 FPS, or 316.87 ms per application-presented
frame. The user reported: "coloring is accurate, refreshing is decent, no
corruption", then clarified that the actual rendering/processing rate remains
too low. This clears the missing observation for this replay; it does not
establish long-duration reliability or diagnose an electrical cause for every
earlier fault. No further code or firmware change was made in this replay.

Read-only inspection in response to the user's question about remaining
optimizations identified these candidates, not implemented improvements:

- Guest-register traffic at native links: the compiler loads each mapped
  guest register in the region preamble and commits each dirty mapping before
  even a successful native tail link. Result-publication stores were already
  reduced, but this separate register traffic remains. A bounded first step
  is skipping old-value loads only where a dominating write makes them
  unnecessary before every possible fault/exit; a larger step is compatible
  register mappings or transfer stubs between linked regions. Precise fault
  state, cold-memory exits and service budgets must remain correct.
- Internal hot-tier selection: `jit_dispatch_code()` increments each
  region's admission counter, but generated native links bypass that C-side
  function. The optional runtime entry counter counts linked entries only in
  diagnostic builds and is not a per-region promotion signal. Consequently,
  a frequently executed linked target can be undercounted by the admission
  policy. How often this affects the current workload is not measured; adding
  per-entry stores blindly could negate the benefit. Sampling or bounded
  link-aware admission needs a controlled comparison.
- Memory-guard work: existing fixed-base groups and two loop-read caches
  already amortize some translation work. Remaining loop accesses still
  calculate a guest page and check alignment/tag before native access. Where
  compiler analysis proves the induction range remains within one validated
  page, bounded chunk-level guards may remove repeated work without weakening
  MMIO, permissions, dirty reporting or page-fault behavior. This is a new
  proof/code-generation task, not permission to delete the current guards.

These are generic emulator/compiler opportunities, not game-specific
patterns. No new FPS gain is claimed. Reaching 5 FPS from this replay would
require lowering frame time from 316.87 to 200 ms, about a 37% reduction;
the earlier approximately 13 ms final-blit measurement cannot explain that
whole gap. Display settings remain unchanged. The fresh user request is to
identify candidates, not to flash another unmeasured configuration.

The user's subsequent request to dive into candidates 2/3 authorized a
bounded implementation/test pass. Its results and current deployment state
are recorded in [the follow-up audit](jit-shortcuts-2026-09-05.md); the
preceding candidate list describes the state before that implementation.
