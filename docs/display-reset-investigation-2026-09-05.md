# Display glitch and later device reboot — 2026-09-05

Latest result, about 12:55 UTC: reconnecting FTDI TX restored interactive
UART control without a firmware change or another reset. The unchanged
100-present benchmark completed in 44.292 seconds (2.2577 FPS). A separate
bounded normal-game replay confirmed 640x400 RGB565 output and about 146
seconds of uninterrupted game framebuffer ownership. It stopped by the
planned SIGTERM guard, not a captured crash. Kernel/ping/sync postchecks
passed, but the user's visual verdict and the earlier fault's cause remain
unresolved. The board is at its shell, SD/swap mounted, UART live at 921600.

Current status, about 12:33 UTC: work is blocked after three consecutive
resumed goal turns without interactive UART control. An existing SSH service
was reachable, but strict host-key verification rejected its changed key
before executing a read-only console query. No trust override, known-host
edit, firmware change, reset or game launch was performed. The serial reader
has been closed cleanly so the user can check the physical input connection.

Latest observation, about 12:28 UTC: the user's retry and requested manual
reset yielded a complete boot capture. The expected diagnostic build and
Linux network responsiveness are verified, but UART commands still receive
no response. Physical FTDI TX-to-MCU RX/common-ground checking is requested;
no additional reset or reflash is needed now. The longer game run has not
started, and neither the performance objective nor fault diagnosis is complete.

Current status, about 11:57 UTC: hardware work is blocked pending restoration
of usable FTDI UART access. The same blocker persisted through three
consecutive goal turns. No claim of 5 FPS, a hardware ceiling, or a crash fix
is made. The display-diagnostic firmware remains the last deployed image.

Latest retry, about 11:54 UTC: UART temporarily recovered and a fresh Linux
boot was captured, but another laptop suspend/resume interrupted access.
The re-enumerated FTDI adapter now returns repetitive unreadable bytes at
both tested baud rates. The longer game run has still not started. Physical
FTDI reconnection is needed before continuing; no MCU reset was requested.

Latest observation, about 11:18 UTC: the 100-present display-diagnostic run
completed at 2.793 FPS, with only expected ownership transitions and no new
captured reset. A planned longer run did not start: the FTDI UART adapter
disconnected during the laptop's suspend/resume interval and remains absent.
Restoring UART access is the next requirement, not a board reset or reflash.

Latest deployment, about 10:49 UTC: the event-only display-ownership diagnostic
was application-only flashed after the exact Espressif download interface
appeared. Written hash verification and boot checks passed. No pre-connection
reset was used; the post-flash reset was initiated from the flasher. The first
display event was a normal terminal takeover with two input bytes. Neither
the intermittent screen glitch nor the earlier whole-device reboot is fixed.

## Current evidence

The brief periodic screen event is unresolved. The user described an event
roughly once per minute despite normally correct colors and refreshing,
and suspected cache/buffer handling. They subsequently reported a device
reboot around 02:09 UTC; a fresh ESP-IDF boot banner and Linux shell were
captured independently. Treat those as potentially different symptoms.

The alignment-only JIT image was installed during this observation:
`build-rv32-jit-alignment-production/uc-rv32ima.bin`, SHA256
`58b76227dce9d7bf62ce6831547bba7381161644516b5d9270b32d3440c08612`.
Its three 100-present measurements completed successfully before the reboot:
31.198, 35.900 and 30.635 seconds, aggregate 3.0696 FPS. That is not a proven
gain over the 2.9990 FPS control given the variation, and it does not prove
long-duration reliability. The FPS objective remains unmet.

Before the reboot, Linux uptimes of 558.79 and 967.42 seconds were read.
After the new boot, uptime was 107.18 seconds. The assistant issued no reset
or flash command at the event. Its UART process was still live, configured
at 921600 baud with DTR/RTS inactive. Whether the user pressed reset or
reconnected power was asked but has not yet been confirmed or excluded.

The UART capture contains two boot sequences: the deliberate flash boot and
the later reboot. There is no readable panic, brownout, DSI-underrun or
frame-transfer-failure message preceding the latter in the saved output.
This is limited evidence: ROM output precedes the application's baud setup,
the bounded reader filters non-ASCII/control bytes, and one post-reboot chunk
was output-truncated. Core dumps are disabled. Missing logs do not establish
an electrical cause or exclude a software/watchdog fault.

Raw output is in `jit-alignment-uart-2026-09-05.log`. No benchmark was
restarted after the unexpected new boot. Its /tmp game startup logs were
lost before zone-address collection; earlier completed benchmark results
and postchecks are already preserved in the UART capture.

## Read-only display-path findings

- In the selected ST7703 backend, panel reset/init occur in
  `display_backend_init()` only. Backlight setup is also initialization-only.
  No periodic panel reset or backlight-off action was found in that backend.
- The SDK's DSI bridge underrun interrupt is enabled. Its handler emits an
  error if it cannot fetch external-memory pixels quickly enough. Logging is
  enabled in this configuration, but no such error was captured. This is
  not a measurement of every possible DSI/PHY/panel failure.
- PPA presentation is a blocking copy into the one continuously scanned
  buffer. The SDK writes back the input window and invalidates the aligned
  output window. There is already explicit CPU/DMA synchronization; blindly
  adding cache flushes is not a diagnosis. Espressif documents the distinction
  between writeback before DMA reads and invalidation before CPU reads of
  DMA-updated memory in its [memory-synchronization guide](https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32p4/api-reference/system/mm_sync.html).
- The current application/SDK synchronization and panel code have not been
  changed during this investigation. Signal integrity, panel power and
  buffer corruption remain hypotheses, not findings about this hardware.

Relevant local sources are `main/display_backend_st7703.c`,
`main/display_bridge.c`, the installed SDK's
`components/esp_lcd/dsi/esp_lcd_panel_dpi.c`,
`components/esp_driver_ppa/src/ppa_srm.c`, and
`components/esp_mm/esp_cache_msync.c`.

## Reproduced software hazard: empty terminal wake reclaims the framebuffer

`display_terminal_process_stream()` changes ownership to the terminal and
marks every terminal row dirty before attempting to receive any bytes.
`display_service_task()` calls it on a terminal notification even if the
stream is empty. Notifications are not a guarantee of remaining input:
the producer enqueues before notifying, while the consumer can also notice
and drain input during a VSYNC wake. A pending notification can consequently
outlive the bytes that caused it.

An empty stream with raw-framebuffer ownership is enough to reproduce the
unwanted redraw request in the current function. The function was copied
verbatim into `display-empty-wake-repro-2026-09-05.c`; queue and rendering
helpers are controlled host-test doubles, not a simulation of the P4's DMA.

```text
empty_queue=1 reclaimed=1 marked_rows=45 flush_calls=1 parsed_bytes=0 receives=1 pending=0
```

The reproduction exits zero when it observes that unwanted behavior. It
does not constitute a passing fix test. The stubs count redraw requests;
they do not verify pixels on the physical panel. No actual stale-notification
event has yet been correlated with the user's brief glitch, and this path
does not explain the later whole-MCU reboot. No ownership fix was applied.

The local compiled reproduction is
`/tmp/rv32-display-owner.E9uAFk/repro`. The source is also saved beside this
document so the counterexample is not dependent on a transient /tmp file.

## Boot-only reset diagnostics deployed

`main/uc-rv32ima.c` now prints `esp_reset_reason()` plus the raw ROM reasons
for CPU0/CPU1 at the start of `app_main()`. ESP-IDF saves its interpreted
reason during startup; the raw values preserve additional platform detail.
The API is documented in Espressif's [reset-reason reference](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-reference/system/misc_system_api.html#reset-reason).

The output format is:

```text
Reset diagnostic: idf=<decimal enum> raw_cpu0=0x<hex> raw_cpu1=0x<hex>
```

This reports the reason for that boot, not a history of resets. Flashing
this new image causes a deliberate reset and cannot retroactively recover
the earlier event's reason. Do not attribute the first post-flash reset
to the spontaneous-fault hypothesis. There is no periodic logging or
change to watchdog, clock, cache, display, network, USB or kernel settings.

The new image built successfully, has zero semantic SDK-configuration
differences from the measured alignment-only build, and passed the separate
emulator page-cache test and `git diff --check`. The JIT source is unchanged
from the previously simulator/hardware-tested alignment-only implementation.

- Build: `build-rv32-reset-diagnostic/uc-rv32ima.bin`
- Application SHA256: `472912b0573729a130dbaf29c620c24dc2d153cd3dcde9541e3d1f5a9612f313`
- ELF SHA256: `562d05c07a37b9c0e97c24cfbd1dc82d617abfec39fecc1b15d39e3b0c6624d5`
- Application bytes: 732,816; static DIRAM: 320,100, 126 above alignment-only.
- App-source SHA256: `732323bdca39032ac4a1e39e1ea82652336f314bdbc31982f2225cd43f58db16`
- Previous app-source SHA256: `8cb66c02053b8244bba76a47a4ecccbb33766a61588f97e6a62ff3ed4a93fdcd`
- JIT-source SHA256: `8651a1924f74edf293162b70efa619f32bd211f816708e45f1cf5348b5c9ae1d`
- Configuration: `/tmp/esp32p4-rv32-reset-diagnostic-sdkconfig`.

Download mode became available and the application-only flash through
`/dev/ttyACM0` at 921600 completed with hash verification around 02:20 UTC.
The full startup capture reports the expected ELF prefix `562d05c07`, passing
PSRAM/JIT self-tests, unchanged 360 MHz CPU, 16 KiB hot tier and 700 KiB SD
cache, and a Linux shell. The first report was:

```text
Reset diagnostic: idf=1 raw_cpu0=0x1 raw_cpu1=0x1
```

SDK classification 1 is `ESP_RST_POWERON`. This was the known post-flash
boot, not the earlier reported reboot; it does not establish a power fault.
The console remained attached with DTR/RTS inactive. At the first follow-up
check, Linux uptime was 308.69 seconds and no additional reset report had
appeared. This is a short idle observation, not a reliability result.
The deployed image is now the reset-diagnostic binary above. Startup and
observation output are in `reset-diagnostic-uart-2026-09-05.log`.
The glitch's visual appearance and whether the 02:09 reboot was spontaneous
remain unanswered questions. Neither fault is diagnosed or fixed yet.

## One bounded diagnostic workload

After the idle observation, SD and the existing swap file were mounted.
The unchanged `doom-bench-640` SHA256 was verified as
`66304de72f65948eaad46284cc3f0bf02ea74ed78387f5dea7ab8a398b532285`.
One identical `demo1` run at RGB565, without sound/music, completed with exit
status zero. No guest diagnostic commands ran during the timed interval:

```text
frames=100 elapsed_ms=31022 fps_x100=322 mode=present
```

That is 3.2235 FPS calculated from 100 application-present calls, not an
independent measurement of physical VSYNC frames. This is a diagnostic
workload, not a new optimization comparison; its timing is within the prior
range. It neither proves an improvement nor resolves the 5 FPS objective.

Postchecks found no remaining game PID, no matching Linux panic/oops/BUG/OOM
messages, 5,536 KiB swap use, three successful gateway pings and a completed
sync. Guest uptimes progressed from 308.69 to 578.71 and 652.35 seconds. Only
the known post-flash reset report appeared in this capture. This bounded
observation did not reproduce a whole-MCU reset; the assistant cannot
independently observe the panel and there is no new physical-glitch verdict.

Raw workload/postcheck output is `reset-diagnostic-workload-2026-09-05.log`.
The diagnostic firmware remains installed, SD/swap remain mounted, and the
game is stopped at the shell. The UART reader remains attached at 921600
with DTR/RTS inactive. No additional reset, performance variant, or speculative
display/cache fix was introduced. A new reset report or description of the
brief panel event is needed to distinguish the remaining hypotheses.

## Follow-up: generated-code lifetime checks, no board reset

The user clarified that assistant-initiated resets must be confined to
download mode and offered to perform a physical reset when requested.
No reset or flash was requested or performed during this follow-up. The
running reset-diagnostic application and production source hashes are
unchanged. Later guest uptimes were 790.31, 1541.61 and 1745.28 seconds,
with no additional reset report. UART output is in
`jit-lifetime-uart-2026-09-05.log`. This does not establish panel stability;
there is still no independently observed or timestamp-correlated screen event.

Read-only review confirmed that `jit_sync_code()` writes generated bytes
back, requests instruction-cache invalidation and executes `fence.i` before
publication. Cold-code installation removes overlapping entries before
reuse; promotion/demotion retarget incoming published and training pointers.
The installed SDK's `esp_cache_msync.c` and `hal/cache_hal.c` were inspected,
including the P4 low-level invalidation path. No missing publication call
was identified. This source inspection is not a hardware cache-coherence
test and does not rule out a different software, cache, power or reset fault.

The existing relocation test covered one promotion, not repeated eviction
with surviving incoming references. The new standalone fixture
`tests/rv32-native-jit-lifetime.c` covers this gap without entering firmware:

- 256 pairs of hot promotion/eviction, checking both published jump targets
  and unconfirmed observations while executing the linked source each time.
- 64 cold-ring reuse cycles. Overwriting an authoritative cold target must
  retire its hot copy and incoming references; the surviving source must
  return safely without entering the overwritten region.
- Recompilation with changing arithmetic constants, rejected replacement,
  whole-cache flush, and cleared hot-occupancy metadata.
- All 32 guest registers, exact chain/local instruction counts, next PC and
  returning slot checked after execution; transition counts also checked in
  the counter-enabled shape.

Only the standalone fixture shrinks the code arenas to provoke reuse: 8 KiB
main arena, split equally between loop/ordinary code, and a hot ring sized
for one compiled region. The running board keeps its existing arena sizes.
The GNU RV32 simulator executes real generated instructions, but the cache
API is stubbed and `fence.i` uses the existing simulator-only bypass. These
checks cannot reproduce hardware cache maintenance, DMA or electrical faults.

The frozen `scripts/test-native-jit.sh` completed with exit zero for both
production and profiling code shapes, including the existing self-tests,
36 memory cases and new lifetime fixture. Artifacts are in
`/tmp/rv32-native-jit.FWcB8M`. `make test` and whitespace checks also passed.
The first harness attempt reported all four fixture passes but exited 2
after comments were edited into the script while it was executing; that
attempt is not counted as a passing harness. The final run used the frozen
script and completed end to end successfully.

A negative control copied the current JIT to
`/tmp/rv32-jit-lifetime-control.uBZp6x/jit-no-retarget.c` and disabled only
incoming-pointer retargeting. The production-mode fixture rejected this
copy with exit 20 at its promotion check, as expected. The actual production
JIT was never changed or flashed for that control.

Final test output and source hashes are in `jit-lifetime-tests-2026-09-05.log`.
This adds tested software-lifetime coverage; it does not establish the
reported reboot's cause, fix the screen glitch, or improve measured FPS.
The reset diagnostic remains installed, with SD/swap mounted and no game
running. No additional performance variant was introduced.

## Event-only display-ownership diagnostic prepared

Reset classification covers a whole-MCU restart but cannot identify a brief
terminal repaint. The next diagnostic retains the existing reset report and
adds two rare-event messages in `main/display_bridge.c`:

```text
Display diagnostic: terminal release at <uptime_ms> ms
Display diagnostic: terminal takeover at <uptime_ms> ms, input=<bytes> bytes
```

Release is logged only when the terminal owned the framebuffer. Takeover is
logged only when ownership/activation changes, after the existing bounded
stream processing and redraw path. The byte count distinguishes an empty
wake from genuine terminal output. Timestamps use the ESP host timer; they
are not Linux guest uptime. No per-frame logging was added.

This intentionally preserves the existing zero-byte takeover behavior: the
goal is to determine whether it occurs on the device and coincides with the
reported blink before treating it as the physical symptom's cause. Rendering,
buffer ownership decisions, cache operations, watchdogs and clocks are not
changed. Event logging can perturb timing, so this is not a performance
comparison image and absence of recurrence will not prove a fix.

- Build: `build-rv32-display-diagnostic/uc-rv32ima.bin`
- Application SHA256: `c4bf84192586afff21d68e96265bae10cedf98d371ed24e4e937cc6fe6075eb6`
- ELF SHA256: `6305d525907129ba66fad058a06ab7da46008ad04f1246010d28e6480b2b5fb4`
- Application bytes: 733,152, 336 above the reset-only diagnostic.
- Static DIRAM: 320,326 bytes, 226 above reset-only; external text stays 329,440 bytes.
- Display-source SHA256: `f3651266f23ae2e39d41d9718de5ae4cc76567c5194a3469338a454298b3b9d2`
- Previous display-source SHA256: `6cb372db264e0b9a9b66002a115c6b6173c591d0970f46b559d91b56202bfb61`
- App/JIT sources retain their previously recorded reset-diagnostic hashes.
- Config: `/tmp/esp32p4-rv32-display-diagnostic-sdkconfig`, zero semantic differences from reset-only.

The full firmware build, separate emulator page-cache tests and whitespace
checks passed. The two instrumented functions were copied verbatim to
`display-owner-diagnostic-repro-2026-09-05.c` with controlled queue/render/time
helpers. This host check passes for an empty takeover, a four-byte takeover,
repeated release, an already-owned empty wake and a missing stream. Its output
contains exactly two takeover messages (input 0 and 4) and one release; the
zero-byte hazard deliberately remains. This is not a P4 cache/DMA simulation
or proof of the physical glitch's cause.

No game or guest kernel source was modified. At preparation time the new
application was not yet deployed; the installed reset-only image was
`472912b0573729a130dbaf29c620c24dc2d153cd3dcde9541e3d1f5a9612f313`.
After a safe SD unmount, the next action is a coordinated application-only
flash via the confirmed Espressif `/dev/ttyACM0` at 921600, in download mode.

At about 03:03 UTC, SD data was synced, swap was disabled and `/mnt/sd` was
unmounted successfully. `/proc/swaps` contains no active swap and the mount
listing has no SD mount. Guest uptime reached 2563.95 seconds with no new
reset report. Only the FTDI serial device is present; the Espressif USB
download interface is absent. Preparation output is in
`display-diagnostic-preparation-2026-09-05.log`. Deployment now awaits the
user's physical download-mode action; no outside-download reset was attempted.

Rechecks through 03:07 UTC still show only FTDI, no Espressif download
interface. The live UART session produced no new reset/fault report during
the bounded waits, and the prepared application hash was reverified. After
the same missing physical mode prevented deployment for three consecutive
goal turns, work was marked blocked awaiting download mode. The diagnostic
was not flashed at that point; neither the performance objective nor fault
diagnosis was claimed complete.

### Deployment after download mode became available

On resumption around 10:48 UTC, the exact Espressif USB by-id device was
present at `/dev/ttyACM0`. The application hash was reverified, and esptool
connected without any pre-connection reset (`--before no-reset`), identifying
ESP32-P4 revision v1.0 and MAC `30:ed:a0:e2:08:a8`. Only the application at
0x10000 was written at 921600; hash verification passed and the tool exited
zero. The subsequent reset was the known deliberate post-flash control.

Boot identified ELF prefix `6305d5259`, compile time Sep 5 02:56:40, CPU at
360 MHz, passing PSRAM/JIT self-tests, 16 KiB hot tier and 700 KiB SD cache.
Guest/render/scanout allocations are unchanged. PPA, display and network
initialization completed and Linux reached its shell. USB host initialized
with no attached peripheral; this is not an attached-device I/O test.

The captured reset report was again `idf=1 raw_cpu0=0x1 raw_cpu1=0x1`, from
the known post-flash boot, not evidence of the earlier fault's cause. The
first real ownership trace was `terminal takeover at 19509 ms, input=2
bytes`, during normal terminal startup. No zero-byte takeover has been
observed on the device yet. The deployed image is now the display diagnostic
`c4bf84192586afff21d68e96265bae10cedf98d371ed24e4e937cc6fe6075eb6`.

### Completed ownership-diagnostic replay

With the existing SD and swap mounted, the unchanged `doom-bench-640`
SHA256 was again verified as
`66304de72f65948eaad46284cc3f0bf02ea74ed78387f5dea7ab8a398b532285`.
The same RGB565, no-sound/no-music `demo1` replay completed with exit zero:

```text
frames=100 elapsed_ms=35804 fps_x100=279 mode=present
```

That is 2.793 FPS for 100 application-present calls. It is within the prior
run spread and is neither a new speedup nor an established regression from
the event logging. No guest diagnostic command ran during the timed window.

The complete captured ownership sequence was:

| Host uptime | Event | Interpretation |
| --- | --- | --- |
| 19,509 ms | Terminal takeover, 2 bytes | Normal terminal startup |
| 209,043 ms | Terminal release | Game claims the raw framebuffer |
| 277,821 ms | Terminal takeover, 44 bytes | Shell prompt after game exit |

No intermediate takeover or zero-byte takeover was captured. The raw-owner
interval was 68.778 seconds, whereas the benchmark's timed 100 presents
covered 35.804 seconds; these are different measurement windows. The exit
transition is expected and is not evidence that the user's intermittent
in-game symptom has been explained. There is still no physical observation
correlated with an unexpected event.

Postchecks found no remaining benchmark PID and no matching Linux
panic/oops/BUG/OOM messages. Guest uptime reached 1,076.64 seconds without
another captured reset report. Gateway ping, swap-use and final sync checks
were not collected for this image before UART access was lost, so they are
not claimed as passing postchecks. SD/swap were last seen mounted; no safe
unmount was performed before the host-side disconnection.

The installed normal game was also inspected read-only for a longer run.
Its SHA256 is
`dbee4967912ebedc42e19b500fdd04b1128b09b271448f8f844f751e6d7b8369`.
It differs from the benchmark and from the locally built Doom executable.
Local source defaults to 640x400 and prints its configured dimensions at
startup, but that does not verify this installed binary's resolution. No
game source or binary was edited, rebuilt or replaced. Its resolution and
long-duration behavior remain unverified because that run never started.

### Host serial disconnection before the longer run

The bounded UART reader exited with `SerialException` before any executable
long-run command group was submitted. At most the opening parenthesis was
attempted; no game-launch line, timer or kill command was sent. Session 45154
has ended and must not be reused.

Read-only host checks found no `/dev/serial/by-id`, `/dev/ttyUSB0`,
`/dev/ttyUSB1` or `/dev/ttyACM0`. The kernel journal records laptop suspend
entry at 11:08:19, resume at about 11:15:34, and the FTDI adapter disconnect
at 11:15:34. This explains the loss of host serial access, not the MCU's
earlier reboot or the panel glitch. No new MCU boot/panic was captured at
the disconnection, and the board's current runtime state is unobservable.

Raw console output is `display-owner-diagnostic-uart-2026-09-05.log`;
relevant host power-management/FTDI events are
`display-diagnostic-host-usb-2026-09-05.log`. The next safe action is to
restore the FTDI connection, verify its exact by-id identity and current tty
assignment, then reopen at 921600 with DTR/RTS inactive. Do not assume a
future `/dev/ttyACM0` is the Espressif device: unrelated USB devices can use
that node too. No reset or reflash was requested for this interruption.

### User-requested RETRY and second host suspend

After `RETRY`, the same FTDI serial identity returned as `ttyUSB0` and was
opened with DTR/RTS inactive. The temporary console helper now accepts an
explicit device argument, so the stable by-id path can be used despite tty
renumbering. This change affects only the host reader, not firmware.

Initial unreadable input prompted a baud-only switch to 115200. Around
11:20 UTC the reader captured a new ROM banner:

```text
rst:0x1 (POWERON),boot:0x30f (SPI_FAST_FLASH_BOOT)
```

Switching back to 921600 recovered application output and a Linux shell.
The display diagnostic reported normal startup takeover at 19,409 ms with
two bytes. The application reset-reason line and most initialization output
were not captured during the baud transition, so full boot checks are not
claimed for this boot. There was no assistant-issued reset or flash command;
the physical cause of the new boot is unknown. C6 association retries were
visible, so networking was not verified as operational in this boot.

Before a guest command could be executed, laptop suspend at 11:20:58 and
resume at 11:52:16 caused another FTDI disconnect/re-enumeration. The adapter
returned as `ttyUSB1`. A fresh by-id connection produced a repetitive `<`
flood rather than a usable prompt. That reader was closed normally. Two
bounded 256-byte raw samples at 921600 and 115200 contained the same repeating
non-text pattern. The host also logged `Unable to read latency timer: -32`.
This is evidence of an unusable observation channel, not a diagnosis of
the game's glitch or an established MCU firmware fault.

The 180-second run was never launched; no game, timer or kill command was
sent. SD/swap state was not successfully queried after the fresh boot and
is unknown. No game source, binary or firmware was changed during RETRY.
There is no live UART session now. Raw retry output, with its garbage-output
truncation limitation, and relevant host events are recorded in
`display-diagnostic-retry-uart-2026-09-05.log`. The requested next action is
a physical FTDI USB reconnection while keeping the host awake, followed by
another identity-checked console connection. No board reset is requested.

An automatic continuation rechecked the connection around 11:56 UTC. The
same FTDI identity remained on `ttyUSB1`, with no competing serial reader or
new USB recovery event. Two bounded half-second receive-only samples again
returned the same repeating non-text pattern at 921600 and 115200, each
containing 311,296 bytes. This does not supply a usable shell or any game
measurement. The port was closed, no guest command/reset was sent, and the
same physical-connection blocker remains. The goal is not complete.

A third consecutive goal-turn check around 11:57 UTC again found the same
FTDI mapping, no competing reader and no new host USB recovery event.
Half-second receive-only samples still returned the repeating non-text
pattern: 307,200 bytes at 921600 and 311,296 at 115200. No reset, flash or
guest command was issued. With console-dependent hardware testing unable
to proceed and the safe identity/ownership/baud checks exhausted, the goal
is marked blocked awaiting a usable physical FTDI connection. This is not
a completion or an assertion that further generic optimization is impossible.

### Retry with board on: boot reception works, interactive input does not

The user retried with the board on and confirmed `/dev/ttyUSB0` permissions.
The exact FTDI identity had reappeared at that node at 12:01. No competing
reader was present. The port opened, but blank input at 921600 and 115200
produced no shell response. A user-performed `RESET` was requested; no
assistant runtime reset or flash command was issued.

The temporary host reader was extended with `:boot`: it arms receive at
115200 and switches to 921600 after a complete ROM `entry 0x...` line.
This changes only host observation, not firmware, and does not pulse DTR/RTS.
Syntax compilation passed, and the subsequent real boot capture verified
that the transition retains both the ROM and application reset reports.

The captured boot following the reset request reports:

- ROM `rst:0x1 (POWERON),boot:0x30f (SPI_FAST_FLASH_BOOT)`.
- Application `Reset diagnostic: idf=1 raw_cpu0=0x1 raw_cpu1=0x1`.
- Expected ELF prefix `6305d5259`, compile time Sep 5 02:56:40, CPU 360 MHz.
- Passing PSRAM and JIT self-tests, unchanged render/scanout addresses,
  16 KiB hot tier and 700 KiB SD cache.
- Display, C6 association and Linux shell startup; initial terminal takeover
  at 19,341 ms with two input bytes. USB initialized without an attached device.

These reset values belong to the boot captured after the manual-reset
request. They do not identify the cause of the earlier unexpected reboot.

However, the subsequent read-only shell commands did not yield guest echo,
output or a new prompt. The visible command lines in the host capture are
local PTY echo, not proof that Linux executed them. A separate LF byte also
produced no response. No mount, swap-enable or game-launch command was sent.
Current SD/swap state consequently remains unverified.

An independent, non-SSH check received all three ICMP replies from
192.168.1.220 with zero loss. The neighbor entry matched the board's known
MAC `cc:ba:97:f0:1b:f4`; this was not merely a reply from an unidentified host.
Linux is therefore not completely stalled at that observation. This does
not prove the shell, hvc receive queue or physical UART RX path is healthy.

Read-only host checks confirmed 921600, 8N1, no RTS/CTS flow control, and no
XON/XOFF flow control. The deployed configuration assigns UART0 TX to GPIO37
and RX to GPIO38. Local source inspection confirms CR-to-LF conversion in
`guest/display-shell.c`, buffered UART input in `main/port-esp.c`, and the
UART-to-hvc receive-queue path in `main/virtio_console_bridge.c`. No runtime
RX-byte/queue counter is available from this capture, so wiring versus
UART/virtio-console input delivery remains unresolved. No speculative source
change was made.

Reader session 54156 eventually terminated with a zero-length-read
`SerialException`. There was no matching new host USB disconnect event, and
the same by-id node remained present; do not label this particular event a
confirmed USB unplug. After its terminal exit, a new reader was opened at
921600 as session 92206. A harmless `echo UART_OK` probe still returned no
guest response. The current reader remains live awaiting the physical input
connection check. This is not a restart caused solely by an observation timeout.

App, display and JIT source hashes were reverified unchanged from the deployed
diagnostic. The complete boot/console capture and matching-MAC ping result
are in `display-diagnostic-manual-reset-2026-09-05.log`. The requested next
check is FTDI TX to MCU RX GPIO38 and common ground, not another MCU reset.

### Safe alternate control check and resumed blocked audit

At about 12:31 UTC, another blank UART input still produced no remote
response. The exact FTDI identity remained mapped to `ttyUSB0`. The existing
rootfs Dropbear setting is enabled, and a bounded TCP check found port 22
open on the already MAC-verified board address.

One bounded SSH attempt was made only to read guest uptime, serial-driver
state, hvc terminal settings and process state. It was not a benchmark and
did not launch the game. The attempt required strict host-key checking,
disabled host-key updates and connection persistence, and used batch mode.
It exited 255 with `REMOTE HOST IDENTIFICATION HAS CHANGED` before any remote
command executed. The offered public-key fingerprint was:

```text
SHA256:7Msa3W+Lez90mF+BlASxKxNpSmQdJIx9nK5kGb0NqdA
```

This fingerprint has not been independently verified. The old trust entry
was not removed or replaced and verification was not bypassed. The local
init source can generate a host key if one is absent, but that possibility
does not independently authenticate this offered key.

The physical UART-input blocker has persisted for three consecutive turns
since the user's `retry, currently box is on`: identity/permissions and both
baud checks, requested-reset boot capture and matching-MAC ping, then another
live UART probe plus the safe alternate-control attempt. Session 92206 was
closed with the helper's `:quit` command, which does not send a board reset
or guest command. No serial or SSH process is intentionally left running.

The goal is marked blocked pending a working UART input connection, or
independent SSH host-identity verification followed by a user choice to use
that diagnostic route. It is not complete. The 5 FPS objective, long game
observation, intermittent panel glitch and earlier reboot diagnosis remain
unresolved; no hardware performance ceiling is asserted.

### FTDI TX reconnection restores control

The user reported reconnecting TX. At about 12:39 UTC, the same adapter on
`ttyUSB0` returned remote command echo, `UART_OK`, and the guest shell prompt.
No new reset or firmware write was required. This resolves the lost console
input observed during the retries; it does not establish the cause of the
earlier screen glitch or whole-device reboot. The prospective offline SSH
key-verification investigation was abandoned once UART worked. No SSH trust
entry was changed and no SSH diagnostic command or game benchmark succeeded.

The guest initially reported uptime 1,178.47 seconds, no active swap and no
SD mount. `/dev/vda` was present. The existing SD filesystem was mounted and
its existing 16 MiB `.doom-swap` file enabled. No game file was replaced.

### Same-image 100-present replay after TX recovery

The benchmark SHA256 again matched
`66304de72f65948eaad46284cc3f0bf02ea74ed78387f5dea7ab8a398b532285`.
With the same RGB565/no-sound/no-music `demo1` arguments, it completed with
exit zero:

```text
frames=100 elapsed_ms=44292 fps_x100=225 mode=present
```

This is 2.2577 FPS, slower than the earlier 35.804-second display-diagnostic
sample and outside the earlier roughly 31-38-second spread. There was no
intervening JIT/cache/clock/firmware change, so this result is not evidence
that a newly applied optimization caused the slowdown. Its cause is not
isolated, and no speedup or hard hardware limit is claimed. No guest
diagnostic command ran during the timed interval.

Ownership changed from terminal to raw framebuffer at host uptime
1,258,699 ms and back at 1,340,325 ms with 44 real terminal bytes, alongside
the returning shell prompt. There was no intermediate takeover or new reset
report. The raw-owner interval of 81.626 seconds is not the same as the
benchmark's 44.292-second timed window. The startup log recorded zone memory
at guest virtual address `0x94fd0040`; no physical/cache-color conclusion is
drawn from that address alone.

Postchecks showed no surviving game process, no matching kernel
panic/oops/BUG/OOM message, 5,348 KiB swap use and guest uptime 1,425.67
seconds. The redirected benchmark startup log was incompletely flushed by
its `_exit`, so absent late startup messages are not evidence of a missing
graphics initialization.

### Completed bounded longer replay

The installed normal game's SHA256 was independently rechecked as
`dbee4967912ebedc42e19b500fdd04b1128b09b271448f8f844f751e6d7b8369`.
That unchanged executable was started as PID 77 with `-timedemo demo1`,
RGB565 and sound/music disabled. Unlike the FPS benchmark, its stdout was
left on the terminal so initialization messages were line-flushed and
captured. This is a stability observation, not an FPS comparison.

The actual binary reported 640x400 virtual/output dimensions, 16 bpp and
RGB565 bit offsets. It also reported the existing Doom engine's 320x200
internal screen and auto-scaling factor two. These are distinct resolutions;
no claim of native 640x400 Doom scene rendering is made. No game source or
binary change was used to obtain this output.

A companion shell guard waited 180 guest seconds, recorded `limit=180s`, and
sent SIGTERM only to the captured game PID. The parent waited for that PID
and cancelled the guard if the game finished sooner. The capture ended with
`Terminated`, `long_exit=143`, and the shell prompt. The limit marker was
read back afterward. This termination was expected and must not be counted
as a spontaneous crash.

| Host uptime | Ownership event | Context |
| --- | --- | --- |
| 1,609,681 ms | Terminal release | Initial game framebuffer setup |
| 1,609,756 ms | Terminal takeover, 51 bytes | Real game initialization output |
| 1,646,885 ms | Terminal release | Initialization ends; moving demo |
| 1,792,792 ms | Terminal takeover, 11 bytes | Planned termination notice |

The moving-demo raw-owner interval was 145.907 seconds. No intermediate
takeover, zero-byte takeover, reboot report or spontaneous termination was
captured. There is no independent camera view and the user has not yet
reported whether a brief physical display event occurred in this window.
This bounded non-reproduction does not prove a fix or exclude the reported
once-per-minute symptom.

Afterward, neither game PID remained, guest uptime was 1,926.06 seconds, and
the kernel fault-message grep again found no matches. Swap use was 5,924 KiB;
all three guest gateway pings succeeded and `sync` returned to the prompt.
No keyboard was attached, so USB initialization is still not an attached-HID
or other USB I/O test. The app/JIT/cache settings remain unchanged, SD and
swap remain mounted, and the current UART reader is session 78532 at 921600
with DTR/RTS inactive. Raw evidence for the TX repair, both game runs and
postchecks is `display-diagnostic-tx-restored-2026-09-05.log`.
