# esp32p4-rv32ima

> Emulated Linux runs at ~77 DMIPS (Dhrystone best-case) | like IBM [RS/6000 (POWER1)](https://en.wikipedia.org/wiki/POWER1)-class, ~1/7th the per-MHz of a SiFive E31.

Run Linux on the ESP32-P4 with a small RISC-V emulator. This project is based on [CNLohr's mini-rv32ima](https://github.com/cnlohr/mini-rv32ima). The ESP32-P4 is currently the only tested target.

<div align="center"><i>wmaker + xterm + fastfetch</i></div><br>
<img width="1000" height="500" alt="20261025_034026" src="https://github.com/user-attachments/assets/e490e69e-5481-43dd-a74d-5aa2ed29d3c2" />

<div align="center">
<img width="400" height="300" alt="20260827_142552" src="https://github.com/user-attachments/assets/fc94d895-d524-4a0b-a763-805fbcfaff4c" />
<img width="400" height="300" alt="20260827_144636" src="https://github.com/user-attachments/assets/ab346992-d67d-4b86-9319-969dcc64a5f5" />
</div>

### Latest measurements

| workload | guest time |
| --- | ---: |
| `find /opt` | 4.0s |
| `cat` libs/bins | 14.7s |
| `curl` https x2 | 16.0s |
| desktop `startx` to quiet | 41s settle (50.4s total) |

## What changed from Epiczhul's + CNLohr's

### Dual-core workload split

Work is split between both ESP32-P4 CPU cores. The Linux guest is still single-core, but display, UART, and network work can run separately from the emulator.

```text
ESP32-P4 (dual-core default)

+------------------------+       +------------------------+
| CPU0                   |       | CPU1                   |
+------------------------+       +------------------------+
| RV32IMA emulator       |       | Display updates        |
| Linux and OpenWrt      |       | UART0 input/output     |
| Guest memory access    |       | C6 network bridge      |
|                        |       | JIT compilation        |
|                        |       | USB HID host           |
|                        |       | microSD / virtio-blk   |
+------------------------+       +------------------------+

     CPU0 and CPU1 exchange work through queues and buffers.
     On a single-core build every task falls back to CPU0.
```

Verified against the code: the emulator loop runs in `app_main` (main task, CPU0); `rv32-net` (`virtio_net_bridge.c:962`), UART TX (`port-esp.c:741`), USB HID (`usb_hid_input.c` `USB_HID_CORE 1`), ESP-Hosted and virtio-blk init (`HostRunOnCpu1`), and the display task (`display_bridge.c:3064`, pinned to the core after `app_main`'s) all run on CPU1.

### MMU and supervisor-mode support

MMU and supervisor-mode support were added. With 32 MB of PSRAM, the emulator can run a Linux kernel with an OpenWrt or BusyBox root filesystem.

```text
CONFIG_MMU=y
```

This work was inspired by [mini-rv32ima-mmu](https://github.com/cmdada/mini-rv32ima-mmu/blob/main/mini-rv32ima.h) and then adapted for this project.

### Faster memory access

Common Linux memory accesses go straight to guest RAM instead of repeating a full page-table lookup. Small caches handle the remaining lookups. Later cache and instruction-fetch changes also reduce repeated address checks and unnecessary invalidation work.

```text
Common path: guest address -> direct RAM offset -> PSRAM
Other path:  guest address -> cached address -> PSRAM or device
```

The native JIT also keeps successor links across data-only page fences ([measurements](docs/cache-page-fences-and-phone-network-2026-09-08.md)).

### WFI handling

WFI handling was fixed. When Linux executes WFI, the emulator pauses guest instructions until an interrupt arrives.

### Asynchronous ST7703 framebuffer

An asynchronous ST7703 framebuffer backend was added. Linux uses it as `fb0` and `tty0`, while CPU1 updates the physical display. Console fills, copies, scrolling, glyphs, and cursor changes can use a bounded command FIFO instead of repeatedly repainting the full framebuffer. This significantly improves console rendering while keeping the interface usable by other guest images.

### virtio-console terminal output

Terminal output can now use the standard `virtio-console` device as `hvc0`. Linux sends an ANSI text stream instead of drawing every character pixel by pixel inside the emulator. CPU1 handles colors, cursor movement, clearing, scrolling, and compact 16-bit glyph runs, removing most of the fbcon, MMU, and PSRAM work. The previous `console-mux` path remains available as a compatibility option.

### ESP32-C6 wireless bridge

ESP32-C6 wireless support is provided through a virtual `eth0` device. Linux does not see a normal Wi-Fi interface, so the ESP32-P4 side manages the Wi-Fi connection. The four-bit SDIO and virtio path now avoids an extra packet copy, batches completion updates, and uses an ordered CPU1 task pipeline. Wi-Fi power saving can be disabled for lower latency, and HT40 can be negotiated with compatible access points.

### Flash, SSH, and guest build

- Flash mode was changed from DIO to QIO. If flashing or booting is unreliable on another board, check this setting.
- Dropbear SSH is included, and the ESP32-P4 gives Linux fresh random data during startup. Connect with the IP address assigned to `eth0`.
- The OpenWrt RV32IMA initramfs holds BusyBox, Dropbear and the guest init; OpenWrt's procd/netifd/apk/curl layer is pruned because it never runs here and would pin guest RAM.
- A repeatable guest build process was added.

> `make guest` builds the OpenWrt root filesystem, Linux Image, and device trees. Check the `Makefile` for the other guest targets.

### Host logging

Normal firmware prints only warnings, errors and the guest console. `idf.py menuconfig` → **RV32 emulator → Enable host diagnostic logs** restores the detailed startup, reset, PSRAM and bridge messages, with separate switches for display, USB and CSR traces. Performance counters are separate opt-in settings.

Measurements: [instruction-frequency profile](docs/low-level-instruction-profile-2026-09-08.md), [Doom palette cache](docs/doom-palette-cache-2026-09-08.md) (3.22 → 4.00 FPS; build instructions in [guest/doom](guest/doom/README.md)).

Flash the P4 through `/dev/ttyACM0`; the FTDI `/dev/ttyUSB0` is the 921600-baud console. `build/openwrt-rv32/` and `build/c6-slave/` hold reusable toolchains, so keep them when cleaning.

### Kernel-mode JIT

With **Also translate supervisor-mode (kernel) code** (on by default), hot Linux kernel code runs natively as well as user code. User code, the kernel, and the kernel with SUM set keep separate data-page caches, so system calls and interrupts switch caches instead of flushing them. Machine mode, CSR, atomic and trap-return instructions stay in the interpreter.

With **Compile native regions on the second core** (on by default), CPU1 compiles: the emulator core queues a hot block, keeps interpreting it, and installs the finished region a moment later, so compiling no longer stops the guest ([measurements](docs/jit-async-compile-2026-10-18.md)).

With **Keep kernel regions across FENCE.I** (on by default), the `fence.i` that Linux issues after mapping each new executable user page drops only compiled user code: kernel code, its links and every warm-up count stay (a write to the kernel's own text still discards everything). Kernel and user code take separate halves of the code arena, so the user code compiled after each fence cannot push kernel code out, and context switches keep the kernel's native links ([measurements](docs/jit-cpu0-overheads-2026-10-19.md)).

### USB port

**USB port** (RV32 Linux hardware bridges) chooses who drives the USB host port. With **Linux (any USB device)**, the default, the guest's dwc2 driver owns the controller: memory sticks, hubs, keyboards and mice all work, but every USB interrupt runs through the emulated kernel (about 1.2 ms each), so an idle mouse alone takes a sixth of the guest's CPU. With **Keyboard and mouse on CPU1**, ESP-IDF's USB host and HID drivers read boot-protocol keyboards and mice (also through a hub) on CPU1 and hand keys, buttons, motion and wheels to Linux as one virtio-input device (`/dev/input/event0`, used by the console and by X); idle devices cost the guest nothing, mouse motion reaches it at most 100 times a second, and Caps/Num/Scroll Lock go back to the keyboard. Other USB devices are not available to Linux in that mode. Either way, plug devices in before power-up: plugging in or out resets the board.

### Read-only desktop disk

The 32 MiB layout adds a 16 MiB `rofs` partition that Linux sees as a read-only virtio disk and mounts at `/opt`. Build the image, then enable **Serve a read-only guest filesystem from flash** in menuconfig:

```sh
make guest-desktop   # Buildroot: Xorg, Window Maker, xterm, vim, curl, fastfetch, ...
idf.py build flash
```

On the guest, `startx` starts Window Maker on the display (with a USB keyboard and mouse). It runs `~/.xinitrc`, which mounts the disk's writable directories, starts X in the disk with chroot and, under X, is the session (the background is `themes/pattern.xbm`). The desktop shares root's home with the shell, so `~/.config/fastfetch` (the logo from `themes/logo.txt`) applies to both. `/opt` is on the shell's `PATH`, so `vim`, `curl` and `fastfetch` run the disk's programs. With the option off, the disk is absent and the firmware is the plain console image. `scripts/build-rofs.sh DIR` packs any other directory instead.

### Guest reboot and download mode

`reboot` or `systemctl reboot` restarts the whole P4, and `systemctl reboot --firmware-setup` enters ROM USB download mode for flashing through `/dev/ttyACM0`. No eFuses are changed. `systemctl` is a reboot-only shim; the image has no systemd.

The helper turns off swap and unmounts SD/USB filesystems first; a busy mount or failed swapoff cancels the reboot. The root filesystem is an initramfs, so changes made there are lost on restart.

### SSH and file transfers

The guest uses the static address `192.168.1.220/24` with gateway `192.168.1.1` (set in `guest/rv32-init.c`), sets its clock once from `pool.ntp.org` at boot (there is no RTC, and HTTPS needs the date), and runs Dropbear with an SFTP server:

```sh
./scripts/guest-ssh 'uptime'
./scripts/guest-scp ./file :/tmp/file
./scripts/guest-scp :/tmp/file ./downloaded-file
```

The helpers use `~/.ssh/esp32p4_rv32` and check host keys against `.guest-work/ssh/known_hosts`. `RV32_GUEST_HOST`, `RV32_SSH_KEY` and `RV32_SSH_PROXY_COMMAND` override the address, key and transport (for example `adb shell -T su -c "nc %h %p"` to relay through a phone). `guest-scp -O` works with images that have no SFTP server.

When present, `.guest-work/ssh/authorized_keys` and `.guest-work/ssh/dropbear_ed25519_host_key` are built into the image, so access and the board's host key survive rebuilds. `DROPBEAR_AUTHORIZED_KEYS` and `DROPBEAR_HOST_KEY` override them, and an empty value disables either one. The host private key ends up in the firmware image, so keep images private.

### Serial receive path

```text
UART0 byte arrives
emulated 16550 reports RX-ready
PLIC source 1 becomes pending
supervisor SEIP is asserted
WFI state is cleared
Linux 8250 interrupt handler reads the byte
```

```text
root@esp32p4-openwrt:/# df -h
Filesystem                Size      Used Available Use% Mounted on
rootfs                   16.0M      5.9M     10.1M  37% /
devtmpfs                  8.0M         0      8.0M   0% /dev
root@esp32p4-openwrt:/#
```

## Issues

- USB passthrough is still experimental. The USB-C port on the tested board is wired as a sink, while USB host mode needs a source configuration and a safe 5 V VBUS supply.
- Do not modify the USB-C CC resistors or power wiring unless you understand the board schematic and USB-C power rules. A board with a native USB-A host port may be a better test platform.
- If you test USB host support on another ESP32-P4 board, sharing the board model and logs would be useful.

## Limitations

There is no general package repository for this exact `riscv32_rv32ima` build. Select the packages you need before building so they are included in the root filesystem. For example:

```sh
./scripts/config --file guest/openwrt/config.seed --enable PACKAGE_xxd
make guest
idf.py build
idf.py -p /dev/ttyACM0 -b 921600 flash
```

## How it works

The ESP32-P4 uses its 32 MB PSRAM chip as memory for the Linux guest. During startup, the firmware reserves this memory, loads the Linux Image and device tree from flash, and starts the RV32IMA emulator. The root filesystem is stored inside the Linux Image.

For custom images, use `make guest*` and `scripts/verify-guest-image.py`. Memory layout settings are in `main/psram.h` and the device-tree files under `main/`.

## Requirements

- One ESP32-P4 development board. This project was tested with the [Waveshare ESP32-P4-WIFI6-Touch-LCD-4B](https://www.waveshare.com/esp32-p4-wifi6-touch-lcd-4b.htm).
- A USB-C cable for flashing and UART debugging.
- ESP-IDF v6.1 or a P4 compatible version.

## How to use

### Wi-Fi bridge

If your board includes an ESP32-C6 coprocessor, enable the Wi-Fi bridge:

```text
CONFIG_RV32_WIFI_BRIDGE=y
```

You can also configure it through:

```sh
idf.py menuconfig
```

The low-latency and HT40 options are enabled by default when the bridge is enabled:

```text
CONFIG_RV32_WIFI_LOW_LATENCY=y
CONFIG_RV32_WIFI_HT40=y
```

Low-latency mode uses more power. HT40 automatically falls back when the access point or channel does not support it.

Configure the Wi-Fi SSID and password before building. The guest uses the static address `192.168.1.220` from `guest/rv32-init.c`; change it there for another network. Check it inside Linux with:

```sh
ip address show dev eth0
```

### Displays

- **Waveshare ESP32-P4-WIFI6-Touch-LCD-4B:** enable the ST7703 display:

  ```text
  CONFIG_RV32_ST7703_DISPLAY=y
  ```

  Disable this option for a headless board.

- **External 240x240 SPI ST7789 display:** select:

  ```text
  CONFIG_RV32_ST7789_DISPLAY=y
  ```

  Configure its SPI controller, clock, SCLK, MOSI, CS, DC, reset, backlight, offsets, inversion, and orientation through `idf.py menuconfig`. The default pin assignment is only a starting point; match it to your module and board.

- **Monochrome OLED or LCD supported by [U8g2](https://github.com/olikraus/u8g2)** (SH1106, SSD1306, SSD1309, SH1107, SSD1322, ST7565/ST7567, ST7920, PCD8544, ...): select:

  ```text
  CONFIG_RV32_U8G2_DISPLAY=y
  ```

  Then pick the display in the top-level **U8g2 display** menu: its controller, then the panel (size, U8g2 module name and bus, for example `128x64 noname (I2C)` for a 1.3" SH1106 OLED), the orientation, pixel inversion and contrast, and the I2C or 4-wire SPI pins. Size, bus and the default I2C address follow from the panel; the panel's help shows its [U8g2 setup function](https://github.com/olikraus/u8g2/wiki/u8g2setupc).

  The panel list covers every display of the vendored U8g2 (`components/u8g2`, without its font tables) that has an I2C or SPI bus. It is generated from U8g2's sources each time ESP-IDF configures the project. The firmware writes the panel geometry into the Linux device tree at boot, so no extra DTB is needed.

  Each configure also saves the chosen display and wiring as a preset, `build/u8g2/display.defaults`. Copy it next to `sdkconfig.defaults` and list it after that file to start another configuration with the same display, as with `guest/configs/`:

  ```sh
  idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;display.defaults" reconfigure
  ```

### Partition tables

Choose a partition table that matches the physical flash size. The current OpenWrt `Image` fits in either layout. Flash size does not change the amount of PSRAM available to the Linux guest.

For a 16 MB flash chip, use this as `partitions.csv`:

```csv
# Name,   Type, SubType, Offset,   Size,      Flags
nvs,      data, nvs,     0x9000,   0x6000,
phy_init, data, phy,     0xf000,   0x1000,
factory,  app,  factory, 0x10000,  0x100000,
kernel,   data, 0x58,    0x110000, 0xEE0000,
dtb,      data, 0x58,    0xFF0000, 0x10000,
```

This leaves 14.875 MiB for the kernel `Image`, including its initramfs.

For a 32 MB flash chip, use this as `partitions.csv`:

```csv
# Name,   Type, SubType, Offset,   Size,      Flags
nvs,      data, nvs,     0x9000,   0x6000,
phy_init, data, phy,     0xf000,   0x1000,
factory,  app,  factory, 0x10000,  0x100000,
kernel,   data, 0x58,    0x110000, 0x1E00000,
dtb,      data, 0x58,    0x1F10000, 0x10000,
```

This leaves 30 MiB for the kernel `Image`. Also select the matching 16 MB or 32 MB flash size in `idf.py menuconfig` before building.

### Build and flash

If networking or login fails, inspect the UART0 log and `guest/rv32-init.c`. The generated root filesystem is available at:

```text
build/guest/rootfs
```

Build the OpenWrt root filesystem, Linux Image, and device trees:

```sh
make guest
```

Build the ESP32-P4 firmware from an initialized ESP-IDF environment:

```sh
idf.py build
```

Flash the firmware, kernel, and selected device tree:

```sh
idf.py -p /dev/ttyACM0 -b 921600 flash
```

Replace `/dev/ttyACM0` with the port used by your board. The `flash` command writes every image registered by this project; `--all` is not needed.

## Notes

> If your MCU is not revision version, like esp32p4x you may try `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_400=y`
> display is just ttyS, you may try fun experiments like graphically render something, you may inspire from `guest/fbstream.c`... if you are brave enough try sd card and minimal freedesktop de/compositor, extremely excited to see the results

`discord: ok.dusty`
