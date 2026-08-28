# esp32p4-rv32ima

Run Linux on the ESP32-P4 with a small RISC-V emulator. This project is based on [CNLohr's mini-rv32ima](https://github.com/cnlohr/mini-rv32ima). The ESP32-P4 is currently the only tested target.

<div align="center">
<img width="400" height="300" alt="20260827_142552" src="https://github.com/user-attachments/assets/fc94d895-d524-4a0b-a763-805fbcfaff4c" />
<img width="400" height="300" alt="20260827_144636" src="https://github.com/user-attachments/assets/ab346992-d67d-4b86-9319-969dcc64a5f5" />
</div>

## What changed from Epiczhul's + CNLohr's

- Work is split between both ESP32-P4 CPU cores. The Linux guest is still single-core, but display, UART, and network work can run separately from the emulator.

  ```text
  ESP32-P4

  +------------------------+       +------------------------+
  | CPU0                   |       | CPU1                   |
  +------------------------+       +------------------------+
  | RV32IMA emulator       |       | Display updates        |
  | Linux and OpenWrt      |       | UART0 input/output     |
  | Guest memory access    |       | C6 network bridge      |
  +------------------------+       +------------------------+

         CPU0 and CPU1 exchange work through queues and buffers.
  ```

- MMU and supervisor-mode support were added. With 32 MB of PSRAM, the emulator can run a Linux kernel with an OpenWrt or BusyBox root filesystem.

  ```text
  CONFIG_MMU=y
  ```

  This work was inspired by [mini-rv32ima-mmu](https://github.com/cmdada/mini-rv32ima-mmu/blob/main/mini-rv32ima.h) and then adapted for this project.

- Common Linux memory accesses go straight to guest RAM instead of repeating a full page-table lookup. Small caches handle the remaining lookups. Later cache and instruction-fetch changes also reduce repeated address checks and unnecessary invalidation work.

  ```text
  Common path: guest address -> direct RAM offset -> PSRAM
  Other path:  guest address -> cached address -> PSRAM or device
  ```

- WFI handling was fixed. When Linux executes WFI, the emulator pauses guest instructions until an interrupt arrives.
- An asynchronous ST7703 framebuffer backend was added. Linux uses it as `fb0` and `tty0`, while CPU1 updates the physical display. Console fills, copies, scrolling, glyphs, and cursor changes can use a bounded command FIFO instead of repeatedly repainting the full framebuffer. This significantly improves console rendering while keeping the interface usable by other guest images.
- Terminal output can now use the standard `virtio-console` device as `hvc0`. Linux sends an ANSI text stream instead of drawing every character pixel by pixel inside the emulator. CPU1 handles colors, cursor movement, clearing, scrolling, and compact 16-bit glyph runs, removing most of the fbcon, MMU, and PSRAM work. The previous `console-mux` path remains available as a compatibility option.
- ESP32-C6 wireless support is provided through a virtual `eth0` device. Linux does not see a normal Wi-Fi interface, so the ESP32-P4 side manages the Wi-Fi connection. The four-bit SDIO and virtio path now avoids an extra packet copy, batches completion updates, and uses an ordered CPU1 task pipeline. Wi-Fi power saving can be disabled for lower latency, and HT40 can be negotiated with compatible access points.
- Flash mode was changed from DIO to QIO. If flashing or booting is unreliable on another board, check this setting.
- Dropbear SSH is included, and the ESP32-P4 gives Linux fresh random data during startup. Connect with the IP address assigned to `eth0`.
- The OpenWrt RV32IMA root filesystem includes BusyBox, curl, APK, and Dropbear.
- A repeatable guest build process was added.

> `make guest` builds the OpenWrt root filesystem, Linux Image, and device trees. Check the `Makefile` for the other guest targets.

### Serial receive path

    UART0 byte arrives
    emulated 16550 reports RX-ready
    PLIC source 1 becomes pending
    supervisor SEIP is asserted
    WFI state is cleared
    Linux 8250 interrupt handler reads the byte

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

- There is no general package repository for this exact `riscv32_rv32ima` build. Select the packages you need before building so they are included in the root filesystem. For example:

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

- If your board includes an ESP32-C6 coprocessor, enable the Wi-Fi bridge:

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

- For the Waveshare ESP32-P4-WIFI6-Touch-LCD-4B display, enable:

  ```text
  CONFIG_RV32_ST7703_DISPLAY=y
  ```

  Disable this option for a headless board.

- For an external 240x240 SPI ST7789 display, select:

  ```text
  CONFIG_RV32_ST7789_DISPLAY=y
  ```

  Configure its SPI controller, clock, SCLK, MOSI, CS, DC, reset, backlight,
  offsets, inversion, and orientation through `idf.py menuconfig`. The default
  pin assignment is only a starting point; match it to your module and board.

- Choose a partition table that matches the physical flash size. The current
  OpenWrt `Image` fits in either layout. Flash size does not change the amount
  of PSRAM available to the Linux guest.

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

  This leaves 30 MiB for the kernel `Image`. Also select the matching 16 MB or
  32 MB flash size in `idf.py menuconfig` before building.

- Configure the Wi-Fi SSID and password before building. The device receives an address through DHCP. Check UART0 or run this inside Linux:

  ```sh
  ip address show dev eth0
  ```

- If networking or login fails, inspect the UART0 log and the guest `/init` script. The generated root filesystem is available at:

  ```text
  build/guest/rootfs
  ```

- Build the OpenWrt root filesystem, Linux Image, and device trees:

  ```sh
  make guest
  ```

- Build the ESP32-P4 firmware from an initialized ESP-IDF environment:

  ```sh
  idf.py build
  ```

- Flash the firmware, kernel, and selected device tree:

  ```sh
  idf.py -p /dev/ttyACM0 -b 921600 flash
  ```

  Replace `/dev/ttyACM0` with the port used by your board. The `flash` command writes every image registered by this project; `--all` is not needed.

## Notes

> If your MCU is not revision version, like esp32p4x you may try `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_400=y`
> display is just ttyS, you may try fun experiments like graphically render something, you may inspire from `guest/fbstream.c`... if you are brave enough try sd card and minimal freedesktop de/compositor, extremely excited to see the results


`discord: ok.dusty`
