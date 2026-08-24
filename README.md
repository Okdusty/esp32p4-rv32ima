# esp32p4-rv32ima
Run linux on various MCUs with the help of RISC-V emulator. This project uses [CNLohr's + mini-rv32ima](https://github.com/cnlohr/mini-rv32ima) RISC-V emulator core to run Linux on various MCUs such as ESP32P4. Although, only ESP32P4 is tested now.

## Why
- Just for fun  // no changes in here

## What changed from Epiczhul's + CNLohr's
- Currently we've divided graphical processing/uart/stream buffer/usb to CPU1, however emulator is single-core and loops under CPU0, multicore along st7703 display/virtio_net_bridge needs smart freertos alignment.
- Now with MMU! Thanks to 32M PSRAM and additional Supervisor set arch built top of mini-rv32ima now it's sufficient enough to fit kernel image + openwrt initramfs / busybox
> ```CONFIG_MMU=y``` along with total 16M space. inspired from [here](https://github.com/cmdada/mini-rv32ima-mmu/blob/main/mini-rv32ima.h) modified significantly for heavy optimizations.
- speaking of optimizations, now in short our cache path replaced from ```guest access => two-level Sv32 walk => PSRAM access``` to ```guest access => direct linear-map conversion => PSRAM access``` implemented several layers of caching, to bypass repeated sv32 page table steps.
- fixed WFI handling, when Linux executes WFI, emulator pauses guest instructions until interrupt arrives.
- added an async st7703 framebuffer backend. linux uses ``simple frame-buffer`` and can use it as `fb0`/`tty0` while physical display transferes handled outisde the emulator loop
- esp32c6 wireless connection implemented, not true wlan/wlp, its virtual eth0 bridge. so modifying any of wireless settings inside the emulator not possible
- changed DIO -> QIO kept the psram freq same, if you are having flashing booting problems, check this out
- ssh/dropbear along with CRNG equiped, you can remotely play via ``ssh root@172.25.115.28``
- added on OpenWRT rv32ima musl rootfs w/ busybox curl, apk package manager, dropbear. 
- added a reproducible guest build pipeline 

> ```make guest``` handles whole Image building, super useful | check `Makefile` for extras

### Serial receive path
    UART0 byte arrives
    emulated 16550 reports RX-ready
    PLIC source 1 becomes pending
    supervisor SEIP is asserted
    WFI state is cleared
    Linux 8250 interrupt handler reads the byte   


 root@esp32p4-openwrt:/# df -h
 Filesystem                Size      Used Available Use% Mounted on
 rootfs                   16.0M      5.9M     10.1M  37% /
 devtmpfs                  8.0M         0      8.0M   0% /dev
 root@esp32p4-openwrt:/#


## Issues

- Currently the usb is dead code, it was just an attempt to share usb high-speed otg with emulator
however it turned out that the board itself was not designed for this purpose ofc. likely fix can be treating type-c as source and making sure no drop out voltage is happening etc. 
if you want to try... you may try buying module that has just usb-a output (haven't tried yet, unsure but can be great, waveshare esp32p4 mini dev board equiped one) OR
if you got esp32-p4-wifi6-touch-lcd-4b board like i do or similar pcb designs with type-c port.
grap the soldering iron (or heatgun idk) and route the resistors on CC1/CC2 to ground (might be wrong please check for sink/source conf) on type-c port
make sure its in type-c source configuration after you verify, use really low power device (maybe a keyboard without a battery to not draw power for its internal components), assuming it to be limited 5v rail that cant supply really much
and share your attempt along with the logs/results with us, really appreciated on your effort!


## Limitations

- package manager has no true use case since there's no suitable/supported upstream for `riscv32_rv32ima` arch, you can only ship the packages embedded. e.g.

OPEN `./guest/openwrt/config.seed` and add `CONFIG_PACKAGE_xxd=y`
make guest

idf.py build; idf.py -p /dev/ttyACM0 -b 921600 flash

## How it works
It uses one 32MB PSRAM chip as the system memory. On startup, it initializes the PSRAM, and load linux kernel Image(an initramfs is embedded which is used as rootfs) and device tree binary from flash to PSRAM, then start the booting.

If you want to flash your custom image, then i recommend using these tools `verify-guest-image.py` / `make guest*` and modify psram.h

## Requirements
- one ESP32-P4 development board. (this one was used to test this project: [esp32-p4-wifi6-touch-lcd-4b](https://www.waveshare.com/esp32-p4-wifi6-touch-lcd-4b.htm)
- a usb-c cable to debug

## How to use

- If your board includes an ESP32-C6 coprocessor, enable the Wi-Fi bridge:

  ```text
  CONFIG_RV32_WIFI_BRIDGE=y
  ```

  Alternatively, configure it through:

  ```sh
  idf.py menuconfig
  ```

- For the Waveshare ESP32-P4-WIFI6-Touch-LCD-4B ST7703 display, enable:

  ```text
  CONFIG_RV32_ST7703_DISPLAY=y
  ```

  This can also be selected through `idf.py menuconfig`. Disable it for
  headless boards.

- Configure your home Wi-Fi or access-point credentials before building. The
  device commonly uses `172.25.115.28`, but its assigned address can vary.
  Check UART0 or run:

  ```sh
  ip address show dev eth0
  ```

- If networking or login fails, inspect the UART0 log and the guest `/init`
  script. The generated root filesystem is available under:

  ```text
  build/guest/rootfs
  ```

- Build the OpenWrt rootfs, Linux Image, and DTBs:

  ```sh
  make guest
  ```

- Build the ESP32-P4 firmware from an initialized ESP-IDF environment:

  ```sh
  idf.py build
  ```

- Flash the complete firmware, kernel, and selected DTB:

  ```sh
  idf.py -p /dev/ttyUSB0 -b 921600 flash
  ```

  `flash` already writes all images registered by the ESP-IDF project;
  `--all` is unnecessary.

i hate using AI, wish i never bought this sub instead autisticly hand write all of these. tried my best not to look codebase like slop... my DM's are open for any kind of questions.

`discord; ok.dusty`
