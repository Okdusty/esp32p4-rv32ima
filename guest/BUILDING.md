# RV32 guest build

The flashable `main/Image` is an uncompressed 32-bit RISC-V Linux `Image`.
Its initramfs is not appended afterward: Linux turns the complete OpenWrt
root directory into `usr/initramfs_inc_data` while linking the kernel, so a
rootfs change always requires relinking `Image`.

Run the complete build from the repository root:

```sh
make guest
```

This performs the following reproducible pipeline:

1. Check out pinned OpenWrt sources below `.guest-work/openwrt`.
2. Apply `guest/openwrt/openwrt-rv32.patch` and its RV32 target overlay.
3. Apply `guest/openwrt/config.seed` using OpenWrt `make defconfig`, compile
   the regular `riscv32_rv32ima` musl packages, and assemble the rootfs with
   OpenWrt's `package/compile` and `package/install` targets. The OpenWrt
   target kernel is deliberately not built.
4. Compile `guest/console-mux.c` with that OpenWrt cross compiler.
5. Check out pinned Linux sources below `.guest-work/linux`.
6. Copy the known-good kernel configuration, then set all required and
   path-dependent options with Linux `scripts/config`; no `.config` text is
   edited manually.
7. Run `make ARCH=riscv ... Image` in that standalone, pinned Linux tree.
   Linux embeds the assembled OpenWrt rootfs as its initramfs. The result is
   `.guest-work/linux/arch/riscv/boot/Image`.
8. Validate the RISC-V header, 4 MiB text offset, PSRAM/DTB boundary, and flash
   partition limit before installing the result as `main/Image`.
9. Preprocess `main/uc.dts` and generate the DTBs covering Wi-Fi with the
   ST7703, ST7789 240x240, SSD1306 128x32, SSD1306 128x64, and headless
   configurations.
   Disabled devices are absent from the hardware description instead of
   probing non-existent emulator MMIO.

The ESP-IDF CMake build selects the correct DTB from
`CONFIG_RV32_WIFI_BRIDGE` and the selected display backend, and registers both
the kernel and DTB partitions in the normal `idf.py flash` operation.

For isolated configuration testing, append one of the defaults in
`guest/configs/` after the project defaults. For example, a headless build is:

```sh
idf.py -B build-minimal \
  -D "SDKCONFIG=$PWD/build-minimal/sdkconfig" \
  -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;guest/configs/minimal.defaults' \
  build
```

The component dependencies remain declared unconditionally because ESP-IDF
discovers them during an early CMake pass before project Kconfig values are
available. The display and Wi-Fi source files, linker placement, public APIs,
and selected Linux DTB are nevertheless all conditional.

Useful targets:

```sh
make guest-rootfs       # OpenWrt packages and rootfs only
make guest-image        # Linux Image and display/Wi-Fi DTBs, existing rootfs
make guest-dtb          # regenerate only the display/Wi-Fi DTBs
make guest-verify       # validate the currently installed main/Image
make clean              # remove outputs/Linux objects; retain download caches
make distclean          # also remove .guest-work checkouts and caches
```

The source and output locations can be overridden without editing scripts:

```sh
OPENWRT_DIR=/path/to/openwrt LINUX_DIR=/path/to/linux JOBS=8 make guest
```

`DROPBEAR_HOST_KEY=/path/to/dropbear_ed25519_host_key` embeds an existing
Dropbear host key. Without it, `/init` generates an Ed25519 key at boot. The
default root password in this development image is `test`.
