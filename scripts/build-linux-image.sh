#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
work_root=${GUEST_WORK_DIR:-"$repo_root/.guest-work"}
output_root=${GUEST_OUTPUT_DIR:-"$repo_root/build/guest"}
linux_dir=${LINUX_DIR:-"$work_root/linux"}
linux_url=${LINUX_URL:-https://github.com/torvalds/linux.git}
linux_revision=${LINUX_REVISION:-26260251022fbc2f248a3d747a9b2b961b18d2d8}
rootfs_dir=${ROOTFS_DIR:-"$output_root/rootfs"}
jobs=${JOBS:-$(nproc)}

mkdir -p "$work_root" "$output_root"
if [[ ! -d "$rootfs_dir" ]]; then
  echo "error: rootfs not found at $rootfs_dir" >&2
  echo "run scripts/build-openwrt-rootfs.sh first" >&2
  exit 1
fi

if [[ ! -d "$linux_dir/.git" ]]; then
  git clone --filter=blob:none "$linux_url" "$linux_dir"
  git -C "$linux_dir" checkout --detach "$linux_revision"
elif [[ $(git -C "$linux_dir" rev-parse HEAD) != "$linux_revision" ]]; then
  echo "error: $linux_dir is not at pinned revision $linux_revision" >&2
  echo "set LINUX_DIR to a clean checkout or move the existing directory" >&2
  exit 1
fi

patch_file="$repo_root/linux-rv32-dma-coherency.patch"
if git -C "$linux_dir" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
  :
elif git -C "$linux_dir" apply --check "$patch_file"; then
  git -C "$linux_dir" apply "$patch_file"
else
  echo "error: Linux RV32 patch is neither applicable nor already applied" >&2
  exit 1
fi

pvfb_patch="$repo_root/linux-rv32-pvfb.patch"
if git -C "$linux_dir" apply --reverse --check "$pvfb_patch" >/dev/null 2>&1; then
  :
elif git -C "$linux_dir" apply --check "$pvfb_patch"; then
  git -C "$linux_dir" apply "$pvfb_patch"
else
  echo "error: Linux RV32 pvfb patch is neither applicable nor already applied" >&2
  exit 1
fi

cursor_patch="$repo_root/linux-fbcon-tile-cursor.patch"
if git -C "$linux_dir" apply --reverse --check "$cursor_patch" >/dev/null 2>&1; then
  :
elif git -C "$linux_dir" apply --check "$cursor_patch"; then
  git -C "$linux_dir" apply "$cursor_patch"
else
  echo "error: Linux fbcon tile-cursor patch is neither applicable nor already applied" >&2
  exit 1
fi

dwc2_fifo_patch="$repo_root/linux-dwc2-bounded-host-fifo.patch"
if git -C "$linux_dir" apply --reverse --check "$dwc2_fifo_patch" >/dev/null 2>&1; then
  :
elif git -C "$linux_dir" apply --check "$dwc2_fifo_patch"; then
  git -C "$linux_dir" apply "$dwc2_fifo_patch"
else
  echo "error: Linux DWC2 FIFO patch is neither applicable nor already applied" >&2
  exit 1
fi
# The emulator pays per guest instruction and per call into the JIT's
# region transitions, so the hot kernel paths (mm, scheduler, timekeeping,
# locking, entry, VFS, EROFS, block, riscv mm, the tree/array libraries,
# networking and ttys) are built at -O2 while the rest stays -Os to fit the 8 MiB
# kernel partition.  lib/math, lib/lz4 and arch/riscv/lib keep their flags:
# the firmware's host routines match those functions by instruction words.
hot_o2_patch="$repo_root/linux-rv32-hot-o2.patch"
if git -C "$linux_dir" apply --reverse --check "$hot_o2_patch" >/dev/null 2>&1; then
  :
elif git -C "$linux_dir" apply --check "$hot_o2_patch"; then
  git -C "$linux_dir" apply "$hot_o2_patch"
else
  echo "error: Linux hot-path -O2 patch is neither applicable nor already applied" >&2
  exit 1
fi
# vm.mmap_readaround_kb: rv32-init caps the read-around of page faults in
# mapped files (program code) at 16 KiB while read() keeps 128 KiB.  With the
# device's default (8 MiB on virtio-blk) every miss in a desktop program read
# megabytes ahead and evicted the rest of a 20 MB guest's page cache.
mmap_readaround_patch="$repo_root/linux-rv32-mmap-readaround.patch"
if git -C "$linux_dir" apply --reverse --check "$mmap_readaround_patch" >/dev/null 2>&1; then
  :
elif git -C "$linux_dir" apply --check "$mmap_readaround_patch"; then
  git -C "$linux_dir" apply "$mmap_readaround_patch"
else
  echo "error: Linux mmap read-around patch is neither applicable nor already applied" >&2
  exit 1
fi
# DMA buffers aligned to the host's 128-byte L2 line (the device tree's
# riscv,cbom-block-size matches): the emulator invalidates whole host lines for
# the guest's cbo.inval and after USB IN transfers, so a 64-byte-aligned buffer
# lost the CPU's writes to its line neighbour (dwc2 disconnect oops).
dma_minalign_patch="$repo_root/linux-rv32-dma-minalign.patch"
if git -C "$linux_dir" apply --reverse --check "$dma_minalign_patch" >/dev/null 2>&1; then
  :
elif git -C "$linux_dir" apply --check "$dma_minalign_patch"; then
  git -C "$linux_dir" apply "$dma_minalign_patch"
else
  echo "error: Linux DMA alignment patch is neither applicable nor already applied" >&2
  exit 1
fi
# Files mapped in place from flash (cramfs on an "mtd-rom" MTD, firmware
# CONFIG_RV32_FLASH_XIP): riscv's set_pte_at must not look for a struct page
# behind an executable PTE that maps flash, and cramfs must not read the last
# page of a page-multiple file as a hole.
xip_patch="$repo_root/linux-rv32-xip-cramfs.patch"
if git -C "$linux_dir" apply --reverse --check "$xip_patch" >/dev/null 2>&1; then
  :
elif git -C "$linux_dir" apply --check "$xip_patch"; then
  git -C "$linux_dir" apply "$xip_patch"
else
  echo "error: Linux XIP cramfs patch is neither applicable nor already applied" >&2
  exit 1
fi
cp "$repo_root/guest/linux/rv32-pvfb.c" \
  "$linux_dir/drivers/video/fbdev/rv32-pvfb.c"

generated_cross_prefix=
if [[ -s "$output_root/openwrt-cross-prefix" ]]; then
  generated_cross_prefix=$(<"$output_root/openwrt-cross-prefix")
fi

if [[ -n ${CROSS_COMPILE:-} && -x "${CROSS_COMPILE}gcc" ]]; then
  cross_prefix=$CROSS_COMPILE
elif [[ -n "$generated_cross_prefix" &&
        -x "${generated_cross_prefix}gcc" ]]; then
  if [[ -n ${CROSS_COMPILE:-} ]]; then
    echo "warning: ignoring unusable CROSS_COMPILE=${CROSS_COMPILE}" >&2
    echo "using generated OpenWrt toolchain: $generated_cross_prefix" >&2
  fi
  cross_prefix=$generated_cross_prefix
elif [[ -n ${CROSS_COMPILE:-} ]]; then
  echo "error: ${CROSS_COMPILE}gcc is not executable" >&2
  exit 1
else
  echo "error: OpenWrt cross-prefix is absent; run make guest-rootfs first" >&2
  exit 1
fi

# OpenWrt's relocatable compiler wrapper consults STAGING_DIR even for the
# freestanding kernel build.  Derive it from the selected prefix so the same
# toolchain works here exactly as it does inside OpenWrt package.mk.  External
# toolchains do not match this layout and are left untouched.
cross_gcc=${cross_prefix}gcc
if [[ "$cross_gcc" == */staging_dir/toolchain-*/bin/* ]]; then
  export STAGING_DIR
  toolchain_bin=$(dirname "$cross_gcc")
  toolchain_root=$(dirname "$toolchain_bin")
  STAGING_DIR=$(dirname "$toolchain_root")
fi

cp "$repo_root/guest/linux/linux.config" "$linux_dir/.config"
config="$linux_dir/scripts/config"
"$config" --file "$linux_dir/.config" \
  --set-str INITRAMFS_SOURCE "$rootfs_dir $repo_root/main/rootfs.devices"
"$config" --file "$linux_dir/.config" --enable BLK_DEV_INITRD
"$config" --file "$linux_dir/.config" --enable INITRAMFS_COMPRESSION_NONE
"$config" --file "$linux_dir/.config" --disable INITRAMFS_COMPRESSION_GZIP
"$config" --file "$linux_dir/.config" --disable INITRAMFS_COMPRESSION_XZ
"$config" --file "$linux_dir/.config" --enable 32BIT
"$config" --file "$linux_dir/.config" --enable ARCH_RV32I
"$config" --file "$linux_dir/.config" --enable MMU
"$config" --file "$linux_dir/.config" --enable RISCV_SBI
"$config" --file "$linux_dir/.config" --enable POWER_RESET
"$config" --file "$linux_dir/.config" --enable POWER_RESET_SYSCON
"$config" --file "$linux_dir/.config" --enable POWER_RESET_SYSCON_POWEROFF
"$config" --file "$linux_dir/.config" --disable SMP
"$config" --file "$linux_dir/.config" --enable FB_RV32_PV
"$config" --file "$linux_dir/.config" --enable FB_TILEBLITTING
"$config" --file "$linux_dir/.config" --enable FRAMEBUFFER_CONSOLE_LEGACY_ACCELERATION

# Build Linux's upstream virtio-gpu 2D/KMS driver into every guest Image. The
# ESP firmware menu option decides whether a device is exposed; when disabled
# the virtio-mmio slot reports device ID zero and this driver remains dormant.
# Keep DRM fbdev emulation off so the existing simplefb/accelerated tty stays
# the fallback console instead of allocating a second full-size shadow fb.
"$config" --file "$linux_dir/.config" --enable DRM
"$config" --file "$linux_dir/.config" --enable DRM_KMS_HELPER
"$config" --file "$linux_dir/.config" --enable DRM_VIRTIO_GPU
"$config" --file "$linux_dir/.config" --enable DRM_VIRTIO_GPU_KMS
"$config" --file "$linux_dir/.config" --disable DRM_FBDEV_EMULATION

# The Nano USB-A connector is wired directly to the ESP32-P4 USB 2.0 HS
# controller. Keep the complete host path built into the Image: DWC2 owns the
# bus, SCSI exposes inserted memory sticks, and the common removable-media
# filesystems make the result useful without loading modules from the rootfs.
"$config" --file "$linux_dir/.config" --enable BLOCK
"$config" --file "$linux_dir/.config" --enable VIRTIO_BLK
"$config" --file "$linux_dir/.config" --enable MSDOS_PARTITION
"$config" --file "$linux_dir/.config" --enable EFI_PARTITION
"$config" --file "$linux_dir/.config" --enable SCSI
"$config" --file "$linux_dir/.config" --disable SCSI_PROC_FS
"$config" --file "$linux_dir/.config" --enable BLK_DEV_SD
"$config" --file "$linux_dir/.config" --enable USB_SUPPORT
"$config" --file "$linux_dir/.config" --enable USB
"$config" --file "$linux_dir/.config" --enable USB_ANNOUNCE_NEW_DEVICES
"$config" --file "$linux_dir/.config" --enable USB_DEFAULT_PERSIST
"$config" --file "$linux_dir/.config" --enable USB_DWC2
"$config" --file "$linux_dir/.config" --enable USB_DWC2_HOST
"$config" --file "$linux_dir/.config" --disable USB_DWC2_PERIPHERAL
"$config" --file "$linux_dir/.config" --disable USB_DWC2_DUAL_ROLE
"$config" --file "$linux_dir/.config" --disable USB_DWC2_DEBUG
"$config" --file "$linux_dir/.config" --enable USB_STORAGE
"$config" --file "$linux_dir/.config" --enable HID_SUPPORT
"$config" --file "$linux_dir/.config" --enable HID
"$config" --file "$linux_dir/.config" --enable HID_GENERIC
"$config" --file "$linux_dir/.config" --enable USB_HID
"$config" --file "$linux_dir/.config" --enable VFAT_FS
"$config" --file "$linux_dir/.config" --enable EXFAT_FS
"$config" --file "$linux_dir/.config" --enable EXT4_FS
# With the USB port in keyboard-and-mouse mode the firmware reads them on CPU1
# and presents a virtio-input device instead of the DWC2 controller.
"$config" --file "$linux_dir/.config" --enable INPUT_EVDEV
"$config" --file "$linux_dir/.config" --enable VIRTIO_INPUT
# The read-only flash disk (desktop) is an LZ4-compressed EROFS image, the X
# server's event loop requires epoll, and curl creates an eventfd for every
# transfer.
"$config" --file "$linux_dir/.config" --enable EROFS_FS
"$config" --file "$linux_dir/.config" --enable EROFS_FS_ZIP
"$config" --file "$linux_dir/.config" --enable EPOLL
"$config" --file "$linux_dir/.config" --enable EVENTFD
# Swap in compressed RAM (rv32-init sets up /dev/zram0): idle anonymous and
# tmpfs pages are compressed, so the page cache keeps program code instead of
# reading it from flash again and again.
"$config" --file "$linux_dir/.config" --enable SWAP
"$config" --file "$linux_dir/.config" --enable ZRAM
"$config" --file "$linux_dir/.config" --enable ZRAM_BACKEND_LZ4
"$config" --file "$linux_dir/.config" --enable ZRAM_DEF_COMP_LZ4
# The multi-generational LRU: with swap, the classic LRU also pushed anonymous
# pages to zram during streaming reads (cat of the desktop disk +23 %); MGLRU
# keeps the swap for idle pages and refaults less during a desktop start.
"$config" --file "$linux_dir/.config" --enable LRU_GEN
"$config" --file "$linux_dir/.config" --enable LRU_GEN_ENABLED
# Desktop programs, libraries and fonts mapped in place from the "xip" flash
# partition: an "mtd-rom" physmap device (uc.dts) mounted as cramfs, whose
# uncompressed page-aligned files are mapped without page cache.
"$config" --file "$linux_dir/.config" --enable MTD
"$config" --file "$linux_dir/.config" --enable MTD_ROM
"$config" --file "$linux_dir/.config" --enable MTD_PHYSMAP
"$config" --file "$linux_dir/.config" --enable MTD_PHYSMAP_OF
"$config" --file "$linux_dir/.config" --enable CRAMFS
"$config" --file "$linux_dir/.config" --enable CRAMFS_MTD
"$config" --file "$linux_dir/.config" --disable CRAMFS_BLOCKDEV
"$config" --file "$linux_dir/.config" --enable NLS_CODEPAGE_437
"$config" --file "$linux_dir/.config" --enable NLS_ISO8859_1
"$config" --file "$linux_dir/.config" --enable NLS_UTF8

make -C "$linux_dir" ARCH=riscv CROSS_COMPILE="$cross_prefix" olddefconfig

# scripts/config records requests even when a symbol's dependencies are not
# met.  Fail here instead of producing an Image that silently lacks the
# standard guest drivers expected by the reset and display firmware.
for required in \
  CONFIG_POWER_RESET_SYSCON=y \
  CONFIG_POWER_RESET_SYSCON_POWEROFF=y \
  CONFIG_DRM=y \
  CONFIG_DRM_VIRTIO_GPU=y \
  CONFIG_DRM_VIRTIO_GPU_KMS=y \
  CONFIG_EROFS_FS=y \
  CONFIG_EROFS_FS_ZIP=y \
  CONFIG_EPOLL=y \
  CONFIG_EVENTFD=y \
  CONFIG_MTD_PHYSMAP_OF=y \
  CONFIG_MTD_ROM=y \
  CONFIG_CRAMFS_MTD=y \
  CONFIG_VIRTIO_INPUT=y; do
  if ! grep -qx "$required" "$linux_dir/.config"; then
    echo "error: Linux configuration did not retain $required" >&2
    exit 1
  fi
done
if ! grep -qx '# CONFIG_DRM_FBDEV_EMULATION is not set' \
  "$linux_dir/.config"; then
  echo "error: DRM fbdev emulation must remain disabled" >&2
  exit 1
fi

# rv32-init links the initramfs's large files to the desktop disk's copy of
# them only when the disk was built from this same rootfs: this ID, which
# build-desktop-rofs.sh copies along.
rm -f "$rootfs_dir/etc/rv32-base-id"
tar --sort=name --mtime=@0 --owner=0 --group=0 --numeric-owner \
  -C "$rootfs_dir" -cf - . | sha256sum | cut -c1-32 \
  >"$rootfs_dir/etc/rv32-base-id"

make -C "$linux_dir" -j"$jobs" ARCH=riscv CROSS_COMPILE="$cross_prefix" Image

image="$linux_dir/arch/riscv/boot/Image"
if [[ ! -f "$image" ]]; then
  echo "error: kernel build did not produce $image" >&2
  exit 1
fi
"$repo_root/scripts/verify-guest-image.py" "$image"

cp "$image" "$output_root/Image"
LINUX_DIR="$linux_dir" "$repo_root/scripts/build-dtb.sh" all
cp "$image" "$repo_root/main/Image"

echo "Installed Linux Image: $repo_root/main/Image"
echo "Installed DTBs for the Wi-Fi/display configuration matrix"
