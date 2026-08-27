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
"$config" --file "$linux_dir/.config" --disable SMP
"$config" --file "$linux_dir/.config" --enable FB_RV32_PV
"$config" --file "$linux_dir/.config" --enable FB_TILEBLITTING
"$config" --file "$linux_dir/.config" --enable FRAMEBUFFER_CONSOLE_LEGACY_ACCELERATION

make -C "$linux_dir" ARCH=riscv CROSS_COMPILE="$cross_prefix" olddefconfig
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
