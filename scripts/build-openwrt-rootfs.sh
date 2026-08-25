#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
work_root=${GUEST_WORK_DIR:-"$repo_root/.guest-work"}
output_root=${GUEST_OUTPUT_DIR:-"$repo_root/build/guest"}
openwrt_dir=${OPENWRT_DIR:-"$work_root/openwrt"}
openwrt_url=${OPENWRT_URL:-https://github.com/openwrt/openwrt.git}
openwrt_revision=${OPENWRT_REVISION:-f0a60eee2fe051741c643ea6118718aae1ef17fb}
jobs=${JOBS:-$(nproc)}

mkdir -p "$work_root" "$output_root"

if [[ ! -d "$openwrt_dir/.git" ]]; then
  git clone --filter=blob:none "$openwrt_url" "$openwrt_dir"
  git -C "$openwrt_dir" checkout --detach "$openwrt_revision"
elif [[ $(git -C "$openwrt_dir" rev-parse HEAD) != "$openwrt_revision" ]]; then
  echo "error: $openwrt_dir is not at pinned revision $openwrt_revision" >&2
  echo "set OPENWRT_DIR to a clean checkout or move the existing directory" >&2
  exit 1
fi

patch_file="$repo_root/guest/openwrt/openwrt-rv32.patch"
if git -C "$openwrt_dir" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
  :
elif git -C "$openwrt_dir" apply --check "$patch_file"; then
  git -C "$openwrt_dir" apply "$patch_file"
else
  echo "error: OpenWrt RV32 patch is neither applicable nor already applied" >&2
  exit 1
fi

cp -a "$repo_root/guest/openwrt/overlay/." "$openwrt_dir/"
cp -a "$repo_root/guest/openwrt/files/." "$openwrt_dir/files/"

if [[ -n ${DROPBEAR_HOST_KEY:-} ]]; then
  install -D -m 0600 "$DROPBEAR_HOST_KEY" \
    "$openwrt_dir/files/etc/dropbear/dropbear_ed25519_host_key"
fi

if [[ -x "$openwrt_dir/scripts/feeds" ]]; then
  (
    cd "$openwrt_dir"
    ./scripts/feeds update -a
    ./scripts/feeds install -a
  )
fi

cp "$repo_root/guest/openwrt/config.seed" "$openwrt_dir/.config"
make -C "$openwrt_dir" defconfig

make -C "$openwrt_dir" -j"$jobs" tools/install toolchain/install

cross_gcc=$(find "$openwrt_dir/staging_dir" \( -type f -o -type l \) |
  grep '/toolchain-riscv32_rv32ima_[^/]*/bin/riscv32-openwrt-linux-musl-gcc$' |
  head -n 1)
if [[ -z "$cross_gcc" || ! -x "$cross_gcc" ]]; then
  echo "error: OpenWrt RV32 cross compiler was not produced" >&2
  exit 1
fi
cross_prefix=${cross_gcc%gcc}
printf '%s\n' "$cross_prefix" >"$output_root/openwrt-cross-prefix"

mkdir -p "$openwrt_dir/files/usr/sbin"
mkdir -p "$openwrt_dir/files/usr/bin"
# OpenWrt's compiler is a relocatable wrapper.  Outside package.mk it needs
# STAGING_DIR explicitly, and GCC's generic RISC-V default may otherwise emit
# a double-float object which cannot link against this RV32IMA/ILP32 musl.
env STAGING_DIR="$openwrt_dir/staging_dir" \
	"$cross_gcc" -Os -pipe -mabi=ilp32 -march=rv32ima \
	-fno-caller-saves -fno-plt -fhonour-copts \
	-Wformat -Werror=format-security -fstack-protector-strong \
	-D_FORTIFY_SOURCE=1 -Wl,-z,now -Wl,-z,relro -s \
	-I"$repo_root/main" \
	"$repo_root/guest/console-mux.c" \
	-o "$openwrt_dir/files/usr/sbin/console-mux"

env STAGING_DIR="$openwrt_dir/staging_dir" \
	"$cross_gcc" -Os -pipe -mabi=ilp32 -march=rv32ima \
	-fno-caller-saves -fno-plt -fhonour-copts \
	-Wformat -Werror=format-security -fstack-protector-strong \
	-D_FORTIFY_SOURCE=1 -Wl,-z,now -Wl,-z,relro -s \
	-I"$repo_root/main" \
	"$repo_root/guest/fbstream.c" \
	-o "$openwrt_dir/files/usr/bin/fbstream"

# Populate the download cache in parallel, then verify it serially so a
# transient per-package error cannot hide in OpenWrt's aggregate target.
make -C "$openwrt_dir" -j"$jobs" download
make -C "$openwrt_dir" -j1 download

# build-linux-image.sh supplies the guest kernel. Building OpenWrt's own
# target/linux is redundant and couples this userspace stage to unrelated
# OpenWrt kernel Kconfig changes. Build/install only the selected packages;
# package/install assembles the complete TARGET_DIR, including files/.
make -C "$openwrt_dir" -j"$jobs" V=s package/compile
make -C "$openwrt_dir" -j1 V=s package/install

rootfs_source=$(find "$openwrt_dir/build_dir" -mindepth 2 -maxdepth 2 \
  -type d -name 'root-esp32p4rv32' | sort | head -n 1)
if [[ -z "$rootfs_source" ]]; then
  echo "error: OpenWrt package/install did not assemble root-esp32p4rv32" >&2
  exit 1
fi

rootfs_archive="$output_root/openwrt-rootfs-rv32.tar.gz.new"
tar --numeric-owner --owner=0 --group=0 -czf "$rootfs_archive" \
  -C "$rootfs_source" .

new_rootfs=$(mktemp -d "$output_root/rootfs.new.XXXXXX")
tar -xzf "$rootfs_archive" -C "$new_rootfs"
if [[ -e "$output_root/rootfs" ]]; then
  mv "$output_root/rootfs" "$output_root/rootfs.previous"
fi
mv "$new_rootfs" "$output_root/rootfs"
rm -rf "$output_root/rootfs.previous"
mv "$rootfs_archive" "$output_root/openwrt-rootfs-rv32.tar.gz"

echo "OpenWrt rootfs: $output_root/rootfs"
echo "Cross prefix:    $cross_prefix"
