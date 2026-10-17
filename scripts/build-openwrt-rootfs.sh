#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
work_root=${GUEST_WORK_DIR:-"$repo_root/.guest-work"}
output_root=${GUEST_OUTPUT_DIR:-"$repo_root/build/guest"}
openwrt_dir=${OPENWRT_DIR:-"$work_root/openwrt"}
openwrt_url=${OPENWRT_URL:-https://github.com/openwrt/openwrt.git}
openwrt_revision=${OPENWRT_REVISION:-f0a60eee2fe051741c643ea6118718aae1ef17fb}
jobs=${JOBS:-$(nproc)}
sdkconfig_file=${SDKCONFIG:-"$repo_root/sdkconfig"}

# Machine-local SSH keys, kept out of the tracked overlay; set either
# variable empty to disable it.
if [[ ! -v DROPBEAR_HOST_KEY && -s "$work_root/ssh/dropbear_ed25519_host_key" ]]; then
  DROPBEAR_HOST_KEY="$work_root/ssh/dropbear_ed25519_host_key"
fi
if [[ ! -v DROPBEAR_AUTHORIZED_KEYS && -s "$work_root/ssh/authorized_keys" ]]; then
  DROPBEAR_AUTHORIZED_KEYS="$work_root/ssh/authorized_keys"
fi

# Menuconfig owns the defaults, while environment overrides make automated
# guest-image comparisons possible without editing any tracked file.
guest_console=${RV32_GUEST_CONSOLE_MODE:-}
if [[ -z $guest_console ]]; then
  if [[ -f $sdkconfig_file ]] &&
      grep -q '^CONFIG_RV32_GUEST_CONSOLE_MUX=y$' "$sdkconfig_file"; then
    guest_console=console-mux
  else
    guest_console=display-shell
  fi
fi
case "$guest_console" in
  display-shell|console-mux) ;;
  *)
    echo "error: RV32_GUEST_CONSOLE_MODE must be display-shell or console-mux" >&2
    exit 1
    ;;
esac

guest_dropbear=${RV32_GUEST_DROPBEAR:-}
if [[ -z $guest_dropbear ]]; then
  if [[ ! -f $sdkconfig_file ]] ||
      grep -q '^CONFIG_RV32_GUEST_DROPBEAR=y$' "$sdkconfig_file"; then
    guest_dropbear=1
  else
    guest_dropbear=0
  fi
fi
case "$guest_dropbear" in
  1|y|yes|true) guest_dropbear=1 ;;
  0|n|no|false) guest_dropbear=0 ;;
  *)
    echo "error: RV32_GUEST_DROPBEAR must be 0 or 1" >&2
    exit 1
    ;;
esac

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

# Do not let credentials supplied to an earlier build leak into a later one
# after its environment override has been removed.  Tracked overlay files, if
# present, are copied back immediately below.
rm -f \
  "$openwrt_dir/files/etc/dropbear/dropbear_ed25519_host_key" \
  "$openwrt_dir/files/etc/dropbear/authorized_keys" \
  "$openwrt_dir/files/root/.ssh/authorized_keys"
cp -a "$repo_root/guest/openwrt/overlay/." "$openwrt_dir/"
cp -a "$repo_root/guest/openwrt/files/." "$openwrt_dir/files/"
install -d -m 0755 "$openwrt_dir/files/etc"
printf '%s\n' "$guest_console" >"$openwrt_dir/files/etc/rv32-console"
printf '%s\n' "$guest_dropbear" >"$openwrt_dir/files/etc/rv32-dropbear"

if [[ -n ${DROPBEAR_HOST_KEY:-} ]]; then
  if [[ ! -r $DROPBEAR_HOST_KEY ]]; then
    echo "error: DROPBEAR_HOST_KEY is not readable: $DROPBEAR_HOST_KEY" >&2
    exit 1
  fi
  install -D -m 0600 "$DROPBEAR_HOST_KEY" \
    "$openwrt_dir/files/etc/dropbear/dropbear_ed25519_host_key"
fi

if [[ -n ${DROPBEAR_AUTHORIZED_KEYS:-} ]]; then
  if [[ ! -s $DROPBEAR_AUTHORIZED_KEYS ]]; then
    echo "error: DROPBEAR_AUTHORIZED_KEYS is empty or unreadable: $DROPBEAR_AUTHORIZED_KEYS" >&2
    exit 1
  fi
  # OpenWrt Dropbear uses /etc/dropbear/authorized_keys.  Also install the
  # conventional root path so the same initramfs remains usable if Dropbear
  # or another SSH server is built with the standard authorized-key lookup.
  install -D -m 0600 "$DROPBEAR_AUTHORIZED_KEYS" \
    "$openwrt_dir/files/etc/dropbear/authorized_keys"
  install -D -m 0600 "$DROPBEAR_AUTHORIZED_KEYS" \
    "$openwrt_dir/files/root/.ssh/authorized_keys"
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
	"$repo_root/guest/display-shell.c" \
	-o "$openwrt_dir/files/usr/sbin/display-shell"

env STAGING_DIR="$openwrt_dir/staging_dir" \
	"$cross_gcc" -Os -pipe -mabi=ilp32 -march=rv32ima \
	-fno-caller-saves -fno-plt -fhonour-copts \
	-Wall -Wextra -Werror -fstack-protector-strong \
	-D_FORTIFY_SOURCE=1 -Wl,-z,now -Wl,-z,relro -s \
	-I"$repo_root/main" \
	"$repo_root/guest/rv32-reboot.c" \
	-o "$openwrt_dir/files/usr/sbin/rv32-reboot"

# PID 1. A compiled init avoids starting a shell command per setup step.
env STAGING_DIR="$openwrt_dir/staging_dir" \
	"$cross_gcc" -Os -pipe -mabi=ilp32 -march=rv32ima \
	-fno-caller-saves -fno-plt -fhonour-copts \
	-Wall -Wextra -Werror -fstack-protector-strong \
	-D_FORTIFY_SOURCE=1 -Wl,-z,now -Wl,-z,relro -s \
	"$repo_root/guest/rv32-init.c" \
	-o "$openwrt_dir/files/init"

env STAGING_DIR="$openwrt_dir/staging_dir" \
	"$cross_gcc" -Os -pipe -mabi=ilp32 -march=rv32ima \
	-fno-caller-saves -fno-plt -fhonour-copts \
	-Wformat -Werror=format-security -fstack-protector-strong \
	-D_FORTIFY_SOURCE=1 -Wl,-z,now -Wl,-z,relro -s \
	-I"$repo_root/main" -I"$repo_root/main/display" \
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

# PID 1 is guest/rv32-init.c and the network is configured statically, so
# OpenWrt's procd/netifd/ubus/uci service layer, its package manager and the
# curl/mbedTLS stack never run.  The initramfs is unpacked into tmpfs and
# pinned in guest RAM for the whole uptime, so every file here costs memory.
(
  cd "$output_root/rootfs"
  shopt -s nullglob
  while read -r pattern; do
    [[ -z $pattern || $pattern == \#* ]] && continue
    for path in $pattern; do
      rm -rf -- "$path"
    done
  done <<'PRUNE'
# package manager, curl, TLS, zlib (only apk and curl linked it)
usr/bin/apk
lib/apk
etc/apk
etc/profile.d/apk-cheatsheet.sh
usr/bin/curl
usr/lib/libcurl.so*
usr/lib/libmbed*.so*
etc/ssl
usr/lib/libz.so*
bin/uclient-fetch
usr/bin/wget
usr/lib/libuclient.so
lib/libustream-ssl.so
# procd / ubus / uci / netifd and their libraries
sbin/procd
sbin/init
sbin/askfirst
sbin/ujail
sbin/ubusd
bin/ubus
lib/libubus.so*
usr/lib/libudebug.so
sbin/netifd
lib/netifd
lib/network
usr/lib/libnl-tiny.so*
sbin/uci
lib/libuci.so*
etc/config
etc/uci-defaults
sbin/validate_data
lib/libvalidate.so
sbin/logd
sbin/logread
# usr/libexec keeps sftp-server for scp/sftp over Dropbear
usr/libexec/validate_firmware_image
usr/libexec/network
usr/libexec/login.sh
usr/libexec/logread-ubox
usr/bin/ucode
usr/bin/ucc
usr/bin/utpl
usr/lib/ucode
usr/lib/libucode.so*
usr/bin/jshn
usr/bin/jsonfilter
usr/lib/libjson-c.so*
lib/libblobmsg_json.so*
lib/libjson_script.so*
usr/share/libubox
lib/libubox.so*
# flash/upgrade tooling for real OpenWrt hardware
lib/libfstools.so*
sbin/mount_root
sbin/factoryreset
sbin/jffs2mark
sbin/jffs2reset
sbin/firstboot
sbin/upgraded
sbin/sysupgrade
etc/sysupgrade.conf
lib/upgrade
lib/preinit
etc/preinit
sbin/mtd
usr/bin/fwtool
usr/bin/usign
usr/bin/signify
# the kernel has no loadable modules
sbin/kmodloader
sbin/insmod
sbin/lsmod
sbin/modinfo
sbin/modprobe
sbin/rmmod
etc/modules.conf
lib/config
lib/firmware
# hotplug / init scripts
sbin/hotplug-call
etc/hotplug.d
etc/hotplug.json
etc/hotplug-preinit.json
usr/sbin/ntpd-hotplug
sbin/udevtrigger
sbin/urngd
sbin/urandom_seed
etc/init.d
etc/rc.d
etc/rc.common
etc/rc.button
sbin/service
sbin/reload_config
sbin/led.sh
sbin/ifup
sbin/ifdown
sbin/ifstatus
sbin/devstatus
bin/board_detect
bin/config_generate
bin/ipcalc.sh
etc/board.d
etc/diag.sh
lib/functions
lib/functions.sh
usr/bin/systemctl
etc/banner.failsafe
# curses add-on libraries; setterm needs only libncursesw
usr/lib/libform*.so*
usr/lib/libmenu*.so*
usr/lib/libpanel*.so*
PRUNE
)
rm -rf "$output_root/rootfs.previous"
mv "$rootfs_archive" "$output_root/openwrt-rootfs-rv32.tar.gz"

echo "OpenWrt rootfs: $output_root/rootfs"
echo "Cross prefix:    $cross_prefix"
echo "Console:         $guest_console"
echo "Dropbear:        $([[ $guest_dropbear == 1 ]] && echo enabled || echo disabled)"
