#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
linux_dir=${LINUX_DIR:-"$repo_root/.guest-work/linux"}
dtc=${DTC:-"$linux_dir/scripts/dtc/dtc"}

if [[ ! -x "$dtc" ]]; then
	if command -v dtc >/dev/null 2>&1; then
		dtc=$(command -v dtc)
	else
		echo "error: no dtc; build Linux first or set DTC=/path/to/dtc" >&2
		exit 1
	fi
fi

build_one()
{
	local wifi=$1
	local display=$2
	local output=$3
	local preprocessed
	preprocessed=$(mktemp "${TMPDIR:-/tmp}/uc-dts.XXXXXX")
	trap 'rm -f "$preprocessed"' RETURN

	local cpp_flags=(-P -nostdinc -undef -x assembler-with-cpp)
	if [[ "$wifi" == 1 ]]; then
		cpp_flags+=(-DRV32_WIFI_BRIDGE=1)
	fi
	case "$display" in
		st7703)
			cpp_flags+=(
				-DRV32_DISPLAY_ENABLED=1
				-DRV32_DISPLAY_WIDTH=720
				-DRV32_DISPLAY_HEIGHT=720
				-DRV32_DISPLAY_STRIDE=1440
				-DRV32_DISPLAY_STAGING_OFFSET=0x000fd200
				-DRV32_DISPLAY_FENCE_OFFSET=0x0011d20c
				-DRV32_DISPLAY_ACCEL_OFFSET=0x0011d210
				-DRV32_DISPLAY_APERTURE_SIZE=0x0011e000)
			;;
		st7789)
			cpp_flags+=(
				-DRV32_DISPLAY_ENABLED=1
				-DRV32_DISPLAY_WIDTH=240
				-DRV32_DISPLAY_HEIGHT=240
				-DRV32_DISPLAY_STRIDE=480
				-DRV32_DISPLAY_STAGING_OFFSET=0x0001c200
				-DRV32_DISPLAY_FENCE_OFFSET=0x0003c20c
				-DRV32_DISPLAY_ACCEL_OFFSET=0x0003c210
				-DRV32_DISPLAY_APERTURE_SIZE=0x0003d000)
			;;
		ssd1306-32)
			cpp_flags+=(
				-DRV32_DISPLAY_ENABLED=1
				-DRV32_DISPLAY_WIDTH=128
				-DRV32_DISPLAY_HEIGHT=32
				-DRV32_DISPLAY_STRIDE=256
				-DRV32_DISPLAY_STAGING_OFFSET=0x00002000
				-DRV32_DISPLAY_FENCE_OFFSET=0x0002200c
				-DRV32_DISPLAY_ACCEL_OFFSET=0x00022010
				-DRV32_DISPLAY_APERTURE_SIZE=0x00023000)
			;;
		ssd1306-64)
			cpp_flags+=(
				-DRV32_DISPLAY_ENABLED=1
				-DRV32_DISPLAY_WIDTH=128
				-DRV32_DISPLAY_HEIGHT=64
				-DRV32_DISPLAY_STRIDE=256
				-DRV32_DISPLAY_STAGING_OFFSET=0x00004000
				-DRV32_DISPLAY_FENCE_OFFSET=0x0002400c
				-DRV32_DISPLAY_ACCEL_OFFSET=0x00024010
				-DRV32_DISPLAY_APERTURE_SIZE=0x00025000)
			;;
		none) ;;
		*)
			echo "error: unknown display backend: $display" >&2
			return 2
			;;
	esac
	cpp "${cpp_flags[@]}" "$repo_root/main/uc.dts" "$preprocessed"
	"$dtc" -I dts -O dtb -o "$output" "$preprocessed"
}

case ${1:-all} in
	wifi)
		build_one 1 st7703 "${2:-$repo_root/main/uc.dtb}"
		;;
	nowifi)
		build_one 0 st7703 "${2:-$repo_root/main/uc-nowifi.dtb}"
		;;
	headless)
		build_one 1 none "${2:-$repo_root/main/uc-headless.dtb}"
		;;
	minimal)
		build_one 0 none "${2:-$repo_root/main/uc-minimal.dtb}"
		;;
	ssd1306-32-wifi)
		build_one 1 ssd1306-32 "${2:-$repo_root/main/uc-ssd1306-32.dtb}"
		;;
	ssd1306-32-nowifi)
		build_one 0 ssd1306-32 "${2:-$repo_root/main/uc-ssd1306-32-nowifi.dtb}"
		;;
	ssd1306-64-wifi)
		build_one 1 ssd1306-64 "${2:-$repo_root/main/uc-ssd1306-64.dtb}"
		;;
	ssd1306-64-nowifi)
		build_one 0 ssd1306-64 "${2:-$repo_root/main/uc-ssd1306-64-nowifi.dtb}"
		;;
	st7789-wifi)
		build_one 1 st7789 "${2:-$repo_root/main/uc-st7789.dtb}"
		;;
	st7789-nowifi)
		build_one 0 st7789 "${2:-$repo_root/main/uc-st7789-nowifi.dtb}"
		;;
	all)
		build_one 1 st7703 "$repo_root/main/uc.dtb"
		build_one 0 st7703 "$repo_root/main/uc-nowifi.dtb"
		build_one 1 none "$repo_root/main/uc-headless.dtb"
		build_one 0 none "$repo_root/main/uc-minimal.dtb"
		build_one 1 ssd1306-32 "$repo_root/main/uc-ssd1306-32.dtb"
		build_one 0 ssd1306-32 "$repo_root/main/uc-ssd1306-32-nowifi.dtb"
		build_one 1 ssd1306-64 "$repo_root/main/uc-ssd1306-64.dtb"
		build_one 0 ssd1306-64 "$repo_root/main/uc-ssd1306-64-nowifi.dtb"
		build_one 1 st7789 "$repo_root/main/uc-st7789.dtb"
		build_one 0 st7789 "$repo_root/main/uc-st7789-nowifi.dtb"
		;;
	*)
		echo "usage: $0 [all|wifi|nowifi|headless|minimal|ssd1306-32-wifi|ssd1306-32-nowifi|ssd1306-64-wifi|ssd1306-64-nowifi|st7789-wifi|st7789-nowifi] [output.dtb]" >&2
		exit 2
		;;
esac
