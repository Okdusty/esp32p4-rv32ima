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
	if [[ "$display" == 1 ]]; then
		cpp_flags+=(-DRV32_ST7703_DISPLAY=1)
	fi
	cpp "${cpp_flags[@]}" "$repo_root/main/uc.dts" "$preprocessed"
	"$dtc" -I dts -O dtb -o "$output" "$preprocessed"
}

case ${1:-all} in
	wifi)
		build_one 1 1 "${2:-$repo_root/main/uc.dtb}"
		;;
	nowifi)
		build_one 0 1 "${2:-$repo_root/main/uc-nowifi.dtb}"
		;;
	headless)
		build_one 1 0 "${2:-$repo_root/main/uc-headless.dtb}"
		;;
	minimal)
		build_one 0 0 "${2:-$repo_root/main/uc-minimal.dtb}"
		;;
	all)
		build_one 1 1 "$repo_root/main/uc.dtb"
		build_one 0 1 "$repo_root/main/uc-nowifi.dtb"
		build_one 1 0 "$repo_root/main/uc-headless.dtb"
		build_one 0 0 "$repo_root/main/uc-minimal.dtb"
		;;
	*)
		echo "usage: $0 [all|wifi|nowifi|headless|minimal] [output.dtb]" >&2
		exit 2
		;;
esac
