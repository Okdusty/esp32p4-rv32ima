#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:-clean}
work_root="$repo_root/.guest-work"
output_root="$repo_root/build/guest"
linux_dir="$work_root/linux"

clean_outputs()
{
  if [[ -f "$linux_dir/Makefile" ]]; then
    make -C "$linux_dir" ARCH=riscv clean
  fi

  rm -rf -- "$output_root"
  rm -f -- \
    "$repo_root/main/Image" \
    "$repo_root/main/uc.dtb" \
    "$repo_root/main/uc-nowifi.dtb" \
    "$repo_root/main/uc-headless.dtb" \
    "$repo_root/main/uc-minimal.dtb"
}

case "$mode" in
clean)
  clean_outputs
  echo "Removed guest outputs and Linux objects; retained .guest-work caches"
  ;;
distclean)
  clean_outputs
  rm -rf -- "$work_root"
  echo "Removed guest outputs, Linux objects, and .guest-work caches"
  ;;
*)
  echo "usage: $0 {clean|distclean}" >&2
  exit 2
  ;;
esac
