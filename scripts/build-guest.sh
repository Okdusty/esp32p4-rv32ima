#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

"$repo_root/scripts/build-openwrt-rootfs.sh"
"$repo_root/scripts/build-linux-image.sh"

echo
echo "Guest build complete. ESP-IDF will flash main/Image and the DTB selected"
echo "idf.py build/flash."
