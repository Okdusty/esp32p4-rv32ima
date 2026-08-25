#!/bin/sh
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_binary=$(mktemp "${TMPDIR:-/tmp}/rv32-emulator-test.XXXXXX")
trap 'rm -f "$test_binary"' EXIT HUP INT TERM

"${HOSTCC:-cc}" -std=gnu11 -O2 -Wall -Wextra \
  -Wno-empty-body -Wno-sign-compare -Wno-unused-parameter \
  "$repo_dir/tests/mini-rv32ima-cache.c" -o "$test_binary"
"$test_binary"
