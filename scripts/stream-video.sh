#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 4 ]]; then
  echo "usage: $0 INPUT [HOST [PORT [FPS]]]" >&2
  exit 2
fi

input=$1
host=${2:-172.25.115.28}
port=${3:-5000}
fps=${4:-10}

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "error: ffmpeg is required on the sending computer" >&2
  exit 1
fi

exec ffmpeg -hide_banner -loglevel warning -re -i "$input" -an \
  -vf "scale=256:144:force_original_aspect_ratio=decrease,pad=256:144:(ow-iw)/2:(oh-ih)/2:black,fps=$fps" \
  -pix_fmt rgb565le -f rawvideo "tcp://$host:$port"
