#!/usr/bin/env bash
set -euo pipefail

host=${RV32_SSH_HOST:-192.168.1.220}
user=${RV32_SSH_USER:-root}
persist=${RV32_SSH_PERSIST:-600}
socket_dir=${RV32_SSH_SOCKET_DIR:-${XDG_RUNTIME_DIR:-/tmp/rv32-ssh-$(id -u)}}

mkdir -p "$socket_dir"
chmod 0700 "$socket_dir"
control_path="$socket_dir/control-%C"

ssh_options=(
  -o "ControlPath=$control_path"
  -o ControlMaster=auto
  -o "ControlPersist=$persist"
  -o Compression=no
  -o ServerAliveInterval=30
  -o ServerAliveCountMax=3
)

if [[ -n ${RV32_SSH_IDENTITY:-} ]]; then
  ssh_options+=(
    -i "$RV32_SSH_IDENTITY"
    -o IdentitiesOnly=yes
  )
fi

usage() {
  cat <<'EOF'
Usage:
  rv32-ssh.sh [REMOTE_COMMAND ...]
  rv32-ssh.sh --to LOCAL_PATH REMOTE_PATH
  rv32-ssh.sh --from REMOTE_PATH LOCAL_PATH
  rv32-ssh.sh --check
  rv32-ssh.sh --stop

The first connection performs SSH key exchange. Later commands and copies
reuse it for RV32_SSH_PERSIST seconds (default: 600).

Environment:
  RV32_SSH_HOST       guest address (default: 192.168.1.220)
  RV32_SSH_USER       guest user (default: root)
  RV32_SSH_IDENTITY   optional private-key path
  RV32_SSH_PERSIST    reuse lifetime in seconds
EOF
}

case ${1:-} in
  --to)
    [[ $# -eq 3 ]] || { usage >&2; exit 2; }
    recursive=()
    [[ -d $2 ]] && recursive=(-r)
    exec scp -O "${ssh_options[@]}" "${recursive[@]}" -- \
      "$2" "$user@$host:$3"
    ;;
  --from)
    [[ $# -eq 3 ]] || { usage >&2; exit 2; }
    exec scp -O "${ssh_options[@]}" -- "$user@$host:$2" "$3"
    ;;
  --check)
    exec ssh "${ssh_options[@]}" -O check "$user@$host"
    ;;
  --stop)
    exec ssh "${ssh_options[@]}" -O exit "$user@$host"
    ;;
  -h|--help)
    usage
    ;;
  *)
    # Do not start a no-command (-N) master: OpenSSH marks that connection as
    # no-more-sessions.  A normal shell/command becomes the master and remains
    # reusable through ControlPersist after the command exits.
    exec ssh "${ssh_options[@]}" "$user@$host" "$@"
    ;;
esac
