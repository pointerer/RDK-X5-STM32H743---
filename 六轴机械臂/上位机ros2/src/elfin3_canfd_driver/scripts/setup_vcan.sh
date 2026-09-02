#!/usr/bin/env bash
set -euo pipefail

interface="${1:-vcan0}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo bash $0 ${interface}" >&2
  exit 1
fi

modprobe vcan
if ! ip link show "${interface}" >/dev/null 2>&1; then
  ip link add dev "${interface}" type vcan
fi
ip link set dev "${interface}" up
ip -details link show "${interface}"
