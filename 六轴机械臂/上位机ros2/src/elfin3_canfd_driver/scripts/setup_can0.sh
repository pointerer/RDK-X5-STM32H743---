#!/usr/bin/env bash
set -euo pipefail

interface="${1:-can0}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo bash $0 ${interface}" >&2
  exit 1
fi

if ! ip link show "${interface}" >/dev/null 2>&1; then
  echo "CAN interface not found: ${interface}" >&2
  exit 1
fi

ip link set dev "${interface}" down || true
ip link set dev "${interface}" type can \
  bitrate 500000 \
  dbitrate 2000000 \
  fd on \
  berr-reporting on \
  restart-ms 100
ip link set dev "${interface}" up

ip -details -statistics link show "${interface}"
