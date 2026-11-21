#!/usr/bin/env bash
# Scan serial ports for Revo2 hands (slave_id 126=left, 127=right).
#
# SDK-free: detection is done by setup/detect_revo2_ports.py, which speaks
# Modbus RTU directly over pyserial. No Stark SDK / C++ build required.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY_DETECTOR="${SCRIPT_DIR}/detect_revo2_ports.py"

if ! command -v python3 >/dev/null 2>&1; then
  echo "[ERROR] python3 not found; required by the Revo2 port detector." >&2
  exit 1
fi

if ! python3 -c 'import serial' >/dev/null 2>&1; then
  echo "[ERROR] python module 'serial' (pyserial) not installed." >&2
  echo "  Install: sudo apt install -y python3-serial   (or: python3 -m pip install pyserial)" >&2
  exit 1
fi

exec timeout 30 python3 "${PY_DETECTOR}" "$@"
