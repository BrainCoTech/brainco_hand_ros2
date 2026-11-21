#!/usr/bin/env bash
set -euo pipefail

# List Revo2 Modbus serial candidates (ttyACM / ttyUSB) with USB topology hints.

ROLES_FILE="${REVO2_USB_ROLES_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/revoarm/revo2_usb_roles.conf}"

list_serial_devs() {
  local d
  shopt -s nullglob
  # Scan all hardware serial devices: ttyUSB*, ttyACM*, ttyCH343USB*, ttyCH34*, ttyS*, etc.
  # Excludes virtual consoles (tty0-tty63) by requiring at least one uppercase letter after "tty".
  for d in /dev/tty[A-Z]*; do
    [[ -c "$d" ]] && echo "$d"
  done
  shopt -u nullglob
}

usb_key_for_dev() {
  local dev="$1"
  local udev_path
  udev_path="$(udevadm info -q path -n "${dev}" 2>/dev/null || true)"
  if [[ -z "${udev_path}" ]]; then
    echo "?"
    return
  fi
  # e.g. .../3-6.2.4.4:1.0/ttyACM0
  if [[ "${udev_path}" == */tty/* ]]; then
    basename "${udev_path%/tty/*}" | sed 's/:.*//'
  else
    basename "${udev_path}" | sed 's/:.*//'
  fi
}

echo "=== Revo2 serial discovery (Modbus) ==="
echo ""

mapfile -t DEVS < <(list_serial_devs | sort -V)
if [[ ${#DEVS[@]} -eq 0 ]]; then
  echo "No /dev/ttyACM*, /dev/ttyUSB*, or /dev/ttyCH343USB* found. Plug in Revo2 USB cables." >&2
  exit 1
fi

declare -A KEY_TO_LIST
for dev in "${DEVS[@]}"; do
  key="$(usb_key_for_dev "${dev}")"
  KEY_TO_LIST["$key"]+="${dev} "
done

printf '%-16s  %-18s  %-12s  %s\n' "DEVICE" "USB_KEY" "VENDOR:MODEL" "ID_PATH"
echo "--------------------------------------------------------------------------------"
for dev in "${DEVS[@]}"; do
  key="$(usb_key_for_dev "${dev}")"
  vid="$(udevadm info -q property -n "${dev}" 2>/dev/null | sed -n 's/^ID_VENDOR_ID=//p' | head -n1)"
  pid="$(udevadm info -q property -n "${dev}" 2>/dev/null | sed -n 's/^ID_MODEL_ID=//p' | head -n1)"
  id_path="$(udevadm info -q property -n "${dev}" 2>/dev/null | sed -n 's/^ID_PATH=//p' | head -c72)"
  printf '%-16s  %-18s  %-12s  %s\n' "${dev}" "${key}" "${vid}:${pid}" "${id_path:-?}"
done

echo ""
echo "Grouped by USB port:"
for key in $(printf '%s\n' "${!KEY_TO_LIST[@]}" | sort); do
  # shellcheck disable=SC2086
  echo "  ${key}: $(echo "${KEY_TO_LIST[$key]}" | tr ' ' '\n' | grep -v '^$' | sort -V | xargs)"
done

echo ""
if [[ -f "${ROLES_FILE}" ]]; then
  echo "Roles file: ${ROLES_FILE} (use substrings from USB_KEY above)"
  echo "  bootstrap: bash $(dirname "$0")/bootstrap_revo2.sh"
else
  echo "One-shot stable naming:"
  echo "  bash $(dirname "$0")/bootstrap_revo2.sh              # default: auto"
  echo "  bash $(dirname "$0")/bootstrap_revo2.sh --manual"
  echo "  bash $(dirname "$0")/bootstrap_revo2.sh /dev/ttyACM6 l /dev/ttyACM1 r"
  echo ""
  echo "Tip: unplug ONE hand — the device that disappears is on that hand's USB port."
fi

echo ""
echo "After udev: /dev/revo2_hand_left  /dev/revo2_hand_right"
echo "Done."
