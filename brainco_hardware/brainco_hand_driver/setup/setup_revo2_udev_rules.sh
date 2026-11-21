#!/usr/bin/env bash
set -euo pipefail

# Auto-generate stable udev rules for Revo2 Modbus serial (ttyACM/ttyUSB).
#
# Examples:
#   sudo bash setup_revo2_udev_rules.sh /dev/ttyACM6 l /dev/ttyACM1 r
#   sudo bash setup_revo2_udev_rules.sh /dev/ttyACM6 /dev/ttyACM1
#   sudo bash setup_revo2_udev_rules.sh /dev/ttyACM6 left
#
# Result symlinks:
#   /dev/revo2_hand_left
#   /dev/revo2_hand_right

RULE_FILE="/etc/udev/rules.d/99-revo2-hands.rules"
LEFT_NAME="revo2_hand_left"
RIGHT_NAME="revo2_hand_right"
PATH_MODE="hub-relative"
RELATIVE_SEGMENTS=2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=role_mapping_utils.sh
source "${SCRIPT_DIR}/role_mapping_utils.sh"

usage() {
  cat <<EOF
Usage:
  sudo bash $0 [--path-mode exact|hub-relative] [--relative-segments N] <dev> <l|r> <dev> <l|r>
  sudo bash $0 [--path-mode exact|hub-relative] [--relative-segments N] <left_dev> <right_dev>
  sudo bash $0 [--path-mode exact|hub-relative] [--relative-segments N] <dev> <left|right>

Examples:
  sudo bash $0 /dev/ttyACM6 l /dev/ttyACM1 r
  sudo bash $0 /dev/ttyACM6 /dev/ttyACM1
EOF
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "[ERROR] missing command: $1"
    exit 1
  }
}

resolve_serial_dev() {
  local input_dev="$1"
  local resolved
  resolved="$(readlink -f "${input_dev}" || true)"
  if [[ -z "${resolved}" || ! -e "${resolved}" ]]; then
    echo "[ERROR] cannot resolve device: ${input_dev}" >&2
    exit 1
  fi
  if [[ "${resolved}" != /dev/tty* ]]; then
    echo "[ERROR] not a serial device: ${input_dev} -> ${resolved}" >&2
    exit 1
  fi
  echo "${resolved}"
}

get_udev_prop() {
  local dev="$1"
  local key="$2"
  udevadm info --query=property --name "${dev}" | awk -F= -v k="${key}" '$1==k {print $2; exit}'
}

last_dot_segments() {
  local value="$1"
  local n="$2"
  local count i start out
  local -a parts
  IFS='.' read -r -a parts <<< "${value}"
  count="${#parts[@]}"

  if (( count <= n )); then
    echo "${value}"
    return
  fi

  start=$((count - n))
  out="${parts[$start]}"
  for ((i = start + 1; i < count; i++)); do
    out="${out}.${parts[$i]}"
  done
  echo "${out}"
}

build_id_path_match() {
  local id_path="$1"

  if [[ "${PATH_MODE}" == "exact" ]]; then
    echo "${id_path}"
    return
  fi

  if [[ "${id_path}" != *"-usb-"* ]]; then
    echo "${id_path}"
    return
  fi

  local usb_tail hops_and_iface hops iface suffix
  usb_tail="${id_path##*-usb-}"
  hops_and_iface="${usb_tail#*:}"
  hops="${hops_and_iface%%:*}"
  iface="${hops_and_iface#*:}"

  if [[ -z "${hops}" || -z "${iface}" || "${hops}" == "${hops_and_iface}" ]]; then
    echo "${id_path}"
    return
  fi

  if [[ "${hops}" != *.* ]]; then
    echo "${id_path}"
    return
  fi

  suffix="$(last_dot_segments "${hops}" "${RELATIVE_SEGMENTS}")"
  if [[ "${suffix}" == "${hops}" ]]; then
    echo "${id_path}"
    return
  fi

  echo "*-usb-*:*.${suffix}:${iface}"
}

get_device_match_key() {
  local dev="$1"
  local id_path id_serial iface_num

  id_path="$(get_udev_prop "${dev}" "ID_PATH")"
  if [[ -n "${id_path}" ]]; then
    build_id_path_match "${id_path}"
    return
  fi

  # Fallback for drivers that don't set ID_PATH (e.g. usb_ch343)
  id_serial="$(get_udev_prop "${dev}" "ID_SERIAL_SHORT")"
  iface_num="$(get_udev_prop "${dev}" "ID_USB_INTERFACE_NUM")"
  if [[ -n "${id_serial}" && -n "${iface_num}" ]]; then
    echo "${id_serial}:${iface_num}"
    return
  fi

  echo "[ERROR] failed to read device match key for ${dev}" >&2
  exit 1
}

build_rule_line() {
  local dev="$1"
  local link_name="$2"
  local vendor_id model_id id_path id_path_match id_serial iface_num match_attr

  vendor_id="$(get_udev_prop "${dev}" "ID_VENDOR_ID")"
  model_id="$(get_udev_prop "${dev}" "ID_MODEL_ID")"
  id_path="$(get_udev_prop "${dev}" "ID_PATH")"
  id_serial="$(get_udev_prop "${dev}" "ID_SERIAL_SHORT")"
  iface_num="$(get_udev_prop "${dev}" "ID_USB_INTERFACE_NUM")"

  if [[ -z "${vendor_id}" || -z "${model_id}" ]]; then
    echo "[ERROR] failed to read udev properties for ${dev}" >&2
    echo "        ID_VENDOR_ID='${vendor_id}', ID_MODEL_ID='${model_id}'" >&2
    exit 1
  fi

  if [[ -n "${id_path}" ]]; then
    # Standard: use ID_PATH (or hub-relative for removable hubs)
    id_path_match="$(build_id_path_match "${id_path}")"
    match_attr="ENV{ID_PATH}==\"${id_path_match}\""
  elif [[ -n "${id_serial}" && -n "${iface_num}" ]]; then
    # Fallback: usb_ch343 driver doesn't set ID_PATH, use ID_SERIAL_SHORT + interface
    match_attr="ENV{ID_SERIAL_SHORT}==\"${id_serial}\", ENV{ID_USB_INTERFACE_NUM}==\"${iface_num}\""
  else
    echo "[ERROR] failed to read udev properties for ${dev}" >&2
    echo "        ID_VENDOR_ID='${vendor_id}', ID_MODEL_ID='${model_id}', ID_PATH='${id_path}'" >&2
    echo "        ID_SERIAL_SHORT='${id_serial}', ID_USB_INTERFACE_NUM='${iface_num}'" >&2
    exit 1
  fi

  echo "SUBSYSTEM==\"tty\", ENV{ID_VENDOR_ID}==\"${vendor_id}\", ENV{ID_MODEL_ID}==\"${model_id}\", ${match_attr}, MODE=\"0666\", SYMLINK+=\"${link_name}\""
}

ensure_distinct_bimanual_rules() {
  local left_match="" right_match=""

  [[ -n "${left_dev}" && -n "${right_dev}" ]] || return 0

  left_match="$(get_device_match_key "${left_dev}")"
  right_match="$(get_device_match_key "${right_dev}")"
  if [[ "${left_match}" != "${right_match}" ]]; then
    return 0
  fi

  if [[ "${PATH_MODE}" == "hub-relative" ]]; then
    echo "[WARN] hub-relative ID_PATH collides (${left_match}); switching to exact mode." >&2
    PATH_MODE="exact"
    left_rule="$(build_rule_line "${left_dev}" "${LEFT_NAME}")"
    right_rule="$(build_rule_line "${right_dev}" "${RIGHT_NAME}")"
    left_match="$(get_device_match_key "${left_dev}")"
    right_match="$(get_device_match_key "${right_dev}")"
  fi

  if [[ "${left_match}" == "${right_match}" ]]; then
    echo "[ERROR] cannot generate distinct udev rules for ${left_dev} and ${right_dev}." >&2
    echo "        Both match '${left_match}'." >&2
    exit 1
  fi
}

cleanup_link() {
  local link_name="$1"
  local link_path="/dev/${link_name}"
  if [[ -L "${link_path}" || -e "${link_path}" ]]; then
    rm -f "${link_path}"
  fi
}

main() {
  local args=()

  while [[ "$#" -gt 0 ]]; do
    case "$1" in
      --path-mode)
        PATH_MODE="$2"
        shift 2
        ;;
      --relative-segments)
        RELATIVE_SEGMENTS="$2"
        shift 2
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      --)
        shift
        while [[ "$#" -gt 0 ]]; do
          args+=("$1")
          shift
        done
        ;;
      -*)
        echo "[ERROR] unknown option: $1" >&2
        usage
        exit 1
        ;;
      *)
        args+=("$1")
        shift
        ;;
    esac
  done

  if [[ "${EUID}" -ne 0 ]]; then
    echo "[ERROR] run as root (use sudo)." >&2
    exit 1
  fi

  if [[ "${#args[@]}" -ne 2 && "${#args[@]}" -ne 4 ]]; then
    usage
    role_map_usage_lines
    exit 1
  fi

  require_cmd udevadm
  require_cmd readlink
  require_cmd awk

  role_map_resolve_tokens args || exit 1

  local left_dev="" right_dev="" left_rule="" right_rule=""
  if [[ -n "${ROLE_MAP_LEFT_DEV}" ]]; then
    left_dev="$(resolve_serial_dev "${ROLE_MAP_LEFT_DEV}")"
    left_rule="$(build_rule_line "${left_dev}" "${LEFT_NAME}")"
  fi
  if [[ -n "${ROLE_MAP_RIGHT_DEV}" ]]; then
    right_dev="$(resolve_serial_dev "${ROLE_MAP_RIGHT_DEV}")"
    right_rule="$(build_rule_line "${right_dev}" "${RIGHT_NAME}")"
  fi

  ensure_distinct_bimanual_rules

  cat > "${RULE_FILE}" <<EOF
# Auto-generated by setup_revo2_udev_rules.sh
# Stable Modbus serial symlinks for Revo2 bimanual teleop.
EOF
  if [[ -n "${left_rule}" ]]; then
    echo "${left_rule}" >> "${RULE_FILE}"
  fi
  if [[ -n "${right_rule}" ]]; then
    echo "${right_rule}" >> "${RULE_FILE}"
  fi

  if [[ -z "${left_rule}" ]]; then
    cleanup_link "${LEFT_NAME}"
  fi
  if [[ -z "${right_rule}" ]]; then
    cleanup_link "${RIGHT_NAME}"
  fi

  udevadm control --reload-rules
  udevadm trigger --subsystem-match=tty

  echo "[OK] udev rules installed: ${RULE_FILE}"
  if [[ -n "${left_dev}" ]]; then
    echo "     left : /dev/${LEFT_NAME} <- ${left_dev}"
  else
    echo "     left : not configured"
  fi
  if [[ -n "${right_dev}" ]]; then
    echo "     right: /dev/${RIGHT_NAME} <- ${right_dev}"
  else
    echo "     right: not configured"
  fi
  echo "     path mode: ${PATH_MODE} (relative segments: ${RELATIVE_SEGMENTS})"
  echo "Reconnect USB or reboot if symlinks are not visible yet."
}

main "$@"
