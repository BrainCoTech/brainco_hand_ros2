#!/usr/bin/env bash
set -euo pipefail

LEFT_LINK="/dev/revo2_hand_left"
RIGHT_LINK="/dev/revo2_hand_right"
RULE_FILE="/etc/udev/rules.d/99-revo2-hands.rules"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

pass() { echo "[PASS] $*"; }
fail() { echo "[FAIL] $*"; FAILED=1; }
info() { echo "[INFO] $*"; }

check_link() {
  local link="$1"
  if [[ -e "${link}" ]]; then
    local real
    real="$(readlink -f "${link}" || true)"
    pass "${link} -> ${real}"
  else
    fail "${link} does not exist"
  fi
}

check_rule_file() {
  if [[ -f "${RULE_FILE}" ]]; then
    pass "udev rule exists: ${RULE_FILE}"
  else
    fail "udev rule missing: ${RULE_FILE}"
    info "Run: sudo bash ${SCRIPT_DIR}/setup_revo2_udev_rules.sh <left> l <right> r"
  fi
}

rule_has_link() {
  local link_name="$1"
  [[ -f "${RULE_FILE}" ]] && grep -Fq "SYMLINK+=\"${link_name}\"" "${RULE_FILE}"
}

main() {
  FAILED=0
  echo "=== Revo2 serial setup check ==="
  check_rule_file

  if rule_has_link "revo2_hand_left"; then
    pass "udev maps revo2_hand_left"
  elif [[ -f "${RULE_FILE}" ]]; then
    fail "udev rule has no SYMLINK+=\"revo2_hand_left\""
  fi

  if rule_has_link "revo2_hand_right"; then
    pass "udev maps revo2_hand_right"
  elif [[ -f "${RULE_FILE}" ]]; then
    fail "udev rule has no SYMLINK+=\"revo2_hand_right\""
  fi

  check_link "${LEFT_LINK}"
  check_link "${RIGHT_LINK}"

  if [[ -e "${LEFT_LINK}" && -e "${RIGHT_LINK}" ]]; then
    local left_real right_real
    left_real="$(readlink -f "${LEFT_LINK}" || true)"
    right_real="$(readlink -f "${RIGHT_LINK}" || true)"
    if [[ -n "${left_real}" && "${left_real}" == "${right_real}" ]]; then
      fail "revo2_hand_left and revo2_hand_right both resolve to ${left_real}"
      info "Re-run: sudo bash ${SCRIPT_DIR}/setup_revo2_udev_rules.sh --path-mode exact <left> l <right> r"
    else
      pass "left/right symlinks point to distinct serial devices"
    fi
  fi

  if [[ "${FAILED}" -eq 0 ]]; then
    echo "[OK] Revo2 serial setup looks good."
    exit 0
  fi
  echo "[WARN] Fix issues above, then re-run bootstrap_revo2.sh"
  exit 1
}

main "$@"
