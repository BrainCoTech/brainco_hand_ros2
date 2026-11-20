#!/usr/bin/env bash
# Shared left/right device mapping parsers (l|r|left|right).
# Source from revo2 udev/bootstrap scripts — do not execute directly.

normalize_side() {
  case "${1,,}" in
    l|left) echo "left" ;;
    r|right) echo "right" ;;
    *) return 1 ;;
  esac
}

is_side_token() {
  normalize_side "$1" >/dev/null 2>&1
}

# Sets ROLE_MAP_LEFT_DEV / ROLE_MAP_RIGHT_DEV (may be empty for single-side).
role_map_resolve_tokens() {
  local -n _tokens=$1
  ROLE_MAP_LEFT_DEV=""
  ROLE_MAP_RIGHT_DEV=""

  if [[ "${#_tokens[@]}" -eq 4 ]]; then
    local side_a side_b
    side_a="$(normalize_side "${_tokens[1]}")" || {
      echo "[ERROR] role must be l|left or r|right: ${_tokens[1]}" >&2
      return 1
    }
    side_b="$(normalize_side "${_tokens[3]}")" || {
      echo "[ERROR] role must be l|left or r|right: ${_tokens[3]}" >&2
      return 1
    }
    if [[ "${side_a}" == "${side_b}" ]]; then
      echo "[ERROR] mapping must include one left (l) and one right (r)." >&2
      return 1
    fi
    if [[ "${side_a}" == "left" ]]; then
      ROLE_MAP_LEFT_DEV="${_tokens[0]}"
      ROLE_MAP_RIGHT_DEV="${_tokens[2]}"
    else
      ROLE_MAP_LEFT_DEV="${_tokens[2]}"
      ROLE_MAP_RIGHT_DEV="${_tokens[0]}"
    fi
    return 0
  fi

  if [[ "${#_tokens[@]}" -eq 2 ]]; then
    if is_side_token "${_tokens[1]}"; then
      local side
      side="$(normalize_side "${_tokens[1]}")"
      if [[ "${side}" == "left" ]]; then
        ROLE_MAP_LEFT_DEV="${_tokens[0]}"
      else
        ROLE_MAP_RIGHT_DEV="${_tokens[0]}"
      fi
      return 0
    fi
    ROLE_MAP_LEFT_DEV="${_tokens[0]}"
    ROLE_MAP_RIGHT_DEV="${_tokens[1]}"
    return 0
  fi

  echo "[ERROR] expected 4 tokens (<dev> <l|r> <dev> <l|r>) or 2 tokens." >&2
  return 1
}

role_map_usage_lines() {
  cat <<'EOF'
  bash SCRIPT.sh /dev/ttyACM6 l /dev/ttyACM1 r
  bash SCRIPT.sh /dev/ttyACM6 /dev/ttyACM1
  bash SCRIPT.sh /dev/ttyACM6 left
EOF
}
