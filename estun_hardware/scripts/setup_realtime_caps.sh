#!/usr/bin/env bash
set -euo pipefail

CAPS_RT_ONLY="cap_sys_nice+ep"
CAPS_WITH_NET="cap_net_raw,cap_net_admin,cap_sys_nice+ep"
CAPS="${CAPS_RT_ONLY}"
MODE="both"
DRY_RUN=0
CHECK_ONLY=0
WITH_NET=0
RT_ONLY=0

usage() {
  cat <<'EOF'
Usage:
  setup_realtime_caps.sh [--both|--estun-only|--official-only] [--with-net] [--rt-only] [--check] [--dry-run]

Description:
  Standardize realtime capabilities for control nodes to reduce onsite ops complexity.
  Default applies CAP_SYS_NICE (minimum privilege for realtime scheduling).
  Use --with-net to additionally apply CAP_NET_RAW,CAP_NET_ADMIN for network fallback scenarios.

Options:
  --both           Apply capabilities to both estun_control_node and ros2_control_node (default)
  --estun-only     Apply capabilities only to estun_control_node
  --official-only  Apply capabilities only to ros2_control_node
  --with-net       Apply CAP_NET_RAW,CAP_NET_ADMIN,CAP_SYS_NICE
  --rt-only        Compatibility option, same behavior as default (CAP_SYS_NICE only)
  --check          Check current capabilities without modifying
  --dry-run        Print commands without executing
  -h, --help       Show this help message
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --both)
      MODE="both"
      shift
      ;;
    --estun-only)
      MODE="estun"
      shift
      ;;
    --official-only)
      MODE="official"
      shift
      ;;
    --with-net)
      WITH_NET=1
      shift
      ;;
    --check)
      CHECK_ONLY=1
      shift
      ;;
    --rt-only)
      RT_ONLY=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ $WITH_NET -eq 1 ]]; then
  CAPS="${CAPS_WITH_NET}"
fi

if [[ $RT_ONLY -eq 1 ]]; then
  CAPS="${CAPS_RT_ONLY}"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${PKG_ROOT}/../../.." && pwd)"

ESTUN_BIN="${WS_ROOT}/install/estun_hardware/lib/estun_hardware/estun_control_node"
OFFICIAL_BIN="/opt/ros/humble/lib/controller_manager/ros2_control_node"

resolve_real_bin() {
  local bin="$1"
  if [[ ! -e "$bin" ]]; then
    echo "$bin"
    return 0
  fi
  if [[ -L "$bin" ]]; then
    readlink -f "$bin"
  else
    echo "$bin"
  fi
}

print_caps() {
  local bin="$1"
  if [[ -e "$bin" ]]; then
    local real_bin
    real_bin="$(resolve_real_bin "$bin")"
    if [[ "$real_bin" != "$bin" ]]; then
      echo "$bin -> $real_bin"
    fi
    local cap_line
    cap_line="$(getcap "$real_bin" || true)"
    if [[ -n "$cap_line" ]]; then
      echo "$cap_line"
    else
      echo "$real_bin (no file capabilities set)"
    fi
  else
    echo "[WARN] Missing binary: $bin"
  fi
}

apply_caps() {
  local bin="$1"
  if [[ ! -e "$bin" ]]; then
    echo "[WARN] Skip missing binary: $bin"
    return 0
  fi
  local real_bin
  real_bin="$(resolve_real_bin "$bin")"
  if [[ ! -f "$real_bin" ]]; then
    echo "[WARN] Skip non-regular target: $real_bin"
    return 0
  fi
  local cmd=(sudo setcap "$CAPS" "$real_bin")
  if [[ $DRY_RUN -eq 1 ]]; then
    echo "[DRY-RUN] ${cmd[*]}"
    return 0
  fi
  if [[ "$real_bin" != "$bin" ]]; then
    echo "[INFO] Applying capabilities to symlink target: $bin -> $real_bin"
  else
    echo "[INFO] Applying capabilities to: $real_bin"
  fi
  "${cmd[@]}"
}

echo "[INFO] estun_control_node: $ESTUN_BIN"
echo "[INFO] ros2_control_node: $OFFICIAL_BIN"
echo "[INFO] capabilities to apply: $CAPS"

if [[ $CHECK_ONLY -eq 1 ]]; then
  echo "[INFO] Checking current capabilities..."
  print_caps "$ESTUN_BIN"
  print_caps "$OFFICIAL_BIN"
  exit 0
fi

case "$MODE" in
  both)
    apply_caps "$ESTUN_BIN"
    apply_caps "$OFFICIAL_BIN"
    ;;
  estun)
    apply_caps "$ESTUN_BIN"
    ;;
  official)
    apply_caps "$OFFICIAL_BIN"
    ;;
esac

echo "[INFO] Final capability state:"
print_caps "$ESTUN_BIN"
print_caps "$OFFICIAL_BIN"
