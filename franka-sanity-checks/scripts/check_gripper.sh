#!/usr/bin/env bash
set -euo pipefail

BIN=""
ROBOT_IP=""
GRIPPER_SPEED="0.03"
PAUSE_S="1.0"
SKIP_HOMING=0
ASSUME_YES=0

usage() {
  cat <<'EOF'
Safety-first Franka gripper check.

Usage:
  scripts/check_gripper.sh --robot-ip <IP> [options]

Options:
  --gripper-speed <m/s>    Gripper move speed, 0 < speed <= 0.2 (default: 0.03)
  --pause-s <seconds>      Pause after each move before reading state (default: 1.0)
  --skip-homing            Skip gripper homing before the move cycle
  --yes                    Skip interactive safety confirmation
  -h, --help               Show this help

Sequence:
  1) Connect to the gripper
  2) Read and print gripper state
  3) Home the gripper unless --skip-homing is set
  4) Open fully
  5) Close fully
  6) Reopen fully
EOF
}

resolve_bin() {
  local candidates=(
    "./build/panda_libfranka_sanity"
    "./build_gripper_check/panda_libfranka_sanity"
  )
  for candidate in "${candidates[@]}"; do
    if [[ -x "$candidate" ]]; then
      BIN="$candidate"
      return 0
    fi
  done
  return 1
}

is_number() {
  awk -v v="$1" 'BEGIN { exit !(v+0==v) }'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-ip)
      [[ $# -ge 2 ]] || { echo "Missing value for --robot-ip"; exit 1; }
      ROBOT_IP="$2"
      shift 2
      ;;
    --gripper-speed)
      [[ $# -ge 2 ]] || { echo "Missing value for --gripper-speed"; exit 1; }
      GRIPPER_SPEED="$2"
      shift 2
      ;;
    --pause-s)
      [[ $# -ge 2 ]] || { echo "Missing value for --pause-s"; exit 1; }
      PAUSE_S="$2"
      shift 2
      ;;
    --skip-homing)
      SKIP_HOMING=1
      shift
      ;;
    --yes)
      ASSUME_YES=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ -z "$ROBOT_IP" ]]; then
  echo "Error: --robot-ip is required."
  usage
  exit 1
fi

is_number "$GRIPPER_SPEED" || { echo "Error: --gripper-speed must be numeric."; exit 1; }
is_number "$PAUSE_S" || { echo "Error: --pause-s must be numeric."; exit 1; }
awk -v v="$GRIPPER_SPEED" 'BEGIN { exit !(v > 0.0 && v <= 0.2) }' \
  || { echo "Error: --gripper-speed must satisfy 0 < speed <= 0.2."; exit 1; }
awk -v v="$PAUSE_S" 'BEGIN { exit !(v >= 0.0 && v <= 10.0) }' \
  || { echo "Error: --pause-s must satisfy 0 <= pause <= 10."; exit 1; }

if ! resolve_bin; then
  echo "Error: panda_libfranka_sanity not found or not executable. Build first:"
  echo "  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release"
  echo "  cmake --build build -j\"$(nproc)\""
  exit 1
fi

if [[ "$ASSUME_YES" -ne 1 ]]; then
  echo "Safety checklist:"
  echo "  1) End effector area is clear."
  echo "  2) Nothing is between the gripper fingers."
  echo "  3) Franka Desk is reachable in case the check faults."
  read -r -p "Type YES to continue: " confirm
  [[ "$confirm" == "YES" ]] || { echo "Aborted by user."; exit 1; }
fi

cmd=(
  "$BIN"
  --robot-ip "$ROBOT_IP"
  --mode gripper-check
  --gripper-speed "$GRIPPER_SPEED"
  --gripper-pause-s "$PAUSE_S"
)

if [[ "$SKIP_HOMING" -eq 1 ]]; then
  cmd+=(--skip-gripper-homing)
fi

exec "${cmd[@]}"
