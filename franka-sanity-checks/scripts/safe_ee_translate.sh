#!/usr/bin/env bash
set -euo pipefail

BIN="./build/panda_libfranka_sanity"
ROBOT_IP=""
AXIS="z"
DELTA_M="0.015"
DURATION_S="8.0"
READ_SAMPLES="100"
AUTO_RECOVER=1
RETURN_TO_START=1
ASSUME_YES=0

usage() {
  cat <<'EOF'
Safety-first tiny EE translation runner.

Usage:
  scripts/safe_ee_translate.sh --robot-ip <IP> [options]

Options:
  --axis x|y|z              Translation axis in base frame (default: z)
  --delta-m <meters>        Signed translation distance (default: 0.015)
  --duration-s <seconds>    Motion duration, must be >= 8.0 (default: 8.0)
  --read-samples <N>        Samples for pre-check read-only mode (default: 100)
  --no-auto-recover         Do not pass --auto-recover to motion command
  --no-return               Do not run reverse move back to start
  --yes                     Skip interactive safety confirmation
  -h, --help                Show this help

Safety limits enforced:
  0 < |delta-m| <= 0.05
  duration-s >= 8.0
  |delta-m| / duration-s <= 0.01 m/s
EOF
}

is_number() {
  awk -v v="$1" 'BEGIN { exit !(v+0==v) }'
}

abs_gt() {
  awk -v v="$1" -v lim="$2" 'BEGIN { if (v < 0) v = -v; exit !(v > lim) }'
}

abs_ge() {
  awk -v v="$1" -v lim="$2" 'BEGIN { if (v < 0) v = -v; exit !(v >= lim) }'
}

lt() {
  awk -v v="$1" -v lim="$2" 'BEGIN { exit !(v < lim) }'
}

negate() {
  awk -v v="$1" 'BEGIN { printf "%.6f", -v }'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-ip)
      [[ $# -ge 2 ]] || { echo "Missing value for --robot-ip"; exit 1; }
      ROBOT_IP="$2"
      shift 2
      ;;
    --axis)
      [[ $# -ge 2 ]] || { echo "Missing value for --axis"; exit 1; }
      AXIS="$2"
      shift 2
      ;;
    --delta-m)
      [[ $# -ge 2 ]] || { echo "Missing value for --delta-m"; exit 1; }
      DELTA_M="$2"
      shift 2
      ;;
    --duration-s)
      [[ $# -ge 2 ]] || { echo "Missing value for --duration-s"; exit 1; }
      DURATION_S="$2"
      shift 2
      ;;
    --read-samples)
      [[ $# -ge 2 ]] || { echo "Missing value for --read-samples"; exit 1; }
      READ_SAMPLES="$2"
      shift 2
      ;;
    --no-auto-recover)
      AUTO_RECOVER=0
      shift
      ;;
    --no-return)
      RETURN_TO_START=0
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

if [[ "$AXIS" != "x" && "$AXIS" != "y" && "$AXIS" != "z" ]]; then
  echo "Error: --axis must be x, y, or z."
  exit 1
fi

is_number "$DELTA_M" || { echo "Error: --delta-m must be numeric."; exit 1; }
is_number "$DURATION_S" || { echo "Error: --duration-s must be numeric."; exit 1; }
is_number "$READ_SAMPLES" || { echo "Error: --read-samples must be numeric."; exit 1; }

abs_ge "$DELTA_M" 0.001 || { echo "Error: |delta-m| must be at least 0.001."; exit 1; }
abs_gt "$DELTA_M" 0.05 && { echo "Error: |delta-m| safety limit is 0.05 m."; exit 1; }
lt "$DURATION_S" 8.0 && { echo "Error: duration-s safety minimum is 8.0 s."; exit 1; }
awk -v d="$DELTA_M" -v t="$DURATION_S" 'BEGIN { if (d < 0) d = -d; exit !((d / t) > 0.01) }' \
  && { echo "Error: |delta-m|/duration-s must be <= 0.01 m/s."; exit 1; }

if [[ ! -x "$BIN" ]]; then
  echo "Error: $BIN not found or not executable. Build first:"
  echo "  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release"
  echo "  cmake --build build -j\"$(nproc)\""
  exit 1
fi

if command -v getcap >/dev/null 2>&1; then
  if ! getcap "$BIN" | grep -q "cap_sys_nice"; then
    echo "Warning: realtime capability not found on $BIN."
    echo "Run: sudo setcap cap_sys_nice,cap_ipc_lock+ep $BIN"
  fi
fi

if [[ "$ASSUME_YES" -ne 1 ]]; then
  echo "Safety checklist:"
  echo "  1) Workspace is clear."
  echo "  2) E-stop is reachable."
  echo "  3) You are ready to abort immediately if motion is unexpected."
  read -r -p "Type YES to continue: " confirm
  [[ "$confirm" == "YES" ]] || { echo "Aborted by user."; exit 1; }
fi

echo "[1/4] Read-only pre-check..."
"$BIN" --robot-ip "$ROBOT_IP" --mode read-only --read-samples "$READ_SAMPLES"

echo "[2/4] Recover-only pre-check..."
"$BIN" --robot-ip "$ROBOT_IP" --mode recover-only

echo "[3/4] Tiny Cartesian translation..."
motion_cmd=("$BIN" --robot-ip "$ROBOT_IP" --mode tiny-cartesian --cart-axis "$AXIS" --delta-m "$DELTA_M" --duration-s "$DURATION_S")
if [[ "$AUTO_RECOVER" -eq 1 ]]; then
  motion_cmd+=(--auto-recover)
fi
"${motion_cmd[@]}"

if [[ "$RETURN_TO_START" -eq 1 ]]; then
  echo "[4/4] Return motion..."
  back_delta="$(negate "$DELTA_M")"
  back_cmd=("$BIN" --robot-ip "$ROBOT_IP" --mode tiny-cartesian --cart-axis "$AXIS" --delta-m "$back_delta" --duration-s "$DURATION_S")
  if [[ "$AUTO_RECOVER" -eq 1 ]]; then
    back_cmd+=(--auto-recover)
  fi
  "${back_cmd[@]}"
else
  echo "[4/4] Skipped return motion (--no-return)."
fi

echo "Done."
