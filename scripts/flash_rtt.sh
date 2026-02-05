#!/bin/sh
set -eu

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT_DIR/build}"
JLINK_DEVICE="${JLINK_DEVICE:-NRF52833_XXAA}"
JLINK_IF="${JLINK_IF:-SWD}"
JLINK_SPEED="${JLINK_SPEED:-4000}"
RTT_CHANNEL="${RTT_CHANNEL:-0}"
LOG_DIR="${JLINK_LOG_DIR:-$HOME/Library/Application Support/SEGGER}"

need_build_dir=1
for arg in "$@"; do
  case "$arg" in
    -d|--build-dir)
      need_build_dir=0
      break
      ;;
  esac
done

cd "$ROOT_DIR"
if [ "$need_build_dir" -eq 1 ]; then
  west flash -r jlink -d "$BUILD_DIR" "$@"
else
  west flash -r jlink "$@"
fi

echo "启动 JLinkRTTLogger (Device=$JLINK_DEVICE, IF=$JLINK_IF, Speed=$JLINK_SPEED, Channel=$RTT_CHANNEL)"
JLinkRTTLogger -Device "$JLINK_DEVICE" -If "$JLINK_IF" -Speed "$JLINK_SPEED" -RTTChannel "$RTT_CHANNEL" &
LOGGER_PID=$!

cleanup() {
  if [ -n "${LOGGER_PID:-}" ]; then
    kill "$LOGGER_PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

start_ts="$(date +%s)"
log_file=""
try=0
while [ "$try" -lt 50 ]; do
  for dir in "$LOG_DIR" "$ROOT_DIR"; do
    [ -d "$dir" ] || continue
    for f in "$dir"/RTT*.log "$dir"/RTTLogger_*.log; do
      [ -e "$f" ] || continue
      mtime="$(stat -f %m "$f" 2>/dev/null || echo 0)"
      if [ "$mtime" -ge "$start_ts" ]; then
        log_file="$f"
        break 2
      fi
    done
  done
  try=$((try + 1))
  sleep 0.2
done

if [ -z "$log_file" ]; then
  echo "未找到新的 RTT 日志文件，继续等待输出..."
  wait "$LOGGER_PID"
  exit 1
fi

echo "RTT 日志文件: $log_file"
tail -f "$log_file"
