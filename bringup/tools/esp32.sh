#!/usr/bin/env bash
# Reusable ESP32 (Arduino) build/flash/monitor/dashboard helper.
# Usage (from a project folder containing esp32.conf):
#   esp32.sh setup | ports | compile | upload | reset | monitor [secs] | serve [http_port] | all
set -euo pipefail
CONF="${ESP32_CONF:-$PWD/esp32.conf}"
[[ -f "$CONF" ]] || { echo "No esp32.conf in $PWD (see esp32.conf.example in the skill folder)"; exit 1; }
# shellcheck disable=SC1090
source "$CONF"
: "${FQBN:?FQBN missing in esp32.conf}" "${SKETCH:?SKETCH missing}"
BAUD="${BAUD:-115200}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$(dirname "$CONF")" && pwd)"

find_port() {
  if [[ -n "${PORT:-}" && -e "$PORT" ]]; then echo "$PORT"; return; fi
  local p; p=$(ls /dev/cu.usbmodem* /dev/cu.usbserial* /dev/cu.wchusbserial* /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -1 || true)
  [[ -n "$p" ]] || { echo "No ESP32 serial port found. Plug the board in (or hold BOOT while plugging in)." >&2; return 1; }
  echo "$p"
}

case "${1:-}" in
  setup)
    command -v arduino-cli >/dev/null || brew install arduino-cli
    arduino-cli config init --overwrite >/dev/null 2>&1 || true
    arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json || true
    arduino-cli core update-index
    arduino-cli core install esp32:esp32
    python3 -c "import serial" 2>/dev/null || python3 -m pip install pyserial
    python3 -c "import esptool" 2>/dev/null || python3 -m pip install esptool
    ;;
  ports) arduino-cli board list ;;
  compile) arduino-cli compile --fqbn "$FQBN" "$PROJECT_DIR/$SKETCH" ;;
  upload)
    P=$(find_port); echo "Uploading to $P"
    arduino-cli compile --fqbn "$FQBN" "$PROJECT_DIR/$SKETCH"
    arduino-cli upload --fqbn "$FQBN" -p "$P" "$PROJECT_DIR/$SKETCH"
    sleep 2; "$0" reset
    ;;
  reset)
    # Watchdog reset re-samples the strapping pins and reliably leaves download mode on ESP32-S3 native USB
    # (a plain RTS reset can drop back into the bootloader).
    P=$(find_port); python3 -m esptool --port "$P" --after watchdog-reset chip-id >/dev/null 2>&1 || true
    sleep 3
    ;;
  monitor)
    P=$(find_port); python3 "$HERE/serial_tail.py" "$P" "$BAUD" "${2:-10}" ;;
  serve)
    P=$(find_port); python3 "$HERE/bridge.py" --port "$P" --baud "$BAUD" --html "$PROJECT_DIR/${DASHBOARD:?DASHBOARD missing in esp32.conf}" --http "${2:-8000}" --logdir "$PROJECT_DIR/logs" ;;
  all) "$0" upload && sleep 3 && "$0" monitor 5 ;;
  *) sed -n '2,4p' "$0"; exit 1 ;;
esac
