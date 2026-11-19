#!/usr/bin/env bash
# Build the Nebulizer firmware and push it to the board over CAN (through the Feather).  Usage: tools/can_update.sh [--no-confirm]
set -euo pipefail
cd "$(dirname "$0")/.."
source esp32.conf
OUT="$PWD/build"; rm -rf "$OUT"
arduino-cli compile --fqbn "$FQBN" --output-dir "$OUT" "$SKETCH" ${FW_TAG:+--build-property "compiler.cpp.extra_flags=-DFW_TAG=\"$FW_TAG\""} >/dev/null
echo "built $(ls "$OUT"/*.ino.bin)"
python3 tools/can_ota.py --bin "$OUT/$SKETCH.ino.bin" "$@"
