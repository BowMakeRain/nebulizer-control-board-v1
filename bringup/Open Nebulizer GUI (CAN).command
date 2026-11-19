#!/bin/bash
# Double-click: GUI fed ONLY by CAN telemetry, through the Feather RP2040 CAN (USB to the Feather, CAN to the Nebulizer board).
cd "$(dirname "$0")"
PORT=$(python3 - <<'PY'
import serial.tools.list_ports as lp
p=[x.device for x in lp.comports() if x.vid==0x239A and 'usbmodem' in x.device]
print(p[0] if p else '/dev/cu.usbmodem101')
PY
)
lsof -ti tcp:8000 | xargs kill 2>/dev/null; sleep 1
echo "Feather port: $PORT   (leave this window open; close it to stop)"
(sleep 2; open http://localhost:8000) &
exec python3 tools/bridge.py --can --port "$PORT" --html lab.html --http 8000 --logdir logs
