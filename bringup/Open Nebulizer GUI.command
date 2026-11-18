#!/bin/bash
# Double-click: starts the serial bridge for the Nebulizer board and opens the GUI.
cd "$(dirname "$0")"
PORT=$(python3 - <<'PY'
import serial.tools.list_ports as lp
p=[x.device for x in lp.comports() if x.vid==0x303A and 'usbmodem' in x.device]
print(p[0] if p else '/dev/cu.usbmodem1101')
PY
)
lsof -ti tcp:8000 | xargs kill 2>/dev/null; sleep 1
echo "Board port: $PORT   (leave this window open; close it to stop)"
(sleep 2; open http://localhost:8000) &
exec python3 tools/bridge.py --port "$PORT" --html lab.html --http 8000 --logdir logs
