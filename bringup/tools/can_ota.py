#!/usr/bin/env python3
"""Update the Nebulizer board's firmware over CAN, through the Feather RP2040 CAN (USB to the Feather).

  python3 tools/can_ota.py --bin build/NebulizerLab.ino.bin [--port /dev/cu.usbmodem101] [--no-confirm]

Close the GUI bridge first (it owns the Feather's serial port). The board must have USB idle (CAN is refused while a USB host
is talking to it). After the update the new image has 60 s to hear from this host over CAN, otherwise the board rolls back to
the previous firmware; this script keeps PINGs going until the new image is confirmed (--no-confirm skips that, to test rollback).
"""
import argparse, glob, sys, time, zlib
import serial
import serial.tools.list_ports as lp

ap = argparse.ArgumentParser()
ap.add_argument("--bin", required=True); ap.add_argument("--port")
ap.add_argument("--no-confirm", action="store_true", help="do not send PINGs after the update (the board should roll back)")
ap.add_argument("--info", action="store_true", help="just print the board's slot / build id")
a = ap.parse_args()

port = a.port or next((p.device for p in lp.comports() if p.vid == 0x239A and "usbmodem" in p.device), None)
if not port: sys.exit("Feather CAN not found (USB vid 239A)")
try: s = serial.Serial(port, 115200, timeout=0.5)
except serial.SerialException as e: sys.exit(f"cannot open {port}: {e}\n(close the GUI bridge: pkill -f tools/bridge.py)")

def line(timeout=1.0):
    end = time.time() + timeout
    while time.time() < end:
        l = s.readline().decode(errors="replace").strip()
        if l and not l.startswith("{"): return l
    return None
def cmd(c): s.write((c + "\n").encode())
def info(retries=3):
    for _ in range(retries):
        s.reset_input_buffer(); cmd("INFO"); end = time.time() + 1.5
        while time.time() < end:
            l = line(0.5)
            if l and l.startswith("INFO"):
                return l
    return None

cmd("JSON,0"); cmd("ARM,0"); time.sleep(0.3); s.reset_input_buffer()
before = info(); print("board before:", before)
if a.info or not before: sys.exit(0 if before else "board not answering over CAN")

data = open(a.bin, "rb").read(); crc = zlib.crc32(data) & 0xFFFFFFFF
print(f"image {a.bin}: {len(data)} bytes, crc32 {crc:08x}")
s.reset_input_buffer(); cmd(f"OTA,{len(data)},{crc:x}")
t0 = time.time(); last = 0; off = 0
while True:
    l = line(30)
    if l is None: sys.exit("no response from Feather")
    if l.startswith("BLK,"):
        n = int(l.split(",")[1]); s.write(data[off:off + n]); off += n
    elif l.startswith("PROG,"):
        done = int(l.split(",")[1])
        if time.time() - last > 1 or done == len(data):
            print(f"  {done}/{len(data)} bytes ({100*done//len(data)}%)  {time.time()-t0:.0f}s"); last = time.time()
    elif l.startswith("OTA DONE"): print(l); break
    elif l.startswith("OTA FAIL"): sys.exit(l)
    elif l.startswith("OTA READY"): print(l)
    else: print("  feather:", l)

slot0 = before.split("slot=")[1].split()[0]
print("board rebooting into the new image ...")
if a.no_confirm:
    print("not confirming: staying silent for 85 s (any CAN command would confirm the new image) ...")
    time.sleep(85)
    final = info(); print("final:", final)
    print("ROLLED BACK to the previous firmware" if final and f"slot={slot0}" in final else "did NOT roll back (new image still running)")
    sys.exit(0)
cmd("ARM,1"); end = time.time() + 120
seen_new = False
while time.time() < end:
    time.sleep(2); l = info(1)
    if l: print("  ", l)
    if l and "slot=" in l:
        slot = l.split("slot=")[1].split()[0]; pend = l.split("pending=")[1].split()[0]
        if slot != slot0: seen_new = True
        if seen_new and pend == "0":
            print("UPDATE CONFIRMED: new firmware is running and has confirmed itself."); cmd("ARM,0"); sys.exit(0)
cmd("ARM,0"); sys.exit("update installed but not confirmed within 120 s (board will roll back on its next reset)")
