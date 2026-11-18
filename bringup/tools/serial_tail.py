#!/usr/bin/env python3
"""Print serial output for N seconds. usage: serial_tail.py PORT BAUD SECONDS"""
import sys, time, serial
port, baud, secs = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
s = serial.Serial(); s.port, s.baudrate, s.timeout = port, baud, 0.5
s.dtr = False; s.rts = False; s.open()  # preset lines so opening does not reset the chip
end = time.time() + secs
got = False
while time.time() < end:
    d = s.read(4096)
    if d:
        got = True
        sys.stdout.write(d.decode(errors="replace")); sys.stdout.flush()
if not got:
    print("(no serial data received)")
