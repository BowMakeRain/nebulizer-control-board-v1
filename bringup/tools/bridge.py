#!/usr/bin/env python3
"""Serial -> web bridge for ESP32 projects.

* Serves an HTML dashboard and streams the board's JSON lines to it over SSE (/events); /latest = last snapshot.
* POST /cmd forwards whitelisted commands to the board (localhost + custom header only).
* Bridge-side commands: ARM,1|0 (bridge sends PING keepalives only while armed AND a browser tab is connected,
  so closing the tab stops the keepalive and the firmware dead-man switches outputs off),
  LOG,start | LOG,stop (flattened CSV written under --logdir, one row per board JSON line).
* Auto-reconnects if the board is unplugged / re-flashed.

usage: bridge.py --port /dev/cu.usbmodemXXX --html page.html [--baud 115200] [--http 8000] [--logdir logs]
"""
import argparse, csv, datetime, json, os, re, subprocess, sys, threading, time, queue
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import serial

ap = argparse.ArgumentParser()
ap.add_argument("--port", required=True); ap.add_argument("--baud", type=int, default=115200)
ap.add_argument("--html", required=True); ap.add_argument("--http", type=int, default=8000)
ap.add_argument("--logdir", default="logs")
ap.add_argument("--cfgfile", default=None, help="JSON file for saved GUI configs (default: configs.json next to --html)")
ap.add_argument("--can", action="store_true", help="port is the Feather CAN bridge: show the Nebulizer telemetry received over CAN")
a = ap.parse_args()
CFG = a.cfgfile or os.path.join(os.path.dirname(os.path.abspath(a.html)), "configs.json")

N = r"-?[0-9]+(?:\.[0-9]+)?"
CMD_RE = re.compile(
    r"^(PING|OFF|CA|FUSE,[01]|H,[0-5],[01]|M,[0-5],[0-2]|C,[0-5]|TH,{n},{n}|[DSF],[0-5],{n}|"
    r"CT,[0-9]{{1,4}}|I2C,[0-9]{{2,4}}|TR,[0-9]{{1,4}}|SOL,[01]|MR,[01]|MP,[01]|ME,[0-2]|MD,[01]|ML,[0-2]|MS,{n}|MA,{n}|MC|G,[0-5],{n},{n},{n}|X,[0-5],{n},{n}|CAN,[0-9]{{1,4}}(?:,[0-2])?|CANT,[0-9A-Fa-f]{{1,3}},[0-9A-Fa-f]{{0,16}})$".format(n=N))
BRIDGE_RE = re.compile(r"^(ARM,[01]|LOG,start|LOG,stop)$")

ser = None
state = {"armed": False}
SHADOW = {}   # values the board does not send over CAN (PID gains etc.): remembered from the commands this bridge forwarded

DEVS = [(0,0x28,"U14","Pressure sensor 0"),(0,0x40,"U9","INA740 REG heater"),(0,0x41,"U12","INA740 NOZZLE heater"),(0,0x44,"U4","SHT45 temp/RH"),
        (0,0x45,"U16","INA740 EXTRA heater 4"),(0,0x48,"U11","ADS7828 ADC mux"),(0,0x50,"U5","24AA02UID EEPROM"),(1,0x08,"J6","Flow sensor"),
        (1,0x28,"U17","Pressure sensor 1"),(1,0x40,"U15","INA740 EXTRA heater 1"),(1,0x41,"U10","INA740 EXTRA heater 2"),(1,0x44,"U13","INA740 EXTRA heater 3")]
INA_IDX = [1, 2, 9, 10, 11, 4]
HKEYS = ["reg", "nozzle", "extra1", "extra2", "extra3", "extra4"]


def shadow_track(cmd):
    try:
        f = cmd.split(",")
        if f[0] in ("D", "G", "X", "F") and len(f) >= 3:
            i = int(f[1]); h = SHADOW.setdefault(i, {})
            if f[0] == "D": h["man"] = float(f[2])
            elif f[0] == "G": h["kp"], h["ki"], h["kd"] = float(f[2]), float(f[3]), float(f[4])
            elif f[0] == "X": h["md"], h["mt"] = float(f[2]), float(f[3])
            elif f[0] == "F": h["hz"] = float(f[2])
    except (ValueError, IndexError): pass


def can_to_board(c):
    """Feather JSON (decoded CAN telemetry) -> the board-schema message lab.html expects."""
    ok, live, sf = c["dev"][1], c["dev"][0], c["sf"]
    alive = bool(c["alive"])
    n = c["n"]
    def V(v, d=None): return v if v is not None else d
    ina = lambda i: c["h"][i]
    devs = []
    for i, (bus, addr, ref, name) in enumerate(DEVS):
        isok = bool(alive and (ok >> i) & 1); islive = bool(alive and (live >> i) & 1); vals = []
        if isok:
            if i in INA_IDX:
                h = ina(INA_IDX.index(i))
                vals = [["Bus voltage (V)", h[6]], ["Current (mA)", h[7]], ["Power (W)", h[8]], ["Die temp (C)", h[9]]]
            elif i in (0, 8): k = 0 if i == 0 else 1; vals = [["Pressure (psi)", c["p"][k]], ["Temperature (C)", c["pt"][k]], ["Status (0=ok)", 0]]
            elif i == 3: vals = [["Temperature (C)", c["sht"][0]], ["Humidity (%RH)", c["sht"][1]]]
            elif i == 7: vals = [["Flow (ml/min)", c["fl"][0]], ["Temperature (C)", c["fl"][1]]]
            elif i == 5: vals = [["CH6 FUSE_IMON", c["adc"][0]], ["CH7 FUSE_TEMP", c["adc"][1]]]
        devs.append({"bus": bus, "addr": addr, "ref": ref, "name": name, "live": islive, "ok": isok, "note": "", "vals": [v for v in vals if v[1] is not None]})
    htr = []
    for i, h in enumerate(c["h"]):
        sh = SHADOW.get(i, {})
        htr.append({"k": HKEYS[i], "mode": h[0], "trip": h[2], "duty_pct": V(h[5], 0), "freq_hz": sh.get("hz", 200), "sp_c": V(h[4], 40), "man_pct": sh.get("man", 0),
                    "kp": sh.get("kp", 3), "ki": sh.get("ki", 0.3), "kd": sh.get("kd", 0), "maxduty_pct": sh.get("md", 30), "maxT_c": sh.get("mt", 100),
                    "adc_raw": -1, "T_ok": 1 if h[1] else 0, "T_c": h[3], "R_ohm": None, "pid_p": 0, "pid_i": 0, "pid_d": 0})
    mf = c["m"][4]
    mot = {"sleep": mf & 1, "en": (mf >> 1) & 3, "dir": (mf >> 3) & 1, "stl": c["m"][5], "stl_dis": (mf >> 4) & 1, "target_hz": c["m"][1] / 10.0,
           "hz": c["m"][0], "real_hz": c["m"][0], "accel": c["m"][2], "nfault": (mf >> 5) & 1, "stl_rep": (mf >> 6) & 1, "trq_mv": c["m"][3]}
    gpio = [["DRV nFAULT", (sf >> 4) & 1], ["STL_REP", (sf >> 5) & 1], ["FUSE_FLT", (sf >> 1) & 1], ["FUSE_PG", sf & 1], ["FUSE_EN", 0 if (sf >> 2) & 1 else 1]]
    smp_dev = [n["press"], n["pow"][0], n["pow"][1], n["env"], n["pow"][5], n["adc"], 0, n["env"], n["press"], n["pow"][2], n["pow"][3], n["pow"][4]]
    return {"t": c["up"], "src": "can", "stuck": [False, False], "unexpected": [], "gpio": gpio, "devs": devs, "htr": htr, "sol": 1 if (sf >> 3) & 1 else 0, "mot": mot,
            "can": {"on": 1, "kbps": 500, "mode": 0, "state": "running" if alive else "no board", "rx": c["rx"], "tx": c["tx"], "tx_fail": c["txf"],
                    "tec": c["dev"][3], "rec": c["dev"][4], "bus_err": 0, "rx_miss": 0, "arb_lost": 0, "log": []},
            "smp": {"cyc": n["dev"], "ctl": n["heat"][0], "dev": smp_dev, "therm": n["heat"]},
            "sys": {"i2c_khz": c["adc"][2], "telem_ms": 0, "ctl_ms": c["adc"][3], "rst": c["rst"], "inhibit": (sf >> 2) & 1, "heap": 0, "ctl_max_ms": 0,
                    "cycle_ms": c["dev"][2], "beta": 3950, "r0": 10000, "deadman_ms": 3000, "ctl_src": "usb" if (sf >> 7) & 1 else "can", "can_ms": c["adc"][3]}}

latest = {"line": None, "ts": 0, "connected": False}
subs, lock = [], threading.Lock()


class Logger:
    """Writes flattened rows; if new columns appear later the file is rewritten with the wider header."""
    def __init__(self, d):
        self.dir, self.f, self.w, self.cols, self.colset, self.rows, self.path = d, None, None, [], set(), 0, None

    @property
    def on(self): return self.f is not None

    def start(self):
        if self.on: return
        os.makedirs(self.dir, exist_ok=True)
        self.path = os.path.join(self.dir, "nebulizer_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")
        self.f = open(self.path, "w", newline=""); self.w = csv.writer(self.f)
        self.cols, self.colset, self.rows = [], set(), 0

    def stop(self):
        if self.f: self.f.close()
        self.f = self.w = None

    def _rewrite(self):
        self.f.close()
        with open(self.path, newline="") as fh: old = list(csv.reader(fh))[1:]
        self.f = open(self.path, "w", newline=""); self.w = csv.writer(self.f)
        self.w.writerow(self.cols)
        for r in old: self.w.writerow(r + [""] * (len(self.cols) - len(r)))

    def write(self, row):
        new = [k for k in row if k not in self.colset]
        if new:
            self.cols += new; self.colset.update(new); self._rewrite()
        self.w.writerow([row.get(c, "") for c in self.cols]); self.f.flush(); self.rows += 1

    def info(self):
        return {"on": self.on, "file": os.path.abspath(self.path) if self.path else None, "rows": self.rows}


logger = Logger(a.logdir)


def flatten(m):
    now = datetime.datetime.now()
    row = {"host_iso": now.isoformat(timespec="milliseconds"), "host_epoch_s": round(now.timestamp(), 3),
           "board_ms": m.get("t")}
    for d in m.get("devs", []):
        p = d["ref"]
        row[p + ".live"] = int(d["live"]); row[p + ".ok"] = int(d["ok"])
        if d.get("note"): row[p + ".note"] = d["note"]
        for k, v in d.get("vals", []): row[f"{p}.{k}"] = v
    for h in m.get("htr", []):
        for k, v in h.items():
            if k != "k": row[f"{h['k']}.{k}"] = int(v) if isinstance(v, bool) else v
    for k, v in m.get("gpio", []): row["gpio." + k] = v
    for k, v in m.get("sys", {}).items(): row["sys." + k] = v
    row["i2c.unexpected"] = ";".join(f"{b}:0x{ad:02X}" for b, ad in m.get("unexpected", []))
    row["i2c.stuck"] = ";".join(str(int(x)) for x in m.get("stuck", []))
    return row


def reader():
    global ser
    while True:
        try:
            s = serial.Serial(); s.port, s.baudrate, s.timeout = a.port, a.baud, 1
            s.dtr = bool(a.can); s.rts = False; s.open()  # do not reset chip on open
            latest["connected"] = True; ser = s
            print("serial connected", a.port, flush=True)
            if a.can: s.write(b"JSON,100\n")
            buf = b""
            while True:
                buf += s.read(s.in_waiting or 1)
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.decode(errors="replace").strip()
                    if not line.startswith("{"):
                        if line: print("board:", line, flush=True)
                        if "waiting for download" in line:     # chip fell into the ROM bootloader (GPIO0 low at reset)
                            print("board is in download mode: resetting it", flush=True)
                            try: s.close()
                            except Exception: pass
                            ser = None; latest["connected"] = False
                            subprocess.run([sys.executable, "-m", "esptool", "--port", a.port, "--after", "watchdog-reset", "chip-id"],
                                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
                            time.sleep(3); raise RuntimeError("recovered from download mode")
                        continue
                    try: m = json.loads(line)
                    except ValueError: continue
                    if a.can:
                        if m.get("can") != 1: continue
                        try: m = can_to_board(m)
                        except Exception as e: print("can decode error:", e, flush=True); continue
                    if logger.on:
                        try: logger.write(flatten(m))
                        except Exception as e: print("log error:", e, flush=True)
                    m["_host"] = {"armed": state["armed"], "log": logger.info()}
                    out = json.dumps(m, separators=(",", ":"))
                    latest["line"], latest["ts"] = out, time.time()
                    with lock:
                        for q in subs: q.put(out)
        except Exception as e:
            latest["connected"] = False; ser = None
            print("serial error:", e, "- retrying", flush=True)
            time.sleep(1)


def keepalive():
    while True:
        time.sleep(1)
        if a.can and ser is not None and time.time() - latest["ts"] > 2:      # (re)ask the Feather for the decoded stream
            try: ser.write(b"JSON,100\n")
            except Exception: pass
        with lock: viewers = len(subs)
        if state["armed"] and viewers and ser is not None:
            try: ser.write(b"PING\n")
            except Exception: pass
        elif state["armed"] and not viewers:
            state["armed"] = False          # tab gone: disarm (firmware dead-man turns outputs off)
            print("no viewers: disarmed", flush=True)


class H(BaseHTTPRequestHandler):
    def log_message(self, *x): pass

    def host_ok(self):   # block DNS-rebinding / cross-site requests
        return self.headers.get("Host", "") in (f"localhost:{a.http}", f"127.0.0.1:{a.http}")

    def do_POST(self):
        if self.path == "/configs":                       # save GUI configs (JSON, <= 256 KB)
            origin_ok = self.headers.get("Origin", f"http://localhost:{a.http}") in (f"http://localhost:{a.http}", f"http://127.0.0.1:{a.http}")
            code = 403
            if self.host_ok() and origin_ok and self.headers.get("X-Requested-With") == "dashboard":
                try:
                    body = self.rfile.read(min(int(self.headers.get("Content-Length", 0)), 262144))
                    json.loads(body); tmp = CFG + ".tmp"
                    with open(tmp, "wb") as fh: fh.write(body)
                    os.replace(tmp, CFG); code = 200
                except Exception: code = 400
            self.send_response(code); self.send_header("Content-Length", "0"); self.end_headers(); return
        origin_ok = self.headers.get("Origin", f"http://localhost:{a.http}") in (
            f"http://localhost:{a.http}", f"http://127.0.0.1:{a.http}")
        ok = self.path == "/cmd" and self.host_ok() and origin_ok and self.headers.get("X-Requested-With") == "dashboard"
        cmd = self.rfile.read(min(int(self.headers.get("Content-Length", 0)), 96)).decode(errors="replace").strip() if ok else ""
        code = 403
        if ok and BRIDGE_RE.match(cmd):
            code = 200
            if cmd.startswith("ARM"): state["armed"] = cmd.endswith("1")
            elif cmd == "LOG,start": logger.start()
            else: logger.stop()
        elif ok and CMD_RE.match(cmd):
            if ser is None: code = 503
            else:
                try: ser.write((cmd + "\n").encode()); code = 200; shadow_track(cmd)
                except Exception: code = 503
        self.send_response(code); self.send_header("Content-Length", "0"); self.end_headers()

    def do_GET(self):
        if not self.host_ok():
            self.send_response(403); self.end_headers(); return
        if self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream"); self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = queue.Queue()
            with lock: subs.append(q)
            try:
                if latest["line"]: self.wfile.write(f"data: {latest['line']}\n\n".encode()); self.wfile.flush()
                while True:
                    try: self.wfile.write(f"data: {q.get(timeout=1)}\n\n".encode())
                    except queue.Empty: self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
            except Exception: pass
            finally:
                with lock: subs.remove(q)
        elif self.path == "/configs":
            body = open(CFG, "rb").read() if os.path.exists(CFG) else b'{"configs":{},"auto":null}'
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers(); self.wfile.write(body)
        elif self.path == "/latest":
            body = json.dumps({"connected": latest["connected"],
                               "age_s": time.time() - latest["ts"] if latest["ts"] else None,
                               "data": json.loads(latest["line"]) if latest["line"] else None}).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers(); self.wfile.write(body)
        else:
            body = open(a.html, "rb").read()
            self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8"); self.end_headers(); self.wfile.write(body)


threading.Thread(target=reader, daemon=True).start()
threading.Thread(target=keepalive, daemon=True).start()
print(f"Dashboard: http://localhost:{a.http}   (JSON snapshot: /latest, CSV logs: {os.path.abspath(a.logdir)}, configs: {CFG})", flush=True)
try:
    ThreadingHTTPServer(("127.0.0.1", a.http), H).serve_forever()
finally:
    logger.stop()
