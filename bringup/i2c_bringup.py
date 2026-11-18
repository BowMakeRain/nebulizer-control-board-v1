"""
Nebulizer Control Board V1 - I2C bring-up / peripheral check  (MicroPython, ESP32-S3)

Run (board on native USB, MicroPython ESP32_GENERIC_S3 firmware installed):
    mpremote run i2c_bringup.py
or paste/run in Thonny.

What it does
  * Scans both I2C buses, compares against the expected device table taken from the
    netlist / schematic, and prints PASS / FAIL / UNEXPECTED per device.
  * For every device that answers, reads real data (IDs, temperature, RH, pressure,
    flow, INA bus voltage/current, all 8 ADS7828 channels with net names).
  * Reads the status of the non-I2C digital lines (INA ALERTs, DRV8434A nFAULT, e-fuse PG/FLT).
  * Drives NO outputs unless ENABLE_FUSE = True (see below).

Pin map (from netlist, ESP32-S3-WROOM-1 module pin -> GPIO):
    Bus 0: SDA-0 = GPIO13 (U2.21), SCL-0 = GPIO12 (U2.20)
    Bus 1: SDA-1 = GPIO1  (U2.39), SCL-1 = GPIO2  (U2.38)
"""
import time
from machine import Pin, I2C

# ----------------------------------------------------------------- configuration
FREQ = 100_000
VDD = 3.3                  # ADS7828 REF_IN is tied to 3V3 (via 5.1 ohm), so this is its reference

# +VBAT_PROTECTED comes from the TPS16851 e-fuse (U1). INA740 bus-voltage and heater
# current only read non-zero when the e-fuse is on. Leave False for a safe I2C-only test;
# set True to drive FUSE_EN (GPIO43) high first. NOTE: GPIO43/44 are also UART0 TX/RX
# (the ROM boot log toggles FUSE_EN at reset).
ENABLE_FUSE = False

# Flow sensor assumptions (Sensirion SLF3x). Change to match the fitted part.
FLOW_START_CMD = b"\x36\x08"   # 0x3608 = water calibration, 0x3616 = isopropanol
FLOW_SCALE = 500.0             # LSB per (ml/min); SLF3S-1300F = 500, SLF3S-0600F = 10 (uL/min -> see datasheet)
FLOW_TEMP_SCALE = 200.0        # LSB per degC

# Honeywell SSC part SSCDANN150PG2A3: 0..150 psi gauge, 10%..90% of 14-bit, I2C 0x28, 3.3 V
P_MIN, P_MAX = 0.0, 150.0
OUT_MIN, OUT_MAX = 1638, 14746

PINS = dict(
    fuse_en=43, fuse_pg=44, fuse_flt=4,
    drv_nfault=8,
    alrt_reg=41, alrt_nozzle=35, alrt_x1=38, alrt_x2=10, alrt_x3=48, alrt_x4=21,
)

BUSES = {
    0: dict(scl=12, sda=13),
    1: dict(scl=2, sda=1),
}

ADS_CH_NAMES = [
    "REG_THERM (J3.1)", "NOZZLE_THERM (J3.3)", "EXTRA_THERM1 (J3.5)", "EXTRA_THERM2 (J3.7)",
    "EXTRA_THERM3 (J3.9)", "EXTRA_THERM4 (J3.11)", "ADS_CH6 / FUSE_IMON (J13.1)",
    "ADS_CH7 / FUSE_TEMP (J13.2)",
]
# ADS7828 single-ended channel -> C2C1C0 select bits
ADS_SEL = [0b000, 0b100, 0b001, 0b101, 0b010, 0b110, 0b011, 0b111]


# ----------------------------------------------------------------- helpers
def crc8(data):
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def rd16(i2c, a, reg):
    d = i2c.readfrom_mem(a, reg, 2)
    return (d[0] << 8) | d[1]


def s16(v):
    return v - 65536 if v & 0x8000 else v


def ntc_c(r, r0=10000.0, beta=3950.0):
    import math
    return 1.0 / (1.0 / 298.15 + math.log(r / r0) / beta) - 273.15


# ----------------------------------------------------------------- probes
# Each probe returns (ok: bool, [lines]). Exceptions are caught by the runner.
def probe_sht45(i2c, a):
    lines = []
    i2c.writeto(a, b"\x89")
    time.sleep_ms(2)
    s = i2c.readfrom(a, 6)
    sn_ok = crc8(s[0:2]) == s[2] and crc8(s[3:5]) == s[5]
    lines.append("serial #%02X%02X%02X%02X  (crc %s)" % (s[0], s[1], s[3], s[4], "ok" if sn_ok else "BAD"))
    i2c.writeto(a, b"\xFD")          # high-precision measurement
    time.sleep_ms(12)
    d = i2c.readfrom(a, 6)
    ok = crc8(d[0:2]) == d[2] and crc8(d[3:5]) == d[5]
    t = -45 + 175 * ((d[0] << 8) | d[1]) / 65535
    rh = max(0.0, min(100.0, -6 + 125 * ((d[3] << 8) | d[4]) / 65535))
    lines.append("T = %.2f C   RH = %.1f %%   (crc %s)" % (t, rh, "ok" if ok else "BAD"))
    return ok and sn_ok, lines


def probe_eeprom(i2c, a):
    ids = i2c.readfrom_mem(a, 0xFA, 6)
    ok = ids[0] == 0x29 and ids[1] == 0x41          # Microchip mfr / 24AA02UID device code
    user = i2c.readfrom_mem(a, 0x00, 16)
    lines = [
        "mfr 0x%02X  dev 0x%02X  (expect 0x29 / 0x41 for 24AA02UID)" % (ids[0], ids[1]),
        "32-bit UID = %02X%02X%02X%02X" % tuple(ids[2:6]),
        "user[0x00..0x0F] = " + " ".join("%02X" % b for b in user),
        "NOTE: WP (pin 4) is tied to 3V3 -> upper half (0x80-0xFF) is write-protected",
    ]
    return ok, lines


def probe_ina740(i2c, a):
    mfr = rd16(i2c, a, 0x3E)
    dev = rd16(i2c, a, 0x3F)
    ok = mfr == 0x5449                               # 'TI'
    cfg = rd16(i2c, a, 0x00)
    vbus = rd16(i2c, a, 0x05) * 0.003125             # 3.125 mV/LSB
    die = s16(rd16(i2c, a, 0x06)) / 16 * 0.125       # 125 mC/LSB, 12-bit left-justified (assumed)
    cur = s16(rd16(i2c, a, 0x07)) * 1.2              # 1.2 mA/LSB (datasheet)
    lines = [
        "MFR_ID 0x%04X (expect 0x5449 'TI')  DEV_ID 0x%04X  CONFIG 0x%04X" % (mfr, dev, cfg),
        "VBUS = %.2f V   I = %.0f mA   Tdie = %.1f C" % (vbus, cur, die),
    ]
    if vbus < 1.0:
        lines.append("VBUS ~0 V: e-fuse (U1) probably off -> set ENABLE_FUSE=True or check FUSE_EN")
    return ok, lines


def probe_ssc(i2c, a):
    i2c.writeto(a, b"")                              # address-only write = start measurement
    time.sleep_ms(10)
    d = i2c.readfrom(a, 4)
    status = d[0] >> 6
    praw = ((d[0] & 0x3F) << 8) | d[1]
    traw = (d[2] << 3) | (d[3] >> 5)
    p = (praw - OUT_MIN) * (P_MAX - P_MIN) / (OUT_MAX - OUT_MIN) + P_MIN
    t = traw * 200.0 / 2047 - 50
    names = {0: "normal", 1: "command mode", 2: "STALE data", 3: "DIAGNOSTIC FAULT"}
    ok = status in (0, 2)
    return ok, ["status=%s  pressure = %.2f psi (raw %d)   T = %.1f C" % (names[status], p, praw, t)]


def probe_ads7828(i2c, a):
    lines = []
    ok = True
    for ch in range(8):
        # SD=1, C2..C0, PD=01 (internal ref OFF, ADC on) because REF_IN is tied to VDD
        cmd = 0x80 | (ADS_SEL[ch] << 4) | (0b01 << 2)
        i2c.writeto(a, bytes([cmd]))
        i2c.readfrom(a, 2)                           # discard first (settling)
        i2c.writeto(a, bytes([cmd]))
        d = i2c.readfrom(a, 2)
        raw = ((d[0] << 8) | d[1]) & 0x0FFF
        v = raw * VDD / 4096
        extra = ""
        if ch < 6:                                   # 10k pull-up to 3V3 + NTC to GND
            if raw >= 4090:
                extra = "  -> open (no thermistor on J3)"
            elif raw <= 5:
                extra = "  -> shorted to GND"
            else:
                r = 10000.0 * v / (VDD - v)
                extra = "  -> NTC %.0f ohm ~ %.1f C (assumes 10k/B3950)" % (r, ntc_c(r))
        lines.append("CH%d %-28s raw %4d  %.3f V%s" % (ch, ADS_CH_NAMES[ch], raw, v, extra))
    return ok, lines


def probe_flow(i2c, a):
    lines = []
    try:                                             # product ID (must be outside measurement mode)
        i2c.writeto(a, b"\x36\x7C")
        i2c.writeto(a, b"\xE1\x02")
        p = i2c.readfrom(a, 18)
        lines.append("product id 0x%02X%02X%02X%02X" % (p[0], p[1], p[3], p[4]))
    except OSError:
        lines.append("product-id read not supported / failed (continuing)")
    i2c.writeto(a, FLOW_START_CMD)
    time.sleep_ms(50)
    i2c.readfrom(a, 9)                               # discard first
    time.sleep_ms(20)
    d = i2c.readfrom(a, 9)
    ok = crc8(d[0:2]) == d[2] and crc8(d[3:5]) == d[5] and crc8(d[6:8]) == d[8]
    flow = s16((d[0] << 8) | d[1]) / FLOW_SCALE
    temp = s16((d[3] << 8) | d[4]) / FLOW_TEMP_SCALE
    lines.append("flow = %.3f (ml/min, scale %g)   T = %.1f C   flags 0x%02X%02X   (crc %s)"
                 % (flow, FLOW_SCALE, temp, d[6], d[7], "ok" if ok else "BAD"))
    try:
        i2c.writeto(a, b"\x3F\xF9")                  # stop continuous measurement
    except OSError:
        pass
    return ok, lines


# ----------------------------------------------------------------- expected device table
# (bus, addr, refdes, name, probe)
EXPECTED = [
    (0, 0x28, "U14", "Pressure sensor (bus 0)", probe_ssc),
    (0, 0x40, "U9",  "INA740 REG heater", probe_ina740),
    (0, 0x41, "U12", "INA740 NOZZLE heater", probe_ina740),
    (0, 0x44, "U4",  "SHT45 temp/RH", probe_sht45),
    (0, 0x45, "U16", "INA740 EXTRA heater 4", probe_ina740),
    (0, 0x48, "U11", "ADS7828 12-bit mux ADC", probe_ads7828),
    (0, 0x50, "U5",  "24AA02UID EEPROM", probe_eeprom),
    (1, 0x08, "J6",  "Flow sensor (external, J6)", probe_flow),
    (1, 0x28, "U17", "Pressure sensor (bus 1)", probe_ssc),
    (1, 0x40, "U15", "INA740 EXTRA heater 1", probe_ina740),
    (1, 0x41, "U10", "INA740 EXTRA heater 2", probe_ina740),
    (1, 0x44, "U13", "INA740 EXTRA heater 3", probe_ina740),
]


# ----------------------------------------------------------------- GPIO status
def gpio_report():
    print("\n=== Digital status lines (inputs only) ===")
    chk = [
        ("DRV8434A nFAULT", "drv_nfault", 1, "low = motor driver fault"),
        ("FUSE_FLT (U1)", "fuse_flt", 1, "low = e-fuse fault"),
        ("FUSE_PG (U1)", "fuse_pg", None, "high = power good (expected only if fuse enabled)"),
        ("ALERT REG heater", "alrt_reg", 1, "open-drain, pulled up; low = alert asserted"),
        ("ALERT NOZZLE heater", "alrt_nozzle", 1, ""),
        ("ALERT EXTRA1", "alrt_x1", 1, ""),
        ("ALERT EXTRA2", "alrt_x2", 1, ""),
        ("ALERT EXTRA3", "alrt_x3", 1, ""),
        ("ALERT EXTRA4", "alrt_x4", 1, ""),
    ]
    for label, key, want, note in chk:
        v = Pin(PINS[key], Pin.IN).value()
        flag = "" if want is None or v == want else "   <-- unexpected"
        print("  %-22s GPIO%-2d = %d  %s%s" % (label, PINS[key], v, note, flag))


# ----------------------------------------------------------------- main
def main():
    print("=" * 66)
    print(" Nebulizer Control Board V1 - I2C bring-up")
    print("=" * 66)

    if ENABLE_FUSE:
        Pin(PINS["fuse_en"], Pin.OUT, value=1)
        time.sleep_ms(300)
        print("FUSE_EN driven high; FUSE_PG = %d" % Pin(PINS["fuse_pg"], Pin.IN).value())
    else:
        print("ENABLE_FUSE=False: e-fuse not driven (INA bus voltage will read ~0 V)")

    buses = {}
    found = {}
    for n, cfg in BUSES.items():
        try:
            buses[n] = I2C(n, scl=Pin(cfg["scl"]), sda=Pin(cfg["sda"]), freq=FREQ)
            found[n] = set(buses[n].scan())
        except Exception as e:
            buses[n] = None
            found[n] = set()
            print("!! Bus %d (SDA=GPIO%d SCL=GPIO%d) init/scan error: %s" % (n, cfg["sda"], cfg["scl"], e))
        print("\nBus %d (SDA=GPIO%d, SCL=GPIO%d) scan: %s" % (
            n, cfg["sda"], cfg["scl"], " ".join("0x%02X" % a for a in sorted(found[n])) or "NOTHING FOUND"))
        if not found[n]:
            print("   -> empty bus: check 3V3, 4.7k pull-ups (R7/R8 bus0, R25/R26 bus1), SDA/SCL shorted to GND?")
            for name, pin in (("SDA", cfg["sda"]), ("SCL", cfg["scl"])):
                if Pin(pin, Pin.IN).value() == 0:
                    print("   -> %s (GPIO%d) is stuck LOW" % (name, pin))

    summary = []
    expected_keys = set()
    for bus, addr, ref, name, probe in EXPECTED:
        expected_keys.add((bus, addr))
        print("\n--- Bus %d  0x%02X  %-6s %s" % (bus, addr, ref, name))
        if addr not in found[bus]:
            print("   [FAIL] no ACK - device not responding")
            summary.append((bus, addr, ref, name, "NO RESPONSE"))
            continue
        try:
            ok, lines = probe(buses[bus], addr)
            for ln in lines:
                print("   " + ln)
            print("   [%s]" % ("PASS" if ok else "WARN"))
            summary.append((bus, addr, ref, name, "OK" if ok else "RESPONDS, DATA SUSPECT"))
        except Exception as e:
            print("   [FAIL] ACKs on scan but data transfer failed: %r" % e)
            summary.append((bus, addr, ref, name, "ACK, READ FAILED"))

    print("\n=== Unexpected devices ===")
    extra = [(b, a) for b in found for a in sorted(found[b]) if (b, a) not in expected_keys]
    if not extra:
        print("  none")
    for b, a in extra:
        hint = ""
        if (b, 0x41) in expected_keys and a == 0x44 or (b, 0x44) in expected_keys and a == 0x41:
            hint = "  (INA A0/A1 strap swap? 0x41 <-> 0x44)"
        print("  Bus %d  0x%02X%s" % (b, a, hint))

    gpio_report()

    print("\n" + "=" * 66)
    print(" SUMMARY")
    print("=" * 66)
    for bus, addr, ref, name, st in summary:
        print("  bus%d 0x%02X %-5s %-30s %s" % (bus, addr, ref, name, st))
    bad = [s for s in summary if s[4] != "OK"]
    print("\n  %d/%d expected devices OK" % (len(summary) - len(bad), len(summary)))


main()
