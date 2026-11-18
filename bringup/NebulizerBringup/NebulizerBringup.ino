/*
  Nebulizer Control Board V1 - I2C bring-up / peripheral check (Arduino, ESP32-S3)

  Arduino IDE settings (Tools menu):
    Board:               ESP32S3 Dev Module
    USB CDC On Boot:     Enabled      <-- needed so Serial goes over the native USB port (J1)
    Flash Size:          16MB
    PSRAM:               QSPI PSRAM   (N16R2)
    Upload Mode:         UART0 / Hardware CDC
  Open Serial Monitor at any baud (native USB). Send any character to re-run the test.

  Bus 0: SDA=GPIO13 SCL=GPIO12 (Wire)     Bus 1: SDA=GPIO1 SCL=GPIO2 (Wire1)
  Drives NO outputs unless ENABLE_FUSE is true.
*/
#include <Wire.h>
#include <math.h>

// Serial must be the native USB port. Otherwise it is UART0 = GPIO43/44 = FUSE_EN/FUSE_PG on this board!
#if !ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot > Enabled, then upload again."
#endif

// ---------------------------------------------------------------- configuration
static const uint32_t I2C_FREQ = 100000;
static const float VDD = 3.3f;          // ADS7828 REF_IN is tied to 3V3

// +VBAT_PROTECTED comes from the TPS16851 e-fuse. INA740 bus voltage reads ~0 V unless it is on.
// GPIO43/44 are also UART0 TX/RX, so the ROM boot log can toggle FUSE_EN at reset.
static const bool ENABLE_FUSE = false;

// Flow sensor (Sensirion SLF3x) assumptions - adjust to the fitted part
static const uint8_t FLOW_START_CMD[2] = {0x36, 0x08};  // 0x3608 water, 0x3616 isopropanol
static const float FLOW_SCALE = 500.0f;                 // LSB per ml/min (SLF3S-1300F)
static const float FLOW_TEMP_SCALE = 200.0f;            // LSB per degC

// Honeywell SSCDANN150PG2A3: 0..150 psi gauge, 10-90% of 14 bit
static const float P_MIN = 0.0f, P_MAX = 150.0f;
static const int OUT_MIN = 1638, OUT_MAX = 14746;

static const int PIN_FUSE_EN = 43, PIN_FUSE_PG = 44, PIN_FUSE_FLT = 4, PIN_DRV_NFAULT = 8;

static const char *ADS_NAMES[8] = {
  "REG_THERM (J3.1)", "NOZZLE_THERM (J3.3)", "EXTRA_THERM1 (J3.5)", "EXTRA_THERM2 (J3.7)",
  "EXTRA_THERM3 (J3.9)", "EXTRA_THERM4 (J3.11)", "ADS_CH6/FUSE_IMON (J13.1)", "ADS_CH7/FUSE_TEMP (J13.2)"
};
static const uint8_t ADS_SEL[8] = {0b000, 0b100, 0b001, 0b101, 0b010, 0b110, 0b011, 0b111};

// ---------------------------------------------------------------- helpers
static uint8_t crc8(const uint8_t *d, int n) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

static bool readBytes(TwoWire &w, uint8_t a, uint8_t n, uint8_t *buf) {
  if (w.requestFrom((int)a, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = w.read();
  return true;
}

static bool readReg(TwoWire &w, uint8_t a, uint8_t reg, uint8_t n, uint8_t *buf) {
  w.beginTransmission(a);
  w.write(reg);
  if (w.endTransmission(false) != 0) return false;
  return readBytes(w, a, n, buf);
}

static bool readReg16(TwoWire &w, uint8_t a, uint8_t reg, uint16_t &v) {
  uint8_t b[2];
  if (!readReg(w, a, reg, 2, b)) return false;
  v = ((uint16_t)b[0] << 8) | b[1];
  return true;
}

static bool writeCmd(TwoWire &w, uint8_t a, const uint8_t *c, uint8_t n) {
  w.beginTransmission(a);
  w.write(c, n);
  return w.endTransmission() == 0;
}

// ---------------------------------------------------------------- probes (return true = healthy)
static bool probeSHT45(TwoWire &w, uint8_t a) {
  uint8_t s[6], d[6];
  const uint8_t c1[1] = {0x89}, c2[1] = {0xFD};
  if (!writeCmd(w, a, c1, 1)) return false;
  delay(2);
  if (!readBytes(w, a, 6, s)) return false;
  bool snOk = crc8(s, 2) == s[2] && crc8(s + 3, 2) == s[5];
  Serial.printf("   serial #%02X%02X%02X%02X (crc %s)\n", s[0], s[1], s[3], s[4], snOk ? "ok" : "BAD");
  if (!writeCmd(w, a, c2, 1)) return false;
  delay(12);
  if (!readBytes(w, a, 6, d)) return false;
  bool ok = crc8(d, 2) == d[2] && crc8(d + 3, 2) == d[5];
  float t = -45.0f + 175.0f * (((d[0] << 8) | d[1]) / 65535.0f);
  float rh = -6.0f + 125.0f * (((d[3] << 8) | d[4]) / 65535.0f);
  rh = constrain(rh, 0.0f, 100.0f);
  Serial.printf("   T = %.2f C   RH = %.1f %%   (crc %s)\n", t, rh, ok ? "ok" : "BAD");
  return ok && snOk;
}

static bool probeEEPROM(TwoWire &w, uint8_t a) {
  uint8_t id[6], user[16];
  if (!readReg(w, a, 0xFA, 6, id)) return false;
  bool ok = id[0] == 0x29 && id[1] == 0x41;
  Serial.printf("   mfr 0x%02X dev 0x%02X (expect 0x29/0x41 for 24AA02UID)\n", id[0], id[1]);
  Serial.printf("   32-bit UID = %02X%02X%02X%02X\n", id[2], id[3], id[4], id[5]);
  if (readReg(w, a, 0x00, 16, user)) {
    Serial.print("   user[0x00..0x0F] =");
    for (int i = 0; i < 16; i++) Serial.printf(" %02X", user[i]);
    Serial.println();
  }
  Serial.println("   NOTE: WP (pin 4) tied to 3V3 -> 0x80-0xFF is write-protected");
  return ok;
}

static bool probeINA740(TwoWire &w, uint8_t a) {
  uint16_t mfr, dev, cfg, vb, dt, cur;
  if (!readReg16(w, a, 0x3E, mfr) || !readReg16(w, a, 0x3F, dev) || !readReg16(w, a, 0x00, cfg) ||
      !readReg16(w, a, 0x05, vb) || !readReg16(w, a, 0x06, dt) || !readReg16(w, a, 0x07, cur))
    return false;
  float vbus = vb * 0.003125f;               // 3.125 mV/LSB
  float die = ((int16_t)dt / 16) * 0.125f;   // 125 mC/LSB, 12-bit left-justified (assumed)
  float mA = (int16_t)cur * 1.2f;            // 1.2 mA/LSB
  Serial.printf("   MFR_ID 0x%04X (expect 0x5449 'TI')  DEV_ID 0x%04X  CONFIG 0x%04X\n", mfr, dev, cfg);
  Serial.printf("   VBUS = %.2f V   I = %.0f mA   Tdie = %.1f C\n", vbus, mA, die);
  if (vbus < 1.0f) Serial.println("   VBUS ~0 V: e-fuse (U1) probably off -> set ENABLE_FUSE=true or check FUSE_EN");
  return mfr == 0x5449;
}

static bool probeSSC(TwoWire &w, uint8_t a) {
  uint8_t d[4];
  w.beginTransmission(a);                    // address-only write = start measurement
  if (w.endTransmission() != 0) return false;
  delay(10);
  if (!readBytes(w, a, 4, d)) return false;
  int status = d[0] >> 6;
  int praw = ((d[0] & 0x3F) << 8) | d[1];
  int traw = (d[2] << 3) | (d[3] >> 5);
  float p = (praw - OUT_MIN) * (P_MAX - P_MIN) / (OUT_MAX - OUT_MIN) + P_MIN;
  float t = traw * 200.0f / 2047.0f - 50.0f;
  const char *names[4] = {"normal", "command mode", "STALE data", "DIAGNOSTIC FAULT"};
  Serial.printf("   status=%s  pressure = %.2f psi (raw %d)  T = %.1f C\n", names[status], p, praw, t);
  return status == 0 || status == 2;
}

static bool probeADS7828(TwoWire &w, uint8_t a) {
  for (int ch = 0; ch < 8; ch++) {
    // SD=1, C2..C0, PD=01 (internal ref OFF, ADC on) because REF_IN is tied to VDD
    uint8_t cmd = 0x80 | (ADS_SEL[ch] << 4) | (0b01 << 2);
    uint8_t d[2];
    if (!writeCmd(w, a, &cmd, 1) || !readBytes(w, a, 2, d)) return false;  // discard (settling)
    if (!writeCmd(w, a, &cmd, 1) || !readBytes(w, a, 2, d)) return false;
    int raw = (((d[0] << 8) | d[1]) & 0x0FFF);
    float v = raw * VDD / 4096.0f;
    Serial.printf("   CH%d %-26s raw %4d  %.3f V", ch, ADS_NAMES[ch], raw, v);
    if (ch < 6) {                            // 10k pull-up to 3V3 + NTC to GND
      if (raw >= 4090) Serial.print("  -> open (no thermistor)");
      else if (raw <= 5) Serial.print("  -> shorted to GND");
      else {
        float r = 10000.0f * v / (VDD - v);
        float c = 1.0f / (1.0f / 298.15f + logf(r / 10000.0f) / 3950.0f) - 273.15f;
        Serial.printf("  -> NTC %.0f ohm ~ %.1f C (assumes 10k/B3950)", r, c);
      }
    }
    Serial.println();
  }
  return true;
}

static bool probeFlow(TwoWire &w, uint8_t a) {
  uint8_t p[18], d[9];
  const uint8_t c1[2] = {0x36, 0x7C}, c2[2] = {0xE1, 0x02}, stop[2] = {0x3F, 0xF9};
  if (writeCmd(w, a, c1, 2) && writeCmd(w, a, c2, 2) && readBytes(w, a, 18, p))
    Serial.printf("   product id 0x%02X%02X%02X%02X\n", p[0], p[1], p[3], p[4]);
  else
    Serial.println("   product-id read failed (continuing)");
  if (!writeCmd(w, a, FLOW_START_CMD, 2)) return false;
  delay(50);
  readBytes(w, a, 9, d);                     // discard first
  delay(20);
  if (!readBytes(w, a, 9, d)) return false;
  bool ok = crc8(d, 2) == d[2] && crc8(d + 3, 2) == d[5] && crc8(d + 6, 2) == d[8];
  float flow = (int16_t)((d[0] << 8) | d[1]) / FLOW_SCALE;
  float temp = (int16_t)((d[3] << 8) | d[4]) / FLOW_TEMP_SCALE;
  Serial.printf("   flow = %.3f ml/min (scale %g)  T = %.1f C  flags 0x%02X%02X  (crc %s)\n",
                flow, FLOW_SCALE, temp, d[6], d[7], ok ? "ok" : "BAD");
  writeCmd(w, a, stop, 2);
  return ok;
}

// ---------------------------------------------------------------- expected device table
typedef bool (*ProbeFn)(TwoWire &, uint8_t);
struct Dev { uint8_t bus; uint8_t addr; const char *ref; const char *name; ProbeFn probe; };

static const Dev DEVS[] = {
  {0, 0x28, "U14", "Pressure sensor (bus 0)", probeSSC},
  {0, 0x40, "U9",  "INA740 REG heater", probeINA740},
  {0, 0x41, "U12", "INA740 NOZZLE heater", probeINA740},
  {0, 0x44, "U4",  "SHT45 temp/RH", probeSHT45},
  {0, 0x45, "U16", "INA740 EXTRA heater 4", probeINA740},
  {0, 0x48, "U11", "ADS7828 12-bit mux ADC", probeADS7828},
  {0, 0x50, "U5",  "24AA02UID EEPROM", probeEEPROM},
  {1, 0x08, "J6",  "Flow sensor (external, J6)", probeFlow},
  {1, 0x28, "U17", "Pressure sensor (bus 1)", probeSSC},
  {1, 0x40, "U15", "INA740 EXTRA heater 1", probeINA740},
  {1, 0x41, "U10", "INA740 EXTRA heater 2", probeINA740},
  {1, 0x44, "U13", "INA740 EXTRA heater 3", probeINA740},
};
static const int NDEV = sizeof(DEVS) / sizeof(DEVS[0]);

struct Alert { const char *label; int pin; };
static const Alert ALERTS[] = {
  {"ALERT REG heater", 41}, {"ALERT NOZZLE heater", 35}, {"ALERT EXTRA1", 38},
  {"ALERT EXTRA2", 10}, {"ALERT EXTRA3", 48}, {"ALERT EXTRA4", 21},
};

static TwoWire &busRef(int b) { return b == 0 ? Wire : Wire1; }
static const int BUS_SDA[2] = {13, 1}, BUS_SCL[2] = {12, 2};

// ---------------------------------------------------------------- main test
static void runTest() {
  Serial.println("\n==================================================================");
  Serial.println(" Nebulizer Control Board V1 - I2C bring-up");
  Serial.println("==================================================================");

  if (ENABLE_FUSE) {
    pinMode(PIN_FUSE_EN, OUTPUT);
    digitalWrite(PIN_FUSE_EN, HIGH);
    delay(300);
    pinMode(PIN_FUSE_PG, INPUT);
    Serial.printf("FUSE_EN driven high; FUSE_PG = %d\n", digitalRead(PIN_FUSE_PG));
  } else {
    Serial.println("ENABLE_FUSE=false: e-fuse not driven (INA bus voltage will read ~0 V)");
  }

  bool found[2][128] = {};
  for (int b = 0; b < 2; b++) {
    TwoWire &w = busRef(b);
    w.end();
    w.begin(BUS_SDA[b], BUS_SCL[b], I2C_FREQ);
    w.setTimeOut(50);
    Serial.printf("\nBus %d (SDA=GPIO%d, SCL=GPIO%d) scan:", b, BUS_SDA[b], BUS_SCL[b]);
    int n = 0;
    for (uint8_t a = 1; a < 127; a++) {
      w.beginTransmission(a);
      if (w.endTransmission() == 0) { found[b][a] = true; n++; Serial.printf(" 0x%02X", a); }
    }
    if (!n) {
      Serial.println(" NOTHING FOUND");
      Serial.println("   -> check 3V3, 4.7k pull-ups (R7/R8 bus0, R25/R26 bus1), SDA/SCL shorted to GND?");
      pinMode(BUS_SDA[b], INPUT); pinMode(BUS_SCL[b], INPUT);
      if (!digitalRead(BUS_SDA[b])) Serial.println("   -> SDA is stuck LOW");
      if (!digitalRead(BUS_SCL[b])) Serial.println("   -> SCL is stuck LOW");
      w.begin(BUS_SDA[b], BUS_SCL[b], I2C_FREQ);
    } else {
      Serial.println();
    }
  }

  int okCount = 0;
  const char *status[NDEV];
  for (int i = 0; i < NDEV; i++) {
    const Dev &d = DEVS[i];
    Serial.printf("\n--- Bus %d  0x%02X  %-4s %s\n", d.bus, d.addr, d.ref, d.name);
    if (!found[d.bus][d.addr]) {
      Serial.println("   [FAIL] no ACK - device not responding");
      status[i] = "NO RESPONSE";
      continue;
    }
    bool ok = d.probe(busRef(d.bus), d.addr);
    Serial.printf("   [%s]\n", ok ? "PASS" : "FAIL/WARN - ACKs but data read failed or suspect");
    status[i] = ok ? "OK" : "ACK, DATA SUSPECT";
    if (ok) okCount++;
  }

  Serial.println("\n=== Unexpected devices ===");
  int extra = 0;
  for (int b = 0; b < 2; b++)
    for (int a = 1; a < 127; a++) {
      if (!found[b][a]) continue;
      bool expected = false;
      for (int i = 0; i < NDEV; i++) if (DEVS[i].bus == b && DEVS[i].addr == a) expected = true;
      if (!expected) {
        Serial.printf("  Bus %d  0x%02X%s\n", b, a,
                      (a == 0x41 || a == 0x44) ? "  (INA A0/A1 strap swap? 0x41 <-> 0x44)" : "");
        extra++;
      }
    }
  if (!extra) Serial.println("  none");

  Serial.println("\n=== Digital status lines (inputs only) ===");
  pinMode(PIN_DRV_NFAULT, INPUT); pinMode(PIN_FUSE_FLT, INPUT);
  if (!ENABLE_FUSE) pinMode(PIN_FUSE_PG, INPUT);
  Serial.printf("  %-22s GPIO%-2d = %d  low = motor driver fault\n", "DRV8434A nFAULT", PIN_DRV_NFAULT, digitalRead(PIN_DRV_NFAULT));
  Serial.printf("  %-22s GPIO%-2d = %d  low = e-fuse fault\n", "FUSE_FLT (U1)", PIN_FUSE_FLT, digitalRead(PIN_FUSE_FLT));
  Serial.printf("  %-22s GPIO%-2d = %d  high = power good (only if fuse enabled)\n", "FUSE_PG (U1)", PIN_FUSE_PG, digitalRead(PIN_FUSE_PG));
  for (const Alert &al : ALERTS) {
    pinMode(al.pin, INPUT);
    int v = digitalRead(al.pin);
    Serial.printf("  %-22s GPIO%-2d = %d  %s\n", al.label, al.pin, v, v ? "" : "<-- asserted/unexpected");
  }

  Serial.println("\n==================================================================");
  Serial.println(" SUMMARY");
  Serial.println("==================================================================");
  for (int i = 0; i < NDEV; i++)
    Serial.printf("  bus%d 0x%02X %-4s %-30s %s\n", DEVS[i].bus, DEVS[i].addr, DEVS[i].ref, DEVS[i].name, status[i]);
  Serial.printf("\n  %d/%d expected devices OK\n", okCount, NDEV);
  Serial.println("\n(send any character to run again)");
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) delay(10);
  delay(1000);
  runTest();
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    runTest();
  }
  delay(20);
}
