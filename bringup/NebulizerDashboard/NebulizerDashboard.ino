/*
  Nebulizer Control Board V1 - live I2C monitor firmware (pairs with dashboard.html)

  Every ~1 s it re-scans both I2C buses, re-reads every device that ACKs, and prints ONE JSON line
  over USB serial. Devices can be unplugged / replugged at any time; they simply show as offline/online.

  Arduino IDE: Board ESP32S3 Dev Module, USB CDC On Boot = Enabled, Flash 16MB, PSRAM QSPI.
  Close the Arduino Serial Monitor before using the dashboard (only one program can hold the port).
  Drives NO outputs unless ENABLE_FUSE is true.
*/
#include <Wire.h>
#include <math.h>

// Serial must be the native USB port. Otherwise it is UART0 = GPIO43/44 = FUSE_EN/FUSE_PG on this board!
#if !ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot > Enabled, then upload again."
#endif

struct Reading;   // forward declaration (Arduino auto-prototypes need it before the first function)
static const uint32_t I2C_FREQ = 100000;
static const uint32_t CYCLE_MS = 1000;
static const float VDD = 3.3f;
static const bool ENABLE_FUSE = false;     // true = drive FUSE_EN (GPIO43) high so INAs see VBAT_PROTECTED

static const uint8_t FLOW_START_CMD[2] = {0x36, 0x08};  // 0x3608 water, 0x3616 isopropanol
static const float FLOW_SCALE = 500.0f, FLOW_TEMP_SCALE = 200.0f;   // adjust to fitted SLF3x
static const float P_MIN = 0.0f, P_MAX = 150.0f;
static const int OUT_MIN = 1638, OUT_MAX = 14746;

static const int PIN_FUSE_EN = 43, PIN_FUSE_PG = 44, PIN_FUSE_FLT = 4, PIN_DRV_NFAULT = 8;
static const int BUS_SDA[2] = {13, 1}, BUS_SCL[2] = {12, 2};

// ---------------------------------------------------------------- heater outputs (LTC7001 IN pins, active HIGH)
// Order: 0 REG (J2.3), 1 NOZZLE (J2.1), 2 EXTRA1 (J2.8), 3 EXTRA2 (J2.12), 4 EXTRA3 (J2.10), 5 EXTRA4 (J2.5)
static const int NHEAT = 6;
static const int HEATER_PIN[NHEAT] = {42, 36, 37, 11, 47, 14};
static const uint32_t MAX_ON_MS = 10000;    // a heater turns itself off this long after the last ON command
static const uint32_t DEADMAN_MS = 3000;    // ALL heaters off if no command/PING arrives for this long
static bool heaterOn[NHEAT] = {};
static uint32_t heaterOffAt[NHEAT] = {};
static uint32_t lastPing = 0;
static bool fuseInhibit = false;            // true = FUSE_EN (GPIO43) driven LOW; false = released (hi-Z, board divider decides)

static void setHeater(int i, bool on) {
  heaterOn[i] = on;
  digitalWrite(HEATER_PIN[i], on ? HIGH : LOW);
  if (on) heaterOffAt[i] = millis() + MAX_ON_MS;
}
static void allOff() { for (int i = 0; i < NHEAT; i++) setHeater(i, false); }
static void serviceSafety() {
  bool any = false;
  for (int i = 0; i < NHEAT; i++) {
    if (!heaterOn[i]) continue;
    any = true;
    if ((int32_t)(millis() - heaterOffAt[i]) >= 0) setHeater(i, false);
  }
  if (any && millis() - lastPing > DEADMAN_MS) allOff();
}
// Commands (one per line): PING | OFF | H,<0-5>,<0|1> | FUSE,<0|1>  (FUSE,1 = inhibit e-fuse)
static void handleLine(char *s) {
  lastPing = millis();
  if (!strcmp(s, "OFF")) { allOff(); return; }
  int i, v;
  if (sscanf(s, "H,%d,%d", &i, &v) == 2 && i >= 0 && i < NHEAT) { setHeater(i, v == 1); return; }
  if (sscanf(s, "FUSE,%d", &v) == 1) {
    fuseInhibit = (v == 1);
    if (fuseInhibit) { pinMode(PIN_FUSE_EN, OUTPUT); digitalWrite(PIN_FUSE_EN, LOW); }
    else pinMode(PIN_FUSE_EN, INPUT);
  }
}
static void handleSerial() {
  static char buf[24]; static int n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (n) { buf[n] = 0; handleLine(buf); } n = 0; }
    else if (n < (int)sizeof(buf) - 1) buf[n++] = c;
  }
}

static const char *ADS_NAMES[8] = {
  "CH0 REG_THERM", "CH1 NOZZLE_THERM", "CH2 EXTRA_THERM1", "CH3 EXTRA_THERM2",
  "CH4 EXTRA_THERM3", "CH5 EXTRA_THERM4", "CH6 FUSE_IMON", "CH7 FUSE_TEMP"
};
static const uint8_t ADS_SEL[8] = {0b000, 0b100, 0b001, 0b101, 0b010, 0b110, 0b011, 0b111};

// ---------------------------------------------------------------- reading container
struct Reading {
  int n = 0;
  const char *k[10];
  float v[10];
  const char *note = "";
  void add(const char *key, float val) { if (n < 10) { k[n] = key; v[n] = val; n++; } }
};

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
  w.beginTransmission(a); w.write(reg);
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
  w.beginTransmission(a); w.write(c, n);
  return w.endTransmission() == 0;
}

// ---------------------------------------------------------------- probes: return true = data valid
static bool probeSHT45(TwoWire &w, uint8_t a, Reading &r) {
  uint8_t d[6]; const uint8_t c[1] = {0xFD};
  if (!writeCmd(w, a, c, 1)) return false;
  delay(12);
  if (!readBytes(w, a, 6, d)) return false;
  if (crc8(d, 2) != d[2] || crc8(d + 3, 2) != d[5]) { r.note = "CRC error"; return false; }
  r.add("Temperature (C)", -45.0f + 175.0f * (((d[0] << 8) | d[1]) / 65535.0f));
  r.add("Humidity (%RH)", constrain(-6.0f + 125.0f * (((d[3] << 8) | d[4]) / 65535.0f), 0.0f, 100.0f));
  return true;
}
static bool probeEEPROM(TwoWire &w, uint8_t a, Reading &r) {
  uint8_t id[6];
  if (!readReg(w, a, 0xFA, 6, id)) return false;
  r.add("Mfr code (0x29 ok)", id[0]);
  r.add("Device code (0x41 ok)", id[1]);
  r.add("UID (32-bit, dec)", (float)(((uint32_t)id[2] << 24) | (id[3] << 16) | (id[4] << 8) | id[5]));
  if (!(id[0] == 0x29 && id[1] == 0x41)) { r.note = "unexpected ID"; return false; }
  return true;
}
static bool probeINA740(TwoWire &w, uint8_t a, Reading &r) {
  uint16_t mfr, vb, dt, cur;
  if (!readReg16(w, a, 0x3E, mfr) || !readReg16(w, a, 0x05, vb) ||
      !readReg16(w, a, 0x06, dt) || !readReg16(w, a, 0x07, cur)) return false;
  r.add("Bus voltage (V)", vb * 0.003125f);
  r.add("Current (mA)", (int16_t)cur * 1.2f);
  r.add("Die temp (C)", ((int16_t)dt / 16) * 0.125f);
  if (mfr != 0x5449) { r.note = "bad MFR_ID"; return false; }
  if (vb * 0.003125f < 1.0f) r.note = "VBUS~0V: e-fuse off?";
  return true;
}
static bool probeSSC(TwoWire &w, uint8_t a, Reading &r) {
  uint8_t d[4];
  w.beginTransmission(a);
  if (w.endTransmission() != 0) return false;
  delay(10);
  if (!readBytes(w, a, 4, d)) return false;
  int status = d[0] >> 6, praw = ((d[0] & 0x3F) << 8) | d[1], traw = (d[2] << 3) | (d[3] >> 5);
  r.add("Pressure (psi)", (praw - OUT_MIN) * (P_MAX - P_MIN) / (OUT_MAX - OUT_MIN) + P_MIN);
  r.add("Temperature (C)", traw * 200.0f / 2047.0f - 50.0f);
  r.add("Status (0=ok)", status);
  if (status == 3) r.note = "diagnostic fault";
  return status == 0 || status == 2;
}
static bool probeADS7828(TwoWire &w, uint8_t a, Reading &r) {
  for (int ch = 0; ch < 8; ch++) {
    uint8_t cmd = 0x80 | (ADS_SEL[ch] << 4) | (0b01 << 2);   // internal ref OFF (REF tied to VDD)
    uint8_t d[2];
    if (!writeCmd(w, a, &cmd, 1) || !readBytes(w, a, 2, d)) return false;
    if (!writeCmd(w, a, &cmd, 1) || !readBytes(w, a, 2, d)) return false;
    r.add(ADS_NAMES[ch], (((d[0] << 8) | d[1]) & 0x0FFF) * VDD / 4096.0f);   // volts
  }
  return true;
}
static bool probeFlow(TwoWire &w, uint8_t a, Reading &r) {
  uint8_t d[9]; const uint8_t stop[2] = {0x3F, 0xF9};
  if (!writeCmd(w, a, FLOW_START_CMD, 2)) return false;
  delay(50); readBytes(w, a, 9, d); delay(20);
  bool got = readBytes(w, a, 9, d);
  writeCmd(w, a, stop, 2);
  if (!got) return false;
  if (crc8(d, 2) != d[2] || crc8(d + 3, 2) != d[5]) { r.note = "CRC error"; return false; }
  r.add("Flow (ml/min)", (int16_t)((d[0] << 8) | d[1]) / FLOW_SCALE);
  r.add("Temperature (C)", (int16_t)((d[3] << 8) | d[4]) / FLOW_TEMP_SCALE);
  return true;
}

// ---------------------------------------------------------------- device table
typedef bool (*ProbeFn)(TwoWire &, uint8_t, Reading &);
struct Dev { uint8_t bus, addr; const char *ref, *name; ProbeFn probe; };
static const Dev DEVS[] = {
  {0, 0x28, "U14", "Pressure sensor", probeSSC},
  {0, 0x40, "U9",  "INA740 REG heater", probeINA740},
  {0, 0x41, "U12", "INA740 NOZZLE heater", probeINA740},
  {0, 0x44, "U4",  "SHT45 temp/RH", probeSHT45},
  {0, 0x45, "U16", "INA740 EXTRA heater 4", probeINA740},
  {0, 0x48, "U11", "ADS7828 ADC mux", probeADS7828},
  {0, 0x50, "U5",  "24AA02UID EEPROM", probeEEPROM},
  {1, 0x08, "J6",  "Flow sensor", probeFlow},
  {1, 0x28, "U17", "Pressure sensor", probeSSC},
  {1, 0x40, "U15", "INA740 EXTRA heater 1", probeINA740},
  {1, 0x41, "U10", "INA740 EXTRA heater 2", probeINA740},
  {1, 0x44, "U13", "INA740 EXTRA heater 3", probeINA740},
};
static const int NDEV = sizeof(DEVS) / sizeof(DEVS[0]);

struct Gp { const char *label; int pin; };
static const Gp GPIOS[] = {
  {"DRV nFAULT", PIN_DRV_NFAULT}, {"FUSE_FLT", PIN_FUSE_FLT}, {"FUSE_PG", PIN_FUSE_PG},
  {"ALERT REG", 41}, {"ALERT NOZZLE", 35}, {"ALERT EXTRA1", 38},
  {"ALERT EXTRA2", 10}, {"ALERT EXTRA3", 48}, {"ALERT EXTRA4", 21},
};

static TwoWire &busRef(int b) { return b == 0 ? Wire : Wire1; }

static void busInit(int b) {
  TwoWire &w = busRef(b);
  w.end();
  w.begin(BUS_SDA[b], BUS_SCL[b], I2C_FREQ);
  w.setTimeOut(50);
}

static void jstr(const char *s) {            // JSON string (inputs are plain ASCII constants)
  Serial.print('"'); Serial.print(s); Serial.print('"');
}

static void cycle() {
  bool found[2][128] = {};
  int count[2] = {0, 0};
  for (int b = 0; b < 2; b++) {
    TwoWire &w = busRef(b);
    for (uint8_t a = 1; a < 127; a++) {
      w.beginTransmission(a);
      if (w.endTransmission() == 0) { found[b][a] = true; count[b]++; }
    }
  }
  bool stuck[2];
  for (int b = 0; b < 2; b++) {              // detect a hung bus (line held low) and recover
    stuck[b] = false;
    if (count[b] == 0) {
      busRef(b).end();
      pinMode(BUS_SDA[b], INPUT); pinMode(BUS_SCL[b], INPUT);
      stuck[b] = !digitalRead(BUS_SDA[b]) || !digitalRead(BUS_SCL[b]);
      busInit(b);
    }
  }

  Serial.printf("{\"t\":%lu,\"stuck\":[%s,%s],\"devs\":[", (unsigned long)millis(),
                stuck[0] ? "true" : "false", stuck[1] ? "true" : "false");
  for (int i = 0; i < NDEV; i++) {
    serviceSafety();
    const Dev &d = DEVS[i];
    Reading r;
    bool live = found[d.bus][d.addr];
    bool ok = false;
    if (live) ok = d.probe(busRef(d.bus), d.addr, r);
    if (i) Serial.print(',');
    Serial.printf("{\"bus\":%d,\"addr\":%d,\"ref\":", d.bus, d.addr); jstr(d.ref);
    Serial.print(",\"name\":"); jstr(d.name);
    Serial.printf(",\"live\":%s,\"ok\":%s,\"note\":", live ? "true" : "false", ok ? "true" : "false");
    jstr(r.note);
    Serial.print(",\"vals\":[");
    for (int j = 0; j < r.n; j++) {
      if (j) Serial.print(',');
      Serial.print("["); jstr(r.k[j]);
      if (isfinite(r.v[j])) Serial.printf(",%.4f]", r.v[j]); else Serial.print(",null]");
    }
    Serial.print("]}");
  }
  Serial.print("],\"unexpected\":[");
  bool first = true;
  for (int b = 0; b < 2; b++)
    for (int a = 1; a < 127; a++) {
      if (!found[b][a]) continue;
      bool exp = (b == 0 && a >= 0x51 && a <= 0x57);   // 24AA02UID answers 0x50-0x57 (no address pins)
      for (int i = 0; i < NDEV; i++) if (DEVS[i].bus == b && DEVS[i].addr == a) exp = true;
      if (!exp) { if (!first) Serial.print(','); Serial.printf("[%d,%d]", b, a); first = false; }
    }
  Serial.print("],\"gpio\":[");
  for (size_t i = 0; i < sizeof(GPIOS) / sizeof(GPIOS[0]); i++) {
    if (i) Serial.print(',');
    Serial.print("["); jstr(GPIOS[i].label); Serial.printf(",%d]", digitalRead(GPIOS[i].pin));
  }
  Serial.print("],\"out\":{\"h\":[");
  for (int i = 0; i < NHEAT; i++) Serial.printf("%s%d", i ? "," : "", heaterOn[i] ? 1 : 0);
  Serial.print("],\"left\":[");
  for (int i = 0; i < NHEAT; i++) {
    int32_t l = heaterOn[i] ? (int32_t)(heaterOffAt[i] - millis()) : 0;
    Serial.printf("%s%d", i ? "," : "", l < 0 ? 0 : (int)l);
  }
  Serial.printf("],\"inhibit\":%d,\"maxon\":%lu}}\n", fuseInhibit ? 1 : 0, (unsigned long)MAX_ON_MS);
}

void setup() {
  for (int i = 0; i < NHEAT; i++) { digitalWrite(HEATER_PIN[i], LOW); pinMode(HEATER_PIN[i], OUTPUT); digitalWrite(HEATER_PIN[i], LOW); }
  Serial.begin(115200);
  for (const Gp &g : GPIOS) pinMode(g.pin, INPUT);
  if (ENABLE_FUSE) { pinMode(PIN_FUSE_EN, OUTPUT); digitalWrite(PIN_FUSE_EN, HIGH); delay(300); }
  busInit(0); busInit(1);
  delay(500);
}

void loop() {
  uint32_t t0 = millis();
  cycle();
  while (millis() - t0 < CYCLE_MS) { handleSerial(); serviceSafety(); delay(5); }
}
