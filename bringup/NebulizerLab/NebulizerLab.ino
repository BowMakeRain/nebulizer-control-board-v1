/*
  Nebulizer PCB V1 Test GUI - firmware (ESP32-S3, Arduino core 3.x)

  * 6 heater channels, each: PWM at a settable frequency, OFF / MANUAL duty / PID on its own thermistor.
  * PID runs on the board at 10 Hz (duty = control signal). Thermistors are read from the ADS7828.
  * ~4 Hz telemetry: one JSON line over USB serial with everything (all I2C devices, heater state,
    thermistors, GPIO status, system stats).
  * Safety: all outputs boot LOW; dead-man (all heaters OFF if no command/PING for 3 s); latched over-temperature
    trip; PID thermistor-fault trip; per-heater max duty and max temperature.

  Heater -> thermistor (ADS7828 channel) -> power monitor
    reg    GPIO42  CH0 REG_THERM    INA U9  (bus0 0x40)   J2.3
    nozzle GPIO36  CH1 NOZZLE_THERM INA U12 (bus0 0x41)   J2.1
    extra1 GPIO37  CH2 EXTRA_THERM1 INA U15 (bus1 0x40)   J2.8
    extra2 GPIO11  CH3 EXTRA_THERM2 INA U10 (bus1 0x41)   J2.12
    extra3 GPIO47  CH4 EXTRA_THERM3 INA U13 (bus1 0x44)   J2.10
    extra4 GPIO14  CH5 EXTRA_THERM4 INA U16 (bus0 0x45)   J2.5

  Commands (one per line): PING | OFF | CA | FUSE,<0|1> | TH,<beta>,<r0>
    M,<i>,<0 off|1 manual|2 pid>   D,<i>,<duty %>   S,<i>,<setpoint C>   G,<i>,<kp>,<ki>,<kd>
    F,<i>,<pwm Hz>   X,<i>,<max duty %>,<max temp C>   C,<i> (clear trip)   CA (clear all)
    CAN,<kbps>[,<mode>]  start CAN (kbps 0 = stop; mode 0 normal, 1 listen-only, 2 self-test)
    CT,<ms> thermistor/PID loop period (5-1000, default 20)   I2C,<kHz> (50-1700)   TR,<ms> telemetry min period (0 = as fast as possible)
    SOL,<0|1>  solenoid (GPIO9, active-high low-side switch; also closed by OFF / dead-man)
    Motor (DRV8434A U6): MP,<0|1> nSLEEP  ME,<0 off|1 on|2 on Hi-Z x8>  MD,<dir>  ML,<0 torque|1 thresh|2 learn>
       MR,<1 stall report off (STL_REP held low) | 0 on>  MS,<step Hz> (0 stop)  MA,<accel Hz/s>  MC clear fault. Boot: asleep, disabled, STEP low.
    CANT,<idhex>,<datahex>  send a standard frame (up to 8 bytes, e.g. CANT,123,DEADBEEF)
  CAN: TJA1051 U3, CAN_TX = GPIO40 (U2.33), CAN_RX = GPIO39 (U2.32), S pin tied to GND (always normal mode),
       R63 (120 ohm CANH-CANL) is DNI: board has NO termination. Boot default: 500 kbps, normal mode.

  Arduino IDE / arduino-cli: ESP32S3 Dev Module, USB CDC On Boot = Enabled (required).
*/
#include <Wire.h>
#include <math.h>
#include <stdarg.h>
#include "driver/twai.h"
#include "esp_system.h"

#if !ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot > Enabled (otherwise Serial = UART0 = GPIO43/44 = FUSE_EN/FUSE_PG)."
#endif
// FUSE_EN is driven HIGH at boot (e-fuse on). FUSE,1 cuts power (LOW), FUSE,0 turns it back on (HIGH).

struct Reading;   // forward declaration for Arduino auto-prototypes

// ---------------------------------------------------------------- configuration
static uint32_t i2cHz = 400000;            // I2C clock, set with I2C,<kHz>
static uint32_t telemMs = 0;               // min time between telemetry frames; 0 = free-run (as fast as the buses allow), TR,<ms>
static uint32_t ctlMs = 20;                // thermistor read + PID period (ms), CT,<ms>; 20 = 50 Hz
static const uint32_t SCAN_MS = 2000;      // full-bus scan for unexpected addresses
static const uint32_t DEADMAN_MS = 3000;   // heaters off if no command/PING this long
static const uint8_t PWM_BITS = 10;
static const float RPULL = 10000.0f;       // thermistor pull-up to 3V3 (R37,R39,R40,R43,R44,R45)
static float thermBeta = 3950.0f, thermR0 = 10000.0f;   // NTC 10k/B3950 assumed; change with TH command

static const uint8_t FLOW_START_CMD[2] = {0x36, 0x08};  // 0x3608 water, 0x3616 isopropanol
static const float FLOW_SCALE = 500.0f, FLOW_TEMP_SCALE = 200.0f;
static const float P_MIN = 0.0f, P_MAX = 150.0f;         // SSC 0..150 psi gauge, 10-90% of 14 bit
static const int OUT_MIN = 1638, OUT_MAX = 14746;

static const int PIN_FUSE_EN = 43, PIN_FUSE_PG = 44, PIN_FUSE_FLT = 4, PIN_DRV_NFAULT = 8;
static const int BUS_SDA[2] = {13, 1}, BUS_SCL[2] = {12, 2};

static const uint8_t ADS_ADDR = 0x48;
static const uint8_t ADS_SEL[8] = {0b000, 0b100, 0b001, 0b101, 0b010, 0b110, 0b011, 0b111};
static const char *ADS_NAMES[8] = {
  "CH0 REG_THERM", "CH1 NOZZLE_THERM", "CH2 EXTRA_THERM1", "CH3 EXTRA_THERM2",
  "CH4 EXTRA_THERM3", "CH5 EXTRA_THERM4", "CH6 FUSE_IMON", "CH7 FUSE_TEMP"
};

// ---------------------------------------------------------------- heaters
struct Heater {
  const char *key; int pin; int therm;
  uint8_t mode = 0;          // 0 off, 1 manual, 2 pid
  uint8_t trip = 0;          // 0 none, 1 over-temp, 2 sensor fault (pid), 3 lost host (dead-man)
  float sp = 40, kp = 3, ki = 0.3f, kd = 0, man = 0, maxDuty = 30, maxT = 100;
  uint32_t freq = 200;
  float duty = 0;            // applied duty, %
  float T = NAN, R = NAN; int raw = -1; bool tok = false, tvalid = false;
  float iTerm = 0, dFilt = 0, lastT = 0, p = 0, i = 0, d = 0;
};
static const int NHEAT = 6;
static Heater H[NHEAT] = {
  {"reg", 42, 0}, {"nozzle", 36, 1}, {"extra1", 37, 2}, {"extra2", 11, 3}, {"extra3", 47, 4}, {"extra4", 14, 5},
};
static uint32_t lastPing = 0, lastCtl = 0, ctlMaxMs = 0;
static bool fuseInhibit = false;

static void applyPwm(Heater &h) {
  uint32_t maxv = (1u << PWM_BITS) - 1;
  float d = constrain(h.duty, 0.0f, 100.0f);
  ledcWrite(h.pin, (uint32_t)lroundf(d / 100.0f * maxv));
}
static void setMode(int i, int m) {
  Heater &h = H[i];
  if (m != 0 && (h.trip == 1 || h.trip == 2)) return;     // latched trips need an explicit clear
  if (h.trip == 3) h.trip = 0;
  if (m == 2 && h.mode != 2) { h.iTerm = 0; h.dFilt = 0; h.lastT = h.T; }
  h.mode = m;
  if (m == 0) { h.duty = 0; h.p = h.i = h.d = 0; applyPwm(h); }
}
// ---------------------------------------------------------------- stepper driver U6 (DRV8434A)
// MCU nets: nSLEEP=GPIO5 ENABLE=GPIO6 DIR=GPIO7 STEP=GPIO15 STL_MODE=GPIO16 TRQ_CNT=GPIO17(analog in)
//           STL_REP=GPIO18(in) nFAULT=GPIO8(in).  M0, M1, VREF are set by resistors on the board (not MCU-controlled).
static const int PIN_M_SLEEP = 5, PIN_M_EN = 6, PIN_M_DIR = 7, PIN_M_STEP = 15, PIN_M_STLMODE = 16, PIN_M_TRQ = 17, PIN_M_STLREP = 18;
struct Motor {
  int sleepHi = 0;          // nSLEEP: 1 = awake
  int en = 0;               // ENABLE: 0 low (outputs off), 1 high (on), 2 Hi-Z (on, torque/stall scale x8)
  int dir = 0;
  int stlDis = 1;           // 1 = hold STL_REP low (stall reporting off, nFAULT won't assert on a stall); 0 = release (stall detect on)
  int stl = 0;              // STL_MODE: 0 torque count (low), 1 stall threshold (high), 2 learning (Hi-Z)
  float target = 0;         // requested STEP frequency, Hz
  float cur = 0;            // actual (ramped) STEP frequency, Hz
  float accel = 2000;       // Hz per second
  uint32_t realHz = 0;
  bool stepAttached = false;
} M;
static void motorApplyPins() {
  pinMode(PIN_M_SLEEP, OUTPUT); digitalWrite(PIN_M_SLEEP, M.sleepHi ? HIGH : LOW);
  if (M.en == 2) pinMode(PIN_M_EN, INPUT); else { pinMode(PIN_M_EN, OUTPUT); digitalWrite(PIN_M_EN, M.en ? HIGH : LOW); }
  pinMode(PIN_M_DIR, OUTPUT); digitalWrite(PIN_M_DIR, M.dir ? HIGH : LOW);
  if (M.stl == 2) pinMode(PIN_M_STLMODE, INPUT); else { pinMode(PIN_M_STLMODE, OUTPUT); digitalWrite(PIN_M_STLMODE, M.stl ? HIGH : LOW); }
  if (M.stlDis) { pinMode(PIN_M_STLREP, OUTPUT); digitalWrite(PIN_M_STLREP, LOW); } else pinMode(PIN_M_STLREP, INPUT);
}
static void motorStepOut(float hz) {                 // STEP = 50 % square wave via LEDC (8 bit)
  if (hz < 1) {
    if (M.stepAttached) { ledcDetach(PIN_M_STEP); M.stepAttached = false; }
    pinMode(PIN_M_STEP, OUTPUT); digitalWrite(PIN_M_STEP, LOW); M.realHz = 0; return;
  }
  uint32_t f = (uint32_t)(hz > 100000 ? 100000 : hz);
  if (!M.stepAttached) { ledcAttach(PIN_M_STEP, f, 8); M.stepAttached = true; } else ledcChangeFrequency(PIN_M_STEP, f, 8);
  ledcWrite(PIN_M_STEP, 128);
  M.realHz = ledcReadFreq(PIN_M_STEP);
}
static void motorStop() { M.target = 0; M.cur = 0; motorStepOut(0); }
static void motorTick(float dt, bool deadman) {
  if (deadman && (M.target > 0 || M.cur > 0)) motorStop();
  if (M.cur == M.target) return;
  float d = M.accel * dt;
  if (M.target > M.cur) M.cur = min(M.target, M.cur + d); else M.cur = max(M.target, M.cur - d);
  motorStepOut(M.cur);
}
static void motorInit() { motorApplyPins(); pinMode(PIN_M_STEP, OUTPUT); digitalWrite(PIN_M_STEP, LOW); }

static const int PIN_SOL = 9;                         // SOL_TRIG (U2.17) -> R58 -> low-side switch Q7, active HIGH
static int solOn = 0;
static uint32_t cycN = 0, ctlN = 0, thermOk[6] = {0, 0, 0, 0, 0, 0};   // sample counters for the rate table
static void solSet(int v) { solOn = v ? 1 : 0; digitalWrite(PIN_SOL, solOn); }
static void allOff() { for (int i = 0; i < NHEAT; i++) setMode(i, 0); motorStop(); solSet(0); }

// ---------------------------------------------------------------- I2C helpers
static TwoWire &busRef(int b) { return b == 0 ? Wire : Wire1; }
static void busInit(int b) {
  TwoWire &w = busRef(b);
  w.end();
  w.begin(BUS_SDA[b], BUS_SCL[b], i2cHz);
  w.setTimeOut(50);
}
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

static bool adsRead(int ch, int &raw) {            // PD=01: internal ref off (REF tied to VDD); 2 reads for settling
  uint8_t cmd = 0x80 | (ADS_SEL[ch] << 4) | (0b01 << 2), d[2];
  for (int k = 0; k < 2; k++) {
    if (!writeCmd(Wire, ADS_ADDR, &cmd, 1) || !readBytes(Wire, ADS_ADDR, 2, d)) return false;
  }
  raw = ((d[0] << 8) | d[1]) & 0x0FFF;
  return true;
}


// ---------------------------------------------------------------- CAN (TWAI -> TJA1051)
static const int PIN_CAN_TX = 40, PIN_CAN_RX = 39;
struct CanFrame { uint32_t ms; uint32_t id; uint8_t len; uint8_t d[8]; uint8_t dir; };   // dir 0 rx, 1 tx
static CanFrame canLog[8]; static int canHead = 0;
static bool canOn = false; static int canKbps = 500, canMode = 0;
static uint32_t canRx = 0, canTx = 0, canTxFail = 0;

static void canLogAdd(const twai_message_t &m, uint8_t dir) {
  CanFrame &f = canLog[canHead]; canHead = (canHead + 1) % 8;
  f.ms = millis(); f.id = m.identifier; f.len = m.data_length_code > 8 ? 8 : m.data_length_code; f.dir = dir;
  memcpy(f.d, m.data, 8);
}
static void canStop() {
  if (canOn) { twai_stop(); twai_driver_uninstall(); }
  canOn = false;
}
static bool canStart(int kbps, int mode) {
  canStop();
  twai_timing_config_t t;
  switch (kbps) {
    case 25:   t = TWAI_TIMING_CONFIG_25KBITS(); break;
    case 50:   t = TWAI_TIMING_CONFIG_50KBITS(); break;
    case 100:  t = TWAI_TIMING_CONFIG_100KBITS(); break;
    case 125:  t = TWAI_TIMING_CONFIG_125KBITS(); break;
    case 250:  t = TWAI_TIMING_CONFIG_250KBITS(); break;
    case 500:  t = TWAI_TIMING_CONFIG_500KBITS(); break;
    case 800:  t = TWAI_TIMING_CONFIG_800KBITS(); break;
    case 1000: t = TWAI_TIMING_CONFIG_1MBITS(); break;
    default: return false;
  }
  twai_mode_t tm = mode == 1 ? TWAI_MODE_LISTEN_ONLY : (mode == 2 ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL);
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, tm);
  g.tx_queue_len = 8; g.rx_queue_len = 16;
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
  canOn = true; canKbps = kbps; canMode = mode;
  return true;
}
static void canSend(uint32_t id, const uint8_t *d, int n) {
  if (!canOn || canMode == 1) { canTxFail++; return; }
  twai_message_t m = {}; m.identifier = id & 0x7FF; m.data_length_code = n;
  if (canMode == 2) m.self = 1;                      // self-test: receive our own frame (no ACK needed)
  memcpy(m.data, d, n);
  esp_err_t er = twai_transmit(&m, 0);
  if (er == ESP_OK) { canTx++; canLogAdd(m, 1); } else canTxFail++;
}
static void canPoll() {                              // call often: drain RX queue
  if (!canOn) return;
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {
    if (m.rtr) continue;
    canRx++; canLogAdd(m, 0);
  }
}
static void canHandleCmd(const char *s) {            // "CANT,123,DEADBEEF"
  unsigned id; char hex[24] = {0};
  if (sscanf(s, "CANT,%x,%16s", &id, hex) >= 1) {
    uint8_t d[8]; int n = 0;
    for (int k = 0; hex[k] && hex[k + 1] && n < 8; k += 2) {
      char b[3] = {hex[k], hex[k + 1], 0}; d[n++] = (uint8_t)strtoul(b, nullptr, 16);
    }
    canSend(id, d, n);
  }
}

// ---------------------------------------------------------------- serial commands
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void handleLine(char *s) {
  lastPing = millis();
  int i, m, v; float a, b, c, d;
  if (!strcmp(s, "OFF")) { allOff(); return; }
  if (sscanf(s, "I2C,%d", &v) == 1 && v >= 50 && v <= 1700) { i2cHz = (uint32_t)v * 1000; busInit(0); busInit(1); return; }
  if (sscanf(s, "CT,%d", &v) == 1 && v >= 5 && v <= 1000) { ctlMs = (uint32_t)v; return; }
  if (sscanf(s, "TR,%d", &v) == 1 && v >= 0 && v <= 5000) { telemMs = (uint32_t)v; return; }
  if (sscanf(s, "SOL,%d", &v) == 1) { solSet(v); return; }
  if (sscanf(s, "MR,%d", &v) == 1) { M.stlDis = v ? 1 : 0; motorApplyPins(); return; }
  if (sscanf(s, "MP,%d", &v) == 1) { M.sleepHi = v ? 1 : 0; if (!M.sleepHi) motorStop(); motorApplyPins(); return; }
  if (sscanf(s, "ME,%d", &v) == 1 && v >= 0 && v <= 2) { M.en = v; if (!v) motorStop(); motorApplyPins(); return; }
  if (sscanf(s, "MD,%d", &v) == 1) { M.dir = v ? 1 : 0; motorApplyPins(); return; }
  if (sscanf(s, "ML,%d", &v) == 1 && v >= 0 && v <= 2) { M.stl = v; motorApplyPins(); return; }
  if (sscanf(s, "MS,%f", &a) == 1) { M.target = clampf(a, 0, 100000); return; }
  if (sscanf(s, "MA,%f", &a) == 1) { M.accel = clampf(a, 10, 1000000); return; }
  if (!strcmp(s, "MC")) { digitalWrite(PIN_M_SLEEP, LOW); delay(5); digitalWrite(PIN_M_SLEEP, M.sleepHi ? HIGH : LOW); return; }   // nSLEEP low pulse clears faults
  if (!strncmp(s, "CANT,", 5)) { canHandleCmd(s); return; }
  { int kb, md = 0; int n = sscanf(s, "CAN,%d,%d", &kb, &md);
    if (n >= 1) { if (kb == 0) canStop(); else canStart(kb, md < 0 || md > 2 ? 0 : md); return; } }
  if (!strcmp(s, "CA")) { for (auto &h : H) if (h.trip) h.trip = 0; return; }
  if (sscanf(s, "M,%d,%d", &i, &m) == 2 && i >= 0 && i < NHEAT && m >= 0 && m <= 2) { setMode(i, m); return; }
  if (sscanf(s, "D,%d,%f", &i, &a) == 2 && i >= 0 && i < NHEAT) { H[i].man = clampf(a, 0, 100); return; }
  if (sscanf(s, "S,%d,%f", &i, &a) == 2 && i >= 0 && i < NHEAT) { H[i].sp = clampf(a, -20, 150); return; }
  if (sscanf(s, "G,%d,%f,%f,%f", &i, &a, &b, &c) == 4 && i >= 0 && i < NHEAT) {
    H[i].kp = clampf(a, 0, 1000); H[i].ki = clampf(b, 0, 1000); H[i].kd = clampf(c, 0, 1000); return;
  }
  if (sscanf(s, "F,%d,%f", &i, &a) == 2 && i >= 0 && i < NHEAT) {
    ledcChangeFrequency(H[i].pin, (uint32_t)clampf(a, 1, 20000), PWM_BITS);
    // LEDC timers are shared between channel pairs: read back the real frequency of every heater
    for (auto &h : H) { h.freq = ledcReadFreq(h.pin); applyPwm(h); }
    return;
  }
  if (sscanf(s, "X,%d,%f,%f", &i, &a, &b) == 3 && i >= 0 && i < NHEAT) {
    H[i].maxDuty = clampf(a, 0, 100); H[i].maxT = clampf(b, 0, 150); return;
  }
  if (sscanf(s, "C,%d", &i) == 1 && i >= 0 && i < NHEAT) { H[i].trip = 0; return; }
  if (sscanf(s, "TH,%f,%f", &a, &b) == 2) { thermBeta = clampf(a, 1000, 10000); thermR0 = clampf(b, 100, 1000000); return; }
  if (sscanf(s, "FUSE,%d", &v) == 1) {
    fuseInhibit = (v == 1);                        // 1 = cut power (FUSE_EN low), 0 = power on (FUSE_EN driven high)
    pinMode(PIN_FUSE_EN, OUTPUT); digitalWrite(PIN_FUSE_EN, fuseInhibit ? LOW : HIGH);
    return;
  }
  (void)d;
}
static void handleSerial() {
  static char buf[64]; static int n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (n) { buf[n] = 0; handleLine(buf); } n = 0; }
    else if (n < (int)sizeof(buf) - 1) buf[n++] = c;
  }
}

// ---------------------------------------------------------------- control loop (10 Hz)
static bool inTick = false;
static void controlTick() {
  uint32_t now = millis();
  if (inTick || now - lastCtl < ctlMs) return;
  inTick = true;
  float dt = lastCtl ? (now - lastCtl) / 1000.0f : ctlMs / 1000.0f;
  if (lastCtl && now - lastCtl > ctlMaxMs) ctlMaxMs = now - lastCtl;
  lastCtl = now; ctlN++;

  bool anyOn = false;
  for (auto &h : H) if (h.mode) anyOn = true;
  bool deadman = anyOn && (now - lastPing > DEADMAN_MS);
  motorTick(dt, now - lastPing > DEADMAN_MS);
  if (solOn && now - lastPing > DEADMAN_MS) solSet(0);

  for (int i = 0; i < NHEAT; i++) {
    Heater &h = H[i];
    int raw;
    bool ok = adsRead(h.therm, raw);
    if (ok) thermOk[i]++;
    h.tok = ok && raw >= 5 && raw <= 4090;               // outside = open / shorted thermistor
    h.raw = ok ? raw : -1;
    if (h.tok) {
      h.R = RPULL * raw / (4096.0f - raw);               // ratiometric (REF = VDD)
      float Tn = 1.0f / (1.0f / 298.15f + logf(h.R / thermR0) / thermBeta) - 273.15f;
      h.T = h.tvalid ? h.T + (1.0f - expf(-dt / 0.144f)) * (Tn - h.T) : Tn;   // low-pass, tau 0.144 s (independent of loop rate)
      h.tvalid = true;
    } else { h.tvalid = false; }

    if (deadman && h.mode) { h.mode = 0; h.trip = 3; }
    if (h.tok && h.T > h.maxT && h.mode) { h.mode = 0; h.trip = 1; }
    if (h.mode == 2 && !h.tok) { h.mode = 0; h.trip = 2; }

    float out = 0;
    if (h.mode == 1) {
      out = h.man; h.p = h.i = h.d = 0;
    } else if (h.mode == 2) {
      float err = h.sp - h.T;
      h.p = h.kp * err;
      h.iTerm = clampf(h.iTerm + h.ki * err * dt, 0, h.maxDuty);   // clamped integrator = anti-windup
      float dm = (h.T - h.lastT) / dt;
      h.dFilt += (1.0f - expf(-dt / 0.28f)) * (dm - h.dFilt);                            // derivative on measurement, filtered
      h.d = -h.kd * h.dFilt;
      h.i = h.iTerm;
      out = h.p + h.i + h.d;
      h.lastT = h.T;
    } else { h.p = h.i = h.d = 0; }
    h.duty = clampf(out, 0, h.maxDuty);
    if (h.mode == 0) h.duty = 0;
    applyPwm(h);
  }
  inTick = false;
}

static uint32_t g_pre = 0;                           // >0: conversion was already triggered at this millis() (pipelined read)
static void waitMs(uint32_t ms) {                    // delay that keeps the control loop and commands alive
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { handleSerial(); controlTick(); canPoll(); delay(1); }
}

// ---------------------------------------------------------------- readings
struct Reading {
  int n = 0;
  const char *k[12];
  float v[12];
  const char *note = "";
  void add(const char *key, float val) { if (n < 12) { k[n] = key; v[n] = val; n++; } }
};
static bool flowRunning = false;

static bool probeSHT45(TwoWire &w, uint8_t a, Reading &r) {
  uint8_t d[6]; const uint8_t c[1] = {0xFD};
  if (g_pre) { uint32_t t = g_pre; g_pre = 0; if (millis() - t < 10) waitMs(10 - (millis() - t)); }
  else { if (!writeCmd(w, a, c, 1)) return false; waitMs(10); }
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
  uint16_t mfr, dev, cfg, vb, dt, cur;
  if (!readReg16(w, a, 0x3E, mfr) || !readReg16(w, a, 0x3F, dev) || !readReg16(w, a, 0x00, cfg) ||
      !readReg16(w, a, 0x05, vb) || !readReg16(w, a, 0x06, dt) || !readReg16(w, a, 0x07, cur)) return false;
  float vbus = vb * 0.003125f, mA = (int16_t)cur * 1.2f;
  r.add("Bus voltage (V)", vbus);
  r.add("Current (mA)", mA);
  r.add("Power (W)", vbus * mA / 1000.0f);
  r.add("Die temp (C)", ((int16_t)dt / 16) * 0.125f);
  r.add("Config", cfg);
  r.add("Device ID", dev);
  if (mfr != 0x5449) { r.note = "bad MFR_ID"; return false; }
  return true;
}
static bool probeSSC(TwoWire &w, uint8_t a, Reading &r) {
  uint8_t d[4];
  if (g_pre) { uint32_t t = g_pre; g_pre = 0; if (millis() - t < 10) waitMs(10 - (millis() - t)); }
  else { w.beginTransmission(a); if (w.endTransmission() != 0) return false; waitMs(10); }
  if (!readBytes(w, a, 4, d)) return false;
  int status = d[0] >> 6, praw = ((d[0] & 0x3F) << 8) | d[1], traw = (d[2] << 3) | (d[3] >> 5);
  r.add("Pressure (psi)", (praw - OUT_MIN) * (P_MAX - P_MIN) / (OUT_MAX - OUT_MIN) + P_MIN);
  r.add("Temperature (C)", traw * 200.0f / 2047.0f - 50.0f);
  r.add("Status (0=ok)", status);
  r.add("Pressure raw", praw);
  if (status == 3) r.note = "diagnostic fault";
  return status == 0 || status == 2;
}
static bool probeADS7828(TwoWire &w, uint8_t a, Reading &r) {
  for (int ch = 0; ch < 8; ch++) {
    int raw;
    if (!adsRead(ch, raw)) return false;
    r.add(ADS_NAMES[ch], raw * 3.3f / 4096.0f);                // volts
  }
  return true;
}
static bool probeFlow(TwoWire &w, uint8_t a, Reading &r) {
  uint8_t d[9];
  if (!flowRunning) {                                           // (re)start after plug-in
    if (!writeCmd(w, a, FLOW_START_CMD, 2)) return false;
    waitMs(50);
    flowRunning = true;
  }
  if (!readBytes(w, a, 9, d)) { flowRunning = false; return false; }
  if (crc8(d, 2) != d[2] || crc8(d + 3, 2) != d[5]) { r.note = "CRC error"; return false; }
  r.add("Flow (ml/min)", (int16_t)((d[0] << 8) | d[1]) / FLOW_SCALE);
  r.add("Temperature (C)", (int16_t)((d[3] << 8) | d[4]) / FLOW_TEMP_SCALE);
  r.add("Flags", (d[6] << 8) | d[7]);
  return true;
}

typedef bool (*ProbeFn)(TwoWire &, uint8_t, Reading &);
struct Dev { uint8_t bus, addr; const char *ref, *name; ProbeFn probe; };
static const Dev DEVS[] = {
  {0, 0x28, "U14", "Pressure sensor 0", probeSSC},
  {0, 0x40, "U9",  "INA740 REG heater", probeINA740},
  {0, 0x41, "U12", "INA740 NOZZLE heater", probeINA740},
  {0, 0x44, "U4",  "SHT45 temp/RH", probeSHT45},
  {0, 0x45, "U16", "INA740 EXTRA heater 4", probeINA740},
  {0, 0x48, "U11", "ADS7828 ADC mux", probeADS7828},
  {0, 0x50, "U5",  "24AA02UID EEPROM", probeEEPROM},
  {1, 0x08, "J6",  "Flow sensor", probeFlow},
  {1, 0x28, "U17", "Pressure sensor 1", probeSSC},
  {1, 0x40, "U15", "INA740 EXTRA heater 1", probeINA740},
  {1, 0x41, "U10", "INA740 EXTRA heater 2", probeINA740},
  {1, 0x44, "U13", "INA740 EXTRA heater 3", probeINA740},
};
static const int NDEV = sizeof(DEVS) / sizeof(DEVS[0]);
static const int FLOW_IDX = 7;

struct Gp { const char *label; int pin; };
static const Gp GPIOS[] = {
  {"DRV nFAULT", PIN_DRV_NFAULT}, {"STL_REP", PIN_M_STLREP}, {"FUSE_FLT", PIN_FUSE_FLT}, {"FUSE_PG", PIN_FUSE_PG}, {"FUSE_EN", PIN_FUSE_EN},
  {"ALERT REG", 41}, {"ALERT NOZZLE", 35}, {"ALERT EXTRA1", 38},
  {"ALERT EXTRA2", 10}, {"ALERT EXTRA3", 48}, {"ALERT EXTRA4", 21},
};

// ---------------------------------------------------------------- telemetry
static char ob[10000];
static size_t ol = 0;
static void ap(const char *f, ...) {
  va_list a; va_start(a, f);
  int n = vsnprintf(ob + ol, sizeof(ob) - ol, f, a);
  va_end(a);
  if (n > 0) ol = min(ol + (size_t)n, sizeof(ob) - 1);
}
static void fnum(float v, const char *fmt = "%.4f") { if (isfinite(v)) ap(fmt, v); else ap("null"); }

static bool scanFound[2][128];
static uint32_t lastScan = 0, lastCycleMs = 0;

static void cycle() {
  uint32_t t0 = millis();
  static bool devOk[NDEV] = {}; static uint32_t smp[NDEV] = {};
  bool live[NDEV]; int liveCount[2] = {0, 0};
  for (int i = 0; i < NDEV; i++) {
    TwoWire &w = busRef(DEVS[i].bus);
    if (devOk[i]) live[i] = true;                      // last read worked: skip the extra address probe (saves a transaction)
    else { w.beginTransmission(DEVS[i].addr); live[i] = (w.endTransmission() == 0); }
    if (live[i]) liveCount[DEVS[i].bus]++;
    controlTick();
  }
  if (!live[FLOW_IDX]) flowRunning = false;
  {                                                    // incremental address scan: 16 addresses per cycle per bus (keeps cycles short)
    static int scanPos = 1;
    for (int b = 0; b < 2; b++)
      for (int k = 0; k < 16; k++) {
        int a = scanPos + k; if (a >= 127) break;
        TwoWire &w = busRef(b); w.beginTransmission(a); scanFound[b][a] = (w.endTransmission() == 0);
      }
    scanPos += 16; if (scanPos >= 127) scanPos = 1;
  }
  bool stuck[2] = {false, false};
  for (int b = 0; b < 2; b++) {
    if (liveCount[b] > 0) continue;
    busRef(b).end();
    pinMode(BUS_SDA[b], INPUT); pinMode(BUS_SCL[b], INPUT);
    stuck[b] = !digitalRead(BUS_SDA[b]) || !digitalRead(BUS_SCL[b]);
    busInit(b);
  }

  Reading rd[NDEV]; bool okd[NDEV];
  auto isSlow = [](int i) { return DEVS[i].probe == probeSSC || DEVS[i].probe == probeSHT45; };
  uint32_t trigT[NDEV] = {};
  for (int i = 0; i < NDEV; i++) {                    // start the slow conversions (SSC, SHT45) first, in parallel
    okd[i] = false;
    if (live[i] && isSlow(i)) {
      TwoWire &w = busRef(DEVS[i].bus);
      if (DEVS[i].probe == probeSHT45) { const uint8_t c = 0xFD; if (writeCmd(w, DEVS[i].addr, &c, 1)) trigT[i] = millis(); }
      else { w.beginTransmission(DEVS[i].addr); if (w.endTransmission() == 0) trigT[i] = millis(); }
    }
  }
  for (int pass = 0; pass < 2; pass++)                // pass 0: fast devices while the slow ones convert; pass 1: slow devices
    for (int i = 0; i < NDEV; i++) {
      if (isSlow(i) != (pass == 1) || !live[i]) continue;
      if (pass == 1) g_pre = trigT[i];
      okd[i] = DEVS[i].probe(busRef(DEVS[i].bus), DEVS[i].addr, rd[i]);
      g_pre = 0;
      devOk[i] = okd[i]; if (okd[i]) smp[i]++;
      if (!okd[i] && live[i]) { TwoWire &w = busRef(DEVS[i].bus); w.beginTransmission(DEVS[i].addr); live[i] = (w.endTransmission() == 0); }
      controlTick();
    }

  ol = 0;
  ap("{\"t\":%lu,\"stuck\":[%s,%s],\"devs\":[", (unsigned long)millis(), stuck[0] ? "true" : "false", stuck[1] ? "true" : "false");
  for (int i = 0; i < NDEV; i++) {
    const Dev &d = DEVS[i]; Reading &r = rd[i]; bool ok = okd[i];
    ap("%s{\"bus\":%d,\"addr\":%d,\"ref\":\"%s\",\"name\":\"%s\",\"live\":%s,\"ok\":%s,\"note\":\"%s\",\"vals\":[",
       i ? "," : "", d.bus, d.addr, d.ref, d.name, live[i] ? "true" : "false", ok ? "true" : "false", r.note);
    for (int j = 0; j < r.n; j++) { ap("%s[\"%s\",", j ? "," : "", r.k[j]); fnum(r.v[j]); ap("]"); }
    ap("]}");
  }
  ap("],\"unexpected\":[");
  bool first = true;
  for (int b = 0; b < 2; b++)
    for (int a = 1; a < 127; a++) {
      if (!scanFound[b][a]) continue;
      bool exp = (b == 0 && a >= 0x51 && a <= 0x57);        // 24AA02UID answers 0x50-0x57
      for (int i = 0; i < NDEV; i++) if (DEVS[i].bus == b && DEVS[i].addr == a) exp = true;
      if (!exp) { ap("%s[%d,%d]", first ? "" : ",", b, a); first = false; }
    }
  ap("],\"gpio\":[");
  for (size_t i = 0; i < sizeof(GPIOS) / sizeof(GPIOS[0]); i++)
    ap("%s[\"%s\",%d]", i ? "," : "", GPIOS[i].label, digitalRead(GPIOS[i].pin));
  ap("],\"htr\":[");
  for (int i = 0; i < NHEAT; i++) {
    Heater &h = H[i];
    ap("%s{\"k\":\"%s\",\"mode\":%d,\"trip\":%d,\"duty_pct\":%.2f,\"freq_hz\":%lu,\"sp_c\":%.2f,\"man_pct\":%.2f,"
       "\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f,\"maxduty_pct\":%.1f,\"maxT_c\":%.1f,\"adc_raw\":%d,\"T_ok\":%d,\"T_c\":",
       i ? "," : "", h.key, h.mode, h.trip, h.duty, (unsigned long)h.freq, h.sp, h.man, h.kp, h.ki, h.kd, h.maxDuty, h.maxT,
       h.raw, h.tok ? 1 : 0);
    fnum(h.tvalid ? h.T : NAN, "%.3f"); ap(",\"R_ohm\":"); fnum(h.tok ? h.R : NAN, "%.1f");
    ap(",\"pid_p\":%.3f,\"pid_i\":%.3f,\"pid_d\":%.3f}", h.p, h.i, h.d);
  }
  ap("]");
  { twai_status_info_t st = {}; if (canOn) twai_get_status_info(&st);
    const char *sn[] = {"stopped", "running", "bus-off", "recovering"};
    ap(",\"can\":{\"on\":%d,\"kbps\":%d,\"mode\":%d,\"state\":\"%s\",\"rx\":%lu,\"tx\":%lu,\"tx_fail\":%lu,"
       "\"tec\":%lu,\"rec\":%lu,\"bus_err\":%lu,\"rx_miss\":%lu,\"arb_lost\":%lu,\"log\":[",
       canOn ? 1 : 0, canKbps, canMode, canOn ? sn[st.state] : "off", (unsigned long)canRx, (unsigned long)canTx,
       (unsigned long)canTxFail, (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter,
       (unsigned long)st.bus_error_count, (unsigned long)st.rx_missed_count, (unsigned long)st.arb_lost_count);
    bool firstF = true;
    for (int k = 0; k < 8; k++) {
      const CanFrame &f = canLog[(canHead + k) % 8]; if (!f.ms) continue;
      ap("%s[%lu,%d,%lu,\"", firstF ? "" : ",", (unsigned long)f.ms, f.dir, (unsigned long)f.id);
      for (int j = 0; j < f.len; j++) ap("%02X", f.d[j]);
      ap("\"]"); firstF = false;
    }
    ap("]}"); }
  ap(",\"sol\":%d", solOn);
  ap(",\"mot\":{\"sleep\":%d,\"en\":%d,\"dir\":%d,\"stl\":%d,\"stl_dis\":%d,\"target_hz\":%.1f,\"hz\":%.1f,\"real_hz\":%lu,\"accel\":%.0f,"
     "\"nfault\":%d,\"stl_rep\":%d,\"trq_mv\":%d}",
     M.sleepHi, M.en, M.dir, M.stl, M.stlDis, M.target, M.cur, (unsigned long)M.realHz, M.accel,
     digitalRead(PIN_DRV_NFAULT), digitalRead(PIN_M_STLREP), (int)analogReadMilliVolts(PIN_M_TRQ));
  cycN++;
  ap(",\"smp\":{\"cyc\":%lu,\"ctl\":%lu,\"dev\":[", (unsigned long)cycN, (unsigned long)ctlN);
  for (int i = 0; i < NDEV; i++) ap("%s%lu", i ? "," : "", (unsigned long)smp[i]);
  ap("],\"therm\":[");
  for (int i = 0; i < 6; i++) ap("%s%lu", i ? "," : "", (unsigned long)thermOk[i]);
  ap("]}");
  ap(",\"sys\":{\"i2c_khz\":%lu,\"telem_ms\":%lu,\"ctl_ms\":%lu,\"rst\":%d,\"inhibit\":%d,\"heap\":%u,\"ctl_max_ms\":%lu,\"cycle_ms\":%lu,\"beta\":%.0f,\"r0\":%.0f,\"deadman_ms\":%lu}}\n",
     (unsigned long)(i2cHz / 1000), (unsigned long)telemMs, (unsigned long)ctlMs, (int)esp_reset_reason(), fuseInhibit ? 1 : 0, (unsigned)ESP.getFreeHeap(), (unsigned long)ctlMaxMs, (unsigned long)lastCycleMs,
     thermBeta, thermR0, (unsigned long)DEADMAN_MS);
  Serial.write((const uint8_t *)ob, ol);
  ctlMaxMs = 0;
  lastCycleMs = millis() - t0;
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  for (auto &h : H) { pinMode(h.pin, OUTPUT); digitalWrite(h.pin, LOW); }   // safe state first
  Serial.begin(115200);
  for (auto &h : H) { ledcAttach(h.pin, h.freq, PWM_BITS); ledcWrite(h.pin, 0); }
  for (const Gp &g : GPIOS) pinMode(g.pin, INPUT);
  pinMode(PIN_FUSE_EN, OUTPUT); digitalWrite(PIN_FUSE_EN, HIGH);   // e-fuse ON at boot (heater gates are LOW)
  busInit(0); busInit(1);
  pinMode(PIN_SOL, OUTPUT); digitalWrite(PIN_SOL, LOW);
  motorInit();
  canStart(500, 0);
  delay(300);
}

void loop() {
  uint32_t t0 = millis();
  cycle();
  do { handleSerial(); controlTick(); canPoll(); if (telemMs) delay(1); } while (millis() - t0 < telemMs);
}
