/*
  Feather RP2040 CAN  <->  Nebulizer PCB V1 : CAN master / USB bridge  (protocol: NebCanProto.h, CAN_PROTOCOL.md)

  * Decodes every telemetry frame from the Nebulizer board (500 kbit/s) and keeps the latest values.
  * USB serial (115200) commands are sent to the board over CAN. Same grammar as the board's own USB commands:
      PING | OFF | CA | FUSE,<0|1> | M,<ch>,<0|1|2> | D,<ch>,<duty%> | S,<ch>,<C> | G,<ch>,<kp>,<ki>,<kd> | F,<ch>,<Hz>
      X,<ch>,<maxduty%>,<maxC> | C,<ch> | SOL,<0|1> | MS,<stepHz> | MA,<Hz/s> | MP,<0|1> | ME,<0|1|2> | MD,<0|1> | ML,<0|1|2>
      MR,<0|1> | MC | TH,<beta>,<r0> | RATE,<ms>
  * Feather-only commands:
      INFO        board firmware slot / pending-verify / build id        OTA,<size>,<crc32hex>  update the board over CAN (use tools/can_ota.py)
      ARM,<0|1>   while armed, a PING goes out every 500 ms (the board turns heaters/motor/solenoid off after 3 s of silence)
      STAT        print a decoded status table   JSON,<ms>  stream decoded telemetry as JSON lines (for bridge.py --can)        STREAM,<ms>   print that table every <ms> (0 = off)
      RATES       per-ID frame rates                  RAW,<0|1>     print every received frame
      TEST        non-energizing self-test: heartbeat, all telemetry IDs, command acks + read-back, rate change
  * USB has priority on the board: while a USB host is talking to it, CAN commands get ACK status BUSY (4); OFF always works.
*/
#include <Adafruit_MCP2515.h>
#include <stdarg.h>
#include "NebCanProto.h"
using namespace nc;
static uint32_t loopsPerSec = 0, loopCnt = 0;

Adafruit_MCP2515 mcp(PIN_CAN_CS);
static const long BITRATE = nc::CAN_KBPS * 1000L;
static const char *HNAME[6] = {"reg", "nozzle", "extra1", "extra2", "extra3", "extra4"};

struct Frm { uint32_t last = 0, n = 0, prevN = 0; float hz = 0; };
static Frm fHb, fSys, fHeat[6], fPow[6], fPress, fEnv, fMot, fMot2, fAdc, fDev;

struct State {
  uint32_t upMs = 0; uint8_t sysFlags = 0, rst = 0, hMask = 0, tMask = 0;
  struct H { uint8_t mode = 0, trip = 0; bool tok = false; float T = NAN, sp = NAN, duty = NAN, V = NAN, mA = NAN, W = NAN, die = NAN; } h[6];
  float p[2] = {NAN, NAN}, pt[2] = {NAN, NAN}, flow = NAN, flowT = NAN, shtT = NAN, rh = NAN;
  uint32_t mHz = 0, mTarget10 = 0, mAccel = 0; uint16_t trq = 0; uint8_t mFlags = 0, mStl = 0; bool haveMotor = false;
  uint8_t boot[4] = {0, 0, 0, 0}; uint16_t boots = 0; uint8_t bootFlags = 0; bool haveBoot = false;
  float imon = NAN, ftemp = NAN; uint16_t i2cKhz = 0, periodMs = 0, live = 0, ok = 0, cycMs = 0; uint8_t tec = 0, rec = 0;
} S;

static uint32_t rxTotal = 0, txTotal = 0, txFail = 0;
static bool armed = false, rawOn = false;
static uint32_t streamMs = 0, lastStream = 0, lastPing = 0, jsonMs = 0, lastJson = 0;
static volatile uint32_t otaAckNext = 0, otaAckGot = 0; static volatile uint8_t otaAckSt = 0; static volatile bool otaAckNew = false;
static volatile uint8_t ackOp = 0, ackSt = 0xFF; static volatile bool ackNew = false; static uint8_t ackEcho[6];

static float f16(int16_t v, float s) { return v == INV16 ? NAN : v / s; }
static float fu16(uint16_t v, float s) { return v == INVU16 ? NAN : v / s; }
static void pv(const char *fmt, float v) { if (isnan(v)) Serial.print("  --"); else Serial.printf(fmt, v); }

static void onFrame(uint32_t id, const uint8_t *d, int len) {
  rxTotal++;
  if (rawOn) { Serial.printf("RX %03lX:", id); for (int i = 0; i < len; i++) Serial.printf(" %02X", d[i]); Serial.println(); }
  Frm *f = nullptr; uint32_t now = millis();
  if (id == ID_ACK && len >= 2) { ackOp = d[0]; ackSt = d[1]; memcpy(ackEcho, d + 2, len - 2 > 6 ? 6 : len - 2); ackNew = true; return; }
  if (len < 8) return;
  if (id == ID_OTA_ACK) { otaAckNext = d[0] | (d[1] << 8) | (d[2] << 16); otaAckSt = d[3]; otaAckGot = get32(d + 4); otaAckNew = true; return; }
  if (id == ID_BOOT) { for (int i = 0; i < 4; i++) S.boot[i] = d[i]; S.boots = get16(d + 4); S.bootFlags = d[6]; S.haveBoot = true; return; }
  if (id == ID_HB) { f = &fHb; }
  else if (id == ID_SYS) { f = &fSys; S.upMs = get32(d); S.sysFlags = d[4]; S.rst = d[5]; S.hMask = d[6]; S.tMask = d[7]; }
  else if (id >= ID_HEAT && id < ID_HEAT + 6) {
    int i = id - ID_HEAT; f = &fHeat[i]; auto &h = S.h[i];
    h.mode = d[0] & 3; h.tok = d[0] & 4; h.trip = (d[0] >> 4) & 7; h.T = f16(geti16(d + 1), 100); h.sp = f16(geti16(d + 3), 100);
    h.duty = get16(d + 5) / 100.0f;
  } else if (id >= ID_POWER && id < ID_POWER + 6) {
    int i = id - ID_POWER; f = &fPow[i]; auto &h = S.h[i];
    h.V = fu16(get16(d), 1000); h.mA = f16(geti16(d + 2), 1); h.W = fu16(get16(d + 4), 100); h.die = f16(geti16(d + 6), 10);
  } else if (id == ID_PRESS) { f = &fPress; S.p[0] = f16(geti16(d), 100); S.p[1] = f16(geti16(d + 2), 100); S.pt[0] = f16(geti16(d + 4), 100); S.pt[1] = f16(geti16(d + 6), 100); }
  else if (id == ID_ENV) { f = &fEnv; S.flow = f16(geti16(d), 100); S.flowT = f16(geti16(d + 2), 100); S.shtT = f16(geti16(d + 4), 100); S.rh = fu16(get16(d + 6), 100); }
  else if (id == ID_MOTOR) { f = &fMot; S.mHz = get32(d); S.trq = get16(d + 4); S.mFlags = d[6]; S.mStl = d[7]; S.haveMotor = true; }
  else if (id == ID_MOTOR2) { f = &fMot2; S.mTarget10 = get32(d); S.mAccel = get32(d + 4); }
  else if (id == ID_ADC) { f = &fAdc; S.imon = fu16(get16(d), 1000); S.ftemp = fu16(get16(d + 2), 1000); S.i2cKhz = get16(d + 4); S.periodMs = get16(d + 6); }
  else if (id == ID_DEV) { f = &fDev; S.live = get16(d); S.ok = get16(d + 2); S.cycMs = get16(d + 4); S.tec = d[6]; S.rec = d[7]; }
  if (f) { f->last = now; f->n++; }
}

static void pump() {                                   // drain the MCP2515 (call often)
  for (int guard = 0; guard < 16; guard++) {
    int n = mcp.parsePacket();
    if (!n && mcp.packetId() == -1) break;
    long id = mcp.packetId(); uint8_t d[8]; int len = 0;
    while (mcp.available() && len < 8) d[len++] = mcp.read();
    if (id >= 0) onFrame((uint32_t)id, d, len);
  }
}
static bool sendFrame(uint32_t id, const uint8_t *d, int n) {
  if (!mcp.beginPacket(id)) { txFail++; return false; }
  for (int i = 0; i < n; i++) mcp.write(d[i]);
  bool ok = mcp.endPacket(); ok ? txTotal++ : txFail++; return ok;
}
static void sendCmd(uint8_t op, uint8_t a1 = 0, uint8_t a2 = 0, uint8_t a3 = 0, uint8_t a4 = 0, uint8_t a5 = 0, uint8_t a6 = 0, uint8_t a7 = 0, int n = 8) {
  uint8_t d[8] = {op, a1, a2, a3, a4, a5, a6, a7}; sendFrame(ID_CMD, d, n);
}
static void sendCmdBuf(uint8_t op, const uint8_t *args, int na) {
  uint8_t d[8] = {op}; memcpy(d + 1, args, na); sendFrame(ID_CMD, d, 1 + na);
}
// wait for the ACK of 'op'; returns status or 0xFF on timeout
static uint8_t waitAck(uint8_t op, uint32_t ms = 400) {
  uint32_t t = millis();
  while (millis() - t < ms) { pump(); if (ackNew) { ackNew = false; if (ackOp == op) return ackSt; } }
  return 0xFF;
}

// ---- motor config is one frame on the wire: keep the other fields from telemetry when a single one changes
static void motorCfg(int sleepHi, int en, int dir, int stl, int stlDis) {
  uint8_t a[5] = {(uint8_t)sleepHi, (uint8_t)en, (uint8_t)dir, (uint8_t)stl, (uint8_t)stlDis}; sendCmdBuf(OP_M_CFG, a, 5);
}
static bool motorShadow(int &sl, int &en, int &dir, int &stl, int &dis) {
  if (!S.haveMotor) { Serial.println("no motor telemetry yet"); return false; }
  sl = S.mFlags & 1; en = (S.mFlags >> 1) & 3; dir = (S.mFlags >> 3) & 1; dis = (S.mFlags >> 4) & 1; stl = S.mStl; return true;
}


// ---------------------------------------------------------------- firmware update of the Nebulizer board over CAN
// PC (tools/can_ota.py) -> USB serial -> here -> CAN. The PC sends 4000-byte blocks when asked ("BLK,<n>").
static bool otaFail(const char *why) { Serial.printf("OTA FAIL %s\n", why); uint8_t x = 0; sendCmdBuf(OP_OTA_ABORT, &x, 0); return false; }
static bool otaRun(uint32_t size, uint32_t crc) {
  jsonMs = 0; streamMs = 0; static uint8_t blk[OTA_BLOCK]; uint8_t a[4];
  put32(a, size); ackNew = false; sendCmdBuf(OP_OTA_BEGIN, a, 4);
  uint8_t st = waitAck(OP_OTA_BEGIN, 20000);                       // the board erases the spare slot first
  if (st != ST_OK) { Serial.printf("OTA FAIL begin status=%u%s\n", st, st == ST_BUSY ? " (USB has control of the board)" : st == 0xFF ? " (no ack)" : ""); return false; }
  Serial.println("OTA READY");
  uint32_t off = 0, t0 = millis(); Serial.setTimeout(8000);
  while (off < size) {
    int len = size - off < (uint32_t)OTA_BLOCK ? (int)(size - off) : OTA_BLOCK;
    Serial.printf("BLK,%d\n", len);
    if ((int)Serial.readBytes((char *)blk, len) != len) return otaFail("timeout reading block from PC");
    int nF = (len + OTA_PAYLOAD - 1) / OTA_PAYLOAD, acked = 0, tries = 0; uint32_t seq0 = off / OTA_PAYLOAD;
    otaAckNew = false;
    while (acked < nF) {
      int wend = acked + OTA_WINDOW < nF ? acked + OTA_WINDOW : nF;
      for (int i = acked; i < wend; i++) {
        uint32_t sq = seq0 + i; uint8_t d[8] = {(uint8_t)sq, (uint8_t)(sq >> 8), (uint8_t)(sq >> 16), 0, 0, 0, 0, 0};
        int n = len - i * OTA_PAYLOAD < OTA_PAYLOAD ? len - i * OTA_PAYLOAD : OTA_PAYLOAD; memcpy(d + 3, blk + i * OTA_PAYLOAD, n);
        if (!sendFrame(ID_OTA_DATA, d, 8)) { delay(2); }
      }
      uint32_t t = millis(); bool prog = false;
      while (millis() - t < 150) {
        pump();
        if (otaAckNew) {
          otaAckNew = false; if (otaAckSt != OTA_OK) { char b[40]; snprintf(b, sizeof b, "board write error %u", otaAckSt); return otaFail(b); }
          int pr = (int)(otaAckNext - seq0); if (pr > nF) pr = nF;
          if (pr > acked) { acked = pr; prog = true; if (acked >= wend) break; }
        }
      }
      if (prog) tries = 0; else if (++tries > 40) return otaFail("no ack from board (CAN link?)");
    }
    off += len; Serial.printf("PROG,%lu,%lu\n", (unsigned long)off, (unsigned long)(millis() - t0));
  }
  put32(a, crc); ackNew = false; sendCmdBuf(OP_OTA_END, a, 4);
  st = waitAck(OP_OTA_END, 10000);
  if (st != ST_OK) { Serial.printf("OTA FAIL end status=%u (CRC/verify)\n", st); return false; }
  Serial.printf("OTA DONE %lu bytes in %lu ms\n", (unsigned long)size, (unsigned long)(millis() - t0)); return true;
}
static void otaInfo() {
  ackNew = false; sendCmd(OP_OTA_INFO, 0, 0, 0, 0, 0, 0, 0, 1);
  uint8_t st = waitAck(OP_OTA_INFO, 500);
  if (st == 0xFF) { Serial.println("INFO timeout"); return; }
  Serial.printf("INFO slot=%u pending=%u ota_active=%u build=%08lX\n", ackEcho[0], ackEcho[1] & 1, (ackEcho[1] >> 1) & 1, (unsigned long)get32(ackEcho + 2));
}

static void handleCmd(char *s) {
  int i, m, v; float a, b, c, d;
  uint8_t x[8];
  if (!strcmp(s, "PING")) { sendCmd(OP_PING, 0, 0, 0, 0, 0, 0, 0, 1); return; }
  if (!strcmp(s, "OFF")) { sendCmd(OP_OFF, 0, 0, 0, 0, 0, 0, 0, 1); return; }
  if (!strcmp(s, "CA")) { sendCmd(OP_CLEAR_ALL, 0, 0, 0, 0, 0, 0, 0, 1); return; }
  if (!strcmp(s, "MC")) { sendCmd(OP_M_CLEAR, 0, 0, 0, 0, 0, 0, 0, 1); return; }
  if (!strcmp(s, "STAT")) { extern void printStatus(); printStatus(); return; }
  if (!strcmp(s, "RATES")) { extern void printRates(); printRates(); return; }
  if (!strcmp(s, "INFO")) { otaInfo(); return; }
  { unsigned long sz, cr; if (sscanf(s, "OTA,%lu,%lx", &sz, &cr) == 2) { otaRun(sz, cr); return; } }
  if (!strcmp(s, "DUMP")) { mcp.dumpRegisters(Serial); return; }
  if (!strcmp(s, "LPS")) { Serial.printf("loops/s %lu  rx %lu\n", (unsigned long)loopsPerSec, (unsigned long)rxTotal); return; }
  if (!strcmp(s, "TEST")) { extern void selfTest(); selfTest(); return; }
  if (sscanf(s, "ARM,%d", &v) == 1) { armed = v; Serial.printf("armed=%d\n", armed); return; }
  if (sscanf(s, "JSON,%d", &v) == 1) { jsonMs = v < 0 ? 0 : v; return; }
  if (sscanf(s, "STREAM,%d", &v) == 1) { streamMs = v < 0 ? 0 : v; return; }
  if (sscanf(s, "RAW,%d", &v) == 1) { rawOn = v; return; }
  if (sscanf(s, "FUSE,%d", &v) == 1) { sendCmd(OP_FUSE, v ? 1 : 0, 0, 0, 0, 0, 0, 0, 2); return; }
  if (sscanf(s, "M,%d,%d", &i, &m) == 2) { sendCmd(OP_H_MODE, i, m, 0, 0, 0, 0, 0, 3); return; }
  if (sscanf(s, "D,%d,%f", &i, &a) == 2) { x[0] = i; put16(x + 1, (uint16_t)constrain(lroundf(a * 100), 0L, 10000L)); sendCmdBuf(OP_H_DUTY, x, 3); return; }
  if (sscanf(s, "S,%d,%f", &i, &a) == 2) { x[0] = i; put16(x + 1, (uint16_t)(int16_t)constrain(lroundf(a * 100), -2000L, 15000L)); sendCmdBuf(OP_H_SP, x, 3); return; }
  if (sscanf(s, "G,%d,%f,%f,%f", &i, &a, &b, &c) == 4) {
    x[0] = i; put16(x + 1, constrain(lroundf(a * 100), 0L, 65535L)); put16(x + 3, constrain(lroundf(b * 100), 0L, 65535L)); put16(x + 5, constrain(lroundf(c * 100), 0L, 65535L));
    sendCmdBuf(OP_H_PID, x, 7); return; }
  if (sscanf(s, "F,%d,%f", &i, &a) == 2) { x[0] = i; put16(x + 1, constrain(lroundf(a), 1L, 20000L)); sendCmdBuf(OP_H_FREQ, x, 3); return; }
  if (sscanf(s, "X,%d,%f,%f", &i, &a, &b) == 3) { x[0] = i; put16(x + 1, constrain(lroundf(a * 100), 0L, 10000L)); put16(x + 3, (uint16_t)(int16_t)constrain(lroundf(b * 10), 0L, 1500L)); sendCmdBuf(OP_H_LIM, x, 5); return; }
  if (sscanf(s, "C,%d", &i) == 1) { sendCmd(OP_H_CLEAR, i, 0, 0, 0, 0, 0, 0, 2); return; }
  if (sscanf(s, "SOL,%d", &v) == 1) { sendCmd(OP_SOL, v ? 1 : 0, 0, 0, 0, 0, 0, 0, 2); return; }
  if (sscanf(s, "MS,%f", &a) == 1) { put32(x, (uint32_t)constrain(lroundf(a * 10), 0L, 1000000L)); sendCmdBuf(OP_M_SPEED, x, 4); return; }
  if (sscanf(s, "MA,%f", &a) == 1) { put32(x, (uint32_t)constrain(lroundf(a), 10L, 1000000L)); sendCmdBuf(OP_M_ACCEL, x, 4); return; }
  int sl, en, dr, st, dis;
  if (sscanf(s, "MP,%d", &v) == 1) { if (motorShadow(sl, en, dr, st, dis)) motorCfg(v ? 1 : 0, en, dr, st, dis); return; }
  if (sscanf(s, "ME,%d", &v) == 1) { if (motorShadow(sl, en, dr, st, dis)) motorCfg(sl, v, dr, st, dis); return; }
  if (sscanf(s, "MD,%d", &v) == 1) { if (motorShadow(sl, en, dr, st, dis)) motorCfg(sl, en, v ? 1 : 0, st, dis); return; }
  if (sscanf(s, "ML,%d", &v) == 1) { if (motorShadow(sl, en, dr, st, dis)) motorCfg(sl, en, dr, v, dis); return; }
  if (sscanf(s, "MR,%d", &v) == 1) { if (motorShadow(sl, en, dr, st, dis)) motorCfg(sl, en, dr, st, v ? 1 : 0); return; }
  if (sscanf(s, "TH,%f,%f", &a, &b) == 2) { put16(x, lroundf(a)); put32(x + 2, (uint32_t)lroundf(b)); sendCmdBuf(OP_THERM, x, 6); return; }
  if (sscanf(s, "RATE,%d", &v) == 1) { put16(x, v); sendCmdBuf(OP_RATE, x, 2); return; }
  (void)d; Serial.println("? unknown command");
}

void printStatus() {
  uint32_t now = millis();
  bool alive = fHb.n && now - fHb.last < 2500;
  Serial.printf("---- board %s", alive ? "ONLINE" : "NO HEARTBEAT");
  if (fSys.n) {
    uint8_t f = S.sysFlags;
    Serial.printf("  up=%.1fs  fusePG=%d fuseFLT=%d cut=%d sol=%d motor-nFAULT=%s deadman=%s control=%s  reset=%d\n", S.upMs / 1000.0f,
      !!(f & SF_FUSE_PG), !!(f & SF_FUSE_FLT), !!(f & SF_INHIBIT), !!(f & SF_SOL), (f & SF_NFAULT_OK) ? "ok" : "FAULT",
      (f & SF_DEADMAN_OK) ? "ok" : "EXPIRED", (f & SF_USB_ACTIVE) ? "USB (CAN cmds refused)" : "CAN free", S.rst);
  } else Serial.println();
  static const char *RS[16] = {"?", "power-on", "ext-pin", "software", "panic", "int-wdt", "task-wdt", "wdt", "deepsleep", "BROWN-OUT", "sdio", "usb", "jtag", "efuse", "glitch", "cpu-lockup"};
  if (S.haveBoot) Serial.printf("boots=%u  last reset reasons (newest first): %s, %s, %s, %s   brown-out reset %s%s\n", S.boots, RS[S.boot[0] & 15], RS[S.boot[1] & 15], RS[S.boot[2] & 15], RS[S.boot[3] & 15], (S.bootFlags & 1) ? "DISABLED" : "enabled", (S.bootFlags & 2) ? "" : "  (history just initialised = power was lost)");
  static const char *MODE[4] = {"OFF", "MAN", "PID", "?"};
  for (int i = 0; i < 6; i++) {
    auto &h = S.h[i];
    Serial.printf("H%d %-7s %s trip=%d T=", i, HNAME[i], MODE[h.mode & 3], h.trip); pv("%5.1fC", h.T);
    Serial.print(" sp="); pv("%5.1f", h.sp); Serial.print(" duty="); pv("%5.1f%%", h.duty);
    Serial.print(" | "); pv("%5.2fV", h.V); pv(" %6.0fmA", h.mA); pv(" %6.2fW", h.W); pv(" die %4.1fC", h.die); Serial.println();
  }
  Serial.print("pressure psi: "); pv("%7.2f", S.p[0]); pv("%7.2f", S.p[1]); Serial.print("  temp C: "); pv("%6.2f", S.pt[0]); pv("%6.2f", S.pt[1]);
  Serial.print("  flow ml/min: "); pv("%6.2f", S.flow); Serial.print("  SHT45: "); pv("%5.1fC", S.shtT); pv(" %4.1f%%RH", S.rh); Serial.println();
  Serial.printf("motor: %lu Hz (target %.1f, accel %lu) sleep=%d en=%d dir=%d stlMode=%d stallRepOff=%d nFAULT=%s STL_REP=%d trq=%umV\n",
    (unsigned long)S.mHz, S.mTarget10 / 10.0f, (unsigned long)S.mAccel, S.mFlags & 1, (S.mFlags >> 1) & 3, (S.mFlags >> 3) & 1, S.mStl,
    (S.mFlags >> 4) & 1, (S.mFlags & 32) ? "ok" : "FAULT", (S.mFlags >> 6) & 1, S.trq);
  Serial.print("efuse IMON/TEMP V: "); pv("%5.3f", S.imon); pv("%6.3f", S.ftemp);
  Serial.printf("   I2C %ukHz live=0x%03X ok=0x%03X cycle=%ums   board CAN tec/rec=%u/%u   period=%ums   (rx %lu tx %lu txfail %lu)\n",
    S.i2cKhz, S.live, S.ok, S.cycMs, S.tec, S.rec, S.periodMs, (unsigned long)rxTotal, (unsigned long)txTotal, (unsigned long)txFail);
}

static void rateTick() {                               // 1 s window per ID
  static uint32_t t0 = 0; uint32_t now = millis();
  loopCnt++;
  if (now - t0 < 1000) return; loopsPerSec = loopCnt; loopCnt = 0; float dt = (now - t0) / 1000.0f; t0 = now;
  Frm *all[] = {&fHb, &fSys, &fPress, &fEnv, &fMot, &fMot2, &fAdc, &fDev, &fHeat[0], &fHeat[1], &fHeat[2], &fHeat[3], &fHeat[4], &fHeat[5],
                &fPow[0], &fPow[1], &fPow[2], &fPow[3], &fPow[4], &fPow[5]};
  for (Frm *f : all) { f->hz = (f->n - f->prevN) / dt; f->prevN = f->n; }
}
static char jb[2600]; static size_t jl = 0;
static void jp(const char *f, ...) { va_list a; va_start(a, f); int n = vsnprintf(jb + jl, sizeof(jb) - jl, f, a); va_end(a); if (n > 0) jl = min(jl + (size_t)n, sizeof(jb) - 1); }
static void jn(float v, int dec = 2) { if (isnan(v)) jp("null"); else jp("%.*f", dec, v); }
void printJson() {                                      // one decoded line for the PC bridge/GUI (lab.html via bridge.py --can)
  uint32_t now = millis(); bool alive = fHb.n && now - fHb.last < 2500; jl = 0;
  jp("{\"can\":1,\"alive\":%d,\"up\":%lu,\"sf\":%u,\"rst\":%u,\"h\":[", alive, (unsigned long)S.upMs, S.sysFlags, S.rst);
  for (int i = 0; i < 6; i++) { auto &h = S.h[i];
    jp("%s[%u,%d,%u,", i ? "," : "", h.mode, h.tok, h.trip); jn(h.T); jp(","); jn(h.sp); jp(","); jn(h.duty); jp(",");
    jn(h.V, 3); jp(","); jn(h.mA, 0); jp(","); jn(h.W); jp(","); jn(h.die, 1); jp("]"); }
  jp("],\"p\":["); jn(S.p[0]); jp(","); jn(S.p[1]); jp("],\"pt\":["); jn(S.pt[0]); jp(","); jn(S.pt[1]);
  jp("],\"fl\":["); jn(S.flow); jp(","); jn(S.flowT); jp("],\"sht\":["); jn(S.shtT); jp(","); jn(S.rh);
  jp("],\"m\":[%lu,%lu,%lu,%u,%u,%u],\"adc\":[", (unsigned long)S.mHz, (unsigned long)S.mTarget10, (unsigned long)S.mAccel, S.trq, S.mFlags, S.mStl);
  jn(S.imon, 3); jp(","); jn(S.ftemp, 3);
  jp(",%u,%u],\"dev\":[%u,%u,%u,%u,%u],\"n\":{\"sys\":%lu,\"heat\":[", S.i2cKhz, S.periodMs, S.live, S.ok, S.cycMs, S.tec, S.rec, (unsigned long)fSys.n);
  for (int i = 0; i < 6; i++) jp("%s%lu", i ? "," : "", (unsigned long)fHeat[i].n);
  jp("],\"pow\":["); for (int i = 0; i < 6; i++) jp("%s%lu", i ? "," : "", (unsigned long)fPow[i].n);
  jp("],\"press\":%lu,\"env\":%lu,\"mot\":%lu,\"adc\":%lu,\"dev\":%lu},\"rx\":%lu,\"tx\":%lu,\"txf\":%lu,\"lps\":%lu}\n",
    (unsigned long)fPress.n, (unsigned long)fEnv.n, (unsigned long)fMot.n, (unsigned long)fAdc.n, (unsigned long)fDev.n,
    (unsigned long)rxTotal, (unsigned long)txTotal, (unsigned long)txFail, (unsigned long)loopsPerSec);
  Serial.write((const uint8_t *)jb, jl);
}
void printRates() {
  Serial.printf("frames/s: HB %.1f SYS %.1f PRESS %.1f ENV %.1f MOT %.1f MOT2 %.1f ADC %.1f DEV %.1f\n", fHb.hz, fSys.hz, fPress.hz, fEnv.hz, fMot.hz, fMot2.hz, fAdc.hz, fDev.hz);
  Serial.print("  heater:"); for (int i = 0; i < 6; i++) Serial.printf(" %.1f", fHeat[i].hz);
  Serial.print("   power:"); for (int i = 0; i < 6; i++) Serial.printf(" %.1f", fPow[i].hz); Serial.println();
}

// ---------------------------------------------------------------- self-test (does not energize any heater or the motor)
static int tPass = 0, tFail = 0;
static void check(const char *name, bool ok, const char *extra = "") {
  Serial.printf("  [%s] %s %s\n", ok ? "PASS" : "FAIL", name, extra); ok ? tPass++ : tFail++;
}
static void settle(uint32_t ms) { uint32_t t = millis(); while (millis() - t < ms) { pump(); rateTick(); } }
void selfTest() {
  tPass = tFail = 0; Serial.println("== TEST (no heaters/motor energized) ==");
  for (Frm *f : {&fHb, &fSys, &fPress, &fEnv, &fMot, &fMot2, &fAdc, &fDev}) { f->n = 0; }
  for (int i = 0; i < 6; i++) { fHeat[i].n = 0; fPow[i].n = 0; }
  settle(DEFAULT_PERIOD_MS * 10 + 1500); rateTick();
  check("heartbeat from board", fHb.n >= 1);
  bool all = fSys.n && fPress.n && fEnv.n && fMot.n && fMot2.n && fAdc.n && fDev.n;
  for (int i = 0; i < 6; i++) all = all && fHeat[i].n && fPow[i].n;
  { char b[64]; snprintf(b, sizeof b, "(SYS %lu HEAT0 %lu PRESS %lu MOT %lu)", (unsigned long)fSys.n, (unsigned long)fHeat[0].n, (unsigned long)fPress.n, (unsigned long)fMot.n);
    check("all 19 telemetry frames arriving", all, b); }
  if (!all) { Serial.println("  telemetry missing: check wiring/termination/bitrate, then retry"); }
  uint8_t st;
  sendCmd(OP_PING, 0, 0, 0, 0, 0, 0, 0, 1); st = waitAck(OP_PING);
  check("PING acked", st == ST_OK || st == ST_BUSY, st == 0xFF ? "(no ack)" : "");
  bool usb = S.sysFlags & SF_USB_ACTIVE;
  if (usb) { Serial.println("  USB is controlling the board: CAN commands will be refused (BUSY). Stop/idle the USB host (3 s) and rerun for the command tests."); }
  else {
    uint8_t x[3]; x[0] = 0; put16(x + 1, (uint16_t)1234);
    sendCmdBuf(OP_H_SP, x, 3); st = waitAck(OP_H_SP); settle(DEFAULT_PERIOD_MS * 2 + 200);
    { char b[48]; snprintf(b, sizeof b, "(sp now %.2f)", S.h[0].sp); check("heater0 setpoint 12.34 C acked + read back", st == ST_OK && fabsf(S.h[0].sp - 12.34f) < 0.02f, b); }
    sendCmd(OP_SOL, 1, 0, 0, 0, 0, 0, 0, 2); st = waitAck(OP_SOL); settle(DEFAULT_PERIOD_MS * 2 + 200);
    check("solenoid ON acked + flag set", st == ST_OK && (S.sysFlags & SF_SOL));
    sendCmd(OP_SOL, 0, 0, 0, 0, 0, 0, 0, 2); st = waitAck(OP_SOL); settle(DEFAULT_PERIOD_MS * 2 + 200);
    check("solenoid OFF acked + flag clear", st == ST_OK && !(S.sysFlags & SF_SOL));
    uint8_t y[4]; put32(y, 3000); sendCmdBuf(OP_M_ACCEL, y, 4); st = waitAck(OP_M_ACCEL); settle(DEFAULT_PERIOD_MS * 2 + 200);
    check("motor accel 3000 acked + read back", st == ST_OK && S.mAccel == 3000);
    put32(y, 2000); sendCmdBuf(OP_M_ACCEL, y, 4); waitAck(OP_M_ACCEL);
    sendCmd(0x7E, 0, 0, 0, 0, 0, 0, 0, 1); st = waitAck(0x7E);
    check("unknown opcode rejected", st == ST_UNKNOWN);
    sendCmd(OP_H_MODE, 9, 1, 0, 0, 0, 0, 0, 3); st = waitAck(OP_H_MODE);
    check("bad heater index rejected", st == ST_BAD_ARG);
    uint8_t r[2]; put16(r, DEFAULT_PERIOD_MS / 2); sendCmdBuf(OP_RATE, r, 2); waitAck(OP_RATE);
    settle(300); uint32_t n0 = fHeat[0].n; settle(2000); float hz = (fHeat[0].n - n0) / 2.0f, ex = 2000.0f / DEFAULT_PERIOD_MS;
    { char b[48]; snprintf(b, sizeof b, "(%.1f Hz, expect ~%.1f)", hz, ex); check("telemetry period halved doubles frame rate", hz > 0.7f * ex && hz < 1.3f * ex, b); }
    put16(r, DEFAULT_PERIOD_MS); sendCmdBuf(OP_RATE, r, 2); waitAck(OP_RATE);
    sendCmd(OP_H_SP, 0, 0, 0, 0, 0, 0, 0, 1);   // short frame: bad arg
    st = waitAck(OP_H_SP); check("short command rejected", st == ST_BAD_ARG);
    x[0] = 0; put16(x + 1, 4000); sendCmdBuf(OP_H_SP, x, 3); waitAck(OP_H_SP);     // restore default setpoint 40 C
  }
  Serial.printf("== %d passed, %d failed ==\n", tPass, tFail);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);                                  // header pin "RX" (GPIO1): wire it to the Nebulizer FUSE_EN pad (= ESP32 UART0 TX) to read its boot log
  pinMode(PIN_CAN_STANDBY, OUTPUT); digitalWrite(PIN_CAN_STANDBY, false);   // transceiver on
  pinMode(PIN_CAN_RESET, OUTPUT);   digitalWrite(PIN_CAN_RESET, true);      // nRESET high = run
  uint32_t t = millis(); while (!Serial && millis() - t < 2000) delay(10);
  if (!mcp.begin(BITRATE)) { while (1) { Serial.println("MCP2515 init FAILED"); delay(1000); } }
  Serial.printf("FEATHER CAN BRIDGE READY %d kbit/s  (commands: STAT STREAM,<ms> RATES RAW,1 TEST ARM,1 + board commands)\n", CAN_KBPS);
}

void loop() {
  static char buf[80]; static int n = 0; uint32_t now = millis();
  pump(); rateTick();
  { static char ub[200]; static int un = 0; static uint32_t ut = 0;      // forward the Nebulizer's UART0 boot log to USB as "UART0: ..." lines
    while (Serial1.available()) { char c = Serial1.read(); ut = now; if (c == '\n' || c == '\r') { if (un) { ub[un] = 0; Serial.printf("UART0: %s\n", ub); un = 0; } } else if (un < (int)sizeof(ub) - 1) ub[un++] = (c >= 32 && c < 127) ? c : '.'; }
    if (un && now - ut > 100) { ub[un] = 0; Serial.printf("UART0: %s\n", ub); un = 0; } }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (n) { buf[n] = 0; handleCmd(buf); } n = 0; }
    else if (n < (int)sizeof(buf) - 1) buf[n++] = c;
  }
  if (armed && now - lastPing >= 500) { lastPing = now; sendCmd(OP_PING, 0, 0, 0, 0, 0, 0, 0, 1); }
  if (jsonMs && now - lastJson >= jsonMs) { lastJson = now; printJson(); }
  if (streamMs && now - lastStream >= streamMs) { lastStream = now; printStatus(); }
  { static uint32_t lastErr = 0; static bool seen = false; if (!seen && ackNew) { seen = true; }
    if (ackNew && now - lastErr > 0) { ackNew = false; lastErr = now;
      Serial.printf("ACK op=0x%02X %s\n", ackOp, ackSt == ST_OK ? "ok" : ackSt == ST_BUSY ? "BUSY (USB has control)" : ackSt == ST_REJECTED ? "REJECTED (trip latched?)" : ackSt == ST_BAD_ARG ? "BAD_ARG" : "UNKNOWN_OP"); } }
}
