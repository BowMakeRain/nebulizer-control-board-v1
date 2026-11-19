// Nebulizer board CAN protocol v1  (see CAN_PROTOCOL.md).
// 11-bit standard IDs, 500 kbit/s, all multi-byte fields little-endian.
// Canonical copy lives in bringup/; identical copies sit in NebulizerLab/ and FeatherCanBridge/ (keep them in sync).
#pragma once
#include <stdint.h>
#include <string.h>

namespace nc {
static const uint8_t PROTO_VER = 1;
// Bus speed shared by the board and the Feather (compile-time; both must match). Slow on purpose while the wiring is poor.
// 50 kbit/s is the slowest rate both the ESP32 TWAI and the MCP2515 (16 MHz) can do from their standard tables.
static const int CAN_KBPS = 50;
static const int DEFAULT_PERIOD_MS = CAN_KBPS >= 250 ? 100 : 250;   // telemetry period: 19 frames per period must fit the bus (~25 % load)
static const int NHEAT = 6;

// ---- IDs (lower = higher priority)
static const uint16_t ID_CMD    = 0x100;   // host  -> board: command (d0 = opcode)
static const uint16_t ID_OTA_DATA = 0x110; // host -> board: firmware chunk [seq24 LE][5 data bytes]
static const uint16_t ID_ACK    = 0x200;   // board -> host : [op, status, echo of cmd d1..d6]
static const uint16_t ID_OTA_ACK = 0x210; // board -> host: [next seq24][status][u32 bytes written]
static const uint16_t ID_SYS    = 0x300;   // board -> host : system / safety flags
static const uint16_t ID_HEAT   = 0x310;   // + channel 0..5 : heater state
static const uint16_t ID_POWER  = 0x320;   // + channel 0..5 : INA740 power monitor
static const uint16_t ID_PRESS  = 0x330;   // pressure x2 + their temperatures
static const uint16_t ID_ENV    = 0x331;   // flow sensor + SHT45
static const uint16_t ID_MOTOR  = 0x340;   // stepper driver state
static const uint16_t ID_MOTOR2 = 0x341;   // stepper target / accel
static const uint16_t ID_ADC    = 0x350;   // e-fuse analog + rates
static const uint16_t ID_DEV    = 0x360;   // I2C device health + CAN error counters
static const uint16_t ID_BOOT   = 0x701;   // 1 Hz with the heartbeat: last 4 reset reasons (newest first), boot count, flags (b0 brown-out reset disabled, b1 history valid)
static const uint16_t ID_HB     = 0x700;   // 1 Hz heartbeat (always, even if telemetry is off)
static const int OTA_PAYLOAD = 5, OTA_WINDOW = 16, OTA_BLOCK = 4000;   // bytes per frame, frames per ack, bytes per PC->Feather block
enum : uint8_t { OTA_OK = 0, OTA_ERR_WRITE = 1, OTA_ERR_VERIFY = 2, OTA_ERR_STATE = 3 };
static const int N_TELEM_FRAMES = 1 + 6 + 6 + 2 + 2 + 1 + 1;   // frames per telemetry period (19)

// ---- opcodes (command d0)
enum : uint8_t {
  OP_PING = 0x01,        // no args (every command also feeds the 3 s dead-man)
  OP_OFF = 0x02,         // all heaters off, motor stopped, solenoid closed
  OP_CLEAR_ALL = 0x03,   // clear all heater trips
  OP_FUSE = 0x04,        // d1: 1 = cut power (FUSE_EN low), 0 = power on
  OP_H_MODE = 0x10,      // d1 ch, d2 mode (0 off, 1 manual, 2 pid)
  OP_H_DUTY = 0x11,      // d1 ch, u16 duty x100 (%)
  OP_H_SP = 0x12,        // d1 ch, i16 setpoint x100 (C)
  OP_H_PID = 0x13,       // d1 ch, u16 kp x100, u16 ki x100, u16 kd x100
  OP_H_LIM = 0x14,       // d1 ch, u16 max duty x100 (%), i16 max temp x10 (C)
  OP_H_FREQ = 0x15,      // d1 ch, u16 PWM Hz
  OP_H_CLEAR = 0x16,     // d1 ch: clear that heater's trip
  OP_SOL = 0x20,         // d1: 1 = on (active high), 0 = off
  OP_M_SPEED = 0x30,     // u32 STEP Hz x10 (0 = stop)
  OP_M_ACCEL = 0x31,     // u32 Hz per second
  OP_M_CFG = 0x32,       // d1 nSLEEP(1 awake), d2 ENABLE(0 off,1 on,2 Hi-Z x8), d3 DIR, d4 STL_MODE(0..2), d5 stall report off(1)
  OP_M_CLEAR = 0x33,     // pulse nSLEEP to clear a driver fault
  OP_THERM = 0x40,       // u16 beta, u32 R0 (ohm)
  OP_RATE = 0x50,        // u16 telemetry period ms (0 = telemetry off, else 10..5000)
  OP_OTA_BEGIN = 0x60,   // u32 image size (bytes). Turns all outputs off, pauses telemetry, erases the spare app slot
  OP_OTA_END = 0x61,     // u32 CRC32 of the image: verify, switch boot slot, reboot. New image must hear from the host within 60 s or it rolls back
  OP_OTA_ABORT = 0x62,
  OP_OTA_INFO = 0x63,    // reply on ID_ACK: [0x63, 0, running slot, state (b0 pending-verify, b1 OTA in progress), u32 build id]
};
// ST_BUSY: USB is controlling the board (it spoke within USB_HOLD_MS); CAN control commands are refused,
// except OFF (always honoured) and PING (acked but does not extend the dead-man). Telemetry is sent on both links.
enum : uint8_t { ST_OK = 0, ST_BAD_ARG = 1, ST_UNKNOWN = 2, ST_REJECTED = 3, ST_BUSY = 4 };
static const uint32_t USB_HOLD_MS = 3000;

// ---- SYS flags (ID_SYS d4)
enum : uint8_t { SF_FUSE_PG = 1, SF_FUSE_FLT = 2, SF_INHIBIT = 4, SF_SOL = 8, SF_NFAULT_OK = 16, SF_STL_REP = 32, SF_DEADMAN_OK = 64,
                 SF_USB_ACTIVE = 128 };   // 1 = USB owns control (CAN commands get ST_BUSY), 0 = CAN may control
// ---- MOTOR flags (ID_MOTOR d6): b0 sleep, b1-2 enable mode, b3 dir, b4 stall-report-off, b5 nFAULT ok, b6 STL_REP

static const int16_t  INV16 = 0x7FFF;      // "no value" for signed 16-bit fields
static const uint16_t INVU16 = 0xFFFF;     // "no value" for unsigned 16-bit fields

inline void put16(uint8_t *d, uint16_t v) { d[0] = v & 0xFF; d[1] = v >> 8; }
inline void put32(uint8_t *d, uint32_t v) { d[0] = v; d[1] = v >> 8; d[2] = v >> 16; d[3] = v >> 24; }
inline uint16_t get16(const uint8_t *d) { return (uint16_t)d[0] | ((uint16_t)d[1] << 8); }
inline int16_t  geti16(const uint8_t *d) { return (int16_t)get16(d); }
inline uint32_t get32(const uint8_t *d) { return (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24); }
}  // namespace nc
