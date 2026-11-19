# Nebulizer CAN protocol v1

11-bit standard IDs, 500 kbit/s, little-endian. Definitions: `NebCanProto.h` (identical copies in `NebulizerLab/` and `FeatherCanBridge/`).
The board runs USB and CAN at the same time. **USB has priority**: while a USB host has sent anything in the last 3 s, CAN control
commands are acked `BUSY (4)` (except `OFF`, always honoured, and `PING`, acked but it doesn't extend the dead-man). When USB goes quiet,
CAN takes over automatically. Telemetry goes out on both links all the time. The 3 s dead-man applies to whichever link is in control.

## Host -> board `0x100`  (d0 = opcode)
| op | name | args |
|---|---|---|
| 01 | PING | – |
| 02 | OFF | – (heaters off, motor stop, solenoid closed) |
| 03 | CLEAR_ALL | – |
| 04 | FUSE | d1: 1 cut power, 0 on |
| 10 | H_MODE | d1 ch, d2 0 off/1 manual/2 pid |
| 11 | H_DUTY | d1 ch, u16 duty%x100 |
| 12 | H_SP | d1 ch, i16 setpoint Cx100 |
| 13 | H_PID | d1 ch, u16 kp x100, ki x100, kd x100 |
| 14 | H_LIM | d1 ch, u16 max duty%x100, i16 max T Cx10 |
| 15 | H_FREQ | d1 ch, u16 Hz |
| 16 | H_CLEAR | d1 ch |
| 20 | SOL | d1 1 on |
| 30 | M_SPEED | u32 step Hz x10 |
| 31 | M_ACCEL | u32 Hz/s |
| 32 | M_CFG | d1 nSLEEP, d2 ENABLE(0/1/2 Hi-Z), d3 DIR, d4 STL_MODE(0..2), d5 stall-report-off |
| 33 | M_CLEAR | – |
| 40 | THERM | u16 beta, u32 R0 |
| 50 | RATE | u16 telemetry period ms (0 off, 10..5000) |

## Board -> host
`0x200` ACK: `[op, status, d1..d6 of the command]`, status 0 ok, 1 bad arg, 2 unknown op, 3 rejected (latched trip), 4 busy (USB in control).

Telemetry (every `RATE` ms, default 100, frames spread evenly): invalid values are `0x7FFF` (signed) / `0xFFFF` (unsigned).
| ID | content |
|---|---|
| 0x300 | u32 uptime ms · flags (b0 FUSE_PG, b1 FUSE_FLT, b2 cut, b3 solenoid, b4 motor nFAULT ok, b5 STL_REP, b6 dead-man ok, b7 USB active) · reset reason · heater-on mask · trip mask |
| 0x310+ch | b0: mode(2b) · T_ok(b2) · trip(b4-6); i16 T x100; i16 SP x100; u16 duty x100; u8 seq |
| 0x320+ch | u16 Vbus mV; i16 mA; u16 W x100; i16 die C x10 |
| 0x330 | i16 pressure0 psi x100, pressure1, temp0 x100, temp1 |
| 0x331 | i16 flow ml/min x100, flow T x100, SHT45 T x100; u16 RH x100 |
| 0x340 | u32 step Hz; u16 TRQ mV; flags (b0 sleep, b1-2 enable, b3 dir, b4 stall-report-off, b5 nFAULT ok, b6 STL_REP); u8 STL_MODE |
| 0x341 | u32 target Hz x10; u32 accel |
| 0x350 | u16 e-fuse IMON mV; u16 e-fuse TEMP mV; u16 I2C kHz; u16 period ms |
| 0x360 | u16 live mask; u16 ok mask; u16 I2C cycle ms; u8 TEC; u8 REC |
| 0x700 | heartbeat 1 Hz: 'N', version, ..., u32 uptime (sent even when telemetry is off) |

USB-side extras: `CANP,<ms>` sets the CAN telemetry period; the JSON `sys` has `ctl_src` ("usb" / "can" / "none") and `can_ms`.

## Feather RP2040 CAN (`FeatherCanBridge/`)
USB serial 115200. Takes the same commands as the board's USB (`M,0,2`, `S,0,5`, `SOL,1`, `MS,400`, `OFF`, ...) and sends them over CAN.
Extras: `ARM,1` (PING every 500 ms), `STAT`, `STREAM,<ms>`, `RATES`, `RAW,1`, `TEST` (non-energizing self-test).

## Firmware update over CAN (board app only)
`tools/can_update.sh` builds `NebulizerLab` and pushes it through the Feather (`tools/can_ota.py --bin <file>` sends an existing .bin).
Close the GUI bridge first (it owns the Feather's port). The board must have USB idle (CAN is refused while a host talks over USB).

* Frames: `OP_OTA_BEGIN` (u32 size) -> board turns everything off, pauses telemetry, erases the spare app slot, then
  `0x110` data frames `[seq24][5 bytes]` with an ack on `0x210` every 16 frames (go-back-N), then `OP_OTA_END` (u32 CRC32).
  `OP_OTA_INFO` replies slot / pending-verify / build id. ~392 KB takes about 50 s.
* Safety: the image goes to the *other* app slot; the running one is untouched until CRC and image checks pass.
  The new image boots "pending verify" and must receive any CAN command from the host within 60 s (the script sends PINGs),
  otherwise it rolls back to the previous firmware. A new image that breaks CAN (pins, bitrate) therefore undoes itself.
  Tested: update + confirm, and rollback when the new image hears nothing.
* Limits: only the application is replaced (bootloader / partition table need USB). A USB flash (`esp32.sh upload`) resets the boot slot.
  Build id: `FW_TAG=<name> tools/can_update.sh` tags an image (shown as `sys.fw` / `sys.slot` / `sys.ota_pend` in the USB JSON).
