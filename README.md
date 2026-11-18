# Nebulizer Control Board V1

ESP32-S3 nebulizer PCB. All bring-up work so far lives in `bringup/`.

- `bringup/NebulizerLab/` – main firmware (I2C sensors, 6 PID heaters, motor, solenoid, CAN)
- `bringup/lab.html` – lab GUI (served by `tools/bridge.py` at http://localhost:8000)
- `bringup/FeatherCanTest/` – RP2040 Feather CAN test partner firmware
- `bringup/NebulizerBringup/`, `NebulizerDashboard/`, `dashboard.html`, `i2c_bringup.py` – earlier bring-up tools
- `bringup/hardware/NebulizerV1.NET` – netlist
- `bringup/tools/` – serial→web bridge and build/flash scripts (from the esp32-dev Claude skill)
- `bringup/esp32.conf` – board FQBN / sketch settings

Board: ESP32-S3-WROOM-1U, native USB. Set `PORT` explicitly if other boards are attached.
