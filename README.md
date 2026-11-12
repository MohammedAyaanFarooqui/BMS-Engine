# ESP32 BMS Protection Controller with Blynk Telemetry

4-cell battery monitor with protection relay, flicker-free LCD and
Blynk IoT telemetry (offline FIFO queue, risk score). Simulated in Wokwi.

## Features
- Median-filtered sampling with sensor-fault detection (range, frozen, jump)
- Adaptive imbalance threshold by SoC
- NORMAL / WARNING / FAULT / RECOVERY state machine driving relay, LEDs, buzzer
- 16x2 I2C LCD with differential rendering
- 26 Blynk datastreams, 20-event offline queue with ordered replay

## Hardware / Simulation
ESP32 DevKit-C, 4 potentiometers (GPIO 32-35), LEDs (15/16/17),
relay (GPIO 2), buzzer (GPIO 25), I2C LCD 0x27 (SDA 21, SCL 22).
Circuit: `simulation/diagram.json`. Wokwi project: <paste your link>

## Setup
1. Copy `firmware/bms_controller/secrets.h.example` to `secrets.h`
2. Fill in your Blynk template ID and auth token
3. Open `bms_controller.ino`, install libraries: Blynk, LiquidCrystal_I2C
4. Build and upload, or paste into Wokwi

## Testing
See `test-data/` for manual scenarios and 5 Hz timeline datasets with
expected outputs. Full details in `docs/Project_Report.pdf`.

## License
MIT