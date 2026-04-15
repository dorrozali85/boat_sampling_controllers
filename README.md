# SRV-01 AquaBot — Autonomous Water Sampling Vessel

## Project Overview

SRV-01 is an autonomous surface vessel designed for aquaculture water quality monitoring. The boat navigates a water body independently, stops at timed intervals, and collects water samples at programmable depths into up to five separate bottles using a stepper-motor-driven sampling platform.

## System Architecture

The system uses two microcontrollers communicating over UART:

| Controller | Role | File |
|---|---|---|
| ESP32 | Navigation master, WiFi AP, web GUI host, motor control | `Roomba_project_V8_GUI_2_260326_fixed_4.ino` |
| Arduino Uno | Sampling platform slave, stepper motor control (CNC Shield v3) | `Nema_17_W_CNC_shield_slave_fixed_280326_fix_4.ino` |

## Repository Contents

```
SRV01_AquaBot/
├── README.md                                        ← This file
├── Roomba_project_V8_GUI_2_260326_fixed_4.ino      ← ESP32 master (DO NOT MODIFY)
├── Nema_17_W_CNC_shield_slave_fixed_280326_fix_4.ino ← Arduino slave (DO NOT MODIFY)
├── docs/
│   ├── ESP32_MASTER.md     ← ESP32 logic, pins, commands, parameters
│   ├── ARDUINO_SLAVE.md    ← Arduino role, state machine, hardware
│   ├── GUI.md              ← Web interface description and controls
│   └── REQUIREMENTS.md     ← System and user interface requirements
```

## Hardware Summary

- **Boat hull**: RC boat with dual brushless motors and ESCs
- **Navigation MCU**: ESP32 (WiFi AP at 192.168.4.1, SSID: ESP32_Robot_HTML, pass: 12345678)
- **Sampling MCU**: Arduino Uno + CNC Shield v3.0
- **Sampling axes**: X = drum/depth (NEMA 17), Y = pump (NEMA 17), Z = turntable (NEMA 17)
- **Sensor**: HMC5883L magnetometer (I2C, heading)
- **Switches**: 3× microswitches (left/front/right obstacle detection)
- **LEDs**: Red (stuck), Green (manual), Blue (autonomous), White (stop)

## Quick Start

1. Power on ESP32 boat controller
2. Connect phone/tablet to WiFi: **ESP32_Robot_HTML** / **12345678**
3. Navigate to **http://192.168.4.1**
4. Press **ARM** to arm ESCs
5. Press **STOP → Engage Manual Mode** for manual control
6. Press **STOP → Start Auto Mode** in the Autonomous tab for autonomous mission

## Version

- ESP32 firmware: v28.3 / GUI v8.4
- Arduino firmware: v28.3
- Last updated: March 2026
