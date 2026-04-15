# GUI — Web Control Interface

## Overview

The GUI is a single-page HTML/CSS/JavaScript application embedded as a raw string literal inside the ESP32 firmware. It is served from `http://192.168.4.1` over the ESP32's WiFi access point. No internet connection is required. The GUI is fully responsive and works on phones, tablets, and laptops.

The GUI polls the ESP32 `/status` endpoint every 500ms to update telemetry. Commands are sent as HTTP GET requests to `/{command}` paths.

## Layout — Responsive Breakpoints

| Screen width | Layout |
|---|---|
| < 660px (phone) | Single column, all sections stacked |
| 660–1019px (tablet) | Manual tab: 2-column (helm left, telemetry right) |
| ≥ 1020px (laptop) | Auto tab: 2-column (mission left, timers+telemetry right); Params: 2-column |

## Top Bar (always visible, sticky)

| Element | Function |
|---|---|
| `SRV-01 \| CONTROL` | Vessel identifier label |
| **STOP** button | Calls `globalStop()` — sends `S`, stops timers, enters stop mode |
| **ARM** button | Calls `armEsc()` — sends `Z`, arms ESCs (no delay) |
| **Refresh** button | Calls `refresh()` — polls `/status` immediately |

## Tab Navigation

Three tabs: **Manual**, **Autonomous**, **Parameters**. Switching tabs triggers a `refresh()`.

---

## Manual Tab

### Helm (Joystick)

A canvas-drawn circular joystick. Drag or touch to control direction. Sends repeat commands every 200ms while held.

| Joystick direction | Command sent | ESP32 action |
|---|---|---|
| Up | `F` | `moveForward()` |
| Down | `B` | `moveBackward()` |
| Left | `L` | `turnLeft()` |
| Right | `R` | `turnRight()` |
| Released / centre | — | Motors stop (no command sent) |

**Engage Manual Mode** button sends `M`. Must be pressed after ARM to enable joystick.

### Sampling Operations

| Button | Command | Arduino action |
|---|---|---|
| Manual Sample | `A` | Sends `:M1\n`, sets `arduinoState = "Manual Sampling..."` |
| Pump On | `Q` | Sends `:M2\n` (manual pump, runs until STOP) |
| Return Zero | `I` | Sends `:B1\n` (retract drum to zero) |
| Sample Depth | `D` | Sends `:D1\n` (lower drum to set depth) |
| Depth Up | `H` | Sends `:F1\n` (raise 90°) |
| Depth Down | `G` | Sends `:E1\n` (lower 90°) |

### Turntable Position

Six amber buttons (0–5). Each sends the corresponding digit as a command, relayed to Arduino as `:P0\n`–`:P5\n`.

### Telemetry Card

Polls every 500ms. Displays:

| Field | Source |
|---|---|
| Mode | ESP32 mode: STOP / MANUAL / AUTONOMOUS |
| Heading | HMC5883L compass reading (degrees) |
| Left Motor | Last left motor power value |
| Right Motor | Last right motor power value |
| ESP32 State | Navigation sub-state (FORWARD, WATER SAMPLE, TURN LEFT, etc.) |
| Stuck | True/False — red highlight when stuck |
| Switch Hit | Last switch number hit (0 = none) |
| Samples | `sample_count` / `sample_max` |
| Arduino State | Live sampling platform status (see ESP32_MASTER.md for all states) |

---

## Autonomous Tab

### Pre-Mission Setup Card

All parameters are saved to browser `localStorage` and restored on page load.

| Control | ID | Command | Notes |
|---|---|---|---|
| No. of Samples | `missionSamples` | `NS:` | Select 1–5. Sent on change. ESP32 only. |
| Sample Depth | `missionDepth` | `SD:` | Select 0.0 / 0.2 / 0.5 / 1.0 / 1.5 / 2.0m. Idle guard applies. |
| Flush Time | `flushTime` | `FT:` | Number input 0–60s. Set button. Idle guard applies. |
| Fill Time | `fillTime` | `BT:` | Number input 0–60s. Set button. Idle guard applies. |
| Sample Interval | `sampleIntervalAuto` | `Y:` | +/- buttons, number input 1–60min. Synced with Parameters tab. |

**Idle guard**: Sample Depth, Flush Time, and Fill Time are only sent when `arduinoState === 'Ready'` or `'Sample Done'`. Otherwise a popup alert is shown: *"Parameter not sent - Arduino must be in idle state"*.

### Action Buttons

| Button | Function |
|---|---|
| **Start Auto Mode** | Sends `T`, starts elapsed and countdown timers |
| **Trigger Stuck** | Sends `C`, manually triggers stuck recovery sequence |

### Mission Timers Card

Dark-themed card with two live timers:

**Elapsed Time (HH:MM:SS)**
- Starts when **Start Auto Mode** is pressed
- Accumulates across the mission (does not reset on stuck or sampling pauses)
- Stops only on **STOP** or **Engage Manual Mode**

**Next Sample In (MM:SS)**
- Counts down from the set sample interval
- Pauses and shows `SAMPLING` / Water collection when `arduinoState` enters active sampling states
- Resets to full interval when `arduinoState` becomes `'Sample Done'`
- Continues counting during stuck manoeuvres (matches ESP32 C++ behaviour — the real timer never pauses for stuck)
- Shows `--:--` when autonomous mode is not running

### Telemetry Card

Same fields as Manual tab telemetry.

---

## Parameters Tab

Two-column layout (on wide screens). All parameters use `+` / `-` adjust buttons, a number input, and a **Set** button. Values saved to `localStorage`.

### Drive

| Parameter | ID | Command | Range | Default |
|---|---|---|---|---|
| Motor Power | `motorPower` | `S:` | 0–255 | 80 |
| Reverse Power | `reversePower` | `R:` | 0–255 | 60 |
| Turn Power | `turnPower` | `P:` | 0–255 | 30 |
| Motor Offset | `motorOffset` | `X:` | 0.5–2.0 | 1.0 |

### Stuck Recovery

| Parameter | ID | Command | Range | Default |
|---|---|---|---|---|
| Stop Time 1 | `stuckStop1` | `K:` | 50–1000ms | 400 |
| Reverse Time | `stuckReverse` | `D:` | 50–1000ms | 2000 |
| Stop Time 2 | `stuckStop2` | `B:` | 50–1000ms | 400 |
| Final Stop | `stuckFinal` | `O:` | 50–1000ms | 400 |

### Navigation Timing

| Parameter | ID | Command | Range | Default |
|---|---|---|---|---|
| Debounce | `debounce` | `E:` | 0–50ms | 30 |
| Min Turn | `minTurn` | `N:` | 0–10000ms | 3000 |
| Max Turn | `maxTurn` | `W:` | 0–10000ms | 6000 |
| Fwd Lock | `forwardLock` | `F:` | 0–10000ms | 2000 |
| Sample Interval | `sampleIntervalParams` | `Y:` | 1–60 min | 3 |

Sample Interval is synced bidirectionally with the Autonomous tab input.

---

## JavaScript Architecture

### Key Global Variables

| Variable | Purpose |
|---|---|
| `autoRunning` | True while autonomous mission is active |
| `elapsedAccum` | Accumulated elapsed ms (survives pause) |
| `elapsedRunStart` | `Date.now()` at last timer start |
| `cdTotalSecs` | Countdown seconds remaining |
| `cdPaused` | True during active sampling — freezes countdown display |
| `cdRunning` | True when countdown is active |
| `lastArduinoState` | Last received `arduino_state` value |
| `timerTick` | `setInterval` handle for 1-second timer |

### Key Functions

| Function | Purpose |
|---|---|
| `sendCommand(cmd)` | `fetch('/' + cmd)` + `refresh()` |
| `setParam(l, v)` | `fetch('/' + l + ':' + v)` + `localStorage.setItem(l, v)` |
| `sendSamplingParam(l, v)` | Idle guard check, then `setParam()` |
| `refresh()` | `fetch('/status')`, parse response, update DOM, call `handleArduinoStateTransition()` |
| `handleArduinoStateTransition(newState)` | Manages countdown pause/reset on state changes |
| `startTimers()` | Initialises and starts elapsed + countdown timers |
| `stopTimers()` | Accumulates elapsed, clears `timerTick` |
| `loadParams()` | Restores all `localStorage` values to input fields on page load |

### Parameter Persistence (localStorage keys)

| Key | Parameter |
|---|---|
| `S` | Motor Power |
| `R` | Reverse Power |
| `P` | Turn Power |
| `X` | Motor Offset |
| `K`, `D`, `B`, `O` | Stuck recovery times |
| `E`, `N`, `W`, `F` | Navigation timing |
| `Y` | Sample interval |
| `FT` | Flush time |
| `BT` | Fill time |
| `NS` | No. of samples |
| `SD` | Sample depth |
