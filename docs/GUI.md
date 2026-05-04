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
| Target | Locked target heading the boat is closed-loop tracking (degrees) |
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
| **Stuck Left** | Sends `CL` — manually triggers stuck recovery turning the boat LEFT (CCW) by a random angle in `[min_turn_angle, max_turn_angle]` |
| **Stuck Right** | Sends `CR` — manually triggers stuck recovery turning the boat RIGHT (CW) |

The single ambiguous `Trigger Stuck` button (`C` command) has been replaced by these two explicit-direction buttons so the operator can deliberately steer the boat away from a known obstacle.

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
- Effectively pauses during stuck recovery — the ESP32 firmware shifts `sampleStart` forward by the stuck duration on stuck exit, so the GUI countdown freezes for the duration of the stuck event and resumes from the same value (prevents stuck-then-immediately-sample chains)
- Shows `--:--` when autonomous mode is not running

### Telemetry Card

Same fields as Manual tab telemetry (including Target heading).

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
| Min Turn Angle | `minTurn` | `MA:` | 5–180° | 30 |
| Max Turn Angle | `maxTurn` | `XA:` | 5–180° | 90 |
| Fwd Lock | `forwardLock` | `F:` | 0–10000ms | 2000 |
| Sample Interval | `sampleIntervalParams` | `Y:` | 1–60 min | 3 |

Min/Max Turn Angle replaced the former Min Turn (`N:`, ms) and Max Turn (`W:`, ms) parameters. localStorage keys are `MA` and `XA` (not `N`/`W`).

Sample Interval is synced bidirectionally with the Autonomous tab input.

### Heading Control

| Parameter | ID | Command | Range | Default |
|---|---|---|---|---|
| Turn Tolerance | `turnTolerance` | `TT:` | 1–30° | 5.0 |
| Heading Kp | `headingKp` | `KP:` | 0.1–10 | 1.5 |
| Turn Timeout | `turnTimeout` | `TM:` | 1–30s | 8 |

- **Turn Tolerance**: How close (in degrees) the boat must be to the new target heading before the stuck turn is considered complete.
- **Heading Kp**: Proportional gain for the P-controller that biases left/right motor power during forward motion to hold the locked heading.
- **Turn Timeout**: Maximum time (seconds) allowed for a compass-controlled stuck turn before the boat gives up and resumes forward anyway.

### Water Sampler

| Parameter | ID | Command | Range | Default |
|---|---|---|---|---|
| Sampler On Board | `wsOn`/`wsOff` | `WS:` | 0 or 1 | ON (1) |
| Self-Ack Delay (s) | `sstDelay` | `SST:` | 1–120s | 10 |
| Reverse Duration (s) | `rsdDelay` | `RSD:` | 0–30s | 7 |

- **Sampler On Board**: Toggle ON/OFF buttons. ON (green) = ESP32 communicates with Arduino Uno. OFF (red) = autonomous missions run without Uno; sample completion is self-acknowledged.
- **Self-Ack Delay**: Only active when Sampler On Board is OFF. Time (seconds) the ESP32 waits before simulating a `:X1` completion signal.
- **Reverse Duration**: At every autonomous sample trigger (both real and simulated paths), the boat reverses for this many seconds to kill forward momentum before the sample wait.

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
| `toggleSampler(on)` | Sends `WS:1` or `WS:0`, saves to localStorage, updates ON/OFF button highlight |

### Parameter Persistence (localStorage keys)

| Key | Parameter |
|---|---|
| `S` | Motor Power |
| `R` | Reverse Power |
| `P` | Turn Power |
| `X` | Motor Offset |
| `K`, `D`, `B`, `O` | Stuck recovery times |
| `E`, `F` | Navigation timing (debounce, forward lock) |
| `MA`, `XA` | Min / Max turn angle (degrees) — replaces former `N`/`W` ms keys |
| `TT`, `KP`, `TM` | Heading control (tolerance, Kp, timeout) |
| `Y` | Sample interval |
| `FT` | Flush time |
| `BT` | Fill time |
| `NS` | No. of samples |
| `SD` | Sample depth |
| `WS` | Sampler On Board (0 or 1) |
| `SST` | Self-Ack Delay (s) |
| `RSD` | Reverse Duration (s) |

### TARGET Field Parsing

`refresh()` parses the `TARGET:` line from `/status` and updates a `p+'-target'` DOM element in both telemetry cards. This shows the locked compass heading the boat is closed-loop tracking (0–360°).
