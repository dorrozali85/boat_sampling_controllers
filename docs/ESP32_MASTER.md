# ESP32 Master — Navigation & Control

## Role

The ESP32 is the system master. It hosts the WiFi access point, serves the web GUI, controls the boat's motors, manages autonomous navigation **with closed-loop magnetometer heading control**, handles obstacle detection and stuck recovery, times the water sampling intervals, and communicates with the Arduino sampling platform over UART.

The firmware also supports a **no-sampler mode** for running missions without the Arduino Uno connected (see § No-Sampler Mode below).

## Firmware Files

| File | Purpose |
|---|---|
| `Roomba_project_V8_GUI_2_260326_fixed_4.ino` | Main firmware (production). Includes magnetometer heading nav, GUI-configurable no-sampler mode, 7s reverse-at-sample-start. |
| `Roomba_project_V8_disabled_uno.ino` | Standalone variant for bench testing without an Arduino Uno. Uses hardcoded `platform_sample_on_board=false` by default. |

Only one `.ino` file should be present in the Arduino sketch folder when compiling — move the other out of the folder before flashing.

## Hardware Pins

| GPIO | Function |
|---|---|
| 4 | ESC A (left motor) PWM |
| 14 | ESC B (right motor) PWM |
| 18 | ESC A reverse enable (yellow wire) |
| 19 | ESC B reverse enable (yellow wire) |
| 16 | UART2 RX ← Arduino TX (requires 5V→3.3V voltage divider) |
| 17 | UART2 TX → Arduino RX |
| 21 | I2C SDA (HMC5883L magnetometer) |
| 22 | I2C SCL (HMC5883L magnetometer) |
| 13 | Left microswitch (INPUT_PULLUP, LOW = hit) |
| 25 | Front microswitch (INPUT_PULLUP, LOW = hit) |
| 26 | Right microswitch (INPUT_PULLUP, LOW = hit) |
| 27 | LED Red (stuck state) |
| 12 | LED Green (manual mode) |
| 33 | LED Blue (autonomous mode) |
| 32 | LED White (stop mode) |

## WiFi

- Mode: Access Point
- SSID: `ESP32_Robot_HTML`
- Password: `12345678`
- IP: `192.168.4.1`
- GUI served at: `http://192.168.4.1`

## Operating Modes

The ESP32 is always in exactly one of three modes:

| Mode | Variable | Description |
|---|---|---|
| Stop | `stopMode = true` | Default on boot. Motors off. Only ARM and mode transitions accepted. |
| Manual | `manualMode = true` | Joystick and sampling operations active. |
| Autonomous | `autonomousMode = true` | Boat navigates forward with closed-loop heading control, avoids obstacles, samples at interval. |

## Motor Control

Motors are driven via PWM Servo signals to ESCs. Forward direction is set by writing a neutral signal (1200µs) to the reverse enable lines. Reverse is engaged by writing 1900µs to both reverse enable lines.

| Signal | Neutral/Forward | Reverse |
|---|---|---|
| ESC main (escA/escB) | 1105–1500µs (proportional to power) | Same — power applied via reverse enable |
| ESC reverse enable (escAReverse/escBReverse) | 1200µs (reverse OFF) | 1900µs (reverse ON) |

`MAX_SAFE_PULSE = 1500µs` — hard throttle cap.

ESCs must be armed before use. ARM sends 1100µs to all four ESC lines. The `delay(2000)` originally in the ARM sequence has been **removed** to prevent HTTP server starvation — ESCs arm without it.

## Magnetometer-Based Heading Navigation

The HMC5883L magnetometer provides closed-loop heading control during autonomous mode. **Replaces the previous open-loop time-based turn approach.**

### Concept

- On **Start Auto Mode** (`T` command), the ESP32 reads the current compass heading and locks it as `targetHeading`.
- During **forward motion**, a P-controller continuously biases the left/right motor differential to hold the boat on `targetHeading`.
- On **stuck detection**, the boat picks a random turn angle from `[min_turn_angle, max_turn_angle]` and updates `targetHeading` (turning right ⇒ `+angle`, turning left ⇒ `−angle`). The boat then turns under magnetometer feedback until it reaches the new heading (or `turn_timeout_ms` elapses).

### Helpers

| Function | Purpose |
|---|---|
| `readHeading()` | Reads `mag.getEvent()`, applies 4° East declination, wraps to 0–360°, stores in `currentHeading` |
| `headingError(target, current)` | Returns wrapped −180..+180 error. Positive ⇒ need to turn right (CW). |
| `computeStuckTurnTarget(switchHit)` | Picks random angle, updates `targetHeading` according to switch direction |

### Closed-Loop Forward (`moveForwardAutonomous`)

```
err        = headingError(targetHeading, currentHeading)
correction = heading_kp * err
leftPwr    = constrain(Motor_Power + correction, 0, 255)
rightPwr   = constrain(Motor_Power - correction, 0, 255)
```

Both reverse-enable lines are held at 1200µs (forward). Motor differential alone produces gentle heading correction.

### Stuck Turn Direction Logic

| Switch hit | Action on `targetHeading` | Boat turn |
|---|---|---|
| Left (1) | `+ randomAngle` | Right (CW) |
| Front (3) | `+ randomAngle` | Right (CW) — always |
| Right (5) | `− randomAngle` | Left (CCW) |

`randomAngle ∈ [min_turn_angle, max_turn_angle]` (uniform random, units = degrees).

## Autonomous Navigation Logic

`loop()` executes this priority chain while in autonomous mode:

1. If `stuckDetected` → run `handleStuckNonBlocking()`
2. Else if `sampling` → run sample handler (real or simulated, see § No-Sampler Mode). Both handlers begin with `moveBackward()` for `reverse_after_sample_duration` seconds — there is **no pre-stop** in the outer `loop()` (a previous pre-stop was clobbering the reverse and has been removed)
3. Else if `aligningAfterSample` → closed-loop turn back to the locked `targetHeading` until \|err\| < `turn_tolerance_deg` or `turn_timeout_ms` elapses, then resume forward
4. Else → `moveForwardAutonomous()` (closed-loop), check switches, check sample interval timer

### Manual Stuck Triggers

In autonomous mode the GUI exposes two manual stuck triggers (the old single-direction `C` command has been removed):

| URL command | Action |
|---|---|
| `/CL` | Sets `stuckDetected = true` and calls `computeStuckTurnTarget(5)` → boat turns LEFT (CCW) |
| `/CR` | Sets `stuckDetected = true` and calls `computeStuckTurnTarget(1)` → boat turns RIGHT (CW) |

### Stuck Recovery Sequence

Triggered when a microswitch fires and `forward_lock_ms` has elapsed since the last lock reset, or via the `/CL` / `/CR` manual triggers.

| Step | Action | Duration |
|---|---|---|
| 1 | Stop | `STUCK_STOP_TIME_1` (default 400ms) |
| 2 | Reverse | `STUCK_REVERSE_TIME` (default 2000ms) |
| 3 | Stop | `STUCK_STOP_TIME_2` (default 400ms) |
| 4 | **Heading-controlled turn** — turns until \|err\| < `turn_tolerance_deg` or `turn_timeout_ms` elapses | dynamic (compass-driven) |
| 5 | Final stop | `STUCK_FINAL_STOP_TIME` (default 400ms) |

After stuck recovery, `forwardLockStart` is reset to give the boat a grace period before microswitches are live again. The boat resumes forward motion using closed-loop tracking of the **new** `targetHeading` set in step 4.

**Sample-interval timer pause:** On stuck entry (`stuckStep == 0`) the firmware records `stuckPauseStart = millis()`. On stuck exit (step 5 → 0), `sampleStart` is shifted forward by the elapsed stuck duration. The effect is that the sample-interval countdown **pauses for the full stuck event** and resumes from where it left off — preventing the boat from finishing stuck recovery and immediately rolling over into a sample with no useful forward navigation between events.

### Post-Sample Heading Alignment

After every successful sample (real-Uno `:X1` ack OR no-sampler self-ack), the firmware sets `aligningAfterSample = true` and `alignStart = millis()`. The boat does **not** resume forward immediately — instead it enters the alignment branch (priority 3 above) and turns under closed-loop magnetometer control until either:

- \|`headingError`\| < `turn_tolerance_deg` (success — boat is back on the locked `targetHeading`), or
- `millis() - alignStart >= turn_timeout_ms` (timeout — give up and resume forward anyway)

This compensates for boat rotation during the ~136s sample wait (wind, current, drum drag) and ensures the boat returns to its pre-sample heading before continuing the survey leg. `STATE` reads `ALIGN→{deg}°` while this is active.

## Sampling

`sampling` flag is set when `millis() - sampleStart >= sample_interval_ms`. The boat enters the sampling branch directly (no pre-stop), then **always reverses for `reverse_after_sample_duration` seconds** (kills forward momentum), then either calls `performAutoSample()` (Uno present) or runs the simulated path (no Uno).

### Reverse-At-Sample-Start

At every autonomous sample trigger, regardless of `isWaterSamplerOnBoard`:

1. `moveBackward()` is invoked
2. After `reverse_after_sample_duration` seconds (default 7s), motors stop
3. Real or simulated sample wait proceeds

This applies in both code paths:
- **Inside `performAutoSample()`** wait loop (when Uno is on board) — runs at the start of the ~136s real sampling cycle
- **Inside the non-blocking skip path in `loop()`** (when Uno is off board) — overlaps the self-ack delay window

### performAutoSample() — Real Arduino Path

1. Checks `sample_count < sample_max` — exits autonomous mode if limit reached
2. Flushes Serial2 buffer
3. Sends `:M1\n` to Arduino
4. Sets `arduinoState = "Auto Sampling..."`
5. **Reverses motors** for `reverse_after_sample_duration` seconds (non-blocking inside the wait loop)
6. Enters wait loop (up to `TIMEOUT_MS = 200000ms`) calling `server.handleClient()` each iteration to keep GUI alive
7. Parses every Serial2 line received — maps Arduino status strings to `arduinoState` for live GUI display
8. On `:X1` received: increments `sample_count`, resets `sampleStart`, clears `sampling` flag
9. On timeout or user STOP: sets `arduinoState` to error string, exits autonomous mode

### Live Arduino State Mapping (during performAutoSample wait loop)

| Arduino serial print | GUI arduinoState |
|---|---|
| `Starting auto sample cycle` | `Starting cycle` |
| `Turning to home position` | `Turning to home` |
| `Drum lowering to sample depth` | `Lowering drum` |
| `Drum at depth - settling before flush` | `Drum at depth` |
| `Flushing at depth` | `Flushing hose` |
| `Flush done - settling` | `Flush settling` |
| `Turning to bottle (N)` | `Turning to bottle` |
| `At bottle - settling before sample` | `At bottle` |
| `Sampling bottle` | `Sampling bottle` |
| `Sample done - settling` | `Sample settling` |
| `Returning turntable to home` | `Returning home` |
| `Turntable home - retracting drum` | `Retracting drum` |
| `Cycle complete - sent :X1` | `Cycle complete` |
| `:X1` | `Sample Done` |
| `:FULL` | `Memory Full` |

## No-Sampler Mode

`isWaterSamplerOnBoard` (default `true`) controls whether the ESP32 communicates with the Arduino Uno during autonomous sampling. When `false`, the ESP32 runs autonomous missions stand-alone by simulating sample completion.

### Behavior when `isWaterSamplerOnBoard = false`

The non-blocking skip path in `loop()` replaces `performAutoSample()`:

1. On `sampling = true`:
   - Calls `moveBackward()` to start reverse phase
   - Sets `arduinoState = "Auto Sampling..."` (so GUI countdown pauses)
   - Records `skipSampleStart = millis()`
2. After `reverse_after_sample_duration` seconds: stops motors (reverse phase end)
3. After `skipSampleDelay` total milliseconds: simulates `:X1` arrival
   - Increments `sample_count`, resets `sampleStart`, clears `sampling`
   - Sets `arduinoState = "Sample Done"` (GUI countdown resets)
4. If `sample_count >= sample_max`: exits autonomous mode

`isWaterSamplerOnBoard = true` keeps the original full Arduino handshake path unchanged.

### Standalone Variant: `Roomba_project_V8_disabled_uno.ino`

A separate firmware file is provided for bench testing without any Uno. Differences from the main firmware:
- Default `platform_sample_on_board = false` (equivalent of `isWaterSamplerOnBoard`)
- Hardcoded `replace_water_sample_delay = 15` seconds
- Self-ack happens **inside** `performAutoSample()`'s wait loop (not in a separate skip path) — minimal-change variant
- All three params (sampler toggle, self-ack delay, reverse duration) are configurable via GUI

The main firmware is preferred for production. The disabled_uno variant is kept for reference and as a minimal-change bench-test build.

## Serial2 (UART to Arduino)

- Baud: 115200
- RX: GPIO16, TX: GPIO17
- In manual mode: `readSerial2NonBlocking()` monitors for `:X1` and `:FULL`
- In autonomous mode (Uno on board): `performAutoSample()` owns Serial2 exclusively
- In autonomous mode (Uno off board): no Serial2 traffic during simulated samples

### Commands sent to Arduino

| Command | Trigger | Effect |
|---|---|---|
| `:M1\n` | Auto sample or Manual Sample button | Start auto sample cycle |
| `:M2\n` | Pump On button | Start manual pump (runs until `:C1`) |
| `:B1\n` | Return Zero button | Retract drum to zero |
| `:D1\n` | Sample Depth button | Lower drum to `SAMPLE_DEPTH_METERS` |
| `:E1\n` | Depth Down button | Lower drum 90° |
| `:F1\n` | Depth Up button | Raise drum 90° |
| `:C1\n` | STOP button | Emergency stop all Arduino motion |
| `:P0`–`:P5\n` | Turntable 0–5 buttons | Rotate turntable to position |
| `:SD{val}\n` | Sample Depth select change | Set Arduino `SAMPLE_DEPTH_METERS` |
| `:FT{val}\n` | Flush Time Set button | Set Arduino `FLUSH_TIME_MS` (seconds) |
| `:BT{val}\n` | Fill Time Set button | Set Arduino `SAMPLE_TIME_MS` (seconds) |

## Tunable Parameters

All parameters are adjustable at runtime via the GUI and persist in browser localStorage.

### Drive

| Parameter | Variable | Default | Command | Range |
|---|---|---|---|---|
| Motor Power | `Motor_Power` | 80 | `S:` | 0–255 |
| Reverse Power | `Reverse_Power` | 60 | `R:` | 0–255 |
| Turn Power | `Turn_Power` | 30 | `P:` | 0–255 |
| Motor Offset | `Motor_Offset` | 1.0 | `X:` | 0.5–2.0 |

### Stuck Recovery (Timing)

| Parameter | Variable | Default | Command | Range |
|---|---|---|---|---|
| Stop Time 1 | `STUCK_STOP_TIME_1` | 400ms | `K:` | 50–1000ms |
| Reverse Time | `STUCK_REVERSE_TIME` | 2000ms | `D:` | 50–1000ms |
| Stop Time 2 | `STUCK_STOP_TIME_2` | 400ms | `B:` | 50–1000ms |
| Final Stop | `STUCK_FINAL_STOP_TIME` | 400ms | `O:` | 50–1000ms |

### Heading Control & Stuck Turn (NEW)

| Parameter | Variable | Default | Command | Range |
|---|---|---|---|---|
| Min Turn Angle | `min_turn_angle` | 30° | `MA:` | 5–180° |
| Max Turn Angle | `max_turn_angle` | 90° | `XA:` | 5–180° |
| Turn Tolerance | `turn_tolerance_deg` | 5.0° | `TT:` | 1–30° |
| Heading Kp | `heading_kp` | 1.5 | `KP:` | 0.1–10 |
| Turn Timeout | `turn_timeout_ms` | 8s | `TM:` | 1–30s |

### Navigation Timing

| Parameter | Variable | Default | Command | Range |
|---|---|---|---|---|
| Debounce | `debounce_ms` | 30ms | `E:` | 0–50ms |
| Forward Lock | `forward_lock_ms` | 2000ms | `F:` | 0–10000ms |
| Sample Interval | `sample_interval_ms` | 60000ms | `Y:` | 1–60 min |

### Sampling Platform

| Parameter | Variable | Default | Command | Relay to Arduino |
|---|---|---|---|---|
| No. of Samples | `sample_max` | 5 | `NS:` | No (ESP32 only) |
| Sample Depth | `sample_depth_m` | 0.2m | `SD:` | Yes → `:SD` |
| Flush Time | `flush_time_ms` | 30s | `FT:` | Yes → `:FT` |
| Fill Time | `fill_time_ms` | 30s | `BT:` | Yes → `:BT` |

**Note:** Sampling platform parameters (SD, FT, BT) are only sent to the Arduino when `arduinoState` is `"Ready"` or `"Sample Done"`. Sending while the Arduino is active will show a popup alert and the parameter will not be transmitted.

### Water Sampler Mode (NEW)

| Parameter | Variable | Default | Command | Range |
|---|---|---|---|---|
| Sampler On Board | `isWaterSamplerOnBoard` | `true` | `WS:` | 0 or 1 |
| Skip Delay | `skipSampleDelay` | 10s | `SST:` | 1–120s |
| Reverse Duration | `reverse_after_sample_duration` | 7s | `RSD:` | 0–30s |

`Skip Delay` only takes effect when Sampler On Board is OFF. `Reverse Duration` always applies (real and simulated sampling paths both reverse at sample start).

## Removed Parameters (formerly present, now obsolete)

| Removed | Replaced by |
|---|---|
| `min_turn_ms` (`N:`) | `min_turn_angle` (`MA:`) |
| `max_turn_ms` (`W:`) | `max_turn_angle` (`XA:`) |
| `currentTurnDuration` (internal) | Heading-controlled turn |
| `turnDirection` (internal char) | `targetHeading` (float) |
| `setTurnDirection(switchHit)` | `computeStuckTurnTarget(switchHit)` |
| `lastHeading` (internal) | `currentHeading` |

## Status Reporting

`handleStatus()` responds to GET `/status` with a plain-text key:value payload polled every 500ms by the GUI:

```
MODE:AUTONOMOUS
LEFT:80
RIGHT:80
ANGLE:245
TARGET:243
STUCK:False
STATE:FORWARD
SWITCH_HIT:0
ARDUINO:Sampling bottle
SAMPLE_COUNT:2
```

`TARGET` is the locked target heading the boat is closed-loop tracking.

| `STATE` value | Condition |
|---|---|
| `STOP` | Stop mode |
| `MANUAL` | Manual mode |
| `FORWARD` | Autonomous, closed-loop forward |
| `FORWARD LOCK {N}ms` | Forward but switches still in grace period |
| `WATER SAMPLE` | Sample wait (real or simulated) in progress |
| `ALIGN→{deg}°` | Post-sample heading realignment in progress |
| `STOP1` / `REVERSE` / `STOP2` / `TURN→{deg}°` / `FINAL STOP` | Active stuck-recovery sub-step |
