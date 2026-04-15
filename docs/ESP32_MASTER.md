# ESP32 Master — Navigation & Control

## Role

The ESP32 is the system master. It hosts the WiFi access point, serves the web GUI, controls the boat's motors, manages autonomous navigation, handles obstacle detection and stuck recovery, times the water sampling intervals, and communicates with the Arduino sampling platform over UART.

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
| Autonomous | `autonomousMode = true` | Boat navigates forward, avoids obstacles, samples at interval. |

## Motor Control

Motors are driven via PWM Servo signals to ESCs. Forward direction is set by writing a neutral signal (1200µs) to the reverse enable lines. Reverse is engaged by writing 1900µs to both reverse enable lines.

| Signal | Neutral/Forward | Reverse |
|---|---|---|
| ESC main (escA/escB) | 1105–1500µs (proportional to power) | Same — power applied via reverse enable |
| ESC reverse enable (escAReverse/escBReverse) | 1200µs (reverse OFF) | 1900µs (reverse ON) |

`MAX_SAFE_PULSE = 1500µs` — hard throttle cap.

ESCs must be armed before use. ARM sends 1100µs to all four ESC lines. The `delay(2000)` originally in the ARM sequence has been **removed** to prevent HTTP server starvation — ESCs arm without it.

## Autonomous Navigation Logic

`loop()` executes this priority chain while in autonomous mode:

1. If `stuckDetected` → run `handleStuckNonBlocking()`
2. Else if `sampling` → stop motors, run `performAutoSample()` (blocking wait up to 200s)
3. Else → `moveForwardAutonomous()`, check switches, check sample interval timer

### Stuck Recovery Sequence

Triggered when a microswitch fires and `forward_lock_ms` has elapsed since last lock reset.

| Step | Action | Duration |
|---|---|---|
| 1 | Stop | `STUCK_STOP_TIME_1` (default 400ms) |
| 2 | Reverse | `STUCK_REVERSE_TIME` (default 2000ms) |
| 3 | Stop | `STUCK_STOP_TIME_2` (default 400ms) |
| 4 | Turn (random duration, direction based on which switch hit) | `min_turn_ms`–`max_turn_ms` |
| 5 | Final stop | `STUCK_FINAL_STOP_TIME` (default 400ms) |

After stuck recovery, `forwardLockStart` is reset to give the boat a grace period before microswitches are live again.

### Turn Direction Logic

- Left switch hit → turn right
- Right switch hit → turn left
- Front switch hit → random direction

## Sampling

`sampling` flag is set when `millis() - sampleStart >= sample_interval_ms`. The boat stops and calls `performAutoSample()`.

### performAutoSample()

1. Checks `sample_count < sample_max` — exits autonomous mode if limit reached
2. Flushes Serial2 buffer
3. Sends `:M1\n` to Arduino
4. Sets `arduinoState = "Auto Sampling..."`
5. Enters wait loop (up to `TIMEOUT_MS = 200000ms`) calling `server.handleClient()` each iteration to keep GUI alive
6. Parses every Serial2 line received — maps Arduino status strings to `arduinoState` for live GUI display
7. On `:X1` received: increments `sample_count`, resets `sampleStart`, clears `sampling` flag
8. On timeout or user STOP: sets `arduinoState` to error string, exits autonomous mode

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

## Serial2 (UART to Arduino)

- Baud: 115200
- RX: GPIO16, TX: GPIO17
- In manual mode: `readSerial2NonBlocking()` monitors for `:X1` and `:FULL`
- In autonomous mode: `performAutoSample()` owns Serial2 exclusively

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

### Navigation Parameters

| Parameter | Variable | Default | Command | Range |
|---|---|---|---|---|
| Motor Power | `Motor_Power` | 80 | `S:` | 0–255 |
| Reverse Power | `Reverse_Power` | 60 | `R:` | 0–255 |
| Turn Power | `Turn_Power` | 30 | `P:` | 0–255 |
| Motor Offset | `Motor_Offset` | 1.0 | `X:` | 0.5–2.0 |
| Debounce | `debounce_ms` | 30 | `E:` | 0–50ms |
| Min Turn Time | `min_turn_ms` | 3000 | `N:` | 0–10000ms |
| Max Turn Time | `max_turn_ms` | 6000 | `W:` | 0–10000ms |
| Forward Lock | `forward_lock_ms` | 2000 | `F:` | 0–10000ms |
| Sample Interval | `sample_interval_ms` | 60000 | `Y:` | 1–60 min |

### Stuck Recovery Parameters

| Parameter | Variable | Default | Command |
|---|---|---|---|
| Stop Time 1 | `STUCK_STOP_TIME_1` | 400ms | `K:` |
| Reverse Time | `STUCK_REVERSE_TIME` | 2000ms | `D:` |
| Stop Time 2 | `STUCK_STOP_TIME_2` | 400ms | `B:` |
| Final Stop | `STUCK_FINAL_STOP_TIME` | 400ms | `O:` |

### Sampling Platform Parameters

| Parameter | Variable | Default | Command | Relay to Arduino |
|---|---|---|---|---|
| No. of Samples | `sample_max` | 5 | `NS:` | No (ESP32 only) |
| Sample Depth | `sample_depth_m` | 0.2m | `SD:` | Yes → `:SD` |
| Flush Time | `flush_time_ms` | 30s | `FT:` | Yes → `:FT` |
| Fill Time | `fill_time_ms` | 30s | `BT:` | Yes → `:BT` |

**Note:** Sampling platform parameters (SD, FT, BT) are only sent to the Arduino when `arduinoState` is `"Ready"` or `"Sample Done"`. Sending while the Arduino is active will show a popup alert and the parameter will not be transmitted.

## Status Reporting

`handleStatus()` responds to GET `/status` with a plain-text key:value payload polled every 500ms by the GUI:

```
MODE:AUTONOMOUS
LEFT:80
RIGHT:80
ANGLE:245
STUCK:False
STATE:FORWARD
SWITCH_HIT:0
ARDUINO:Sampling bottle
SAMPLE_COUNT:2
```
