# Arduino Uno Slave — Water Sampling Platform

## Role

The Arduino Uno controls the physical water sampling platform. It receives commands from the ESP32 over UART, drives three NEMA 17 stepper motors via a CNC Shield v3.0, and executes a fully non-blocking state machine to perform the complete sample collection sequence. It reports status strings and completion signals back to the ESP32.

## Hardware

### CNC Shield v3.0 Pin Mapping

| Axis | Motor | STEP pin | DIR pin | Function |
|---|---|---|---|---|
| X | NEMA 17 | 2 | 5 | Drum / depth — lowers/raises hose |
| Y | NEMA 17 | 3 | 6 | Pump — draws water |
| Z | NEMA 17 | 4 | 7 | Turntable — rotates bottle carousel |
| ENABLE | Shared | 8 | — | LOW = all drivers enabled |

All axes use 3 microstepping jumpers installed on CNC Shield = 1/16 microstepping = **3200 steps/revolution**.

### Stepper Configuration

| Axis | Steps/Rev | Step Delay | Notes |
|---|---|---|---|
| X (drum) | 3200 | 1200µs | Bidirectional, position tracked in `posX` |
| Y (pump) | 800 | 80µs target, 300µs soft start | Always one direction, timed operation |
| Z (turntable) | 3200 | 1200µs | One direction only, position tracked in `posZ` |

### Depth Calibration

- 1 full revolution of drum = ~0.1m of hose
- 1 metre = 27825 steps (`STEPS_PER_METER`)
- Maximum safe depth: 2.0m (hard clamp)
- Default sample depth: 0.2m (configurable via `:SD` command)

### Turntable Positions

6 positions, each 60° apart:

| Position | Use |
|---|---|
| 0 | Home / flush position (waste) |
| 1–5 | Sample bottles 1–5 |

## Serial Protocol (115200 baud, 8N1)

### Commands received from ESP32

| Command | Action |
|---|---|
| `:M1\n` | Start auto sample cycle (if `sampleCount < MAX_SAMPLES`) |
| `:M2\n` | Start manual pump (runs until `:C1`) |
| `:B1\n` | Return drum to zero position |
| `:D1\n` | Lower drum to `SAMPLE_DEPTH_METERS` |
| `:E1\n` | Lower drum 90° (one step increment) |
| `:F1\n` | Raise drum 90° |
| `:C1\n` | Emergency stop — all motion halted, state machine reset to IDLE |
| `:P0`–`:P5\n` | Move turntable to position 0–5 |
| `:SD{float}\n` | Set sample depth in metres (0.0–2.0) |
| `:FT{int}\n` | Set flush pump duration in seconds (0–60) |
| `:BT{int}\n` | Set bottle fill pump duration in seconds (0–60) |

### Responses sent to ESP32

| Response | When sent |
|---|---|
| `:X1\n` | Cycle fully complete — drum physically retracted to zero |
| `:FULL\n` | Auto sample rejected — `sampleCount >= MAX_SAMPLES` |
| `OK\n` | Acknowledgement after most commands |
| Status strings | At each state machine transition (see state machine table) |

## State Machine

The `handleSampleStateMachine()` function runs every `loop()` iteration. It is fully non-blocking — all waits use `millis()` comparisons against `stateStartTime`.

### Complete Sequence

| State | Wait condition | Then |
|---|---|---|
| `STATE_IDLE` | — | Does nothing |
| `STATE_TURN_HOME` | — (immediate) | Calls `doTurntableTo(0)`, transitions to `STATE_DEPTH_DOWN` |
| `STATE_DEPTH_DOWN` | `targetZ == 0` (turntable at home) | Calls `doSampleDepthDown()`, transitions to `STATE_WAIT_DRUM_DEPTH` |
| `STATE_WAIT_DRUM_DEPTH` | `targetX == 0` (drum at sample depth) | Records `stateStartTime`, transitions to `STATE_PRE_FLUSH_SETTLE` |
| `STATE_PRE_FLUSH_SETTLE` | 5 seconds elapsed | Starts flush pump (`pumpIsFlush = true`), transitions to `STATE_FLUSH` |
| `STATE_FLUSH` | `!pumpRunning` (flush complete) | Records `stateStartTime`, transitions to `STATE_POST_FLUSH_SETTLE` |
| `STATE_POST_FLUSH_SETTLE` | 5 seconds elapsed | Calls `doTurntableTo(nextBottle)`, transitions to `STATE_TURN_NEXT` |
| `STATE_TURN_NEXT` | `targetZ == 0` (turntable at bottle) | Records `stateStartTime`, transitions to `STATE_PRE_SAMPLE_SETTLE` |
| `STATE_PRE_SAMPLE_SETTLE` | 5 seconds elapsed | Starts fill pump (`pumpIsFlush = false`), transitions to `STATE_REAL_SAMPLE` |
| `STATE_REAL_SAMPLE` | `!pumpRunning` (fill complete) | Records `stateStartTime`, transitions to `STATE_POST_SAMPLE_SETTLE` |
| `STATE_POST_SAMPLE_SETTLE` | 5 seconds elapsed | Increments `sampleCount`, calls `doTurntableTo(0)`, transitions to `STATE_TURN_HOME_FINAL` |
| `STATE_TURN_HOME_FINAL` | `targetZ == 0` (turntable at home) | Calls `doReturnToZero()`, transitions to `STATE_WAIT_DRUM_HOME` |
| `STATE_WAIT_DRUM_HOME` | `targetX == 0` (drum fully retracted) | Sets state to `STATE_IDLE`, sends `:X1\n` to ESP32 |

### Serial Prints at Each Transition

These are received by the ESP32 and displayed live in the GUI Arduino State field:

| State transition | Serial print |
|---|---|
| STATE_TURN_HOME entered | `Turning to home position` |
| STATE_DEPTH_DOWN → STATE_WAIT_DRUM_DEPTH | `Drum lowering to sample depth` |
| STATE_WAIT_DRUM_DEPTH → STATE_PRE_FLUSH_SETTLE | `Drum at depth - settling before flush` |
| STATE_PRE_FLUSH_SETTLE → STATE_FLUSH | `Flushing at depth` |
| STATE_FLUSH → STATE_POST_FLUSH_SETTLE | `Flush done - settling` |
| STATE_POST_FLUSH_SETTLE → STATE_TURN_NEXT | `Turning to bottle (N)` |
| STATE_TURN_NEXT → STATE_PRE_SAMPLE_SETTLE | `At bottle - settling before sample` |
| STATE_PRE_SAMPLE_SETTLE → STATE_REAL_SAMPLE | `Sampling bottle` |
| STATE_REAL_SAMPLE → STATE_POST_SAMPLE_SETTLE | `Sample done - settling` |
| STATE_POST_SAMPLE_SETTLE → STATE_TURN_HOME_FINAL | `Returning turntable to home` |
| STATE_TURN_HOME_FINAL → STATE_WAIT_DRUM_HOME | `Turntable home - retracting drum` |
| STATE_WAIT_DRUM_HOME → STATE_IDLE | `Cycle complete - sent :X1` then `:X1\n` |

## Pump Control

The pump (Y-axis) uses a soft-start ramp to avoid mechanical stress:

- Start delay: 300µs between steps
- Target delay: 80µs between steps
- Ramp interval: 100ms per step increment

### Pump Context Flag

`pumpIsFlush` determines which timer is applied when `pumpRunning` is true:

- `pumpIsFlush = true` → `FLUSH_TIME_MS` timer (flush phase, position 0)
- `pumpIsFlush = false` → `SAMPLE_TIME_MS` timer (fill phase, position 1–5)

When the timer expires, `pumpRunning` is set false. The state machine detects `!pumpRunning` to advance to the next state. `:X1` is **never** sent by the pump timeout — only by `STATE_WAIT_DRUM_HOME`.

## Tunable Variables

All can be updated at runtime via serial commands without recompiling:

| Variable | Default | Command | Description |
|---|---|---|---|
| `SAMPLE_DEPTH_METERS` | 0.2m | `:SD{float}` | Target sample depth |
| `FLUSH_TIME_MS` | 30000ms | `:FT{int seconds}` | Flush pump duration |
| `SAMPLE_TIME_MS` | 30000ms | `:BT{int seconds}` | Bottle fill pump duration |

These reset to defaults on power cycle. The ESP32 GUI restores them from localStorage on page load.

## Sample Count Behaviour

- `sampleCount` increments in `STATE_POST_SAMPLE_SETTLE` after a successful fill
- `sampleCount` is **not reset** on emergency stop (`:C1`) — by design, so a paused mission can resume filling the next bottles
- `sampleCount` resets only on Arduino power cycle
- Maximum samples: `MAX_SAMPLES = 5` (const, not tunable at runtime)

## Emergency Stop

`:C1` calls `doStop()` which immediately:
- Sets `pumpRunning = false`
- Sets `targetX = targetY = targetZ = 0` (all motion halts within one loop iteration)
- Sets `currentState = STATE_IDLE`
- Does not reset `sampleCount`
