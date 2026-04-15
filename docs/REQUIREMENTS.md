# Requirements

## 1. System Overview

SRV-01 is an autonomous aquaculture water sampling vessel. The system must collect water samples from a fish pond or water body at programmable depth, at timed intervals, into up to five separate sample bottles, without requiring operator presence during sampling.

---

## 2. Hardware Requirements

### 2.1 Vessel

- Dual-motor surface vessel with ESC-controlled brushless motors
- Capable of forward, backward, left turn, and right turn motion
- Motors controlled by PWM servo signals (1000–2000µs range)
- Reverse direction implemented via separate reverse-enable ESC signal line
- Maximum safe throttle pulse: 1500µs

### 2.2 ESP32 Controller

- ESP32 module with WiFi capability
- Minimum 4 PWM-capable GPIO pins for motor ESC signals
- I2C bus for magnetometer
- UART2 (GPIO 16/17) for Arduino communication
- 3 digital input pins for microswitches (INPUT_PULLUP)
- 4 digital output pins for status LEDs
- 5V→3.3V voltage divider on UART2 RX line (Arduino TX is 5V logic)

### 2.3 Sampling Platform

- Arduino Uno with CNC Shield v3.0
- 3× NEMA 17 stepper motors (X: drum/depth, Y: pump, Z: turntable)
- All axes: 1/16 microstepping (3 jumpers installed per axis on CNC Shield)
- Turntable: 6-position carousel, 60° between positions, positions 1–5 for sample bottles, position 0 for flush/waste
- Drum: hose winding mechanism, ~0.1m per revolution, maximum safe depth 2.0m
- Pump: peristaltic or centrifugal pump, one direction only, soft-start ramp

### 2.4 Sensors

- HMC5883L magnetometer on I2C (SDA: GPIO21, SCL: GPIO22)
- 4° East magnetic declination correction applied
- 3× microswitches for obstacle detection (left, front, right)

### 2.5 Indicators

- LED Red: stuck recovery in progress
- LED Green: manual mode active
- LED Blue: autonomous mode active
- LED White: stop mode

---

## 3. Functional Requirements

### 3.1 Operating Modes

- FR-01: The system shall support three mutually exclusive modes: Stop, Manual, Autonomous
- FR-02: The system shall boot into Stop mode by default
- FR-03: Mode transitions from Stop are: Stop → Manual (via M command), Stop → Autonomous (via T command)
- FR-04: Any mode can transition to Stop via the STOP button at any time
- FR-05: ESCs must be armed (Z command) before motor commands have effect

### 3.2 Manual Mode

- FR-06: In manual mode the operator shall be able to control forward, backward, left turn, and right turn motion via the joystick
- FR-07: In manual mode the operator shall be able to manually trigger a sampling cycle
- FR-08: In manual mode the operator shall be able to manually control the pump, drum depth, and turntable position independently
- FR-09: Manual joystick commands shall repeat at 200ms intervals while held

### 3.3 Autonomous Mode

- FR-10: In autonomous mode the boat shall move forward continuously
- FR-11: On obstacle detection (microswitch hit) after the forward lock period, the boat shall execute the stuck recovery sequence automatically
- FR-12: At the configured sample interval, the boat shall stop and execute a full water sample collection cycle
- FR-13: After sample collection the boat shall resume forward motion immediately
- FR-14: The system shall stop autonomous mode and enter stop mode when all configured samples have been collected

### 3.4 Stuck Recovery

- FR-15: Stuck recovery shall follow the sequence: stop → reverse → stop → turn (random duration, switch-based direction) → final stop → resume forward
- FR-16: All stuck recovery timing parameters shall be configurable at runtime without reflashing
- FR-17: The sample interval countdown shall continue during stuck recovery

### 3.5 Water Sampling Sequence

The Arduino shall execute the following sequence for each sample cycle:

- FR-18: Rotate turntable to position 0 (home/flush position) and wait for physical completion
- FR-19: Lower drum to configured sample depth and wait for physical completion
- FR-20: Wait 5 seconds (pre-flush settle)
- FR-21: Run flush pump for configured flush time at sample depth
- FR-22: Wait 5 seconds (post-flush settle)
- FR-23: Rotate turntable to next bottle position (1–5 in sequence) and wait for physical completion
- FR-24: Wait 5 seconds (pre-sample settle)
- FR-25: Run fill pump for configured fill time
- FR-26: Wait 5 seconds (post-sample settle)
- FR-27: Rotate turntable back to position 0 and wait for physical completion
- FR-28: Retract drum fully to zero and wait for physical completion
- FR-29: Only after drum is confirmed at zero, send completion signal `:X1` to ESP32
- FR-30: After all 5 bottles are filled, reject further sample commands with `:FULL` signal

### 3.6 Resume After Interruption

- FR-31: If autonomous mode is stopped and restarted mid-mission, the Arduino shall continue filling the next available bottle (sampleCount is not reset on stop)
- FR-32: The ESP32 sample count shall reset to 0 each time autonomous mode is started fresh (T command)

---

## 4. Performance Requirements

- PR-01: GUI status polling interval: 500ms maximum latency
- PR-02: Joystick command repeat rate: 200ms
- PR-03: Arduino state machine loop: non-blocking (no `delay()` calls in main cycle)
- PR-04: ESP32 autonomous wait loop: must call `server.handleClient()` each iteration to keep GUI responsive during sampling
- PR-05: Sampling cycle timeout: 200 seconds (covers full cycle with margin)
- PR-06: Maximum sample depth: 2.0m (hard clamped in Arduino)
- PR-07: Turntable positioning accuracy: ±0.5° (limited by stepper resolution at 3200 steps/rev)

---

## 5. User Interface Requirements

### 5.1 Accessibility

- UIR-01: GUI shall be usable on a smartphone with no pinch-zoom required
- UIR-02: All buttons shall be large enough for touch operation (minimum 38px height)
- UIR-03: GUI shall function on iOS Safari and Android Chrome without modification
- UIR-04: No internet connection required — all assets embedded in firmware

### 5.2 Operator Workflow — Pre-Mission

- UIR-05: Operator shall be able to set all mission parameters from a single card (Pre-Mission Setup) before starting
- UIR-06: Sample depth options shall include 0.0m, 0.2m, 0.5m, 1.0m, 1.5m, 2.0m
- UIR-07: Number of samples shall be selectable 1–5
- UIR-08: Flush time and fill time shall be independently settable 0–60 seconds
- UIR-09: Sample interval shall be settable 1–60 minutes
- UIR-10: All mission parameters shall persist in browser localStorage and restore automatically on page reload
- UIR-11: Sampling platform parameters (depth, flush time, fill time) shall only be sendable when the Arduino is in idle state — if not idle, a clear popup message shall inform the operator

### 5.3 Mission Monitoring

- UIR-12: A live elapsed time display (HH:MM:SS) shall run from mission start and accumulate across the mission
- UIR-13: A live countdown timer (MM:SS) shall show time until next sample
- UIR-14: The countdown shall show "SAMPLING / Water collection" during active sample collection
- UIR-15: The countdown shall reset to the full interval after each completed sample
- UIR-16: The Arduino State field shall display live granular status during sampling (turning, flushing, filling, retracting, etc.)
- UIR-17: The sample count shall be visible in the telemetry card at all times

### 5.4 Safety

- UIR-18: A STOP button shall be permanently visible in the top bar on all tabs
- UIR-19: STOP shall immediately halt all motion and send stop command to Arduino
- UIR-20: The operator shall not be able to start motor motion without first pressing ARM
- UIR-21: The operator shall not be able to enter manual or autonomous mode without first pressing STOP (mode gate)

### 5.5 Technician Parameters

- UIR-22: Motor power, reverse power, turn power, and motor offset shall be adjustable from the Parameters tab
- UIR-23: All stuck recovery timing values shall be adjustable from the Parameters tab
- UIR-24: Navigation timing (debounce, min/max turn, forward lock) shall be adjustable from the Parameters tab
- UIR-25: All parameter changes shall take effect immediately without reflashing or rebooting

---

## 6. Known Constraints and Design Decisions

| Decision | Rationale |
|---|---|
| ARM `delay(2000)` removed | Was blocking HTTP server causing motor cutoff. ESCs arm without it. |
| Sampling platform params only sent when Arduino is idle | Prevents parameter change mid-cycle corrupting the state machine |
| `:X1` sent only from `STATE_WAIT_DRUM_HOME` | Ensures boat never resumes motion until drum is physically confirmed at zero |
| ESP32 timeout for sample cycle: 200s | Full cycle at 2m depth takes ~136s minimum; 200s gives 64s safety margin |
| `sampleCount` not reset on stop | Allows mission resume after stuck recovery or operator intervention |
| No `delay()` in Arduino state machine | Maintains responsive stepper control and serial communication throughout |
