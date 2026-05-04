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
- 4° East magnetic declination correction applied in firmware
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

- FR-10: In autonomous mode the boat shall move forward continuously using closed-loop magnetometer heading control. On Start Auto Mode, the current compass heading is locked as `targetHeading`. A P-controller continuously biases the left/right motor differential to maintain that heading.
- FR-11: On obstacle detection (microswitch hit) after the forward lock period, the boat shall execute the stuck recovery sequence automatically. The recovery turn direction is determined by which switch fired; the turn magnitude is a random angle within the configured min/max turn angle range.
- FR-12: At the configured sample interval, the boat shall stop and execute a full water sample collection cycle
- FR-13: After sample collection the boat shall first re-align to the locked `targetHeading` (the heading captured before sampling started) using closed-loop magnetometer control, then resume forward motion. Alignment is considered complete when \|heading_error\| < `turn_tolerance_deg` or `turn_timeout_ms` elapses.
- FR-14: The system shall stop autonomous mode and enter stop mode when all configured samples have been collected
- FR-15: At every autonomous sample trigger, the boat shall reverse for the configured reverse duration before the sample wait begins (applies to both real and simulated sampling paths). The outer loop must NOT call `stopCar()` immediately before invoking the sample handler, since that would clobber the reverse phase.

### 3.4 Stuck Recovery

- FR-16: Stuck recovery shall follow the sequence: stop → reverse → stop → heading-controlled turn → final stop → resume forward
- FR-17: The stuck turn (step 4) shall use the HMC5883L magnetometer to turn until the heading error is within `turn_tolerance_deg`, or until `turn_timeout_ms` elapses (whichever comes first)
- FR-18: After a stuck recovery, the new `targetHeading` set during the turn shall become the heading used for subsequent closed-loop forward motion
- FR-19: All stuck recovery timing and heading parameters shall be configurable at runtime without reflashing
- FR-20: The sample-interval countdown shall **pause** for the duration of any stuck recovery event, and resume from where it paused on stuck exit. Implementation: capture `stuckPauseStart` at stuck entry; on exit shift `sampleStart` forward by `(now − stuckPauseStart)`. This prevents the boat from completing stuck recovery and immediately rolling over into a sample with no useful navigation between events.

### 3.5 Water Sampling Sequence

The Arduino shall execute the following sequence for each sample cycle:

- FR-21: Rotate turntable to position 0 (home/flush position) and wait for physical completion
- FR-22: Lower drum to configured sample depth and wait for physical completion
- FR-23: Wait 5 seconds (pre-flush settle)
- FR-24: Run flush pump for configured flush time at sample depth
- FR-25: Wait 5 seconds (post-flush settle)
- FR-26: Rotate turntable to next bottle position (1–5 in sequence) and wait for physical completion
- FR-27: Wait 5 seconds (pre-sample settle)
- FR-28: Run fill pump for configured fill time
- FR-29: Wait 5 seconds (post-sample settle)
- FR-30: Rotate turntable back to position 0 and wait for physical completion
- FR-31: Retract drum fully to zero and wait for physical completion
- FR-32: Only after drum is confirmed at zero, send completion signal `:X1` to ESP32
- FR-33: After all 5 bottles are filled, reject further sample commands with `:FULL` signal

### 3.6 No-Sampler Mode

- FR-34: The system shall support a no-sampler mode (`isWaterSamplerOnBoard = false`) in which autonomous missions run without an Arduino Uno connected
- FR-35: In no-sampler mode, the ESP32 shall self-acknowledge sample completion after a configurable `skipSampleDelay` (Self-Ack Delay), simulating the `:X1` arrival
- FR-36: The no-sampler mode flag and both timing parameters (Self-Ack Delay, Reverse Duration) shall be configurable at runtime via the GUI Parameters tab without reflashing

### 3.7 Resume After Interruption

- FR-37: If autonomous mode is stopped and restarted mid-mission, the Arduino shall continue filling the next available bottle (sampleCount is not reset on stop)
- FR-38: The ESP32 sample count shall reset to 0 each time autonomous mode is started fresh (T command)

---

## 4. Performance Requirements

- PR-01: GUI status polling interval: 500ms maximum latency
- PR-02: Joystick command repeat rate: 200ms
- PR-03: Arduino state machine loop: non-blocking (no `delay()` calls in main cycle)
- PR-04: ESP32 autonomous wait loop: must call `server.handleClient()` each iteration to keep GUI responsive during sampling
- PR-05: Sampling cycle timeout: 200 seconds (covers full cycle with margin)
- PR-06: Maximum sample depth: 2.0m (hard clamped in Arduino)
- PR-07: Turntable positioning accuracy: ±0.5° (limited by stepper resolution at 3200 steps/rev)
- PR-08: Closed-loop heading correction shall update every `loop()` iteration during forward motion (no additional delay)

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
- UIR-18: Both telemetry cards (Manual and Autonomous tabs) shall display the live compass heading and the locked target heading the boat is tracking

### 5.4 Safety

- UIR-19: A STOP button shall be permanently visible in the top bar on all tabs
- UIR-20: STOP shall immediately halt all motion and send stop command to Arduino
- UIR-21: The operator shall not be able to start motor motion without first pressing ARM
- UIR-22: The operator shall not be able to enter manual or autonomous mode without first pressing STOP (mode gate)
- UIR-22a: The Autonomous tab shall expose two explicit-direction manual stuck triggers — **Stuck Left** (`CL`) and **Stuck Right** (`CR`) — replacing the previous single ambiguous-direction "Trigger Stuck" button. Each shall enter the stuck-recovery sequence with a `targetHeading` adjusted by a random angle in `[min_turn_angle, max_turn_angle]` in the requested direction.

### 5.5 Technician Parameters

- UIR-23: Motor power, reverse power, turn power, and motor offset shall be adjustable from the Parameters tab
- UIR-24: All stuck recovery timing values shall be adjustable from the Parameters tab
- UIR-25: Navigation timing (debounce, min/max turn angle in degrees, forward lock) shall be adjustable from the Parameters tab
- UIR-26: Heading control parameters (P-controller Kp, turn tolerance, turn timeout) shall be adjustable from the Parameters tab
- UIR-27: Water sampler mode parameters (Sampler On Board toggle, Self-Ack Delay, Reverse Duration) shall be adjustable from the Parameters tab
- UIR-28: All parameter changes shall take effect immediately without reflashing or rebooting

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
| Timed turns replaced by magnetometer heading control | Removes environmental dependency (wind/current) from turn completion; compass ensures the boat actually reaches the intended new heading |
| Turn angle (degrees) replaces turn duration (ms) | Degrees are physically meaningful regardless of motor power or surface conditions |
| Reverse at every autonomous sample start | Kills residual forward momentum before the sample wait; prevents drift from the sample point |
| No-sampler mode with self-ack | Allows full autonomous mission testing / field use without the Arduino Uno, using the same code path |
| localStorage keys `MA`/`XA` for turn angles | Old keys `N`/`W` held millisecond values; new keys prevent stale ms values from loading into degree-based inputs after firmware update |
| No `stopCar()` before sample handler in `loop()` | Pre-stop was firing every loop iteration during `sampling=true` and immediately clobbering the `moveBackward()` of both the real-Uno and no-sampler paths. Removing it lets the reverse phase actually run. |
| Sample-interval clock pauses during stuck recovery | Without pause, finishing stuck recovery often left zero or negative time on the interval, causing an immediate sample with no useful navigation between events. |
| Post-sample heading realignment | The boat can rotate substantially during a ~136s sample wait (wind, current, drum drag). Realigning before resuming forward keeps the survey leg straight. |
| Two-direction manual stuck (`CL`/`CR`) instead of single `C` | Operators want to deliberately steer away from a known obstacle, not roll a die for direction. |
| `CL`/`CR` chosen over bare `L`/`R` | `L` and `R` are joystick commands in manual mode; reusing them would cause command-namespace collision. Two-character tokens fall through `handleCommandPath()`'s length-1 router into the multi-char branch where an exact `==` match is unambiguous. |
