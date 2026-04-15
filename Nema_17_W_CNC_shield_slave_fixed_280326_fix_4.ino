// Arduino Uno Sampling Platform Slave (version 28.3 / GUI v8.3)
//fixed the flush to occur after lowred to depth in sample.


#define LED 13

#define STEP_X 2
#define DIR_X 5
#define STEP_Y 3
#define DIR_Y 6
#define STEP_Z 4
#define DIR_Z 7
#define ENABLE_PIN 8

// Individual microstepping and speed per axis (task 10)
// For now, all axes use 3200 steps/rev and 1200us delay - Put 3 Jumpers in the CNC_shield
const float STEPS_PER_REV_X = 3200.0;  // X-axis (drum): steps per full revolution
const float STEPS_PER_DEGREE_X = STEPS_PER_REV_X / 360.0;  // X-axis: steps per degree
const unsigned long STEP_DELAY_US_X = 1200;  // X-axis: delay between steps (speed control)

// --- Y-axis (pump) Soft Start Parameters --- 
const float STEPS_PER_REV_Y = 800.0;           // Y-axis (pump): steps per full revolution - 3 jumpers in the cnc
const unsigned long TARGET_DELAY_Y = 80;     // Y-axis: target speed delay (micros)
const unsigned long START_DELAY_Y = 300;      // Y-axis: slow start delay (micros)
unsigned long currentDelayY = START_DELAY_Y;   
unsigned long lastRampTimeY = 0;               // Tracks acceleration timing
const unsigned long RAMP_INTERVAL_MS = 100;     // Ramp speed update every 10ms

const float STEPS_PER_REV_Z = 3200.0;  // Z-axis (turntable): steps per full revolution - put 3 jumpers in cnc
const float STEPS_PER_DEGREE_Z = STEPS_PER_REV_Z / 360.0;  // Z-axis: steps per degree
const unsigned long STEP_DELAY_US_Z = 1200;  // Z-axis: delay between steps (speed control)

long posX = 0;  // Track position only for X-axis (drum depth) - positive = down/depth increase (task 11)
long posY = 0;  // No position tracking for Y (pump) (task 13)
long posZ = 0;  // No position tracking for Z (turntable) (task 12)

bool pumpRunning = false;  // Flag for Y-axis pump (task 13: timed or infinite until stop)
bool pumpIsFlush = false;  // True during flush phase, false during bottle-fill phase
unsigned long pumpStart = 0;
unsigned long FLUSH_TIME_MS = 30000;   // Tunable: flush pump duration (ms)
unsigned long SAMPLE_TIME_MS = 30000;  // Tunable: bottle-fill pump duration (ms)

int sampleCount = 0;
const int MAX_SAMPLES = 5;

// Separate targets and last step times for each axis (task 9: separate control loops)
long targetX = 0;  // Signed target steps for X
long targetY = 0;  // Not used for Y (pump is flag-based), but for consistency
long targetZ = 0;  // Positive only for Z (one direction)

unsigned long lastStepTimeX = 0;
unsigned long lastStepTimeY = 0;
unsigned long lastStepTimeZ = 0;

float SAMPLE_DEPTH_METERS = 0.2;            // Tunable: desired sample depth in meters
const float MAX_SAFE_DEPTH_METERS = 2.0;    // Hard safety limit
const float STEPS_PER_METER = 27825.0;     // 3200 steps/rev * 10 rev per meter (since 0.1m per 90°)

enum SampleState {
  STATE_IDLE,
  STATE_TURN_HOME,
  STATE_DEPTH_DOWN,
  STATE_WAIT_DRUM_DEPTH,    // Wait for drum to physically reach sample depth
  STATE_PRE_FLUSH_SETTLE,   // 5s settle after drum arrives, before flush
  STATE_FLUSH,
  STATE_POST_FLUSH_SETTLE,  // 5s settle after flush, before turning to bottle
  STATE_TURN_NEXT,
  STATE_PRE_SAMPLE_SETTLE,  // 5s settle after turntable at bottle, before fill
  STATE_REAL_SAMPLE,
  STATE_POST_SAMPLE_SETTLE, // 5s settle after fill, before returning home
  STATE_TURN_HOME_FINAL,
  STATE_WAIT_DRUM_HOME      // Wait for drum to physically finish retracting
};

SampleState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
int nextBottle = 0;  // Temp for next position

void setup() {
  Serial.begin(115200);
  pinMode(STEP_X, OUTPUT);
  pinMode(DIR_X, OUTPUT);
  pinMode(STEP_Y, OUTPUT);
  pinMode(DIR_Y, OUTPUT);
  pinMode(STEP_Z, OUTPUT);
  pinMode(DIR_Z, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // ENABLE ALL DRIVERS
  digitalWrite(LED, LOW);
  delay(2000);
}

void loop() {
  handleSerial();
  stepX();  // Separate stepping for X-axis (task 9)
  stepY();  // Separate stepping for Y-axis (pump) (task 9)
  stepZ();  // Separate stepping for Z-axis (task 9)
  handleSampleStateMachine();
}

// Separate stepping function for X-axis (drum) - tracks posX, handles positive/negative (task 9, 11)
void stepX() {
  if (targetX == 0) return;
  unsigned long now = micros();
  if (now - lastStepTimeX < STEP_DELAY_US_X) return;
  lastStepTimeX = now;
  digitalWrite(STEP_X, HIGH);
  digitalWrite(STEP_X, LOW);
  if (targetX > 0) {
    posX++;  // Positive = down (depth increase)
    targetX--;
  } else {
    posX--;  // Negative = up (depth decrease)
    targetX++;
  }
}

// Separate stepping function for Y-axis (pump) - Legalized Non-Blocking Ramp
void stepY() {
  if (!pumpRunning) {
    currentDelayY = START_DELAY_Y; // Reset ramp for next start
    return;
  }
  
  unsigned long nowMicros = micros();
  unsigned long nowMillis = millis();

  // 1. Timing check for physical step
  if (nowMicros - lastStepTimeY >= currentDelayY) {
    lastStepTimeY = nowMicros;
    
    digitalWrite(DIR_Y, HIGH);  // Pump always one direction
    digitalWrite(STEP_Y, HIGH);
    delayMicroseconds(5);       // Ensure pulse is long enough for A4988
    digitalWrite(STEP_Y, LOW);
  }

  // 2. Time-based acceleration ramp
  if (currentDelayY > TARGET_DELAY_Y) {
    if (nowMillis - lastRampTimeY >= RAMP_INTERVAL_MS) {
      lastRampTimeY = nowMillis;
      if (currentDelayY > TARGET_DELAY_Y + 5) {
        currentDelayY -= 5; 
      } else {
        currentDelayY = TARGET_DELAY_Y;
      }
    }
  }

  // 3. Timeout check for sampling — use flush or fill duration depending on context
  if (nowMillis - pumpStart >= (pumpIsFlush ? FLUSH_TIME_MS : SAMPLE_TIME_MS)) {
    pumpRunning = false;
    Serial.println(pumpIsFlush ? "Flush complete" : "Fill complete");
  }
}

// Separate stepping function for Z-axis (turntable) - no position tracking, always positive/one-way (task 9, 12)
void stepZ() {
  if (targetZ == 0) return;
  unsigned long now = micros();
  if (now - lastStepTimeZ < STEP_DELAY_US_Z) return;
  lastStepTimeZ = now;
  
  digitalWrite(STEP_Z, HIGH);
  // Small delay for pulse width if needed, though A4988 is fast
  digitalWrite(STEP_Z, LOW);
  
  targetZ--; // Just countdown the remaining steps to move
}

void handleSerial() {
  static String cmd = "";
  if (Serial.available()) {
    char c = Serial.read();
    cmd += c;
    if (c == '\n') {
      cmd.trim();
      if (cmd == ":M1") doAutoSample();
      else if (cmd == ":M2") doManualPumpStart();
      else if (cmd == ":B1") doReturnToZero();
      else if (cmd == ":D1") doSampleDepthDown();
      else if (cmd == ":E1") doDepthDown();
      else if (cmd == ":F1") doDepthUp();
      else if (cmd == ":C1") doStop();

      else if (cmd.startsWith(":SD")) {
        float depth = cmd.substring(3).toFloat();
        depth = constrain(depth, 0.0, 2.0);
        SAMPLE_DEPTH_METERS = depth;
        Serial.println("Sample depth set: " + String(SAMPLE_DEPTH_METERS, 1) + "m");
      }
      else if (cmd.startsWith(":FT")) {
        unsigned long t = (unsigned long)cmd.substring(3).toInt() * 1000;
        FLUSH_TIME_MS = constrain(t, 0UL, 60000UL);
        Serial.println("Flush time set: " + String(FLUSH_TIME_MS / 1000) + "s");
      }
      else if (cmd.startsWith(":BT")) {
        unsigned long t = (unsigned long)cmd.substring(3).toInt() * 1000;
        SAMPLE_TIME_MS = constrain(t, 0UL, 60000UL);
        Serial.println("Fill time set: " + String(SAMPLE_TIME_MS / 1000) + "s");
      }

      else if (cmd.startsWith(":P") && cmd.length() == 3) {
        char posChar = cmd[2];
        if (posChar >= '0' && posChar <= '5') {
          int pos = posChar - '0';
          doTurntableTo(pos);
        }
      }
      cmd = "";
    }
  }
}

void moveRelative(long &target, long steps, int dirPin) {
  digitalWrite(dirPin, steps > 0 ? HIGH : LOW);
  target = steps > 0 ? steps : -steps;
}

void doAutoSample() {
  if (sampleCount >= MAX_SAMPLES) {
    Serial.print(":FULL\n");
    Serial.println("Auto sample rejected - max samples reached");
    return;
  }
  Serial.print("OK\n");
  Serial.println("Starting auto sample cycle");
  currentState = STATE_TURN_HOME;  // Start the state machine
}

void doManualPumpStart() {
  Serial.print("OK\n");
  Serial.println("Starting manual pump (continuous until stop)");
  pumpRunning = true;
  pumpStart = millis();
}

void doTurntableTo(int bottle) {
  // 1. Calculate the absolute step position for the requested bottle
  // Every bottle is 60 degrees apart. 
  // Formula: (Bottle #) * (60 degrees) * (Steps per Degree)
  long absoluteTargetSteps = (long)(bottle * 60.0 * STEPS_PER_DEGREE_Z);

  // 2. Calculate the relative distance from where we are (posZ) to where we want to be
  long stepsToMove = absoluteTargetSteps - posZ;

  // 3. PREVENT UNNECESSARY MOVE: If we are already there, just exit
  if (stepsToMove == 0) {
    Serial.print("OK\n");
    Serial.print("Turntable already at position ");
    Serial.println(bottle);
    return; 
  }

  // 4. Set Direction and Target
  digitalWrite(DIR_Z, stepsToMove > 0 ? HIGH : LOW);
  targetZ = abs(stepsToMove);

  // 5. Update our internal tracker to the new absolute position
  // We update it now so the next command knows where we 'will' be
  posZ = absoluteTargetSteps;

  Serial.print("OK\n");
  Serial.print("Moving turntable to position ");
  Serial.print(bottle);
  Serial.print(" (Moving ");
  Serial.print(stepsToMove);
  Serial.println(" steps)");
}



void doReturnToZero() {
  Serial.print("OK\n");
  Serial.println("Returning depth drum to zero position");
  long steps = -posX;
  digitalWrite(DIR_X, steps > 0 ? HIGH : LOW);
  targetX = steps;  // Keep signed for direction
  Serial.print("Current drum position (steps): ");
  Serial.println(posX);
}

void doDepthDown() {
  Serial.print("OK\n");
  Serial.println("Lowering depth drum (90 degrees down)");
  long steps = 90 * STEPS_PER_DEGREE_X;
  digitalWrite(DIR_X, HIGH);  // Positive = down
  targetX = steps;
  Serial.print("Current drum position (steps): ");
  Serial.println(posX);
}

void doDepthUp() {
  Serial.print("OK\n");
  Serial.println("Raising depth drum (90 degrees up)");
  long steps = -90 * STEPS_PER_DEGREE_X;   
  //5.75 cm = 1/4 turn drum @ 90* (6400/360)    //  full turn is 360deg = 6400 steps = 23cm so 1cm = 6400/23=278.26 steps per cm *100 = ~27825.0 steps per meter
  digitalWrite(DIR_X, LOW);  // Negative = up
  targetX = steps;
  Serial.print("Current drum position (steps): ");
  Serial.println(posX);
}

void doSampleDepthDown() {
  float desiredDepth = SAMPLE_DEPTH_METERS;
  if (desiredDepth > MAX_SAFE_DEPTH_METERS) {
    desiredDepth = MAX_SAFE_DEPTH_METERS;  // Silent clamp
  }

  long targetSteps = (long)(desiredDepth * STEPS_PER_METER + 0.5);  // Round to nearest step

  Serial.print("OK\n");
  Serial.print("Lowering hose to sample depth ");
  Serial.print(desiredDepth);
  Serial.print(" m (");
  Serial.print(targetSteps);
  Serial.println(" steps)");

  digitalWrite(DIR_X, HIGH);  // Positive = down
  targetX = targetSteps;
}

void handleSampleStateMachine() {
  unsigned long now = millis();

  switch (currentState) {

    case STATE_IDLE:
      break;

    case STATE_TURN_HOME:
      // Issue turntable move to position 0, then wait for it to arrive
      doTurntableTo(0);
      currentState = STATE_DEPTH_DOWN;
      Serial.println("Turning to home position");
      break;

    case STATE_DEPTH_DOWN:
      if (targetZ == 0) {  // Wait for turntable physically at home
        doSampleDepthDown();
        currentState = STATE_WAIT_DRUM_DEPTH;
        Serial.println("Drum lowering to sample depth");
      }
      break;

    case STATE_WAIT_DRUM_DEPTH:
      if (targetX == 0) {  // Wait for drum physically at sample depth
        stateStartTime = now;
        currentState = STATE_PRE_FLUSH_SETTLE;
        Serial.println("Drum at depth - settling before flush");
      }
      break;

    case STATE_PRE_FLUSH_SETTLE:
      if (now - stateStartTime >= 5000) {  // 5s settle
        pumpIsFlush = true;
        pumpRunning = true;
        pumpStart = now;
        currentState = STATE_FLUSH;
        Serial.println("Flushing at depth");
      }
      break;

    case STATE_FLUSH:
      if (!pumpRunning) {  // Wait for flush pump complete
        stateStartTime = now;
        currentState = STATE_POST_FLUSH_SETTLE;
        Serial.println("Flush done - settling");
      }
      break;

    case STATE_POST_FLUSH_SETTLE:
      if (now - stateStartTime >= 5000) {  // 5s settle
        nextBottle = sampleCount + 1;
        doTurntableTo(nextBottle);
        currentState = STATE_TURN_NEXT;
        Serial.print("Turning to bottle (");
        Serial.print(nextBottle);
        Serial.println(")");
      }
      break;

    case STATE_TURN_NEXT:
      if (targetZ == 0) {  // Wait for turntable physically at bottle
        stateStartTime = now;
        currentState = STATE_PRE_SAMPLE_SETTLE;
        Serial.println("At bottle - settling before sample");
      }
      break;

    case STATE_PRE_SAMPLE_SETTLE:
      if (now - stateStartTime >= 5000) {  // 5s settle
        pumpIsFlush = false;
        pumpRunning = true;
        pumpStart = now;
        currentState = STATE_REAL_SAMPLE;
        Serial.println("Sampling bottle");
      }
      break;

    case STATE_REAL_SAMPLE:
      if (!pumpRunning) {  // Wait for bottle fill complete
        stateStartTime = now;
        currentState = STATE_POST_SAMPLE_SETTLE;
        Serial.println("Sample done - settling");
      }
      break;

    case STATE_POST_SAMPLE_SETTLE:
      if (now - stateStartTime >= 5000) {  // 5s settle
        sampleCount++;
        doTurntableTo(0);
        currentState = STATE_TURN_HOME_FINAL;
        Serial.println("Returning turntable to home");
      }
      break;

    case STATE_TURN_HOME_FINAL:
      if (targetZ == 0) {  // Wait for turntable physically at home
        doReturnToZero();
        currentState = STATE_WAIT_DRUM_HOME;
        Serial.println("Turntable home - retracting drum");
      }
      break;

    case STATE_WAIT_DRUM_HOME:
      if (targetX == 0) {  // Wait for drum physically fully retracted
        currentState = STATE_IDLE;
        Serial.print(":X1\n");
        Serial.println("Cycle complete - sent :X1");
      }
      break;
  }
}

void doStop() {
  Serial.print("OK\n");
  Serial.print("STOPPED\n");
  Serial.println("Emergency stop - pump off, all movements halted");
  pumpRunning = false;
  targetX = 0;
  targetY = 0;  // Not used, but for consistency
  targetZ = 0;
  digitalWrite(LED, LOW);
  currentState = STATE_IDLE;  // Reset state machine on stop (but sampleCount unchanged)
}