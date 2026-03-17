/*
 * Omni-Wheel G-code Plotter Firmware
 * ====================================
 * This is a generic G-code plotter that runs on a 3-wheel omni robot.
 * It reads G-code from the included drawing.h file and executes it.
 * The user can replace drawing.h with any G-code file to draw
 * different designs — the firmware itself never needs to change.
 *
 * Think of this like a CNC plotter:
 *   - The firmware is the "machine" (handles motors, encoders, PID)
 *   - The drawing.h is the "job file" (the G-code program to draw)
 *   - Press the button to start the job
 *
 * Supported G-code:
 *   G0 Xn Yn       - Rapid move (pen up)
 *   G1 Xn Yn       - Linear move (pen down)
 *   G2 Xn Yn In Jn - Clockwise arc (pen down)
 *   G3 Xn Yn In Jn - Counter-clockwise arc (pen down)
 *   M3              - Pen down
 *   M5              - Pen up
 *   ; comment       - Ignored
 *
 * Hardware:
 *   - Arduino Uno R3
 *   - Adafruit Motor Shield V2 (I2C)
 *   - 3x N20 DC motors with hall-effect encoders (6V, 1:50 gear)
 *     M1 = front (90°), M2 = rear-left (210°), M3 = rear-right (330°)
 *   - SG90 servo on Motor Shield Servo 1 header (pin 10)
 *   - Push button on pin 12 wired to GND (uses INPUT_PULLUP)
 *
 * PID controller follows curiores/ArduinoTutorials pattern.
 */

// ============================================================
//  INCLUDES
// ============================================================

#include <Wire.h>                    // I2C bus (needed by motor shield)
#include <Adafruit_MotorShield.h>    // Adafruit Motor Shield V2 driver
#include <Servo.h>                   // Servo library for pen lift
#include "drawing.h"                 // G-code program to draw (user-replaceable)

// ============================================================
//  HARDWARE OBJECTS
// ============================================================

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  // Motor shield (default I2C address)
Adafruit_DCMotor *m1 = AFMS.getMotor(1);  // Front motor (90°)
Adafruit_DCMotor *m2 = AFMS.getMotor(2);  // Rear-left motor (210°)
Adafruit_DCMotor *m3 = AFMS.getMotor(3);  // Rear-right motor (330°)
Servo penServo;                             // Pen lift servo

// ============================================================
//  CONFIGURATION — Adjust these for your robot
// ============================================================

// -- Pin assignments --
#define SERVO_PIN       10   // Servo signal (Motor Shield Servo 1 header)
#define BUTTON_PIN      12   // Start button (wired to GND, uses internal pullup)

// -- Encoder pins --
#define ENC1_A  2   // Motor 1 encoder channel A (hardware interrupt INT0)
#define ENC1_B  4   // Motor 1 encoder channel B (direction)
#define ENC2_A  3   // Motor 2 encoder channel A (hardware interrupt INT1)
#define ENC2_B  5   // Motor 2 encoder channel B (direction)
#define ENC3_A  8   // Motor 3 encoder channel A (pin-change interrupt PCINT0)
#define ENC3_B  7   // Motor 3 encoder channel B (direction)

// -- Servo angles --
#define PEN_UP_ANGLE    45   // Servo angle when pen is lifted (degrees)
#define PEN_DOWN_ANGLE  0    // Servo angle when pen touches paper (degrees)
#define PEN_SETTLE_MS   300  // Wait time after servo move (ms)

// -- Encoder/wheel specs --
#define ENCODER_CPR   700.0f   // Ticks per wheel revolution (14 CPR * 50:1 gear)
#define WHEEL_DIA_MM  36.0f    // Wheel outer diameter (mm)
#define MM_PER_TICK   ((PI * WHEEL_DIA_MM) / ENCODER_CPR)  // ~0.1616 mm/tick

// -- PID gains (3 separate sets for different move types) --
// kp: proportional — main correction force toward target
// kd: derivative — braking to prevent overshoot
// ki: integral — builds up when stuck to push through friction

// Travel PID (G0): pen up, gentle cruise
float kp_travel = 40.0;
float kd_travel = 2.0;
float ki_travel = 0.0;

// Straight-line PID (G1): pen down, accuracy matters
float kp_line = 10.0;
float kd_line = 0.0;
float ki_line = 0.0;

// Curve PID (arc segments): aggressive to push through small segments
float kp_curve = 80.0;
float kd_curve = 10.0;
float ki_curve = 15.0;

// Active PID gains (set before each move)
float kp, kd, ki;

// -- Movement parameters --
#define MOVE_TIMEOUT_MS   8000UL   // Max time per move before giving up (ms)
#define POS_TOL_MM        3.0f     // Arrival tolerance (mm)

// -- Arc parameters --
#define ARC_TIMEOUT_MS    20000UL  // Max time for any arc (ms)
#define ARC_RAMP_RAD      0.5f    // Deceleration zone near end (~30°, adjustable)
int arcMinSpd = 80;                // Slowest speed at end of ramp (adjustable)
int arcCruiseSpd = 255;            // Full-speed cruise for long arcs (adjustable)

// ============================================================
//  WHEEL GEOMETRY
//
//  Three omni-wheels at 120-degree intervals.
//  These angles define where each motor is mounted on the robot.
//  sin/cos are pre-computed in setup() for speed.
// ============================================================

const float W_ANG[3] = {              // Wheel mounting angles (radians)
   90.0f * PI / 180.0f,               // Motor 1: front (90°)
  210.0f * PI / 180.0f,               // Motor 2: rear-left (210°)
  330.0f * PI / 180.0f                // Motor 3: rear-right (330°)
};
float sinW[3], cosW[3];               // Pre-computed trig values

// ============================================================
//  ENCODER STATE
//
//  Encoder tick counters are updated by ISRs (interrupt service
//  routines) that fire automatically on encoder pulses.
//  "volatile" tells the compiler these change asynchronously.
// ============================================================

volatile int posi[3] = {0, 0, 0};     // Raw tick counters (updated by ISRs)
int8_t encSign[3] = {1, 1, 1};        // Polarity: +1 normal, -1 reversed (set by calibration)

// ============================================================
//  PID CONTROLLER STATE
// ============================================================

long prevT = 0;            // Previous timestamp in microseconds
float eprevX = 0;          // Previous X error (for derivative)
float eprevY = 0;          // Previous Y error (for derivative)
float eintegralX = 0;      // Accumulated X error (integral term)
float eintegralY = 0;      // Accumulated Y error (integral term)

// ============================================================
//  ODOMETRY STATE (position tracking)
// ============================================================

float posX = 0.0f;                    // X position since last reset (mm)
float posY = 0.0f;                    // Y position since last reset (mm)
int posPrev[3] = {0, 0, 0};           // Previous encoder snapshots

// ============================================================
//  PATH LENGTH STATE (used for circle drawing)
// ============================================================

float pathLen = 0.0f;                  // Distance traveled since last reset
float prevPathX = 0.0f;               // Previous position for distance calc
float prevPathY = 0.0f;

// ============================================================
//  G-CODE INTERPRETER STATE
//
//  The interpreter tracks absolute position in the G-code
//  coordinate system. This persists across all moves.
//  (Odometry resets between moves, but gcX/gcY are cumulative.)
// ============================================================

float gcX = 0.0f;          // Absolute X in G-code coords (mm)
float gcY = 0.0f;          // Absolute Y in G-code coords (mm)
bool penIsDown = false;     // Current pen state

// ============================================================
//  ENCODER ISRs (Interrupt Service Routines)
//
//  These run automatically when an encoder pulse is detected.
//  They count ticks to track wheel rotation.
//
//  Pattern from curiores/ArduinoTutorials:
//  On RISING edge of channel A, read channel B:
//    B HIGH → forward → increment
//    B LOW  → backward → decrement
// ============================================================

// Motor 1 ISR — hardware interrupt on pin 2 (INT0)
void enc1ISR() {
  int b = digitalRead(ENC1_B);  // Read direction channel
  if (b > 0) { posi[0]++; }    // Forward
  else       { posi[0]--; }    // Backward
}

// Motor 2 ISR — hardware interrupt on pin 3 (INT1)
void enc2ISR() {
  int b = digitalRead(ENC2_B);  // Read direction channel
  if (b > 0) { posi[1]++; }    // Forward
  else       { posi[1]--; }    // Backward
}

// Motor 3 ISR — pin-change interrupt on pin 8 (PCINT0)
// Pin-change fires on BOTH edges, so we filter for rising only
volatile uint8_t enc3PrevA = 0;       // Previous state of channel A
ISR(PCINT0_vect) {
  uint8_t a = digitalRead(ENC3_A);    // Read current channel A state
  if (a && !enc3PrevA) {              // Rising edge only (was 0, now 1)
    int b = digitalRead(ENC3_B);      // Read direction channel
    if (b > 0) { posi[2]++; }        // Forward
    else       { posi[2]--; }        // Backward
  }
  enc3PrevA = a;                      // Save for next edge comparison
}

// ============================================================
//  ODOMETRY FUNCTIONS
//
//  Reads encoder ticks, converts to mm of wheel travel,
//  then uses forward kinematics to estimate robot center
//  movement in X and Y.
// ============================================================

// Update robot position from encoder readings
void updateOdometry() {
  // Atomically read all encoder counters (disable interrupts briefly)
  int pos[3];
  noInterrupts();
  pos[0] = posi[0]; pos[1] = posi[1]; pos[2] = posi[2];
  interrupts();

  // Calculate distance each wheel moved since last update
  float d[3];
  for (uint8_t i = 0; i < 3; i++) {
    d[i] = (float)(pos[i] - posPrev[i]) * MM_PER_TICK * encSign[i];
    posPrev[i] = pos[i];
  }

  // Forward kinematics: 3 wheel distances → robot X,Y displacement
  // The (2/3) factor comes from the pseudo-inverse of the kinematics matrix
  posX += (2.0f / 3.0f) * (-sinW[0]*d[0] - sinW[1]*d[1] - sinW[2]*d[2]);
  posY += (2.0f / 3.0f) * ( cosW[0]*d[0] + cosW[1]*d[1] + cosW[2]*d[2]);
}

// Reset position to (0,0) — call before each relative move
void resetOdometry() {
  noInterrupts();
  posPrev[0] = posi[0]; posPrev[1] = posi[1]; posPrev[2] = posi[2];
  interrupts();
  posX = 0.0f; posY = 0.0f;
}

// Reset path length counter (for measuring circle circumference)
void resetPathLength() {
  pathLen = 0.0f;
  prevPathX = posX; prevPathY = posY;
}

// Add distance traveled since last call to path length
void updatePathLength() {
  float dx = posX - prevPathX;
  float dy = posY - prevPathY;
  pathLen += sqrtf(dx * dx + dy * dy);
  prevPathX = posX; prevPathY = posY;
}

// ============================================================
//  MOTOR CONTROL
// ============================================================

// Stop all motors (coast, not brake)
void stopAll() {
  m1->run(RELEASE); m1->setSpeed(0);
  m2->run(RELEASE); m2->setSpeed(0);
  m3->run(RELEASE); m3->setSpeed(0);
}

// Drive one motor: dir = 1 (forward), -1 (backward), 0 (release)
void setMotor(Adafruit_DCMotor *m, int dir, int pwmVal) {
  m->setSpeed(pwmVal);
  if (dir == 1)       { m->run(FORWARD);  }
  else if (dir == -1) { m->run(BACKWARD); }
  else                { m->run(RELEASE);  }
}

// ============================================================
//  INVERSE KINEMATICS
//
//  Converts robot velocity (vx, vy) → individual wheel speeds.
//  Each wheel drives perpendicular to its axle, so we project
//  the velocity onto each wheel's drive direction.
// ============================================================

// Calculate wheel speeds from desired velocity
void inverseKinematics(float vx, float vy, float *w) {
  for (uint8_t i = 0; i < 3; i++)
    w[i] = -sinW[i] * vx + cosW[i] * vy;
}

// Select which PID gain set to use
void useTravelPID()  { kp = kp_travel; kd = kd_travel; ki = ki_travel; }
void useLinePID()    { kp = kp_line;   kd = kd_line;   ki = ki_line;   }
void useCurvePID()   { kp = kp_curve;  kd = kd_curve;  ki = ki_curve;  }

// Convert float wheel speeds to direction + PWM and send to motors
int minPWM = 80; // minimum motor power to overcome friction and actually have motor move
void applyWheelSpeeds(float *w) {
  Adafruit_DCMotor *motors[3] = {m1, m2, m3};

  // Find the smallest and largest active wheel speeds
  float minActive = 999, maxActive = 0;
  for (uint8_t i = 0; i < 3; i++) {
    float a = fabs(w[i]);
    if (a >= 5) {
      if (a < minActive) minActive = a;
      if (a > maxActive) maxActive = a;
    }
  }
  // Scale up so minimum active wheel hits minPWM, but don't let max exceed 255
  if (minActive < minPWM && minActive >= 5) {
    float idealScale = (float)minPWM / minActive;
    float maxScale = 255.0f / maxActive;
    float scale = min(idealScale, maxScale);
    for (uint8_t i = 0; i < 3; i++) w[i] *= scale;
  }

  for (uint8_t i = 0; i < 3; i++) {
    int pwr = (int)fabs(w[i]);       // Power = absolute value
    if (pwr > 255) pwr = 255;        // Cap at max PWM
    int dir = 1;                      // Assume forward
    if (w[i] < 0) dir = -1;          // Negative = backward
    if (pwr < 5) dir = 0;            // Too small = release
    setMotor(motors[i], dir, pwr);
  }
}

// Drive in direction (vx, vy) at given speed — used for open-loop circles
void driveVelocity(float vx, float vy, int spd) {
  float w[3];
  inverseKinematics(vx, vy, w);
  float peak = max(max(fabsf(w[0]), fabsf(w[1])), fabsf(w[2]));
  if (peak > 0.001f) {
    float s = (float)spd / peak;
    w[0] *= s; w[1] *= s; w[2] *= s;
  }
  applyWheelSpeeds(w);
}

// ============================================================
//  PID POSITION CONTROLLER
//
//  Blocking function that moves the robot to a target position
//  using closed-loop PID feedback. Runs until arrival or timeout.
//
//  Target is RELATIVE to last resetOdometry() call.
//
//  PID formula (curiores tutorial pattern):
//    error = target - current
//    derivative = (error - prev_error) / deltaT
//    integral += error * deltaT
//    output = kp*error + kd*derivative + ki*integral
// ============================================================

void moveTo(float targetX, float targetY) {
  // Reset PID state for this new move
  eprevX = 0; eprevY = 0;
  eintegralX = 0; eintegralY = 0;
  prevT = micros();    // Start timing

  unsigned long deadline = millis() + MOVE_TIMEOUT_MS;  // Safety timeout

  while (true) {
    // Time since last iteration (microsecond precision)
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;  // Convert to seconds
    prevT = currT;

    updateOdometry();  // Read encoders and update position

    // Position error
    float eX = targetX - posX;
    float eY = targetY - posY;
    float dist = sqrtf(eX * eX + eY * eY);

    // Arrived?
    if (dist < POS_TOL_MM) { stopAll(); break; }

    // Timeout?
    if (millis() >= deadline) {
      Serial.print(F("timeout pos=")); Serial.print(posX);
      Serial.print(F(",")); Serial.println(posY);
      stopAll(); break;
    }

    // Derivative: how fast error is changing (damping)
    float dedtX = (eX - eprevX) / deltaT;
    float dedtY = (eY - eprevY) / deltaT;

    // Integral: accumulated error (pushes through friction)
    eintegralX += eX * deltaT;
    eintegralY += eY * deltaT;

    // PID output: velocity command
    float uX = kp * eX + kd * dedtX + ki * eintegralX;
    float uY = kp * eY + kd * dedtY + ki * eintegralY;

    // Convert to wheel speeds and drive
    float w[3];
    inverseKinematics(uX, uY, w);
    applyWheelSpeeds(w);

    // Save error for next derivative
    eprevX = eX; eprevY = eY;
  }
}

// ============================================================
//  PEN CONTROL
// ============================================================

// Lift pen off paper
void penUp() {
  if (penIsDown) {
    penServo.write(PEN_UP_ANGLE);
    delay(PEN_SETTLE_MS);
    penIsDown = false;
  }
}

// Lower pen onto paper
void penDown() {
  if (!penIsDown) {
    penServo.write(PEN_DOWN_ANGLE);
    delay(PEN_SETTLE_MS);
    penIsDown = true;
  }
}

// ============================================================
//  G-CODE EXECUTION: ABSOLUTE MOVE
//
//  Converts absolute G-code coordinates to a relative PID move.
//  The robot's odometry resets between moves, but the G-code
//  state (gcX, gcY) tracks cumulative absolute position.
//
//  IMPORTANT: Diagonal moves cause the robot to rotate because
//  it has no IMU to correct heading drift. Pure X is worst
//  (only M1 does real work). Pure Y is best (M2 + M3 equal).
//
//  To avoid this, all moves are decomposed into two directions
//  that each engage exactly 2 wheels symmetrically:
//
//    Step 1: Move at 30° from X-axis (60° from Y)
//            → engages M1 + M3 equally, M2 idle
//    Step 2: Move in pure Y
//            → engages M2 + M3 equally, M1 idle
//
//  These two directions span the full 2D plane and both use
//  2 wheels driving equally, which prevents twisting.
//
//  Math: any (dx, dy) = a*(cos30°, sin30°) + b*(0, 1)
//        a = dx / cos30°  (covers all of dx, plus some dy)
//        b = dy - a*sin30° (covers remaining dy)
// ============================================================

// cos(30°) and sin(30°) — the 30° direction unit vector
#define COS30 0.8660254f
#define SIN30 0.5f

// 2-wheel decomposed move: 30° direction first, then pure Y
// Both legs engage exactly 2 wheels symmetrically to prevent twisting
void gcMoveTo(float absX, float absY) {
  float dx = absX - gcX;
  float dy = absY - gcY;

  // Skip tiny moves
  if (fabsf(dx) < 0.5f && fabsf(dy) < 0.5f) {
    gcX = absX; gcY = absY;
    return;
  }

  // Step 1: Move at 30° from X-axis to cover all of dx
  // Direction (cos30°, sin30°) engages M1 + M3 equally, M2 is idle
  // a = dx / cos30° gives us the distance along the 30° direction
  // This move covers all of dx and also covers (a * sin30°) of dy
  if (fabsf(dx) >= 0.5f) {
    float a = dx / COS30;          // Distance along 30° direction
    float moveX = dx;              // = a * cos30° (all of dx)
    float moveY = a * SIN30;       // Partial dy covered by this leg
    resetOdometry();
    moveTo(moveX, moveY);
    gcX += moveX;
    gcY += moveY;
    dy = absY - gcY;              // Recalculate remaining Y distance
  }

  // Step 2: Move in pure Y to cover remaining dy
  // Pure Y engages M2 + M3 equally, M1 is idle
  if (fabsf(dy) >= 0.5f) {
    resetOdometry();
    moveTo(0, dy);                // Pure Y — no X component
    gcY = absY;
  }

  // Ensure exact final position (floating point cleanup)
  gcX = absX;
  gcY = absY;
}

// ============================================================
//  G-CODE EXECUTION: SMOOTH ARC (velocity sweep)
//
//  Unified smooth arc traversal for ALL arcs — full circles,
//  partial arcs, even short ones like a smile.
//
//  Drives tangent to the arc continuously (no start-stop segments).
//  Tracks progress via: theta = path_length / radius
//  Speed ramps down near the end to prevent overshoot.
//
//  KEY FIX: Uses the actual starting angle on the circle to
//  compute the correct tangent direction. Previously it always
//  started driving rightward (+X) regardless of where on the
//  circle the robot was, which squashed the shape.
//
//  Tangent math:
//    Position on circle at angle α: (cx + r·cos(α), cy + r·sin(α))
//    CCW tangent at α: (-sin(α), cos(α))
//    CW  tangent at α: (sin(α), -cos(α))
// ============================================================

void gcSmoothArc(float radius, float startAng, float sweep, bool ccw) {
  resetOdometry();
  resetPathLength();

  // Scale cruise speed for short arcs — don't blast full speed
  // on a tiny 20mm smile arc. Linear scale below 80mm arc length.
  float arcLen = radius * sweep;
  int cruise = arcCruiseSpd;
  if (arcLen < 80.0f) {
    cruise = arcMinSpd + (int)((arcCruiseSpd - arcMinSpd) * (arcLen / 80.0f));
  }

  float theta = 0.0f;
  unsigned long t0 = millis();

  while (theta < sweep) {
    // Deceleration ramp near the end
    float remaining = sweep - theta;
    int spd;
    if (remaining < ARC_RAMP_RAD) {
      float frac = remaining / ARC_RAMP_RAD;  // 1.0 → 0.0
      spd = arcMinSpd + (int)((cruise - arcMinSpd) * frac);
    } else {
      spd = cruise;
    }

    // Current angle on the circle
    float ang = ccw ? (startAng + theta) : (startAng - theta);

    // Tangent direction (perpendicular to radius, in direction of travel)
    float vx, vy;
    if (ccw) { vx = -sinf(ang); vy =  cosf(ang); }  // CCW tangent
    else     { vx =  sinf(ang); vy = -cosf(ang); }   // CW tangent

    driveVelocity(vx, vy, spd);
    delay(1);

    updateOdometry();
    updatePathLength();
    theta = pathLen / radius;  // angle = arc_length / radius

    if (millis() - t0 > ARC_TIMEOUT_MS) {
      Serial.println(F("arc timeout"));
      break;
    }
  }
  stopAll();
}

// ============================================================
//  G-CODE EXECUTION: ARC (G2/G3)
//
//  All arcs use smooth velocity sweep — full circles, partial
//  arcs, and short arcs like smiles all get the same smooth
//  continuous treatment.
//
//  Parameters:
//    endX, endY — arc endpoint (absolute)
//    ci, cj — offset from current position to arc center
//    ccw — true for G3 (counter-clockwise), false for G2 (clockwise)
// ============================================================

void gcArc(float endX, float endY, float ci, float cj, bool ccw) {
  // Arc center in absolute coordinates
  float cx = gcX + ci;
  float cy = gcY + cj;
  float radius = sqrtf(ci * ci + cj * cj);

  // Starting angle (from center to current position)
  float startAng = atan2f(gcY - cy, gcX - cx);

  // Calculate sweep angle
  float sweep;
  if (fabsf(endX - gcX) < 1.0f && fabsf(endY - gcY) < 1.0f) {
    // Full circle (endpoint ≈ startpoint)
    sweep = 2.0f * PI;
  } else {
    // Partial arc
    float endAng = atan2f(endY - cy, endX - cx);
    if (ccw) {
      sweep = endAng - startAng;
      if (sweep <= 0) sweep += 2.0f * PI;
    } else {
      sweep = startAng - endAng;
      if (sweep <= 0) sweep += 2.0f * PI;
    }
  }

  Serial.print(F("Arc R=")); Serial.print(radius);
  Serial.print(F(" sweep=")); Serial.println(sweep * 180.0f / PI);

  penDown();
  gcSmoothArc(radius, startAng, sweep, ccw);
  gcX = endX; gcY = endY;
}

// ============================================================
//  G-CODE PARSER
//
//  Reads text G-code lines and extracts commands + parameters.
// ============================================================

// Find a parameter letter in a G-code line and return its numeric value
// Example: parseParam("G1 X50.5 Y-20", 'X', 0) → 50.5
float parseParam(const char *line, char param, float defVal) {
  const char *p = line;
  while (*p) {
    if (*p == param || *p == (param + 32)) {  // Case insensitive
      return atof(p + 1);
    }
    p++;
  }
  return defVal;  // Not found
}

// Parse the command code from a G-code line
// Returns: G0→0, G1→10, G2→20, G3→30, M3→1003, M5→1005, -1→skip
int parseCommand(const char *line) {
  const char *p = line;
  while (*p == ' ') p++;                              // Skip spaces
  if (*p == ';' || *p == '\0' || *p == '\n') return -1;  // Comment/empty

  if (*p == 'G' || *p == 'g') return (int)(atof(p + 1) * 10);
  if (*p == 'M' || *p == 'm') return 1000 + atoi(p + 1);
  return -1;
}

// Execute one G-code line
void executeLine(const char *line) {
  int cmd = parseCommand(line);
  if (cmd < 0) return;  // Skip comments and empty lines

  float x, y, i, j;

  switch (cmd) {
    case 0:   // G0 — Rapid move (pen up)
      x = parseParam(line, 'X', gcX);
      y = parseParam(line, 'Y', gcY);
      Serial.print(F("G0 ")); Serial.print(x); Serial.print(F(",")); Serial.println(y);
      penUp();
      useTravelPID();
      gcMoveTo(x, y);
      break;

    case 10:  // G1 — Linear draw (pen down)
      x = parseParam(line, 'X', gcX);
      y = parseParam(line, 'Y', gcY);
      Serial.print(F("G1 ")); Serial.print(x); Serial.print(F(",")); Serial.println(y);
      penDown();
      useLinePID();
      gcMoveTo(x, y);
      break;

    case 20:  // G2 — Clockwise arc
      x = parseParam(line, 'X', gcX);
      y = parseParam(line, 'Y', gcY);
      i = parseParam(line, 'I', 0);
      j = parseParam(line, 'J', 0);
      Serial.print(F("G2 arc to ")); Serial.print(x); Serial.print(F(",")); Serial.println(y);
      gcArc(x, y, i, j, false);
      break;

    case 30:  // G3 — Counter-clockwise arc
      x = parseParam(line, 'X', gcX);
      y = parseParam(line, 'Y', gcY);
      i = parseParam(line, 'I', 0);
      j = parseParam(line, 'J', 0);
      Serial.print(F("G3 arc to ")); Serial.print(x); Serial.print(F(",")); Serial.println(y);
      gcArc(x, y, i, j, true);
      break;

    case 1003: // M3 — Pen down
      penDown();
      break;

    case 1005: // M5 — Pen up
      penUp();
      break;
  }
}

// ============================================================
//  G-CODE PROGRAM RUNNER
//
//  Reads a PROGMEM string line by line and executes each.
//  Uses a 64-byte buffer for one line at a time.
// ============================================================

void runGCodeProgram(const char *pgmStr) {
  char buf[64];           // Line buffer
  uint8_t bi = 0;         // Buffer index
  uint16_t pi = 0;        // PROGMEM index

  while (true) {
    char c = pgm_read_byte(&pgmStr[pi++]);  // Read one char from flash

    if (c == '\n' || c == '\0') {
      buf[bi] = '\0';                 // Null-terminate
      if (bi > 0) executeLine(buf);   // Execute non-empty lines
      bi = 0;                         // Reset buffer
      if (c == '\0') break;           // End of program
    } else {
      if (bi < 63) buf[bi++] = c;     // Buffer character (overflow safe)
    }
  }
}

// ============================================================
//  ENCODER AUTO-CALIBRATION
//
//  Spins each motor FORWARD briefly, checks if encoder counts
//  go up or down. If down, flips polarity for that motor.
//  This fixes reversed encoder wiring automatically.
// ============================================================

void calibrateEncoders() {
  Serial.println(F("\n--- Encoder Calibration ---"));
  Adafruit_DCMotor *motors[3] = {m1, m2, m3};

  for (uint8_t i = 0; i < 3; i++) {
    noInterrupts(); int before = posi[i]; interrupts();

    motors[i]->setSpeed(180);     // ~70% power
    motors[i]->run(FORWARD);
    delay(300);                   // Spin for 300ms
    motors[i]->run(RELEASE);
    motors[i]->setSpeed(0);
    delay(100);                   // Coast to stop

    noInterrupts(); int after = posi[i]; interrupts();
    int delta = after - before;

    Serial.print(F("M")); Serial.print(i + 1);
    Serial.print(F(": ")); Serial.print(delta);

    if (delta > 5)       { encSign[i] = 1;  Serial.println(F(" OK")); }
    else if (delta < -5) { encSign[i] = -1; Serial.println(F(" REVERSED")); }
    else                 { encSign[i] = 1;  Serial.println(F(" WARNING")); }
  }
  Serial.println(F("--- Done ---\n"));
}

// ============================================================
//  SETUP — runs once on power-on/reset
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println(F("\n=== Omni G-code Plotter ==="));

  // Pre-compute wheel angle trig (used in kinematics)
  for (uint8_t i = 0; i < 3; i++) {
    sinW[i] = sinf(W_ANG[i]);
    cosW[i] = cosf(W_ANG[i]);
  }

  // Initialize motor shield
  if (!AFMS.begin()) {
    Serial.println(F("Motor Shield not found!"));
    while (1);
  }
  stopAll();

  // Initialize pen servo
  penServo.attach(SERVO_PIN);
  penDown();
  delay(100); 
  penUp();

  // Configure encoder pins
  pinMode(ENC1_A, INPUT); pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT); pinMode(ENC2_B, INPUT);
  pinMode(ENC3_A, INPUT); pinMode(ENC3_B, INPUT);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2ISR, RISING);
  enc3PrevA = digitalRead(ENC3_A);
  PCICR  |= (1 << PCIE0);     // Enable pin-change interrupt for port B
  PCMSK0 |= (1 << PCINT0);    // Enable pin 8 specifically

  // Calibrate encoders
  calibrateEncoders();

  // Clear encoder counters so calibration ticks don't affect first move
  noInterrupts();
  posi[0] = 0; posi[1] = 0; posi[2] = 0;
  interrupts();
  posPrev[0] = 0; posPrev[1] = 0; posPrev[2] = 0;
  posX = 0; posY = 0;

  // Configure start button (wired to GND, uses internal pullup)
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println(F("Press button to start drawing..."));
}

// ============================================================
//  LOOP — waits for button press, draws, then waits again
//
//  After each drawing completes, the robot is ready for another
//  button press. No need to reset the Arduino between drawings.
//  The G-code state resets each time so the robot draws from
//  wherever it currently sits (that position becomes the new origin).
// ============================================================

void loop() {
  // Wait for button press (pin reads LOW when pressed)
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(200);  // Debounce — ignore contact bounce from the switch

    Serial.println(F("GO!"));

    // Reset G-code state — current position becomes the new origin (0,0)
    gcX = 0;
    gcY = 0;
    penIsDown = false;   // Ensure pen state is clean
    penServo.write(PEN_UP_ANGLE);  // Make sure pen is up

    // Run the G-code program
    runGCodeProgram(gcode_program);

    // Drawing complete
    stopAll();
    penUp();
    Serial.println(F("\nDone! Press button to draw again..."));

    // Wait for button to be released before accepting another press
    // This prevents accidental double-triggers
    while (digitalRead(BUTTON_PIN) == LOW) { delay(10); }
    delay(200);  // Debounce the release
  }
}
