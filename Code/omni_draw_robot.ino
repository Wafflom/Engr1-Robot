/*
 * Omni Robot — Concentric Shapes Demo (Encoder + PID)
 * ====================================================
 * Draws a circle, pentagram star, and hexagon all sharing
 * the same center point.  Uses encoder feedback with PID
 * position control for accurate movement.
 *
 * Shape data lives in shapes.h (PROGMEM).
 *
 * At startup the robot auto-calibrates encoder polarity by
 * briefly spinning each motor FORWARD and checking which
 * direction the encoder counts.  If a motor's encoder counts
 * negative, that encoder's sign is flipped automatically.
 *
 * Hardware:
 *   - Arduino Uno R3
 *   - Adafruit Motor Shield V2
 *   - 3x N20 DC motors w/ magnetic hall-effect encoders (6V, 1:50)
 *     M1 = front (90°), M2 = rear-left (210°), M3 = rear-right (330°)
 *   - SG90 servo on Shield Servo 1 (pin 10)
 *
 * Encoder specs:
 *   14 counts per motor revolution, 1:50 gear ratio
 *   700 counts per output shaft revolution
 *   Wheel diameter: 36 mm → ~0.1616 mm per tick
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "shapes.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *m1 = AFMS.getMotor(1);
Adafruit_DCMotor *m2 = AFMS.getMotor(2);
Adafruit_DCMotor *m3 = AFMS.getMotor(3);
Servo penServo;

// ---- CONFIG ----
#define SERVO_PIN       10
#define PEN_UP_ANGLE    45
#define PEN_DOWN_ANGLE  0
#define PEN_SETTLE_MS   300
#define SPEED           255       // full PWM for open-loop drive

// ---- ENCODER PINS ----
#define ENC1_A  2    // hardware interrupt INT0
#define ENC1_B  4
#define ENC2_A  3    // hardware interrupt INT1
#define ENC2_B  5
#define ENC3_A  8    // pin-change interrupt PCINT0
#define ENC3_B  7

// ---- ENCODER MATH ----
#define ENCODER_CPR   700.0f
#define WHEEL_DIA_MM  36.0f
#define MM_PER_TICK   ((PI * WHEEL_DIA_MM) / ENCODER_CPR)

// ---- PID GAINS ----
#define KP  4.0f
#define KI  0.8f
#define KD  0.5f
#define INTEGRAL_CAP  200.0f

// ---- CONTROL ----
#define LOOP_MS         5         // 200 Hz PID loop
#define POS_TOL_MM      1.0f      // arrival threshold
#define SETTLE_COUNT    5         // consecutive loops within tolerance
#define MAX_PWM         255
#define MOVE_TIMEOUT_MS 8000UL    // safety timeout per segment

// ---- WHEEL GEOMETRY ----
const float W_ANG[3] = {
   90.0f * PI / 180.0f,
  210.0f * PI / 180.0f,
  330.0f * PI / 180.0f
};
float sinW[3], cosW[3];

// ============================================================
//  ENCODER STATE
// ============================================================

volatile long ticks[3] = {0, 0, 0};
long tickSnap[3]       = {0, 0, 0};

// Encoder polarity: +1 or -1 per motor (set by auto-calibration)
int8_t encSign[3] = {1, 1, 1};

// Odometry — position in mm relative to last reset
float posX = 0.0f, posY = 0.0f;

// PID state
float targetX = 0.0f, targetY = 0.0f;
float integralX = 0.0f, integralY = 0.0f;
float prevErrX  = 0.0f, prevErrY  = 0.0f;
unsigned long loopTime = 0;

// Path length tracker (for circle)
float pathLen = 0.0f;
float prevPathX = 0.0f, prevPathY = 0.0f;

// ============================================================
//  ENCODER ISRs
//
//  Raw counting — polarity is corrected in updateOdometry()
//  using encSign[], which is set by auto-calibration.
// ============================================================

void enc1ISR() {
  ticks[0] += digitalRead(ENC1_B) ? -1 : 1;
}

void enc2ISR() {
  ticks[1] += digitalRead(ENC2_B) ? -1 : 1;
}

volatile uint8_t enc3PrevA = 0;
ISR(PCINT0_vect) {
  uint8_t a = digitalRead(ENC3_A);
  if (a && !enc3PrevA) {
    ticks[2] += digitalRead(ENC3_B) ? -1 : 1;
  }
  enc3PrevA = a;
}

// ============================================================
//  ODOMETRY
// ============================================================

void resetOdometry() {
  noInterrupts();
  tickSnap[0] = ticks[0];
  tickSnap[1] = ticks[1];
  tickSnap[2] = ticks[2];
  interrupts();
  posX = 0.0f;
  posY = 0.0f;
}

void updateOdometry() {
  noInterrupts();
  long s0 = ticks[0], s1 = ticks[1], s2 = ticks[2];
  interrupts();

  // Apply polarity correction from auto-calibration
  float d[3];
  d[0] = (float)(s0 - tickSnap[0]) * MM_PER_TICK * encSign[0];
  d[1] = (float)(s1 - tickSnap[1]) * MM_PER_TICK * encSign[1];
  d[2] = (float)(s2 - tickSnap[2]) * MM_PER_TICK * encSign[2];
  tickSnap[0] = s0;
  tickSnap[1] = s1;
  tickSnap[2] = s2;

  // Forward kinematics
  posX += (2.0f / 3.0f) * (-sinW[0]*d[0] - sinW[1]*d[1] - sinW[2]*d[2]);
  posY += (2.0f / 3.0f) * ( cosW[0]*d[0] + cosW[1]*d[1] + cosW[2]*d[2]);
}

void resetPathLength() {
  pathLen = 0.0f;
  prevPathX = posX;
  prevPathY = posY;
}

void updatePathLength() {
  float dx = posX - prevPathX;
  float dy = posY - prevPathY;
  pathLen += sqrtf(dx * dx + dy * dy);
  prevPathX = posX;
  prevPathY = posY;
}

// ============================================================
//  MOTOR CONTROL
// ============================================================

void stopAll() {
  m1->run(RELEASE); m1->setSpeed(0);
  m2->run(RELEASE); m2->setSpeed(0);
  m3->run(RELEASE); m3->setSpeed(0);
}

void driveMotor(Adafruit_DCMotor *m, float pwm) {
  if (fabsf(pwm) < 5) { m->setSpeed(0); m->run(RELEASE); return; }
  m->setSpeed((uint8_t)min(fabsf(pwm), (float)MAX_PWM));
  m->run(pwm > 0 ? FORWARD : BACKWARD);
}

// ============================================================
//  INVERSE KINEMATICS
// ============================================================

void inverseKinematics(float vx, float vy, float *w) {
  for (uint8_t i = 0; i < 3; i++)
    w[i] = -sinW[i] * vx + cosW[i] * vy;
}

// Drive with velocity normalization (for open-loop segments)
void driveVelocity(float vx, float vy) {
  float w[3];
  inverseKinematics(vx, vy, w);

  float peak = max(max(fabsf(w[0]), fabsf(w[1])), fabsf(w[2]));
  if (peak > 0.001f) {
    float s = (float)SPEED / peak;
    w[0] *= s;  w[1] *= s;  w[2] *= s;
  }

  driveMotor(m1, w[0]);
  driveMotor(m2, w[1]);
  driveMotor(m3, w[2]);
}

// ============================================================
//  PID POSITION CONTROLLER
//
//  Moves robot to (targetX, targetY) relative to last reset.
//  Returns true when within POS_TOL_MM for SETTLE_COUNT loops.
// ============================================================

bool runPID(float dt) {
  float errX = targetX - posX;
  float errY = targetY - posY;
  float dist = sqrtf(errX * errX + errY * errY);

  if (dist < POS_TOL_MM) return true;

  // Integrate with anti-windup
  integralX = constrain(integralX + errX * dt, -INTEGRAL_CAP, INTEGRAL_CAP);
  integralY = constrain(integralY + errY * dt, -INTEGRAL_CAP, INTEGRAL_CAP);

  // Derivative
  float dErrX = (errX - prevErrX) / dt;
  float dErrY = (errY - prevErrY) / dt;
  prevErrX = errX;
  prevErrY = errY;

  // PID output → velocity command
  float cmdX = KP * errX + KI * integralX + KD * dErrX;
  float cmdY = KP * errY + KI * integralY + KD * dErrY;

  // Convert to wheel speeds
  float w[3];
  inverseKinematics(cmdX, cmdY, w);

  // Saturate while preserving direction
  float peak = max(max(fabsf(w[0]), fabsf(w[1])), fabsf(w[2]));
  if (peak > MAX_PWM) {
    float scale = (float)MAX_PWM / peak;
    w[0] *= scale;  w[1] *= scale;  w[2] *= scale;
  }

  driveMotor(m1, w[0]);
  driveMotor(m2, w[1]);
  driveMotor(m3, w[2]);

  return false;
}

// ============================================================
//  MOVE TO — blocking PID movement
// ============================================================

void moveTo(float x, float y) {
  targetX = x;
  targetY = y;
  integralX = integralY = 0.0f;
  prevErrX  = prevErrY  = 0.0f;

  uint8_t settleCount = 0;
  unsigned long deadline = millis() + MOVE_TIMEOUT_MS;
  loopTime = millis();

  while (true) {
    if (millis() >= deadline) {
      Serial.print(F("timeout pos="));
      Serial.print(posX); Serial.print(F(",")); Serial.println(posY);
      break;
    }

    unsigned long now = millis();
    if (now - loopTime >= LOOP_MS) {
      float dt = (float)(now - loopTime) / 1000.0f;
      loopTime = now;
      updateOdometry();

      if (runPID(dt)) {
        settleCount++;
        if (settleCount >= SETTLE_COUNT) break;
      } else {
        settleCount = 0;
      }
    }
  }

  stopAll();
}

// ============================================================
//  PEN HELPERS
// ============================================================

void penUp()   { penServo.write(PEN_UP_ANGLE);   delay(PEN_SETTLE_MS); }
void penDown() { penServo.write(PEN_DOWN_ANGLE);  delay(PEN_SETTLE_MS); }

// ============================================================
//  CIRCLE (procedural — continuous velocity sweep)
//
//  Tracks arc length via encoders, advances angle θ = arc/R.
//  Full speed, no PID — just distance-based stopping.
// ============================================================

void drawCircle() {
  Serial.println(F("Circle"));

  // Travel: center → bottom of circle
  resetOdometry();
  moveTo(0, -RADIUS_MM);

  // Pen down, draw circle
  penDown();
  resetOdometry();
  resetPathLength();

  float theta = 0.0f;
  unsigned long t0 = millis();

  while (theta < 2.0f * PI) {
    driveVelocity(cosf(theta), sinf(theta));
    delay(1);

    updateOdometry();
    updatePathLength();
    theta = pathLen / RADIUS_MM;

    if (millis() - t0 > CIRCLE_TIMEOUT_MS) {
      Serial.println(F("circle timeout"));
      break;
    }
  }

  stopAll();
  penUp();

  // Travel: bottom of circle → center
  resetOdometry();
  moveTo(0, RADIUS_MM);
}

// ============================================================
//  GENERIC POLYGON DRAWER
//
//  Reads segment directions from PROGMEM (shapes.h).
//  Uses PID moveTo() for each segment: computes the target
//  position by accumulating direction × distance.
// ============================================================

void drawPolygon(const float segments[][2], uint8_t numSides,
                 float seg_mm, const __FlashStringHelper *name) {
  Serial.println(name);

  // Travel: center → vertex 0 (straight up by RADIUS_MM)
  resetOdometry();
  moveTo(0, RADIUS_MM);

  // Pen down, draw all sides using PID
  penDown();
  resetOdometry();

  float cumX = 0.0f, cumY = 0.0f;
  for (uint8_t s = 0; s < numSides; s++) {
    float dx = pgm_read_float(&segments[s][0]);
    float dy = pgm_read_float(&segments[s][1]);

    // Normalize direction, then scale to segment length
    float len = sqrtf(dx * dx + dy * dy);
    if (len > 0.001f) {
      cumX += (dx / len) * seg_mm;
      cumY += (dy / len) * seg_mm;
    }

    moveTo(cumX, cumY);
  }

  stopAll();
  penUp();

  // Travel: vertex 0 (top) → center
  resetOdometry();
  moveTo(0, -RADIUS_MM);
}

// ============================================================
//  ENCODER AUTO-CALIBRATION
//
//  At startup, briefly spins each motor FORWARD and checks
//  which direction the encoder counts.  If negative, flips
//  that encoder's sign so odometry always matches motor
//  direction.  This eliminates the #1 cause of "robot drives
//  forever" — inverted encoder polarity.
// ============================================================

void calibrateEncoders() {
  Serial.println(F("\n--- Encoder Auto-Calibration ---"));

  Adafruit_DCMotor *motors[3] = {m1, m2, m3};

  for (uint8_t i = 0; i < 3; i++) {
    // Snapshot ticks before spinning
    noInterrupts();
    long before = ticks[i];
    interrupts();

    // Spin motor FORWARD at moderate speed
    motors[i]->setSpeed(180);
    motors[i]->run(FORWARD);
    delay(300);
    motors[i]->run(RELEASE);
    motors[i]->setSpeed(0);
    delay(100);  // let motor coast to stop

    // Snapshot ticks after
    noInterrupts();
    long after = ticks[i];
    interrupts();

    long delta = after - before;

    Serial.print(F("M")); Serial.print(i + 1);
    Serial.print(F(" FORWARD: ticks="));
    Serial.print(delta);

    if (delta > 5) {
      encSign[i] = 1;
      Serial.println(F("  OK"));
    } else if (delta < -5) {
      encSign[i] = -1;
      Serial.println(F("  REVERSED → auto-flipped"));
    } else {
      encSign[i] = 1;
      Serial.println(F("  WARNING: no counts — check wiring!"));
    }
  }

  Serial.println(F("--- Calibration Complete ---\n"));
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println(F("\n=== Concentric Shapes (Encoder+PID) ==="));
  Serial.print(F("Radius: ")); Serial.print(RADIUS_MM); Serial.println(F(" mm"));

  // Pre-compute trig
  for (uint8_t i = 0; i < 3; i++) {
    sinW[i] = sinf(W_ANG[i]);
    cosW[i] = cosf(W_ANG[i]);
  }

  // Motor shield
  if (!AFMS.begin()) {
    Serial.println(F("Motor Shield not found!"));
    while (1);
  }
  stopAll();

  // Servo
  penServo.attach(SERVO_PIN);
  penUp();

  // Encoder pins
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP);
  pinMode(ENC3_B, INPUT_PULLUP);

  // Hardware interrupts for encoders 1 & 2
  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2ISR, RISING);

  // Pin-change interrupt for encoder 3 (pin 8 = PCINT0)
  enc3PrevA = digitalRead(ENC3_A);
  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);

  // Auto-calibrate encoder polarity
  calibrateEncoders();

  Serial.println(F("Starting in 3s..."));
  delay(3000);

  // ---- Draw all three shapes from the same center ----
  drawCircle();
  delay(200);

  drawPolygon(star_segments, STAR_NUM_SIDES, STAR_SEG_MM, F("Star"));
  delay(200);

  drawPolygon(hex_segments, HEX_NUM_SIDES, HEX_SEG_MM, F("Hexagon"));

  // Done
  stopAll();
  Serial.println(F("\nDone! Reset to repeat."));
}

void loop() { }
