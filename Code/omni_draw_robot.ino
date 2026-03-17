/*
 * Omni Robot — Concentric Shapes Demo (Encoder + PID)
 * ====================================================
 * Draws a circle, pentagram star, and hexagon all sharing
 * the same center point.  Encoder feedback with PID position
 * control following the curiores/ArduinoTutorials pattern.
 *
 * Shape data lives in shapes.h (PROGMEM).
 *
 * At startup the robot auto-calibrates encoder polarity by
 * briefly spinning each motor FORWARD and checking which
 * direction the encoder counts.
 *
 * Hardware:
 *   - Arduino Uno R3
 *   - Adafruit Motor Shield V2 (I2C)
 *   - 3x N20 DC motors w/ magnetic hall-effect encoders (6V, 1:50)
 *     M1 = front (90°), M2 = rear-left (210°), M3 = rear-right (330°)
 *   - SG90 servo on Shield Servo 1 (pin 10)
 *
 * Encoder: 14 CPR × 50:1 gear = 700 ticks/output-rev
 * Wheel diameter: 36 mm → ~0.1616 mm per tick
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
#define SPEED           255

// ---- ENCODER PINS ----
#define ENC1_A  2    // INT0
#define ENC1_B  4
#define ENC2_A  3    // INT1
#define ENC2_B  5
#define ENC3_A  8    // PCINT0
#define ENC3_B  7

// ---- ENCODER / WHEEL MATH ----
#define ENCODER_CPR   700.0f
#define WHEEL_DIA_MM  36.0f
#define MM_PER_TICK   ((PI * WHEEL_DIA_MM) / ENCODER_CPR)

// ---- PID GAINS (following curiores tutorial pattern) ----
// Position PID: kp + kd, no ki (same as tutorial part4)
float kp = 1.0;
float kd = 0.025;
float ki = 0.0;

#define MOVE_TIMEOUT_MS 8000UL
#define POS_TOL_MM      1.0f

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

volatile int posi[3] = {0, 0, 0};

// Encoder polarity: +1 or -1 per motor (set by auto-calibration)
int8_t encSign[3] = {1, 1, 1};

// PID state (following tutorial: micros-based timing)
long prevT = 0;
float eprevX = 0;
float eprevY = 0;
float eintegralX = 0;
float eintegralY = 0;

// Odometry
float posX = 0.0f, posY = 0.0f;
int posPrev[3] = {0, 0, 0};

// Path length tracker (for circle)
float pathLen = 0.0f;
float prevPathX = 0.0f, prevPathY = 0.0f;

// ============================================================
//  ENCODER ISRs
//  Following curiores tutorial pattern exactly:
//  On RISING edge of A, read B: B high → increment, B low → decrement
// ============================================================

void enc1ISR() {
  int b = digitalRead(ENC1_B);
  if (b > 0) { posi[0]++; }
  else       { posi[0]--; }
}

void enc2ISR() {
  int b = digitalRead(ENC2_B);
  if (b > 0) { posi[1]++; }
  else       { posi[1]--; }
}

volatile uint8_t enc3PrevA = 0;
ISR(PCINT0_vect) {
  uint8_t a = digitalRead(ENC3_A);
  if (a && !enc3PrevA) {
    int b = digitalRead(ENC3_B);
    if (b > 0) { posi[2]++; }
    else       { posi[2]--; }
  }
  enc3PrevA = a;
}

// ============================================================
//  ODOMETRY — read encoder ticks, convert to mm
// ============================================================

void updateOdometry() {
  // Atomic read (following tutorial: noInterrupts/interrupts)
  int pos[3];
  noInterrupts();
  pos[0] = posi[0];
  pos[1] = posi[1];
  pos[2] = posi[2];
  interrupts();

  // Apply polarity correction and convert to mm displacement
  float d[3];
  for (uint8_t i = 0; i < 3; i++) {
    d[i] = (float)(pos[i] - posPrev[i]) * MM_PER_TICK * encSign[i];
    posPrev[i] = pos[i];
  }

  // Forward kinematics
  posX += (2.0f / 3.0f) * (-sinW[0]*d[0] - sinW[1]*d[1] - sinW[2]*d[2]);
  posY += (2.0f / 3.0f) * ( cosW[0]*d[0] + cosW[1]*d[1] + cosW[2]*d[2]);
}

void resetOdometry() {
  noInterrupts();
  posPrev[0] = posi[0];
  posPrev[1] = posi[1];
  posPrev[2] = posi[2];
  interrupts();
  posX = 0.0f;
  posY = 0.0f;
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
//  Following curiores tutorial: setMotor(dir, pwmVal, ...)
//  Adapted for Adafruit Motor Shield (no separate PWM/IN pins)
// ============================================================

void stopAll() {
  m1->run(RELEASE); m1->setSpeed(0);
  m2->run(RELEASE); m2->setSpeed(0);
  m3->run(RELEASE); m3->setSpeed(0);
}

void setMotor(Adafruit_DCMotor *m, int dir, int pwmVal) {
  m->setSpeed(pwmVal);
  if (dir == 1)       { m->run(FORWARD);  }
  else if (dir == -1) { m->run(BACKWARD); }
  else                { m->run(RELEASE);  }
}

// ============================================================
//  INVERSE KINEMATICS
// ============================================================

void inverseKinematics(float vx, float vy, float *w) {
  for (uint8_t i = 0; i < 3; i++)
    w[i] = -sinW[i] * vx + cosW[i] * vy;
}

// Drive open-loop at full speed in direction (vx, vy)
void driveVelocity(float vx, float vy) {
  float w[3];
  inverseKinematics(vx, vy, w);

  float peak = max(max(fabsf(w[0]), fabsf(w[1])), fabsf(w[2]));
  if (peak > 0.001f) {
    float s = (float)SPEED / peak;
    w[0] *= s;  w[1] *= s;  w[2] *= s;
  }

  Adafruit_DCMotor *motors[3] = {m1, m2, m3};
  for (uint8_t i = 0; i < 3; i++) {
    int pwr = (int)fabs(w[i]);
    if (pwr > 255) pwr = 255;
    int dir = 1;
    if (w[i] < 0) dir = -1;
    if (pwr < 5) dir = 0;
    setMotor(motors[i], dir, pwr);
  }
}

// ============================================================
//  PID MOVE TO
//
//  Following curiores tutorial pattern exactly:
//  - micros() for deltaT
//  - error = target - current (in mm, since we have XY odometry)
//  - u = kp*e + kd*dedt + ki*eintegral
//  - pwr = fabs(u), clamped to 255
//  - dir from sign of u
//  - No anti-windup, no deadband, no boost
//  - Loop runs as fast as possible (no fixed interval)
// ============================================================

void moveTo(float targetX, float targetY) {
  eprevX = 0;
  eprevY = 0;
  eintegralX = 0;
  eintegralY = 0;
  prevT = micros();

  unsigned long deadline = millis() + MOVE_TIMEOUT_MS;

  while (true) {
    // time difference (following tutorial exactly)
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;
    prevT = currT;

    // update position from encoders
    updateOdometry();

    // error
    float eX = targetX - posX;
    float eY = targetY - posY;
    float dist = sqrtf(eX * eX + eY * eY);

    // arrived?
    if (dist < POS_TOL_MM) {
      stopAll();
      break;
    }

    // timeout?
    if (millis() >= deadline) {
      Serial.print(F("timeout pos="));
      Serial.print(posX); Serial.print(F(",")); Serial.println(posY);
      stopAll();
      break;
    }

    // derivative
    float dedtX = (eX - eprevX) / deltaT;
    float dedtY = (eY - eprevY) / deltaT;

    // integral
    eintegralX = eintegralX + eX * deltaT;
    eintegralY = eintegralY + eY * deltaT;

    // control signal
    float uX = kp * eX + kd * dedtX + ki * eintegralX;
    float uY = kp * eY + kd * dedtY + ki * eintegralY;

    // convert to wheel speeds via inverse kinematics
    float w[3];
    inverseKinematics(uX, uY, w);

    // apply to motors (following tutorial: fabs for power, sign for direction)
    Adafruit_DCMotor *motors[3] = {m1, m2, m3};
    for (uint8_t i = 0; i < 3; i++) {
      int pwr = (int)fabs(w[i]);
      if (pwr > 255) pwr = 255;
      int dir = 1;
      if (w[i] < 0) dir = -1;
      if (pwr < 5) dir = 0;
      setMotor(motors[i], dir, pwr);
    }

    // store previous error
    eprevX = eX;
    eprevY = eY;
  }
}

// ============================================================
//  PEN HELPERS
// ============================================================

void penUp()   { penServo.write(PEN_UP_ANGLE);   delay(PEN_SETTLE_MS); }
void penDown() { penServo.write(PEN_DOWN_ANGLE);  delay(PEN_SETTLE_MS); }

// ============================================================
//  CIRCLE (procedural — continuous velocity sweep)
//  Tracks arc length via encoders, advances angle θ = arc/R.
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
//  Reads segment directions from PROGMEM (shapes.h).
//  Uses PID moveTo() for each vertex.
// ============================================================

void drawPolygon(const float segments[][2], uint8_t numSides,
                 float seg_mm, const __FlashStringHelper *name) {
  Serial.println(name);

  // Travel: center → vertex 0 (straight up)
  resetOdometry();
  moveTo(0, RADIUS_MM);

  // Pen down, draw all sides
  penDown();
  resetOdometry();

  float cumX = 0.0f, cumY = 0.0f;
  for (uint8_t s = 0; s < numSides; s++) {
    float dx = pgm_read_float(&segments[s][0]);
    float dy = pgm_read_float(&segments[s][1]);

    // Normalize direction, scale to segment length
    float len = sqrtf(dx * dx + dy * dy);
    if (len > 0.001f) {
      cumX += (dx / len) * seg_mm;
      cumY += (dy / len) * seg_mm;
    }

    moveTo(cumX, cumY);
  }

  stopAll();
  penUp();

  // Travel: vertex 0 → center
  resetOdometry();
  moveTo(0, -RADIUS_MM);
}

// ============================================================
//  ENCODER AUTO-CALIBRATION
//  Spins each motor FORWARD, checks encoder direction,
//  auto-flips polarity if reversed.
// ============================================================

void calibrateEncoders() {
  Serial.println(F("\n--- Encoder Calibration ---"));

  Adafruit_DCMotor *motors[3] = {m1, m2, m3};

  for (uint8_t i = 0; i < 3; i++) {
    noInterrupts();
    int before = posi[i];
    interrupts();

    motors[i]->setSpeed(180);
    motors[i]->run(FORWARD);
    delay(300);
    motors[i]->run(RELEASE);
    motors[i]->setSpeed(0);
    delay(100);

    noInterrupts();
    int after = posi[i];
    interrupts();

    int delta = after - before;

    Serial.print(F("M")); Serial.print(i + 1);
    Serial.print(F(" FORWARD: ticks="));
    Serial.print(delta);

    if (delta > 5) {
      encSign[i] = 1;
      Serial.println(F("  OK"));
    } else if (delta < -5) {
      encSign[i] = -1;
      Serial.println(F("  REVERSED -> flipped"));
    } else {
      encSign[i] = 1;
      Serial.println(F("  WARNING: no counts!"));
    }
  }

  Serial.println(F("--- Calibration Done ---\n"));
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println(F("\n=== Concentric Shapes (Encoder+PID) ==="));
  Serial.print(F("Radius: ")); Serial.print(RADIUS_MM); Serial.println(F(" mm"));

  for (uint8_t i = 0; i < 3; i++) {
    sinW[i] = sinf(W_ANG[i]);
    cosW[i] = cosf(W_ANG[i]);
  }

  if (!AFMS.begin()) {
    Serial.println(F("Motor Shield not found!"));
    while (1);
  }
  stopAll();

  penServo.attach(SERVO_PIN);
  penUp();

  // Encoder pins
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);
  pinMode(ENC3_A, INPUT);
  pinMode(ENC3_B, INPUT);

  // Interrupts (following tutorial: RISING on channel A)
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

  // ---- Draw all three shapes ----
  drawCircle();
  delay(200);

  drawPolygon(star_segments, STAR_NUM_SIDES, STAR_SEG_MM, F("Star"));
  delay(200);

  drawPolygon(hex_segments, HEX_NUM_SIDES, HEX_SEG_MM, F("Hexagon"));

  stopAll();
  Serial.println(F("\nDone! Reset to repeat."));
}

void loop() { }
