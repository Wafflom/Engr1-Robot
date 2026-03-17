/*
 * Omni Robot — Circle + Smiley Face
 * ===================================
 * Draws a circle, then a smiley face inside it (two eyes + mouth).
 * Press button on pin 12 to start. Encoder + PID position control.
 *
 * Hardware:
 *   - Arduino Uno R3
 *   - Adafruit Motor Shield V2 (I2C)
 *   - 3x N20 DC motors w/ magnetic hall-effect encoders (6V, 1:50)
 *     M1 = front (90°), M2 = rear-left (210°), M3 = rear-right (330°)
 *   - SG90 servo on Shield Servo 1 (pin 10)
 *   - Push button on pin 12 (pulled high, press = LOW)
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
#define BUTTON_PIN      12

// ---- ENCODER PINS ----
#define ENC1_A  2
#define ENC1_B  4
#define ENC2_A  3
#define ENC2_B  5
#define ENC3_A  8
#define ENC3_B  7

// ---- ENCODER / WHEEL MATH ----
#define ENCODER_CPR   700.0f
#define WHEEL_DIA_MM  36.0f
#define MM_PER_TICK   ((PI * WHEEL_DIA_MM) / ENCODER_CPR)

// ---- PID GAINS ----
float kp = 40.0;
float kd = 0.5;
float ki = 10.0;

#define MOVE_TIMEOUT_MS 8000UL
#define POS_TOL_MM      3.0f

// ---- WHEEL GEOMETRY ----
const float W_ANG[3] = {
   90.0f * PI / 180.0f,
  210.0f * PI / 180.0f,
  330.0f * PI / 180.0f
};
float sinW[3], cosW[3];

// ---- ENCODER STATE ----
volatile int posi[3] = {0, 0, 0};
int8_t encSign[3] = {1, 1, 1};

// ---- PID STATE ----
long prevT = 0;
float eprevX = 0, eprevY = 0;
float eintegralX = 0, eintegralY = 0;

// ---- ODOMETRY ----
float posX = 0.0f, posY = 0.0f;
int posPrev[3] = {0, 0, 0};

// ---- PATH LENGTH (for circles) ----
float pathLen = 0.0f;
float prevPathX = 0.0f, prevPathY = 0.0f;

// ============================================================
//  ENCODER ISRs (curiores tutorial pattern)
// ============================================================

void enc1ISR() {
  int b = digitalRead(ENC1_B);
  if (b > 0) { posi[0]++; } else { posi[0]--; }
}

void enc2ISR() {
  int b = digitalRead(ENC2_B);
  if (b > 0) { posi[1]++; } else { posi[1]--; }
}

volatile uint8_t enc3PrevA = 0;
ISR(PCINT0_vect) {
  uint8_t a = digitalRead(ENC3_A);
  if (a && !enc3PrevA) {
    int b = digitalRead(ENC3_B);
    if (b > 0) { posi[2]++; } else { posi[2]--; }
  }
  enc3PrevA = a;
}

// ============================================================
//  ODOMETRY
// ============================================================

void updateOdometry() {
  int pos[3];
  noInterrupts();
  pos[0] = posi[0]; pos[1] = posi[1]; pos[2] = posi[2];
  interrupts();

  float d[3];
  for (uint8_t i = 0; i < 3; i++) {
    d[i] = (float)(pos[i] - posPrev[i]) * MM_PER_TICK * encSign[i];
    posPrev[i] = pos[i];
  }

  posX += (2.0f / 3.0f) * (-sinW[0]*d[0] - sinW[1]*d[1] - sinW[2]*d[2]);
  posY += (2.0f / 3.0f) * ( cosW[0]*d[0] + cosW[1]*d[1] + cosW[2]*d[2]);
}

void resetOdometry() {
  noInterrupts();
  posPrev[0] = posi[0]; posPrev[1] = posi[1]; posPrev[2] = posi[2];
  interrupts();
  posX = 0.0f; posY = 0.0f;
}

void resetPathLength() {
  pathLen = 0.0f;
  prevPathX = posX; prevPathY = posY;
}

void updatePathLength() {
  float dx = posX - prevPathX;
  float dy = posY - prevPathY;
  pathLen += sqrtf(dx * dx + dy * dy);
  prevPathX = posX; prevPathY = posY;
}

// ============================================================
//  MOTOR CONTROL (curiores tutorial pattern)
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

void applyWheelSpeeds(float *w) {
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

void driveVelocity(float vx, float vy) {
  float w[3];
  inverseKinematics(vx, vy, w);
  float peak = max(max(fabsf(w[0]), fabsf(w[1])), fabsf(w[2]));
  if (peak > 0.001f) {
    float s = (float)SPEED / peak;
    w[0] *= s; w[1] *= s; w[2] *= s;
  }
  applyWheelSpeeds(w);
}

// ============================================================
//  PID MOVE TO (curiores tutorial pattern)
// ============================================================

void moveTo(float targetX, float targetY) {
  eprevX = 0; eprevY = 0;
  eintegralX = 0; eintegralY = 0;
  prevT = micros();

  unsigned long deadline = millis() + MOVE_TIMEOUT_MS;

  while (true) {
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;
    prevT = currT;

    updateOdometry();

    float eX = targetX - posX;
    float eY = targetY - posY;
    float dist = sqrtf(eX * eX + eY * eY);

    if (dist < POS_TOL_MM) { stopAll(); break; }
    if (millis() >= deadline) {
      Serial.print(F("timeout pos=")); Serial.print(posX);
      Serial.print(F(",")); Serial.println(posY);
      stopAll(); break;
    }

    float dedtX = (eX - eprevX) / deltaT;
    float dedtY = (eY - eprevY) / deltaT;
    eintegralX = eintegralX + eX * deltaT;
    eintegralY = eintegralY + eY * deltaT;

    float uX = kp * eX + kd * dedtX + ki * eintegralX;
    float uY = kp * eY + kd * dedtY + ki * eintegralY;

    float w[3];
    inverseKinematics(uX, uY, w);
    applyWheelSpeeds(w);

    eprevX = eX; eprevY = eY;
  }
}

// ============================================================
//  PEN HELPERS
// ============================================================

void penUp()   { penServo.write(PEN_UP_ANGLE);   delay(PEN_SETTLE_MS); }
void penDown() { penServo.write(PEN_DOWN_ANGLE);  delay(PEN_SETTLE_MS); }

// ============================================================
//  DRAW CIRCLE (open-loop velocity sweep, encoder distance)
//  Draws a circle of given radius, starting from bottom.
//  Assumes robot is already at center before calling.
// ============================================================

void drawCircleAt(float cx, float cy, float radius, unsigned long timeout) {
  // Move from wherever we are to the bottom of the circle
  resetOdometry();
  moveTo(cx, cy - radius);

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
    theta = pathLen / radius;

    if (millis() - t0 > timeout) {
      Serial.println(F("circle timeout"));
      break;
    }
  }

  stopAll();
  penUp();

  // Return to circle center
  resetOdometry();
  moveTo(-cx, -(cy - radius));
}

// ============================================================
//  DRAW MOUTH (line segments through PROGMEM points)
//  Moves to first point pen-down, then traces through all.
// ============================================================

void drawMouth() {
  Serial.println(F("Mouth"));

  // Move to first mouth point
  float firstX = pgm_read_float(&mouth_pts[0][0]);
  float firstY = pgm_read_float(&mouth_pts[0][1]);
  resetOdometry();
  moveTo(firstX, firstY);

  penDown();
  resetOdometry();

  // Draw through remaining points relative to first point
  float cumX = 0.0f, cumY = 0.0f;
  for (uint8_t i = 1; i < MOUTH_NUM_PTS; i++) {
    float px = pgm_read_float(&mouth_pts[i][0]);
    float py = pgm_read_float(&mouth_pts[i][1]);
    cumX = px - firstX;
    cumY = py - firstY;
    moveTo(cumX, cumY);
  }

  stopAll();
  penUp();

  // Return to center: we're at last mouth point
  float lastX = pgm_read_float(&mouth_pts[MOUTH_NUM_PTS - 1][0]);
  float lastY = pgm_read_float(&mouth_pts[MOUTH_NUM_PTS - 1][1]);
  resetOdometry();
  moveTo(-lastX, -lastY);
}

// ============================================================
//  DRAW SMILEY FACE
//  From center: outer circle, left eye, right eye, mouth.
// ============================================================

void drawSmiley() {
  // 1. Outer circle
  Serial.println(F("Outer circle"));
  drawCircleAt(0, 0, RADIUS_MM, CIRCLE_TIMEOUT_MS);
  delay(200);

  // 2. Left eye (small circle)
  Serial.println(F("Left eye"));
  drawCircleAt(-EYE_OFFSET_X, EYE_OFFSET_Y, EYE_RADIUS_MM, EYE_TIMEOUT_MS);
  delay(200);

  // 3. Right eye (small circle)
  Serial.println(F("Right eye"));
  drawCircleAt(EYE_OFFSET_X, EYE_OFFSET_Y, EYE_RADIUS_MM, EYE_TIMEOUT_MS);
  delay(200);

  // 4. Mouth
  drawMouth();
}

// ============================================================
//  ENCODER AUTO-CALIBRATION
// ============================================================

void calibrateEncoders() {
  Serial.println(F("\n--- Encoder Calibration ---"));
  Adafruit_DCMotor *motors[3] = {m1, m2, m3};

  for (uint8_t i = 0; i < 3; i++) {
    noInterrupts(); int before = posi[i]; interrupts();

    motors[i]->setSpeed(180);
    motors[i]->run(FORWARD);
    delay(300);
    motors[i]->run(RELEASE);
    motors[i]->setSpeed(0);
    delay(100);

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
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println(F("\n=== Smiley Face Robot ==="));

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

  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2ISR, RISING);
  enc3PrevA = digitalRead(ENC3_A);
  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);

  calibrateEncoders();

  // Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println(F("Press button to start..."));
  while (digitalRead(BUTTON_PIN) == HIGH) {
    delay(10);
  }
  delay(200);  // debounce
  Serial.println(F("GO!"));

  // Draw!
  drawSmiley();

  stopAll();
  Serial.println(F("\nDone! Reset to repeat."));
}

void loop() { }
