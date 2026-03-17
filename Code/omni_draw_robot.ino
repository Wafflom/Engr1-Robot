/*
 * Omni Robot — Concentric Shapes Demo (Encoder Feedback)
 * =======================================================
 * Draws a circle, pentagram star, and hexagon all sharing
 * the same center point.  Uses encoder feedback for accurate
 * distance control instead of pure timing.
 *
 * Based on the working open-loop version — same shape logic,
 * same inverse kinematics, same motor shield + servo control.
 * Only change: straight-line segments use encoder-measured
 * distance instead of timed duration.  Circle uses encoder
 * arc-length tracking.
 *
 * Hardware:
 *   - Arduino Uno R3
 *   - Adafruit Motor Shield V2
 *   - 3x N20 DC motors w/ magnetic hall-effect encoders (6V, 1:50)
 *     M1 = front (90°), M2 = rear-left (210°), M3 = rear-right (330°)
 *   - SG90 servo on Shield Servo 1 (pin 10)
 *
 * Encoder specs:
 *   14 counts per motor revolution
 *   1:50 gear ratio → 700 counts per output shaft revolution
 *   Wheel diameter: 36 mm → ~0.1616 mm per tick
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

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
#define SPEED           255       // full PWM

// Shape size: radius of the circumscribed circle (mm)
// All shapes (circle, star, hexagon) are inscribed in this radius.
#define RADIUS_MM       50.0f

// Derived segment lengths
#define STAR_SEG_MM     (1.9021f * RADIUS_MM)   // pentagram skip-1 chord
#define HEX_SEG_MM      RADIUS_MM               // hexagon side = circumradius

// ---- ENCODER CONFIG ----
#define ENC1_A  2    // hardware interrupt INT0
#define ENC1_B  4
#define ENC2_A  3    // hardware interrupt INT1
#define ENC2_B  5
#define ENC3_A  8    // pin-change interrupt PCINT0
#define ENC3_B  7

#define ENCODER_CPR   700.0f
#define WHEEL_DIA_MM  36.0f
#define MM_PER_TICK   ((PI * WHEEL_DIA_MM) / ENCODER_CPR)   // ~0.1616 mm

// ---- CONTROL ----
#define TICK_MS         1         // 1 ms control loop — fast for encoders
#define TIMEOUT_MS      8000UL    // safety timeout per segment
#define DIST_TOL_MM     1.5f      // close-enough threshold

// Wheel angles (radians)
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

// Odometry — cumulative displacement in mm
float odoX = 0.0f, odoY = 0.0f;

// ============================================================
//  ENCODER ISRs
//
//  Rising edge on channel A → read channel B for direction.
//  Direction sign: if motor FORWARD moves the wheel such that
//  B is LOW on A's rising edge, that's the positive direction.
//  If your robot moves BACKWARD instead of FORWARD, swap the
//  +1/-1 in all three ISRs below.
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
  odoX = 0.0f;
  odoY = 0.0f;
}

void updateOdometry() {
  noInterrupts();
  long s0 = ticks[0], s1 = ticks[1], s2 = ticks[2];
  interrupts();

  float d[3];
  d[0] = (float)(s0 - tickSnap[0]) * MM_PER_TICK;
  d[1] = (float)(s1 - tickSnap[1]) * MM_PER_TICK;
  d[2] = (float)(s2 - tickSnap[2]) * MM_PER_TICK;
  tickSnap[0] = s0;
  tickSnap[1] = s1;
  tickSnap[2] = s2;

  // Forward kinematics — convert wheel displacements to robot XY
  odoX += (2.0f / 3.0f) * (-sinW[0]*d[0] - sinW[1]*d[1] - sinW[2]*d[2]);
  odoY += (2.0f / 3.0f) * ( cosW[0]*d[0] + cosW[1]*d[1] + cosW[2]*d[2]);
}

// Total path length tracker (for circle arc measurement)
float pathLen = 0.0f;
float prevOdoX = 0.0f, prevOdoY = 0.0f;

void resetPathLength() {
  pathLen = 0.0f;
  prevOdoX = odoX;
  prevOdoY = odoY;
}

void updatePathLength() {
  float dx = odoX - prevOdoX;
  float dy = odoY - prevOdoY;
  pathLen += sqrtf(dx * dx + dy * dy);
  prevOdoX = odoX;
  prevOdoY = odoY;
}

// ============================================================
//  MOTOR CONTROL  (same as working open-loop code)
// ============================================================

void stopAll() {
  m1->run(RELEASE); m1->setSpeed(0);
  m2->run(RELEASE); m2->setSpeed(0);
  m3->run(RELEASE); m3->setSpeed(0);
}

void driveMotor(Adafruit_DCMotor *m, float pwm) {
  if (fabsf(pwm) < 5) { m->setSpeed(0); m->run(RELEASE); return; }
  m->setSpeed((uint8_t)min(fabsf(pwm), 255.0f));
  m->run(pwm > 0 ? FORWARD : BACKWARD);
}

// ============================================================
//  INVERSE KINEMATICS → DRIVE
//  Normalizes so peak wheel = SPEED (255) PWM
// ============================================================

void driveVelocity(float vx, float vy) {
  float w[3];
  for (uint8_t i = 0; i < 3; i++)
    w[i] = -sinW[i] * vx + cosW[i] * vy;

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
//  DRIVE STRAIGHT — encoder-measured distance
//
//  Drives in direction (vx, vy) at full speed until encoders
//  say we've traveled 'dist_mm' millimeters.  Falls back to
//  timeout if encoders aren't responding.
// ============================================================

void driveStraight(float vx, float vy, float dist_mm) {
  resetOdometry();
  unsigned long t0 = millis();

  while (true) {
    driveVelocity(vx, vy);
    delay(TICK_MS);
    updateOdometry();

    // Distance traveled (magnitude of displacement vector)
    float traveled = sqrtf(odoX * odoX + odoY * odoY);

    if (traveled >= dist_mm - DIST_TOL_MM) break;

    if (millis() - t0 > TIMEOUT_MS) {
      Serial.print(F("timeout dist="));
      Serial.println(traveled);
      break;
    }
  }
}

// ============================================================
//  PEN HELPERS
// ============================================================

void penUp()   { penServo.write(PEN_UP_ANGLE);   delay(PEN_SETTLE_MS); }
void penDown() { penServo.write(PEN_DOWN_ANGLE);  delay(PEN_SETTLE_MS); }

// ============================================================
//  CIRCLE
//
//  Velocity direction sweeps continuously.  Instead of using
//  a fixed time period, we track total arc length via encoders
//  and advance the angle: θ = arc_length / radius.
//  Stops when θ ≥ 2π (one full revolution).
// ============================================================

void drawCircle() {
  Serial.println(F("Circle"));

  // Travel: center → bottom of circle (pen up)
  driveStraight(0, -1, RADIUS_MM);
  stopAll();

  // Pen down, draw one full circle
  penDown();
  resetOdometry();
  resetPathLength();

  float targetArc = 2.0f * PI * RADIUS_MM;
  float theta = 0.0f;
  unsigned long t0 = millis();

  while (theta < 2.0f * PI) {
    // Drive tangent to circle at current angle
    driveVelocity(cosf(theta), sinf(theta));
    delay(TICK_MS);

    updateOdometry();
    updatePathLength();

    // Advance angle based on arc distance
    theta = pathLen / RADIUS_MM;

    if (millis() - t0 > 15000UL) {
      Serial.println(F("circle timeout"));
      break;
    }
  }

  stopAll();
  penUp();

  // Travel: bottom of circle → center
  driveStraight(0, 1, RADIUS_MM);
  stopAll();
}

// ============================================================
//  STAR (5-pointed pentagram)
//
//  Same vertex math as working code, but segments use
//  encoder-measured distance instead of timed duration.
// ============================================================

void drawStar() {
  Serial.println(F("Star"));

  float vx[5], vy[5];
  for (int i = 0; i < 5; i++) {
    float a = PI / 2.0f + i * 2.0f * PI / 5.0f;
    vx[i] = cosf(a);
    vy[i] = sinf(a);
  }

  // Travel: center → vertex 0 (top)
  driveStraight(0, 1, RADIUS_MM);
  stopAll();

  // Pen down, draw pentagram
  penDown();
  const int order[] = {0, 2, 4, 1, 3, 0};
  for (int s = 0; s < 5; s++) {
    float dx = vx[order[s+1]] - vx[order[s]];
    float dy = vy[order[s+1]] - vy[order[s]];
    driveStraight(dx, dy, STAR_SEG_MM);
  }
  stopAll();
  penUp();

  // Travel: vertex 0 (top) → center
  driveStraight(0, -1, RADIUS_MM);
  stopAll();
}

// ============================================================
//  HEXAGON
//
//  Same vertex math as working code, encoder-measured distance.
// ============================================================

void drawHexagon() {
  Serial.println(F("Hexagon"));

  float vx[6], vy[6];
  for (int i = 0; i < 6; i++) {
    float a = PI / 2.0f - i * PI / 3.0f;  // CW from top
    vx[i] = cosf(a);
    vy[i] = sinf(a);
  }

  // Travel: center → vertex 0 (top)
  driveStraight(0, 1, RADIUS_MM);
  stopAll();

  // Pen down, draw 6 sides
  penDown();
  for (int s = 0; s < 6; s++) {
    int next = (s + 1) % 6;
    float dx = vx[next] - vx[s];
    float dy = vy[next] - vy[s];
    driveStraight(dx, dy, HEX_SEG_MM);
  }
  stopAll();
  penUp();

  // Travel: vertex 0 (top) → center
  driveStraight(0, -1, RADIUS_MM);
  stopAll();
}

// ============================================================
//  ENCODER SELF-TEST
//
//  Briefly spins each motor and checks that its encoder
//  counts in the expected direction.  Prints results to
//  Serial so you can verify wiring before drawing.
// ============================================================

void encoderSelfTest() {
  Serial.println(F("\n--- Encoder Self-Test ---"));

  Adafruit_DCMotor *motors[3] = {m1, m2, m3};
  const char *names[3] = {"M1", "M2", "M3"};

  for (uint8_t i = 0; i < 3; i++) {
    // Record starting ticks
    noInterrupts();
    long before = ticks[i];
    interrupts();

    // Spin motor FORWARD briefly
    motors[i]->setSpeed(180);
    motors[i]->run(FORWARD);
    delay(300);
    motors[i]->run(RELEASE);
    motors[i]->setSpeed(0);
    delay(100);

    noInterrupts();
    long after = ticks[i];
    interrupts();

    long delta = after - before;
    Serial.print(names[i]);
    Serial.print(F(" FORWARD: ticks="));
    Serial.print(delta);

    if (delta > 5)       Serial.println(F("  OK (positive)"));
    else if (delta < -5) Serial.println(F("  REVERSED — flip ISR sign!"));
    else                 Serial.println(F("  NO COUNTS — check wiring!"));
  }

  Serial.println(F("--- End Self-Test ---\n"));
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println(F("\n=== Concentric Shapes Demo (Encoder) ==="));
  Serial.print(F("Radius: ")); Serial.print(RADIUS_MM); Serial.println(F(" mm"));
  Serial.print(F("Star seg: ")); Serial.print(STAR_SEG_MM); Serial.println(F(" mm"));
  Serial.print(F("Hex seg: ")); Serial.print(HEX_SEG_MM); Serial.println(F(" mm"));

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

  // Run encoder self-test
  encoderSelfTest();

  Serial.println(F("Starting in 3s..."));
  delay(3000);

  // ---- Draw all three shapes from the same center ----
  drawCircle();
  delay(200);

  drawStar();
  delay(200);

  drawHexagon();

  // Done
  stopAll();
  Serial.println(F("\nDone! Reset to repeat."));
}

void loop() { }
