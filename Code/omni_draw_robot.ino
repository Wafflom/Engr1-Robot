/*
 * Omni-Wheel Drawing Robot — 25cm Star
 *
 * Hardware:
 *   Arduino Uno R3 + Adafruit Motor Shield V2
 *   3x N20 motors with magnetic encoders (6V, 1:50)
 *   M1 = front (90°), M2 = rear-left (210°), M3 = rear-right (330°)
 *   SG90 servo on pin 10, button on pin 12
 *   Wheel diameter: 36mm, robot radius: 88mm
 *
 * On power-up: calibrates encoder directions automatically.
 * Press button to draw a 25cm radius star.
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

// ---- PINS ----
#define ENC1_A    2
#define ENC1_B    4
#define ENC2_A    3
#define ENC2_B    5
#define ENC3_A    8
#define ENC3_B    7
#define BTN_PIN   12
#define SERVO_PIN 10

// ---- PHYSICAL ----
// Encoder: 7 pole pairs = 14 edges per motor rev (CHANGE interrupt)
// Gear ratio 1:50 → 14 * 50 = 700 ticks per wheel revolution
#define ENCODER_CPR   700.0f
#define WHEEL_DIA_MM  36.0f
#define MM_PER_TICK   ((PI * WHEEL_DIA_MM) / ENCODER_CPR)

// ---- SERVO ----
#define PEN_UP_ANGLE    45
#define PEN_DOWN_ANGLE  0
#define PEN_SETTLE_MS   300

// ---- PID ----
#define KP  8.0f
#define KI  1.0f
#define KD  0.5f

// ---- MOTION ----
#define LOOP_MS       10
#define POS_TOL_MM    2.0f
#define SPEED         255
#define INTEGRAL_CAP  300.0f

// ---- STAR ----
#define STAR_RADIUS   250.0f   // 25cm

// ---- GLOBALS ----
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *mot[3];
Servo penServo;

volatile long encTicks[3] = {0, 0, 0};
long prevTicks[3] = {0, 0, 0};
int8_t encSign[3] = {1, 1, 1};   // corrected by calibration
float sinW[3], cosW[3];
float posX = 0, posY = 0;
float tgtX = 0, tgtY = 0;
bool hasTarget = false;
float intgX = 0, intgY = 0;
float peX = 0, peY = 0;
unsigned long lastLoop = 0;

// ---- ENCODER ISRs ----
// CHANGE mode: fires on both rising AND falling edges of channel A.
// Direction: if A==B → count up, if A!=B → count down.
// Auto-calibration at startup corrects the sign if needed.
void isr1() { encTicks[0] += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? 1 : -1; }
void isr2() { encTicks[1] += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? 1 : -1; }

// Encoder 3: pin-change interrupt already fires on both edges.
ISR(PCINT0_vect) {
  encTicks[2] += (digitalRead(ENC3_A) == digitalRead(ENC3_B)) ? 1 : -1;
}

// ---- MOTOR HELPERS ----
void driveMotor(uint8_t idx, float pwm) {
  if (fabsf(pwm) < 1) {
    mot[idx]->setSpeed(0);
    mot[idx]->run(RELEASE);
    return;
  }
  mot[idx]->setSpeed((uint8_t)min(fabsf(pwm), 255.0f));
  mot[idx]->run(pwm > 0 ? FORWARD : BACKWARD);
}

void stopAll() {
  for (uint8_t i = 0; i < 3; i++) {
    mot[i]->setSpeed(0);
    mot[i]->run(RELEASE);
  }
}

// ---- ODOMETRY ----
// Reads encoders, applies sign correction, updates posX/posY.
void updateOdometry() {
  noInterrupts();
  long snap[3] = { encTicks[0], encTicks[1], encTicks[2] };
  interrupts();

  float d[3];
  for (uint8_t i = 0; i < 3; i++) {
    d[i] = (float)(snap[i] - prevTicks[i]) * MM_PER_TICK * encSign[i];
    prevTicks[i] = snap[i];
  }

  posX += (2.0f / 3.0f) * (-sinW[0]*d[0] - sinW[1]*d[1] - sinW[2]*d[2]);
  posY += (2.0f / 3.0f) * ( cosW[0]*d[0] + cosW[1]*d[1] + cosW[2]*d[2]);
}

// ---- PID ----
bool pidStep(float dt) {
  if (!hasTarget) return true;

  float ex = tgtX - posX;
  float ey = tgtY - posY;
  float dist = sqrtf(ex * ex + ey * ey);

  if (dist < POS_TOL_MM) {
    stopAll();
    hasTarget = false;
    intgX = intgY = peX = peY = 0;
    return true;
  }

  intgX = constrain(intgX + ex * dt, -INTEGRAL_CAP, INTEGRAL_CAP);
  intgY = constrain(intgY + ey * dt, -INTEGRAL_CAP, INTEGRAL_CAP);

  float dErrX = (ex - peX) / dt;
  float dErrY = (ey - peY) / dt;

  float cx = KP * ex + KI * intgX + KD * dErrX;
  float cy = KP * ey + KI * intgY + KD * dErrY;
  peX = ex;
  peY = ey;

  // Inverse kinematics: velocity -> wheel speeds
  float w[3];
  for (uint8_t i = 0; i < 3; i++)
    w[i] = -sinW[i] * cx + cosW[i] * cy;

  // Normalize so peak wheel = SPEED (full power)
  float pk = max(max(fabsf(w[0]), fabsf(w[1])), fabsf(w[2]));
  if (pk > 0.001f) {
    float s = (float)SPEED / pk;
    w[0] *= s; w[1] *= s; w[2] *= s;
  }

  driveMotor(0, w[0]);
  driveMotor(1, w[1]);
  driveMotor(2, w[2]);
  return false;
}

// ---- PEN ----
void penUp()   { penServo.write(PEN_UP_ANGLE);   delay(PEN_SETTLE_MS); }
void penDown() { penServo.write(PEN_DOWN_ANGLE);  delay(PEN_SETTLE_MS); }

// ---- MOVE TO POSITION (blocking) ----
void moveTo(float x, float y) {
  tgtX = x;
  tgtY = y;
  hasTarget = true;
  intgX = intgY = peX = peY = 0;

  // Timeout scales with distance — 50ms per mm, minimum 5s
  float dx = x - posX;
  float dy = y - posY;
  float dist = sqrtf(dx * dx + dy * dy);
  unsigned long timeout = max(5000UL, (unsigned long)(dist * 50));
  unsigned long deadline = millis() + timeout;
  unsigned long lastPrint = 0;

  Serial.print(F("-> "));
  Serial.print(x, 1);
  Serial.print(F(", "));
  Serial.println(y, 1);

  while (hasTarget) {
    if (millis() >= deadline) {
      Serial.println(F("TIMEOUT"));
      stopAll();
      hasTarget = false;
      return;
    }

    unsigned long now = millis();
    if (now - lastLoop >= LOOP_MS) {
      float dt = (float)(now - lastLoop) / 1000.0f;
      lastLoop = now;
      updateOdometry();
      pidStep(dt);
    }

    // Print position every 500ms for debugging
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      Serial.print(F("  at "));
      Serial.print(posX, 1);
      Serial.print(F(", "));
      Serial.println(posY, 1);
    }
  }
}

// ---- HOME RESET ----
void homeReset() {
  stopAll();
  noInterrupts();
  encTicks[0] = encTicks[1] = encTicks[2] = 0;
  interrupts();
  prevTicks[0] = prevTicks[1] = prevTicks[2] = 0;
  posX = posY = 0;
  hasTarget = false;
  intgX = intgY = peX = peY = 0;
}

// ---- ENCODER CALIBRATION ----
// Runs each motor FORWARD briefly and checks if encoder counts positive.
// If not, flips the sign so odometry always agrees with motor direction.
void calibrateEncoders() {
  Serial.println(F("Calibrating encoders..."));

  for (uint8_t i = 0; i < 3; i++) {
    // Zero this encoder
    noInterrupts();
    encTicks[i] = 0;
    interrupts();

    // Run motor FORWARD at moderate speed
    mot[i]->setSpeed(200);
    mot[i]->run(FORWARD);
    delay(300);
    mot[i]->setSpeed(0);
    mot[i]->run(RELEASE);
    delay(100);

    // Read result
    noInterrupts();
    long count = encTicks[i];
    interrupts();

    Serial.print(F("  M"));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(count);

    if (count < 0) {
      encSign[i] = -1;
      Serial.println(F(" -> flipped"));
    } else if (count > 0) {
      encSign[i] = 1;
      Serial.println(F(" -> OK"));
    } else {
      encSign[i] = 1;
      Serial.println(F(" -> NO TICKS! Check wiring"));
    }
  }

  // Reset everything after calibration
  noInterrupts();
  encTicks[0] = encTicks[1] = encTicks[2] = 0;
  interrupts();
  prevTicks[0] = prevTicks[1] = prevTicks[2] = 0;
}

// ---- DRAW STAR ----
// 5-pointed pentagram, 25cm radius, centered at starting position.
void drawStar() {
  Serial.println(F("Drawing star..."));

  // 5 vertices on a circle, starting at top (90 degrees)
  float vx[5], vy[5];
  for (int i = 0; i < 5; i++) {
    float a = PI / 2.0f + i * 2.0f * PI / 5.0f;
    vx[i] = STAR_RADIUS * cosf(a);
    vy[i] = STAR_RADIUS * sinf(a);
  }

  // Pentagram order: skip every other vertex
  const int order[] = {0, 2, 4, 1, 3, 0};

  // Travel to first vertex pen up
  penUp();
  moveTo(vx[order[0]], vy[order[0]]);

  // Draw the star
  penDown();
  for (int i = 1; i <= 5; i++) {
    moveTo(vx[order[i]], vy[order[i]]);
  }

  // Lift pen, return home
  penUp();
  moveTo(0, 0);

  stopAll();
  Serial.println(F("Done!"));
}

// ---- SETUP ----
void setup() {
  Serial.begin(115200);
  Serial.println(F("Omni Draw Robot - Star"));

  // Precompute wheel angle trig
  const float angles[] = {
    90.0f  * PI / 180.0f,
    210.0f * PI / 180.0f,
    330.0f * PI / 180.0f
  };
  for (uint8_t i = 0; i < 3; i++) {
    sinW[i] = sinf(angles[i]);
    cosW[i] = cosf(angles[i]);
  }

  // Motor shield
  if (!AFMS.begin()) {
    Serial.println(F("No motor shield!"));
    while (1);
  }
  mot[0] = AFMS.getMotor(1);
  mot[1] = AFMS.getMotor(2);
  mot[2] = AFMS.getMotor(3);
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

  // Encoder interrupts — CHANGE mode for full 14 CPR
  attachInterrupt(digitalPinToInterrupt(ENC1_A), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), isr2, CHANGE);
  // Encoder 3: pin-change interrupt fires on both edges already
  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);

  // Auto-detect encoder direction by running each motor briefly
  calibrateEncoders();

  // Button
  pinMode(BTN_PIN, INPUT_PULLUP);

  lastLoop = millis();
  Serial.println(F("Ready - press button to draw star"));
}

// ---- LOOP ----
void loop() {
  if (digitalRead(BTN_PIN) == LOW) {
    delay(50);
    if (digitalRead(BTN_PIN) == LOW) {
      while (digitalRead(BTN_PIN) == LOW) delay(10);
      homeReset();
      drawStar();
    }
  }
}
