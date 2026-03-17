/*
 * Omni-Wheel Drawing Robot
 *
 * Three-wheel holonomic robot that draws shapes from coordinate data.
 * Each wheel has its own velocity PID loop tracking encoder feedback.
 * A motion planner computes desired wheel speeds from the position
 * error, and the per-wheel PIDs drive each motor to match.
 *
 * Hardware:
 *   Arduino Uno R3 + Adafruit Motor Shield V2
 *   3x N20 DC motors w/ magnetic encoders (6V, 1:50, 14 CPR)
 *   M1 = front (90°), M2 = rear-left (210°), M3 = rear-right (330°)
 *   SG90 servo on pin 10, pushbutton on pin 12
 *   Wheel diameter: 36mm, center-to-wheel: 88mm
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "drawing.h"

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
#define ENCODER_CPR   700.0f    // 14 edges/motor-rev × 50:1 gear
#define WHEEL_DIA_MM  36.0f
#define MM_PER_TICK   ((PI * WHEEL_DIA_MM) / ENCODER_CPR)

// ---- SERVO ----
#define PEN_UP_ANGLE    45
#define PEN_DOWN_ANGLE  0
#define PEN_SETTLE_MS   300

// ---- PER-WHEEL VELOCITY PID ----
#define W_KP  0.3f
#define W_KI  0.1f
#define W_KD  0.005f
#define W_INTEGRAL_CAP  600.0f
#define SPD_FILTER    0.3f      // EMA alpha for wheel speed (0–1, lower = smoother)

// ---- MOTION PLANNER ----
#define RAMP_GAIN     3.0f      // decel near target: speed = dist * RAMP_GAIN
#define MIN_SPEED     15.0f     // mm/s minimum (prevents stall near target)
#define POS_TOL       3.0f      // mm — close enough to stop
#define LOOP_MS       20        // control loop period (ms)

// ---- GLOBALS ----
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *mot[3];
Servo penServo;

volatile long encTicks[3] = {0, 0, 0};
long prevTicks[3] = {0, 0, 0};
int8_t encSign[3] = {1, 1, 1};

float sinW[3], cosW[3];
float posX = 0, posY = 0;

// Per-wheel PID state
float wIntg[3] = {0, 0, 0};
float wPrevErr[3] = {0, 0, 0};
float filtSpd[3] = {0, 0, 0};  // low-pass filtered wheel speed (ticks/s)

unsigned long lastLoop = 0;

// ============================================================
//  ENCODER ISRs — CHANGE mode, 14 counts per motor revolution
// ============================================================
void isr1() { encTicks[0] += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? 1 : -1; }
void isr2() { encTicks[1] += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? 1 : -1; }
ISR(PCINT0_vect) {
  encTicks[2] += (digitalRead(ENC3_A) == digitalRead(ENC3_B)) ? 1 : -1;
}

// ============================================================
//  MOTOR OUTPUT — raw PWM, no tricks
// ============================================================
void setMotor(uint8_t i, int16_t pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm == 0) {
    mot[i]->setSpeed(0);
    mot[i]->run(RELEASE);
  } else if (pwm > 0) {
    mot[i]->setSpeed(pwm);
    mot[i]->run(FORWARD);
  } else {
    mot[i]->setSpeed(-pwm);
    mot[i]->run(BACKWARD);
  }
}

void stopAll() {
  for (uint8_t i = 0; i < 3; i++) {
    setMotor(i, 0);
    wIntg[i] = 0;
    wPrevErr[i] = 0;
    filtSpd[i] = 0;
  }
}

// ============================================================
//  ODOMETRY — read encoders, compute wheel speeds & position
// ============================================================
void readEncoders(float dt, float wheelSpd[3]) {
  noInterrupts();
  long snap[3] = { encTicks[0], encTicks[1], encTicks[2] };
  interrupts();

  float dMM[3];
  for (uint8_t i = 0; i < 3; i++) {
    long delta = snap[i] - prevTicks[i];
    prevTicks[i] = snap[i];
    float signedDelta = (float)delta * encSign[i];
    dMM[i] = signedDelta * MM_PER_TICK;
    float rawSpd = signedDelta / dt;
    filtSpd[i] += SPD_FILTER * (rawSpd - filtSpd[i]);  // EMA low-pass
    wheelSpd[i] = filtSpd[i];
  }

  // Forward kinematics: wheel displacements → robot displacement
  posX += (2.0f/3.0f) * (-sinW[0]*dMM[0] - sinW[1]*dMM[1] - sinW[2]*dMM[2]);
  posY += (2.0f/3.0f) * ( cosW[0]*dMM[0] + cosW[1]*dMM[1] + cosW[2]*dMM[2]);
}

// ============================================================
//  PER-WHEEL VELOCITY PID
//  Input:  target speed (ticks/s), actual speed (ticks/s)
//  Output: PWM value (-255 to 255)
// ============================================================
int16_t wheelPID(uint8_t i, float target, float actual, float dt) {
  float err = target - actual;
  wIntg[i] = constrain(wIntg[i] + err * dt, -W_INTEGRAL_CAP, W_INTEGRAL_CAP);
  float deriv = (err - wPrevErr[i]) / dt;
  wPrevErr[i] = err;
  float out = W_KP * err + W_KI * wIntg[i] + W_KD * deriv;
  return constrain((int16_t)out, -255, 255);
}

// ============================================================
//  PEN
// ============================================================
void penUp()   { penServo.write(PEN_UP_ANGLE);   delay(PEN_SETTLE_MS); }
void penDown() { penServo.write(PEN_DOWN_ANGLE);  delay(PEN_SETTLE_MS); }

// ============================================================
//  MOVE TO POSITION (blocking)
//
//  1. Compute direction and distance to target
//  2. Choose speed (ramp down near target)
//  3. Inverse kinematics → desired wheel speeds (ticks/s)
//  4. Per-wheel PID → PWM for each motor
//  5. Repeat until within POS_TOL or timeout
// ============================================================
void moveTo(float tx, float ty) {
  // Reset wheel PIDs for fresh move
  for (uint8_t i = 0; i < 3; i++) { wIntg[i] = 0; wPrevErr[i] = 0; }

  // Timeout proportional to distance
  float initDist = sqrtf((tx-posX)*(tx-posX) + (ty-posY)*(ty-posY));
  unsigned long deadline = millis() + max(5000UL, (unsigned long)(initDist * 60));
  unsigned long lastPrint = 0;

  Serial.print(F("-> "));
  Serial.print(tx, 1); Serial.print(F(", ")); Serial.println(ty, 1);

  while (true) {
    if (millis() >= deadline) {
      Serial.println(F("TIMEOUT"));
      stopAll();
      return;
    }

    unsigned long now = millis();
    if (now - lastLoop < LOOP_MS) continue;
    float dt = (float)(now - lastLoop) / 1000.0f;
    lastLoop = now;

    // 1. Read encoders → wheel speeds + update position
    float wheelSpd[3];
    readEncoders(dt, wheelSpd);

    // 2. Position error
    float ex = tx - posX;
    float ey = ty - posY;
    float dist = sqrtf(ex * ex + ey * ey);

    if (dist < POS_TOL) {
      stopAll();
      return;
    }

    // 3. Desired robot velocity — direction × speed (no cap, PID saturates at 255)
    float speed = max(MIN_SPEED, dist * RAMP_GAIN);
    float vx = (ex / dist) * speed;     // mm/s
    float vy = (ey / dist) * speed;

    // 4. Inverse kinematics → desired wheel speeds in ticks/s
    float tgtSpd[3];
    for (uint8_t i = 0; i < 3; i++) {
      float wheelMM = -sinW[i] * vx + cosW[i] * vy;
      tgtSpd[i] = wheelMM / MM_PER_TICK;
    }

    // 5. Per-wheel PID → motor PWM
    for (uint8_t i = 0; i < 3; i++) {
      setMotor(i, wheelPID(i, tgtSpd[i], wheelSpd[i], dt));
    }

    // Debug
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      Serial.print(F("  "));
      Serial.print(posX, 1); Serial.print(F(", ")); Serial.println(posY, 1);
    }
  }
}

// ============================================================
//  HOME RESET
// ============================================================
void homeReset() {
  stopAll();
  noInterrupts();
  encTicks[0] = encTicks[1] = encTicks[2] = 0;
  interrupts();
  prevTicks[0] = prevTicks[1] = prevTicks[2] = 0;
  posX = posY = 0;
}

// ============================================================
//  ENCODER CALIBRATION
//  Runs each motor FORWARD briefly, checks encoder sign.
// ============================================================
void calibrateEncoders() {
  Serial.println(F("Calibrating encoders..."));
  for (uint8_t i = 0; i < 3; i++) {
    noInterrupts();
    encTicks[i] = 0;
    interrupts();

    mot[i]->setSpeed(200);
    mot[i]->run(FORWARD);
    delay(300);
    mot[i]->setSpeed(0);
    mot[i]->run(RELEASE);
    delay(100);

    noInterrupts();
    long c = encTicks[i];
    interrupts();

    Serial.print(F("  M")); Serial.print(i + 1);
    Serial.print(F(": ")); Serial.print(c);

    if (c < 0)      { encSign[i] = -1; Serial.println(F(" flipped")); }
    else if (c > 0)  { encSign[i] =  1; Serial.println(F(" ok")); }
    else              { encSign[i] =  1; Serial.println(F(" NO TICKS")); }
  }

  noInterrupts();
  encTicks[0] = encTicks[1] = encTicks[2] = 0;
  interrupts();
  prevTicks[0] = prevTicks[1] = prevTicks[2] = 0;
}

// ============================================================
//  DRAWING ENGINE
//  Reads (x,y) pairs from PROGMEM, handles pen up/down marks.
// ============================================================
void executeDrawing() {
  Serial.println(F("Drawing..."));
  penUp();
  bool firstPt = true;

  for (uint16_t i = 0; i < DRAWING_DATA_LEN - 1; i += 2) {
    float x = pgm_read_float(&drawing_data[i]);
    float y = pgm_read_float(&drawing_data[i + 1]);

    if (x < PATH_END_MARK + 1.0f) break;
    if (x < PEN_UP_MARK + 1.0f) {
      penUp();
      firstPt = true;
      continue;
    }

    if (firstPt) {
      moveTo(x, y);
      penDown();
      firstPt = false;
    } else {
      moveTo(x, y);
    }
  }

  penUp();
  moveTo(0, 0);
  stopAll();
  Serial.println(F("Done!"));
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("Omni Draw Robot"));

  // Precompute wheel trig
  const float angles[] = { 90*DEG_TO_RAD, 210*DEG_TO_RAD, 330*DEG_TO_RAD };
  for (uint8_t i = 0; i < 3; i++) {
    sinW[i] = sinf(angles[i]);
    cosW[i] = cosf(angles[i]);
  }

  // Motor shield
  if (!AFMS.begin()) { Serial.println(F("No shield!")); while (1); }
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

  // Encoder interrupts (CHANGE mode for full 14 CPR)
  attachInterrupt(digitalPinToInterrupt(ENC1_A), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), isr2, CHANGE);
  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);

  // Auto-calibrate encoder directions
  calibrateEncoders();

  // Button
  pinMode(BTN_PIN, INPUT_PULLUP);

  lastLoop = millis();
  Serial.println(F("Ready — press button"));
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  if (digitalRead(BTN_PIN) == LOW) {
    delay(50);
    if (digitalRead(BTN_PIN) == LOW) {
      while (digitalRead(BTN_PIN) == LOW) delay(10);
      homeReset();
      executeDrawing();
    }
  }
}
