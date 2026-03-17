/*
 * Omni-Wheel Drawing Robot — Star Edition
 *
 * Three-wheel holonomic robot with encoder feedback, PID position
 * control, and a servo pen-lift mechanism. Press the button to
 * draw a five-pointed star.
 *
 * Hardware:
 *   Arduino Uno R3
 *   Adafruit Motor Shield V2 (I2C)
 *   3x N20 DC motors w/ magnetic hall-effect encoders (6V, 1:50)
 *     M1 = front wheel       (90°)
 *     M2 = rear-left wheel   (210°)
 *     M3 = rear-right wheel  (330°)
 *   SG90 servo on shield Servo 1 header (pin 10) — pen lift
 *   Pushbutton on pin 12 to GND
 *
 * Encoder specs:
 *   14 counts per motor revolution
 *   1:50 gear ratio
 *   700 counts per output shaft revolution
 *
 * Chassis:
 *   Wheel diameter: 36 mm
 *   Wheelbase radius (center to wheel): 88 mm
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "star_drawing.h"

/* ============================================================
 *  ROBOT PARAMETERS
 * ============================================================ */

// Encoder: 14 ticks/motor-rev × 50:1 gear = 700 ticks/wheel-rev
#define ENCODER_CPR   700.0f
#define WHEEL_DIA_MM  36.0f
#define WHEELBASE_MM  88.0f
#define MM_PER_TICK   ((PI * WHEEL_DIA_MM) / ENCODER_CPR)  // ~0.1616 mm

// Servo pen lift
#define SERVO_PIN      10
#define PEN_UP_ANGLE   45
#define PEN_DOWN_ANGLE 0
#define PEN_SETTLE_MS  300

// PID gains
#define KP  3.0f
#define KI  0.5f
#define KD  0.4f
#define INTEGRAL_CAP  300.0f

// Control loop
#define LOOP_MS         10       // 100 Hz
#define POS_TOL_MM      0.8f    // arrival threshold
#define MAX_PWM         240
#define DEADBAND        130
#define MOVE_TIMEOUT_MS 10000UL  // 10 s per segment

// Encoder pins (hall-effect A/B pairs)
#define ENC1_A 2   // hardware interrupt INT0
#define ENC1_B 4
#define ENC2_A 3   // hardware interrupt INT1
#define ENC2_B 5
#define ENC3_A 8   // pin-change interrupt PCINT0
#define ENC3_B 7

// Button
#define BTN_PIN 12

/* ============================================================
 *  GLOBALS
 * ============================================================ */

Adafruit_MotorShield shield = Adafruit_MotorShield();
Adafruit_DCMotor *motor[3];
Servo pen;

// Encoder tick counters (written in ISRs, read in main loop)
volatile long ticks[3] = {0, 0, 0};
long ticksPrev[3]      = {0, 0, 0};

// Pre-computed wheel angle trig (set once in setup)
float sinA[3];
float cosA[3];

// Odometry position (mm)
float posX = 0.0f;
float posY = 0.0f;

// PID state
float targetX   = 0.0f;
float targetY   = 0.0f;
bool  targeting  = false;
float integralX  = 0.0f;
float integralY  = 0.0f;
float prevErrX   = 0.0f;
float prevErrY   = 0.0f;

unsigned long loopTime = 0;
bool busy = false;

/* ============================================================
 *  ENCODER INTERRUPTS
 * ============================================================
 *
 * Each encoder outputs two hall-effect signals (A and B) that
 * are 90° out of phase. On the rising edge of A, reading B
 * tells us the direction: B low = forward, B high = reverse.
 *
 * Encoders 1 & 2 use the Uno's two hardware interrupt pins
 * (D2 = INT0, D3 = INT1). Encoder 3 uses pin-change interrupt
 * on D8 (PCINT0) since there are no more hardware interrupts.
 */

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

/* ============================================================
 *  MOTOR HELPERS
 * ============================================================ */

void setMotor(uint8_t idx, float pwm) {
  if (fabsf(pwm) < DEADBAND) {
    motor[idx]->setSpeed(0);
    motor[idx]->run(RELEASE);
    return;
  }
  uint8_t speed = (uint8_t)constrain(fabsf(pwm), 0, MAX_PWM);
  motor[idx]->setSpeed(speed);
  motor[idx]->run(pwm > 0 ? FORWARD : BACKWARD);
}

void stopMotors() {
  for (uint8_t i = 0; i < 3; i++) {
    motor[i]->setSpeed(0);
    motor[i]->run(RELEASE);
  }
}

/* ============================================================
 *  KINEMATICS
 * ============================================================
 *
 * Three omni-wheels at 120° spacing allow the robot to translate
 * in any direction without rotating. Inverse kinematics converts
 * a desired (vx, vy) into individual wheel speeds:
 *
 *   wheel[i] = -sin(angle[i]) * vx + cos(angle[i]) * vy
 */

void inverseKinematics(float vx, float vy, float *w) {
  for (uint8_t i = 0; i < 3; i++) {
    w[i] = -sinA[i] * vx + cosA[i] * vy;
  }
}

/* ============================================================
 *  ODOMETRY
 * ============================================================
 *
 * Read encoder deltas and compute robot displacement using
 * forward kinematics (the inverse of the wheel equations).
 */

void updateOdometry() {
  // Atomic snapshot of tick counters
  noInterrupts();
  long snap[3] = {ticks[0], ticks[1], ticks[2]};
  interrupts();

  float d[3];
  for (uint8_t i = 0; i < 3; i++) {
    d[i] = (float)(snap[i] - ticksPrev[i]) * MM_PER_TICK;
    ticksPrev[i] = snap[i];
  }

  // Forward kinematics: average wheel contributions
  posX += (2.0f / 3.0f) * (-sinA[0] * d[0] - sinA[1] * d[1] - sinA[2] * d[2]);
  posY += (2.0f / 3.0f) * ( cosA[0] * d[0] + cosA[1] * d[1] + cosA[2] * d[2]);
}

/* ============================================================
 *  PID POSITION CONTROLLER
 * ============================================================
 *
 * Independent PID on X and Y axes. Returns true when the robot
 * has arrived within POS_TOL_MM of the target.
 */

bool runPID(float dt) {
  if (!targeting) return true;

  float errX = targetX - posX;
  float errY = targetY - posY;
  float dist = sqrtf(errX * errX + errY * errY);

  // Arrived?
  if (dist < POS_TOL_MM) {
    stopMotors();
    targeting = false;
    integralX = integralY = 0.0f;
    prevErrX  = prevErrY  = 0.0f;
    return true;
  }

  // Integrate with anti-windup clamp
  integralX = constrain(integralX + errX * dt, -INTEGRAL_CAP, INTEGRAL_CAP);
  integralY = constrain(integralY + errY * dt, -INTEGRAL_CAP, INTEGRAL_CAP);

  // Derivative
  float dErrX = (errX - prevErrX) / dt;
  float dErrY = (errY - prevErrY) / dt;
  prevErrX = errX;
  prevErrY = errY;

  // PID output
  float cmdX = KP * errX + KI * integralX + KD * dErrX;
  float cmdY = KP * errY + KI * integralY + KD * dErrY;

  // Convert to wheel speeds
  float w[3];
  inverseKinematics(cmdX, cmdY, w);

  // Saturate while preserving direction
  float peak = max(max(fabsf(w[0]), fabsf(w[1])), fabsf(w[2]));
  if (peak > MAX_PWM) {
    float scale = (float)MAX_PWM / peak;
    w[0] *= scale;
    w[1] *= scale;
    w[2] *= scale;
  }

  setMotor(0, w[0]);
  setMotor(1, w[1]);
  setMotor(2, w[2]);

  return false;
}

/* ============================================================
 *  PEN CONTROL
 * ============================================================ */

void penUp() {
  pen.write(PEN_UP_ANGLE);
  delay(PEN_SETTLE_MS);
}

void penDown() {
  pen.write(PEN_DOWN_ANGLE);
  delay(PEN_SETTLE_MS);
}

/* ============================================================
 *  MOVEMENT
 * ============================================================ */

void moveTo(float x, float y) {
  targetX  = x;
  targetY  = y;
  targeting = true;
  integralX = integralY = 0.0f;
  prevErrX  = prevErrY  = 0.0f;

  unsigned long deadline = millis() + MOVE_TIMEOUT_MS;

  while (targeting) {
    if (millis() >= deadline) {
      Serial.println(F("move timeout"));
      stopMotors();
      targeting = false;
      return;
    }
    unsigned long now = millis();
    if (now - loopTime >= LOOP_MS) {
      float dt = (float)(now - loopTime) / 1000.0f;
      loopTime = now;
      updateOdometry();
      runPID(dt);
    }
  }
}

void resetPosition() {
  stopMotors();
  noInterrupts();
  ticks[0] = ticks[1] = ticks[2] = 0;
  interrupts();
  ticksPrev[0] = ticksPrev[1] = ticksPrev[2] = 0;
  posX = posY = 0.0f;
  targeting = false;
  integralX = integralY = 0.0f;
  prevErrX  = prevErrY  = 0.0f;
}

/* ============================================================
 *  DRAWING ENGINE
 * ============================================================
 *
 * Reads coordinate pairs from PROGMEM. Special sentinel values
 * control the pen:
 *   PEN_UP_MARK  (-8888) — lift pen before next move
 *   PATH_END_MARK (-9999) — drawing is finished
 */

void drawShape() {
  Serial.println(F("Drawing star..."));
  busy = true;
  penUp();

  bool penIsUp = true;

  for (uint16_t i = 0; i < DRAWING_DATA_LEN - 1; i += 2) {
    float x = pgm_read_float(&drawing_data[i]);
    float y = pgm_read_float(&drawing_data[i + 1]);

    // End of drawing?
    if (x < PATH_END_MARK + 1.0f) break;

    // Pen lift command?
    if (x < PEN_UP_MARK + 1.0f) {
      penUp();
      penIsUp = true;
      continue;
    }

    moveTo(x, y);

    // Put pen down after arriving at first point of a new stroke
    if (penIsUp) {
      penDown();
      penIsUp = false;
    }
  }

  // Finish: lift pen and return home
  penUp();
  moveTo(0.0f, 0.0f);
  stopMotors();
  busy = false;
  Serial.println(F("Star complete"));
}

/* ============================================================
 *  SETUP
 * ============================================================ */

void setup() {
  Serial.begin(115200);
  Serial.println(F("Star Drawing Robot"));

  // Pre-compute wheel angle trig values
  // Wheels at 90°, 210°, 330° (120° apart)
  const float angles[3] = {
    90.0f  * PI / 180.0f,
    210.0f * PI / 180.0f,
    330.0f * PI / 180.0f
  };
  for (uint8_t i = 0; i < 3; i++) {
    sinA[i] = sinf(angles[i]);
    cosA[i] = cosf(angles[i]);
  }

  // Motor shield
  if (!shield.begin()) {
    Serial.println(F("Motor shield not found!"));
    while (1);
  }
  motor[0] = shield.getMotor(1);
  motor[1] = shield.getMotor(2);
  motor[2] = shield.getMotor(3);
  stopMotors();

  // Servo
  pen.attach(SERVO_PIN);
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

  // Button (pulled high, active low)
  pinMode(BTN_PIN, INPUT_PULLUP);

  loopTime = millis();
  Serial.println(F("Ready — press button to draw star"));
}

/* ============================================================
 *  MAIN LOOP
 * ============================================================ */

void loop() {
  if (digitalRead(BTN_PIN) == LOW && !busy) {
    delay(50);  // debounce
    if (digitalRead(BTN_PIN) == LOW) {
      // Wait for button release
      while (digitalRead(BTN_PIN) == LOW) delay(10);
      resetPosition();
      drawShape();
    }
  }
}
