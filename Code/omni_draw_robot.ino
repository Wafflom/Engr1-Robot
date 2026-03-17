/*
 * Omni-Wheel Drawing Robot
 * ========================
 * 3 omni-wheels at 120° apart, encoder feedback + PID,
 * servo pen lift. Press button to draw.
 *
 * Hardware:
 *   - Arduino Uno R3
 *   - Adafruit Motor Shield V2
 *   - 3x N20 DC motors with magnetic encoders (6V, 1:50)
 *   - M1 = front (90°), M2 = rear-left (210°), M3 = rear-right (330°)
 *   - SG90 servo on Shield Servo 1 (pin 10)
 *   - Pushbutton on pin 12 to GND
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "hello_drawing.h"

// ---- CONFIG ----
#define ENCODER_CPR     600.0f
#define WHEEL_DIA_MM    36.0f
#define MM_PER_TICK     ((PI * WHEEL_DIA_MM) / ENCODER_CPR)

#define SERVO_PIN       10
#define PEN_UP_ANGLE    45
#define PEN_DOWN_ANGLE  0
#define PEN_SETTLE_MS   300

#define KP  3.0f
#define KI  0.5f
#define KD  0.4f

#define LOOP_MS         10
#define POS_TOL_MM      0.8f
#define MAX_PWM         240
#define DEADBAND        130
#define INTEGRAL_CAP    300.0f
#define MOVE_TIMEOUT_MS 10000UL

// Encoder pins
#define ENC1_A  2
#define ENC1_B  4
#define ENC2_A  3
#define ENC2_B  5
#define ENC3_A  8
#define ENC3_B  7

#define BTN_PIN 12

// ---- GLOBALS ----
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *mot[3];
Servo penServo;

volatile long encTicks[3] = {0, 0, 0};
long prevTicks[3] = {0, 0, 0};
float sinW[3], cosW[3];
float posX = 0, posY = 0;
float tgtX = 0, tgtY = 0;
bool hasTarget = false;
float intgX = 0, intgY = 0;
float peX = 0, peY = 0;
unsigned long lastLoop = 0;
bool drawing = false;

// ---- ENCODER ISRs ----
void isr1() { encTicks[0] += digitalRead(ENC1_B) ? -1 : 1; }
void isr2() { encTicks[1] += digitalRead(ENC2_B) ? -1 : 1; }

volatile uint8_t enc3_prevA = 0;
ISR(PCINT0_vect) {
  uint8_t a = digitalRead(ENC3_A);
  if (a && !enc3_prevA)
    encTicks[2] += digitalRead(ENC3_B) ? -1 : 1;
  enc3_prevA = a;
}

// ---- MOTOR CONTROL ----
void driveMotor(uint8_t idx, float pwm) {
  if (fabsf(pwm) < DEADBAND) {
    mot[idx]->setSpeed(0);
    mot[idx]->run(RELEASE);
    return;
  }
  mot[idx]->setSpeed((uint8_t)constrain(fabsf(pwm), 0, MAX_PWM));
  mot[idx]->run(pwm > 0 ? FORWARD : BACKWARD);
}

void stopAll() {
  for (uint8_t i = 0; i < 3; i++) {
    mot[i]->setSpeed(0);
    mot[i]->run(RELEASE);
  }
}

// ---- KINEMATICS ----
void inverseK(float vx, float vy, float *w) {
  for (uint8_t i = 0; i < 3; i++)
    w[i] = -sinW[i] * vx + cosW[i] * vy;
}

void updateOdometry() {
  noInterrupts();
  long snap[3] = { encTicks[0], encTicks[1], encTicks[2] };
  interrupts();

  float d[3];
  for (uint8_t i = 0; i < 3; i++) {
    d[i] = (float)(snap[i] - prevTicks[i]) * MM_PER_TICK;
    prevTicks[i] = snap[i];
  }

  posX += (2.0f/3.0f) * (-sinW[0]*d[0] - sinW[1]*d[1] - sinW[2]*d[2]);
  posY += (2.0f/3.0f) * ( cosW[0]*d[0] + cosW[1]*d[1] + cosW[2]*d[2]);
}

// ---- PID CONTROLLER ----
bool pidStep(float dt) {
  if (!hasTarget) return true;

  float ex = tgtX - posX;
  float ey = tgtY - posY;
  float dist = sqrtf(ex*ex + ey*ey);

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

  float w[3];
  inverseK(cx, cy, w);

  float pk = max(max(fabsf(w[0]), fabsf(w[1])), fabsf(w[2]));
  if (pk > MAX_PWM) {
    float s = (float)MAX_PWM / pk;
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

// ---- MOVEMENT ----
void moveTo(float x, float y) {
  tgtX = x;
  tgtY = y;
  hasTarget = true;
  intgX = intgY = peX = peY = 0;

  unsigned long deadline = millis() + MOVE_TIMEOUT_MS;

  while (hasTarget) {
    if (millis() >= deadline) {
      Serial.println(F("timeout"));
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
  }
}

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

// ---- DRAWING ENGINE ----
void executeDrawing() {
  Serial.println(F("Drawing..."));
  drawing = true;
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
  moveTo(0.0f, 0.0f);
  drawing = false;
  Serial.println(F("Done"));
}

// ---- SETUP ----
void setup() {
  Serial.begin(115200);
  Serial.println(F("Omni Draw Robot"));

  const float angles[] = {
    90.0f  * PI / 180.0f,
    210.0f * PI / 180.0f,
    330.0f * PI / 180.0f
  };
  for (uint8_t i = 0; i < 3; i++) {
    sinW[i] = sinf(angles[i]);
    cosW[i] = cosf(angles[i]);
  }

  if (!AFMS.begin()) {
    Serial.println(F("No shield!"));
    while (1);
  }

  mot[0] = AFMS.getMotor(1);
  mot[1] = AFMS.getMotor(2);
  mot[2] = AFMS.getMotor(3);
  stopAll();

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
  attachInterrupt(digitalPinToInterrupt(ENC1_A), isr1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), isr2, RISING);

  // Pin-change interrupt for encoder 3 (pin 8)
  enc3_prevA = digitalRead(ENC3_A);
  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);

  // Button
  pinMode(BTN_PIN, INPUT_PULLUP);

  lastLoop = millis();
  Serial.println(F("Ready — press button"));
}

// ---- LOOP ----
void loop() {
  // Button check with debounce
  if (digitalRead(BTN_PIN) == LOW && !drawing) {
    delay(50);
    if (digitalRead(BTN_PIN) == LOW) {
      while (digitalRead(BTN_PIN) == LOW) delay(10);
      homeReset();
      executeDrawing();
    }
  }
}
