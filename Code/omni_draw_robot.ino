/*
 * Omni-Wheel Drawing Robot — Arduino Uno R3
 * ==========================================
 *
 * WHAT THIS DOES:
 *   This robot has 3 wheels arranged in a triangle, each angled
 *   120 degrees apart. By spinning them at different speeds and
 *   directions, the robot can slide in ANY direction without
 *   turning — this is called "holonomic" or "omni" drive.
 *
 *   A pen is mounted in the center on a servo. The servo lifts
 *   the pen up (to travel without drawing) or pushes it down
 *   (to draw on paper). The robot reads a list of coordinates
 *   from memory and drives between them, creating a drawing.
 *
 *   Each motor has a magnetic encoder (two hall-effect sensors)
 *   that counts how much the wheel has turned. The code uses
 *   these counts to estimate where the robot is (odometry) and
 *   a PID controller to steer it accurately to each target point.
 *
 * HARDWARE:
 *   - Arduino Uno R3
 *   - Adafruit Motor Shield V2 (communicates over I2C: pins A4/A5)
 *   - 3x Adafruit N20 DC Motor with Magnetic Encoder (6V, 1:50 gear ratio)
 *   - SG90 Micro Servo on Motor Shield "Servo 1" header (pin 10)
 *   - Momentary pushbutton wired from pin 12 to GND
 *   - Red LED with 220-ohm resistor on pin 11
 *   - Green LED with 220-ohm resistor on pin 13
 *
 * WHEEL LAYOUT (looking down at the robot from above):
 *
 *              Wheel 1 / Motor M1
 *                   (front)
 *                  90 degrees
 *                 /          \
 *                /            \
 *   Wheel 2 / M2              Wheel 3 / M3
 *   (rear-left)                (rear-right)
 *   210 degrees                330 degrees
 *
 *   The "angle" of each wheel is measured counter-clockwise
 *   from the +X axis (pointing right). So 90 degrees = pointing up.
 *
 * ENCODER WIRING (each motor has 6 wires):
 *   Wire 1: Motor +    --> Motor Shield terminal (M1, M2, or M3)
 *   Wire 2: Motor -    --> Motor Shield terminal (M1, M2, or M3)
 *   Wire 3: Encoder VCC --> Arduino 5V
 *   Wire 4: Encoder GND --> Arduino GND
 *   Wire 5: Hall sensor A (signal) --> Interrupt-capable pin (see below)
 *   Wire 6: Hall sensor B (signal) --> Any digital pin (see below)
 *
 * PIN MAP (what's connected where on the Uno):
 *   D0  — Serial RX (reserved for USB communication)
 *   D1  — Serial TX (reserved for USB communication)
 *   D2  — Encoder 1, Hall A  (uses hardware interrupt INT0)
 *   D3  — Encoder 2, Hall A  (uses hardware interrupt INT1)
 *   D4  — Encoder 1, Hall B  (plain digital read)
 *   D5  — Encoder 2, Hall B  (plain digital read)
 *   D7  — Encoder 3, Hall B  (plain digital read)
 *   D8  — Encoder 3, Hall A  (uses pin-change interrupt PCINT0)
 *   D10 — Servo signal (Motor Shield "Servo 1" header)
 *   D11 — Red LED (turns on while drawing is in progress)
 *   D12 — Start button (pressed = connects pin to GND)
 *   D13 — Green LED (turns on when drawing is finished)
 *   A4  — SDA (I2C data line, used by Motor Shield — do not touch)
 *   A5  — SCL (I2C clock line, used by Motor Shield — do not touch)
 *
 * HOW THE ENCODERS WORK:
 *   Inside each motor are two tiny magnets on the motor shaft and
 *   two hall-effect sensors (A and B) positioned so their signals
 *   are 90 degrees out of phase. As the shaft spins, each sensor
 *   outputs a pulse — but because they're offset, we can tell
 *   WHICH DIRECTION the shaft is spinning by checking sensor B
 *   at the moment sensor A fires.
 *
 *   We set up an interrupt on sensor A's RISING edge. When it fires:
 *     - If sensor B reads LOW  --> shaft is spinning forward  --> count +1
 *     - If sensor B reads HIGH --> shaft is spinning backward --> count -1
 *
 *   The encoder produces 12 pulses per motor-shaft revolution.
 *   The motor has a 50:1 gearbox, so the output shaft (the one
 *   the wheel is on) turns 50x slower. That means:
 *     12 pulses × 50 = 600 encoder ticks per wheel revolution.
 *
 *   This matches Adafruit's own sample code for this motor.
 *
 * HOW TO USE:
 *   1. Upload this code (with hello_drawing.h in the same folder)
 *   2. Open Serial Monitor at 115200 baud
 *   3. Press the button on pin 12, OR type 'D' and hit Enter
 *   4. Red LED turns on while drawing
 *   5. Green LED turns on when finished
 *   6. To draw something different, swap the #include file below
 */

// ============================================================
//  LIBRARIES
// ============================================================

#include <Wire.h>                  // I2C communication (needed by Motor Shield)
#include <Adafruit_MotorShield.h>  // controls the motors via I2C
#include <Servo.h>                 // controls the pen-lift servo

// This header file contains the drawing coordinates stored in
// flash memory (PROGMEM). Swap this line to draw something else.
#include "hello_drawing.h"

// ============================================================
//  CONFIGURATION — numbers you may need to adjust for your robot
// ============================================================

// How many encoder ticks equal one full wheel revolution.
// 12 pulses per motor rev × 50:1 gear ratio = 600
#define ENCODER_CPR       600.0f

// Outer diameter of your omni wheels in millimeters.
// Measure yours with calipers — this directly affects accuracy!
#define WHEEL_DIA_MM      36.0f

// Distance from the center of the robot to where each wheel
// touches the ground, in millimeters.
#define ROBOT_RADIUS_MM   60.0f

// How many millimeters the robot moves per single encoder tick.
// This comes from: wheel circumference / ticks per revolution.
// Circumference = pi × diameter, so:
//   MM_PER_TICK = (pi × 36) / 600 ≈ 0.1885 mm per tick
#define MM_PER_TICK       ((PI * WHEEL_DIA_MM) / ENCODER_CPR)

// --- Servo settings ---
#define SERVO_PIN         10   // Motor Shield "Servo 1" is wired to pin 10
#define PEN_UP_ANGLE      90   // servo angle when pen is lifted (degrees)
#define PEN_DOWN_ANGLE    45   // servo angle when pen touches paper (degrees)
#define PEN_SETTLE_MS     250  // milliseconds to wait for servo to finish moving

// --- PID controller gains ---
// These control how aggressively the robot corrects its position.
// KP = proportional: bigger error → stronger correction
// KI = integral: accumulated error over time → fixes persistent drift
// KD = derivative: rate of error change → dampens oscillation
// Start with KP=3 and KI=KD=0, then increase KD, then add small KI.
#define KP  3.0f
#define KI  0.5f
#define KD  0.4f

// --- Motion settings ---
#define LOOP_MS           10       // run the control loop every 10 ms (100 Hz)
#define POS_TOL_MM        0.8f     // if we're within 0.8mm of the target, consider it "arrived"
#define MAX_PWM           240      // maximum motor power (0-255). lower = slower but more precise
#define DEADBAND          130      // below this PWM the motors buzz but don't actually turn (~50%)
#define INTEGRAL_CAP      300.0f   // limit on the integral term to prevent "windup"
#define MOVE_TIMEOUT_MS   10000UL  // give up on a move after 10 seconds (safety)

// --- Pin assignments ---
// Encoder 1 (Motor M1 — front wheel)
#define ENC1_A    2    // hall sensor A → pin 2 (has hardware interrupt INT0)
#define ENC1_B    4    // hall sensor B → pin 4 (read digitally for direction)

// Encoder 2 (Motor M2 — rear-left wheel)
#define ENC2_A    3    // hall sensor A → pin 3 (has hardware interrupt INT1)
#define ENC2_B    5    // hall sensor B → pin 5

// Encoder 3 (Motor M3 — rear-right wheel)
#define ENC3_A    8    // hall sensor A → pin 8 (uses pin-change interrupt PCINT0)
#define ENC3_B    7    // hall sensor B → pin 7

#define BTN_PIN   12   // start button (other leg of button goes to GND)
#define RED_LED   11   // red LED anode (through 220-ohm resistor to GND)
#define GREEN_LED 13   // green LED anode (through 220-ohm resistor to GND)

// ============================================================
//  GLOBAL VARIABLES
// ============================================================

// Create the motor shield object (talks to the shield over I2C)
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Get pointers to each of the 3 DC motor ports on the shield.
// We store them in an array so we can loop over them easily.
// mot[0] = M1 (front), mot[1] = M2 (rear-left), mot[2] = M3 (rear-right)
Adafruit_DCMotor *mot[3];

// Create the servo object for the pen lifter
Servo penServo;
bool penIsDown = false;  // tracks whether pen is currently touching paper

// Encoder tick counters.
// "volatile" tells the compiler these can change at ANY time
// (inside an interrupt), so it must always re-read them from
// memory instead of caching them in a register.
volatile long encTicks[3] = {0, 0, 0};

// Stores the encoder values from the PREVIOUS loop iteration,
// so we can calculate how much each wheel moved since last time.
long prevTicks[3] = {0, 0, 0};

// Precomputed sine and cosine of each wheel's mounting angle.
// We calculate these once in setup() and reuse them constantly
// in the kinematics math, which saves a lot of CPU time vs
// calling sin()/cos() every loop iteration.
float sinW[3], cosW[3];

// The robot's estimated position in millimeters, calculated
// from encoder readings (this is called "odometry").
// posX increases to the right, posY increases forward.
float posX = 0.0f, posY = 0.0f;

// The target position the robot is currently trying to reach.
float tgtX = 0.0f, tgtY = 0.0f;
bool  hasTarget = false;  // true when the robot is actively moving somewhere

// PID controller internal state.
// intgX/Y = accumulated error over time (integral term)
// peX/peY = previous error (used to calculate derivative term)
float intgX = 0, intgY = 0;
float peX = 0, peY = 0;

// Timestamp of the last control loop execution (milliseconds)
unsigned long lastLoop = 0;

// Buffer for reading serial commands character by character
char sBuf[64];       // holds characters as they arrive
uint8_t sIdx = 0;    // index of next free spot in the buffer

// True while a drawing is in progress (prevents re-triggering)
bool drawing = false;

// ============================================================
//  ENCODER INTERRUPT SERVICE ROUTINES (ISRs)
//
//  These functions are called automatically by the hardware
//  whenever an encoder's Hall sensor A has a RISING edge
//  (signal goes from LOW to HIGH). They run instantly,
//  interrupting whatever the main code was doing.
//
//  Inside the ISR, we read Hall sensor B to determine direction:
//    - B is LOW  → the motor is spinning forward  → add 1
//    - B is HIGH → the motor is spinning backward  → subtract 1
//
//  ISRs must be FAST — no Serial.print, no delay, no math.
//  Just read a pin and increment a counter.
// ============================================================

// Called automatically when Encoder 1, Hall A rises (pin 2)
void isr1() {
  // digitalRead(ENC1_B) returns 0 (LOW) or 1 (HIGH).
  // If B is HIGH (truthy), motor is going backward → subtract 1.
  // If B is LOW (falsy), motor is going forward → add 1.
  encTicks[0] += digitalRead(ENC1_B) ? -1 : 1;
}

// Called automatically when Encoder 2, Hall A rises (pin 3)
void isr2() {
  encTicks[1] += digitalRead(ENC2_B) ? -1 : 1;
}

// Encoder 3 is special because pin 8 doesn't have a dedicated
// hardware interrupt like pins 2 and 3 do. Instead, we use a
// "Pin Change Interrupt" (PCINT). The problem is that PCINT
// fires on BOTH rising AND falling edges — we can't choose.
// So we manually track the previous state of Hall A and only
// count when we see a LOW-to-HIGH transition (rising edge).

// Stores the last known state of Encoder 3's Hall A pin.
// "volatile" because it's read and written inside the ISR.
volatile uint8_t enc3_prevA = 0;

// This ISR is called for ANY change on pins 8-13 (Port B).
// We only enabled the interrupt for pin 8 (see setup), so in
// practice it only fires for pin 8 changes.
ISR(PCINT0_vect) {
  uint8_t a = digitalRead(ENC3_A);  // read current state of Hall A

  // Check if this is a rising edge: A is now HIGH and was previously LOW
  if (a && !enc3_prevA) {
    // It's a rising edge — count this tick
    encTicks[2] += digitalRead(ENC3_B) ? -1 : 1;
  }

  enc3_prevA = a;  // remember current state for next time
}

// ============================================================
//  MOTOR CONTROL HELPERS
// ============================================================

// Drive a single motor at a given PWM level.
// Positive pwm = FORWARD, negative pwm = BACKWARD.
// If the PWM is too low (below the deadband), release the motor
// entirely so it doesn't buzz/whine without actually turning.
void driveMotor(uint8_t idx, float pwm) {
  if (fabsf(pwm) < DEADBAND) {
    // Motor would stall at this low power — just stop it
    mot[idx]->setSpeed(0);
    mot[idx]->run(RELEASE);   // RELEASE = coast (no braking)
    return;
  }

  // Clamp the absolute value between 0 and MAX_PWM
  mot[idx]->setSpeed((uint8_t)constrain(fabsf(pwm), 0, MAX_PWM));

  // Set direction based on sign of pwm
  mot[idx]->run(pwm > 0 ? FORWARD : BACKWARD);
}

// Stop all 3 motors immediately
void stopAll() {
  for (uint8_t i = 0; i < 3; i++) {
    mot[i]->setSpeed(0);
    mot[i]->run(RELEASE);
  }
}

// ============================================================
//  KINEMATICS — the math that connects wheel speeds to robot motion
//
//  INVERSE KINEMATICS: "I want the robot to move at velocity
//  (vx, vy) — what speed should each wheel spin at?"
//
//  For each wheel at angle θ:
//    wheel_speed = -sin(θ) × vx + cos(θ) × vy
//
//  This works because each omni wheel can only push
//  perpendicular to its axle. The sin/cos terms project the
//  desired velocity onto each wheel's "push" direction.
//
//  FORWARD KINEMATICS (odometry): the reverse — "given how
//  much each wheel turned, where did the robot move?"
//  We use the pseudo-inverse of the same matrix.
// ============================================================

// Inverse kinematics: desired robot velocity → 3 wheel speeds.
// vx = desired speed in X (rightward), vy = desired speed in Y (forward).
// Results are written into the w[] array: w[0], w[1], w[2].
void inverseK(float vx, float vy, float *w) {
  for (uint8_t i = 0; i < 3; i++)
    w[i] = -sinW[i] * vx + cosW[i] * vy;
}

// Read the encoders, calculate how far each wheel moved since
// the last call, and update the robot's estimated position.
void updateOdometry() {
  // Temporarily disable interrupts while we copy the encoder values.
  // This prevents an ISR from changing encTicks mid-read, which
  // could give us a corrupted (half-updated) value.
  noInterrupts();
  long snap[3] = { encTicks[0], encTicks[1], encTicks[2] };
  interrupts();  // re-enable interrupts immediately

  // Calculate how far each wheel moved (in mm) since last time
  float d[3];
  for (uint8_t i = 0; i < 3; i++) {
    d[i] = (float)(snap[i] - prevTicks[i]) * MM_PER_TICK;
    prevTicks[i] = snap[i];  // save for next iteration
  }

  // Forward kinematics: convert 3 wheel displacements into
  // robot X/Y displacement. The (2/3) factor comes from the
  // pseudo-inverse of the 3-wheel kinematics matrix.
  // This gives us how far the robot's CENTER moved.
  posX += (2.0f/3.0f) * (-sinW[0]*d[0] - sinW[1]*d[1] - sinW[2]*d[2]);
  posY += (2.0f/3.0f) * ( cosW[0]*d[0] + cosW[1]*d[1] + cosW[2]*d[2]);
}

// ============================================================
//  PID POSITION CONTROLLER
//
//  PID stands for Proportional-Integral-Derivative. It's the
//  most common control algorithm in robotics.
//
//  Every 10ms, it:
//    1. Calculates the ERROR (how far we are from the target)
//    2. Calculates three correction terms:
//       P = proportional to error (big error → big correction)
//       I = proportional to accumulated error over time
//           (fixes small persistent offsets)
//       D = proportional to how fast the error is changing
//           (prevents overshooting and oscillation)
//    3. Adds them up: correction = KP×P + KI×I + KD×D
//    4. Feeds the correction into the inverse kinematics
//       to compute wheel speeds
//
//  Returns true when the robot has reached the target.
// ============================================================

bool pidStep(float dt) {
  if (!hasTarget) return true;  // nothing to do

  // Calculate error: how far off are we in X and Y?
  float ex = tgtX - posX;  // positive = target is to the right
  float ey = tgtY - posY;  // positive = target is ahead

  // Calculate straight-line distance to target
  float dist = sqrtf(ex*ex + ey*ey);

  // If we're close enough, declare success and stop
  if (dist < POS_TOL_MM) {
    stopAll();
    hasTarget = false;
    intgX = intgY = peX = peY = 0;  // reset PID state for next move
    return true;  // arrived!
  }

  // --- Integral term (I) ---
  // Add current error × time to the running total.
  // Constrain it to prevent "windup" — where the integral
  // grows huge while the robot is stuck, then overcorrects.
  intgX = constrain(intgX + ex * dt, -INTEGRAL_CAP, INTEGRAL_CAP);
  intgY = constrain(intgY + ey * dt, -INTEGRAL_CAP, INTEGRAL_CAP);

  // --- Derivative term (D) ---
  // How fast is the error changing? (error now - error last time) / time
  float dErrX = (ex - peX) / dt;
  float dErrY = (ey - peY) / dt;

  // --- Combine P + I + D into a commanded velocity ---
  float cx = KP * ex + KI * intgX + KD * dErrX;
  float cy = KP * ey + KI * intgY + KD * dErrY;

  // Save current error for next iteration's derivative calculation
  peX = ex;
  peY = ey;

  // Convert the desired velocity (cx, cy) into individual wheel speeds
  float w[3];
  inverseK(cx, cy, w);

  // If any wheel speed exceeds MAX_PWM, scale ALL of them down
  // proportionally. This preserves the direction of motion while
  // keeping every motor within its limits.
  float pk = max(max(fabsf(w[0]), fabsf(w[1])), fabsf(w[2]));
  if (pk > MAX_PWM) {
    float s = (float)MAX_PWM / pk;  // scaling factor
    w[0] *= s;
    w[1] *= s;
    w[2] *= s;
  }

  // Send the computed speeds to the motors
  driveMotor(0, w[0]);
  driveMotor(1, w[1]);
  driveMotor(2, w[2]);

  return false;  // not there yet
}

// ============================================================
//  PEN CONTROL
//  The servo physically lifts or lowers the pen.
//  We track the state to avoid redundant servo writes.
// ============================================================

void penUp() {
  if (penIsDown) {                     // only act if pen is currently down
    penServo.write(PEN_UP_ANGLE);      // command servo to lift position
    delay(PEN_SETTLE_MS);              // wait for servo to physically get there
    penIsDown = false;
  }
}

void penDown() {
  if (!penIsDown) {                    // only act if pen is currently up
    penServo.write(PEN_DOWN_ANGLE);    // command servo to lowered position
    delay(PEN_SETTLE_MS);              // wait for servo to physically get there
    penIsDown = true;
  }
}

// ============================================================
//  BLOCKING MOVE
//  Commands the robot to move to coordinates (x, y) in mm
//  and WAITS here until it arrives (or times out).
//  "Blocking" means the function doesn't return until done.
// ============================================================

void moveTo(float x, float y) {
  // Set the target for the PID controller
  tgtX = x;
  tgtY = y;
  hasTarget = true;

  // Reset PID state so old accumulated error doesn't carry over
  intgX = intgY = peX = peY = 0;

  // Safety: if we haven't arrived after 10 seconds, give up.
  // This prevents the robot from running forever if it's stuck.
  unsigned long deadline = millis() + MOVE_TIMEOUT_MS;

  // Sit in this loop, running the PID controller repeatedly,
  // until either we arrive (hasTarget becomes false) or we time out.
  while (hasTarget) {
    // Check for timeout
    if (millis() >= deadline) {
      Serial.println(F("WARN: timeout"));
      stopAll();
      hasTarget = false;
      return;
    }

    // Run the control loop at the configured rate (every LOOP_MS)
    unsigned long now = millis();
    if (now - lastLoop >= LOOP_MS) {
      // Calculate how much real time has passed (in seconds)
      // for the PID's integral and derivative calculations
      float dt = (float)(now - lastLoop) / 1000.0f;
      lastLoop = now;

      updateOdometry();  // read encoders, update posX/posY
      pidStep(dt);       // compute and apply motor corrections
    }
  }
}

// ============================================================
//  HOME / RESET
//  Stops everything and resets the position to (0, 0).
//  Call this before starting a new drawing so the robot
//  treats its current physical location as the origin.
// ============================================================

void homeReset() {
  stopAll();

  // Zero out encoder counters (must disable interrupts briefly
  // so an ISR doesn't write to them while we're clearing them)
  noInterrupts();
  encTicks[0] = encTicks[1] = encTicks[2] = 0;
  interrupts();

  prevTicks[0] = prevTicks[1] = prevTicks[2] = 0;
  posX = posY = 0;     // robot is now at the origin
  hasTarget = false;
  intgX = intgY = peX = peY = 0;
}

// ============================================================
//  DRAWING ENGINE
//  Reads coordinate data from the PROGMEM array in the header
//  file and executes it as a series of pen-up travels and
//  pen-down line segments.
//
//  The data format is pairs of floats: (x, y), (x, y), ...
//  with special sentinel values mixed in:
//    PEN_UP_MARK (-8888) = lift the pen; next point is a travel
//    PATH_END_MARK (-9999) = drawing is finished
//
//  Between sentinels, each point is connected to the next with
//  a straight pen-down line segment.
// ============================================================

void executeDrawing() {
  Serial.println(F("--- Drawing start ---"));
  drawing = true;                   // flag to prevent re-triggering
  digitalWrite(RED_LED, HIGH);      // red LED on = drawing in progress
  digitalWrite(GREEN_LED, LOW);     // green LED off

  penUp();  // make sure pen starts in the air

  // firstPt tracks whether we're at the beginning of a new stroke.
  // The first point after a PEN_UP_MARK is a "travel-to" destination:
  // we move there with pen up, THEN lower the pen.
  bool firstPt = true;

  // Walk through the drawing data array two floats at a time (x, y pairs).
  // DRAWING_DATA_LEN is the total number of floats in the array.
  for (uint16_t i = 0; i < DRAWING_DATA_LEN - 1; i += 2) {

    // pgm_read_float() reads a float from PROGMEM (flash memory).
    // Regular array access won't work for PROGMEM data on AVR.
    float x = pgm_read_float(&drawing_data[i]);
    float y = pgm_read_float(&drawing_data[i + 1]);

    // Check for end-of-drawing marker (-9999)
    // We use < instead of == because float comparison can be imprecise
    if (x < PATH_END_MARK + 1.0f) break;

    // Check for pen-up marker (-8888)
    if (x < PEN_UP_MARK + 1.0f) {
      penUp();           // lift the pen
      firstPt = true;    // next real point starts a new stroke
      continue;          // skip to next data pair
    }

    if (firstPt) {
      // This is the first point of a new stroke.
      // Travel here with pen up (it's already up), then put pen down.
      Serial.print(F(">> "));        // ">>" means travel (pen up)
      Serial.print(x, 1);
      Serial.print(',');
      Serial.println(y, 1);

      moveTo(x, y);     // drive to this point (pen is up)
      penDown();         // now lower the pen to start drawing
      firstPt = false;

    } else {
      // Continuation of current stroke — draw a line to this point
      Serial.print(F("-- "));        // "--" means draw (pen down)
      Serial.print(x, 1);
      Serial.print(',');
      Serial.println(y, 1);

      moveTo(x, y);     // drive here with pen down (draws a line)
    }
  }

  // Drawing is complete — lift pen and go back to origin
  penUp();
  Serial.println(F("Returning home..."));
  moveTo(0.0f, 0.0f);   // drive back to starting position

  // Update status indicators
  drawing = false;
  digitalWrite(RED_LED, LOW);       // red off
  digitalWrite(GREEN_LED, HIGH);    // green on = finished!
  Serial.println(F("--- Done ---"));
}

// ============================================================
//  SERIAL COMMAND HANDLER
//  Parses text commands from the Serial Monitor for testing.
//  Type a command and press Enter.
// ============================================================

void handleCmd(const char *c) {
  // Skip any leading spaces
  while (*c == ' ') c++;

  // Check the first character (OR with 0x20 makes it lowercase,
  // so both 'D' and 'd' match — a quick case-insensitive trick)
  switch (c[0] | 0x20) {

    case 'g': {
      // G X Y — move to a specific position
      float x, y;
      // sscanf tries to parse two float numbers from the string
      if (sscanf(c + 1, "%f %f", &x, &y) == 2) {
        moveTo(x, y);
        Serial.println(F("OK"));
      } else {
        Serial.println(F("ERR: G X Y"));  // couldn't parse coordinates
      }
      break;
    }

    case 'p':
      // PU = pen up, PD = pen down
      if ((c[1] | 0x20) == 'u') { penUp();   Serial.println(F("OK")); }
      if ((c[1] | 0x20) == 'd') { penDown(); Serial.println(F("OK")); }
      break;

    case 'h':
      // H = home (reset position to 0,0)
      homeReset();
      Serial.println(F("OK"));
      break;

    case 'd':
      // D = run the drawing
      homeReset();
      executeDrawing();
      break;

    case '?':
      // ? = print current estimated position
      Serial.print(F("POS "));
      Serial.print(posX, 2);   // 2 decimal places
      Serial.print(' ');
      Serial.println(posY, 2);
      break;

    default:
      // Unknown command — show help
      Serial.println(F("Commands: D G PU PD H ?"));
  }
}

// ============================================================
//  SETUP — runs once when the Arduino powers on or resets
// ============================================================

void setup() {
  // Start serial communication at 115200 baud for the Serial Monitor
  Serial.begin(115200);
  Serial.println(F("\n=== OMNI DRAW ROBOT v3 (Uno R3) ==="));

  // --- Precompute wheel angle trig ---
  // The three wheels are at 90°, 210°, and 330° (counter-clockwise from +X).
  // We convert to radians and compute sin/cos once, since these never change.
  const float angles[] = {
    90.0f  * PI / 180.0f,   // wheel 1: 90° = straight ahead
    210.0f * PI / 180.0f,   // wheel 2: 210° = rear-left
    330.0f * PI / 180.0f    // wheel 3: 330° = rear-right
  };
  for (uint8_t i = 0; i < 3; i++) {
    sinW[i] = sinf(angles[i]);
    cosW[i] = cosf(angles[i]);
  }

  // --- Initialize the Motor Shield ---
  // begin() starts I2C communication with the shield.
  // If the shield isn't connected or powered, this fails.
  if (!AFMS.begin()) {
    Serial.println(F("FATAL: no shield"));
    while (1);  // halt forever — can't do anything without motors
  }

  // Get pointers to each motor port on the shield
  mot[0] = AFMS.getMotor(1);  // M1 terminal = front wheel
  mot[1] = AFMS.getMotor(2);  // M2 terminal = rear-left wheel
  mot[2] = AFMS.getMotor(3);  // M3 terminal = rear-right wheel
  stopAll();                   // make sure everything starts stopped
  Serial.println(F("Motors OK"));

  // --- Initialize the servo ---
  penServo.attach(SERVO_PIN);      // tell the Servo library which pin to use
  penServo.write(PEN_UP_ANGLE);    // start with pen lifted
  penIsDown = false;
  Serial.println(F("Servo OK (pin 10)"));

  // --- Configure encoder pins ---
  // INPUT_PULLUP enables the Arduino's internal pull-up resistor,
  // which keeps the pin at HIGH when nothing is driving it LOW.
  // The encoder's open-drain outputs pull the pin LOW on each pulse.
  pinMode(ENC1_A, INPUT_PULLUP);  // Motor 1 Hall A
  pinMode(ENC1_B, INPUT_PULLUP);  // Motor 1 Hall B
  pinMode(ENC2_A, INPUT_PULLUP);  // Motor 2 Hall A
  pinMode(ENC2_B, INPUT_PULLUP);  // Motor 2 Hall B
  pinMode(ENC3_A, INPUT_PULLUP);  // Motor 3 Hall A
  pinMode(ENC3_B, INPUT_PULLUP);  // Motor 3 Hall B

  // --- Set up encoder interrupts ---

  // Encoders 1 and 2 use the Uno's two dedicated hardware interrupts.
  // digitalPinToInterrupt() converts pin 2 → INT0 and pin 3 → INT1.
  // RISING means the ISR fires when the signal goes from LOW to HIGH.
  attachInterrupt(digitalPinToInterrupt(ENC1_A), isr1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), isr2, RISING);

  // Encoder 3 uses a Pin Change Interrupt because pin 8 doesn't
  // have a dedicated hardware interrupt.
  //
  // PCICR = Pin Change Interrupt Control Register.
  //   Setting bit PCIE0 enables the PCINT[7:0] group (Port B, pins 8-13).
  //
  // PCMSK0 = Pin Change Mask Register for Port B.
  //   Setting bit PCINT0 unmasks pin 8 specifically.
  //   (Other Port B pins remain masked = won't trigger the ISR.)
  enc3_prevA = digitalRead(ENC3_A);  // initialize the edge tracker
  PCICR  |= (1 << PCIE0);           // enable pin change interrupts for Port B
  PCMSK0 |= (1 << PCINT0);          // listen only on pin 8 (PCINT0)
  Serial.println(F("Encoders OK (12 PPR x 50:1 x 3)"));

  // --- Configure LED pins as outputs ---
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);     // start with both LEDs off
  digitalWrite(GREEN_LED, LOW);

  // --- Configure button pin ---
  // INPUT_PULLUP keeps the pin HIGH normally.
  // When the button is pressed, it connects the pin to GND → reads LOW.
  pinMode(BTN_PIN, INPUT_PULLUP);

  // Record the starting time for the control loop
  lastLoop = millis();

  Serial.println(F("READY — press button or send D to draw"));
}

// ============================================================
//  LOOP — runs over and over after setup() finishes
// ============================================================

void loop() {

  // --- Run the control loop at a fixed rate ---
  // This checks if enough time has passed since the last iteration.
  // If so, it reads the encoders and runs the PID controller.
  // This happens even when we're not actively drawing, so the
  // robot can respond to manual G commands from serial.
  unsigned long now = millis();
  if (now - lastLoop >= LOOP_MS) {
    // Calculate elapsed time in seconds for PID math
    float dt = (float)(now - lastLoop) / 1000.0f;
    lastLoop = now;

    updateOdometry();              // read encoders → update posX, posY
    if (hasTarget) pidStep(dt);    // if we have a destination, correct course
  }

  // --- Check the start button ---
  // Only trigger if it reads LOW (pressed) and we're not already drawing.
  if (digitalRead(BTN_PIN) == LOW && !drawing) {
    delay(50);  // debounce: wait 50ms and check again to filter noise

    if (digitalRead(BTN_PIN) == LOW) {
      // Button is really pressed — wait for the user to release it
      // so we don't accidentally trigger multiple times
      while (digitalRead(BTN_PIN) == LOW) delay(10);

      // Now start the drawing
      homeReset();
      executeDrawing();
    }
  }

  // --- Read serial commands one character at a time ---
  // Characters arrive one by one. We accumulate them in sBuf[]
  // until we see a newline (Enter key), then process the whole line.
  while (Serial.available()) {
    char c = Serial.read();  // grab one character from the serial buffer

    if (c == '\n' || c == '\r') {
      // Newline received — process the command if we have any characters
      if (sIdx > 0) {
        sBuf[sIdx] = '\0';    // null-terminate the string
        handleCmd(sBuf);       // parse and execute the command
        sIdx = 0;              // reset buffer for next command
      }
    } else if (sIdx < sizeof(sBuf) - 1) {
      // Regular character — add it to the buffer
      // (leave room for the null terminator, hence -1)
      sBuf[sIdx++] = c;
    }
  }
}
