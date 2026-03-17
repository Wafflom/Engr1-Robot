# Code Documentation — Omni-Wheel Drawing Robot

This document explains the firmware architecture, every major function, and how to create your own drawings.

---

## Table of Contents

- [File Overview](#file-overview)
- [Configuration Parameters](#configuration-parameters)
- [Architecture](#architecture)
- [Encoder System](#encoder-system)
- [Motor Control](#motor-control)
- [Kinematics](#kinematics)
- [Odometry](#odometry)
- [PID Controller](#pid-controller)
- [Pen Servo](#pen-servo)
- [Drawing Engine](#drawing-engine)
- [Serial Command Interface](#serial-command-interface)
- [Main Loop](#main-loop)
- [Creating Custom Drawings](#creating-custom-drawings)
- [Tuning Guide](#tuning-guide)

---

## File Overview

| File | Purpose |
|------|---------|
| `omni_draw_robot.ino` | Main firmware — all control logic, kinematics, PID, I/O |
| `hello_drawing.h` | Drawing coordinate data stored in PROGMEM (flash memory) |

---

## Configuration Parameters

All tunable values are `#define` constants near the top of `omni_draw_robot.ino`:

### Physical Dimensions

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ENCODER_CPR` | 600.0 | Encoder counts per wheel revolution (12 PPR x 50:1 gear) |
| `WHEEL_DIA_MM` | 48.0 | Outer diameter of the omni-wheels in mm |
| `ROBOT_RADIUS_MM` | 60.0 | Distance from robot center to wheel contact point in mm |
| `MM_PER_TICK` | 0.2513 | Linear distance per encoder tick: (pi x 48) / 600 |

### Servo

| Parameter | Default | Description |
|-----------|---------|-------------|
| `PEN_UP_ANGLE` | 90 | Servo angle when pen is lifted (degrees) |
| `PEN_DOWN_ANGLE` | 45 | Servo angle when pen touches paper (degrees) |
| `PEN_SETTLE_MS` | 250 | Delay after servo move to let it settle (ms) |

### PID Gains

| Parameter | Default | Description |
|-----------|---------|-------------|
| `KP` | 3.0 | Proportional gain — how aggressively it corrects position error |
| `KI` | 0.5 | Integral gain — corrects for accumulated steady-state error |
| `KD` | 0.4 | Derivative gain — dampens oscillation |
| `INTEGRAL_CAP` | 300.0 | Anti-windup limit for the integral term |

### Motion

| Parameter | Default | Description |
|-----------|---------|-------------|
| `LOOP_MS` | 10 | Control loop period in ms (100 Hz) |
| `POS_TOL_MM` | 0.8 | How close (mm) the robot must get to consider it "arrived" |
| `MAX_PWM` | 180 | Maximum motor power (out of 255) |
| `DEADBAND` | 25 | Minimum PWM needed to overcome motor static friction |
| `MOVE_TIMEOUT_MS` | 10000 | Timeout per move segment (ms) |

---

## Architecture

The firmware runs a simple superloop at 100 Hz:

```
setup()
  │
  ▼
loop() ─────────────────────────────────┐
  │                                     │
  ├── Is it time for a control tick?    │
  │     YES:                            │
  │     ├── updateOdometry()            │
  │     └── pidStep(dt)                 │
  │                                     │
  ├── Button pressed?                   │
  │     YES: executeDrawing()           │
  │                                     │
  ├── Serial command available?         │
  │     YES: handleCmd()                │
  │                                     │
  └─────────────────────────────────────┘
```

Drawing is **blocking** — when `executeDrawing()` runs, it takes over the loop until all coordinates are drawn. The PID loop runs inside `moveTo()` during drawing.

---

## Encoder System

### How It Works

Each motor has two Hall-effect sensors (A and B) mounted 90 degrees apart on the motor shaft. This is called **quadrature encoding**. By watching the rising edge of sensor A and checking the state of sensor B, we determine both speed and direction:

```
Hall A:  ─┐  ┌──┐  ┌──┐  ┌──
          └──┘  └──┘  └──┘
Hall B:  ──┐  ┌──┐  ┌──┐  ┌─
           └──┘  └──┘  └──┘

Forward:  When A rises, B is LOW   → count++
Backward: When A rises, B is HIGH  → count--
```

### Interrupt Strategy

The Arduino Uno has only 2 hardware interrupt pins (D2, D3), but we need 3 encoder channels:

| Motor | Hall A Pin | Method |
|-------|-----------|--------|
| Motor 1 | D2 | Hardware interrupt `INT0` (fastest) |
| Motor 2 | D3 | Hardware interrupt `INT1` (fastest) |
| Motor 3 | D8 | Pin-change interrupt `PCINT0` (fires on both edges, filtered in software) |

### ISR Functions

```
isr1()             — Triggered on D2 rising edge, reads D4 for direction
isr2()             — Triggered on D3 rising edge, reads D5 for direction
ISR(PCINT0_vect)   — Triggered on D8 any edge, manually checks for rising, reads D7
```

The encoder counters (`enc1`, `enc2`, `enc3`) are `volatile long` variables, safely read in `updateOdometry()` with interrupts briefly disabled.

---

## Motor Control

### `driveMotor(int idx, int pwm)`

Drives one of the three motors at a given power level.

- `idx`: 0, 1, or 2 (maps to M1, M2, M3 on the shield)
- `pwm`: -255 to +255 (negative = backward, positive = forward)
- Values below `DEADBAND` (25) are zeroed to prevent stalling
- Values are clamped to `MAX_PWM` (180)
- Uses the Adafruit Motor Shield library: `motor->setSpeed()` and `motor->run(FORWARD/BACKWARD/RELEASE)`

### `stopAll()`

Releases all three motors (coasting stop, not braking).

---

## Kinematics

### Inverse Kinematics: `inverseK(float vx, float vy, float w[])`

Converts a desired robot velocity (vx, vy) in mm/s into individual wheel speeds.

Each wheel is mounted at an angle. The wheel can only drive perpendicular to its axle, so the component of the velocity that each wheel "sees" depends on its angle:

```
wheel_speed[i] = -sin(angle[i]) * vx + cos(angle[i]) * vy
```

Pre-computed sin/cos values for the three wheel angles (90, 210, 330 degrees):

| Wheel | Angle | sin | cos |
|-------|-------|-----|-----|
| 1 (front) | 90 | -1.0 | 0.0 |
| 2 (rear-left) | 210 | -0.5 | -0.866 |
| 3 (rear-right) | 330 | 0.5 | -0.866 |

The output `w[]` is then used to set motor PWM values proportionally.

---

## Odometry

### `updateOdometry()`

Reads the encoder counters and estimates how far the robot center has moved since the last update.

**Steps:**
1. Disable interrupts and snapshot encoder counts
2. Calculate deltas: `d[i] = (current_count - previous_count) * MM_PER_TICK`
3. Re-enable interrupts
4. Apply forward kinematics (inverse of the wheel-speed equation):

```
dx = (2/3) * (-sin(90)*d1 + -sin(210)*d2 + -sin(330)*d3)
dy = (2/3) * ( cos(90)*d1 +  cos(210)*d2 +  cos(330)*d3)
```

5. Update global position: `posX += dx`, `posY += dy`

The `(2/3)` factor comes from the pseudo-inverse of the 3x2 kinematics matrix for a 3-wheel holonomic robot.

---

## PID Controller

### `pidStep(float dt)` — runs every 10 ms

A standard PID position controller that calculates the velocity command needed to reach a target position.

**Calculation (for each axis X and Y independently):**

```
error        = target - current_position
integral    += error * dt               (capped at ±INTEGRAL_CAP)
derivative   = (error - previous_error) / dt
command      = KP * error + KI * integral + KD * derivative
```

**Additional logic:**
- **Velocity saturation**: If the total velocity magnitude exceeds `MAX_PWM`, both components are scaled down proportionally (preserving direction)
- **Arrival check**: Returns `true` when `sqrt(ex^2 + ey^2) < POS_TOL_MM` (0.8 mm)
- The command velocities are passed through `inverseK()` to get per-wheel PWM values
- Integral terms are reset when a new target is set (prevents wind-up carryover)

### `moveTo(float x, float y)`

Blocking function that:
1. Sets the target position
2. Resets PID state (integrals, previous errors)
3. Loops at 100 Hz running `updateOdometry()` + `pidStep()` until arrival or timeout (10 s)
4. Stops all motors when done

---

## Pen Servo

### `penUp()` / `penDown()`

- Connected to pin D10 via the Motor Shield's "Servo 1" header
- `penUp()`: Moves to 90 degrees (pen off paper), waits 250 ms
- `penDown()`: Moves to 45 degrees (pen on paper), waits 250 ms
- Tracks current state to avoid redundant servo writes

These angles may need adjustment depending on your pen holder geometry. Modify `PEN_UP_ANGLE` and `PEN_DOWN_ANGLE` in the configuration section.

---

## Drawing Engine

### `executeDrawing()`

The main drawing routine. Reads coordinate pairs from the `drawing_data[]` array stored in PROGMEM (flash memory).

**Flow:**

```
homeReset()  →  penUp()  →  read first point
     │
     ▼
┌── Read next (x, y) from PROGMEM ──────────────────┐
│                                                     │
│   Is it PATH_END_MARK (-9999)?  → penUp() → home   │
│                                                     │
│   Is it PEN_UP_MARK (-8888)?                        │
│     YES: penUp(), read next point,                  │
│          moveTo(point), penDown()                   │
│     NO:  moveTo(x, y) with pen down                 │
│                                                     │
└─────────────────────────────────────────────────────┘
```

**Sentinel values in the coordinate data:**

| Value | Meaning |
|-------|---------|
| `-8888.0` | Pen up marker — lift pen before the next move |
| `-9999.0` | End of drawing — stop and return home |

### Status indicators during drawing

- **Red LED (D11)**: ON while drawing
- **Green LED (D13)**: ON when drawing is complete
- **Serial output**: Logs each move (`>> x,y` for pen-up travel, `-- x,y` for pen-down drawing)

---

## Serial Command Interface

### `handleCmd(const char *c)`

Parses newline-terminated commands from the Serial Monitor (115200 baud):

| Command | Example | What it does |
|---------|---------|-------------|
| `D` | `D` | Execute the full drawing sequence |
| `G X Y` | `G 25.0 10.5` | Move to coordinates (25.0, 10.5) mm |
| `PU` | `PU` | Lift the pen |
| `PD` | `PD` | Lower the pen |
| `H` | `H` | Reset position to (0, 0), zero encoders |
| `?` | `?` | Print current (posX, posY) to serial |

Commands are read character by character in the main loop, buffered until a newline (`\n`) is received, then dispatched to `handleCmd()`.

---

## Main Loop

```c
void loop() {
    // 1. Control tick every LOOP_MS (10 ms)
    if (millis() - lastTick >= LOOP_MS) {
        updateOdometry();
        pidStep(dt);
        lastTick = millis();
    }

    // 2. Button check (with 50 ms debounce)
    if (buttonPressed && debounced) {
        executeDrawing();  // blocks until done
    }

    // 3. Serial command reader
    if (Serial.available()) {
        // buffer characters, dispatch on newline
    }
}
```

---

## Creating Custom Drawings

### Step 1: Design your drawing as a series of (x, y) coordinates in mm

Plan your drawing as connected line segments. Each continuous stroke is a series of points the robot moves through with the pen down.

### Step 2: Format the data in `hello_drawing.h`

```c
#include <avr/pgmspace.h>

#define PEN_UP_MARK  -8888.0f
#define PATH_END_MARK -9999.0f

const float drawing_data[] PROGMEM = {
    // --- Stroke 1 (pen starts down) ---
    0.0f,  0.0f,     // move to (0, 0)
    0.0f,  30.0f,    // draw line to (0, 30)

    // --- Lift pen, move to new stroke ---
    PEN_UP_MARK, PEN_UP_MARK,
    10.0f, 0.0f,     // travel to (10, 0) with pen up, then pen down

    // --- Stroke 2 ---
    10.0f, 30.0f,    // draw line to (10, 30)

    // --- End ---
    PATH_END_MARK, PATH_END_MARK
};

const int drawing_len = sizeof(drawing_data) / sizeof(drawing_data[0]);
```

### Rules

1. Coordinates are flat pairs: `x1, y1, x2, y2, ...`
2. The first point is a pen-down move (the robot travels there with pen up automatically, then lowers)
3. Use `PEN_UP_MARK, PEN_UP_MARK` between strokes — the robot lifts the pen, reads the next point, travels there, then lowers the pen
4. End with `PATH_END_MARK, PATH_END_MARK`
5. All values must have the `f` suffix (they are floats)
6. The array must be `PROGMEM` — this stores it in flash (32 KB) instead of SRAM (2 KB)

### Tips

- Keep coordinates positive and reasonable (under ~200 mm in each direction)
- The robot's accuracy is about 0.8 mm, so details smaller than ~2 mm won't render well
- Test individual moves with the `G X Y` serial command before committing to a full drawing
- Use the `?` command to check actual vs. expected position after moves

---

## Tuning Guide

If the robot isn't drawing accurately, adjust these parameters:

### Robot drives too aggressively or overshoots

- Decrease `KP` (try 2.0)
- Increase `KD` (try 0.8)

### Robot stops short of target / drifts

- Increase `KI` (try 1.0)
- Decrease `POS_TOL_MM` (try 0.5 — but too small may cause oscillation)

### Robot oscillates around the target

- Decrease `KP`
- Increase `KD`
- Increase `POS_TOL_MM`

### Motors stall at low speeds

- Increase `DEADBAND` (try 30-40)

### Drawing is scaled wrong

- Measure your actual wheel diameter and update `WHEEL_DIA_MM`
- Verify `ENCODER_CPR` matches your encoder (12 PPR x gear ratio)

### Pen doesn't lift cleanly / drags

- Adjust `PEN_UP_ANGLE` and `PEN_DOWN_ANGLE`
- Increase `PEN_SETTLE_MS` if the servo is slow
