# Code Documentation — Omni-Wheel G-code Plotter

This document explains the firmware architecture and how to create your own drawings.

---

## File Overview

| File | Purpose |
|------|---------|
| `omni_draw_robot.ino` | Plotter firmware — G-code interpreter, PID control, motor/encoder/servo drivers |
| `drawing.h` | G-code drawing program stored in PROGMEM — **replace this to draw something different** |

The firmware is a generic G-code plotter. You never need to modify `omni_draw_robot.ino` — just replace `drawing.h` with your own G-code.

---

## Architecture

```
Power on / Reset
      │
      ▼
  setup()
      │
      ├── Initialize hardware (motors, encoders, servo)
      ├── Calibrate encoder polarity (auto-detects reversed wiring)
      ├── Wait for button press on pin 12
      │
      ▼
  runGCodeProgram(gcode_program)
      │
      ├── Read G-code line from PROGMEM
      ├── Parse command (G0, G1, G2, G3, M3, M5)
      ├── Execute:
      │     G0 → penUp() + gcMoveTo()
      │     G1 → penDown() + gcMoveTo()
      │     G2/G3 → gcArc() (full circle or segmented)
      │     M3/M5 → penDown() / penUp()
      └── Repeat until end of program
      │
      ▼
  Done — motors stopped, pen up
```

---

## Configuration Parameters

All tunable values are near the top of `omni_draw_robot.ino`:

### Physical Dimensions

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ENCODER_CPR` | 700.0 | Encoder ticks per wheel revolution (14 CPR × 50:1 gear) |
| `WHEEL_DIA_MM` | 36.0 | Outer diameter of the omni-wheels (mm) |
| `MM_PER_TICK` | ~0.1616 | mm per encoder tick (computed: π × 36 / 700) |

### Servo

| Parameter | Default | Description |
|-----------|---------|-------------|
| `PEN_UP_ANGLE` | 45 | Servo angle when pen is lifted (degrees) |
| `PEN_DOWN_ANGLE` | 0 | Servo angle when pen touches paper (degrees) |
| `PEN_SETTLE_MS` | 300 | Delay after servo move (ms) |

### PID Gains

Three separate PID gain sets are used for different move types:

| Gain Set | kp | kd | ki | Used For |
|----------|-----|------|------|----------|
| Travel | 20.0 | 0.5 | 1.0 | G0 rapid moves (pen up) |
| Line | 30.0 | 0.5 | 1.0 | G1 straight draws (pen down) |
| Curve | 30.0 | 0.5 | 1.0 | Arc segment sub-moves |

### Movement

| Parameter | Default | Description |
|-----------|---------|-------------|
| `POS_TOL_MM` | 3.0 | Arrival tolerance (mm) — how close counts as "arrived" |
| `MOVE_TIMEOUT_MS` | 8000 | Max time per move segment (ms) |
| `ARC_SEG_MM` | 10.0 | Arc subdivision step size (mm) |
| `CIRCLE_TIMEOUT_MS` | 20000 | Max time for a full circle (ms) |
| `minPWM` | 50 | Minimum PWM to overcome motor dead zone |
| `SPEED` | 255 | Full PWM for open-loop circle drawing |

---

## How the G-code Interpreter Works

### Coordinate System

- All G-code coordinates are **absolute** in **millimeters**
- Origin (0, 0) = where the robot was when the button was pressed
- The interpreter tracks absolute position in `gcX`, `gcY`
- Each move is converted to a relative PID move internally

### Move Execution (2-Wheel Decomposition)

Diagonal moves cause the robot to rotate because it has no IMU to correct heading drift. To prevent this, each G0/G1 move is decomposed into two legs that each engage exactly 2 wheels symmetrically:

1. **Leg 1 — 30° direction**: Covers all of dx (and some dy). Engages M1 + M3 equally, M2 idle.
2. **Leg 2 — Pure Y**: Covers the remaining dy. Engages M2 + M3 equally, M1 idle.

For each leg:
1. Reset odometry to (0, 0)
2. Run PID controller to move the delta relative
3. Update `gcX`, `gcY` to the new absolute position

Arc segments use direct diagonal moves (no decomposition) since the segments are small enough (~10mm) that rotation is minimal.

### Arc Handling

- **Full circles** (endpoint ≈ startpoint): Uses open-loop velocity sweep for smoothness. The robot drives tangent to the circle at full speed, tracking angle via `theta = path_length / radius`.
- **Partial arcs**: Subdivided into straight-line segments (~10mm each) and PID-moved through each point. This is how most low-end CNC controllers handle arcs.

---

## PID Controller

Follows the [curiores/ArduinoTutorials](https://github.com/curiores/ArduinoTutorials) pattern:

```
error = target - current_position
derivative = (error - prev_error) / deltaT
integral += error * deltaT
output = kp * error + kd * derivative + ki * integral
```

- Uses `micros()` for precise timing
- Runs as fast as possible (no fixed loop interval)
- Motor power = `fabs(output)`, direction = `sign(output)`
- Minimum PWM floor (`minPWM = 50`) to overcome motor dead zone — wheel speeds are scaled up so the slowest active wheel meets this threshold

### Tuning Guide

| Problem | Fix |
|---------|-----|
| Robot overshoots target | Increase `kd` (try 0.8–1.0) |
| Robot doesn't reach target / stalls | Increase `kp` (try 50–60) or `ki` (try 15) |
| Robot oscillates back and forth | Decrease `kp`, increase `kd` |
| Drawing is wrong size | Verify `WHEEL_DIA_MM` and `ENCODER_CPR` match your hardware |
| Pen drags / doesn't lift | Adjust `PEN_UP_ANGLE` and `PEN_DOWN_ANGLE` |

---

## Creating a drawing.h File

### Format

The G-code is stored as a single C string in PROGMEM (flash memory):

```c
#ifndef DRAWING_H
#define DRAWING_H
#include <avr/pgmspace.h>

const char gcode_program[] PROGMEM =
  "G0 X0 Y-50\n"
  "G3 X0 Y-50 I0 J50\n"
  "G0 X0 Y0\n"
;

#endif
```

### Rules

1. Each G-code line must end with `\n`
2. The variable must be named `gcode_program`
3. Must use `PROGMEM` (stores in flash, not RAM)
4. Lines starting with `;` are treated as comments
5. Max line length: 63 characters
6. Supported: G0, G1, G2, G3, M3, M5

### Generating G-code

**From Inkscape (free vector editor):**
1. Draw your design as vector paths
2. Extensions → Gcodetools → Path to Gcode
3. Copy the output, clean up unsupported commands
4. Paste into `drawing.h`

**From online tools:**
- [jscut.org](https://jscut.org/) — paste SVG, export G-code
- [svg2gcode](https://sameer.github.io/svg2gcode/) — browser-based converter

**By hand** — for simple shapes, just write G0/G1/G2/G3 commands directly.

### Cleaning Up Generated G-code

Most G-code generators output commands this robot doesn't support. Remove or replace:

| Remove | Replace with |
|--------|-------------|
| `Z` moves (Z-1, Z1) | `M3` (pen down) / `M5` (pen up) |
| `F` feed rates | Delete (robot ignores speed commands) |
| `S` spindle speed | Delete |
| `G20` (inches) | Convert coordinates to mm |
| `G90`/`G91` | Delete (robot always uses absolute positioning) |
| `T` tool changes | Delete |

---

## Encoder System

### Interrupt Strategy

| Motor | Pin | Method |
|-------|-----|--------|
| Motor 1 | D2 | Hardware interrupt INT0 (fastest) |
| Motor 2 | D3 | Hardware interrupt INT1 (fastest) |
| Motor 3 | D8 | Pin-change interrupt PCINT0 (filtered for rising edge) |

### Auto-Calibration

At startup, the firmware briefly spins each motor FORWARD and checks if the encoder counts go up or down. If they go down, that encoder's polarity is flipped. This handles reversed encoder wiring automatically.

---

## Kinematics

### Inverse Kinematics (velocity → wheel speeds)

```
wheel_speed[i] = -sin(angle[i]) * vx + cos(angle[i]) * vy
```

### Forward Kinematics (wheel distances → robot displacement)

```
dx = (2/3) * (-sin(90°)*d1 - sin(210°)*d2 - sin(330°)*d3)
dy = (2/3) * ( cos(90°)*d1 + cos(210°)*d2 + cos(330°)*d3)
```

The `(2/3)` factor comes from the pseudo-inverse of the 3-wheel kinematics matrix.
