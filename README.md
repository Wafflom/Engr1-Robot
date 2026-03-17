# Omni-Wheel G-code Plotter

An Arduino-based three-wheeled holonomic robot that draws any design from G-code. Built for Engineering 1 coursework.

The robot uses three omni-wheels arranged 120 degrees apart, allowing it to slide in any direction without turning. A servo-actuated pen mechanism lifts and lowers to create drawings. The firmware is a generic G-code interpreter — just change the G-code file to draw something different.

---

## Table of Contents

- [How It Works](#how-it-works)
- [Features](#features)
- [Hardware Overview](#hardware-overview)
- [Bill of Materials](#bill-of-materials)
- [Wiring Schematic](#wiring-schematic)
- [Wheel Layout](#wheel-layout)
- [Assembly](#assembly)
- [Software Setup](#software-setup)
- [Usage](#usage)
- [Creating Custom Drawings](#creating-custom-drawings)
- [G-code Reference](#g-code-reference)
- [License](#license)

---

## How It Works

The robot is essentially a portable CNC plotter:

1. **Firmware** (`omni_draw_robot.ino`) — the "machine". Handles motors, encoders, PID control, and G-code parsing. You never need to modify this.
2. **Drawing file** (`drawing.h`) — the "job". Contains the G-code program that defines what to draw. Swap this file to draw different designs.
3. **Button** — press to start the job.

The firmware reads the G-code line by line, moves the robot to each position using encoder-feedback PID control, and raises/lowers the pen as commanded.

---

## Features

- **G-code interpreter** — supports G0 (rapid), G1 (linear), G2/G3 (arc), M3/M5 (pen control)
- **Holonomic drive** — moves in any direction without rotating, using 3 omni-wheels at 120-degree intervals
- **PID position control** — closed-loop encoder feedback for accurate movement
- **Magnetic encoder feedback** — Hall-effect encoders on each motor (700 ticks/wheel revolution)
- **Auto-calibration** — detects reversed encoder wiring at startup and corrects automatically
- **Pen lift servo** — SG90 micro servo raises and lowers a pen
- **Button start** — press to begin drawing
- **PROGMEM storage** — G-code stored in flash memory (32KB) to conserve SRAM (2KB)

---

## Hardware Overview

| Component | Specification |
|-----------|---------------|
| Microcontroller | Arduino Uno R3 (ATmega328P, 16 MHz) |
| Motor Driver | Adafruit Motor Shield V2 (I2C) |
| Motors | 3x N20 DC Motors, 6V, 1:50 gear ratio |
| Encoders | Magnetic Hall-effect, 14 CPR (700 ticks/rev after gearing) |
| Pen Servo | SG90 Micro Servo |
| Wheels | 36 mm diameter omni-wheels |

---

## Bill of Materials

| Qty | Part | Notes |
|-----|------|-------|
| 1 | Arduino Uno R3 | Main controller |
| 1 | Adafruit Motor Shield V2 | Stacks on top of the Uno |
| 3 | N20 DC motor with magnetic encoder | 6V, 1:50 gear ratio, 6-wire |
| 3 | 36 mm omni-wheels | Press-fit onto motor shafts |
| 1 | SG90 micro servo | For pen lift mechanism |
| 1 | Momentary pushbutton | Wired between pin 12 and GND |
| — | Jumper wires, mounting hardware | As needed |

---

## Wiring Schematic

### Pin Reference Table

| Arduino Pin | Function | Connects To | Notes |
|-------------|----------|-------------|-------|
| D2 | Encoder 1 Hall A | Motor 1 encoder (front) | Hardware interrupt INT0, rising edge |
| D3 | Encoder 2 Hall A | Motor 2 encoder (rear-left) | Hardware interrupt INT1, rising edge |
| D4 | Encoder 1 Hall B | Motor 1 encoder (front) | Digital read for direction |
| D5 | Encoder 2 Hall B | Motor 2 encoder (rear-left) | Digital read for direction |
| D7 | Encoder 3 Hall B | Motor 3 encoder (rear-right) | Digital read for direction |
| D8 | Encoder 3 Hall A | Motor 3 encoder (rear-right) | Pin-change interrupt PCINT0 |
| D10 | Servo PWM | SG90 pen servo | Via Motor Shield Servo 1 header |
| D12 | Start button | Pushbutton to GND | INPUT_PULLUP, active LOW |
| A4 (SDA) | I2C data | Motor Shield V2 | Shared I2C bus |
| A5 (SCL) | I2C clock | Motor Shield V2 | Shared I2C bus |

### Motor Shield Connections

| Terminal | Motor | Position |
|----------|-------|----------|
| M1 | N20 DC motor + encoder | Front (90°) |
| M2 | N20 DC motor + encoder | Rear-left (210°) |
| M3 | N20 DC motor + encoder | Rear-right (330°) |
| VIN / GND | 6V power supply | Motor power input |

### Encoder Wiring (each motor, 6 wires)

| Wire | Connects To |
|------|-------------|
| Motor + | Motor Shield M terminal |
| Motor - | Motor Shield M terminal |
| Encoder VCC | Arduino 5V rail |
| Encoder GND | Arduino GND rail |
| Hall Sensor A | See pin table (interrupt pin) |
| Hall Sensor B | See pin table (direction pin) |

---

## Wheel Layout

Three omni-wheels at 120-degree intervals allow the robot to translate in any direction without rotating.

---

## Assembly

1. Mount the three N20 motors in the chassis at 120-degree intervals
2. Press omni-wheels onto the motor shafts
3. Stack the Adafruit Motor Shield onto the Arduino Uno
4. Connect motor power wires to M1, M2, M3 terminals on the shield
5. Wire encoder signals to the Arduino pins as shown above
6. Mount the SG90 servo for the pen lift mechanism
7. Connect the servo to the "Servo 1" header on the Motor Shield
8. Wire the button between pin 12 and GND (no resistor needed — firmware uses internal pullup)

---

## Software Setup

### Requirements

- [Arduino IDE](https://www.arduino.cc/en/software) (1.8+ or 2.x)
- Libraries (install via Library Manager):
  - `Adafruit Motor Shield V2 Library`
  - `Servo` (built-in)
  - `Wire` (built-in)

### Upload

1. Open `Code/omni_draw_robot.ino` in the Arduino IDE
2. Select **Board: Arduino Uno** and the correct **Port**
3. Click **Upload**
4. Open the Serial Monitor at **115200 baud** to see status messages

See [`Code/README.md`](Code/README.md) for firmware documentation, G-code reference, and how to create custom drawings.

---

## Usage

1. Power on the Arduino
2. Place the robot on a flat surface with paper underneath
3. Insert a pen or marker into the pen holder
4. Open Serial Monitor at 115200 baud to see status
5. **Press the button** on pin 12 to start drawing
6. When finished, the robot stops and lifts the pen
7. Press the Arduino **reset button** to draw again

### Default Drawing

The included `drawing.h` draws a **smiley face** — a 150mm radius circle with two eyes (vertical lines) and a smile (arc).

---

## Creating Custom Drawings

### Option 1: Write G-code by hand

For simple shapes (lines, circles, rectangles), write the G-code directly. See the [G-code Reference](#g-code-reference) below.

### Option 2: Generate G-code from a vector drawing

1. **Draw your design** in [Inkscape](https://inkscape.org/) (free) or any vector editor
2. **Convert to G-code** using one of these tools:
   - **Inkscape + Gcodetools extension**: Extensions → Gcodetools → Path to Gcode
   - **Inkscape + J Tech Laser extension**: good for simple path export
   - **[jscut.org](https://jscut.org/)**: browser-based, paste SVG and export G-code
   - **[svg2gcode](https://sameer.github.io/svg2gcode/)**: browser-based SVG to G-code converter
3. **Clean up the G-code**:
   - Remove Z-axis moves (replace with M3/M5 for pen down/up, or just remove)
   - Remove feed rate commands (F values) — the robot ignores them
   - Remove any G-code commands the robot doesn't support
   - Make sure all coordinates are in mm
4. **Paste into `drawing.h`**: Replace the contents of `gcode_program[]` with your G-code
5. **Upload** to Arduino

### Option 3: Export from CAD software

Most CAD/CAM software can export G-code (Fusion 360, FreeCAD, etc.). Export as a 2D profile operation with:
- Units: mm
- No Z-axis moves (or replace with pen up/down)
- G0/G1/G2/G3 commands only

### Tips

- Keep your drawing within a reasonable area (300mm x 300mm works well)
- The robot's accuracy is about 3mm, so fine details won't render well
- Simpler designs with fewer moves work best
- Test with a dry run first (pen removed) to check the path

---

## G-code Reference

### Supported Commands

| Command | Description | Example |
|---------|-------------|---------|
| `G0 Xn Yn` | Rapid move — pen lifts, robot travels to (X, Y) | `G0 X50 Y-30` |
| `G1 Xn Yn` | Linear draw — pen lowers, draws line to (X, Y) | `G1 X100 Y0` |
| `G2 Xn Yn In Jn` | Clockwise arc to (X, Y), center offset (I, J) | `G2 X10 Y0 I5 J0` |
| `G3 Xn Yn In Jn` | Counter-clockwise arc to (X, Y), center offset (I, J) | `G3 X0 Y-10 I0 J10` |
| `M3` | Pen down (explicit) | `M3` |
| `M5` | Pen up (explicit) | `M5` |
| `; text` | Comment (ignored) | `; this is a comment` |

### Coordinate System

- All coordinates are **absolute** in **millimeters**
- Origin (0, 0) is where the robot sits when you press the button
- X = left/right, Y = forward/backward

### Arc Commands (G2/G3)

- `X, Y` = endpoint of the arc
- `I, J` = offset from the **current position** to the arc center
- If the endpoint equals the start point, it draws a **full circle**
- G2 = clockwise, G3 = counter-clockwise

### Example: Drawing a Square

```gcode
; 50mm square starting at origin
G0 X0 Y0        ; move to start
G1 X50 Y0       ; draw right
G1 X50 Y50      ; draw up
G1 X0 Y50       ; draw left
G1 X0 Y0        ; draw down (close square)
G0 X0 Y0        ; return to origin
```

### Example: Drawing a Circle

```gcode
; 40mm radius circle centered at origin
G0 X0 Y-40           ; move to bottom of circle
G3 X0 Y-40 I0 J40    ; full CCW circle (center offset 0, +40)
G0 X0 Y0             ; return to origin
```

---

## License

This project was built for educational purposes as part of an Engineering 1 course.
