# Omni-Wheel Drawing Robot

An Arduino-based three-wheeled holonomic robot that draws pre-programmed patterns on paper. Built for Engineering 1 coursework.

The robot uses three omni-wheels arranged 120 degrees apart, allowing it to slide in any direction without turning. A servo-actuated pen mechanism lifts and lowers to create drawings from coordinate data stored in flash memory.

---

## Table of Contents

- [Features](#features)
- [Hardware Overview](#hardware-overview)
- [Bill of Materials](#bill-of-materials)
- [Pinout Diagram](#pinout-diagram)
- [Wiring Reference](#wiring-reference)
- [Assembly](#assembly)
- [Software Setup](#software-setup)
- [Usage](#usage)
- [Serial Commands](#serial-commands)
- [Project Structure](#project-structure)
- [License](#license)

---

## Features

- **Holonomic drive** — moves in any direction without rotating, using 3 omni-wheels at 120-degree intervals
- **PID position control** — closed-loop feedback at 100 Hz for accurate movement (within 0.8 mm)
- **Magnetic encoder feedback** — quadrature Hall-effect encoders on each motor (600 ticks/revolution)
- **Pen lift servo** — SG90 micro servo raises and lowers a pen for drawing
- **PROGMEM drawing storage** — coordinate paths stored in flash memory to conserve SRAM
- **Serial command interface** — manual control and debugging at 115200 baud
- **Button start with LED indicators** — push to draw, red LED while active, green LED when done

---

## Hardware Overview

| Component | Specification |
|-----------|---------------|
| Microcontroller | Arduino Uno R3 (ATmega328P, 16 MHz) |
| Motor Driver | Adafruit Motor Shield V2 (I2C) |
| Motors | 3x Adafruit N20 DC Motors, 6V, 1:50 gear ratio |
| Encoders | Magnetic, 12 PPR (600 ticks/rev after gearing) |
| Pen Servo | SG90 Micro Servo |
| Wheels | 48 mm diameter omni-wheels |
| Robot Radius | 60 mm (center to wheel contact) |

---

## Bill of Materials

| Qty | Part | Notes |
|-----|------|-------|
| 1 | Arduino Uno R3 | Main controller |
| 1 | Adafruit Motor Shield V2 | Stacks on top of the Uno |
| 3 | N20 DC motor with magnetic encoder | 6V, 1:50 gear ratio, 6-wire |
| 3 | 48 mm omni-wheels | Press-fit onto motor shafts |
| 1 | SG90 micro servo | For pen lift mechanism |
| 1 | Momentary pushbutton | Normally open |
| 1 | Red LED + 220 ohm resistor | Drawing indicator |
| 1 | Green LED + 220 ohm resistor | Complete indicator |
| — | Jumper wires, mounting hardware | As needed |

---

## Pinout Diagram

```
                        ARDUINO UNO R3
                   ┌──────────────────────┐
                   │          USB         │
                   │       ┌──────┐       │
                   │       └──────┘       │
                   │                      │
         (TX) D1  ─┤                      ├─  Vin
         (RX) D0  ─┤                      ├─  GND
              D2  ─┤■ Encoder 1 Hall A    ├─  GND
              D3  ─┤■ Encoder 2 Hall A    ├─  5V
              D4  ─┤■ Encoder 1 Hall B    ├─  3.3V
              D5  ─┤■ Encoder 2 Hall B    ├─  RESET
              D6  ─┤                      ├─  IOREF
              D7  ─┤■ Encoder 3 Hall B    ├─
              D8  ─┤■ Encoder 3 Hall A    ├─  A0
              D9  ─┤                      ├─  A1
             D10  ─┤■ Pen Servo (PWM)     ├─  A2
             D11  ─┤■ Red LED             ├─  A3
             D12  ─┤■ Start Button        ├─  A4 ■ SDA (Motor Shield)
             D13  ─┤■ Green LED           ├─  A5 ■ SCL (Motor Shield)
                   │                      │
                   └──────────────────────┘

            ■ = Pin in use

  ┌──────────────────────────────────────────────────────────┐
  │                  ADAFRUIT MOTOR SHIELD V2                │
  │                  (stacked on Arduino Uno)                │
  │                                                         │
  │   Motor Terminals            Servo Headers              │
  │  ┌────┐ ┌────┐ ┌────┐      ┌──────────┐               │
  │  │ M1 │ │ M2 │ │ M3 │      │ Servo 1  │──→ Pin D10    │
  │  │    │ │    │ │    │      │ (pen)    │               │
  │  └─┬──┘ └─┬──┘ └─┬──┘      └──────────┘               │
  │    │      │      │                                     │
  │    │      │      │         Communication               │
  │    │      │      │         I2C: SDA (A4), SCL (A5)     │
  └────┼──────┼──────┼─────────────────────────────────────┘
       │      │      │
       ▼      ▼      ▼
    Motor1  Motor2  Motor3
   (front) (r-left)(r-right)
```

### Wheel Layout (Top-Down View)

```
                   Front
                     │
              Wheel 1 (M1)
                  ●  90°
                 /|\
                / | \
               /  |  \
              /   |   \
             /    |    \
            /     |     \
    Wheel 2 (M2)  │  Wheel 3 (M3)
       ●  210°    │    330°  ●
                  │
            ← 60 mm →
         (center to wheel)

    All wheels angled for holonomic motion.
    Robot can translate in any direction
    without rotating.
```

---

## Wiring Reference

### Encoder Connections (per motor)

Each N20 motor has 6 wires:

| Wire | Function | Connection |
|------|----------|------------|
| 1 | Motor (+) | Motor Shield terminal (M1/M2/M3) |
| 2 | Motor (-) | Motor Shield terminal (M1/M2/M3) |
| 3 | Encoder VCC | Arduino 5V |
| 4 | Encoder GND | Arduino GND |
| 5 | Hall Sensor A | See pin table below |
| 6 | Hall Sensor B | See pin table below |

### Encoder Pin Assignments

| Motor | Hall A Pin | Interrupt Type | Hall B Pin |
|-------|-----------|----------------|------------|
| Motor 1 (front) | D2 | Hardware INT0 | D4 |
| Motor 2 (rear-left) | D3 | Hardware INT1 | D5 |
| Motor 3 (rear-right) | D8 | Pin-change PCINT0 | D7 |

### LED and Button Wiring

```
  D11 ──── [220Ω] ──── RED LED ──── GND     (drawing active)
  D13 ──── [220Ω] ──── GREEN LED ── GND     (drawing complete)
  D12 ──── BUTTON ──── GND                  (uses internal pullup)
```

---

## Assembly

> **CAD files and detailed assembly instructions will be added soon.**

General steps:

1. Mount the three N20 motors in the chassis at 120-degree intervals
2. Press omni-wheels onto the motor shafts
3. Stack the Adafruit Motor Shield onto the Arduino Uno
4. Connect motor power wires to M1, M2, M3 terminals on the shield
5. Wire encoder signals to the Arduino pins as shown above
6. Mount the SG90 servo for the pen lift mechanism
7. Connect the servo to the "Servo 1" header on the Motor Shield
8. Wire the button, LEDs, and resistors on a small breadboard or perfboard

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

---

## Usage

1. Power on the Arduino — the serial monitor prints `READY`
2. Place the robot on a flat surface with paper underneath
3. Insert a pen or marker into the pen holder
4. **Press the button** or send `D` via serial to start drawing
5. The red LED turns on while drawing
6. When finished, the robot returns to its starting position, the red LED turns off, and the green LED turns on

### Default Drawing

The robot draws **"HELLO"** in block letters (~92 mm wide x 30 mm tall). Custom drawings can be created by modifying `Code/hello_drawing.h`.

---

## Serial Commands

Connect at **115200 baud**. Available commands:

| Command | Action |
|---------|--------|
| `D` | Start the drawing sequence |
| `G X Y` | Move to position (X, Y) in mm |
| `PU` | Pen up (lift) |
| `PD` | Pen down (lower) |
| `H` | Home — reset position to (0, 0) |
| `?` | Print current position |

---

## Project Structure

```
Engr1-Robot/
├── README.md               ← You are here
├── Code/
│   ├── README.md           ← Detailed code documentation
│   ├── omni_draw_robot.ino ← Main firmware (Arduino sketch)
│   └── hello_drawing.h     ← Drawing coordinate data
└── (CAD files coming soon)
```

See [`Code/README.md`](Code/README.md) for a full breakdown of the firmware architecture, functions, and how to create custom drawings.

---

## License

This project was built for educational purposes as part of an Engineering 1 course.
