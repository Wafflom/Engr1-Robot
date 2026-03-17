/*
 * drawing.h — G-code drawing file
 *
 * This file contains the G-code program that the robot will draw.
 * Replace the contents of gcode_program[] with your own G-code
 * to draw different shapes.
 *
 * SUPPORTED G-CODE COMMANDS:
 *   G0 Xn Yn       — Rapid move (pen lifts, travels to position)
 *   G1 Xn Yn       — Linear draw (pen lowers, draws line to position)
 *   G2 Xn Yn In Jn — Clockwise arc (pen down)
 *   G3 Xn Yn In Jn — Counter-clockwise arc (pen down)
 *   M3              — Pen down (explicit)
 *   M5              — Pen up (explicit)
 *   ; comment       — Lines starting with ; are ignored
 *
 * COORDINATE SYSTEM:
 *   - All coordinates are in millimeters (absolute positioning)
 *   - Origin (0,0) is where the robot starts when you press the button
 *   - X = left/right, Y = forward/backward
 *   - I,J in arc commands = offset from current position to arc center
 *
 * ARC COMMANDS (G2/G3):
 *   - G2 = clockwise, G3 = counter-clockwise
 *   - X,Y = endpoint of the arc
 *   - I,J = offset from current position to the arc's center point
 *   - If endpoint equals startpoint, it draws a full 360-degree circle
 *
 * HOW TO GENERATE G-CODE:
 *   1. Draw your design in Inkscape (free vector editor)
 *   2. Install the "Gcodetools" or "J Tech Laser" extension
 *   3. Export as G-code (.gcode or .nc file)
 *   4. Paste the G-code lines into the string below
 *   5. Remove any commands the robot doesn't support (Z moves, feed rates, etc.)
 *   6. Make sure coordinates are in mm and fit your drawing area
 *
 * CURRENT DRAWING: Smiley face (hexagon outline + line-segment smile)
 */

#ifndef DRAWING_H
#define DRAWING_H

#include <avr/pgmspace.h>

const char gcode_program[] PROGMEM =
  // --- Smiley Face (150mm radius hexagon) ---
  // All coordinates in mm, origin = center of face

  // Outer hexagon: 6 vertices at 150mm radius
  // Vertices at 60° intervals starting from bottom (270°)
  //   0°: (150, 0)   60°: (75, 130)   120°: (-75, 130)
  // 180°: (-150, 0)  240°: (-75,-130)  300°: (75, -130)
  "G0 X75 Y-130\n"            // Rapid to bottom-right vertex (300°)
  "G1 X150 Y0\n"              // Edge to right (0°)
  "G1 X75 Y130\n"             // Edge to top-right (60°)
  "G1 X-75 Y130\n"            // Edge to top-left (120°)
  "G1 X-150 Y0\n"             // Edge to left (180°)
  "G1 X-75 Y-130\n"           // Edge to bottom-left (240°)
  "G1 X75 Y-130\n"            // Edge back to start (300°)

  // Left eye: vertical line
  "G0 X-70 Y70\n"             // Rapid to top of left eye
  "G1 X-70 Y10\n"             // Draw line downward

  // Right eye: vertical line
  "G0 X70 Y70\n"              // Rapid to top of right eye
  "G1 X70 Y10\n"              // Draw line downward

  // Mouth: smooth smile as line segments (5 points along a curve)
  // Parabolic-ish arc: y = -50 - 30*sin(t), x spans -60 to +60
  "G0 X-60 Y-30\n"            // Rapid to left corner of mouth
  "G1 X-35 Y-55\n"            // Segment 1: curve down-left
  "G1 X0 Y-65\n"              // Segment 2: curve to bottom center
  "G1 X35 Y-55\n"             // Segment 3: curve up-right
  "G1 X60 Y-30\n"             // Segment 4: curve to right corner

  // Return to center
  "G0 X0 Y0\n"
;

#endif
