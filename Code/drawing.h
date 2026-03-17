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
 * CURRENT DRAWING: Smiley face (150mm diameter circle)
 */

#ifndef DRAWING_H
#define DRAWING_H

#include <avr/pgmspace.h>

const char gcode_program[] PROGMEM =
  // --- Smiley Face (150mm radius) ---
  // All coordinates in mm, origin = center of face

  // Outer circle: move to bottom, draw full CCW circle
  "G0 X0 Y-150\n"           // Rapid move to bottom of circle
  "G3 X0 Y-150 I0 J150\n"   // Full CCW circle (center offset: 0, +150 from bottom)

  // Left eye: vertical line (pure Y direction for accuracy)
  "G0 X-45 Y50\n"           // Rapid move to top of left eye
  "G1 X-45 Y20\n"           // Draw line downward (pure Y, uses M2+M3)

  // Right eye: vertical line (pure Y direction for accuracy)
  "G0 X45 Y50\n"            // Rapid move to top of right eye
  "G1 X45 Y20\n"            // Draw line downward (pure Y, uses M2+M3)

  // Mouth: smile arc (CCW from left to right, curving down)
  // Arc center at (0, 20), radius ~64mm
  "G0 X-50 Y-20\n"          // Rapid move to left corner of mouth
  "G3 X50 Y-20 I50 J40\n"   // CCW arc to right corner (smile shape)

  // Return to center
  "G0 X0 Y0\n"              // Rapid move back to origin
;

#endif
