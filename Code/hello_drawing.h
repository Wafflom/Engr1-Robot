/*
 * hello_drawing.h — Drawing data for the word "HELLO"
 * ====================================================
 *
 * This file stores all the coordinates the robot needs to draw
 * the word "HELLO" in block letters. The data lives in PROGMEM
 * (the Uno's flash memory, which has 32KB) instead of SRAM
 * (which only has 2KB and fills up fast with variables).
 *
 * HOW THE DATA FORMAT WORKS:
 *
 *   The drawing_data[] array is a flat list of float numbers,
 *   read in pairs: (x, y), (x, y), (x, y), ...
 *
 *   Normally, consecutive points are connected by straight lines
 *   drawn with the pen DOWN.
 *
 *   Two special marker values control the pen:
 *
 *     PEN_UP_MARK (-8888, -8888)
 *       Means: "lift the pen." The NEXT point after this marker
 *       is where the robot travels to (with pen still up).
 *       Once it arrives there, it puts the pen back down and
 *       continues drawing.
 *
 *     PATH_END_MARK (-9999, -9999)
 *       Means: "the drawing is done." Must be the last entry.
 *
 *   Example of how the robot reads the data:
 *
 *     0, 0,         ← travel to (0,0) with pen up, then pen down
 *     0, 30,        ← draw a line from (0,0) to (0,30)
 *     -8888, -8888, ← lift pen
 *     10, 0,        ← travel to (10,0) with pen up, then pen down
 *     10, 30,       ← draw a line from (10,0) to (10,30)
 *     -9999, -9999  ← done!
 *
 * COORDINATE SYSTEM:
 *   X = rightward (millimeters)
 *   Y = upward / forward (millimeters)
 *   Origin (0,0) = where the robot starts
 *
 * HOW TO MAKE YOUR OWN DRAWING:
 *   1. Copy this file and give it a new name (e.g. "star.h")
 *   2. Replace the drawing_data[] array with your coordinates
 *   3. Make sure you end with PATH_END_MARK, PATH_END_MARK
 *   4. In the .ino file, change: #include "hello_drawing.h"
 *      to: #include "star.h"
 *   5. Upload!
 */

// Header guard: prevents this file from being included twice,
// which would cause a "multiple definition" compiler error.
#ifndef HELLO_DRAWING_H
#define HELLO_DRAWING_H

// Gives us access to pgm_read_float() and the PROGMEM keyword
#include <avr/pgmspace.h>

// Sentinel values used to control pen state in the data array.
// These are intentionally extreme numbers that would never appear
// as real coordinates, so the code can tell them apart.
#define PEN_UP_MARK   -8888.0f    // signals: lift the pen
#define PATH_END_MARK -9999.0f    // signals: drawing is finished

// ============================================================
//  "HELLO" — block letter coordinates
//
//  Each letter is drawn as a series of straight-line strokes.
//  Between letters (and between strokes within a letter), we
//  insert PEN_UP_MARK to lift the pen and reposition.
//
//  Letter sizes (all in mm):
//    H : 16mm wide × 30mm tall  (3 strokes)
//    E : 12mm wide × 30mm tall  (2 strokes)
//    L : 12mm wide × 30mm tall  (1 stroke)
//    L : 12mm wide × 30mm tall  (1 stroke)
//    O : 16mm wide × 30mm tall  (1 stroke, closed rectangle)
//
//  Spacing: 6mm gap between each letter
//
//  Total width: 16 + 6 + 12 + 6 + 12 + 6 + 12 + 6 + 16 = 92mm
// ============================================================

// PROGMEM tells the compiler to store this array in flash memory
// instead of SRAM. On the Uno, SRAM is only 2KB — too small for
// large drawings. Flash has 32KB and is normally used for code,
// but PROGMEM lets us put read-only data there too.
//
// The tradeoff: you can't read PROGMEM data with normal array
// syntax. You have to use pgm_read_float(&array[index]).

const float drawing_data[] PROGMEM = {

  // ==================== LETTER H (x: 0 to 16) ====================

  // Stroke 1: left vertical bar (bottom to top)
  0.0f, 0.0f,        // start at bottom-left of the H
  0.0f, 30.0f,       // draw straight up 30mm

  PEN_UP_MARK, PEN_UP_MARK,   // lift pen, reposition

  // Stroke 2: horizontal crossbar (left to right)
  0.0f, 15.0f,       // start at left side, halfway up
  16.0f, 15.0f,      // draw across to right side

  PEN_UP_MARK, PEN_UP_MARK,   // lift pen, reposition

  // Stroke 3: right vertical bar (bottom to top)
  16.0f, 0.0f,       // start at bottom-right of the H
  16.0f, 30.0f,      // draw straight up 30mm

  PEN_UP_MARK, PEN_UP_MARK,   // lift pen, move to next letter

  // ==================== LETTER E (x: 22 to 34) ====================
  // (starts at x=22 because H ends at x=16, plus 6mm gap)

  // Stroke 1: three sides in one continuous stroke
  // (top bar + left bar + bottom bar, drawn as one path)
  34.0f, 30.0f,      // start at top-right corner
  22.0f, 30.0f,      // draw left across the top
  22.0f, 0.0f,       // draw down the left side
  34.0f, 0.0f,       // draw right across the bottom

  PEN_UP_MARK, PEN_UP_MARK,   // lift pen, reposition

  // Stroke 2: middle horizontal bar
  22.0f, 15.0f,      // start at left side, middle height
  32.0f, 15.0f,      // draw across (slightly shorter than top/bottom)

  PEN_UP_MARK, PEN_UP_MARK,   // lift pen, move to next letter

  // ==================== LETTER L (x: 40 to 52) ====================

  // Single stroke: vertical bar + bottom horizontal
  40.0f, 30.0f,      // start at top
  40.0f, 0.0f,       // draw down to bottom-left corner
  52.0f, 0.0f,       // draw right along the bottom

  PEN_UP_MARK, PEN_UP_MARK,   // lift pen, move to next letter

  // ==================== LETTER L (x: 58 to 70) ====================

  // Same shape as the previous L, just shifted right
  58.0f, 30.0f,      // start at top
  58.0f, 0.0f,       // draw down
  70.0f, 0.0f,       // draw right along bottom

  PEN_UP_MARK, PEN_UP_MARK,   // lift pen, move to next letter

  // ==================== LETTER O (x: 76 to 92) ====================

  // Single stroke: closed rectangle (returns to start point)
  76.0f, 0.0f,       // start at bottom-left
  92.0f, 0.0f,       // draw right along bottom
  92.0f, 30.0f,      // draw up the right side
  76.0f, 30.0f,      // draw left across the top
  76.0f, 0.0f,       // draw down to starting point (closes the rectangle)

  // ==================== END OF DRAWING ====================
  PATH_END_MARK, PATH_END_MARK  // tells the drawing engine to stop
};

// Calculate how many floats are in the array.
// sizeof(drawing_data) gives total bytes; dividing by sizeof(float)
// gives the number of float elements (each coordinate is one float).
// The drawing engine uses this to know when to stop reading.
#define DRAWING_DATA_LEN  (sizeof(drawing_data) / sizeof(float))

#endif  // HELLO_DRAWING_H
