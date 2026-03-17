/*
 * drawing.h — Default drawing: 25cm radius pentagram star
 *
 * Format: flat array of (x, y) pairs in mm, stored in PROGMEM.
 *   PEN_UP_MARK  = lift pen, next point is a pen-up travel
 *   PATH_END_MARK = end of drawing
 *
 * To use a different drawing, replace this file or change the
 * #include in omni_draw_robot.ino.
 */

#ifndef DRAWING_H
#define DRAWING_H

#include <avr/pgmspace.h>

#define PEN_UP_MARK   -8888.0f
#define PATH_END_MARK -9999.0f

// Pentagram star, 250mm radius, top vertex at (0, 250)
// Vertex order: V0 -> V2 -> V4 -> V1 -> V3 -> V0
const float drawing_data[] PROGMEM = {
  PEN_UP_MARK, PEN_UP_MARK,
    0.00f,   250.00f,     // V0  top
  -146.95f, -202.25f,     // V2  lower-left
   237.76f,   77.25f,     // V4  upper-right
  -237.76f,   77.25f,     // V1  upper-left
   146.95f, -202.25f,     // V3  lower-right
    0.00f,   250.00f,     // V0  close
  PATH_END_MARK, PATH_END_MARK
};

#define DRAWING_DATA_LEN (sizeof(drawing_data) / sizeof(drawing_data[0]))

#endif
