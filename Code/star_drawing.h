/*
 * star_drawing.h — Five-pointed star coordinates
 *
 * A regular 5-pointed star drawn as a single continuous stroke.
 * The star is drawn by connecting every other vertex of a regular
 * pentagon, which creates the classic star shape without lifting
 * the pen.
 *
 * Star parameters:
 *   Outer radius: 40mm (tip to center)
 *   Center offset: (0, 50) mm from robot start position
 *   Total size: ~76mm wide x ~72mm tall
 *
 * Drawing order connects vertices 0 -> 2 -> 4 -> 1 -> 3 -> 0
 * (skipping one vertex each time to form the star pattern).
 */

#ifndef STAR_DRAWING_H
#define STAR_DRAWING_H

#include <avr/pgmspace.h>

#define PEN_UP_MARK   -8888.0f
#define PATH_END_MARK -9999.0f

/*
 * Star vertices (radius 40mm, centered at (0, 50)):
 *
 *   Vertex 0 (top,  90°): (  0.00,  90.00)
 *   Vertex 1 (162°):       (-38.04,  62.36)
 *   Vertex 2 (234°):       (-23.51,  17.64)
 *   Vertex 3 (306°):       ( 23.51,  17.64)
 *   Vertex 4 ( 18°):       ( 38.04,  62.36)
 *
 * Star stroke order: 0 -> 2 -> 4 -> 1 -> 3 -> 0
 */

const float drawing_data[] PROGMEM = {

  // Single continuous stroke forming the star
    0.00f,  90.00f,    // vertex 0 (top)
  -23.51f,  17.64f,    // vertex 2 (bottom-left)
   38.04f,  62.36f,    // vertex 4 (right)
  -38.04f,  62.36f,    // vertex 1 (left)
   23.51f,  17.64f,    // vertex 3 (bottom-right)
    0.00f,  90.00f,    // back to vertex 0 (close the star)

  PATH_END_MARK, PATH_END_MARK
};

#define DRAWING_DATA_LEN (sizeof(drawing_data) / sizeof(float))

#endif
