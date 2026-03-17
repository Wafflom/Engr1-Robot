/*
 * shapes.h — Smiley face drawing data
 *
 * All coordinates in mm, relative to face center.
 * RADIUS_MM is the outer circle radius.
 */

#ifndef SHAPES_H
#define SHAPES_H

#include <avr/pgmspace.h>

// Outer circle radius (mm)
#define RADIUS_MM         50.0f
#define CIRCLE_TIMEOUT_MS 15000UL

// ---- SMILEY FACE PARAMETERS ----

// Eyes: small circles drawn procedurally
#define EYE_RADIUS_MM     5.0f
#define EYE_OFFSET_X      15.0f   // left/right from center
#define EYE_OFFSET_Y      12.0f   // above center
#define EYE_TIMEOUT_MS    5000UL

// Mouth: arc from left to right, curving downward
// Arc on circle centered at (0, 2), radius 20mm, from 210° to 330°
#define MOUTH_NUM_PTS     9

const float mouth_pts[][2] PROGMEM = {
  { -17.32f,  -8.00f },   // 210°
  { -14.14f, -12.14f },   // 225°
  { -10.00f, -15.32f },   // 240°
  {  -5.18f, -17.32f },   // 255°
  {   0.00f, -18.00f },   // 270°
  {   5.18f, -17.32f },   // 285°
  {  10.00f, -15.32f },   // 300°
  {  14.14f, -12.14f },   // 315°
  {  17.32f,  -8.00f },   // 330°
};

#endif
