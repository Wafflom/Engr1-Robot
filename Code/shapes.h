/*
 * shapes.h — Drawing data for concentric shapes
 *
 * Stores shape segment directions in PROGMEM so the main sketch
 * stays clean and new shapes can be added by defining new arrays.
 *
 * Each shape is an array of direction vectors (dx, dy) per segment.
 * The main sketch handles distance via encoder + PID control.
 *
 * Shape parameters:
 *   RADIUS_MM — circumscribed circle radius (shared by all shapes)
 */

#ifndef SHAPES_H
#define SHAPES_H

#include <avr/pgmspace.h>

// Shape size (mm) — shared by all shapes
#define RADIUS_MM     50.0f

// ============================================================
//  CIRCLE parameters
// ============================================================

// Circle is handled procedurally (continuous velocity sweep).
#define CIRCLE_TIMEOUT_MS  15000UL

// ============================================================
//  STAR (5-pointed pentagram)
//
//  Pentagram order: 0→2→4→1→3→0 (skip-1 vertices).
//  Vertex 0 at top (90°).
//  Segment length = 2·R·sin(72°) ≈ 1.902·R
// ============================================================

#define STAR_SEG_MM       (1.9021f * RADIUS_MM)
#define STAR_NUM_SIDES    5

const float star_segments[][2] PROGMEM = {
  { -0.5878f, -1.8090f },   // v0 → v2
  {  1.5388f,  1.1180f },   // v2 → v4
  { -1.9021f,  0.0000f },   // v4 → v1
  {  1.5388f, -1.1180f },   // v1 → v3
  { -0.5878f,  1.8090f },   // v3 → v0
};

// ============================================================
//  HEXAGON
//
//  Regular hexagon, side = circumradius = RADIUS_MM.
//  Vertex 0 at top (90°), drawn clockwise.
// ============================================================

#define HEX_SEG_MM        RADIUS_MM
#define HEX_NUM_SIDES     6

const float hex_segments[][2] PROGMEM = {
  {  0.8660f, -0.5000f },   // v0 → v1
  {  0.0000f, -1.0000f },   // v1 → v2
  { -0.8660f, -0.5000f },   // v2 → v3
  { -0.8660f,  0.5000f },   // v3 → v4
  {  0.0000f,  1.0000f },   // v4 → v5
  {  0.8660f,  0.5000f },   // v5 → v0
};

#endif
