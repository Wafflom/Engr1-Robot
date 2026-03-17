/*
 * drawing_hello.h — G-code drawing file
 *
 * Draws "HELLO" in squared block letters, 50mm tall.
 * No curves — all straight lines only.
 *
 * Letters are ~30mm wide with 10mm spacing between them.
 * Total width: ~190mm, height: 50mm
 *
 * To use this drawing, copy it over drawing.h:
 *   cp drawing_hello.h drawing.h
 *
 * CURRENT DRAWING: HELLO (block letters, 50mm tall)
 */

#ifndef DRAWING_H
#define DRAWING_H

#include <avr/pgmspace.h>

const char gcode_program[] PROGMEM =
  // --- HELLO in block letters (50mm tall) ---
  // All coordinates in mm, origin = bottom-left of H

  // === H (x: 0-30, y: 0-50) ===
  // Left vertical stroke
  "G0 X0 Y0\n"
  "G1 X0 Y50\n"
  // Crossbar
  "G0 X0 Y25\n"
  "G1 X30 Y25\n"
  // Right vertical stroke
  "G0 X30 Y50\n"
  "G1 X30 Y0\n"

  // === E (x: 40-70, y: 0-50) ===
  // Left vertical stroke
  "G0 X40 Y0\n"
  "G1 X40 Y50\n"
  // Top horizontal
  "G1 X70 Y50\n"
  // Middle horizontal
  "G0 X40 Y25\n"
  "G1 X65 Y25\n"
  // Bottom horizontal
  "G0 X40 Y0\n"
  "G1 X70 Y0\n"

  // === L (x: 80-110, y: 0-50) ===
  // Vertical stroke
  "G0 X80 Y50\n"
  "G1 X80 Y0\n"
  // Bottom horizontal
  "G1 X110 Y0\n"

  // === L (x: 120-150, y: 0-50) ===
  // Vertical stroke
  "G0 X120 Y50\n"
  "G1 X120 Y0\n"
  // Bottom horizontal
  "G1 X150 Y0\n"

  // === O (x: 160-190, y: 0-50) ===
  // Rectangle (squared, no curves)
  "G0 X160 Y0\n"
  "G1 X190 Y0\n"
  "G1 X190 Y50\n"
  "G1 X160 Y50\n"
  "G1 X160 Y0\n"

  // Return to origin
  "G0 X0 Y0\n"
;

#endif
