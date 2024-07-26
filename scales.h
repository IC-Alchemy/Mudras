// Scales.h

#ifndef SCALES_H
#define SCALES_H

#include <Arduino.h>

enum ScaleType {
  Major,
  Minor,
  MinorPentatonic,
  HarmonicMinor,
  Diminished,
  WholeTone,
  Chromatic,
  ScaleTypeCount
};

extern const char *scaleNames[ScaleTypeCount];
extern const uint8_t scalePatterns[ScaleTypeCount][12];
extern uint8_t fullScales[ScaleTypeCount][37];

void generateFullScales();

#endif