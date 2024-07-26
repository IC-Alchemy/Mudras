// Scales.cpp
#include "Scales.h"

const char *scaleNames[ScaleTypeCount] = {
    "Major",      "Minor",      "Minor Pentatonic", "Harmonic Minor",
    "Diminished", "Whole Tone", "Chromatic"};

const uint8_t scalePatterns[ScaleTypeCount][12] = {
    {0, 2, 4, 5, 7, 9, 11, 255},           // Major
    {0, 2, 3, 5, 7, 8, 10, 255},           // Minor
    {0, 3, 5, 7, 10, 255},                 // Minor Pentatonic
    {0, 2, 3, 5, 7, 8, 11, 255},           // Harmonic Minor
    {0, 1, 3, 4, 6, 7, 9, 10, 255},        // Diminished
    {0, 2, 4, 6, 8, 10, 255},              // Whole Tone
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11} // Chromatic
};

uint8_t fullScales[ScaleTypeCount][37];

void generateFullScales() {
  for (uint8_t scaleIndex = 0; scaleIndex < ScaleTypeCount; scaleIndex++) {
    uint8_t noteIndex = 0;
    uint8_t patternIndex = 0;
    uint8_t octave = 0;
    uint8_t octaveMultiplier = 0;

    while (noteIndex < 37) {
      if (scalePatterns[scaleIndex][patternIndex] == 255) {
        patternIndex = 0;
        octave++;
        octaveMultiplier = octave * 12;
        if (octave >= 4)
          break;
      }

      fullScales[scaleIndex][noteIndex] =
          scalePatterns[scaleIndex][patternIndex] + octaveMultiplier;
      noteIndex++;
      patternIndex++;
    }

    // Fill remaining slots with 36
    memset(&fullScales[scaleIndex][noteIndex], 36, 37 - noteIndex);
  }
}