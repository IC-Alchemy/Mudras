// globals.h
#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

extern volatile bool clockInput;
extern uint8_t count;
extern uint8_t countSize;
extern int NoteMode;
extern uint8_t note;
extern int mm;
extern int raw_mm;
extern int dacU;
extern uint8_t sequence[64];
extern uint8_t sequenceLinear[64];

#define RESOLUTION_12_BIT 4096

const float semitones_per_octave = 12.0f;
const float volts_per_semitone = 1.0f / semitones_per_octave;
const float a4_frequency = 440.0f;
const uint8_t a4_midi_note = 69;

#endif