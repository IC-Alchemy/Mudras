#include "Adafruit_VL53L0X.h"
#include "myfunctions.h"
#include <ResponsiveAnalogRead.h>
#include <Wire.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
#define RESOLUTION_12_BIT 4096

ResponsiveAnalogRead noteSeqPot(A0, true, 0.1);
ResponsiveAnalogRead scaleKnob(A1, true, 0.1);
volatile bool clockInput = false;
uint8_t count = -1; // variable to use in the actual sequencer
uint8_t countSize = 3;

int NoteMode = 0;

uint8_t note = 0;
int mm = 0;
int raw_mm = 0;

float semitones_per_octave = 12.0f;
float volts_per_semitone = 1.0f / semitones_per_octave;
float a4_frequency = 440.0f;
uint8_t a4_midi_note = 69;

int dacU = 0;
uint8_t sequence[64];
uint8_t sequenceLinear[64];

//  DIY Module Calibration
//  Do Not Attempt to Calibrate Unless You Have a High Quality Multimeter
//  If the voltage at the 3rd Octave does not equal 3.000
//  then change fix to MultiMeterReading- 3.000

float fix = 0.000;

uint8_t NumberOfNotes;     //  Number of notes the sequencer uses
uint8_t NumScaleModes = 8; //   7 scales plus linear mode
int TopRange = 700;        //   top distane in mm of distance sensor

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

const char *scaleNames[] = {"Major",          "Minor",      "Minor Pentatonic",
                            "Harmonic Minor", "Diminished", "Whole Tone",
                            "Chromatic"};

const uint8_t scalePatterns[][12] = {
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
} //  The following is scaled for a Audio Potentiometer,  we were forced to use
//  one due to supply chain issues.  This lines up exactly with the official
//  Mudras, but if you are DIYing it makes sense to use a linear pot and simply
//  scale
void scaleMode() { // choose scales
  int scalemode = scaleKnob.getValue();

  if (scalemode < 18) {
    NoteMode = 0; // Major
    NumberOfNotes = 23;
  } else if (scalemode > 18 && scalemode <= 53) {
    NoteMode = 1; // minor
    NumberOfNotes = 23;
  } else if (scalemode >= 54 && scalemode <= 105) {
    NoteMode = 2;
    NumberOfNotes = 19; // pentatonic
  } else if (scalemode >= 106 && scalemode <= 180) {
    NoteMode = 3; // harmonic minor
    NumberOfNotes = 23;
  } else if (scalemode >= 181 && scalemode <= 227) {
    NoteMode = 4;
    NumberOfNotes = 29;
  } else if (scalemode >= 228 && scalemode <= 535) {
    NoteMode = 5; // whole tone
    NumberOfNotes = 20;
  } else if (scalemode >= 536 && scalemode <= 799) {
    NoteMode = 6; // chromatic
    NumberOfNotes = 35;
  } else if (scalemode >= 800) {
    NoteMode = 7; // linear mode
  }
}
//  The following is scaled for a Audio Potentiometer,  we were forced to use
//  one due to supply chain issues.  This lines up exactly with the official
//  Mudras, but if you are DIYing it makes sense to use a linear pot

void loopLength() {
  int seqLength = noteSeqPot.getValue();

  if (seqLength <= 9) {
    countSize = 1;
  } else if (seqLength >= 10 && seqLength <= 45) {
    countSize = 2;
  } else if (seqLength >= 46 && seqLength <= 120) {
    countSize = 3;
  } else if (seqLength >= 121 && seqLength <= 210) {
    countSize = 7;
  } else if (seqLength >= 211 && seqLength <= 360) {
    countSize = 15;
  } else if (seqLength >= 361 && seqLength <= 799) {
    countSize = 31;
  } else if (seqLength >= 800) {
    countSize = 63;
  }
}

void ReadKnobs() {
  noteSeqPot.update();
  scaleKnob.update();

  if (scaleKnob.hasChanged()) {
    scaleMode();
  }
  if (noteSeqPot.hasChanged()) {
    loopLength();
  }
}

uint32_t volts_to_DAC(float volts, uint32_t resolution = RESOLUTION_12_BIT,
                      float min_volt = 0.0f, float max_volt = 3.3f) {
  float volt_range = max_volt - min_volt;

  float normalized_value = (volts - min_volt) / volt_range;

  uint32_t dacUnit = ceil(normalized_value * (resolution - 1));

  return dacUnit;
}

float midi_note_to_volts(uint32_t midi_note) {

  return (midi_note)*volts_per_semitone;
}

// Function to play the sequence
void playSeq() {
  if (NoteMode != 7) {

    // Convert scaled distance reading note to volts
    float volts = midi_note_to_volts(fullScales[NoteMode][sequence[count]]);

    //  Convert Volts to DacUnit to feed to the Dac
    dacU = volts_to_DAC(volts, RESOLUTION_12_BIT, 0.0f, 3.3f + fix);
    // Write the DAC with the correct pitch from the sequence
    analogWrite(PA4, dacU);

  } else {
    // Get the linear sequence
    int linear = sequenceLinear[count];
    // Write the DAC with the correct pitch from the linear sequence
    analogWrite(PA4, linear);
  }
}

void recordSeq() {
  // Check if the hand position is out of range or below 50 mm
  if (mm > TopRange || mm < 50) {

    //  if hand is out of range record a 0
    mm = 0;
    sequence[count] = mm;
    playSeq();

    return;
  }

  // Check if the mode is linear
  if (NoteMode != 7) { //  Scale Mode
    // Map the hand position to the number of notes in the scale
    mm = map(mm, 50, TopRange, 0, NumberOfNotes);
    mm = constrain(mm, 0, NumberOfNotes);
    // Record the sequence
    sequence[count] = mm;

  } else //  Linear Mode

  {

    // Map the hand position to the range [0, 4095]
    raw_mm = map(mm, 50, TopRange, 0, 4095);
    // Constrain the mapped value within the range [50, 4095]
    raw_mm = constrain(raw_mm, 0, 4095);
    // Record the linear sequence
    sequenceLinear[count] = raw_mm;
  }

  playSeq();
}

//   This is the interupt code

void gate() {
  //  Rising Edge Detected on Clock input
  clockInput = true;
}

void reset() {
  //  Rising Edge Detected on Reset Input

  count = 0;
}

void readsensor() { mm = lox.readRange(); }

void setup() {

  generateFullScales();
  Wire.begin();
  Wire.setClock(400000);
  analogWriteResolution(12);
  analogWrite(PA4, 0); /// Set DAC OutPut to Zero

  pinMode(PB5, INPUT_PULLUP); //  PB5     TOUCH BUTTON
  pinMode(PB4, INPUT);        // clock  input
  pinMode(PD3, INPUT);        //  RESET

  //  interupts
  attachInterrupt(digitalPinToInterrupt(PB4), gate, FALLING);
  attachInterrupt(digitalPinToInterrupt(PD3), reset, FALLING);

  //  Start the distance sensor
  if (!lox.begin()) {

    while (10)
      ;
  }
  lox.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  lox.startRangeContinuous(42);

  noteSeqPot.setActivityThreshold(10);
  scaleKnob.setActivityThreshold(10);

  //  update pots so the module begins where the pots are currently
  noteSeqPot.update();
  scaleKnob.update();
  loopLength();
  scaleMode();
}

void incrementAndWrapCount(byte &count, byte maxSize) {
  count++;
  if (count > maxSize) {
    count = 0;
  }
}

void loop() {

  ReadKnobs();

  if (lox.isRangeComplete()) {
    readsensor();
  }

  if (clockInput) {
    //  reset the interupt flag
    clockInput = 0;

    //   If record button is held
    if (digitalRead(PB5) == LOW) {
      recordSeq();

    }

    else {
      playSeq();
    }
    // Increment the count and wrap it around
    incrementAndWrapCount(count, countSize);
  }
