/*
 * HAND-CONTROLLED CV SEQUENCER & DIRECT CV CONTROLLER
 * ====================================================
 * A dual-mode gestural music controller that converts hand position into CV
 * Automatically switches between sequencer mode and direct CV mode
 *
 * HARDWARE CONNECTIONS:
 * ---------------------
 * D4  (PA4) - Control button input (active LOW with internal pullup)
 * D5  (PA5) - External gate/trigger input for sequence advancement (falling edge triggered)
 * PA0       - Frequency/Loop Length knob (analog input, 0-1023)
 * PA1       - Scale selection knob (analog input, 0-1023)
 * PA4       - 12-bit DAC output for CV (0-3.3V control voltage for modular synths)
 * SDA/SCL   - I2C connection to VL53L0X distance sensor (hand tracking)
 *
 * TWO OPERATING MODES (automatically detected):
 * ----------------------------------------------
 *
 * MODE 1: SEQUENCER MODE (gate-driven)
 * - Active when gate signals are received on D5
 * - External clock advances through recorded sequence
 * - D4 button behavior:
 *   • PRESSED: RECORD mode - captures hand movements into sequence
 *   • RELEASED: PLAY mode - plays back recorded sequence
 * - PA0 knob sets loop length (2 to 64 steps)
 * - PA1 knob selects musical scale
 *
 * MODE 2: DIRECT CV MODE (button-driven)
 * - Active when no gate signals received for 3 seconds
 * - No sequencing - immediate hand-to-CV response
 * - D4 button behavior:
 *   • PRESSED: Live hand control - CV follows hand position in real-time
 *   • RELEASED: Sample & Hold - freezes CV at last hand position
 * - PA1 knob selects musical scale (same as sequencer mode)
 * - PA0 knob has no effect in this mode
 *
 * MODE SWITCHING:
 * ---------------
 * - Powers up in DIRECT CV MODE
 * - Automatically switches to SEQUENCER MODE when first gate received on D5
 * - Automatically switches back to DIRECT CV MODE after 3 seconds without gates
 * - Seamless transition - no manual switching required
 *
 * COMMON FEATURES (both modes):
 * -----------------------------
 * - Move hand over sensor (20-675mm range) to control pitch
 * - PA1 knob selects from 8 modes:
 *   0-6: Musical scales (Major, Minor, Pentatonic, etc.)
 *   7: Linear voltage mode (smooth CV sweep, no scale quantization)
 */

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// VL53L0X Time-of-Flight distance sensor object (measures hand position)
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// INTERRUPT SYSTEM: Flag set by gate ISR, processed in main loop
// volatile = can be modified by interrupt, prevents compiler optimization issues
volatile bool poo = false;  // True when gate trigger received on D5

#include <ResponsiveAnalogRead.h>
#define RESOLUTION_12_BIT 4096  // 12-bit DAC resolution (0-4095)

// Analog input handlers with noise filtering
ResponsiveAnalogRead freqKnob(PA0, true);   // Loop length control knob
ResponsiveAnalogRead scaleKnob(PA1, true);  // Musical scale selection knob

// SENSOR READINGS:
int raw_mm = 0;   // Raw distance in mm, mapped to 0-4095 for linear CV mode
int mm = 0;       // Distance mapped to 0-18 for scale note index

// SEQUENCER STATE:
int count2 = -1;      // Current step in sequence (0 to countSize), starts at -1 so first gate makes it 0
int countFlag = 0;    // Tracks mode transitions: 1=was in play mode, 0=was in record mode
int NoteMode = 0;     // Selected scale: 0-6 = musical scales, 7 = linear voltage mode


// MUSICAL SCALE DEFINITIONS:
// Each row is a different scale, containing MIDI note numbers (semitones from C0)
// Hand position (mm) is mapped to index 0-18, which looks up the note in the selected scale
// NoteMode (0-6) selects which scale row to use, NoteMode 7 = linear voltage mode (bypasses scales)
int scale[7][24] = {
    // Scale 0: Major scale (W-W-H-W-W-W-H pattern)
    // Intervals: Root, Maj2, Maj3, P4, P5, Maj6, Maj7, Octave...
    {0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 17, 19, 21, 23, 24, 26, 28, 29, 31, 33, 35, 36, 36, 36},

    // Scale 1: Harmonic Minor (W-H-W-W-H-Aug2-H pattern)
    // Intervals: Root, Maj2, min3, P4, P5, min6, Maj7, Octave...
    {0, 2, 3, 5, 7, 8, 11, 12, 14, 15, 17, 19, 20, 23, 24, 26, 27, 29, 31, 32, 35, 36, 36, 36},

    // Scale 2: Mixolydian mode (W-W-H-W-W-H-W pattern)
    // Like major but with flatted 7th - common in blues/rock
    {0, 2, 4, 5, 7, 9, 10, 12, 14, 16, 17, 19, 21, 22, 24, 26, 28, 29, 31, 33, 34, 36, 36, 36},

    // Scale 3: Minor Pentatonic (doubled notes for easier playing)
    // 5-note scale, each note appears twice for more forgiving hand positioning
    {0, 0, 3, 3, 5, 5, 7, 7, 10, 10, 12, 12, 15, 15, 17, 17, 19, 19, 22, 22, 24, 24, 27, 29},

    // Scale 4: Natural Minor (W-H-W-W-H-W-W pattern)
    // Intervals: Root, Maj2, min3, P4, P5, min6, min7, Octave...
    {0, 2, 3, 5, 7, 8, 10, 12, 14, 15, 17, 19, 20, 22, 24, 26, 27, 29, 31, 32, 34, 36, 36, 36},

    // Scale 5: Whole Tone scale (all whole steps)
    // Creates dreamy, ambiguous tonality - 6 notes per octave
    {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 38, 38, 38, 38},

    // Scale 6: Chromatic (all 12 semitones)
    // Every note in the octave - no scale quantization
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23}
};

int countSize = 3;  // Maximum step number in sequence (actual length = countSize + 1)
                    // Set by loopLength() function based on freqKnob position

// CV (CONTROL VOLTAGE) CONVERSION CONSTANTS:
// Standard modular synth CV: 1 Volt per Octave (1V/Oct)
float semitones_per_octave = 12.0f;                           // 12 semitones = 1 octave
float volts_per_semitone = 1.0f / semitones_per_octave;       // 0.0833V per semitone (1V/12)
float a4_frequency = 440.0f;                                  // A4 reference (not currently used)
uint32_t a4_midi_note = 69;                                   // MIDI note 69 = A4 (not currently used)

int dacU = 0;         // DAC output value (0-4095) calculated and sent to PA4
int sequence[64];     // Recorded sequence storage: holds note indices (0-18) or raw voltage values

/*
 * DAC VOLTAGE CONVERSION FUNCTION
 * ================================
 * Converts desired output voltage to 12-bit DAC value (0-4095)
 *
 * Parameters:
 *   volts       - Desired output voltage (typically 0-3.3V for CV)
 *   resolution  - DAC resolution (4096 for 12-bit)
 *   min_voltage - Minimum output voltage (0V)
 *   max_voltage - Maximum output voltage (3.3V for STM32)
 *
 * Returns: DAC register value (0-4095)
 *
 * Example: 1.65V input → returns 2048 (middle of 12-bit range)
 */
uint32_t volts_to_DAC(float volts, uint32_t resolution = RESOLUTION_12_BIT, float min_voltage = 0.0f, float max_voltage = 3.3f)
{
  float voltage_range = max_voltage - min_voltage;              // 3.3V range
  float normalized_value = (volts - min_voltage) / voltage_range; // 0.0 to 1.0
  uint32_t dacUnit = ceil(normalized_value * (resolution - 1));  // Scale to 0-4095
  return dacUnit;
}

/*
 * MIDI NOTE TO VOLTAGE CONVERSION
 * ================================
 * Converts MIDI note number to CV voltage using 1V/Octave standard
 *
 * MIDI note numbers are semitones from C-1 (note 0)
 * Each semitone = 1/12 volt = 0.0833V
 *
 * Examples:
 *   MIDI 0  (C-1)  → 0.00V
 *   MIDI 12 (C0)   → 1.00V
 *   MIDI 24 (C1)   → 2.00V
 *   MIDI 60 (C4)   → 5.00V (middle C)
 *
 * Note: This function outputs voltage, not DAC values
 *       Use volts_to_DAC() to convert the result for PA4 output
 */
float midi_note_to_volts(uint32_t midi_note)
{
  return midi_note * volts_per_semitone;  // note * (1V/12) = voltage
}

/*
 * INTERRUPT SERVICE ROUTINE (ISR)
 * ================================
 * Triggered by FALLING edge on pin D5 (external gate/trigger input)
 *
 * INTERRUPT FLOW:
 * 1. External clock/sequencer sends gate signal to D5
 * 2. When D5 goes HIGH→LOW (falling edge), this ISR fires immediately
 * 3. ISR sets 'poo' flag to true (volatile ensures visibility to main loop)
 * 4. ISR exits quickly (critical for interrupt safety)
 * 5. Main loop detects 'poo' flag and advances sequence step
 *
 * WHY THIS DESIGN:
 * - ISR kept minimal (just set flag) to avoid timing issues
 * - Actual sequence logic runs in main loop where it's safe to do complex operations
 * - Allows external sync with drum machines, modular sequencers, or MIDI clock
 *
 * MODE SWITCHING:
 * - Receiving gate signals automatically switches to sequencer mode
 * - If no gates received for GATE_TIMEOUT period, switches to direct CV mode
 *
 * Attached in setup() with: attachInterrupt(digitalPinToInterrupt(D5), gate, FALLING);
 */
void gate()
{
  poo = true;  // Signal main loop that a gate trigger was received
}

/*
 * DISTANCE SENSOR READING & PROCESSING
 * =====================================
 * Reads VL53L0X Time-of-Flight sensor and converts distance to musical parameters
 *
 * SENSOR RANGE: 20mm (close) to 675mm (far) - about 2 feet of hand travel
 *
 * TWO OUTPUT FORMATS:
 * 1. mm (0-18)      - For scale mode: index into scale arrays
 *                     Closer hand = lower notes, farther = higher notes
 *
 * 2. raw_mm (0-4095) - For linear mode (NoteMode 7): direct DAC voltage
 *                      Provides smooth voltage sweep without scale quantization
 *
 * Called from main loop when lox.isRangeComplete() returns true
 */
void readsensor()
{
  mm = lox.readRange();  // Read distance from VL53L0X sensor in millimeters

  // Reject out-of-range readings (sensor returns >675mm on errors/no target)
  if (mm > 675) {
    mm = 0;  // Default to closest position on error
  }

  // LINEAR MODE MAPPING (for NoteMode 7):
  // Map 20-650mm distance → 0-4095 DAC value (full 12-bit range)
  // This creates smooth voltage sweep from 0V to 3.3V
  raw_mm = mm;
  raw_mm = map(mm, 20, 650, 0, 4095);
  constrain(raw_mm, 0, 4095);  // Ensure within DAC range

  // SCALE MODE MAPPING (for NoteMode 0-6):
  // Map 20-675mm distance → 0-18 note index
  // This index looks up notes in the scale[NoteMode][index] array
  // Gives 19 possible notes across the hand travel range
  mm = map(mm, 20, 675, 0, 18);
  constrain(mm, 0, 18);  // Ensure within scale array bounds
}





/*
 * PLAY SEQUENCE FUNCTION
 * ======================
 * Outputs the stored sequence value at current step (count2) to DAC on PA4
 * Called when D4 button is released (playback mode)
 *
 * TWO PLAYBACK MODES:
 *
 * 1. LINEAR MODE (NoteMode == 7):
 *    - Reads raw DAC value directly from sequence[count2]
 *    - Outputs smooth voltage (no scale quantization)
 *    - sequence[] contains values 0-4095 (full DAC range)
 *
 * 2. SCALE MODES (NoteMode 0-6):
 *    - sequence[count2] contains note index (0-18)
 *    - Looks up MIDI note: scale[NoteMode][sequence[count2]]
 *    - Converts MIDI note → voltage → DAC value
 *    - Outputs quantized to selected musical scale
 *
 * OUTPUT: Writes to PA4 (12-bit DAC) creating 0-3.3V CV signal
 */
void playSeq()
{
  if (NoteMode == 7)  // LINEAR MODE: Direct voltage output
  {
    dacU = sequence[count2];  // Get stored DAC value (0-4095)
    analogWrite(PA4, dacU);   // Output to CV (bypasses scale conversion)
  }
  else  // SCALE MODES 0-6: Quantized musical output
  {
    // Lookup chain: sequence[step] → scale index → MIDI note → voltage → DAC value
    float volts = midi_note_to_volts(scale[NoteMode][sequence[count2]]);
    dacU = volts_to_DAC(volts, RESOLUTION_12_BIT, 0.0f, 3.3f);
    analogWrite(PA4, dacU);  // Output quantized CV to modular synth
  }
}

/*
 * RECORD SEQUENCE FUNCTION
 * ========================
 * Captures current hand position into sequence array at current step (count2)
 * Called when D4 button is held down (record mode)
 *
 * RECORDING BEHAVIOR:
 * - Stores hand position into sequence[count2]
 * - Immediately calls playSeq() so you hear what you're recording
 * - Each gate trigger on D5 advances to next step and records new position
 *
 * TWO RECORDING MODES:
 *
 * 1. LINEAR MODE (NoteMode == 7):
 *    - Stores raw_mm (0-4095) directly into sequence
 *    - Records smooth voltage values for CV modulation
 *
 * 2. SCALE MODES (NoteMode 0-6):
 *    - Stores mm (0-18) as note index into sequence
 *    - Records quantized to selected musical scale
 *    - Later playback looks up actual MIDI notes from scale array
 */
void recordSeq()
{
  if (NoteMode == 7)  // LINEAR MODE: Record raw voltage
  {
    sequence[count2] = raw_mm;  // Store DAC value (0-4095) for smooth CV
    playSeq();  // Play back immediately (monitoring while recording)
  }
  else  // SCALE MODES 0-6: Record note index
  {
    sequence[count2] = mm;  // Store scale index (0-18) for quantized notes
    playSeq();  // Play back immediately (monitoring while recording)
  }
}




/*
 * KNOB HANDLER (called every loop iteration)
 * ===========================================
 * Reads both analog knobs and processes changes
 * Uses ResponsiveAnalogRead library for noise filtering and change detection
 */
void knobs()
{
  freqKnob.update();   // Read PA0 (loop length knob)
  scaleKnob.update();  // Read PA1 (scale selection knob)

  // Only process when knob values actually change (reduces CPU load)
  if (scaleKnob.hasChanged())
  {
    scaleMode();  // Update which musical scale is active
  }
  if (freqKnob.hasChanged())
  {
    loopLength();  // Update sequence loop length
  }
}

/*
 * SCALE MODE SELECTION
 * ====================
 * Maps PA1 knob position (0-1023) to NoteMode (0-7)
 *
 * SCALE SELECTION:
 * NoteMode 0 = Major
 * NoteMode 1 = Harmonic Minor
 * NoteMode 2 = Mixolydian
 * NoteMode 3 = Minor Pentatonic (doubled)
 * NoteMode 4 = Natural Minor
 * NoteMode 5 = Whole Tone
 * NoteMode 6 = Chromatic
 * NoteMode 7 = LINEAR (no scale, direct voltage control)
 */
void scaleMode()
{
  int scalemode = scaleKnob.getValue();  // Read knob (0-1023)
  NoteMode = map(scalemode, 0, 1023, 0, 7);  // Map to 8 modes (0-7)
}

/*
 * LOOP LENGTH SELECTION
 * =====================
 * Maps PA0 knob position (0-1023) to sequence length
 *
 * SEQUENCE LENGTHS:
 * countSize = maximum step index (actual length = countSize + 1)
 *
 * Knob Position → countSize → Actual Steps:
 *   Minimum     →     1     →   2 steps
 *   Low         →     2     →   3 steps
 *   Med-Low     →     3     →   4 steps
 *   Medium      →     7     →   8 steps
 *   Med-High    →    15     →  16 steps
 *   High        →    31     →  32 steps
 *   Maximum     →    63     →  64 steps
 *
 * Sequence wraps: when count2 > countSize, it resets to 0
 */
void loopLength()
{
  int seqLength = freqKnob.getValue();  // Read knob (0-1023)

  // Map to 7 discrete length options (0-6)
  int range = map(seqLength, 0, 1023, 0, 6);

  // Set maximum step index based on knob position
  switch (range)
  {
  case 0:
    countSize = 1;   // 2-step sequence
    break;
  case 1:
    countSize = 2;   // 3-step sequence
    break;
  case 2:
    countSize = 3;   // 4-step sequence (default on startup)
    break;
  case 3:
    countSize = 7;   // 8-step sequence
    break;
  case 4:
    countSize = 15;  // 16-step sequence
    break;
  case 5:
    countSize = 31;  // 32-step sequence
    break;
  case 6:
    countSize = 63;  // 64-step sequence (maximum)
    break;
  }
}

/*
 * SETUP - HARDWARE INITIALIZATION
 * ================================
 * Runs once at power-up to configure all hardware peripherals
 */
void setup()
{
  // I2C INITIALIZATION (for VL53L0X distance sensor)
  Wire.begin();
  Wire.setClock(400000);  // 400 kHz I2C (fast mode) for quicker sensor readings

  // DAC CONFIGURATION
  analogWriteResolution(12);  // Set PA4 DAC to 12-bit resolution (0-4095)
                              // Provides 0.8mV steps across 3.3V range

  // DIGITAL INPUT PINS:
  pinMode(D4, INPUT_PULLUP);  // Record button (active LOW, internal pullup resistor)
                              // Pressed = LOW, Released = HIGH

  pinMode(D5, INPUT);         // External gate/trigger input (no pullup)
                              // Expects external clock signal from sequencer/drum machine

  // INTERRUPT ATTACHMENT:
  // This allows external clock to advance sequence steps in sync with other gear
  attachInterrupt(digitalPinToInterrupt(D5), gate, FALLING);

  // VL53L0X SENSOR INITIALIZATION:
  if (!lox.begin())  // Initialize sensor, returns false on failure
  {
    while (1);  
      
  }

  // Main loop checks lox.isRangeComplete() to know when new reading is ready
  lox.startRangeContinuous(); // Start continuous ranging mode (sensor constantly measures distance)  

  // INITIAL CONFIGURATION:
  loopLength();  // Read freqKnob and set initial sequence length (default countSize = 3)


}



/*
 * MAIN LOOP -
 * ====================================
 * Continuously runs to handle sensor reading, knob changes, and CV output
 */


void loop()
{
  // CONTINUOUS TASKS (run every loop iteration):

  knobs();  // Check for knob changes (scale selection, loop length)

  // Read distance sensor only when new measurement is ready
  // (non-blocking - doesn't wait for sensor)
  if (lox.isRangeComplete()) {
    readsensor();  // Update mm (0-18) and raw_mm (0-4095) from hand position
  }



    // GATE TRIGGER PROCESSING:
    // Check if interrupt flag was set by gate() ISR (external trigger on D5)
    if (poo)
    {
      poo = false;  // Clear the interrupt flag

      // ADVANCE SEQUENCE STEP:
      count2++;  // Move to next step in sequence

      // LOOP WRAPAROUND:
      // When we exceed the loop length, wrap back to step 0
      if (count2 > countSize)
      {
        count2 = 0;  // Reset to beginning of sequence
      }

      // MODE SELECTION: Record vs. Playback
      // Determined by D4 button state (INPUT_PULLUP: pressed=LOW, released=HIGH)

      if (digitalRead(D4) == LOW)  // RECORD MODE: Button is pressed
      {
        // TRANSITION DETECTION:
        // If we just switched from play mode to record mode, reset sequence position
        // countFlag tracks previous mode: 1=was playing, 0=was recording
        if (countFlag == 1)
        {
          count2 = 0;      // Start recording from step 0
          countFlag = 0;   // Mark that we're now in record mode
        }

        // RECORD CURRENT HAND POSITION:
        // Stores current sensor reading into sequence[count2]
        // Also plays back immediately so you hear what you're recording
        recordSeq();
      }
      else  // PLAYBACK MODE: Button is released
      {
        countFlag = 1;  // Mark that we're in play mode (for transition detection)

        // PLAY STORED SEQUENCE:
        // Outputs sequence[count2] to DAC on PA4
        // Creates CV voltage for modular synth
        playSeq();
      }
    }
  
  
}
