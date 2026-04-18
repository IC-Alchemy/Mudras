# Mudras

Hand-controlled CV sequencer firmware for [`mudras.ino`](mudras.ino).

## Overview

Mudras is a gestural sequencer for modular and CV-based instruments. It uses a VL53L0X time-of-flight sensor to track hand position, converts that motion into either quantized pitch CV or smooth linear control voltage, and records/playbacks sequences in sync with an external gate or clock.

## Features

- Hand-position control with a VL53L0X distance sensor
- External clock/gate input for step advancement
- Record and playback modes selected with a button
- Variable loop lengths from 2 to 64 steps
- 7 quantized musical scales plus 1 linear CV mode
- 12-bit DAC CV output on `PA4`

## Hardware Connections

Mudras Uses a STM32G431, but it will work and this code is compatible with any STM32 Chip that has a built in DAC

| Pin | Function |
| --- | --- |
| `D4` | Record/play button input (`INPUT_PULLUP`, active low) |
| `D5` | External gate/trigger input, falling-edge interrupt |
| `PA0` | Loop length knob |
| `PA1` | Scale selection knob |
| `PA4` | 12-bit DAC CV output |
| `SDA/SCL` | I2C connection to VL53L0X sensor |

## How It Works

### Clocking

- An external trigger on `D5` advances the sequencer.
- The interrupt service routine only sets a flag.
- Step advance, record, and playback logic are handled in the main loop.

### Recording and Playback

- Hold the `D4` button to record.
- Release the `D4` button to play back the stored sequence.
- Entering record mode from playback resets recording to step 0.

### Sensor Mapping

The distance sensor is read continuously:

- In scale modes, hand distance is mapped to a note index.
- In linear mode, hand distance is mapped directly to the full 12-bit DAC range.

## Scale Modes

The scale knob selects one of 8 modes:

| Mode | Behavior |
| --- | --- |
| 0 | Major |
| 1 | Dorian |
| 2 | Minor Pentatonic |
| 3 | Harmonic Minor |
| 4 | Diminished |
| 5 | Whole Tone |
| 6 | Chromatic |
| 7 | Linear CV (no quantization) |

## Loop Lengths

The loop length knob selects one of these sequence lengths:

- 2 steps
- 3 steps
- 4 steps
- 8 steps
- 16 steps
- 32 steps
- 64 steps

## Signal Path

### Scale Modes

1. Hand distance is mapped to a scale index.
2. The scale table converts that index to a note.
3. The note is converted to volts using 1V/octave.
4. Voltage is converted to a 12-bit DAC value.
5. The DAC outputs CV on `PA4`.

### Linear Mode

1. Hand distance is mapped directly to `0-4095`.
2. The raw DAC value is recorded or played back.
3. The DAC outputs smooth unquantized CV on `PA4`.

## Dependencies

This sketch depends on:

- `Wire`
- `Adafruit_VL53L0X`
- `ResponsiveAnalogRead`

## Firmware Notes

- DAC output uses 12-bit resolution.
- I2C is configured for 400 kHz.
- The sequencer stores up to 64 steps in memory.
- The external gate input is expected to be level-shifted/protected before reaching `D5`.

## Usage Summary

1. Power the module and connect the VL53L0X sensor.
2. Feed an external clock or gate into `D5`.
3. Set loop length with the knob on `PA0`.
4. Choose a scale or linear mode with the knob on `PA1`.
5. Hold `D4` and move your hand over the sensor to record steps.
6. Release `D4` to play the sequence back.

## File

- Main firmware: [`mudras.ino`](mudras.ino)
