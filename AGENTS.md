# Tremolo Guitar Effect Pedal

This project defines a tremolo effect pedal with calibration, save-error handling, and LED feedback using the DaisyPetal hardware platform.

## Features

- **Tremolo Effect**: Modulates amplitude in a periodic wave pattern
- **Modulation LFO**: Modulates the amplitude, rate or both of the tremolo
- **Calibration**: Stores user settings with checksum validation
- **Error Handling**: Visual feedback for EEPROM write failures
- **LED Feedback**: Calibrates, save, and quarter-note indicator LEDs
- **MIDI Support**: When present MIDI Clock sets the tremolo rate. All knobs, footswitches and toggle switches controllable via MIDI CC. NOTE: that the DIP switches are not MIDI controllable

## Coding Patterns

- If libDaisy or DaisySP has a class that solves the task at hand, always prefer those over creating custom code. This includes, but is not limited to oscillators, ADSR envelopes, MIDI control, and DSP effects like phaser and reverb.
- Only once it is confirmed that the available libraries do not have classes that can achieve the task at-hand should a custom implementation be written.
