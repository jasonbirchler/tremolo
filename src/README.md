# Stereo Tremolo

Stereo tremolo for the GuitarML FunBox v3 (Daisy Seed) platform.

## Controls

| Control | Description | Comment |
| --- | --- | --- |
| Knob 1 | Division | Quarter, dotted quarter, eighth, dotted eighth, sixteenth, dotted sixteenth. Stepped with hysteresis. |
| Knob 2 | Depth | 0 = dry, 1 = full tremolo (troughs mute). |
| Knob 3 | Shape | Discrete selection of sine, triangle, saw, reverse saw, square, random. Equal bands with hysteresis. |
| Knob 4 | Stereo spread | Moves the right LFO from 0 to 90 degrees out of phase (linear) and boosts the mid/side side level from parity to 1.5x the mid (logarithmic). |
| Knob 5 | Modulation amount | Depth of the second, independent LFO. |
| Knob 6 | LFO rate | Sets the second LFO rate relative to the tremolo rate: 32x slower at 0, 1x at noon, and 1.5x at 1, with geometric steps in between. |
| 3-Way Switch 1 | Second LFO waveform | Left = sine, center = saw, right = square. |
| 3-Way Switch 2 | Second LFO target | Left = depth, center = depth + rate, right = rate. |
| 3-Way Switch 3 | Unused | |
| DIP Switch 1 | Input mode | ON = mono (left input duplicated), OFF = stereo. |
| DIP Switch 2 | Output mode | ON = mono (L/R summed to both outputs), OFF = stereo. |
| DIP Switch 3 | Envelope input | ON = envelope retrigger and envelope LED blink enabled. |
| DIP Switch 4 | Effect selection | ON = phaser, OFF = vibrato. |
| FS 1 | Bypass | Toggles immediately on press. Release within 600 ms latches the new state; holding longer than 600 ms makes it momentary and restores the previous state on release. |
| FS 2 | Tap tempo | Updates the base tempo. |
| LED 1 | Effect active | Off while bypassed and not momentarily engaged. |
| LED 2 | Tempo | Blinks on each quarter note and pulses on each tap. |
| Audio In 1/2 | Left/right input | Mono mode uses input 1 only. |
| Audio Out 1/2 | Left/right output | Stereo. |
| MIDI In | MIDI clock | 24 PPQN (0xF8). MIDI clock wins while present; tap tempo resumes after 2 s without clock. |

## Behavior notes

- The second LFO rate follows the tremolo rate. Knob 6 scales it geometrically from 1/32 the tremolo rate at 0, through parity at noon, to 1.5x the tremolo rate at 1.
- Rate modulation is exponential: at full modulation amount the primary LFO rate sweeps between 0.5x and 2x (one octave down/up). The target is smoothed to avoid clicks.
- Bypass is a click-free wet/dry crossfade, not a relay. The wet path keeps running while bypassed.
- Tempo range is 30..300 BPM; the default is 120 BPM.
- Tempo changes and parameter changes are smoothed to avoid zipper noise.

## Build

The repository expects `libDaisy` and `DaisySP` submodules at the repository root. Run the top-level build to apply the FunBox hardware patch, build both libraries, and build the firmware:

```sh
make
```

Outputs: `build/tremolo.elf`, `build/tremolo.hex`, `build/tremolo.bin`.

Flash the Daisy Seed (put it in DFU mode first):

```sh
make program-dfu
```

## Host tests

```sh
make test
```

The test target compiles `src/tremolo_engine.cpp` and
`test/tremolo_engine_test.cpp` with the host compiler. The engine has no
libDaisy or DaisySP dependencies.

## Required FunBox hardware patch

`mod/daisy_petal.h` and `mod/daisy_petal.cpp` are the canonical FunBox patch. The top-level build synchronizes both into `libDaisy/src` and rebuilds the libraries automatically. Use `make patch-libdaisy` to synchronize only, or `make libs` to synchronize and build the libraries.

The patch adds the FunBox pin mapping, the six-knob layout, the switches, the MIDI UART on `D30`, and `DaisyPetal::InitMidi()`.
