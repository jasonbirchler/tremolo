#pragma once
#ifndef TREMOLO_PEDAL_H
#define TREMOLO_PEDAL_H

#include "calibration_state.h"
#include "daisy_petal.h"
#include "daisysp.h"
#include "led_controller.h"
#include "midi_bridge.h"
#include "midi_control.h"
#include "midi_mapper.h"
#include "tremolo_engine.h"
#include "vibrato.h"

#include <cstddef>
#include <cstdint>

// Top-level pedal. ProcessBlock() runs in the audio callback and must not
// touch MIDI hardware. ServiceBackground() runs in the main loop and owns MIDI
// ingest, tap tempo, and calibration persistence.
class Pedal
{
  public:
    explicit Pedal(daisy::DaisyPetal& hw);

    void Init(float samplerate);

    void ProcessBlock(daisy::AudioHandle::InputBuffer  in,
                      daisy::AudioHandle::OutputBuffer out,
                      size_t                           size);

    void ServiceBackground(uint32_t nowMs);

  private:
    void ReadFootswitches(uint32_t nowMs);
    void ReadSwitches(const MidiCcController& midi);
    void ToggleCalibration(uint32_t nowMs);
    void ApplyEngineControls(const MidiCcController& midi);
    void ApplyMidiPresses(const MidiFootswitchPresses& presses, uint32_t nowMs);
    void UpdateModulation(float bpm);
    void RenderSamples(daisy::AudioHandle::InputBuffer  in,
                       daisy::AudioHandle::OutputBuffer out,
                       size_t                           size,
                       uint32_t                         nowMs);
    void UpdateLeds(uint32_t nowMs);

    daisy::DaisyPetal& hw_;

    CalibrationState  calibration_;
    LedController     leds_;
    MidiBridge        bridge_;
    MidiControl       midi_;
    MidiOverrideMapper midiMapper_;

    TremoloEngine   engine_;
    TempoController tempo_;
    daisysp::Metro  quarterNoteMetro_;
    daisysp::Phaser phaser_[2];
    Vibrato         vibrato_[2];
    daisy::Parameter knobs_[6];

    float effectMix_            = 0.0f;
    float effectMixCoefficient_ = 0.0f;
    int   lastEffectOrder_      = TremoloEngine::EFFECT_ORDER_EFFECT_FIRST;

    bool     bypassed_              = true;
    uint32_t bypassPressMs_         = 0;
    bool     fs1PressToggledBypass_ = false;

    bool     bothFootswitchesHeld_        = false;
    uint32_t bothFootswitchesHoldStartMs_ = 0;
    bool     bothHoldConsumed_            = false;
    bool     dualHoldGestureActive_       = false;

    bool stereo_              = false;
    bool monoOutput_          = false;
    bool retriggerEnabled_    = false;
    bool phaserMode_          = false;
    bool envelopeBlinkEnabled_ = false;
};

#endif
