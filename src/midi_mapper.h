#pragma once
#ifndef TREMOLO_MIDI_MAPPER_H
#define TREMOLO_MIDI_MAPPER_H

#include "midi_bridge.h"
#include "tremolo_engine.h"

// Audio-thread side of MIDI overrides. Applies the snapshot's CC values to the
// knobs and switches, and detects when a physical control has been moved so
// that the override is released. Releases are requested through the bridge
// and applied by the main loop.
class MidiOverrideMapper
{
  public:
    explicit MidiOverrideMapper(MidiBridge& bridge);

    float ResolveKnob(const MidiCcController& controls,
                      int                     index,
                      float                   hardwareValue);

    void ResolveSwitch(const MidiCcController& controls,
                       int                     index,
                       bool&                   left,
                       bool&                   right);

  private:
    static int PhysicalSwitchPosition(bool left, bool right);

    MidiBridge& bridge_;

    float lastHardwareKnobs_[MidiCcController::KNOB_COUNT]      = {};
    bool  knobHadOverride_[MidiCcController::KNOB_COUNT]        = {};
    bool  knobReleasePending_[MidiCcController::KNOB_COUNT]     = {};

    int   lastPhysicalSwitches_[MidiCcController::SWITCH_COUNT] = {};
    bool  switchHadOverride_[MidiCcController::SWITCH_COUNT]    = {};
    bool  switchReleasePending_[MidiCcController::SWITCH_COUNT] = {};
};

#endif
