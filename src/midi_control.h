#pragma once
#ifndef TREMOLO_MIDI_CONTROL_H
#define TREMOLO_MIDI_CONTROL_H

#include "daisy_petal.h"
#include "midi_bridge.h"
#include "tremolo_engine.h"

#include <cstdint>

// Main-loop side of MIDI input. Drains the UART queue, applies CC values,
// forwards clock ticks to the tempo source, and publishes a snapshot for the
// audio thread. Must never be called from the audio callback.
class MidiControl
{
  public:
    explicit MidiControl(MidiBridge& bridge);

    void Poll(daisy::MidiUartHandler& midi,
              TempoController&        tempo,
              uint32_t                nowMs);

  private:
    void ApplyReleaseRequests();

    MidiBridge&      bridge_;
    MidiCcController cc_;
};

#endif
