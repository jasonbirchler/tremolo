#include "midi_control.h"

using namespace daisy;

MidiControl::MidiControl(MidiBridge& bridge) : bridge_(bridge) {}

void MidiControl::ApplyReleaseRequests()
{
    const uint32_t requests = bridge_.TakeReleaseRequests();

    for(int i = 0; i < MidiCcController::KNOB_COUNT; ++i)
        if(requests & (1u << i))
            cc_.ReleaseKnob(i);

    for(int i = 0; i < MidiCcController::SWITCH_COUNT; ++i)
        if(requests & (1u << (MidiCcController::KNOB_COUNT + i)))
            cc_.ReleaseSwitch(i);
}

void MidiControl::Poll(MidiUartHandler& midi,
                       TempoController& tempo,
                       uint32_t         nowMs)
{
    ApplyReleaseRequests();

    // Listen() also restarts the UART receiver if it stopped after an overrun.
    midi.Listen();
    while(midi.HasEvents())
    {
        MidiEvent event = midi.PopEvent();
        if(event.type == SystemRealTime)
        {
            if(event.srt_type == TimingClock)
                tempo.MidiClock(nowMs);
            continue;
        }

        if(event.type != ControlChange)
            continue;

        const ControlChangeEvent cc = event.AsControlChange();
        if(cc.control_number == MidiCcController::FOOTSWITCH_1_CC)
        {
            const bool wasPressed = cc_.FootswitchOverridden(0)
                                    && cc_.FootswitchPressed(0);
            cc_.ControlChange(cc.control_number, cc.value);
            if(!wasPressed && cc_.FootswitchPressed(0))
                bridge_.PostFootswitchPress(0);
        }
        else if(cc.control_number == MidiCcController::FOOTSWITCH_2_CC)
        {
            const bool wasPressed = cc_.FootswitchOverridden(1)
                                    && cc_.FootswitchPressed(1);
            cc_.ControlChange(cc.control_number, cc.value);
            if(!wasPressed && cc_.FootswitchPressed(1))
                bridge_.PostFootswitchPress(1);
        }
        else
        {
            cc_.ControlChange(cc.control_number, cc.value);
        }
    }

    bridge_.Publish(MidiSnapshot{cc_, tempo.Bpm(nowMs)});
}
