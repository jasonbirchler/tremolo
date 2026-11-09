#include "midi_mapper.h"
#include "pedal_constants.h"

#include <cmath>

MidiOverrideMapper::MidiOverrideMapper(MidiBridge& bridge) : bridge_(bridge) {}

float MidiOverrideMapper::ResolveKnob(const MidiCcController& controls,
                                      int                     index,
                                      float                   hardwareValue)
{
    // A release is in flight until the main loop publishes a snapshot without
    // the override. Until then the hardware value is used.
    if(knobReleasePending_[index])
    {
        if(!controls.KnobOverridden(index))
            knobReleasePending_[index] = false;
        return hardwareValue;
    }

    if(controls.KnobOverridden(index))
    {
        if(!knobHadOverride_[index])
        {
            lastHardwareKnobs_[index] = hardwareValue;
            knobHadOverride_[index]   = true;
        }
        else if(fabsf(hardwareValue - lastHardwareKnobs_[index])
                > kKnobPickupTolerance)
        {
            bridge_.RequestKnobRelease(index);
            knobHadOverride_[index]    = false;
            knobReleasePending_[index] = true;
            return hardwareValue;
        }
    }
    else
    {
        knobHadOverride_[index] = false;
        return hardwareValue;
    }

    return controls.KnobValue(index);
}

int MidiOverrideMapper::PhysicalSwitchPosition(bool left, bool right)
{
    if(left)
        return MidiCcController::SWITCH_LEFT;
    if(right)
        return MidiCcController::SWITCH_RIGHT;
    return MidiCcController::SWITCH_CENTER;
}

void MidiOverrideMapper::ResolveSwitch(const MidiCcController& controls,
                                       int                     index,
                                       bool&                   left,
                                       bool&                   right)
{
    const int physical = PhysicalSwitchPosition(left, right);

    if(switchReleasePending_[index])
    {
        if(!controls.SwitchOverridden(index))
            switchReleasePending_[index] = false;
        return;
    }

    if(!controls.SwitchOverridden(index))
    {
        switchHadOverride_[index] = false;
        return;
    }

    if(!switchHadOverride_[index])
    {
        lastPhysicalSwitches_[index] = physical;
        switchHadOverride_[index]    = true;
    }
    else if(physical != lastPhysicalSwitches_[index])
    {
        bridge_.RequestSwitchRelease(index);
        switchHadOverride_[index]    = false;
        switchReleasePending_[index] = true;
        return;
    }

    const int position = controls.SwitchPosition(index);
    left               = position == MidiCcController::SWITCH_LEFT;
    right              = position == MidiCcController::SWITCH_RIGHT;
}
