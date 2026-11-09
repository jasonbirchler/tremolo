#include "pedal.h"
#include "funbox.h"
#include "pedal_constants.h"

#include <cmath>

using namespace daisy;
using namespace daisysp;
using namespace funbox;

namespace
{
constexpr float    kEffectMixSmoothingSeconds = 0.005f;
constexpr float    kEffectMixSilentLevel      = 1.0e-4f;
constexpr float    kMaxVibratoDepth           = 0.2f;
constexpr uint32_t kBypassMomentaryMs         = 500;
constexpr float    kSpreadCurve               = 3.0f;

// Exponential mapping for the Spread (Phase Offset) knob. Control is subtle
// near zero and becomes more rapid toward full scale.
float SpreadFromKnob(float knob)
{
    if(knob <= 0.0f)
        return 0.0f;
    if(knob >= 1.0f)
        return 1.0f;
    return (expf(kSpreadCurve * knob) - 1.0f) / (expf(kSpreadCurve) - 1.0f);
}

// Mono fold-down: the mid component. Averaging keeps correlated content at
// unity gain instead of the +6 dB a raw L+R sum would produce.
float SumToMono(float left, float right)
{
    return (left + right) * 0.5f;
}

template <typename Effect>
void ProcessEffect(Effect& effectL, Effect& effectR, TremoloEngine& engine,
                   bool effectFirst, float effectMix, float inL, float inR,
                   float* outL, float* outR)
{
    if(effectFirst)
    {
        const float effectedL = effectL.Process(inL);
        const float effectedR = effectR.Process(inR);
        engine.Process(inL + (effectedL - inL) * effectMix,
                       inR + (effectedR - inR) * effectMix,
                       outL,
                       outR);
        return;
    }

    float tremL = 0.0f;
    float tremR = 0.0f;
    engine.Process(inL, inR, &tremL, &tremR);

    const float effectedL = effectL.Process(tremL);
    const float effectedR = effectR.Process(tremR);
    *outL = tremL + (effectedL - tremL) * effectMix;
    *outR = tremR + (effectedR - tremR) * effectMix;
}
} // namespace

Pedal::Pedal(DaisyPetal& hw)
    : hw_(hw),
      calibration_(hw.seed.qspi),
      midi_(bridge_),
      midiMapper_(bridge_)
{
}

void Pedal::Init(float samplerate)
{
    calibration_.Load();

    quarterNoteMetro_.Init(tempo_.Bpm(System::GetNow()) / 60.0f, samplerate);
    effectMixCoefficient_
        = 1.0f - expf(-1.0f / (kEffectMixSmoothingSeconds * samplerate));

    knobs_[0].Init(hw_.knob[Funbox::KNOB_1], 0.0f, 1.0f, Parameter::LINEAR);
    knobs_[1].Init(hw_.knob[Funbox::KNOB_2], 0.0f, 1.0f, Parameter::LINEAR);
    knobs_[2].Init(hw_.knob[Funbox::KNOB_3], 0.0f, 1.0f, Parameter::LINEAR);
    knobs_[3].Init(hw_.knob[Funbox::KNOB_4], 0.0f, 1.0f, Parameter::LINEAR);
    knobs_[4].Init(hw_.knob[Funbox::KNOB_5], 0.0f, 1.0f, Parameter::LINEAR);
    knobs_[5].Init(hw_.knob[Funbox::KNOB_6], 0.0f, 1.0f, Parameter::LINEAR);

    leds_.Init(hw_.seed.GetPin(Funbox::LED_1), hw_.seed.GetPin(Funbox::LED_2));

    engine_.Init(samplerate);
    engine_.SetBypassed(true);
    engine_.SetGainCompensation(calibration_.ActiveGain());

    for(size_t channel = 0; channel < 2; ++channel)
    {
        phaser_[channel].Init(samplerate);
        phaser_[channel].SetLfoDepth(0.5f);
        phaser_[channel].SetLfoFreq(engine_.GetModRate());
        phaser_[channel].SetPoles(4);
        phaser_[channel].SetFeedback(0.5f);

        vibrato_[channel].Init(samplerate);
        vibrato_[channel].SetLfoDepth(kMaxVibratoDepth);
        vibrato_[channel].SetLfoFreq(engine_.GetModRate());
    }
}

void Pedal::ServiceBackground(uint32_t nowMs)
{
    // Taps requested by the audio thread are applied here, so the tempo
    // source is only ever touched from the main loop.
    const uint32_t taps = bridge_.TakeTapRequests();
    for(uint32_t i = 0; i < taps; ++i)
        tempo_.Tap(nowMs);

    midi_.Poll(hw_.midi, tempo_, nowMs);

    if(calibration_.ServicePersistence(nowMs)
       == CalibrationState::PersistStatus::Failed)
        leds_.StartSaveErrorFlash(nowMs);
}

void Pedal::ProcessBlock(AudioHandle::InputBuffer  in,
                         AudioHandle::OutputBuffer out,
                         size_t                    size)
{
    hw_.ProcessAllControls();

    const uint32_t      now  = System::GetNow();
    const MidiSnapshot& midi = bridge_.ReadSnapshot();

    ReadFootswitches(now);
    ReadSwitches(midi.controls);
    ApplyEngineControls(midi.controls);
    ApplyMidiPresses(bridge_.TakeFootswitchPresses(), now);

    UpdateModulation(midi.bpm);
    RenderSamples(in, out, size, now);
    UpdateLeds(now);
}

void Pedal::ReadFootswitches(uint32_t now)
{
    Switch& fs1 = hw_.switches[Funbox::FOOTSWITCH_1];
    Switch& fs2 = hw_.switches[Funbox::FOOTSWITCH_2];

    const bool fs1Down = fs1.Pressed();
    const bool fs2Down = fs2.Pressed();

    // Holding both footswitches for the calibration time toggles the mode.
    if(fs1Down && fs2Down)
    {
        if(!bothFootswitchesHeld_)
        {
            bothFootswitchesHeld_        = true;
            bothFootswitchesHoldStartMs_ = now;
            bothHoldConsumed_            = false;
            dualHoldGestureActive_       = true;

            // If footswitch 1 was pressed before footswitch 2, its bypass
            // toggle already fired. Undo it so the hold is silent.
            if(fs1PressToggledBypass_)
            {
                bypassed_              = !bypassed_;
                fs1PressToggledBypass_ = false;
            }
        }
        else if(!bothHoldConsumed_
                && (now - bothFootswitchesHoldStartMs_) >= kCalibrationHoldMs)
        {
            bothHoldConsumed_ = true;
            ToggleCalibration(now);
        }
        return;
    }

    bothFootswitchesHeld_ = false;
    bothHoldConsumed_     = false;

    // Ignore every release edge belonging to a dual-footswitch gesture until
    // both switches are up, so leaving calibration cannot toggle bypass.
    if(dualHoldGestureActive_)
    {
        if(!fs1Down && !fs2Down)
            dualHoldGestureActive_ = false;
        return;
    }

    if(calibration_.Active())
        return;

    if(fs1.RisingEdge())
    {
        bypassPressMs_         = now;
        bypassed_              = !bypassed_;
        fs1PressToggledBypass_ = true;
    }

    if(fs1.FallingEdge())
    {
        // A long hold is momentary: undo the press toggle.
        if(now - bypassPressMs_ > kBypassMomentaryMs)
            bypassed_ = !bypassed_;
        fs1PressToggledBypass_ = false;
    }

    if(fs2.FallingEdge())
    {
        bridge_.RequestTap();
        leds_.PulseRight(now);
    }
}

void Pedal::ToggleCalibration(uint32_t nowMs)
{
    const bool entering = calibration_.Toggle();
    leds_.ResetEnvelopeTracking(engine_.GetEnvelope()
                                >= calibration_.ActiveThreshold());

    if(entering)
        leds_.StartCalibrationFlash(nowMs);
}

void Pedal::ReadSwitches(const MidiCcController& midi)
{
    // 3-way Switch 1: left = sine, center = sawtooth, right = reverse sawtooth.
    bool s1Left  = hw_.switches[Funbox::SWITCH_1_LEFT].Pressed();
    bool s1Right = hw_.switches[Funbox::SWITCH_1_RIGHT].Pressed();
    midiMapper_.ResolveSwitch(midi, 0, s1Left, s1Right);
    if(s1Left)
        engine_.SetModWaveform(TremoloEngine::MOD_SINE);
    else if(s1Right)
        engine_.SetModWaveform(TremoloEngine::MOD_REVERSE_SAW);
    else
        engine_.SetModWaveform(TremoloEngine::MOD_SAW);

    // 3-way Switch 2: left = depth, center = depth + rate, right = rate.
    bool s2Left  = hw_.switches[Funbox::SWITCH_2_LEFT].Pressed();
    bool s2Right = hw_.switches[Funbox::SWITCH_2_RIGHT].Pressed();
    midiMapper_.ResolveSwitch(midi, 1, s2Left, s2Right);
    if(s2Left)
        engine_.SetModTarget(TremoloEngine::TARGET_DEPTH);
    else if(s2Right)
        engine_.SetModTarget(TremoloEngine::TARGET_RATE);
    else
        engine_.SetModTarget(TremoloEngine::TARGET_DEPTH_RATE);

    // 3-way Switch 3 selects effect order: left = effect first, center = off,
    // right = tremolo first.
    bool s3Left  = hw_.switches[Funbox::SWITCH_3_LEFT].Pressed();
    bool s3Right = hw_.switches[Funbox::SWITCH_3_RIGHT].Pressed();
    midiMapper_.ResolveSwitch(midi, 2, s3Left, s3Right);
    if(s3Left)
        engine_.SetEffectOrder(TremoloEngine::EFFECT_ORDER_EFFECT_FIRST);
    else if(s3Right)
        engine_.SetEffectOrder(TremoloEngine::EFFECT_ORDER_TREMOLO_FIRST);
    else
        engine_.SetEffectOrder(TremoloEngine::EFFECT_ORDER_OFF);

    // DIP 1: input mode (ON = mono, OFF = stereo).
    // DIP 2: output mode (ON = mono, OFF = stereo).
    // DIP 3: envelope input (retrigger and envelope LED blink).
    // DIP 4: effect selection (ON = phaser, OFF = vibrato).
    stereo_           = !hw_.switches[Funbox::SWITCH_DIP_1].Pressed();
    monoOutput_       = hw_.switches[Funbox::SWITCH_DIP_2].Pressed();
    retriggerEnabled_ = hw_.switches[Funbox::SWITCH_DIP_3].Pressed();
    phaserMode_       = hw_.switches[Funbox::SWITCH_DIP_4].Pressed();

    envelopeBlinkEnabled_ = retriggerEnabled_ && !bypassed_;
}

void Pedal::ApplyEngineControls(const MidiCcController& midi)
{
    const float knob1 = midiMapper_.ResolveKnob(midi, 0, knobs_[0].Process());
    const float knob2 = midiMapper_.ResolveKnob(midi, 1, knobs_[1].Process());
    const float knob3 = midiMapper_.ResolveKnob(midi, 2, knobs_[2].Process());
    const float knob4 = midiMapper_.ResolveKnob(midi, 3, knobs_[3].Process());
    const float knob5 = midiMapper_.ResolveKnob(midi, 4, knobs_[4].Process());
    const float knob6 = midiMapper_.ResolveKnob(midi, 5, knobs_[5].Process());

    engine_.SetDivision(
        TremoloEngine::DivisionFromKnob(knob1, engine_.GetDivision()));
    engine_.SetDepth(knob2);
    engine_.SetShape(TremoloEngine::ShapeFromKnob(knob3, engine_.GetShape()));
    engine_.SetSpread(SpreadFromKnob(knob4));

    const ModValues mod = calibration_.ResolveKnobs(knob5, knob6);
    engine_.SetModAmount(mod.amount);
    engine_.SetModRate(mod.rate);

    engine_.SetRetriggerThreshold(calibration_.ActiveThreshold());
    engine_.SetGainCompensation(calibration_.ActiveGain());
    engine_.SetBypassed(bypassed_);
    engine_.SetRetriggerEnabled(retriggerEnabled_);
}

void Pedal::ApplyMidiPresses(const MidiFootswitchPresses& presses, uint32_t now)
{
    // Footswitch CCs are ignored while calibrating, like the physical pedals.
    if(calibration_.Active())
        return;

    // Each press toggles; an even number of presses in one block cancels out.
    if(presses.footswitch1 & 1u)
        bypassed_ = !bypassed_;

    if(presses.footswitch2 != 0)
    {
        bridge_.RequestTap();
        leds_.PulseRight(now);
    }
}

void Pedal::UpdateModulation(float bpm)
{
    engine_.SetTempo(bpm);
    quarterNoteMetro_.SetFreq(bpm / 60.0f);

    const float modRate = engine_.GetModRate();
    for(size_t channel = 0; channel < 2; ++channel)
    {
        phaser_[channel].SetLfoFreq(modRate);
        vibrato_[channel].SetLfoFreq(modRate);
    }

    const float vibratoDepth = kMaxVibratoDepth * (1.0f - engine_.GetDepth());
    for(size_t channel = 0; channel < 2; ++channel)
        vibrato_[channel].SetLfoDepth(vibratoDepth);
}

void Pedal::RenderSamples(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size,
                          uint32_t                  now)
{
    const int effectOrder = engine_.GetEffectOrder();
    if(effectOrder != TremoloEngine::EFFECT_ORDER_OFF)
        lastEffectOrder_ = effectOrder;

    const bool effectEngaged
        = !bypassed_ && effectOrder != TremoloEngine::EFFECT_ORDER_OFF;
    const float targetEffectMix = effectEngaged ? 1.0f : 0.0f;
    const bool  effectFirst
        = lastEffectOrder_ == TremoloEngine::EFFECT_ORDER_EFFECT_FIRST;

    for(size_t i = 0; i < size; i++)
    {
        const float inL = in[0][i];
        const float inR = stereo_ ? in[1][i] : inL;

        float outL = inL;
        float outR = inR;

        effectMix_ += effectMixCoefficient_ * (targetEffectMix - effectMix_);

        if(effectOrder == TremoloEngine::EFFECT_ORDER_OFF
           && effectMix_ < kEffectMixSilentLevel)
        {
            engine_.Process(inL, inR, &outL, &outR);
        }
        else if(phaserMode_)
        {
            ProcessEffect(phaser_[0], phaser_[1], engine_, effectFirst,
                          effectMix_, inL, inR, &outL, &outR);
        }
        else
        {
            ProcessEffect(vibrato_[0], vibrato_[1], engine_, effectFirst,
                          effectMix_, inL, inR, &outL, &outR);
        }

        if(monoOutput_)
        {
            const float mono = SumToMono(outL, outR);
            out[0][i]        = mono;
            out[1][i]        = mono;
        }
        else
        {
            out[0][i] = outL;
            out[1][i] = outR;
        }

        if(quarterNoteMetro_.Process())
            leds_.PulseRight(now);
    }
}

void Pedal::UpdateLeds(uint32_t now)
{
    LedInputs inputs;
    inputs.bypassed             = bypassed_;
    inputs.calibrating          = calibration_.Active();
    inputs.envelopeBlinkEnabled = envelopeBlinkEnabled_;
    inputs.envelope             = engine_.GetEnvelope();
    inputs.envelopeThreshold    = calibration_.ActiveThreshold();

    leds_.Update(now, inputs);
}
