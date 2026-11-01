#include "daisy_petal.h"
#include "daisysp.h"
#include "funbox.h"
#include "tremolo_engine.h"

#include <cmath>

//
// Stereo tremolo for the GuitarML FunBox v3 / Daisy Seed platform.
//
// Controls: six knobs, three 3-way switches, four DIP switches, two SPST
// footswitches, two LEDs, TRS MIDI clock input, stereo audio I/O.
//

using namespace daisy;
using namespace daisysp;
using namespace funbox;

DaisyPetal hw;
Parameter  param1, param2, param3, param4, param5, param6;

TremoloEngine  engine;
TempoController tempo;

bool bypassed  = true;
uint32_t bypassPressMs = 0;

bool pswitch1[2], pswitch2[2], pswitch3[2], pdip[4];
int  switch1[2], switch2[2], switch3[2], dip[4];

Led led1, led2;

uint32_t led2PulseUntil = 0;
float    beatPhase      = 0.0f;

static void TriggerLed2Pulse(uint32_t nowMs)
{
    led2PulseUntil = nowMs + 60;
}

// Exponential mapping for the Spread (Phase Offset) knob. Control is subtle
// near zero and becomes more rapid toward full scale, giving finer tactile
// resolution over small phase shifts.
static float SpreadFromKnob(float knob)
{
    const float kCurve = 3.0f;
    if(knob <= 0.0f)
        return 0.0f;
    if(knob >= 1.0f)
        return 1.0f;
    return (expf(kCurve * knob) - 1.0f) / (expf(kCurve) - 1.0f);
}

void updateSwitch1() // left = sine, center = saw, right = square
{
    if(pswitch1[0] == true)
        engine.SetModWaveform(TremoloEngine::MOD_SINE);
    else if(pswitch1[1] == true)
        engine.SetModWaveform(TremoloEngine::MOD_SQUARE);
    else
        engine.SetModWaveform(TremoloEngine::MOD_SAW);
}

void updateSwitch2() // left = depth, center = depth + rate, right = rate
{
    if(pswitch2[0] == true)
        engine.SetModTarget(TremoloEngine::TARGET_DEPTH);
    else if(pswitch2[1] == true)
        engine.SetModTarget(TremoloEngine::TARGET_RATE);
    else
        engine.SetModTarget(TremoloEngine::TARGET_DEPTH_RATE);
}

void updateSwitch3() // unused
{
}

void UpdateButtons()
{
    const uint32_t now = System::GetNow();
    Switch&        fs1 = hw.switches[Funbox::FOOTSWITCH_1];
    Switch&        fs2 = hw.switches[Funbox::FOOTSWITCH_2];

    if(fs1.RisingEdge())
    {
        // Toggle immediately on press; the release decides whether the change
        // is latched (short press) or momentary (held longer than 600 ms).
        bypassPressMs = now;
        bypassed      = !bypassed;
    }

    if(fs1.FallingEdge())
    {
        // A long hold is momentary: undo the press toggle to restore the
        // previous state. A short press leaves the immediate toggle latched.
        if(now - bypassPressMs > 500)
            bypassed = !bypassed;
    }

    if(fs2.FallingEdge())
    {
        tempo.Tap(now);
        TriggerLed2Pulse(now);
    }
}

void UpdateSwitches()
{
    // 3-way Switch 1 (apply every block so the startup position is honoured)
    for(int i = 0; i < 2; i++)
        pswitch1[i] = hw.switches[switch1[i]].Pressed();
    updateSwitch1();

    // 3-way Switch 2
    for(int i = 0; i < 2; i++)
        pswitch2[i] = hw.switches[switch2[i]].Pressed();
    updateSwitch2();

    // 3-way Switch 3 (unused, tracked for completeness)
    for(int i = 0; i < 2; i++)
        pswitch3[i] = hw.switches[switch3[i]].Pressed();

    // DIP switches; DIP 1 selects mono (off) or stereo (on) input.
    for(int i = 0; i < 4; i++)
        pdip[i] = hw.switches[dip[i]].Pressed();
}

static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    hw.ProcessAnalogControls();
    hw.ProcessDigitalControls();

    UpdateButtons();
    UpdateSwitches();

    const uint32_t now = System::GetNow();

    const float vparam1 = param1.Process();
    const float vparam2 = param2.Process();
    const float vparam3 = param3.Process();
    const float vparam4 = param4.Process();
    const float vparam5 = param5.Process();
    const float vparam6 = param6.Process();

    engine.SetDivision(
        TremoloEngine::DivisionFromKnob(vparam1, engine.GetDivision()));
    engine.SetDepth(vparam2);
    engine.SetShape(TremoloEngine::ShapeFromKnob(vparam3, engine.GetShape()));
    engine.SetSpread(SpreadFromKnob(vparam4));
    engine.SetModAmount(vparam5);
    engine.SetModRate(vparam6);
    engine.SetBypassed(bypassed);

    // Drain MIDI input each block. Listen() also restarts the UART receiver if
    // it stopped after an overrun.
    hw.midi.Listen();
    while(hw.midi.HasEvents())
    {
        const MidiEvent event = hw.midi.PopEvent();
        if(event.type == SystemRealTime
           && event.srt_type == TimingClock)
            tempo.MidiClock(now);
    }

    const float bpm = tempo.Bpm(now);
    engine.SetTempo(bpm);

    const float sampleRate = hw.AudioSampleRate();
    const bool  stereo     = pdip[0];

    for(size_t i = 0; i < size; i++)
    {
        const float inL = in[0][i];
        const float inR = stereo ? in[1][i] : inL;

        float outL = inL;
        float outR = inR;
        engine.Process(inL, inR, &outL, &outR);

        out[0][i] = outL;
        out[1][i] = outR;
    }

    // Quarter-note blink for LED 2.
    beatPhase += static_cast<float>(size) * (bpm / 60.0f) / sampleRate;
    if(beatPhase >= 1.0f)
    {
        beatPhase -= floorf(beatPhase);
        TriggerLed2Pulse(now);
    }

    const bool active = !bypassed;
    led1.Set(active ? 1.0f : 0.0f);
    led1.Update();

    const bool blinkOn = static_cast<int32_t>(led2PulseUntil - now) > 0;
    led2.Set(blinkOn ? 1.0f : 0.0f);
    led2.Update();
}

int main(void)
{
    hw.Init();
    hw.SetAudioBlockSize(48);

    const float samplerate = hw.AudioSampleRate();

    switch1[0] = Funbox::SWITCH_1_LEFT;
    switch1[1] = Funbox::SWITCH_1_RIGHT;
    switch2[0] = Funbox::SWITCH_2_LEFT;
    switch2[1] = Funbox::SWITCH_2_RIGHT;
    switch3[0] = Funbox::SWITCH_3_LEFT;
    switch3[1] = Funbox::SWITCH_3_RIGHT;
    dip[0]     = Funbox::SWITCH_DIP_1;
    dip[1]     = Funbox::SWITCH_DIP_2;
    dip[2]     = Funbox::SWITCH_DIP_3;
    dip[3]     = Funbox::SWITCH_DIP_4;

    for(int i = 0; i < 2; i++)
    {
        pswitch1[i] = false;
        pswitch2[i] = false;
        pswitch3[i] = false;
    }
    for(int i = 0; i < 4; i++)
        pdip[i] = false;

    param1.Init(hw.knob[Funbox::KNOB_1], 0.0f, 1.0f, Parameter::LINEAR);
    param2.Init(hw.knob[Funbox::KNOB_2], 0.0f, 1.0f, Parameter::LINEAR);
    param3.Init(hw.knob[Funbox::KNOB_3], 0.0f, 1.0f, Parameter::LINEAR);
    param4.Init(hw.knob[Funbox::KNOB_4], 0.0f, 1.0f, Parameter::LINEAR);
    param5.Init(hw.knob[Funbox::KNOB_5], 0.0f, 1.0f, Parameter::LINEAR);
    param6.Init(hw.knob[Funbox::KNOB_6], 0.0f, 1.0f, Parameter::LINEAR);

    led1.Init(hw.seed.GetPin(Funbox::LED_1), false);
    led1.Set(0.0f);
    led1.Update();

    led2.Init(hw.seed.GetPin(Funbox::LED_2), false);
    led2.Set(0.0f);
    led2.Update();

    engine.Init(samplerate);
    engine.SetBypassed(true);

    hw.InitMidi();
    hw.midi.StartReceive();

    hw.StartAdc();
    hw.StartAudio(AudioCallback);

    while(1)
        System::Delay(10);
}
