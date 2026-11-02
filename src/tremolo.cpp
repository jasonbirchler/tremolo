#include "daisy_petal.h"
#include "daisysp.h"
#include "funbox.h"
#include "tremolo_engine.h"

#include <cmath>
#include <cstring>

//
// Stereo tremolo for the GuitarML FunBox v3 / Daisy Seed platform.
//
// Controls: six knobs, three 3-way switches, four DIP switches, two SPST
// footswitches, two LEDs, TRS MIDI clock input, stereo audio I/O.
//

using namespace daisy;
using namespace daisysp;
using namespace funbox;

namespace
{
constexpr uint32_t kCalibrationHoldMs        = 5000;
constexpr uint32_t kCalibrationFlashOnMs     = 150;
constexpr uint32_t kCalibrationFlashPeriodMs = 300;
constexpr uint32_t kCalibrationFlashTotalMs  = 600;
constexpr uint32_t kSaveRetryMs              = 1000;
constexpr uint32_t kSaveErrorFlashOnMs       = 80;
constexpr uint32_t kSaveErrorFlashPeriodMs   = 160;
constexpr uint32_t kSaveErrorFlashTotalMs    = 640;
constexpr float    kCalibrationPulseHz       = 1.0f;
constexpr float    kDefaultEnvelopeThreshold = 0.5f;
constexpr float    kTwoPi                    = 6.28318530717958647692f;

// Last 4 kB sector of the 8 MB QSPI flash. Firmware images are stored from
// the start of QSPI, so this sector survives normal reflashing and reboots.
constexpr uint32_t kCalibrationStorageAddress = 0x007FF000;
constexpr uint32_t kCalibrationMagic          = 0x43414C31u; // "CAL1"
constexpr uint32_t kCalibrationVersion        = 1u;

struct CalibrationRecord
{
    uint32_t magic;
    uint32_t version;
    float    threshold;
    uint32_t checksum;
};
static_assert(sizeof(CalibrationRecord) == 16,
              "CalibrationRecord must be 16 bytes");
} // namespace

DaisyPetal hw;

namespace
{
uint32_t CalibrationChecksum(const CalibrationRecord& record)
{
    uint32_t bits;
    std::memcpy(&bits, &record.threshold, sizeof(bits));
    return record.magic ^ record.version ^ bits ^ 0xA5A5A5A5u;
}

void InvalidateCalibrationCache()
{
    auto* mapped = static_cast<uint8_t*>(
        hw.seed.qspi.GetData(kCalibrationStorageAddress));
    dsy_dma_invalidate_cache_for_buffer(mapped, sizeof(CalibrationRecord));
}

float LoadCalibrationThreshold()
{
    InvalidateCalibrationCache();

    CalibrationRecord record;
    std::memcpy(&record,
                hw.seed.qspi.GetData(kCalibrationStorageAddress),
                sizeof(record));

    const float threshold = record.threshold;
    if(record.magic == kCalibrationMagic
       && record.version == kCalibrationVersion
       && record.checksum == CalibrationChecksum(record)
       && threshold >= 0.0f && threshold <= 1.0f)
        return threshold;

    return kDefaultEnvelopeThreshold;
}

bool SaveCalibrationThreshold(float threshold)
{
    // A NaN threshold fails both range checks and falls back to the default.
    if(!(threshold >= 0.0f && threshold <= 1.0f))
        threshold = kDefaultEnvelopeThreshold;

    // QSPI cannot be written while the application executes from it.
    if(System::GetProgramMemoryRegion() == System::MemoryRegion::QSPI)
        return false;

    CalibrationRecord record;
    record.magic     = kCalibrationMagic;
    record.version   = kCalibrationVersion;
    record.threshold = threshold;
    record.checksum  = CalibrationChecksum(record);

    const uint32_t address = kCalibrationStorageAddress;
    if(hw.seed.qspi.Erase(address, address + sizeof(record))
       != QSPIHandle::Result::OK)
        return false;
    if(hw.seed.qspi.Write(address,
                          sizeof(record),
                          reinterpret_cast<uint8_t*>(&record))
       != QSPIHandle::Result::OK)
        return false;

    InvalidateCalibrationCache();
    CalibrationRecord readback;
    std::memcpy(&readback,
                hw.seed.qspi.GetData(address),
                sizeof(readback));

    return readback.magic == record.magic
           && readback.version == record.version
           && readback.checksum == record.checksum
           && readback.threshold == record.threshold;
}
} // namespace

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

// Envelope-triggered LED blink (enabled by DIP 2 outside calibration)
bool  envelopeBlinkEnabled = false;
float envelopeThreshold     = kDefaultEnvelopeThreshold;

// Calibration mode is entered and exited by holding both footswitches.
bool     calibrationMode         = false;
uint32_t calibrationFlashStartMs = 0;
bool     calibrationFlashActive  = false;

volatile bool thresholdSaveRequested = false;
volatile bool calibrationSaveErrorActive = false;
volatile uint32_t calibrationSaveErrorStartMs = 0;

bool     bothFootswitchesHeld        = false;
uint32_t bothFootswitchesHoldStartMs = 0;
bool     bothHoldConsumed            = false;
bool     dualHoldGestureActive       = false;
bool     fs1PressToggledBypass       = false;

enum BlinkState { BLINK_IDLE, BLINK_ON1, BLINK_OFF1, BLINK_ON2, BLINK_OFF2 };
BlinkState blinkState       = BLINK_IDLE;
uint32_t blinkStateStartMs  = 0;
bool  envelopeAboveThreshold = false;  // previous envelope state for edge detect
static float storedModRate  = 0.5f;    // preserves mod rate when envelope mode is on

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

void updateSwitch1() // left = sine, center = sawtooth, right = reverse sawtooth
{
    if(pswitch1[0] == true)
        engine.SetModWaveform(TremoloEngine::MOD_SINE);
    else if(pswitch1[1] == true)
        engine.SetModWaveform(TremoloEngine::MOD_REVERSE_SAW);
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

static void ToggleCalibration(uint32_t nowMs)
{
    calibrationMode = !calibrationMode;

    blinkState             = BLINK_IDLE;
    envelopeAboveThreshold = engine.GetEnvelope() >= envelopeThreshold;

    if(calibrationMode)
    {
        // Flash both LEDs twice when entering calibration.
        calibrationFlashStartMs = nowMs;
        calibrationFlashActive  = true;
    }
    else
    {
        // Persist the tuned threshold after leaving calibration.
        thresholdSaveRequested = true;
    }
}

void UpdateButtons()
{
    const uint32_t now = System::GetNow();
    Switch&        fs1 = hw.switches[Funbox::FOOTSWITCH_1];
    Switch&        fs2 = hw.switches[Funbox::FOOTSWITCH_2];

    const bool fs1Down = fs1.Pressed();
    const bool fs2Down = fs2.Pressed();

    // Holding both footswitches for the calibration time toggles the mode.
    if(fs1Down && fs2Down)
    {
        if(!bothFootswitchesHeld)
        {
            bothFootswitchesHeld        = true;
            bothFootswitchesHoldStartMs = now;
            bothHoldConsumed            = false;
            dualHoldGestureActive       = true;

            // If footswitch 1 was pressed before footswitch 2, its normal
            // bypass toggle already fired. Undo it so the hold is silent.
            if(fs1PressToggledBypass)
            {
                bypassed              = !bypassed;
                fs1PressToggledBypass = false;
            }
        }
        else if(!bothHoldConsumed
                && (now - bothFootswitchesHoldStartMs) >= kCalibrationHoldMs)
        {
            bothHoldConsumed = true;
            ToggleCalibration(now);
        }
        return;
    }

    bothFootswitchesHeld = false;
    bothHoldConsumed     = false;

    // Ignore every release edge belonging to a dual-footswitch gesture. Keep
    // suppressing until both switches are up so exiting calibration cannot
    // toggle bypass or register a tap tempo.
    if(dualHoldGestureActive)
    {
        if(!fs1Down && !fs2Down)
            dualHoldGestureActive = false;
        return;
    }

    // Normal footswitch actions are disabled while calibrating.
    if(calibrationMode)
        return;

    if(fs1.RisingEdge())
    {
        // Toggle immediately on press; the release decides whether the change
        // is latched (short press) or momentary (held longer than 600 ms).
        bypassPressMs         = now;
        bypassed              = !bypassed;
        fs1PressToggledBypass = true;
    }

    if(fs1.FallingEdge())
    {
        // A long hold is momentary: undo the press toggle to restore the
        // previous state. A short press leaves the immediate toggle latched.
        if(now - bypassPressMs > 500)
            bypassed = !bypassed;
        fs1PressToggledBypass = false;
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
    // DIP 2 enables envelope-triggered LED blink mode.
    for(int i = 0; i < 4; i++)
        pdip[i] = hw.switches[dip[i]].Pressed();

    envelopeBlinkEnabled = pdip[1];
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
    if(calibrationMode)
    {
        // Calibration: knob 6 sets the threshold; the mod rate is held.
        envelopeThreshold = vparam6;
        engine.SetModRate(storedModRate);
    }
    else
    {
        // Normal operation: knob 6 controls modulation rate.
        storedModRate = vparam6;
        engine.SetModRate(vparam6);
    }
    engine.SetRetriggerThreshold(envelopeThreshold);
    engine.SetBypassed(bypassed);
    engine.SetRetriggerEnabled(pdip[1]);

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

    const float sampleRate = engine.GetSampleRate();
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

    float led1Value = bypassed ? 0.0f : 1.0f;
    float led2Value = static_cast<int32_t>(led2PulseUntil - now) > 0 ? 1.0f : 0.0f;

    if(calibrationMode || envelopeBlinkEnabled)
    {
        const float env = engine.GetEnvelope();
        const bool  nowAbove = env >= envelopeThreshold;

        // Rising edge triggers the blink sequence
        if(nowAbove && !envelopeAboveThreshold)
        {
            blinkState          = BLINK_ON1;
            blinkStateStartMs   = now;
        }
        envelopeAboveThreshold = nowAbove;

        // State machine: ON1 (100ms) -> OFF1 (400ms) -> ON2 (100ms) -> OFF2 -> idle
        switch(blinkState)
        {
            case BLINK_ON1:
                led1Value = 1.0f;
                if(now - blinkStateStartMs >= 100)
                {
                    blinkState        = BLINK_OFF1;
                    blinkStateStartMs = now;
                }
                break;
            case BLINK_OFF1:
                led1Value = 0.0f;
                if(now - blinkStateStartMs >= 400)  // 500ms total from ON1 start
                {
                    blinkState        = BLINK_ON2;
                    blinkStateStartMs = now;
                }
                break;
            case BLINK_ON2:
                led1Value = 1.0f;
                if(now - blinkStateStartMs >= 100)
                {
                    blinkState        = BLINK_OFF2;
                    blinkStateStartMs = now;
                }
                break;
            case BLINK_OFF2:
                led1Value = 0.0f;
                if(now - blinkStateStartMs >= 400)
                    blinkState = BLINK_IDLE;
                break;
            case BLINK_IDLE:
            default:
                // Outside calibration fall back to the bypass indicator.
                led1Value = calibrationMode ? 0.0f
                                            : (bypassed ? 0.0f : 1.0f);
                break;
        }
    }
    else
    {
        blinkState = BLINK_IDLE;
    }

    // Two-flash indication on both LEDs when entering calibration.
    if(calibrationMode && calibrationFlashActive)
    {
        const uint32_t elapsed = now - calibrationFlashStartMs;
        if(elapsed < kCalibrationFlashTotalMs)
        {
            const bool on
                = (elapsed % kCalibrationFlashPeriodMs) < kCalibrationFlashOnMs;
            led1Value = on ? 1.0f : 0.0f;
            led2Value = on ? 1.0f : 0.0f;
        }
        else
        {
            calibrationFlashActive = false;
        }
    }

    // While calibrating, the right LED breathes continuously.
    if(calibrationMode && !calibrationFlashActive)
    {
        const float seconds = static_cast<float>(now) * 0.001f;
        led2Value = 0.5f + 0.5f * sinf(kTwoPi * kCalibrationPulseHz * seconds);
    }

    // Rapid four-flash indication if calibration persistence failed.
    if(calibrationSaveErrorActive)
    {
        const uint32_t elapsed = now - calibrationSaveErrorStartMs;
        if(elapsed < kSaveErrorFlashTotalMs)
        {
            const bool on
                = (elapsed % kSaveErrorFlashPeriodMs) < kSaveErrorFlashOnMs;
            led1Value = on ? 1.0f : 0.0f;
            led2Value = on ? 1.0f : 0.0f;
        }
        else
        {
            calibrationSaveErrorActive = false;
        }
    }

    led1.Set(led1Value);
    led1.Update();

    led2.Set(led2Value);
    led2.Update();
}

int main(void)
{
    hw.Init();
    hw.SetAudioBlockSize(48);

    envelopeThreshold = LoadCalibrationThreshold();

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

    uint32_t nextSaveAttemptMs = 0;

    while(1)
    {
        // Persist calibration outside the audio callback to avoid blocking it.
        const uint32_t now = System::GetNow();
        if(thresholdSaveRequested
           && static_cast<int32_t>(now - nextSaveAttemptMs) >= 0)
        {
            if(SaveCalibrationThreshold(envelopeThreshold))
            {
                thresholdSaveRequested  = false;
                nextSaveAttemptMs       = 0;
            }
            else
            {
                calibrationSaveErrorActive  = true;
                calibrationSaveErrorStartMs = now;
                nextSaveAttemptMs           = now + kSaveRetryMs;
            }
        }
        System::Delay(10);
    }
}
