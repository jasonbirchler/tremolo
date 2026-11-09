#include "daisy_petal.h"
#include "daisysp.h"
#include "funbox.h"
#include "tremolo_engine.h"
#include "vibrato.h"

#include <cmath>
#include <cstddef>
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
constexpr float    kDefaultGainCompensation  = 0.0f;
constexpr float    kKnobPickupTolerance      = 0.02f;
constexpr float    kEffectMixSmoothingSeconds = 0.005f;
constexpr float    kMaxVibratoDepth           = 0.2f;
static constexpr uint32_t kEnvelopeBlinkDurations[] = {100, 400, 100, 400};
constexpr size_t kEnvelopeBlinkPhaseCount
    = sizeof(kEnvelopeBlinkDurations) / sizeof(kEnvelopeBlinkDurations[0]);

// Last 4 kB sector of the 8 MB QSPI flash. Firmware images are stored from
// the start of QSPI, so this sector survives normal reflashing and reboots.
constexpr uint32_t kCalibrationStorageAddress = 0x007FF000;
constexpr uint32_t kCalibrationMagic          = 0x43414C32u; // "CAL2"
constexpr uint32_t kCalibrationVersion        = 2u;
constexpr uint32_t kLegacyCalibrationMagic    = 0x43414C31u; // "CAL1"
constexpr uint32_t kLegacyCalibrationVersion  = 1u;

struct CalibrationRecord
{
    uint32_t magic;
    uint32_t version;
    float    threshold;
    float    gain;
    uint32_t checksum;
};
static_assert(sizeof(CalibrationRecord) == 20,
              "CalibrationRecord must be 20 bytes");

struct LegacyCalibrationRecord
{
    uint32_t magic;
    uint32_t version;
    float    threshold;
    uint32_t checksum;
};
static_assert(sizeof(LegacyCalibrationRecord) == 16,
              "LegacyCalibrationRecord must be 16 bytes");

struct CalibrationSettings
{
    float threshold;
    float gain;
};

bool FlashActive(uint32_t now,
                 uint32_t start,
                 uint32_t totalMs,
                 uint32_t periodMs,
                 uint32_t onMs)
{
    const uint32_t elapsed = now - start;
    return periodMs != 0 && elapsed < totalMs && elapsed % periodMs < onMs;
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

DaisyPetal hw;

namespace
{
uint32_t FloatBits(float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

uint32_t CalibrationChecksum(const CalibrationRecord& record)
{
    return record.magic ^ record.version ^ FloatBits(record.threshold)
           ^ FloatBits(record.gain) ^ 0xA5A5A5A5u;
}

uint32_t LegacyCalibrationChecksum(const LegacyCalibrationRecord& record)
{
    return record.magic ^ record.version ^ FloatBits(record.threshold)
           ^ 0xA5A5A5A5u;
}

void InvalidateCalibrationCache()
{
    auto* mapped = static_cast<uint8_t*>(
        hw.seed.qspi.GetData(kCalibrationStorageAddress));
    dsy_dma_invalidate_cache_for_buffer(mapped, sizeof(CalibrationRecord));
}

CalibrationSettings LoadCalibrationSettings()
{
    InvalidateCalibrationCache();

    CalibrationRecord record;
    std::memcpy(&record,
                hw.seed.qspi.GetData(kCalibrationStorageAddress),
                sizeof(record));

    const float threshold = record.threshold;
    const float gain      = record.gain;
    if(record.magic == kCalibrationMagic
       && record.version == kCalibrationVersion
       && record.checksum == CalibrationChecksum(record)
       && threshold >= 0.0f && threshold <= 1.0f
       && gain >= 0.0f && gain <= 1.0f)
        return {threshold, gain};

    // Preserve the threshold from a record written before gain compensation
    // was added.
    LegacyCalibrationRecord legacy;
    std::memcpy(&legacy,
                hw.seed.qspi.GetData(kCalibrationStorageAddress),
                sizeof(legacy));
    if(legacy.magic == kLegacyCalibrationMagic
       && legacy.version == kLegacyCalibrationVersion
       && legacy.checksum == LegacyCalibrationChecksum(legacy)
       && legacy.threshold >= 0.0f && legacy.threshold <= 1.0f)
        return {legacy.threshold, kDefaultGainCompensation};

    return {kDefaultEnvelopeThreshold, kDefaultGainCompensation};
}

bool SaveCalibrationSettings(float threshold, float gain)
{
    // A NaN setting fails both range checks and falls back to the default.
    if(!(threshold >= 0.0f && threshold <= 1.0f))
        threshold = kDefaultEnvelopeThreshold;
    if(!(gain >= 0.0f && gain <= 1.0f))
        gain = kDefaultGainCompensation;

    // QSPI cannot be written while the application executes from it.
    if(System::GetProgramMemoryRegion() == System::MemoryRegion::QSPI)
        return false;

    CalibrationRecord record;
    record.magic     = kCalibrationMagic;
    record.version   = kCalibrationVersion;
    record.threshold = threshold;
    record.gain      = gain;
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
           && readback.threshold == record.threshold
           && readback.gain == record.gain;
}
} // namespace

Parameter  param1, param2, param3, param4, param5, param6;

TremoloEngine   engine;
TempoController tempo;
MidiCcController midiControl;
daisysp::Phaser phaser[2];
Vibrato         vibrato[2];

float effectMix            = 0.0f;
float effectMixCoefficient = 0.0f;
int   lastEffectOrder      = TremoloEngine::EFFECT_ORDER_EFFECT_FIRST;

bool bypassed  = true;
uint32_t bypassPressMs = 0;

bool switch1LeftPressed  = false;
bool switch1RightPressed = false;
bool switch2LeftPressed  = false;
bool switch2RightPressed = false;
bool switch3LeftPressed  = false;
bool switch3RightPressed = false;
bool dipSwitch1On        = false;
bool dipSwitch2On        = false;
bool dipSwitch3On        = false;
bool dipSwitch4On        = false;

Led led1, led2;

uint32_t led2PulseUntil = 0;
Metro    quarterNoteMetro;

// Envelope-triggered LED blink (enabled by DIP 2 outside calibration)
bool  envelopeBlinkEnabled = false;
float envelopeThreshold     = kDefaultEnvelopeThreshold;
float gainCompensation      = kDefaultGainCompensation;

// Calibration mode is entered and exited by holding both footswitches.
bool     calibrationMode         = false;
uint32_t calibrationFlashStartMs = 0;
bool     calibrationFlashActive  = false;

// Values edited in calibration stay pending until they are saved successfully.
float pendingEnvelopeThreshold = kDefaultEnvelopeThreshold;
float pendingGainCompensation  = kDefaultGainCompensation;
float saveThresholdValue       = kDefaultEnvelopeThreshold;
float saveGainValue            = kDefaultGainCompensation;

volatile bool thresholdSaveRequested = false;
volatile bool calibrationSaveErrorActive = false;
volatile uint32_t calibrationSaveErrorStartMs = 0;

bool     bothFootswitchesHeld        = false;
uint32_t bothFootswitchesHoldStartMs = 0;
bool     bothHoldConsumed            = false;
bool     dualHoldGestureActive       = false;
bool     fs1PressToggledBypass       = false;

size_t   envelopeBlinkPhase   = kEnvelopeBlinkPhaseCount;
uint32_t envelopeBlinkStartMs = 0;
bool     envelopeAboveThreshold = false;
static float storedModRate  = 0.5f;    // preserves mod rate when envelope mode is on
static float storedModAmount = 0.0f;   // preserves mod amount while calibrating
static bool  modRatePickupPending   = false;
static bool  modAmountPickupPending = false;

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
    if(switch1LeftPressed == true)
        engine.SetModWaveform(TremoloEngine::MOD_SINE);
    else if(switch1RightPressed == true)
        engine.SetModWaveform(TremoloEngine::MOD_REVERSE_SAW);
    else
        engine.SetModWaveform(TremoloEngine::MOD_SAW);
}

void updateSwitch2() // left = depth, center = depth + rate, right = rate
{
    if(switch2LeftPressed == true)
        engine.SetModTarget(TremoloEngine::TARGET_DEPTH);
    else if(switch2RightPressed == true)
        engine.SetModTarget(TremoloEngine::TARGET_RATE);
    else
        engine.SetModTarget(TremoloEngine::TARGET_DEPTH_RATE);
}

void updateSwitch3()
{
    if(switch3LeftPressed == true)
        engine.SetEffectOrder(TremoloEngine::EFFECT_ORDER_EFFECT_FIRST);
    else if(switch3RightPressed == true)
        engine.SetEffectOrder(TremoloEngine::EFFECT_ORDER_TREMOLO_FIRST);
    else
        engine.SetEffectOrder(TremoloEngine::EFFECT_ORDER_OFF);
}

static void ToggleCalibration(uint32_t nowMs)
{
    calibrationMode = !calibrationMode;

    envelopeBlinkPhase      = kEnvelopeBlinkPhaseCount;
    envelopeAboveThreshold = engine.GetEnvelope()
                             >= (calibrationMode ? pendingEnvelopeThreshold
                                                 : envelopeThreshold);

    if(calibrationMode)
    {
        pendingEnvelopeThreshold = envelopeThreshold;
        pendingGainCompensation  = gainCompensation;
        modAmountPickupPending   = true;
        modRatePickupPending     = true;

        // Flash both LEDs twice when entering calibration.
        calibrationFlashStartMs = nowMs;
        calibrationFlashActive  = true;
    }
    else
    {
        // Persist the tuned settings after leaving calibration.
        saveThresholdValue     = pendingEnvelopeThreshold;
        saveGainValue          = pendingGainCompensation;
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

// A MIDI CC takes over the corresponding control. Moving the physical knob or
// switch afterwards hands control back to the hardware, so the knobs always
// win once they are touched.
static float lastHardwareKnobs[MidiCcController::KNOB_COUNT] = {};
static bool  knobHadMidiOverride[MidiCcController::KNOB_COUNT] = {};

static float ResolveKnobValue(int index, float hardwareValue)
{
    if(midiControl.KnobOverridden(index))
    {
        if(!knobHadMidiOverride[index])
        {
            // Capture the physical position when MIDI takes over so a
            // deliberate turn can be detected.
            lastHardwareKnobs[index] = hardwareValue;
            knobHadMidiOverride[index] = true;
        }
        else if(fabsf(hardwareValue - lastHardwareKnobs[index])
                > kKnobPickupTolerance)
        {
            midiControl.ReleaseKnob(index);
            knobHadMidiOverride[index] = false;
        }
    }
    else
    {
        knobHadMidiOverride[index] = false;
    }

    return midiControl.KnobOverridden(index) ? midiControl.KnobValue(index)
                                             : hardwareValue;
}

static int PhysicalSwitchPosition(bool left, bool right)
{
    if(left)
        return MidiCcController::SWITCH_LEFT;
    if(right)
        return MidiCcController::SWITCH_RIGHT;
    return MidiCcController::SWITCH_CENTER;
}

static int  lastPhysicalSwitchPositions[MidiCcController::SWITCH_COUNT] = {};
static bool switchHadMidiOverride[MidiCcController::SWITCH_COUNT] = {};

static void ApplySwitchOverride(int index, bool& left, bool& right)
{
    const int physical = PhysicalSwitchPosition(left, right);

    if(midiControl.SwitchOverridden(index))
    {
        if(!switchHadMidiOverride[index])
        {
            lastPhysicalSwitchPositions[index] = physical;
            switchHadMidiOverride[index] = true;
        }
        else if(physical != lastPhysicalSwitchPositions[index])
        {
            midiControl.ReleaseSwitch(index);
            switchHadMidiOverride[index] = false;
        }
    }
    else
    {
        switchHadMidiOverride[index] = false;
    }

    if(!midiControl.SwitchOverridden(index))
        return;

    const int position = midiControl.SwitchPosition(index);
    left               = position == MidiCcController::SWITCH_LEFT;
    right              = position == MidiCcController::SWITCH_RIGHT;
}

void UpdateSwitches()
{
    // 3-way Switch 1 (apply every block so the startup position is honoured)
    switch1LeftPressed = hw.switches[Funbox::SWITCH_1_LEFT].Pressed();
    switch1RightPressed = hw.switches[Funbox::SWITCH_1_RIGHT].Pressed();
    ApplySwitchOverride(0, switch1LeftPressed, switch1RightPressed);
    updateSwitch1();

    // 3-way Switch 2
    switch2LeftPressed = hw.switches[Funbox::SWITCH_2_LEFT].Pressed();
    switch2RightPressed = hw.switches[Funbox::SWITCH_2_RIGHT].Pressed();
    ApplySwitchOverride(1, switch2LeftPressed, switch2RightPressed);
    updateSwitch2();

    // 3-way Switch 3 selects the phaser or vibrato order (left, center = off,
    // right). DIP 3 chooses which effect the switch controls.
    switch3LeftPressed = hw.switches[Funbox::SWITCH_3_LEFT].Pressed();
    switch3RightPressed = hw.switches[Funbox::SWITCH_3_RIGHT].Pressed();
    ApplySwitchOverride(2, switch3LeftPressed, switch3RightPressed);
    updateSwitch3();

    // DIP switches; DIP 1 selects mono (off) or stereo (on) input.
    // DIP 2 enables envelope-triggered LED blink mode.
    // DIP 3 selects phaser (on) or vibrato (off) routing for Switch 3.
    dipSwitch1On = hw.switches[Funbox::SWITCH_DIP_1].Pressed();
    dipSwitch2On = hw.switches[Funbox::SWITCH_DIP_2].Pressed();
    dipSwitch3On = hw.switches[Funbox::SWITCH_DIP_3].Pressed();
    dipSwitch4On = hw.switches[Funbox::SWITCH_DIP_4].Pressed();

    envelopeBlinkEnabled = dipSwitch2On;
}

static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    hw.ProcessAllControls();

    UpdateButtons();
    UpdateSwitches();

    const uint32_t now = System::GetNow();

    const float vparam1 = ResolveKnobValue(0, param1.Process());
    const float vparam2 = ResolveKnobValue(1, param2.Process());
    const float vparam3 = ResolveKnobValue(2, param3.Process());
    const float vparam4 = ResolveKnobValue(3, param4.Process());
    const float vparam5 = ResolveKnobValue(4, param5.Process());
    const float vparam6 = ResolveKnobValue(5, param6.Process());

    engine.SetDivision(
        TremoloEngine::DivisionFromKnob(vparam1, engine.GetDivision()));
    engine.SetDepth(vparam2);
    engine.SetShape(TremoloEngine::ShapeFromKnob(vparam3, engine.GetShape()));
    engine.SetSpread(SpreadFromKnob(vparam4));

    if(calibrationMode)
    {
        // Calibration: knob 5 sets gain compensation, knob 6 sets the
        // threshold. Both stay pending until the settings are saved.
        pendingGainCompensation  = vparam5;
        pendingEnvelopeThreshold = vparam6;
        engine.SetModAmount(storedModAmount);
        engine.SetModRate(storedModRate);
    }
    else
    {
        // Hold the mod controls until their knobs are returned to the saved
        // positions, so a value dialled in during calibration cannot leak
        // into the normal effect.
        if(modAmountPickupPending)
        {
            if(fabsf(vparam5 - storedModAmount) <= kKnobPickupTolerance)
                modAmountPickupPending = false;
        }
        else
        {
            storedModAmount = vparam5;
        }

        if(modRatePickupPending)
        {
            if(fabsf(vparam6 - storedModRate) <= kKnobPickupTolerance)
                modRatePickupPending = false;
        }
        else
        {
            storedModRate = vparam6;
        }

        engine.SetModAmount(storedModAmount);
        engine.SetModRate(storedModRate);
    }

    engine.SetRetriggerThreshold(calibrationMode ? pendingEnvelopeThreshold
                                                 : envelopeThreshold);
    engine.SetGainCompensation(gainCompensation);
    engine.SetBypassed(bypassed);
    engine.SetRetriggerEnabled(dipSwitch2On);

    // Drain MIDI input each block. Listen() also restarts the UART receiver if
    // it stopped after an overrun.
    hw.midi.Listen();
    while(hw.midi.HasEvents())
    {
        MidiEvent event = hw.midi.PopEvent();
        if(event.type == SystemRealTime)
        {
            if(event.srt_type == TimingClock)
                tempo.MidiClock(now);
            continue;
        }

        if(event.type == ControlChange)
        {
            const ControlChangeEvent cc = event.AsControlChange();

            // Footswitch CCs behave like the physical pedals: a press edge
            // toggles bypass or registers a tap tempo.
            if(cc.control_number == MidiCcController::FOOTSWITCH_1_CC)
            {
                const bool wasPressed = midiControl.FootswitchOverridden(0)
                                        && midiControl.FootswitchPressed(0);
                midiControl.ControlChange(cc.control_number, cc.value);
                if(!wasPressed && midiControl.FootswitchPressed(0)
                   && !calibrationMode)
                    bypassed = !bypassed;
            }
            else if(cc.control_number == MidiCcController::FOOTSWITCH_2_CC)
            {
                const bool wasPressed = midiControl.FootswitchOverridden(1)
                                        && midiControl.FootswitchPressed(1);
                midiControl.ControlChange(cc.control_number, cc.value);
                if(!wasPressed && midiControl.FootswitchPressed(1)
                   && !calibrationMode)
                {
                    tempo.Tap(now);
                    TriggerLed2Pulse(now);
                }
            }
            else
            {
                midiControl.ControlChange(cc.control_number, cc.value);
            }
        }
    }

    const float bpm = tempo.Bpm(now);
    engine.SetTempo(bpm);
    quarterNoteMetro.SetFreq(bpm / 60.0f);

    const bool  stereo     = dipSwitch1On;

    const float modRate = engine.GetModRate();
    for(size_t channel = 0; channel < 2; ++channel)
    {
        phaser[channel].SetLfoFreq(modRate);
        vibrato[channel].SetLfoFreq(modRate);
    }

    const float vibratoDepth
        = kMaxVibratoDepth * (1.0f - engine.GetDepth());
    for(size_t channel = 0; channel < 2; ++channel)
        vibrato[channel].SetLfoDepth(vibratoDepth);

    const bool phaserMode = dipSwitch3On;
    const int  effectOrder = engine.GetEffectOrder();
    if(effectOrder != TremoloEngine::EFFECT_ORDER_OFF)
        lastEffectOrder = effectOrder;

    const bool effectEngaged
        = !bypassed && effectOrder != TremoloEngine::EFFECT_ORDER_OFF;
    const float targetEffectMix = effectEngaged ? 1.0f : 0.0f;

    for(size_t i = 0; i < size; i++)
    {
        const float inL = in[0][i];
        const float inR = stereo ? in[1][i] : inL;

        float outL = inL;
        float outR = inR;

        effectMix += effectMixCoefficient * (targetEffectMix - effectMix);

        if(effectOrder == TremoloEngine::EFFECT_ORDER_OFF
           && effectMix < 1.0e-4f)
        {
            engine.Process(inL, inR, &outL, &outR);
        }
        else if(phaserMode)
        {
            ProcessEffect(phaser[0],
                          phaser[1],
                          engine,
                          lastEffectOrder
                              == TremoloEngine::EFFECT_ORDER_EFFECT_FIRST,
                          effectMix,
                          inL,
                          inR,
                          &outL,
                          &outR);
        }
        else
        {
            ProcessEffect(vibrato[0],
                          vibrato[1],
                          engine,
                          lastEffectOrder
                              == TremoloEngine::EFFECT_ORDER_EFFECT_FIRST,
                          effectMix,
                          inL,
                          inR,
                          &outL,
                          &outR);
        }

        out[0][i] = outL;
        out[1][i] = outR;

        if(quarterNoteMetro.Process())
            TriggerLed2Pulse(now);
    }

    float led1Value = bypassed ? 0.0f : 1.0f;
    float led2Value = static_cast<int32_t>(led2PulseUntil - now) > 0 ? 1.0f : 0.0f;

    if(calibrationMode || envelopeBlinkEnabled)
    {
        const float env = engine.GetEnvelope();
        const bool  nowAbove
            = env >= (calibrationMode ? pendingEnvelopeThreshold
                                      : envelopeThreshold);

        if(nowAbove && !envelopeAboveThreshold)
        {
            envelopeBlinkPhase   = 0;
            envelopeBlinkStartMs = now;
        }
        envelopeAboveThreshold = nowAbove;

        if(envelopeBlinkPhase < kEnvelopeBlinkPhaseCount)
        {
            led1Value = envelopeBlinkPhase % 2 == 0 ? 1.0f : 0.0f;
            if(now - envelopeBlinkStartMs
               >= kEnvelopeBlinkDurations[envelopeBlinkPhase])
            {
                ++envelopeBlinkPhase;
                envelopeBlinkStartMs = now;
            }
        }
        else
        {
            led1Value = calibrationMode ? 0.0f
                                        : (bypassed ? 0.0f : 1.0f);
        }
    }
    else
    {
        envelopeBlinkPhase = kEnvelopeBlinkPhaseCount;
    }

    // Two-flash indication on both LEDs when entering calibration.
    if(calibrationMode && calibrationFlashActive)
    {
        const uint32_t elapsed = now - calibrationFlashStartMs;
        if(elapsed < kCalibrationFlashTotalMs)
        {
            const bool on = FlashActive(now,
                                        calibrationFlashStartMs,
                                        kCalibrationFlashTotalMs,
                                        kCalibrationFlashPeriodMs,
                                        kCalibrationFlashOnMs);
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
        led2Value = 0.5f + 0.5f * sinf(TWOPI_F * kCalibrationPulseHz * seconds);
    }

    // Rapid four-flash indication if calibration persistence failed.
    if(calibrationSaveErrorActive)
    {
        const uint32_t elapsed = now - calibrationSaveErrorStartMs;
        if(elapsed < kSaveErrorFlashTotalMs)
        {
            const bool on = FlashActive(now,
                                        calibrationSaveErrorStartMs,
                                        kSaveErrorFlashTotalMs,
                                        kSaveErrorFlashPeriodMs,
                                        kSaveErrorFlashOnMs);
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

    const CalibrationSettings settings = LoadCalibrationSettings();
    envelopeThreshold        = settings.threshold;
    gainCompensation         = settings.gain;
    pendingEnvelopeThreshold = settings.threshold;
    pendingGainCompensation  = settings.gain;
    saveThresholdValue       = settings.threshold;
    saveGainValue            = settings.gain;

    const float samplerate = hw.AudioSampleRate();
    quarterNoteMetro.Init(tempo.Bpm(System::GetNow()) / 60.0f, samplerate);
    effectMixCoefficient
        = 1.0f - expf(-1.0f / (kEffectMixSmoothingSeconds * samplerate));

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
    engine.SetGainCompensation(gainCompensation);

    for(size_t channel = 0; channel < 2; ++channel)
    {
        phaser[channel].Init(samplerate);
        phaser[channel].SetLfoDepth(0.5f);
        phaser[channel].SetLfoFreq(engine.GetModRate());
        phaser[channel].SetPoles(4);
        phaser[channel].SetFeedback(0.5f);

        vibrato[channel].Init(samplerate);
        vibrato[channel].SetLfoDepth(kMaxVibratoDepth);
        vibrato[channel].SetLfoFreq(engine.GetModRate());
    }

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
            if(SaveCalibrationSettings(saveThresholdValue, saveGainValue))
            {
                envelopeThreshold      = saveThresholdValue;
                gainCompensation       = saveGainValue;
                thresholdSaveRequested = false;
                nextSaveAttemptMs      = 0;
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
