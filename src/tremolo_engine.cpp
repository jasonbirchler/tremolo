#include "tremolo_engine.h"

#include <cmath>

#include "Utility/dsp.h"

namespace
{
using daisysp::fastmod1f;
using daisysp::fclamp;
using daisysp::fonepole;

const float kSmoothingSeconds = 0.005f;
const float kGainSmoothingSeconds = 0.001f;
const float kMinBpm = 30.0f;
const float kMaxBpm = 300.0f;
const float kRateModOctaves = 1.0f;
const float kModRateSlowMultiplier = 1.0f / 32.0f;
const float kModRateFastMultiplier = 1.5f;
const float kSpreadPhaseCycles = 0.25f;
const float kSpreadSideMaxGain = 1.5f;
const float kEnvelopeAttackSeconds = 0.001f;
const float kEnvelopeReleaseSeconds = 0.1f;
const float kEnvelopeRearmRatio = 0.7f;
const float kDefaultRetriggerThreshold = 0.75f;
const float kMaxGainCompensationDb = 6.0f;

float ModWaveformValue(int waveform, float phase)
{
    switch(waveform)
    {
        case TremoloEngine::MOD_SAW: return 2.0f * phase - 1.0f;
        case TremoloEngine::MOD_REVERSE_SAW: return 1.0f - 2.0f * phase;
        case TremoloEngine::MOD_SINE:
        default: return sinf(TWOPI_F * phase);
    }
}
} // namespace

void TapTempo::Reset()
{
    for(int i = 0; i < kHistory; ++i)
        intervals_[i] = 0;
    count_      = 0;
    index_      = 0;
    lastTapMs_  = 0;
    hasLastTap_ = false;
    bpm_        = 120.0f;
}

void TapTempo::Tap(uint32_t nowMs)
{
    if(hasLastTap_)
    {
        const uint32_t gap = nowMs - lastTapMs_;

        if(gap > 3000)
        {
            count_ = 0;
            index_ = 0;
        }
        else if(gap >= 200 && gap <= 2000)
        {
            intervals_[index_] = gap;
            index_             = (index_ + 1) % kHistory;
            if(count_ < kHistory)
                ++count_;

            uint32_t sum = 0;
            for(int i = 0; i < count_; ++i)
                sum += intervals_[i];

            if(sum > 0)
            {
                const float average = static_cast<float>(sum)
                                      / static_cast<float>(count_);
                bpm_ = fclamp(60000.0f / average, kMinBpm, kMaxBpm);
            }
        }
    }

    lastTapMs_  = nowMs;
    hasLastTap_ = true;
}

void MidiClockTempo::Reset()
{
    for(int i = 0; i < kHistory; ++i)
        intervals_[i] = 0;
    count_      = 0;
    index_      = 0;
    lastClockMs_ = 0;
    hasClock_    = false;
    bpm_         = 120.0f;
}

void MidiClockTempo::Tick(uint32_t nowMs)
{
    if(!hasClock_)
    {
        hasClock_    = true;
        lastClockMs_ = nowMs;
        return;
    }

    const uint32_t gap = nowMs - lastClockMs_;

    // Several clock messages can be drained in one audio block and therefore
    // share a timestamp. Ignore those instead of treating them as an invalid
    // interval and wiping the history.
    if(gap == 0)
        return;

    if(gap > kMaxIntervalMs)
    {
        count_       = 0;
        index_       = 0;
        lastClockMs_ = nowMs;
        return;
    }

    lastClockMs_       = nowMs;
    intervals_[index_] = gap;
    index_             = (index_ + 1) % kHistory;
    if(count_ < kHistory)
        ++count_;

    // Wait for a full set of intervals before trusting the measurement.
    if(count_ < kHistory)
        return;

    uint32_t sum = 0;
    for(int i = 0; i < kHistory; ++i)
        sum += intervals_[i];

    if(sum > 0)
    {
        const float average = static_cast<float>(sum)
                              / static_cast<float>(kHistory);
        bpm_ = fclamp(60000.0f / (average * 24.0f), kMinBpm, kMaxBpm);
    }
}

bool MidiClockTempo::IsActive(uint32_t nowMs) const
{
    if(!hasClock_)
        return false;
    return (nowMs - lastClockMs_) < kTimeoutMs;
}

void MidiCcController::Reset()
{
    for(int i = 0; i < KNOB_COUNT; ++i)
    {
        knobOverridden_[i] = false;
        knobValue_[i]      = 0.0f;
    }
    for(int i = 0; i < SWITCH_COUNT; ++i)
    {
        switchOverridden_[i] = false;
        switchPosition_[i]   = SWITCH_CENTER;
    }
    for(int i = 0; i < FOOTSWITCH_COUNT; ++i)
    {
        footswitchOverridden_[i] = false;
        footswitchPressed_[i]    = false;
    }
}

int MidiCcController::SwitchPositionFromValue(uint8_t value)
{
    if(value <= 42)
        return SWITCH_LEFT;
    if(value <= 85)
        return SWITCH_CENTER;
    return SWITCH_RIGHT;
}

bool MidiCcController::ControlChange(uint8_t control, uint8_t value)
{
    if(control >= KNOB_1_CC && control < KNOB_1_CC + KNOB_COUNT)
    {
        const int index        = control - KNOB_1_CC;
        knobOverridden_[index] = true;
        knobValue_[index]      = static_cast<float>(value) / 127.0f;
        return true;
    }

    if(control >= SWITCH_1_CC && control < SWITCH_1_CC + SWITCH_COUNT)
    {
        const int index          = control - SWITCH_1_CC;
        switchOverridden_[index] = true;
        switchPosition_[index]   = SwitchPositionFromValue(value);
        return true;
    }

    if(control >= FOOTSWITCH_1_CC
       && control < FOOTSWITCH_1_CC + FOOTSWITCH_COUNT)
    {
        const int index              = control - FOOTSWITCH_1_CC;
        footswitchOverridden_[index] = true;
        footswitchPressed_[index]    = value >= 64;
        return true;
    }

    return false;
}

bool MidiCcController::KnobOverridden(int index) const
{
    if(index < 0 || index >= KNOB_COUNT)
        return false;
    return knobOverridden_[index];
}

float MidiCcController::KnobValue(int index) const
{
    if(index < 0 || index >= KNOB_COUNT)
        return 0.0f;
    return knobValue_[index];
}

bool MidiCcController::SwitchOverridden(int index) const
{
    if(index < 0 || index >= SWITCH_COUNT)
        return false;
    return switchOverridden_[index];
}

int MidiCcController::SwitchPosition(int index) const
{
    if(index < 0 || index >= SWITCH_COUNT)
        return SWITCH_CENTER;
    return switchPosition_[index];
}

bool MidiCcController::FootswitchOverridden(int index) const
{
    if(index < 0 || index >= FOOTSWITCH_COUNT)
        return false;
    return footswitchOverridden_[index];
}

bool MidiCcController::FootswitchPressed(int index) const
{
    if(index < 0 || index >= FOOTSWITCH_COUNT)
        return false;
    return footswitchPressed_[index];
}

void MidiCcController::ReleaseKnob(int index)
{
    if(index >= 0 && index < KNOB_COUNT)
        knobOverridden_[index] = false;
}

void MidiCcController::ReleaseSwitch(int index)
{
    if(index >= 0 && index < SWITCH_COUNT)
        switchOverridden_[index] = false;
}

void MidiCcController::ReleaseFootswitch(int index)
{
    if(index >= 0 && index < FOOTSWITCH_COUNT)
        footswitchOverridden_[index] = false;
}

void TremoloEngine::Init(float sampleRate)
{
    sampleRate_ = sampleRate > 0.0f ? sampleRate : 48000.0f;

    parameterCoefficient_
        = 1.0f - expf(-1.0f / (kSmoothingSeconds * sampleRate_));
    mixCoefficient_ = parameterCoefficient_;
    gainCoefficient_ = 1.0f - expf(-1.0f / (kGainSmoothingSeconds * sampleRate_));
    envelopeAttackCoefficient_
        = 1.0f - expf(-1.0f / (kEnvelopeAttackSeconds * sampleRate_));
    envelopeReleaseCoefficient_
        = 1.0f - expf(-1.0f / (kEnvelopeReleaseSeconds * sampleRate_));

    targetTempo_         = 120.0f;
    division_            = DIV_QUARTER;
    shape_               = WAVE_SINE;
    targetDepth_         = 0.0f;
    targetSpread_        = 0.0f;
    targetModAmount_     = 0.0f;
    targetModMultiplier_ = ModRateMultiplierFromKnob(0.5f);
    modWaveform_         = MOD_SINE;
    modTarget_           = TARGET_DEPTH;
    effectOrder_          = EFFECT_ORDER_OFF;
    targetInputGain_     = 1.0f;
    bypassed_            = true;

    retriggerEnabled_    = false;
    retriggerArmed_      = true;
    retriggerThreshold_  = kDefaultRetriggerThreshold;
    envelope_            = 0.0f;

    Reset();
}

void TremoloEngine::Reset()
{
    smoothedDepth_     = targetDepth_;
    smoothedSpread_    = targetSpread_;
    smoothedModAmount_ = targetModAmount_;
    smoothedModRate_
        = PrimaryFrequency(targetTempo_, division_) * targetModMultiplier_;
    smoothedGainL_ = 1.0f;
    smoothedGainR_ = 1.0f;
    smoothedInputGain_ = targetInputGain_;

    effectiveDepth_  = targetDepth_;
    effectiveSpread_ = targetSpread_;
    effectiveRate_   = 1.0f;

    primaryPhase_   = 0.0f;
    secondaryPhase_ = 0.0f;

    randomState_  = 0x12345678u;
    randomLeft_   = NextRandom();
    randomRight_  = NextRandom();

    previousRightPhase_ = fastmod1f(smoothedSpread_ * kSpreadPhaseCycles);

    mix_ = bypassed_ ? 0.0f : 1.0f;
}

void TremoloEngine::SetTempo(float bpm)
{
    targetTempo_ = fclamp(bpm, kMinBpm, kMaxBpm);
}

void TremoloEngine::SetDivision(int division)
{
    if(division < 0)
        division = 0;
    if(division >= DIV_COUNT)
        division = DIV_COUNT - 1;
    division_ = division;
}

void TremoloEngine::SetDepth(float depth)
{
    targetDepth_ = Clamp01(depth);
}

void TremoloEngine::SetShape(int waveform)
{
    if(waveform < 0)
        waveform = 0;
    if(waveform >= WAVE_COUNT)
        waveform = WAVE_COUNT - 1;
    shape_ = waveform;
}

void TremoloEngine::SetSpread(float spread)
{
    targetSpread_ = Clamp01(spread);
}

void TremoloEngine::SetModAmount(float amount)
{
    targetModAmount_ = Clamp01(amount);
}

void TremoloEngine::SetModRate(float knob)
{
    targetModMultiplier_ = ModRateMultiplierFromKnob(knob);
}

void TremoloEngine::SetGainCompensation(float knob)
{
    targetInputGain_ = InputGainFromKnob(knob);
}

void TremoloEngine::SetModWaveform(int waveform)
{
    if(waveform < 0)
        waveform = 0;
    if(waveform >= MOD_COUNT)
        waveform = MOD_COUNT - 1;
    modWaveform_ = waveform;
}

void TremoloEngine::SetModTarget(int target)
{
    if(target < 0)
        target = 0;
    if(target >= TARGET_COUNT)
        target = TARGET_COUNT - 1;
    modTarget_ = target;
}

void TremoloEngine::SetEffectOrder(int order)
{
    if(order < 0)
        order = 0;
    if(order >= EFFECT_ORDER_COUNT)
        order = EFFECT_ORDER_COUNT - 1;
    effectOrder_ = order;
}

void TremoloEngine::Process(float inL, float inR, float* outL, float* outR)
{
    fonepole(smoothedInputGain_, targetInputGain_, gainCoefficient_);
    inL *= smoothedInputGain_;
    inR *= smoothedInputGain_;

    UpdateEnvelope(inL, inR);

    const float targetMix = bypassed_ ? 0.0f : 1.0f;
    const float targetModRate
        = PrimaryFrequency(targetTempo_, division_) * targetModMultiplier_;

    fonepole(smoothedDepth_, targetDepth_, parameterCoefficient_);
    fonepole(smoothedSpread_, targetSpread_, parameterCoefficient_);
    fonepole(smoothedModAmount_, targetModAmount_, parameterCoefficient_);
    fonepole(smoothedModRate_, targetModRate, parameterCoefficient_);
    fonepole(mix_, targetMix, mixCoefficient_);

    const float secondaryIncrement = smoothedModRate_ / sampleRate_;

    const float modLfo = ModWaveformValue(modWaveform_, secondaryPhase_);
    const float modValue = smoothedModAmount_ * modLfo;

    float depthEff  = smoothedDepth_;
    float spreadEff = smoothedSpread_;
    float rateScale = 1.0f;

    switch(modTarget_)
    {
        case TARGET_DEPTH_RATE:
            depthEff += modValue;
            rateScale = exp2f(modValue * kRateModOctaves);
            break;
        case TARGET_RATE:
            rateScale = exp2f(modValue * kRateModOctaves);
            break;
        case TARGET_DEPTH:
        default: depthEff += modValue; break;
    }

    depthEff  = Clamp01(depthEff);
    spreadEff = Clamp01(spreadEff);

    // Smooth the effective values too, so changing the modulation target or
    // hitting a square modulation edge does not produce a click.
    fonepole(effectiveDepth_, depthEff, parameterCoefficient_);
    fonepole(effectiveSpread_, spreadEff, parameterCoefficient_);
    fonepole(effectiveRate_, rateScale, parameterCoefficient_);

    depthEff  = effectiveDepth_;
    spreadEff = effectiveSpread_;

    const float primaryFrequency
        = PrimaryFrequency(targetTempo_, division_) * effectiveRate_;
    const float primaryIncrement = primaryFrequency / sampleRate_;

    const float phaseL = primaryPhase_;
    const float phaseR
        = fastmod1f(primaryPhase_ + spreadEff * kSpreadPhaseCycles);

    const float lfoL = WaveformValue(shape_, phaseL, randomLeft_);
    const float lfoR = WaveformValue(shape_, phaseR, randomRight_);

    const float targetGainL = fclamp(
        1.0f - depthEff * (1.0f - lfoL) * 0.5f, 0.0f, 1.0f);
    const float targetGainR = fclamp(
        1.0f - depthEff * (1.0f - lfoR) * 0.5f, 0.0f, 1.0f);

    fonepole(smoothedGainL_, targetGainL, gainCoefficient_);
    fonepole(smoothedGainR_, targetGainR, gainCoefficient_);

    const float gainL = smoothedGainL_;
    const float gainR = smoothedGainR_;

    float wetL = inL * gainL;
    float wetR = inR * gainR;

    ApplySpread(&wetL, &wetR, spreadEff);

    *outL = inL + (wetL - inL) * mix_;
    *outR = inR + (wetR - inR) * mix_;

    previousRightPhase_ = phaseR;

    primaryPhase_ += primaryIncrement;
    if(primaryPhase_ >= 1.0f)
    {
        primaryPhase_ -= floorf(primaryPhase_);
        randomLeft_ = NextRandom();
    }

    secondaryPhase_ += secondaryIncrement;
    if(secondaryPhase_ >= 1.0f)
        secondaryPhase_ -= floorf(secondaryPhase_);

    const float nextRightPhase
        = fastmod1f(primaryPhase_ + spreadEff * kSpreadPhaseCycles);
    if(nextRightPhase < previousRightPhase_)
        randomRight_ = NextRandom();
}

void TremoloEngine::UpdateEnvelope(float inL, float inR)
{
    // Track the peak amplitude of the two input channels. A fast attack keeps
    // note starts sharp; a slow release prevents the follower from chattering
    // between the individual cycles of a low-frequency input.
    const float amplitude = fmaxf(fabsf(inL), fabsf(inR));
    const float coefficient
        = amplitude > envelope_ ? envelopeAttackCoefficient_
                                : envelopeReleaseCoefficient_;
    fonepole(envelope_, amplitude, coefficient);

    if(!retriggerEnabled_)
        return;

    // Re-arm only after the signal falls back below the threshold, so a
    // sustained note restarts the LFO once rather than on every sample.
    if(!retriggerArmed_)
    {
        if(envelope_ < retriggerThreshold_ * kEnvelopeRearmRatio)
            retriggerArmed_ = true;
        return;
    }

    if(envelope_ >= retriggerThreshold_)
    {
        RestartPrimaryLfo();
        retriggerArmed_ = false;
    }
}

void TremoloEngine::RestartPrimaryLfo()
{
    primaryPhase_      = 0.0f;
    previousRightPhase_ = 0.0f;
    randomLeft_        = NextRandom();
    randomRight_       = NextRandom();
}

float TremoloEngine::Clamp01(float value)
{
    return fclamp(value, 0.0f, 1.0f);
}

float TremoloEngine::InputGainFromKnob(float knob)
{
    const float decibels = fclamp(knob, 0.0f, 1.0f) * kMaxGainCompensationDb;
    return powf(10.0f, decibels / 20.0f);
}

float TremoloEngine::DivisionMultiplier(int division)
{
    switch(division)
    {
        case DIV_DOTTED_QUARTER: return 2.0f / 3.0f;
        case DIV_EIGHTH: return 2.0f;
        case DIV_DOTTED_EIGHTH: return 4.0f / 3.0f;
        case DIV_SIXTEENTH: return 4.0f;
        case DIV_DOTTED_SIXTEENTH: return 8.0f / 3.0f;
        case DIV_QUARTER:
        default: return 1.0f;
    }
}

float TremoloEngine::PrimaryFrequency(float bpm, int division)
{
    return (bpm / 60.0f) * DivisionMultiplier(division);
}

float TremoloEngine::ModRateMultiplierFromKnob(float knob)
{
    knob = Clamp01(knob);
    if(knob <= 0.5f)
        return powf(kModRateSlowMultiplier, 1.0f - 2.0f * knob);
    return powf(kModRateFastMultiplier, 2.0f * (knob - 0.5f));
}

int TremoloEngine::DivisionFromKnob(float knob, int currentDivision)
{
    knob = Clamp01(knob);
    if(currentDivision < 0)
        currentDivision = 0;
    if(currentDivision >= DIV_COUNT)
        currentDivision = DIV_COUNT - 1;

    const float hysteresis = 0.03f;
    int         division   = currentDivision;

    while(division < DIV_COUNT - 1
          && knob > static_cast<float>(division + 1)
                        / static_cast<float>(DIV_COUNT)
                        + hysteresis)
        ++division;

    while(division > 0
          && knob < static_cast<float>(division)
                        / static_cast<float>(DIV_COUNT)
                        - hysteresis)
        --division;

    return division;
}

float TremoloEngine::WaveformValue(int waveform, float phase,
                                   float randomValue)
{
    const float p = fastmod1f(phase);

    switch(waveform)
    {
        case WAVE_TRIANGLE:
            return 1.0f
                   - 4.0f * fabsf(fastmod1f(p + 0.25f) - 0.5f);
        case WAVE_SAW: return 2.0f * p - 1.0f;
        case WAVE_REVERSE_SAW: return 1.0f - 2.0f * p;
        case WAVE_SQUARE: return p < 0.5f ? 1.0f : -1.0f;
        case WAVE_RANDOM: return randomValue;
        case WAVE_SINE:
        default: return sinf(TWOPI_F * p);
    }
}

int TremoloEngine::ShapeFromKnob(float knob, int currentShape)
{
    knob = Clamp01(knob);
    if(currentShape < 0)
        currentShape = 0;
    if(currentShape >= WAVE_COUNT)
        currentShape = WAVE_COUNT - 1;

    const float hysteresis = 0.03f;
    int         shape      = currentShape;

    while(shape < WAVE_COUNT - 1
          && knob > static_cast<float>(shape + 1)
                        / static_cast<float>(WAVE_COUNT)
                        + hysteresis)
        ++shape;

    while(shape > 0
          && knob < static_cast<float>(shape)
                        / static_cast<float>(WAVE_COUNT)
                        - hysteresis)
        --shape;

    return shape;
}

float TremoloEngine::SpreadSideGain(float spread)
{
    // The side level rises logarithmically from unity (equal to mid) at 0 to
    // 1.5x the mid level at full spread.
    return powf(kSpreadSideMaxGain, fclamp(spread, 0.0f, 1.0f));
}

void TremoloEngine::ApplySpread(float* left, float* right, float spread)
{
    const float sideGain = SpreadSideGain(spread);
    const float mid      = (*left + *right) * 0.5f;
    const float side     = (*left - *right) * 0.5f;

    *left  = mid + sideGain * side;
    *right = mid - sideGain * side;
}

uint32_t TremoloEngine::NextRandomBits(uint32_t state)
{
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

float TremoloEngine::NextRandom()
{
    randomState_ = NextRandomBits(randomState_);
    const float unit
        = static_cast<float>(randomState_ >> 8) / 16777216.0f;
    return unit * 2.0f - 1.0f;
}
