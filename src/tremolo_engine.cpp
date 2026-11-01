#include "tremolo_engine.h"

#include <cmath>

namespace
{
const float kTwoPi = 6.28318530717958647692f;
const float kSmoothingSeconds = 0.005f;
const float kGainSmoothingSeconds = 0.001f;
const float kMinBpm = 30.0f;
const float kMaxBpm = 300.0f;
const float kRateModOctaves = 1.0f;

float ClampFloat(float value, float low, float high)
{
    if(value < low)
        return low;
    if(value > high)
        return high;
    return value;
}

float ModWaveformValue(int waveform, float phase)
{
    switch(waveform)
    {
        case TremoloEngine::MOD_SAW: return 2.0f * phase - 1.0f;
        case TremoloEngine::MOD_SQUARE: return phase < 0.5f ? 1.0f : -1.0f;
        case TremoloEngine::MOD_SINE:
        default: return sinf(kTwoPi * phase);
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
                bpm_ = ClampFloat(60000.0f / average, kMinBpm, kMaxBpm);
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
        bpm_ = ClampFloat(60000.0f / (average * 24.0f), kMinBpm, kMaxBpm);
    }
}

bool MidiClockTempo::IsActive(uint32_t nowMs) const
{
    if(!hasClock_)
        return false;
    return (nowMs - lastClockMs_) < kTimeoutMs;
}

void TremoloEngine::Init(float sampleRate)
{
    sampleRate_ = sampleRate > 0.0f ? sampleRate : 48000.0f;

    parameterCoefficient_
        = 1.0f - expf(-1.0f / (kSmoothingSeconds * sampleRate_));
    mixCoefficient_ = parameterCoefficient_;
    gainCoefficient_ = 1.0f - expf(-1.0f / (kGainSmoothingSeconds * sampleRate_));

    targetTempo_     = 120.0f;
    division_        = DIV_QUARTER;
    shape_           = WAVE_SINE;
    targetDepth_     = 0.0f;
    targetOffset_    = 0.0f;
    targetModAmount_ = 0.0f;
    modWaveform_     = MOD_SINE;
    modTarget_       = TARGET_DEPTH;
    bypassed_        = true;

    Reset();
}

void TremoloEngine::Reset()
{
    smoothedDepth_     = targetDepth_;
    smoothedOffset_    = targetOffset_;
    smoothedModAmount_ = targetModAmount_;
    smoothedGainL_     = 1.0f;
    smoothedGainR_     = 1.0f;

    effectiveDepth_  = targetDepth_;
    effectiveOffset_ = targetOffset_;
    effectiveRate_   = 1.0f;

    primaryPhase_   = 0.0f;
    secondaryPhase_ = 0.0f;

    randomState_  = 0x12345678u;
    randomLeft_   = NextRandom();
    randomRight_  = NextRandom();

    previousRightPhase_ = FractionalPart(smoothedOffset_ * 0.25f);

    mix_ = bypassed_ ? 0.0f : 1.0f;
}

void TremoloEngine::SetTempo(float bpm)
{
    targetTempo_ = ClampFloat(bpm, kMinBpm, kMaxBpm);
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

void TremoloEngine::SetOffset(float offset)
{
    targetOffset_ = Clamp01(offset);
}

void TremoloEngine::SetModAmount(float amount)
{
    targetModAmount_ = Clamp01(amount);
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

void TremoloEngine::Process(float inL, float inR, float* outL, float* outR)
{
    const float secondaryFrequency = (targetTempo_ / 60.0f) / 16.0f;
    const float secondaryIncrement = secondaryFrequency / sampleRate_;
    const float targetMix = bypassed_ ? 0.0f : 1.0f;

    smoothedDepth_
        += parameterCoefficient_ * (targetDepth_ - smoothedDepth_);
    smoothedOffset_
        += parameterCoefficient_ * (targetOffset_ - smoothedOffset_);
    smoothedModAmount_
        += parameterCoefficient_ * (targetModAmount_ - smoothedModAmount_);
    mix_ += mixCoefficient_ * (targetMix - mix_);

    const float modLfo = ModWaveformValue(modWaveform_, secondaryPhase_);
    const float modValue = smoothedModAmount_ * modLfo;

    float depthEff  = smoothedDepth_;
    float offsetEff = smoothedOffset_;
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
    offsetEff = Clamp01(offsetEff);

    // Smooth the effective values too, so changing the modulation target or
    // hitting a square modulation edge does not produce a click.
    effectiveDepth_
        += parameterCoefficient_ * (depthEff - effectiveDepth_);
    effectiveOffset_
        += parameterCoefficient_ * (offsetEff - effectiveOffset_);
    effectiveRate_
        += parameterCoefficient_ * (rateScale - effectiveRate_);

    depthEff  = effectiveDepth_;
    offsetEff = effectiveOffset_;

    const float primaryFrequency
        = PrimaryFrequency(targetTempo_, division_) * effectiveRate_;
    const float primaryIncrement = primaryFrequency / sampleRate_;

    const float phaseL = primaryPhase_;
    const float phaseR
        = FractionalPart(primaryPhase_ + offsetEff * 0.25f);

    const float lfoL = WaveformValue(shape_, phaseL, randomLeft_);
    const float lfoR = WaveformValue(shape_, phaseR, randomRight_);

    const float targetGainL = ClampFloat(
        1.0f - depthEff * (1.0f - lfoL) * 0.5f, 0.0f, 1.0f);
    const float targetGainR = ClampFloat(
        1.0f - depthEff * (1.0f - lfoR) * 0.5f, 0.0f, 1.0f);

    smoothedGainL_ += gainCoefficient_ * (targetGainL - smoothedGainL_);
    smoothedGainR_ += gainCoefficient_ * (targetGainR - smoothedGainR_);

    const float gainL = smoothedGainL_;
    const float gainR = smoothedGainR_;

    const float wetL = inL * gainL;
    const float wetR = inR * gainR;

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
        = FractionalPart(primaryPhase_ + offsetEff * 0.25f);
    if(nextRightPhase < previousRightPhase_)
        randomRight_ = NextRandom();
}

float TremoloEngine::Clamp01(float value)
{
    return ClampFloat(value, 0.0f, 1.0f);
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
    const float p = FractionalPart(phase);

    switch(waveform)
    {
        case WAVE_TRIANGLE:
            return 1.0f
                   - 4.0f * fabsf(FractionalPart(p + 0.25f) - 0.5f);
        case WAVE_SAW: return 2.0f * p - 1.0f;
        case WAVE_REVERSE_SAW: return 1.0f - 2.0f * p;
        case WAVE_SQUARE: return p < 0.5f ? 1.0f : -1.0f;
        case WAVE_RANDOM: return randomValue;
        case WAVE_SINE:
        default: return sinf(kTwoPi * p);
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

float TremoloEngine::FractionalPart(float value)
{
    return value - floorf(value);
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
