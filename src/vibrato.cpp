#include "vibrato.h"

#include <cmath>

namespace
{
constexpr float kTwoPi = 6.28318530717958647692f;

float ClampFloat(float value, float low, float high)
{
    if(value < low)
        return low;
    if(value > high)
        return high;
    return value;
}
} // namespace

void Vibrato::Init(float sampleRate)
{
    sampleRate_ = sampleRate > 0.0f ? sampleRate : 48000.0f;

    baseDelaySamples_ = kBaseDelayMs * 0.001f * sampleRate_;
    modRangeSamples_  = kMaxModDelayMs * 0.001f * sampleRate_;

    lfoPhase_     = 0.0f;
    lfoIncrement_ = 0.0f;
    lfoDepth_     = 0.5f;

    delay_.Init();
    delay_.SetDelay(baseDelaySamples_);
}

float Vibrato::Process(float in)
{
    const float lfo = ProcessLfo();

    float delaySamples
        = baseDelaySamples_ + lfoDepth_ * modRangeSamples_ * lfo;
    delaySamples = ClampFloat(delaySamples,
                              1.0f,
                              static_cast<float>(kDelayLength - 2));

    delay_.SetDelay(delaySamples);
    const float out = delay_.Read();
    delay_.Write(in);

    return out;
}

void Vibrato::SetLfoFreq(float freq)
{
    freq = ClampFloat(freq, 0.0f, sampleRate_ * 0.5f);
    lfoIncrement_ = freq / sampleRate_;
}

void Vibrato::SetLfoDepth(float depth)
{
    lfoDepth_ = ClampFloat(depth, 0.0f, 1.0f);
}

float Vibrato::ProcessLfo()
{
    const float value = sinf(kTwoPi * lfoPhase_);

    lfoPhase_ += lfoIncrement_;
    if(lfoPhase_ >= 1.0f)
        lfoPhase_ -= floorf(lfoPhase_);

    return value;
}
