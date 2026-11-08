#include "vibrato.h"

#include <cmath>

#include "Utility/dsp.h"

void Vibrato::Init(float sampleRate)
{
    sampleRate_ = sampleRate > 0.0f ? sampleRate : 48000.0f;

    baseDelaySamples_ = kBaseDelayMs * 0.001f * sampleRate_;
    modRangeSamples_  = kMaxModDelayMs * 0.001f * sampleRate_;

    lfoDepth_     = 0.5f;

    lfo_.Init(sampleRate_);
    lfo_.SetWaveform(daisysp::Oscillator::WAVE_SIN);
    lfo_.SetAmp(1.0f);
    lfo_.SetFreq(0.0f);

    delay_.Init();
    delay_.SetDelay(baseDelaySamples_);
}

float Vibrato::Process(float in)
{
    const float lfo = ProcessLfo();

    float delaySamples
        = baseDelaySamples_ + lfoDepth_ * modRangeSamples_ * lfo;
    delaySamples = daisysp::fclamp(delaySamples,
                                   1.0f,
                                   static_cast<float>(kDelayLength - 2));

    delay_.SetDelay(delaySamples);
    const float out = delay_.Read();
    delay_.Write(in);

    return out;
}

void Vibrato::SetLfoFreq(float freq)
{
    freq = daisysp::fclamp(freq, 0.0f, sampleRate_ * 0.5f);
    lfo_.SetFreq(freq);
}

void Vibrato::SetLfoDepth(float depth)
{
    lfoDepth_ = daisysp::fclamp(depth, 0.0f, 1.0f);
}

float Vibrato::ProcessLfo()
{
    return lfo_.Process();
}
