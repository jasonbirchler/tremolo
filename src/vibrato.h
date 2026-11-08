#pragma once
#ifndef VIBRATO_H
#define VIBRATO_H

#include <cstdint>
#include "Utility/delayline.h"
#include "Synthesis/oscillator.h"

// Modulated-delay vibrato. The LFO rate is supplied by the caller so it can be
// locked to the tremolo engine's modulation LFO.
class Vibrato
{
  public:
    Vibrato() {}

    void Init(float sampleRate);

    float Process(float in);

    void SetLfoFreq(float freq);
    void SetLfoDepth(float depth);

  private:
    static constexpr int32_t kDelayLength = 2400; // 50 ms at 48 kHz
    static constexpr float   kBaseDelayMs = 5.0f;
    static constexpr float   kMaxModDelayMs = 4.0f;

    float ProcessLfo();

    float sampleRate_;
    float lfoDepth_;

    daisysp::Oscillator lfo_;

    float baseDelaySamples_;
    float modRangeSamples_;

    daisysp::DelayLine<float, kDelayLength> delay_;
};

#endif // VIBRATO_H
