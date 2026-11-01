#pragma once
#ifndef TREMOLO_ENGINE_H
#define TREMOLO_ENGINE_H

#include <cstdint>

// Platform-independent tremolo engine. This deliberately depends only on the
// C++ standard library so it can be compiled and tested on the host.
class TapTempo
{
  public:
    TapTempo() { Reset(); }

    void Reset();
    void Tap(uint32_t nowMs);
    float Bpm() const { return bpm_; }

  private:
    static const int kHistory = 3;

    uint32_t intervals_[kHistory];
    int      count_;
    int      index_;
    uint32_t lastTapMs_;
    bool     hasLastTap_;
    float    bpm_;
};

class MidiClockTempo
{
  public:
    MidiClockTempo() { Reset(); }

    void Reset();
    void Tick(uint32_t nowMs);
    bool  IsActive(uint32_t nowMs) const;
    float Bpm() const { return bpm_; }

  private:
    static const int kHistory = 24;
    static const uint32_t kTimeoutMs = 2000;
    static const uint32_t kMaxIntervalMs = 1000;

    uint32_t intervals_[kHistory];
    int      count_;
    int      index_;
    uint32_t lastClockMs_;
    bool     hasClock_;
    float    bpm_;
};

// MIDI clock wins while it is present; otherwise the last tap tempo (or the
// 120 BPM default) is used.
class TempoController
{
  public:
    void  Tap(uint32_t nowMs) { tap_.Tap(nowMs); }
    void  MidiClock(uint32_t nowMs) { clock_.Tick(nowMs); }
    float Bpm(uint32_t nowMs) const
    {
        return clock_.IsActive(nowMs) ? clock_.Bpm() : tap_.Bpm();
    }
    bool MidiActive(uint32_t nowMs) const { return clock_.IsActive(nowMs); }

    TapTempo&       TapSource() { return tap_; }
    MidiClockTempo& ClockSource() { return clock_; }

  private:
    TapTempo       tap_;
    MidiClockTempo clock_;
};

class TremoloEngine
{
  public:
    enum Division
    {
        DIV_DOTTED_QUARTER = 0,
        DIV_QUARTER,
        DIV_DOTTED_EIGHTH,
        DIV_EIGHTH,
        DIV_DOTTED_SIXTEENTH,
        DIV_SIXTEENTH,
        DIV_COUNT
    };

    enum Waveform
    {
        WAVE_SINE = 0,
        WAVE_TRIANGLE,
        WAVE_SAW,
        WAVE_REVERSE_SAW,
        WAVE_SQUARE,
        WAVE_RANDOM,
        WAVE_COUNT
    };

    enum ModWaveform
    {
        MOD_SINE = 0,
        MOD_SAW,
        MOD_REVERSE_SAW,
        MOD_COUNT
    };

    enum ModTarget
    {
        TARGET_DEPTH = 0,
        TARGET_DEPTH_RATE,
        TARGET_RATE,
        TARGET_COUNT
    };

    TremoloEngine() { Init(48000.0f); }

    void Init(float sampleRate);
    void Reset();

    void SetTempo(float bpm);
    void SetDivision(int division);
    void SetDepth(float depth);
    void SetShape(int waveform);
    void SetSpread(float spread);
    void SetModAmount(float amount);
    void SetModRate(float knob);
    void SetModWaveform(int waveform);
    void SetModTarget(int target);
    void SetBypassed(bool bypassed) { bypassed_ = bypassed; }
    void SetRetriggerEnabled(bool enabled) { retriggerEnabled_ = enabled; }
    void SetRetriggerThreshold(float threshold)
    {
        retriggerThreshold_ = Clamp01(threshold);
    }

    int   GetDivision() const { return division_; }
    int   GetShape() const { return shape_; }
    float GetModRate() const { return smoothedModRate_; }
    float GetPrimaryPhase() const { return primaryPhase_; }
    float GetSecondaryPhase() const { return secondaryPhase_; }
    float GetRightPhase() const { return previousRightPhase_; }
    float GetEnvelope() const { return envelope_; }

    void Process(float inL, float inR, float* outL, float* outR);

    static float Clamp01(float value);
    static float DivisionMultiplier(int division);
    static float PrimaryFrequency(float bpm, int division);
    static float ModRateMultiplierFromKnob(float knob);
    static int   DivisionFromKnob(float knob, int currentDivision);
    static int   ShapeFromKnob(float knob, int currentShape);
    static float WaveformValue(int waveform, float phase, float randomValue);
    static float SpreadSideGain(float spread);
    static void  ApplySpread(float* left, float* right, float spread);

  private:
    static float FractionalPart(float value);
    static uint32_t NextRandomBits(uint32_t state);
    float           NextRandom();
    void            UpdateEnvelope(float inL, float inR);
    void            RestartPrimaryLfo();

    float sampleRate_;

    float targetTempo_;
    int   division_;

    float targetDepth_;
    float targetSpread_;
    float targetModAmount_;
    float targetModMultiplier_;

    float smoothedDepth_;
    float smoothedSpread_;
    float smoothedModAmount_;
    float smoothedModRate_;
    float smoothedGainL_;
    float smoothedGainR_;
    float effectiveDepth_;
    float effectiveSpread_;
    float effectiveRate_;

    int shape_;
    int modWaveform_;
    int modTarget_;

    float    primaryPhase_;
    float    secondaryPhase_;
    float    randomLeft_;
    float    randomRight_;
    float    previousRightPhase_;
    uint32_t randomState_;

    bool  bypassed_;
    float mix_;
    float mixCoefficient_;
    float parameterCoefficient_;
    float gainCoefficient_;

    bool  retriggerEnabled_;
    bool  retriggerArmed_;
    float retriggerThreshold_;
    float envelope_;
    float envelopeAttackCoefficient_;
    float envelopeReleaseCoefficient_;};

#endif // TREMOLO_ENGINE_H
