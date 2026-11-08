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

// Maps incoming MIDI Control Change messages onto the pedal controls. A
// received value overrides the corresponding physical knob, switch, or
// footswitch so the internal parameter follows MIDI rather than the hardware
// position. Moving the physical control (or calling the matching Release
// method) hands control back to the hardware.
class MidiCcController
{
  public:
    enum
    {
        KNOB_COUNT       = 6,
        SWITCH_COUNT     = 3,
        FOOTSWITCH_COUNT = 2,

        KNOB_1_CC        = 21,
        SWITCH_1_CC      = 27,
        FOOTSWITCH_1_CC  = 30,
        FOOTSWITCH_2_CC  = 31,

        SWITCH_LEFT   = 0,
        SWITCH_CENTER = 1,
        SWITCH_RIGHT  = 2
    };

    MidiCcController() { Reset(); }

    void Reset();

    // Applies a Control Change message. Returns true when the controller
    // number maps to a pedal parameter.
    bool ControlChange(uint8_t control, uint8_t value);

    bool  KnobOverridden(int index) const;
    float KnobValue(int index) const;

    bool SwitchOverridden(int index) const;
    int  SwitchPosition(int index) const;

    bool FootswitchOverridden(int index) const;
    bool FootswitchPressed(int index) const;

    void ReleaseKnob(int index);
    void ReleaseSwitch(int index);
    void ReleaseFootswitch(int index);

    // Splits the 0..127 CC range into three equal bands.
    static int SwitchPositionFromValue(uint8_t value);

  private:
    bool  knobOverridden_[KNOB_COUNT];
    float knobValue_[KNOB_COUNT];

    bool switchOverridden_[SWITCH_COUNT];
    int  switchPosition_[SWITCH_COUNT];

    bool footswitchOverridden_[FOOTSWITCH_COUNT];
    bool footswitchPressed_[FOOTSWITCH_COUNT];
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

    enum EffectOrder
    {
        EFFECT_ORDER_EFFECT_FIRST,
        EFFECT_ORDER_TREMOLO_FIRST,
        EFFECT_ORDER_OFF,
        EFFECT_ORDER_COUNT
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
    void SetEffectOrder(int order);
    void SetGainCompensation(float knob);
    void SetBypassed(bool bypassed) { bypassed_ = bypassed; }
    void SetRetriggerEnabled(bool enabled) { retriggerEnabled_ = enabled; }
    void SetRetriggerThreshold(float threshold)
    {
        retriggerThreshold_ = Clamp01(threshold);
    }

    int   GetDivision() const { return division_; }
    int   GetShape() const { return shape_; }
    int   GetEffectOrder() const { return effectOrder_; }
    float GetDepth() const { return smoothedDepth_; }
    float GetModRate() const { return smoothedModRate_; }
    float GetPrimaryPhase() const { return primaryPhase_; }
    float GetSecondaryPhase() const { return secondaryPhase_; }
    float GetRightPhase() const { return previousRightPhase_; }
    float GetEnvelope() const { return envelope_; }
    float GetSampleRate() const { return sampleRate_; }
    float GetGainCompensation() const { return smoothedInputGain_; }


    void Process(float inL, float inR, float* outL, float* outR);

    static float Clamp01(float value);
    static float InputGainFromKnob(float knob);
    static float DivisionMultiplier(int division);
    static float PrimaryFrequency(float bpm, int division);
    static float ModRateMultiplierFromKnob(float knob);
    static int   DivisionFromKnob(float knob, int currentDivision);
    static int   ShapeFromKnob(float knob, int currentShape);
    static float WaveformValue(int waveform, float phase, float randomValue);
    static float SpreadSideGain(float spread);
    static void  ApplySpread(float* left, float* right, float spread);

  private:
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
    float targetInputGain_;
    float smoothedInputGain_;
    float effectiveDepth_;
    float effectiveSpread_;
    float effectiveRate_;

    int shape_;
    int modWaveform_;
    int modTarget_;
    int effectOrder_;

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
