#include "tremolo_engine.h"

#include <cmath>
#include <cstdio>
#include <vector>

static int failures = 0;

#define CHECK(condition)                                                       \
    do                                                                         \
    {                                                                          \
        if(!(condition))                                                       \
        {                                                                      \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);   \
            ++failures;                                                        \
        }                                                                      \
    } while(0)

#define CHECK_NEAR(actual, expected, tolerance)                                \
    do                                                                         \
    {                                                                          \
        const double a_ = (actual);                                            \
        const double e_ = (expected);                                          \
        const double t_ = (tolerance);                                         \
        if(std::fabs(a_ - e_) > t_)                                            \
        {                                                                      \
            std::printf("FAIL %s:%d: %s = %.8f, expected %.8f (tol %.8f)\n",   \
                        __FILE__, __LINE__, #actual, a_, e_, t_);              \
            ++failures;                                                        \
        }                                                                      \
    } while(0)

static void Warmup(TremoloEngine& engine, int samples, float inL, float inR)
{
    float outL = 0.0f;
    float outR = 0.0f;
    for(int i = 0; i < samples; ++i)
        engine.Process(inL, inR, &outL, &outR);
}

static bool WaitForPrimaryWrap(TremoloEngine& engine)
{
    float outL       = 0.0f;
    float outR       = 0.0f;
    float prevPhase  = engine.GetPrimaryPhase();

    for(int i = 0; i < 2500000; ++i)
    {
        engine.Process(1.0f, 1.0f, &outL, &outR);
        const float phase = engine.GetPrimaryPhase();
        if(phase < prevPhase)
            return true;
        prevPhase = phase;
    }
    return false;
}

static void TestDepthZeroIsUnity()
{
    TremoloEngine engine;
    engine.Init(48000.0f);
    engine.SetBypassed(false);
    engine.SetDepth(0.0f);
    engine.SetShape(TremoloEngine::WAVE_SINE);

    Warmup(engine, 4096, 0.5f, 0.25f);

    float outL = 0.0f;
    float outR = 0.0f;
    engine.Process(0.5f, 0.25f, &outL, &outR);

    CHECK_NEAR(outL, 0.5f, 1e-5);
    CHECK_NEAR(outR, 0.25f, 1e-5);
}

static void TestDepthOneFullRange()
{
    TremoloEngine engine;
    engine.Init(48000.0f);
    engine.SetBypassed(false);
    engine.SetDepth(1.0f);
    engine.SetShape(TremoloEngine::WAVE_SINE);
    engine.SetTempo(60.0f);
    engine.SetDivision(TremoloEngine::DIV_QUARTER);

    Warmup(engine, 8192, 1.0f, 1.0f);

    float minimum = 2.0f;
    float maximum = -1.0f;
    float outL    = 0.0f;
    float outR    = 0.0f;

    for(int i = 0; i < 96000; ++i)
    {
        engine.Process(1.0f, 1.0f, &outL, &outR);
        if(outL < minimum)
            minimum = outL;
        if(outL > maximum)
            maximum = outL;
    }

    CHECK(minimum < 0.001f);
    CHECK(maximum > 0.999f);
}

static void TestDivisionMultipliers()
{
    CHECK_NEAR(TremoloEngine::DivisionMultiplier(
                   TremoloEngine::DIV_QUARTER),
               1.0f, 1e-6);
    CHECK_NEAR(TremoloEngine::DivisionMultiplier(
                   TremoloEngine::DIV_DOTTED_QUARTER),
               2.0f / 3.0f, 1e-6);
    CHECK_NEAR(TremoloEngine::DivisionMultiplier(
                   TremoloEngine::DIV_EIGHTH),
               2.0f, 1e-6);
    CHECK_NEAR(TremoloEngine::DivisionMultiplier(
                   TremoloEngine::DIV_DOTTED_EIGHTH),
               4.0f / 3.0f, 1e-6);
    CHECK_NEAR(TremoloEngine::DivisionMultiplier(
                   TremoloEngine::DIV_SIXTEENTH),
               4.0f, 1e-6);
    CHECK_NEAR(TremoloEngine::DivisionMultiplier(
                   TremoloEngine::DIV_DOTTED_SIXTEENTH),
               8.0f / 3.0f, 1e-6);

    CHECK_NEAR(TremoloEngine::PrimaryFrequency(
                   120.0f, TremoloEngine::DIV_QUARTER),
               2.0f, 1e-6);
    CHECK_NEAR(TremoloEngine::PrimaryFrequency(
                   120.0f, TremoloEngine::DIV_EIGHTH),
               4.0f, 1e-6);
}

static void TestDivisionHysteresis()
{
    int division = 0;
    CHECK(TremoloEngine::DivisionFromKnob(0.0f, division) == 0);
    // Just past the boundary without enough margin: still division 0.
    CHECK(TremoloEngine::DivisionFromKnob(0.18f, 0) == 0);
    CHECK(TremoloEngine::DivisionFromKnob(0.20f, 0) == 1);
    // Moving back just below the boundary without margin keeps division 1.
    CHECK(TremoloEngine::DivisionFromKnob(0.16f, 1) == 1);
    CHECK(TremoloEngine::DivisionFromKnob(0.13f, 1) == 0);
    CHECK(TremoloEngine::DivisionFromKnob(1.0f, 0) == 5);
}

static void TestSpread()
{
    // Side gain rises logarithmically from parity at 0 to 1.5x at 1.
    CHECK_NEAR(TremoloEngine::SpreadSideGain(0.0f), 1.0f, 1e-6);
    CHECK_NEAR(TremoloEngine::SpreadSideGain(1.0f), 1.5f, 1e-6);
    CHECK_NEAR(TremoloEngine::SpreadSideGain(0.5f), std::sqrt(1.5f), 1e-6);
    CHECK_NEAR(TremoloEngine::SpreadSideGain(-1.0f), 1.0f, 1e-6);
    CHECK_NEAR(TremoloEngine::SpreadSideGain(2.0f), 1.5f, 1e-6);

    // Zero spread is a transparent mid/side pass.
    float left  = 0.3f;
    float right = -0.7f;
    TremoloEngine::ApplySpread(&left, &right, 0.0f);
    CHECK_NEAR(left, 0.3f, 1e-6);
    CHECK_NEAR(right, -0.7f, 1e-6);

    // Full spread boosts the side component to 1.5x the mid component.
    left  = 1.0f;
    right = 0.0f;
    TremoloEngine::ApplySpread(&left, &right, 1.0f);
    CHECK_NEAR(left, 1.25f, 1e-6);
    CHECK_NEAR(right, -0.25f, 1e-6);

    const float sampleRate = 48000.0f;

    // The right-channel LFO phase offset is linear in the spread control.
    TremoloEngine phaseEngine;
    phaseEngine.Init(sampleRate);
    phaseEngine.SetBypassed(false);
    phaseEngine.SetDepth(1.0f);
    phaseEngine.SetShape(TremoloEngine::WAVE_SINE);
    phaseEngine.SetTempo(60.0f);
    phaseEngine.SetDivision(TremoloEngine::DIV_QUARTER);
    phaseEngine.SetSpread(1.0f);

    Warmup(phaseEngine, 8192, 1.0f, 1.0f);

    const float beforePhase = phaseEngine.GetPrimaryPhase();
    float       outL        = 0.0f;
    float       outR        = 0.0f;
    phaseEngine.Process(1.0f, 1.0f, &outL, &outR);

    // Full spread is a quarter cycle (90 degrees) of right-channel offset.
    const float expectedPhase
        = std::fmod(beforePhase + 0.25f, 1.0f);
    CHECK_NEAR(phaseEngine.GetRightPhase(), expectedPhase, 1e-3);

    // At zero spread a mono input stays in phase across both channels.
    TremoloEngine monoEngine;
    monoEngine.Init(sampleRate);
    monoEngine.SetBypassed(false);
    monoEngine.SetDepth(1.0f);
    monoEngine.SetShape(TremoloEngine::WAVE_SINE);
    monoEngine.SetTempo(60.0f);
    monoEngine.SetDivision(TremoloEngine::DIV_QUARTER);
    monoEngine.SetSpread(0.0f);

    Warmup(monoEngine, 8192, 1.0f, 1.0f);
    for(int i = 0; i < 1024; ++i)
    {
        monoEngine.Process(1.0f, 1.0f, &outL, &outR);
        CHECK_NEAR(outL, outR, 1e-6);
    }
}

static void TestShapeSelection()
{
    // The knob is split into six equal bands, one per waveform.
    for(int waveform = 0; waveform < TremoloEngine::WAVE_COUNT; ++waveform)
    {
        const float knob = (static_cast<float>(waveform) + 0.5f)
                           / static_cast<float>(TremoloEngine::WAVE_COUNT);
        CHECK(TremoloEngine::ShapeFromKnob(knob, waveform) == waveform);
    }
    CHECK(TremoloEngine::ShapeFromKnob(0.0f, TremoloEngine::WAVE_SINE)
          == TremoloEngine::WAVE_SINE);
    CHECK(TremoloEngine::ShapeFromKnob(1.0f, TremoloEngine::WAVE_SINE)
          == TremoloEngine::WAVE_RANDOM);

    // Hysteresis around the sine/triangle boundary at 1/6.
    CHECK(TremoloEngine::ShapeFromKnob(0.18f, TremoloEngine::WAVE_SINE)
          == TremoloEngine::WAVE_SINE);
    CHECK(TremoloEngine::ShapeFromKnob(0.20f, TremoloEngine::WAVE_SINE)
          == TremoloEngine::WAVE_TRIANGLE);
    CHECK(TremoloEngine::ShapeFromKnob(0.16f, TremoloEngine::WAVE_TRIANGLE)
          == TremoloEngine::WAVE_TRIANGLE);
    CHECK(TremoloEngine::ShapeFromKnob(0.13f, TremoloEngine::WAVE_TRIANGLE)
          == TremoloEngine::WAVE_SINE);

    // Each waveform is evaluated directly, with no interpolation.
    const float phase = 0.137f;
    CHECK_NEAR(TremoloEngine::WaveformValue(
                   TremoloEngine::WAVE_SINE, phase, 0.0f),
               std::sin(6.28318530717958647692 * phase), 1e-6);
    CHECK_NEAR(TremoloEngine::WaveformValue(
                   TremoloEngine::WAVE_RANDOM, phase, 0.42f),
               0.42f, 1e-6);
}

static void TestRandomHoldsForOneCycle()
{
    TremoloEngine engine;
    engine.Init(48000.0f);
    engine.SetBypassed(false);
    engine.SetDepth(1.0f);
    engine.SetShape(TremoloEngine::WAVE_RANDOM);
    engine.SetTempo(60.0f);
    engine.SetDivision(TremoloEngine::DIV_QUARTER);

    Warmup(engine, 4096, 1.0f, 1.0f);

    float outL = 0.0f;
    float outR = 0.0f;

    CHECK(WaitForPrimaryWrap(engine));

    // Let the gain smoothing settle on the new random value before sampling.
    Warmup(engine, 480, 1.0f, 1.0f);

    engine.Process(1.0f, 1.0f, &outL, &outR);
    const float heldValue = outL;

    for(int i = 0; i < 46000; ++i)
    {
        engine.Process(1.0f, 1.0f, &outL, &outR);
        CHECK_NEAR(outL, heldValue, 1e-4);
    }

    bool changed = false;
    for(int cycle = 0; cycle < 4 && !changed; ++cycle)
    {
        CHECK(WaitForPrimaryWrap(engine));
        Warmup(engine, 480, 1.0f, 1.0f);
        engine.Process(1.0f, 1.0f, &outL, &outR);
        if(std::fabs(outL - heldValue) > 1e-5f)
            changed = true;
    }
    CHECK(changed);
}

static void TestModRateMapping()
{
    CHECK_NEAR(TremoloEngine::ModRateMultiplierFromKnob(-1.0f),
               1.0f / 32.0f, 1e-6);
    CHECK_NEAR(TremoloEngine::ModRateMultiplierFromKnob(0.0f),
               1.0f / 32.0f, 1e-6);
    CHECK_NEAR(TremoloEngine::ModRateMultiplierFromKnob(0.25f),
               std::sqrt(1.0f / 32.0f), 1e-6);
    CHECK_NEAR(TremoloEngine::ModRateMultiplierFromKnob(0.5f), 1.0f, 1e-6);
    CHECK_NEAR(TremoloEngine::ModRateMultiplierFromKnob(0.75f),
               std::sqrt(1.5f), 1e-6);
    CHECK_NEAR(TremoloEngine::ModRateMultiplierFromKnob(1.0f), 1.5f, 1e-6);
    CHECK_NEAR(TremoloEngine::ModRateMultiplierFromKnob(2.0f), 1.5f, 1e-6);

    TremoloEngine engine;
    engine.Init(48000.0f);
    engine.SetTempo(60.0f);
    engine.SetDivision(TremoloEngine::DIV_QUARTER);

    float outL = 0.0f;
    float outR = 0.0f;

    engine.SetModRate(0.5f);
    for(int i = 0; i < 4800; ++i)
        engine.Process(1.0f, 1.0f, &outL, &outR);
    CHECK_NEAR(engine.GetModRate(), 1.0f, 1e-4);

    engine.SetModRate(0.0f);
    for(int i = 0; i < 4800; ++i)
        engine.Process(1.0f, 1.0f, &outL, &outR);
    CHECK_NEAR(engine.GetModRate(), 1.0f / 32.0f, 1e-4);

    engine.SetModRate(1.0f);
    for(int i = 0; i < 4800; ++i)
        engine.Process(1.0f, 1.0f, &outL, &outR);
    CHECK_NEAR(engine.GetModRate(), 1.5f, 1e-4);

    engine.SetModRate(0.5f);
    engine.SetTempo(120.0f);
    for(int i = 0; i < 4800; ++i)
        engine.Process(1.0f, 1.0f, &outL, &outR);
    CHECK_NEAR(engine.GetModRate(), 2.0f, 1e-4);
}

static void TestModulationClamps()
{
    CHECK_NEAR(TremoloEngine::Clamp01(-0.5f), 0.0f, 1e-6);
    CHECK_NEAR(TremoloEngine::Clamp01(0.5f), 0.5f, 1e-6);
    CHECK_NEAR(TremoloEngine::Clamp01(1.5f), 1.0f, 1e-6);

    TremoloEngine engine;
    engine.Init(48000.0f);
    engine.SetBypassed(false);
    engine.SetTempo(30.0f); // 0.5 Hz primary
    engine.SetDivision(TremoloEngine::DIV_QUARTER);
    engine.SetDepth(0.9f);
    engine.SetShape(TremoloEngine::WAVE_SINE);
    engine.SetSpread(0.0f);
    engine.SetModAmount(1.0f);
    engine.SetModRate(0.5f); // Multiplier 1.0, so 0.5 Hz modulation LFO
    engine.SetModWaveform(TremoloEngine::MOD_REVERSE_SAW);
    engine.SetModTarget(TremoloEngine::TARGET_DEPTH);

    Warmup(engine, 8192, 0.75f, 0.75f);

    float outL = 0.0f;
    float outR = 0.0f;

    // The reverse sawtooth ramps from +1 at phase 0 to -1 at phase 1. With
    // depth 0.9 and mod amount 1.0, the effective depth is driven to zero once
    // phase >= 0.95. At 0.5 Hz the last 5% of the cycle lasts 4800 samples,
    // which is long enough for the 5 ms depth smoothing to settle.
    bool foundNegativeHalf = false;
    for(int i = 0; i < 500000; ++i)
    {
        engine.Process(0.75f, 0.75f, &outL, &outR);
        if(engine.GetSecondaryPhase() > 0.95f)
        {
            foundNegativeHalf = true;
            break;
        }
    }
    CHECK(foundNegativeHalf);

    // Let the effective depth and gain smoothing settle. This advances the
    // phase by ~0.025, so it stays below 1.0 and inside the clamped region.
    for(int i = 0; i < 2400; ++i)
        engine.Process(0.75f, 0.75f, &outL, &outR);

    CHECK(engine.GetSecondaryPhase() < 1.0f);
    CHECK_NEAR(outL, 0.75f, 1e-4);
    CHECK_NEAR(outR, 0.75f, 1e-4);
}

static void CheckRateTarget(int target)
{
    TremoloEngine engine;
    engine.Init(48000.0f);
    engine.SetBypassed(false);
    engine.SetDepth(0.0f); // Keep the gain at unity; only the phase matters.
    engine.SetShape(TremoloEngine::WAVE_SINE);
    engine.SetTempo(60.0f);
    engine.SetDivision(TremoloEngine::DIV_QUARTER);
    engine.SetModAmount(1.0f);
    engine.SetModRate(0.0f); // Slowest LFO, so the mod value is near constant
    engine.SetModWaveform(TremoloEngine::MOD_REVERSE_SAW);
    engine.SetModTarget(target);

    float outL = 0.0f;
    float outR = 0.0f;

    Warmup(engine, 8192, 1.0f, 1.0f);

    // The reverse sawtooth ramps from +1 at phase 0 down to -1 at phase 1.
    // The modulation LFO is 1/32 Hz, so over the 0.1 s measurement window the
    // mod value moves by less than 0.01 and the rate is effectively constant.

    // Wait until the secondary phase wraps back to near zero, where the mod
    // value is +1 and the rate should settle at 2x.
    while(engine.GetSecondaryPhase() > 0.005f)
        engine.Process(1.0f, 1.0f, &outL, &outR);
    for(int i = 0; i < 2400; ++i) // Let the 5 ms rate smoothing settle.
        engine.Process(1.0f, 1.0f, &outL, &outR);

    const float fastStart = engine.GetPrimaryPhase();
    for(int i = 0; i < 4800; ++i) // 0.1 s
        engine.Process(1.0f, 1.0f, &outL, &outR);
    float fastDelta = engine.GetPrimaryPhase() - fastStart;
    if(fastDelta < 0.0f)
        fastDelta += 1.0f;

    // Now wait for the secondary phase to approach 1, where the mod value is
    // -1 and the rate should settle at 0.5x.
    while(engine.GetSecondaryPhase() < 0.995f)
        engine.Process(1.0f, 1.0f, &outL, &outR);
    for(int i = 0; i < 2400; ++i)
        engine.Process(1.0f, 1.0f, &outL, &outR);

    const float slowStart = engine.GetPrimaryPhase();
    for(int i = 0; i < 4800; ++i) // 0.1 s
        engine.Process(1.0f, 1.0f, &outL, &outR);
    float slowDelta = engine.GetPrimaryPhase() - slowStart;
    if(slowDelta < 0.0f)
        slowDelta += 1.0f;

    // A 1 Hz primary advances 0.1 cycles in 0.1 s before modulation.
    CHECK_NEAR(fastDelta, 0.2f, 0.005f);
    CHECK_NEAR(slowDelta, 0.05f, 0.005f);
}

static void TestRateModulationTarget()
{
    CheckRateTarget(TremoloEngine::TARGET_RATE);
    CheckRateTarget(TremoloEngine::TARGET_DEPTH_RATE);
}

static void TestTapTempo()
{
    TapTempo tap;
    CHECK_NEAR(tap.Bpm(), 120.0f, 1e-6);

    tap.Tap(0);
    tap.Tap(500);
    CHECK_NEAR(tap.Bpm(), 120.0f, 1e-6);

    tap.Tap(1000);
    CHECK_NEAR(tap.Bpm(), 120.0f, 1e-6);

    // 100 ms interval is out of range and must be ignored.
    tap.Tap(1100);
    CHECK_NEAR(tap.Bpm(), 120.0f, 1e-6);

    // 500 ms after the ignored tap remains valid.
    tap.Tap(1600);
    CHECK_NEAR(tap.Bpm(), 120.0f, 1e-6);

    TapTempo slow;
    slow.Tap(0);
    slow.Tap(667);
    slow.Tap(1334);
    CHECK_NEAR(slow.Bpm(), 60000.0f / 667.0f, 0.01f);

    // A gap longer than 3 s resets the tap history.
    slow.Tap(5000);
    CHECK_NEAR(slow.Bpm(), 60000.0f / 667.0f, 0.01f);
}

static void TestMidiClock()
{
    MidiClockTempo clock;

    uint32_t lastTime = 0;
    for(int i = 0; i <= 25; ++i)
    {
        lastTime = static_cast<uint32_t>(
            std::floor(static_cast<double>(i) * 500.0 / 24.0 + 0.5));
        clock.Tick(lastTime);
    }

    CHECK_NEAR(clock.Bpm(), 120.0f, 0.01f);
    CHECK(clock.IsActive(lastTime + 1999));
    CHECK(!clock.IsActive(lastTime + 2000));

    // A long gap resets the history but not the last measured tempo.
    clock.Tick(lastTime + 5000);
    clock.Tick(lastTime + 5021);
    CHECK_NEAR(clock.Bpm(), 120.0f, 0.01f);
}

static void TestTempoSelection()
{
    TempoController controller;

    controller.Tap(1000);
    controller.Tap(1667);
    controller.Tap(2334);
    const float tapBpm = controller.Bpm(2400);
    CHECK_NEAR(tapBpm, 60000.0f / 667.0f, 0.01f);

    uint32_t lastClock = 0;
    for(int i = 0; i <= 25; ++i)
    {
        lastClock = 3000
                    + static_cast<uint32_t>(
                        std::floor(static_cast<double>(i) * 500.0 / 24.0
                                   + 0.5));
        controller.MidiClock(lastClock);
    }

    CHECK(controller.MidiActive(lastClock));
    CHECK_NEAR(controller.Bpm(lastClock), 120.0f, 0.01f);

    // After two seconds without MIDI clock the tap tempo becomes active again.
    CHECK(!controller.MidiActive(lastClock + 2000));
    CHECK_NEAR(controller.Bpm(lastClock + 2000), tapBpm, 0.01f);
}

static void TestEnvelopeRetrigger()
{
    const float sampleRate = 48000.0f;
    const float threshold  = 0.75f;

    TremoloEngine engine;
    engine.Init(sampleRate);
    engine.SetBypassed(false);
    engine.SetDepth(1.0f);
    engine.SetShape(TremoloEngine::WAVE_SINE);
    engine.SetTempo(60.0f);
    engine.SetDivision(TremoloEngine::DIV_QUARTER);
    engine.SetRetriggerThreshold(threshold);
    engine.SetRetriggerEnabled(true);

    float outL = 0.0f;
    float outR = 0.0f;

    // Advance the LFO roughly a quarter cycle with a signal that stays below
    // the trigger threshold.
    for(int i = 0; i < 12000; ++i)
        engine.Process(0.001f, 0.001f, &outL, &outR);

    const float phaseBefore = engine.GetPrimaryPhase();
    CHECK(phaseBefore > 0.2f);

    // A burst above the threshold restarts the LFO. The attack takes a few
    // samples, and the extra samples advance the phase only slightly.
    for(int i = 0; i < 128; ++i)
        engine.Process(1.0f, 1.0f, &outL, &outR);

    CHECK(engine.GetEnvelope() >= threshold);
    CHECK(engine.GetPrimaryPhase() < 0.01f);

    // While the input stays above the threshold the LFO keeps running instead
    // of being restarted on every sample.
    const float phaseHeld = engine.GetPrimaryPhase();
    for(int i = 0; i < 100; ++i)
        engine.Process(0.5f, 0.5f, &outL, &outR);
    CHECK(engine.GetPrimaryPhase() > phaseHeld);

    // Drop below the re-arm level, then trigger a second time.
    for(int i = 0; i < 30000; ++i)
        engine.Process(0.0f, 0.0f, &outL, &outR);
    CHECK(engine.GetEnvelope() < threshold);

    for(int i = 0; i < 128; ++i)
        engine.Process(1.0f, 1.0f, &outL, &outR);
    CHECK(engine.GetPrimaryPhase() < 0.01f);

    // With retriggering disabled the input burst must not reset the phase.
    TremoloEngine disabled;
    disabled.Init(sampleRate);
    disabled.SetBypassed(false);
    disabled.SetDepth(1.0f);
    disabled.SetTempo(60.0f);
    disabled.SetDivision(TremoloEngine::DIV_QUARTER);

    for(int i = 0; i < 12000; ++i)
        disabled.Process(0.001f, 0.001f, &outL, &outR);

    const float disabledBefore = disabled.GetPrimaryPhase();
    for(int i = 0; i < 32; ++i)
        disabled.Process(0.5f, 0.5f, &outL, &outR);

    CHECK(disabled.GetPrimaryPhase() > disabledBefore);
    CHECK(disabled.GetPrimaryPhase() > 0.2f);
}

static void TestGainCompensation()
{
    CHECK_NEAR(TremoloEngine::InputGainFromKnob(0.0f), 1.0f, 1e-6);
    CHECK_NEAR(TremoloEngine::InputGainFromKnob(1.0f),
               std::pow(10.0f, 2.0f / 20.0f), 1e-6);
    CHECK_NEAR(TremoloEngine::InputGainFromKnob(0.5f),
               std::pow(10.0f, 1.0f / 20.0f), 1e-6);
    CHECK_NEAR(TremoloEngine::InputGainFromKnob(-1.0f), 1.0f, 1e-6);
    CHECK_NEAR(TremoloEngine::InputGainFromKnob(2.0f),
               std::pow(10.0f, 2.0f / 20.0f), 1e-6);

    TremoloEngine engine;
    engine.Init(48000.0f);
    engine.SetBypassed(false);
    engine.SetDepth(0.0f);
    engine.SetSpread(0.0f);
    engine.SetGainCompensation(1.0f);

    Warmup(engine, 4800, 0.5f, 0.25f);

    float outL = 0.0f;
    float outR = 0.0f;
    engine.Process(0.5f, 0.25f, &outL, &outR);

    const float gain = std::pow(10.0f, 2.0f / 20.0f);
    CHECK_NEAR(outL, 0.5f * gain, 1e-4);
    CHECK_NEAR(outR, 0.25f * gain, 1e-4);
}

static void TestMidiCcMapping()
{
    MidiCcController midi;

    // Nothing is overridden until a mapped CC arrives.
    for(int i = 0; i < MidiCcController::KNOB_COUNT; ++i)
        CHECK(!midi.KnobOverridden(i));
    for(int i = 0; i < MidiCcController::SWITCH_COUNT; ++i)
        CHECK(!midi.SwitchOverridden(i));
    for(int i = 0; i < MidiCcController::FOOTSWITCH_COUNT; ++i)
        CHECK(!midi.FootswitchOverridden(i));

    // Knob 1..6 map to CC 21..26 and scale 0..127 to 0..1.
    for(int i = 0; i < MidiCcController::KNOB_COUNT; ++i)
    {
        CHECK(midi.ControlChange(21 + i, 127));
        CHECK(midi.KnobOverridden(i));
        CHECK_NEAR(midi.KnobValue(i), 1.0f, 1e-6);
    }
    CHECK(midi.ControlChange(21, 0));
    CHECK_NEAR(midi.KnobValue(0), 0.0f, 1e-6);
    CHECK(midi.ControlChange(22, 64));
    CHECK_NEAR(midi.KnobValue(1), 64.0f / 127.0f, 1e-6);

    // Switch 1..3 map to CC 27..29 across three equal bands.
    CHECK(midi.ControlChange(27, 0));
    CHECK(midi.SwitchOverridden(0));
    CHECK(midi.SwitchPosition(0) == MidiCcController::SWITCH_LEFT);
    CHECK(midi.ControlChange(27, 42));
    CHECK(midi.SwitchPosition(0) == MidiCcController::SWITCH_LEFT);
    CHECK(midi.ControlChange(27, 43));
    CHECK(midi.SwitchPosition(0) == MidiCcController::SWITCH_CENTER);
    CHECK(midi.ControlChange(27, 85));
    CHECK(midi.SwitchPosition(0) == MidiCcController::SWITCH_CENTER);
    CHECK(midi.ControlChange(27, 86));
    CHECK(midi.SwitchPosition(0) == MidiCcController::SWITCH_RIGHT);
    CHECK(midi.ControlChange(27, 127));
    CHECK(midi.SwitchPosition(0) == MidiCcController::SWITCH_RIGHT);

    CHECK(midi.ControlChange(28, 43));
    CHECK(midi.SwitchOverridden(1));
    CHECK(midi.SwitchPosition(1) == MidiCcController::SWITCH_CENTER);
    CHECK(midi.ControlChange(29, 86));
    CHECK(midi.SwitchOverridden(2));
    CHECK(midi.SwitchPosition(2) == MidiCcController::SWITCH_RIGHT);

    // Footswitch 1 and 2 map to CC 30 and 31, pressed at value 64.
    CHECK(midi.ControlChange(30, 63));
    CHECK(midi.FootswitchOverridden(0));
    CHECK(!midi.FootswitchPressed(0));
    CHECK(midi.ControlChange(30, 64));
    CHECK(midi.FootswitchPressed(0));
    CHECK(midi.ControlChange(31, 127));
    CHECK(midi.FootswitchOverridden(1));
    CHECK(midi.FootswitchPressed(1));

    // Unmapped controllers are ignored.
    CHECK(!midi.ControlChange(20, 127));
    CHECK(!midi.ControlChange(32, 127));
    CHECK(!midi.ControlChange(0, 127));
    CHECK(!midi.ControlChange(64, 127));

    // Releasing an override hands control back to the hardware.
    midi.ReleaseKnob(0);
    CHECK(!midi.KnobOverridden(0));
    CHECK(midi.KnobOverridden(1));
    midi.ReleaseSwitch(2);
    CHECK(!midi.SwitchOverridden(2));
    CHECK(midi.SwitchOverridden(0));
    midi.ReleaseFootswitch(0);
    CHECK(!midi.FootswitchOverridden(0));
    CHECK(midi.FootswitchOverridden(1));

    // Out-of-range indexes are harmless.
    midi.ReleaseKnob(-1);
    midi.ReleaseKnob(MidiCcController::KNOB_COUNT);
    CHECK(!midi.KnobOverridden(-1));
    CHECK(!midi.SwitchOverridden(MidiCcController::SWITCH_COUNT));
    CHECK(!midi.FootswitchOverridden(MidiCcController::FOOTSWITCH_COUNT));
}

int main()
{
    TestDepthZeroIsUnity();
    TestDepthOneFullRange();
    TestDivisionMultipliers();
    TestDivisionHysteresis();
    TestSpread();
    TestShapeSelection();
    TestRandomHoldsForOneCycle();
    TestModRateMapping();
    TestModulationClamps();
    TestRateModulationTarget();
    TestTapTempo();
    TestMidiClock();
    TestTempoSelection();
    TestEnvelopeRetrigger();
    TestGainCompensation();
    TestMidiCcMapping();

    if(failures == 0)
    {
        std::printf("All tremolo engine tests passed.\n");
        return 0;
    }

    std::printf("%d test assertion(s) failed.\n", failures);
    return 1;
}
