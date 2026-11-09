#include "led_controller.h"

#include "daisysp.h"

#include <cmath>

using namespace daisy;
using namespace daisysp;

namespace
{
constexpr uint32_t kTapPulseMs                = 60;
constexpr uint32_t kCalibrationFlashOnMs      = 150;
constexpr uint32_t kCalibrationFlashPeriodMs  = 300;
constexpr uint32_t kCalibrationFlashTotalMs   = 600;
constexpr uint32_t kSaveErrorFlashOnMs        = 80;
constexpr uint32_t kSaveErrorFlashPeriodMs    = 160;
constexpr uint32_t kSaveErrorFlashTotalMs     = 640;
constexpr float    kCalibrationPulseHz        = 1.0f;

constexpr uint32_t kEnvelopeBlinkDurations[LedController::kEnvelopeBlinkPhaseCount]
    = {100, 400, 100, 400};

bool FlashActive(uint32_t now,
                 uint32_t start,
                 uint32_t totalMs,
                 uint32_t periodMs,
                 uint32_t onMs)
{
    const uint32_t elapsed = now - start;
    return periodMs != 0 && elapsed < totalMs && elapsed % periodMs < onMs;
}
} // namespace

void LedController::Init(Pin led1Pin, Pin led2Pin)
{
    led1_.Init(led1Pin, false);
    led1_.Set(0.0f);
    led1_.Update();

    led2_.Init(led2Pin, false);
    led2_.Set(0.0f);
    led2_.Update();
}

void LedController::PulseRight(uint32_t nowMs)
{
    led2PulseUntilMs_ = nowMs + kTapPulseMs;
}

void LedController::StartCalibrationFlash(uint32_t nowMs)
{
    calibrationFlashStartMs_ = nowMs;
    calibrationFlashActive_  = true;
}

void LedController::StartSaveErrorFlash(uint32_t nowMs)
{
    saveErrorStartMs_ = nowMs;
    saveErrorActive_  = true;
}

void LedController::ResetEnvelopeTracking(bool aboveThreshold)
{
    envelopeBlinkPhase_     = kEnvelopeBlinkPhaseCount;
    envelopeAboveThreshold_ = aboveThreshold;
}

void LedController::Update(uint32_t now, const LedInputs& in)
{
    float led1Value = in.bypassed ? 0.0f : 1.0f;
    float led2Value = static_cast<int32_t>(led2PulseUntilMs_ - now) > 0 ? 1.0f
                                                                        : 0.0f;

    if(in.calibrating || in.envelopeBlinkEnabled)
    {
        const bool nowAbove = in.envelope >= in.envelopeThreshold;

        if(nowAbove && !envelopeAboveThreshold_)
        {
            envelopeBlinkPhase_   = 0;
            envelopeBlinkStartMs_ = now;
        }
        envelopeAboveThreshold_ = nowAbove;

        if(envelopeBlinkPhase_ < kEnvelopeBlinkPhaseCount)
        {
            led1Value = envelopeBlinkPhase_ % 2 == 0 ? 1.0f : 0.0f;
            if(now - envelopeBlinkStartMs_
               >= kEnvelopeBlinkDurations[envelopeBlinkPhase_])
            {
                ++envelopeBlinkPhase_;
                envelopeBlinkStartMs_ = now;
            }
        }
        else
        {
            led1Value = in.calibrating ? 0.0f : (in.bypassed ? 0.0f : 1.0f);
        }
    }
    else
    {
        envelopeBlinkPhase_ = kEnvelopeBlinkPhaseCount;
    }

    if(in.calibrating && calibrationFlashActive_)
    {
        if(now - calibrationFlashStartMs_ < kCalibrationFlashTotalMs)
        {
            const bool on = FlashActive(now,
                                        calibrationFlashStartMs_,
                                        kCalibrationFlashTotalMs,
                                        kCalibrationFlashPeriodMs,
                                        kCalibrationFlashOnMs);
            led1Value = on ? 1.0f : 0.0f;
            led2Value = on ? 1.0f : 0.0f;
        }
        else
        {
            calibrationFlashActive_ = false;
        }
    }

    if(in.calibrating && !calibrationFlashActive_)
    {
        const float seconds = static_cast<float>(now) * 0.001f;
        led2Value = 0.5f + 0.5f * sinf(TWOPI_F * kCalibrationPulseHz * seconds);
    }

    if(saveErrorActive_)
    {
        if(now - saveErrorStartMs_ < kSaveErrorFlashTotalMs)
        {
            const bool on = FlashActive(now,
                                        saveErrorStartMs_,
                                        kSaveErrorFlashTotalMs,
                                        kSaveErrorFlashPeriodMs,
                                        kSaveErrorFlashOnMs);
            led1Value = on ? 1.0f : 0.0f;
            led2Value = on ? 1.0f : 0.0f;
        }
        else
        {
            saveErrorActive_ = false;
        }
    }

    led1_.Set(led1Value);
    led1_.Update();

    led2_.Set(led2Value);
    led2_.Update();
}
