#pragma once
#ifndef TREMOLO_LED_CONTROLLER_H
#define TREMOLO_LED_CONTROLLER_H

#include "daisy_petal.h"

#include <cstddef>
#include <cstdint>

struct LedInputs
{
    bool  bypassed;
    bool  calibrating;
    bool  envelopeBlinkEnabled;
    float envelope;
    float envelopeThreshold;
};

// Owns both LEDs and every visual indication: the envelope blink pattern,
// the tap/quarter-note pulse, the calibration flashes and breathing, and the
// save-error flash.
class LedController
{
  public:
    static constexpr size_t kEnvelopeBlinkPhaseCount = 4;

    void Init(daisy::Pin led1Pin, daisy::Pin led2Pin);

    void PulseRight(uint32_t nowMs);
    void StartCalibrationFlash(uint32_t nowMs);
    void StartSaveErrorFlash(uint32_t nowMs);
    void ResetEnvelopeTracking(bool aboveThreshold);

    // Audio thread. Call once per block.
    void Update(uint32_t nowMs, const LedInputs& inputs);

  private:
    daisy::Led led1_;
    daisy::Led led2_;

    uint32_t led2PulseUntilMs_ = 0;

    size_t   envelopeBlinkPhase_   = kEnvelopeBlinkPhaseCount;
    uint32_t envelopeBlinkStartMs_ = 0;
    bool     envelopeAboveThreshold_ = false;

    uint32_t calibrationFlashStartMs_ = 0;
    bool     calibrationFlashActive_  = false;

    volatile bool     saveErrorActive_  = false;
    volatile uint32_t saveErrorStartMs_ = 0;
};

#endif
