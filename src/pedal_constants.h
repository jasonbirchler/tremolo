#pragma once
#ifndef TREMOLO_PEDAL_CONSTANTS_H
#define TREMOLO_PEDAL_CONSTANTS_H

#include <cstdint>

constexpr uint32_t kCalibrationHoldMs        = 5000;
constexpr float    kDefaultEnvelopeThreshold = 0.5f;
constexpr float    kDefaultGainCompensation  = 0.0f;
constexpr float    kKnobPickupTolerance      = 0.02f;

#endif
