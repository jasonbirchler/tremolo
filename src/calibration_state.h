#pragma once
#ifndef TREMOLO_CALIBRATION_STATE_H
#define TREMOLO_CALIBRATION_STATE_H

#include "daisy_petal.h"

#include <cstdint>

struct CalibrationSettings
{
    float threshold;
    float gain;

    bool operator==(const CalibrationSettings& other) const
    {
        return threshold == other.threshold && gain == other.gain;
    }

    bool operator!=(const CalibrationSettings& other) const
    {
        return !(*this == other);
    }
};

struct ModValues
{
    float amount;
    float rate;
};

// Owns the calibration mode, the persisted envelope threshold and gain
// compensation, the pending values edited while calibrating, and the QSPI
// save/retry cycle. Calibration is entered and left by the Pedal.
class CalibrationState
{
  public:
    enum class PersistStatus
    {
        Idle,
        Saved,
        Failed
    };

    explicit CalibrationState(daisy::QSPIHandle& qspi);

    // Reads the persisted settings from QSPI. Call once before audio starts.
    void Load();

    // Flips calibration mode and returns the new state. Entering seeds the
    // pending values from the persisted ones; leaving queues a save.
    bool Toggle();

    bool Active() const { return active_; }

    // Threshold/gain the engine should use right now.
    float ActiveThreshold() const
    {
        return active_ ? pendingThreshold_ : threshold_;
    }

    float ActiveGain() const { return active_ ? pendingGain_ : gain_; }

    // Maps the two mod knobs to engine values. In calibration the knobs drive
    // the pending gain/threshold and the stored mod values are held. Outside
    // calibration the knobs must be returned to their stored positions before
    // they take effect again.
    ModValues ResolveKnobs(float amountKnob, float rateKnob);

    // Called from the main loop. Retries a queued save after a failure.
    PersistStatus ServicePersistence(uint32_t nowMs);

  private:
    bool WriteStorage(float threshold, float gain);
    void InvalidateCache();

    using Storage = daisy::PersistentStorage<CalibrationSettings>;

    daisy::QSPIHandle& qspi_;
    Storage            storage_;

    float threshold_        = 0.0f;
    float gain_             = 0.0f;
    float pendingThreshold_ = 0.0f;
    float pendingGain_      = 0.0f;

    volatile bool     saveRequested_    = false;
    float             saveThreshold_    = 0.0f;
    float             saveGain_         = 0.0f;
    uint32_t          nextSaveAttemptMs_ = 0;

    bool  active_                 = false;
    float storedModAmount_        = 0.0f;
    float storedModRate_          = 0.5f;
    bool  modAmountPickupPending_ = false;
    bool  modRatePickupPending_   = false;
};

#endif
