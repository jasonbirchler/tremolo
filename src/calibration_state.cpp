#include "calibration_state.h"
#include "pedal_constants.h"

#include <cmath>
#include <cstring>

using namespace daisy;

namespace
{
constexpr uint32_t kSaveRetryMs = 1000;

// Last 4 kB sector of the 8 MB QSPI flash. Firmware images are stored from
// the start of QSPI, so this sector survives normal reflashing and reboots.
constexpr uint32_t kCalibrationStorageAddress = 0x007FF000;

using CalibrationStorage = PersistentStorage<CalibrationSettings>;

// Mirrors PersistentStorage's internal SaveStruct so the persisted record can
// be verified after Save().
struct StoredCalibration
{
    CalibrationStorage::State state;
    CalibrationSettings     settings;
};

bool InRange(float value)
{
    // NaN fails both comparisons.
    return value >= 0.0f && value <= 1.0f;
}
} // namespace

CalibrationState::CalibrationState(QSPIHandle& qspi)
    : qspi_(qspi), storage_(qspi)
{
}

void CalibrationState::InvalidateCache()
{
    auto* mapped = static_cast<uint8_t*>(qspi_.GetData(kCalibrationStorageAddress));
    dsy_dma_invalidate_cache_for_buffer(mapped, sizeof(StoredCalibration));
}

void CalibrationState::Load()
{
    const CalibrationSettings defaults{kDefaultEnvelopeThreshold,
                                       kDefaultGainCompensation};

    InvalidateCache();
    storage_.Init(defaults, kCalibrationStorageAddress);

    CalibrationSettings settings = storage_.GetSettings();
    if(!InRange(settings.threshold))
        settings.threshold = kDefaultEnvelopeThreshold;
    if(!InRange(settings.gain))
        settings.gain = kDefaultGainCompensation;

    threshold_        = settings.threshold;
    gain_             = settings.gain;
    pendingThreshold_ = settings.threshold;
    pendingGain_      = settings.gain;
    saveThreshold_    = settings.threshold;
    saveGain_         = settings.gain;
}

bool CalibrationState::Toggle()
{
    active_ = !active_;

    if(active_)
    {
        pendingThreshold_       = threshold_;
        pendingGain_            = gain_;
        modAmountPickupPending_ = true;
        modRatePickupPending_   = true;
        return true;
    }

    saveThreshold_  = pendingThreshold_;
    saveGain_       = pendingGain_;
    saveRequested_  = true;
    return false;
}

ModValues CalibrationState::ResolveKnobs(float amountKnob, float rateKnob)
{
    if(active_)
    {
        pendingGain_      = amountKnob;
        pendingThreshold_ = rateKnob;
        return ModValues{storedModAmount_, storedModRate_};
    }

    if(modAmountPickupPending_)
    {
        if(fabsf(amountKnob - storedModAmount_) <= kKnobPickupTolerance)
            modAmountPickupPending_ = false;
    }
    else
    {
        storedModAmount_ = amountKnob;
    }

    if(modRatePickupPending_)
    {
        if(fabsf(rateKnob - storedModRate_) <= kKnobPickupTolerance)
            modRatePickupPending_ = false;
    }
    else
    {
        storedModRate_ = rateKnob;
    }

    return ModValues{storedModAmount_, storedModRate_};
}

CalibrationState::PersistStatus CalibrationState::ServicePersistence(uint32_t nowMs)
{
    if(!saveRequested_
       || static_cast<int32_t>(nowMs - nextSaveAttemptMs_) < 0)
        return PersistStatus::Idle;

    if(WriteStorage(saveThreshold_, saveGain_))
    {
        threshold_        = saveThreshold_;
        gain_             = saveGain_;
        saveRequested_    = false;
        nextSaveAttemptMs_ = 0;
        return PersistStatus::Saved;
    }

    nextSaveAttemptMs_ = nowMs + kSaveRetryMs;
    return PersistStatus::Failed;
}

bool CalibrationState::WriteStorage(float threshold, float gain)
{
    if(!InRange(threshold))
        threshold = kDefaultEnvelopeThreshold;
    if(!InRange(gain))
        gain = kDefaultGainCompensation;

    // QSPI cannot be written while the application executes from it.
    if(System::GetProgramMemoryRegion() == System::MemoryRegion::QSPI)
        return false;

    const CalibrationSettings requested{threshold, gain};
    storage_.GetSettings() = requested;
    storage_.Save();

    InvalidateCache();
    StoredCalibration readback;
    std::memcpy(&readback,
                qspi_.GetData(kCalibrationStorageAddress),
                sizeof(readback));

    const bool persisted
        = readback.state == CalibrationStorage::State::FACTORY
          || readback.state == CalibrationStorage::State::USER;
    return persisted && readback.settings == requested;
}
