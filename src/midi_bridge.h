#pragma once
#ifndef TREMOLO_MIDI_BRIDGE_H
#define TREMOLO_MIDI_BRIDGE_H

#include "tremolo_engine.h"

#include <atomic>
#include <cstdint>

// Everything the audio thread needs from MIDI input, published once per main
// loop iteration.
struct MidiSnapshot
{
    MidiCcController controls;
    float            bpm = 120.0f;
};

// Footswitch press counts accumulated since the audio thread last took them.
struct MidiFootswitchPresses
{
    uint32_t footswitch1;
    uint32_t footswitch2;
};

// Lock-free single-producer/single-consumer latest-value buffer. The producer
// and consumer never touch the same slot, so neither side blocks or retries.
template <typename T>
class TripleBuffer
{
  public:
    TripleBuffer() : middle_(1) {}

    void Write(const T& value)
    {
        buffers_[writeIndex_] = value;
        writeIndex_
            = middle_.exchange(writeIndex_ | kDirtyFlag,
                               std::memory_order_acq_rel)
              & kIndexMask;
    }

    const T& Read()
    {
        if(middle_.load(std::memory_order_acquire) & kDirtyFlag)
            readIndex_ = middle_.exchange(readIndex_, std::memory_order_acq_rel)
                         & kIndexMask;
        return buffers_[readIndex_];
    }

  private:
    enum : uint32_t
    {
        kDirtyFlag = 0x4,
        kIndexMask = 0x3
    };

    T                     buffers_[3] = {};
    std::atomic<uint32_t> middle_;
    uint32_t              writeIndex_ = 0;
    uint32_t              readIndex_  = 2;
};

// Mailbox between the main loop (MIDI producer) and the audio callback
// (consumer of MIDI state, producer of override releases and tap requests).
class MidiBridge
{
  public:
    // Main loop side.
    void Publish(const MidiSnapshot& snapshot) { snapshots_.Write(snapshot); }

    uint32_t TakeReleaseRequests()
    {
        return releaseRequests_.exchange(0);
    }

    void PostFootswitchPress(int index)
    {
        (index == 0 ? footswitch1Presses_ : footswitch2Presses_)
            .fetch_add(1);
    }

    uint32_t TakeTapRequests() { return tapRequests_.exchange(0); }

    // Audio callback side.
    const MidiSnapshot& ReadSnapshot() { return snapshots_.Read(); }

    void RequestKnobRelease(int index)
    {
        releaseRequests_.fetch_or(1u << index);
    }

    void RequestSwitchRelease(int index)
    {
        releaseRequests_.fetch_or(1u << (MidiCcController::KNOB_COUNT + index));
    }

    MidiFootswitchPresses TakeFootswitchPresses()
    {
        return MidiFootswitchPresses{footswitch1Presses_.exchange(0),
                                     footswitch2Presses_.exchange(0)};
    }

    void RequestTap() { tapRequests_.fetch_add(1); }

  private:
    TripleBuffer<MidiSnapshot> snapshots_;

    std::atomic<uint32_t> releaseRequests_{0};
    std::atomic<uint32_t> footswitch1Presses_{0};
    std::atomic<uint32_t> footswitch2Presses_{0};
    std::atomic<uint32_t> tapRequests_{0};
};

#endif
