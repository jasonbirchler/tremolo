#include "midi_bridge.h"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <thread>

namespace
{
int failures = 0;

#define EXPECT(cond)                                                     \
    do                                                                   \
    {                                                                    \
        if(!(cond))                                                      \
        {                                                                \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, \
                         #cond);                                         \
            ++failures;                                                  \
        }                                                                \
    } while(0)

// Every field carries the same sequence number, so a torn read (fields from
// two different writes) is detectable.
struct Payload
{
    uint64_t seq;
    uint64_t fields[7];

    bool Consistent() const
    {
        for(uint64_t f : fields)
            if(f != seq)
                return false;
        return true;
    }
};

Payload MakePayload(uint64_t seq)
{
    Payload p;
    p.seq = seq;
    for(uint64_t& f : p.fields)
        f = seq;
    return p;
}

void TestSingleThreaded()
{
    TripleBuffer<Payload> buf;

    // Reading before any write returns the default-constructed value.
    EXPECT(buf.Read().seq == 0);

    buf.Write(MakePayload(1));
    EXPECT(buf.Read().seq == 1);

    // Repeated reads with no new write return the same value.
    EXPECT(buf.Read().seq == 1);
    EXPECT(buf.Read().seq == 1);

    // Several writes without a read: the reader sees only the latest.
    buf.Write(MakePayload(2));
    buf.Write(MakePayload(3));
    buf.Write(MakePayload(4));
    const Payload latest = buf.Read();
    EXPECT(latest.seq == 4);
    EXPECT(latest.Consistent());

    // Interleaved write/read sequences keep returning the newest value.
    for(uint64_t s = 5; s < 100; ++s)
    {
        buf.Write(MakePayload(s));
        if(s % 3 == 0)
            EXPECT(buf.Read().seq == s);
    }
    EXPECT(buf.Read().seq == 99);
}

void TestMidiSnapshotRoundTrip()
{
    MidiBridge bridge;
    MidiSnapshot snap;
    snap.bpm = 137.5f;
    bridge.Publish(snap);
    EXPECT(bridge.ReadSnapshot().bpm == 137.5f);
}

// One producer publishes an ever-increasing sequence while one consumer reads
// as fast as it can. The consumer must never see a torn value, never see the
// sequence move backwards, and must see the final value after the producer
// stops.
void TestConcurrentProducerConsumer()
{
    constexpr uint64_t kWrites = 5000000;

    TripleBuffer<Payload> buf;
    std::atomic<bool>     producerDone{false};
    uint64_t              torn      = 0;
    uint64_t              backwards = 0;
    uint64_t              reads     = 0;
    uint64_t              lastSeen  = 0;

    std::thread consumer([&]() {
        while(!producerDone.load(std::memory_order_acquire))
        {
            const Payload p = buf.Read();
            ++reads;
            if(!p.Consistent())
                ++torn;
            if(p.seq < lastSeen)
                ++backwards;
            lastSeen = p.seq;
        }
        const Payload finalValue = buf.Read();
        if(!finalValue.Consistent())
            ++torn;
        if(finalValue.seq != kWrites)
            ++backwards;
    });

    std::thread producer([&]() {
        for(uint64_t s = 1; s <= kWrites; ++s)
            buf.Write(MakePayload(s));
        producerDone.store(true, std::memory_order_release);
    });

    producer.join();
    consumer.join();

    std::fprintf(stderr,
                 "concurrent: writes=%llu reads=%llu torn=%llu backwards=%llu\n",
                 static_cast<unsigned long long>(kWrites),
                 static_cast<unsigned long long>(reads),
                 static_cast<unsigned long long>(torn),
                 static_cast<unsigned long long>(backwards));

    EXPECT(torn == 0);
    EXPECT(backwards == 0);
    EXPECT(reads > 0);
}
} // namespace

int main()
{
    TestSingleThreaded();
    TestMidiSnapshotRoundTrip();
    TestConcurrentProducerConsumer();

    if(failures != 0)
    {
        std::fprintf(stderr, "%d assertion(s) failed\n", failures);
        return 1;
    }
    std::printf("triple_buffer_test: all tests passed\n");
    return 0;
}
