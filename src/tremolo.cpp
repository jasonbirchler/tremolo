#include "daisy_petal.h"
#include "daisysp.h"
#include "pedal.h"

using namespace daisy;

namespace
{
DaisyPetal hw;
Pedal      pedal(hw);

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    pedal.ProcessBlock(in, out, size);
}
} // namespace

int main(void)
{
    hw.Init();
    hw.SetAudioBlockSize(48);

    pedal.Init(hw.AudioSampleRate());

    hw.InitMidi();
    hw.midi.StartReceive();

    hw.StartAdc();
    hw.StartAudio(AudioCallback);

    while(1)
    {
        pedal.ServiceBackground(System::GetNow());
        System::Delay(1);
    }
}
