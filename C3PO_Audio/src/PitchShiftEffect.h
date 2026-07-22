#ifndef PITCH_SHIFT_EFFECT_H
#define PITCH_SHIFT_EFFECT_H

#include <Arduino.h>
#include <AudioStream.h>

class PitchShiftEffect : public AudioStream {
    public:
    PitchShiftEffect()
        : AudioStream(1, inputQueueArray)
    {
        _stepSize = _fftFrameSize / PitchShiftEffect::_osamp;
        _freqPerBin = AUDIO_SAMPLE_RATE / (float)_fftFrameSize;
        _expct = 2.0f * (float)M_PI * (float)_stepSize / (float)_fftFrameSize;
        _inFifoLatency = _fftFrameSize - _stepSize;
        if (_gRover == false) {
            _gRover = _inFifoLatency;
        }
    }

    virtual void update(void);

    void setPitchAmount(float factor)
    {
        _pitchShift = factor;
    }

    private:
    audio_block_t * inputQueueArray[1] = {nullptr};

    float smbAtan2(float x, float y);

    float _pitchShift = 1.0f;
    static const long _osamp = 4;
    static const long _fftFrameSize = 1024;
    static const long _fftFrameSize2 = _fftFrameSize / 2;

    float _gInFIFO[_fftFrameSize] = {0};
    float _gOutFIFO[_fftFrameSize] = {0};
    float _gFFTworksp[_fftFrameSize * 2] = {0};
    float _gLastPhase[_fftFrameSize2 + 1] = {0};
    float _gSumPhase[_fftFrameSize2 + 1] = {0};
    float _gOutputAccum[_fftFrameSize * 2] = {0};
    float _gAnaFreq[_fftFrameSize] = {0};
    float _gAnaMagn[_fftFrameSize] = {0};
    float _gSynFreq[_fftFrameSize] = {0};
    float _gSynMagn[_fftFrameSize] = {0};

    long _gRover = false;
    float _magn = 0.0f;
    float _phase = 0.0f;
    float _tmp = 0.0f;
    float _window = 0.0f;
    float _real = 0.0f;
    float _imag = 0.0f;
    float _freqPerBin = 0.0f;
    float _expct = 0.0f;
    long _indexP = 0;
    long _qpd = 0;
    long _inFifoLatency = 0;
    long _stepSize = 0;

    public:
    void process(long numSampsToProcess, int16_t * indata, int16_t * outdata);
    void process_fft(float * fftBuffer, long fftFrameSize, long sign);
};

#endif // PITCH_SHIFT_EFFECT_H
