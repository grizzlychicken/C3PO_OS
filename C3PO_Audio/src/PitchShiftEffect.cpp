
#include "PitchShiftEffect.h"

void PitchShiftEffect::update(void)
{
    audio_block_t * input_block = receiveReadOnly(0);
    if (!input_block) {
        return;
    }

    audio_block_t * output_block = allocate();
    if (!output_block) {
        release(input_block);
        return;
    }

    process(AUDIO_BLOCK_SAMPLES, input_block->data, output_block->data);

    transmit(output_block);

    release(input_block);
    release(output_block);
}

void PitchShiftEffect::process(long sampleCount, int16_t * indata, int16_t * outdata)
{
    for (long i = 0; i < sampleCount; i++) {
        /* As long as we have not yet collected enough data just read in */
        _gInFIFO[_gRover] = (float)(indata[i]) / (float)INT16_MAX;                      // convert from int16_t to float
        outdata[i] = (int16_t)(_gOutFIFO[_gRover - _inFifoLatency] * (float)INT16_MAX); // convert from float to int16_t
        _gRover++;

        /* now we have enough data for processing */
        if (_gRover >= _fftFrameSize) {
            _gRover = _inFifoLatency;

            /* do windowing and re,im interleave */
            for (long k = 0; k < _fftFrameSize; k++) {
                _window = -0.5f * cosf(2.0f * M_PI * (float)k / (float)_fftFrameSize) + 0.5f;
                _gFFTworksp[2 * k] = _gInFIFO[k] * _window;
                _gFFTworksp[2 * k + 1] = 0.;
            }


            /* ***************** ANALYSIS ******************* */
            /* do transform */
            process_fft(_gFFTworksp, _fftFrameSize, -1);

            /* this is the analysis step */
            for (long k = 0; k <= _fftFrameSize2; k++) {
                /* de-interlace FFT buffer */
                _real = _gFFTworksp[2 * k];
                _imag = _gFFTworksp[2 * k + 1];

                /* compute magnitude and phase */
                _magn = 2. * sqrtf(_real * _real + _imag * _imag);
                _phase = atan2f(_imag, _real);

                /* compute phase difference */
                _tmp = _phase - _gLastPhase[k];
                _gLastPhase[k] = _phase;

                /* subtract expected phase difference */
                _tmp -= (float)k * _expct;

                /* map delta phase into +/- Pi interval */
                _qpd = _tmp / M_PI;
                if (_qpd >= 0)
                    _qpd += _qpd & 1;
                else
                    _qpd -= _qpd & 1;
                _tmp -= M_PI * (float)_qpd;

                /* get deviation from bin frequency from the +/- Pi interval */
                _tmp = _osamp * _tmp / (2. * M_PI);

                /* compute the k-th partials' true frequency */
                _tmp = (float)k * _freqPerBin + _tmp * _freqPerBin;

                /* store magnitude and true frequency in analysis arrays */
                _gAnaMagn[k] = _magn;
                _gAnaFreq[k] = _tmp;
            }

            /* ***************** PROCESSING ******************* */
            /* this does the actual pitch shifting */
            memset(_gSynMagn, 0, _fftFrameSize * sizeof(float));
            memset(_gSynFreq, 0, _fftFrameSize * sizeof(float));
            for (long k = 0; k <= _fftFrameSize2; k++) {
                _indexP = k * _pitchShift;
                if (_indexP <= _fftFrameSize2) {
                    _gSynMagn[_indexP] += _gAnaMagn[k];
                    _gSynFreq[_indexP] = _gAnaFreq[k] * _pitchShift;
                }
            }

            /* ***************** SYNTHESIS ******************* */
            /* this is the synthesis step */
            for (long k = 0; k <= _fftFrameSize2; k++) {
                /* get magnitude and true frequency from synthesis arrays */
                _magn = _gSynMagn[k];
                _tmp = _gSynFreq[k];

                /* subtract bin mid frequency */
                _tmp -= (float)k * _freqPerBin;

                /* get bin deviation from freq deviation */
                _tmp /= _freqPerBin;

                /* take osamp into account */
                _tmp = 2.0f * (float)M_PI * _tmp / _osamp;

                /* add the overlap phase advance back in */
                _tmp += (float)k * _expct;

                /* accumulate delta phase to get bin phase */
                _gSumPhase[k] += _tmp;
                _phase = _gSumPhase[k];

                /* get real and imag part and re-interleave */
                _gFFTworksp[2 * k] = _magn * cosf(_phase);
                _gFFTworksp[2 * k + 1] = _magn * sinf(_phase);
            }

            /* zero negative frequencies */
            for (long k = _fftFrameSize + 2; k < 2 * _fftFrameSize; k++) {
                _gFFTworksp[k] = 0.0f;
            }

            /* do inverse transform */
            process_fft(_gFFTworksp, _fftFrameSize, 1);

            /* do windowing and add to output accumulator */
            for (long k = 0; k < _fftFrameSize; k++) {
                _window = -0.5f * cosf(2.0f * (float)M_PI * (float)k / (float)_fftFrameSize) + 0.5f;
                _gOutputAccum[k] += 2.0f * _window * _gFFTworksp[2 * k] / (_fftFrameSize2 * _osamp);
            }

            // I think this loop can be folded into the one above?
            for (long k = 0; k < _stepSize; k++) {
                _gOutFIFO[k] = _gOutputAccum[k];
            }

            /* shift accumulator */
            memmove(_gOutputAccum, _gOutputAccum + _stepSize, _fftFrameSize * sizeof(float));

            /* move input FIFO */
            for (long k = 0; k < _inFifoLatency; k++) {
                _gInFIFO[k] = _gInFIFO[k + _stepSize];
            }
        }
    }
}

void PitchShiftEffect::process_fft(float * fftBuffer, long fftFrameSize, long sign)
{
    float wr, wi, arg, *p1, *p2, temp;
    float tr, ti, ur, ui, *p1r, *p1i, *p2r, *p2i;

    for (long i = 2; i < 2 * fftFrameSize - 2; i += 2) {
        long j = 0;
        for (long bitm = 2; bitm < 2 * fftFrameSize; bitm <<= 1) {
            if (i & bitm) {
                j++;
            }
            j <<= 1;
        }
        if (i < j) {
            p1 = fftBuffer + i;
            p2 = fftBuffer + j;
            temp = *p1;
            *(p1++) = *p2;
            *(p2++) = temp;
            temp = *p1;
            *p1 = *p2;
            *p2 = temp;
        }
    }

    long le = 2;
    long le2 = 0;
    for (long k = 0; k < (long)(log(fftFrameSize) / log(2.) + .5); k++) {
        le <<= 1;
        le2 = le >> 1;
        ur = 1.0f;
        ui = 0.0f;
        arg = M_PI / (le2 >> 1);
        wr = cos(arg);
        wi = sign * sin(arg);
        for (long j = 0; j < le2; j += 2) {
            p1r = fftBuffer + j;
            p1i = p1r + 1;
            p2r = p1r + le2;
            p2i = p2r + 1;
            for (long i = j; i < 2 * fftFrameSize; i += le) {
                tr = *p2r * ur - *p2i * ui;
                ti = *p2r * ui + *p2i * ur;
                *p2r = *p1r - tr;
                *p2i = *p1i - ti;
                *p1r += tr;
                *p1i += ti;
                p1r += le;
                p1i += le;
                p2r += le;
                p2i += le;
            }
            tr = ur * wr - ui * wi;
            ui = ur * wi + ui * wr;
            ur = tr;
        }
    }
}

float PitchShiftEffect::smbAtan2(float x, float y)
{
    float signx;
    if (x > 0.0f) {
        signx = 1.0f;
    } else {
        signx = -1.0f;
    }

    if (x == 0.0f) {
        return 0.0f;
    }
    if (y == 0.0f) {
        return signx * (float)M_PI / 2.0f;
    }
    return atan2f(x, y);
}