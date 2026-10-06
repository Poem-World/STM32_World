/**
 * @file FrequencyFilters.c
 * @brief 카테고리 2: 주파수 선택 필터 구현 (Robert Bristow-Johnson Audio EQ Cookbook 공식 기반)
 */

#include "DigitalFilters/FrequencyFilters.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void BiquadFilter_Init(BiquadFilter_t *f, BiquadFilterType_t type, float sampleRate, float cutoffFreq, float qFactor)
{
    if (!f) return;
    if (sampleRate <= 0.0f) sampleRate = 100.0f;
    if (cutoffFreq <= 0.0f) cutoffFreq = 1.0f;
    if (cutoffFreq >= (sampleRate * 0.499f)) cutoffFreq = sampleRate * 0.499f;
    if (qFactor <= 0.001f) qFactor = 0.7071f;

    f->d1 = 0.0f;
    f->d2 = 0.0f;

    float omega = 2.0f * (float)M_PI * cutoffFreq / sampleRate;
    float sinOmega = sinf(omega);
    float cosOmega = cosf(omega);
    float alpha = sinOmega / (2.0f * qFactor);

    float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f;
    float a0 = 1.0f, a1 = 0.0f, a2 = 0.0f;

    switch (type) {
    case FILTER_TYPE_LOWPASS:
        b0 = (1.0f - cosOmega) * 0.5f;
        b1 =  1.0f - cosOmega;
        b2 = (1.0f - cosOmega) * 0.5f;
        a0 =  1.0f + alpha;
        a1 = -2.0f * cosOmega;
        a2 =  1.0f - alpha;
        break;

    case FILTER_TYPE_HIGHPASS:
        b0 =  (1.0f + cosOmega) * 0.5f;
        b1 = -(1.0f + cosOmega);
        b2 =  (1.0f + cosOmega) * 0.5f;
        a0 =   1.0f + alpha;
        a1 =  -2.0f * cosOmega;
        a2 =   1.0f - alpha;
        break;

    case FILTER_TYPE_BANDPASS:
        b0 =  alpha;
        b1 =  0.0f;
        b2 = -alpha;
        a0 =  1.0f + alpha;
        a1 = -2.0f * cosOmega;
        a2 =  1.0f - alpha;
        break;

    case FILTER_TYPE_BANDSTOP:
        b0 =  1.0f;
        b1 = -2.0f * cosOmega;
        b2 =  1.0f;
        a0 =  1.0f + alpha;
        a1 = -2.0f * cosOmega;
        a2 =  1.0f - alpha;
        break;

    default:
        break;
    }

    /* a0로 정규화 */
    float invA0 = 1.0f / a0;
    f->b0 = b0 * invA0;
    f->b1 = b1 * invA0;
    f->b2 = b2 * invA0;
    f->a1 = a1 * invA0;
    f->a2 = a2 * invA0;
}

float BiquadFilter_Update(BiquadFilter_t *f, float input)
{
    if (!f) return input;

    /* Direct Form II Transposed 구조:
     * y[n] = b0 * x[n] + d1
     * d1   = b1 * x[n] - a1 * y[n] + d2
     * d2   = b2 * x[n] - a2 * y[n]
     */
    float output = (f->b0 * input) + f->d1;
    f->d1 = (f->b1 * input) - (f->a1 * output) + f->d2;
    f->d2 = (f->b2 * input) - (f->a2 * output);

    return output;
}

void BiquadFilter_Reset(BiquadFilter_t *f)
{
    if (!f) return;
    f->d1 = 0.0f;
    f->d2 = 0.0f;
}

void LowPassFilter_Init(BiquadFilter_t *f, float sampleRate, float cutoffFreq, float qFactor)
{
    BiquadFilter_Init(f, FILTER_TYPE_LOWPASS, sampleRate, cutoffFreq, qFactor);
}

void HighPassFilter_Init(BiquadFilter_t *f, float sampleRate, float cutoffFreq, float qFactor)
{
    BiquadFilter_Init(f, FILTER_TYPE_HIGHPASS, sampleRate, cutoffFreq, qFactor);
}

void BandPassFilter_Init(BiquadFilter_t *f, float sampleRate, float centerFreq, float qFactor)
{
    BiquadFilter_Init(f, FILTER_TYPE_BANDPASS, sampleRate, centerFreq, qFactor);
}

void BandStopFilter_Init(BiquadFilter_t *f, float sampleRate, float notchFreq, float qFactor)
{
    BiquadFilter_Init(f, FILTER_TYPE_BANDSTOP, sampleRate, notchFreq, qFactor);
}
