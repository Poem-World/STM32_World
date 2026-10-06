/**
 * @file SmoothingFilters.c
 * @brief 카테고리 1: 스무딩 및 노이즈 제거 필터 구현 (Cortex-M4 최적화)
 */

#include "DigitalFilters/SmoothingFilters.h"
#include <math.h>
#include <string.h>

/* =========================================================================
 * 1. 이동 평균 필터 (Moving Average Filter)
 *    링 버퍼 + 누적합 갱신을 통한 O(1) 초고속 알고리즘
 * ========================================================================= */
void MovingAvgFilter_Init(MovingAvgFilter_t *f, uint16_t windowSize)
{
    if (!f) return;
    if (windowSize == 0) windowSize = 1;
    if (windowSize > MOVING_AVG_MAX_WINDOW) windowSize = MOVING_AVG_MAX_WINDOW;

    memset(f->buffer, 0, sizeof(f->buffer));
    f->windowSize = windowSize;
    f->index = 0;
    f->count = 0;
    f->sum = 0.0f;
}

float MovingAvgFilter_Update(MovingAvgFilter_t *f, float input)
{
    if (!f || f->windowSize == 0) return input;

    if (f->count < f->windowSize) {
        /* 초기 버퍼 채우기 단계 */
        f->buffer[f->index] = input;
        f->sum += input;
        f->count++;
        f->index = (f->index + 1) % f->windowSize;
        return f->sum / (float)f->count;
    } else {
        /* 윈도우 가득 찬 상태: O(1) 이동 평균 갱신 */
        f->sum -= f->buffer[f->index];
        f->buffer[f->index] = input;
        f->sum += input;
        f->index = (f->index + 1) % f->windowSize;
        return f->sum / (float)f->windowSize;
    }
}

void MovingAvgFilter_Reset(MovingAvgFilter_t *f)
{
    if (!f) return;
    memset(f->buffer, 0, sizeof(f->buffer));
    f->index = 0;
    f->count = 0;
    f->sum = 0.0f;
}

/* =========================================================================
 * 2. 지수 이동 평균 필터 (Exponential Moving Average, EMA)
 *    y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
 * ========================================================================= */
void EmaFilter_Init(EmaFilter_t *f, float alpha)
{
    if (!f) return;
    if (alpha <= 0.0f) alpha = 0.01f;
    if (alpha > 1.0f) alpha = 1.0f;

    f->alpha = alpha;
    f->output = 0.0f;
    f->initialized = 0;
}

float EmaFilter_Update(EmaFilter_t *f, float input)
{
    if (!f) return input;
    if (!f->initialized) {
        f->output = input;
        f->initialized = 1;
        return input;
    }
    f->output = (f->alpha * input) + ((1.0f - f->alpha) * f->output);
    return f->output;
}

void EmaFilter_Reset(EmaFilter_t *f)
{
    if (!f) return;
    f->output = 0.0f;
    f->initialized = 0;
}

/* =========================================================================
 * 3. 가우시안 필터 (Gaussian Filter)
 * ========================================================================= */
void GaussianFilter_Init(GaussianFilter_t *f, uint16_t kernelSize, float sigma)
{
    if (!f) return;
    if (kernelSize == 0) kernelSize = 1;
    if (kernelSize > GAUSSIAN_MAX_KERNEL) kernelSize = GAUSSIAN_MAX_KERNEL;
    if (sigma <= 0.0001f) sigma = 1.0f;

    f->kernelSize = kernelSize;
    f->index = 0;
    f->count = 0;
    memset(f->buffer, 0, sizeof(f->buffer));

    /* 1D 가우시안 커널 계산 및 정규화 */
    float sum = 0.0f;
    int half = (int)(kernelSize / 2);
    for (int i = 0; i < (int)kernelSize; i++) {
        float x = (float)(i - half);
        float val = expf(-(x * x) / (2.0f * sigma * sigma));
        f->kernel[i] = val;
        sum += val;
    }
    for (int i = 0; i < (int)kernelSize; i++) {
        f->kernel[i] /= sum;
    }
}

float GaussianFilter_Update(GaussianFilter_t *f, float input)
{
    if (!f || f->kernelSize == 0) return input;

    f->buffer[f->index] = input;
    f->index = (f->index + 1) % f->kernelSize;
    if (f->count < f->kernelSize) f->count++;

    /* 컨볼루션 연산 */
    float out = 0.0f;
    float weightSum = 0.0f;
    uint16_t ptr = f->index;
    for (int i = 0; i < f->count; i++) {
        ptr = (ptr == 0) ? (f->kernelSize - 1) : (ptr - 1);
        float w = f->kernel[i];
        out += f->buffer[ptr] * w;
        weightSum += w;
    }
    return (weightSum > 0.0f) ? (out / weightSum) : input;
}

void GaussianFilter_Reset(GaussianFilter_t *f)
{
    if (!f) return;
    memset(f->buffer, 0, sizeof(f->buffer));
    f->index = 0;
    f->count = 0;
}

/* =========================================================================
 * 4. 미디언 / 중앙값 필터 (Median Filter) - 삽입 정렬 기반 고속 중앙값 추출
 * ========================================================================= */
void MedianFilter_Init(MedianFilter_t *f, uint16_t windowSize)
{
    if (!f) return;
    if (windowSize == 0) windowSize = 1;
    if (windowSize > MEDIAN_MAX_WINDOW) windowSize = MEDIAN_MAX_WINDOW;

    memset(f->buffer, 0, sizeof(f->buffer));
    f->windowSize = windowSize;
    f->index = 0;
    f->count = 0;
}

float MedianFilter_Update(MedianFilter_t *f, float input)
{
    if (!f || f->windowSize == 0) return input;

    f->buffer[f->index] = input;
    f->index = (f->index + 1) % f->windowSize;
    if (f->count < f->windowSize) f->count++;

    /* 버퍼 복사 후 삽입 정렬 (소규모 배열에 가장 빠르고 결정론적인 정렬) */
    float sorted[MEDIAN_MAX_WINDOW];
    memcpy(sorted, f->buffer, f->count * sizeof(float));

    for (int i = 1; i < (int)f->count; i++) {
        float key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    /* 중앙값 반환 */
    if (f->count % 2 == 1) {
        return sorted[f->count / 2];
    } else {
        return (sorted[(f->count / 2) - 1] + sorted[f->count / 2]) * 0.5f;
    }
}

void MedianFilter_Reset(MedianFilter_t *f)
{
    if (!f) return;
    memset(f->buffer, 0, sizeof(f->buffer));
    f->index = 0;
    f->count = 0;
}
