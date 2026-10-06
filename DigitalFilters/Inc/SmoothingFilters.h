/**
 * @file SmoothingFilters.h
 * @brief 카테고리 1: 스무딩 및 노이즈 제거 필터
 *
 * 1. 이동 평균 필터 (Moving Average Filter - Circular Buffer 최적화, O(1) 시간 복잡도)
 * 2. 지수 이동 평균 필터 (Exponential Moving Average Filter - 메모리 0, O(1))
 * 3. 가우시안 필터 (Gaussian Filter - 1D 이산 가우시안 커널 컨볼루션)
 * 4. 미디언 / 중앙값 필터 (Median Filter - 정렬 기반 스파이크 노이즈 제거)
 */

#ifndef SMOOTHING_FILTERS_H
#define SMOOTHING_FILTERS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOVING_AVG_MAX_WINDOW  64
#define GAUSSIAN_MAX_KERNEL    15
#define MEDIAN_MAX_WINDOW      31

/* =========================================================================
 * 1. 이동 평균 필터 (Moving Average Filter) - O(1) 실행
 * ========================================================================= */
typedef struct {
    float buffer[MOVING_AVG_MAX_WINDOW];
    uint16_t windowSize;
    uint16_t index;
    uint16_t count;
    float sum;
} MovingAvgFilter_t;

void  MovingAvgFilter_Init(MovingAvgFilter_t *f, uint16_t windowSize);
float MovingAvgFilter_Update(MovingAvgFilter_t *f, float input);
void  MovingAvgFilter_Reset(MovingAvgFilter_t *f);

/* =========================================================================
 * 2. 지수 이동 평균 필터 (Exponential Moving Average, EMA) - 메모리 극소, O(1)
 * ========================================================================= */
typedef struct {
    float alpha;       /* 가중 계수 (0.0 < alpha <= 1.0) */
    float output;
    uint8_t initialized;
} EmaFilter_t;

void  EmaFilter_Init(EmaFilter_t *f, float alpha);
float EmaFilter_Update(EmaFilter_t *f, float input);
void  EmaFilter_Reset(EmaFilter_t *f);

/* =========================================================================
 * 3. 가우시안 필터 (Gaussian Filter) - 대칭 가우시안 커널 가중합
 * ========================================================================= */
typedef struct {
    float buffer[GAUSSIAN_MAX_KERNEL];
    float kernel[GAUSSIAN_MAX_KERNEL];
    uint16_t kernelSize;   /* 홀수 권장 (3, 5, 7 ... 최대 GAUSSIAN_MAX_KERNEL) */
    uint16_t index;
    uint16_t count;
} GaussianFilter_t;

void  GaussianFilter_Init(GaussianFilter_t *f, uint16_t kernelSize, float sigma);
float GaussianFilter_Update(GaussianFilter_t *f, float input);
void  GaussianFilter_Reset(GaussianFilter_t *f);

/* =========================================================================
 * 4. 미디언 / 중앙값 필터 (Median Filter) - 임펄스/스파이크 노이즈 제거
 * ========================================================================= */
typedef struct {
    float buffer[MEDIAN_MAX_WINDOW];
    uint16_t windowSize;   /* 홀수 권장 (3 ~ MEDIAN_MAX_WINDOW) */
    uint16_t index;
    uint16_t count;
} MedianFilter_t;

void  MedianFilter_Init(MedianFilter_t *f, uint16_t windowSize);
float MedianFilter_Update(MedianFilter_t *f, float input);
void  MedianFilter_Reset(MedianFilter_t *f);

#ifdef __cplusplus
}
#endif

#endif /* SMOOTHING_FILTERS_H */
