/**
 * @file DigitalFilters.h
 * @brief STM32 Cortex-M4 최적화 실시간 디지털 필터 라이브러리 메인 헤더
 *
 * 카테고리 구성:
 * 1. Smoothing & Noise Reduction (스무딩 및 노이즈 제거)
 *    - Moving Average Filter (이동 평균 필터)
 *    - Exponential Moving Average Filter (지수 이동 평균 필터)
 *    - Gaussian Filter (가우시안 필터)
 *    - Median Filter (미디언 / 중앙값 필터)
 *
 * 2. Frequency Selective Filters (주파수 선택 필터 - Biquad IIR)
 *    - Low-Pass Filter (저역 통과 필터, LPF)
 *    - High-Pass Filter (고역 통과 필터, HPF)
 *    - Band-Pass Filter (대역 통과 필터, BPF)
 *    - Band-Stop / Notch Filter (대역 차단 / 노치 필터, BSF)
 *
 * 3. Estimation & Adaptive Filters (추정 및 예측 적응형 필터)
 *    - 1D Kalman Filter (1차원 스칼라 칼만 필터)
 *    - Normalized LMS Adaptive Filter (정규화 적응형 LMS 필터)
 */

#ifndef DIGITAL_FILTERS_H
#define DIGITAL_FILTERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "SmoothingFilters.h"
#include "FrequencyFilters.h"
#include "EstimationFilters.h"
#include "NonLinearFilters.h"

#ifdef __cplusplus
}
#endif

#endif /* DIGITAL_FILTERS_H */
