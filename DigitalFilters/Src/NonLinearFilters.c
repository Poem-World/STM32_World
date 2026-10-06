/**
 * @file NonLinearFilters.c
 * @brief 비선형 신호 처리 및 특수 제어 필터 구현
 */

#include "DigitalFilters/NonLinearFilters.h"
#include <math.h>

/* =========================================================================
 * 1. 데드존 및 부호 함수 (Deadband & Signum)
 * ========================================================================= */

/* 1개 인자: 대칭형 [-th, +th] 초기화 */
void DeadbandFilter_Init1(DeadbandFilter_t *f, float threshold)
{
    if (!f) return;
    float th = (threshold >= 0.0f) ? threshold : -threshold;
    f->posThreshold = th;
    f->negThreshold = -th;
}

/* 2개 인자: 불평형/비대칭형 [negTh, posTh] 초기화 */
void DeadbandFilter_Init2(DeadbandFilter_t *f, float negThreshold, float posThreshold)
{
    if (!f) return;
    /* negThreshold는 음수 또는 0, posThreshold는 양수 또는 0으로 정규화 */
    f->negThreshold = (negThreshold <= 0.0f) ? negThreshold : -negThreshold;
    f->posThreshold = (posThreshold >= 0.0f) ? posThreshold : -posThreshold;
}

float DeadbandFilter_Update(DeadbandFilter_t *f, float input)
{
    if (!f) return input;

    /* 불감대 구간 [negThreshold, posThreshold] 내의 신호는 0으로 절삭 */
    if (input >= f->negThreshold && input <= f->posThreshold) {
        return 0.0f;
    }

    /* 초과 시: 원본 크기와 원래 부호 그대로 통과 */
    return input;
}

float DeadbandFilter_UpdateLinear(DeadbandFilter_t *f, float input)
{
    if (!f) return input;

    /* 불감대 구간 내는 0 */
    if (input >= f->negThreshold && input <= f->posThreshold) {
        return 0.0f;
    }

    /* 상한/하한 초과 시: 임계값을 뺀 연속 크기로 복원 (단차 충격 제거) */
    if (input > f->posThreshold) {
        return input - f->posThreshold;
    } else {
        return input - f->negThreshold;
    }
}

/* =========================================================================
 * 2. 히스테리시스 제로 크로싱 디텍터 (Hysteresis Zero-Crossing Detector)
 * ========================================================================= */

/* 1개 인자: 대칭형 [-th, +th] 초기화 */
void HysteresisZcd_Init1(HysteresisZcd_t *f, float threshold)
{
    if (!f) return;
    float th = (threshold >= 0.0f) ? threshold : -threshold;
    f->upperThreshold = th;
    f->lowerThreshold = -th;
    f->currentState = 0;
}

/* 2개 인자: 불평형/비대칭형 [lowerThresh, upperThresh] 초기화 */
void HysteresisZcd_Init2(HysteresisZcd_t *f, float lowerThresh, float upperThresh)
{
    if (!f) return;
    f->lowerThreshold = lowerThresh;
    f->upperThreshold = upperThresh;
    f->currentState = 0;
}

ZeroCrossEvent_t HysteresisZcd_Update(HysteresisZcd_t *f, float input)
{
    if (!f) return ZERO_CROSS_NONE;

    ZeroCrossEvent_t event = ZERO_CROSS_NONE;

    if (input >= f->upperThreshold) {
        /* 상한 통과 */
        if (f->currentState <= 0) {
            f->currentState = 1;
            event = ZERO_CROSS_RISING;
        }
    } else if (input <= f->lowerThreshold) {
        /* 하한 통과 */
        if (f->currentState >= 0) {
            f->currentState = -1;
            event = ZERO_CROSS_FALLING;
        }
    }

    return event;
}

int8_t HysteresisZcd_GetState(const HysteresisZcd_t *f)
{
    return f ? f->currentState : 0;
}

void HysteresisZcd_Reset(HysteresisZcd_t *f)
{
    if (!f) return;
    f->currentState = 0;
}
