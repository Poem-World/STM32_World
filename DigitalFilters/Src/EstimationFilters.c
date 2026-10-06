/**
 * @file EstimationFilters.c
 * @brief 카테고리 3: 추정 및 예측 적응형 필터 구현 (Cortex-M4 FPU 최적화)
 */

#include "DigitalFilters/EstimationFilters.h"
#include <string.h>

/* =========================================================================
 * 1. 1차원 스칼라 칼만 필터 (Kalman Filter 1D)
 *    예측 단계:
 *      x_pred = x
 *      p_pred = p + q
 *    갱신 단계:
 *      k = p_pred / (p_pred + r)
 *      x = x_pred + k * (measurement - x_pred)
 *      p = (1 - k) * p_pred
 * ========================================================================= */
void KalmanFilter1D_Init(KalmanFilter1D_t *f, float processNoise, float measureNoise, float initValue)
{
    if (!f) return;
    f->q = (processNoise > 0.0f) ? processNoise : 0.001f;
    f->r = (measureNoise > 0.0f) ? measureNoise : 0.05f;
    f->x = initValue;
    f->p = 1.0f;
    f->k = 0.0f;
}

float KalmanFilter1D_Update(KalmanFilter1D_t *f, float measurement)
{
    if (!f) return measurement;

    /* 1. 시간 갱신 (예측) */
    float p_pred = f->p + f->q;

    /* 2. 측정 갱신 (보정) */
    f->k = p_pred / (p_pred + f->r);
    f->x = f->x + f->k * (measurement - f->x);
    f->p = (1.0f - f->k) * p_pred;

    return f->x;
}

void KalmanFilter1D_Reset(KalmanFilter1D_t *f, float initValue)
{
    if (!f) return;
    f->x = initValue;
    f->p = 1.0f;
    f->k = 0.0f;
}

/* =========================================================================
 * 2. 정규화 LMS (NLMS) 적응형 필터
 *    y[n] = sum(w[k] * x[n-k])
 *    e[n] = d[n] - y[n]
 *    norm_x = sum(x[n-k]^2) + eps
 *    w[k] = w[k] + (mu / norm_x) * e[n] * x[n-k]
 * ========================================================================= */
void NlmsFilter_Init(NlmsFilter_t *f, uint16_t numTaps, float mu)
{
    if (!f) return;
    if (numTaps == 0) numTaps = 1;
    if (numTaps > NLMS_MAX_TAPS) numTaps = NLMS_MAX_TAPS;
    if (mu <= 0.0f) mu = 0.05f;
    if (mu >= 2.0f) mu = 1.9f;

    f->numTaps = numTaps;
    f->index = 0;
    f->mu = mu;
    f->eps = 1e-6f;

    memset(f->weights, 0, sizeof(f->weights));
    memset(f->buffer, 0, sizeof(f->buffer));
}

float NlmsFilter_Update(NlmsFilter_t *f, float input, float desired, float *pError)
{
    if (!f || f->numTaps == 0) return input;

    /* 1. 입력 버퍼 삽입 */
    f->buffer[f->index] = input;

    /* 2. 필터 출력 계산 y[n] 및 입력 벡터 에너지 norm_x 계산 */
    float y = 0.0f;
    float norm_x = 0.0f;
    uint16_t ptr = f->index;

    for (uint16_t i = 0; i < f->numTaps; i++) {
        float x_val = f->buffer[ptr];
        y += f->weights[i] * x_val;
        norm_x += x_val * x_val;

        ptr = (ptr == 0) ? (f->numTaps - 1) : (ptr - 1);
    }

    norm_x += f->eps;

    /* 3. 오차 계산 e[n] */
    float err = desired - y;
    if (pError) {
        *pError = err;
    }

    /* 4. 정규화 계수 갱신 (Normalized Weight Adaptation) */
    float step = (f->mu / norm_x) * err;
    ptr = f->index;
    for (uint16_t i = 0; i < f->numTaps; i++) {
        f->weights[i] += step * f->buffer[ptr];
        ptr = (ptr == 0) ? (f->numTaps - 1) : (ptr - 1);
    }

    /* 다음 입력용 버퍼 포인터 이동 */
    f->index = (f->index + 1) % f->numTaps;

    return y;
}

void NlmsFilter_Reset(NlmsFilter_t *f)
{
    if (!f) return;
    f->index = 0;
    memset(f->weights, 0, sizeof(f->weights));
    memset(f->buffer, 0, sizeof(f->buffer));
}
