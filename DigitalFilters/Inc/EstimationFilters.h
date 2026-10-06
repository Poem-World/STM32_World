/**
 * @file EstimationFilters.h
 * @brief 카테고리 3: 추정 및 예측 적응형 필터
 *
 * 1. 1차원 칼만 필터 (1D Scalar Kalman Filter - 센서 노이즈 제거 및 상태 추정, O(1))
 * 2. 정규화 최소평균제곱(Normalized LMS) 적응형 필터 (NLMS Adaptive Filter - 시변 노이즈 제거)
 */

#ifndef ESTIMATION_FILTERS_H
#define ESTIMATION_FILTERS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NLMS_MAX_TAPS  32

/* =========================================================================
 * 1. 1D 칼만 필터 (1-Dimensional Kalman Filter)
 *    센서(가속도, 각속도, 엔코더)의 측정 노이즈(R)와 프로세스 노이즈(Q)를 반영하여 최적 추정
 * ========================================================================= */
typedef struct {
    float q;      /* 프로세스 잡음 공분산 (Process noise covariance, 예: 0.001f ~ 0.1f) */
    float r;      /* 측정 잡음 공분산 (Measurement noise covariance, 예: 0.01f ~ 1.0f) */
    float x;      /* 추정된 상태 값 (Estimated state) */
    float p;      /* 추정 오차 공분산 (Estimation error covariance) */
    float k;      /* 칼만 이득 (Kalman gain) */
} KalmanFilter1D_t;

/**
 * 1D 칼만 필터 초기화
 * @param f 필터 구조체 포인터
 * @param processNoise 프로세스 노이즈 (Q)
 * @param measureNoise 측정 노이즈 (R)
 * @param initValue    초기 상태 값
 */
void  KalmanFilter1D_Init(KalmanFilter1D_t *f, float processNoise, float measureNoise, float initValue);

/**
 * 단일 측정값 갱신 및 상태 추정치 산출 (시간 복잡도 O(1), ~20 사이클)
 */
float KalmanFilter1D_Update(KalmanFilter1D_t *f, float measurement);

void  KalmanFilter1D_Reset(KalmanFilter1D_t *f, float initValue);

/* =========================================================================
 * 2. 정규화 LMS (Normalized LMS, NLMS) 적응형 필터
 *    신호 입력 x[n]과 원하는 기준 신호 d[n] 사이에서 필터 가중치 w를 실시간 자가 적응
 * ========================================================================= */
typedef struct {
    float weights[NLMS_MAX_TAPS]; /* 필터 가중치 계수 벡터 */
    float buffer[NLMS_MAX_TAPS];  /* 입력 신호 지연 버퍼 */
    uint16_t numTaps;             /* 탭 수 (1 ~ NLMS_MAX_TAPS) */
    uint16_t index;               /* 순환 버퍼 인덱스 */
    float mu;                     /* 적응 스텝 사이즈 (Learning rate, 0.0 < mu < 2.0) */
    float eps;                    /* 분모 0 방지 정규화 상수 (예: 1e-6f) */
} NlmsFilter_t;

/**
 * NLMS 적응형 필터 초기화
 * @param f       필터 구조체 포인터
 * @param numTaps 필터 차수/탭 수 (최대 NLMS_MAX_TAPS)
 * @param mu      학습률 (Step size, 예: 0.1f)
 */
void  NlmsFilter_Init(NlmsFilter_t *f, uint16_t numTaps, float mu);

/**
 * 적응형 필터 갱신
 * @param f         필터 구조체 포인터
 * @param input     현재 입력 신호 x[n]
 * @param desired   원하는 목표 기준 신호 d[n] (노이즈 캔슬링의 경우 오염된 원신호)
 * @param pError    출력 오차 e[n] 반환 포인터 (선택적, NULL 가능)
 * @return 필터 출력 추정치 y[n]
 */
float NlmsFilter_Update(NlmsFilter_t *f, float input, float desired, float *pError);

void  NlmsFilter_Reset(NlmsFilter_t *f);

#ifdef __cplusplus
}
#endif

#endif /* ESTIMATION_FILTERS_H */
