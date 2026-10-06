/**
 * @file NonLinearFilters.h
 * @brief 비선형 신호 처리 및 특수 제어 필터 (Nonlinear & Threshold Filters)
 *
 * =============================================================================
 * [핵심 기능 및 수학적 수식 (Core Features & Mathematical Formulation)]
 * =============================================================================
 *
 * 1. 데드존 및 부호 함수 (Deadband & Signum Filter)
 *   (1) 부호 함수 (Sign / Signum Function):
 *          sgn(x) = +1.0  (x > 0)
 *                    0.0  (x == 0)
 *                   -1.0  (x < 0)
 *
 *   (2) 대칭형 데드존 (Symmetric Deadband: [-Vth, +Vth]):
 *       - 단순 절삭 (Step-Cut):
 *          y = 0                      (|x| <= Vth)
 *          y = sgn(x) * |x| = x       (|x| > Vth)
 *       - 선형 연속 (Linear Continuous):
 *          y = 0                      (|x| <= Vth)
 *          y = sgn(x) * (|x| - Vth)   (|x| > Vth)
 *
 *   (3) 비대칭/불평형 데드존 (Asymmetric Deadband: [Vth_neg, Vth_pos]):
 *       (단, Vth_neg <= 0, Vth_pos >= 0)
 *       - 단순 절삭 (Step-Cut):
 *          y = 0                      (Vth_neg <= x <= Vth_pos)
 *          y = x                      (x > Vth_pos 또는 x < Vth_neg)
 *       - 선형 연속 (Linear Continuous):
 *          y = 0                      (Vth_neg <= x <= Vth_pos)
 *          y = x - Vth_pos            (x > Vth_pos)
 *          y = x - Vth_neg            (x < Vth_neg)
 *
 * 2. 히스테리시스 제로 크로싱 디텍터 (Hysteresis Zero-Crossing Detector)
 *   (1) 대칭형 (Symmetric: Upper = +Vth, Lower = -Vth):
 *   (2) 비대칭형 (Asymmetric: Upper = Vth_pos, Lower = Vth_neg):
 *   - 상태 천이 (State Transition):
 *          State[k] = +1 (High) : if x[k] >= UpperThreshold
 *          State[k] = -1 (Low)  : if x[k] <= LowerThreshold
 *          State[k] = State[k-1]: if LowerThreshold < x[k] < UpperThreshold (유지)
 *   - 영점 교차 이벤트 판별 (Zero-Crossing Events):
 *          RISING  (상향 영점 교차): State[k-1] <= 0 이고 x[k] >= UpperThreshold 일 때 발생
 *          FALLING (하향 영점 교차): State[k-1] >= 0 이고 x[k] <= LowerThreshold 일 때 발생
 *          NONE    (변화 없음)     : 임계값 미도달 또는 상태 불변
 * =============================================================================
 */

#ifndef NON_LINEAR_FILTERS_H
#define NON_LINEAR_FILTERS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * 1. 데드존 및 부호 함수 (Deadband & Signum)
 * ========================================================================= */

/**
 * 부호 판별 함수 (Signum / Sign Function)
 * @param val 입력 값
 * @return val > 0 ? +1.0f : (val < 0 ? -1.0f : 0.0f)
 */
static inline float Filter_Sign(float val)
{
    if (val > 0.0f) return 1.0f;
    if (val < 0.0f) return -1.0f;
    return 0.0f;
}

/**
 * 데드존 필터 구조체 (대칭 및 비대칭 공용)
 */
typedef struct {
    float posThreshold; /* 양의 불감대 상한 Vth_pos (>= 0) */
    float negThreshold; /* 음의 불감대 하한 Vth_neg (<= 0) */
} DeadbandFilter_t;

/**
 * C 언어 단일/복수 인자 기반 오버로딩 지원 내부 함수
 */
void DeadbandFilter_Init1(DeadbandFilter_t *f, float threshold);
void DeadbandFilter_Init2(DeadbandFilter_t *f, float negThreshold, float posThreshold);

/**
 * 단순 데드존 + 원신호 부호 복원 (Step-Cut Deadband)
 * negThreshold <= input <= posThreshold 이면 0.0f 반환
 * 범위 초과 시 원래 부호와 크기 그대로 통과
 */
float DeadbandFilter_Update(DeadbandFilter_t *f, float input);

/**
 * 선형 연속 데드존 (Linear Rescaled Deadband)
 * 서보 제어기 지터 방지용으로 급격한 단차 없이 0부터 부드럽게 시작:
 * negThreshold <= input <= posThreshold 이면 0.0f
 * input > posThreshold 이면 input - posThreshold
 * input < negThreshold 이면 input - negThreshold
 */
float DeadbandFilter_UpdateLinear(DeadbandFilter_t *f, float input);

/* =========================================================================
 * 2. 히스테리시스 제로 크로싱 디텍터 (Hysteresis Zero-Crossing Detector)
 * ========================================================================= */

typedef enum {
    ZERO_CROSS_NONE = 0,     /* 변화 없음 */
    ZERO_CROSS_RISING = 1,   /* 음수 -> 양수로 영점 상향 통과 */
    ZERO_CROSS_FALLING = -1  /* 양수 -> 음수로 영점 하향 통과 */
} ZeroCrossEvent_t;

/**
 * 히스테리시스 제로 크로싱 디텍터 구조체 (대칭 및 비대칭 공용)
 */
typedef struct {
    float upperThreshold; /* 상한 임계값 (+Vth > 0) */
    float lowerThreshold; /* 하한 임계값 (-Vth < 0) */
    int8_t currentState;  /* 현재 판별 상태 (+1: High, -1: Low, 0: 초기) */
} HysteresisZcd_t;

/**
 * C 언어 단일/복수 인자 기반 오버로딩 지원 내부 함수
 */
void HysteresisZcd_Init1(HysteresisZcd_t *f, float threshold);
void HysteresisZcd_Init2(HysteresisZcd_t *f, float lowerThresh, float upperThresh);

/**
 * 샘플 입력 시 제로 크로싱 감지 및 현재 상태 갱신
 * @param f     디텍터 구조체 포인터
 * @param input 현재 센서/신호 입력
 * @return ZeroCrossEvent_t (ZERO_CROSS_NONE, ZERO_CROSS_RISING, ZERO_CROSS_FALLING)
 */
ZeroCrossEvent_t HysteresisZcd_Update(HysteresisZcd_t *f, float input);

/**
 * 현재 필터링된 부호 상태 반환 (+1 또는 -1)
 */
int8_t HysteresisZcd_GetState(const HysteresisZcd_t *f);

/**
 * 디텍터 상태 리셋
 */
void   HysteresisZcd_Reset(HysteresisZcd_t *f);

#ifdef __cplusplus
}

/* =========================================================================
 * C++ 함수 오버로딩 (C++ 환경 지원)
 * ========================================================================= */
inline void DeadbandFilter_Init(DeadbandFilter_t *f, float threshold) {
    DeadbandFilter_Init1(f, threshold);
}
inline void DeadbandFilter_Init(DeadbandFilter_t *f, float negThreshold, float posThreshold) {
    DeadbandFilter_Init2(f, negThreshold, posThreshold);
}

inline void HysteresisZcd_Init(HysteresisZcd_t *f, float threshold) {
    HysteresisZcd_Init1(f, threshold);
}
inline void HysteresisZcd_Init(HysteresisZcd_t *f, float lowerThresh, float upperThresh) {
    HysteresisZcd_Init2(f, lowerThresh, upperThresh);
}

#else

/* =========================================================================
 * C11 _Generic 기반 함수 오버로딩 매크로 (동일한 함수 이름으로 1개/2개 인자 처리)
 * ========================================================================= */
#define _DF_INIT_SELECT(_1, _2, _3, NAME, ...) NAME

#define DeadbandFilter_Init(...) \
    _DF_INIT_SELECT(__VA_ARGS__, DeadbandFilter_Init2, DeadbandFilter_Init1)(__VA_ARGS__)

#define HysteresisZcd_Init(...) \
    _DF_INIT_SELECT(__VA_ARGS__, HysteresisZcd_Init2, HysteresisZcd_Init1)(__VA_ARGS__)

#endif

/*
================================================================================
                           사용 예제 (Examples)
================================================================================

[예제 1] 대칭형 데드존 (Symmetric Deadband) - 동일한 함수 이름 호출:
--------------------------------------------------------------------------------
    DeadbandFilter_t dbSym;
    // 인자 1개: [-0.05, +0.05] 대칭 영역 0 절삭
    DeadbandFilter_Init(&dbSym, 0.05f);

    float out1 = DeadbandFilter_UpdateLinear(&dbSym, rawInput);

[예제 2] 비대칭/불평형 데드존 (Asymmetric Deadband) - 동일한 함수 이름 호출:
--------------------------------------------------------------------------------
    DeadbandFilter_t dbAsym;
    // 인자 2개: 음수 하한 -0.02f, 양수 상한 +0.08f 불평형 영역 0 절삭
    DeadbandFilter_Init(&dbAsym, -0.02f, 0.08f);

    float out2 = DeadbandFilter_UpdateLinear(&dbAsym, rawInput);

[예제 3] 대칭형 제로 크로싱 디텍터 (Symmetric Hysteresis ZCD):
--------------------------------------------------------------------------------
    HysteresisZcd_t zcdSym;
    // 인자 1개: 상한 +0.1f, 하한 -0.1f 대칭 슈미트 트리거
    HysteresisZcd_Init(&zcdSym, 0.1f);

    ZeroCrossEvent_t ev1 = HysteresisZcd_Update(&zcdSym, waveSample);

[예제 4] 비대칭형 제로 크로싱 디텍터 (Asymmetric Hysteresis ZCD):
--------------------------------------------------------------------------------
    HysteresisZcd_t zcdAsym;
    // 인자 2개: 하한 -0.05f, 상한 +0.15f 불평형 슈미트 트리거
    HysteresisZcd_Init(&zcdAsym, -0.05f, 0.15f);

    ZeroCrossEvent_t ev2 = HysteresisZcd_Update(&zcdAsym, waveSample);
================================================================================
*/

#endif /* NON_LINEAR_FILTERS_H */
