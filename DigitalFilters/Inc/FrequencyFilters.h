/**
 * @file FrequencyFilters.h
 * @brief 카테고리 2: 주파수 선택 필터 (Biquad IIR 직접형 II Direct Form II Transposed 구조)
 *
 * Cortex-M4 단정밀도 FPU 최적화 (채널당 수십 사이클 실행).
 * 1. 저역 통과 필터 (Low-Pass Filter, LPF)
 * 2. 고역 통과 필터 (High-Pass Filter, HPF)
 * 3. 대역 통과 필터 (Band-Pass Filter, BPF)
 * 4. 대역 차단 / 노치 필터 (Band-Stop / Notch Filter, BSF)
 */

#ifndef FREQUENCY_FILTERS_H
#define FREQUENCY_FILTERS_H

#ifdef __cplusplus
extern "C" {
#endif

/* 필터 타입 열거형 */
typedef enum {
    FILTER_TYPE_LOWPASS = 0,
    FILTER_TYPE_HIGHPASS,
    FILTER_TYPE_BANDPASS,
    FILTER_TYPE_BANDSTOP
} BiquadFilterType_t;

/**
 * 2차 IIR 바이쿼드(Biquad) 필터 구조체
 * 전달함수 H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
 * Direct Form II Transposed 구조 적용으로 상태 지연 메모리 d1, d2만 유지
 */
typedef struct {
    /* 정규화된 필터 계수 (a0 = 1.0) */
    float b0, b1, b2;
    float a1, a2;

    /* 상태 지연 변수 */
    float d1;
    float d2;
} BiquadFilter_t;

/**
 * 범용 바이쿼드 필터 계수 설정 함수
 *
 * @param f         필터 구조체 포인터
 * @param type      필터 타입 (LPF, HPF, BPF, BSF)
 * @param sampleRate 샘플링 주파수 (Hz, 예: 100.0f)
 * @param cutoffFreq 차단/중심 주파수 (Hz)
 * @param qFactor   Q 팩터 (일반적인 Butterworth 응답: 0.7071f, BPF/BSF는 대역폭 제어: 1.0f~10.0f)
 */
void  BiquadFilter_Init(BiquadFilter_t *f, BiquadFilterType_t type, float sampleRate, float cutoffFreq, float qFactor);

/**
 * 단일 샘플 실시간 필터링 연산 (인라인급 고속 FPU 실행, ~15 사이클)
 */
float BiquadFilter_Update(BiquadFilter_t *f, float input);

/**
 * 내부 상태 지연값 초기화
 */
void  BiquadFilter_Reset(BiquadFilter_t *f);

/* 편의 래퍼 함수 */
void LowPassFilter_Init(BiquadFilter_t *f, float sampleRate, float cutoffFreq, float qFactor);
void HighPassFilter_Init(BiquadFilter_t *f, float sampleRate, float cutoffFreq, float qFactor);
void BandPassFilter_Init(BiquadFilter_t *f, float sampleRate, float centerFreq, float qFactor);
void BandStopFilter_Init(BiquadFilter_t *f, float sampleRate, float notchFreq, float qFactor);

#define LowPassFilter_Update(f, in)   BiquadFilter_Update((f), (in))
#define HighPassFilter_Update(f, in)  BiquadFilter_Update((f), (in))
#define BandPassFilter_Update(f, in)  BiquadFilter_Update((f), (in))
#define BandStopFilter_Update(f, in)  BiquadFilter_Update((f), (in))

#ifdef __cplusplus
}
#endif

#endif /* FREQUENCY_FILTERS_H */
