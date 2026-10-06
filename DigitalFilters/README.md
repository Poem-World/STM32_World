# Embedded Real-Time Digital Filters Library (C99 / C11 / C++)

Cortex-M FPU(Floating Point Unit) 하드웨어 가속에 최적화된 독립형(Standalone) 실시간 디지털 필터 라이브러리입니다. 모션 제어, 센서 퓨전, 모터 제어기, 오디오/계측 신호 처리를 위해 설계되었습니다.

---

## 1. 라이브러리 구성 (Library Structure)

```
DigitalFilters/
├── DigitalFilters.h       # 통합 인클루드 마스터 헤더
├── SmoothingFilters.h/c   # 1. 스무딩 및 노이즈 제거 필터
├── FrequencyFilters.h/c   # 2. 2차 Biquad IIR 주파수 선택 필터
├── EstimationFilters.h/c  # 3. 1D 칼만 및 NLMS 적응형 필터
├── NonLinearFilters.h/c   # 4. 데드존 및 히스테리시스 ZCD
└── README.md              # 상세 기술 문서 및 수식 가이드
```

---

## 2. 카테고리 1: 스무딩 및 노이즈 제거 (Smoothing & Noise Reduction)

센서 입력의 무작위 백색 잡음(White Noise)과 순간적인 이상치/스파이크를 제거합니다.

### 2.1 이동 평균 필터 (Moving Average Filter)
- **알고리즘**: 순환 링 버퍼(Circular Buffer)와 누적합(Running Sum) 기반으로 윈도우 크기($N$)와 무관하게 **$O(1)$** 연산 보장.
- **수학식**:
  $$y[k] = y[k-1] + \frac{x[k] - x[k - N]}{N}$$

### 2.2 지수 이동 평균 필터 (Exponential Moving Average, EMA)
- **알고리즘**: 1계 가중 지수 감쇠 필터 (버퍼 메모리 없음, 곱셈 2회/덧셈 1회).
- **수학식**:
  $$y[k] = \alpha \cdot x[k] + (1 - \alpha) \cdot y[k-1] \quad (0 < \alpha \le 1)$$
- **차단 주파수($f_c$) 환산식**:
  $$\alpha \approx \frac{2\pi f_c / f_s}{1 + 2\pi f_c / f_s}$$

### 2.3 가우시안 필터 (Gaussian Filter)
- **알고리즘**: 1D 이산 가우시안 정규분포 커널 컨볼루션(Convolution).
- **수학식**:
  $$w[i] = \frac{1}{\sqrt{2\pi}\sigma} \exp\left(-\frac{i^2}{2\sigma^2}\right), \quad y[k] = \frac{\sum_{i=-M}^{M} w[i] \cdot x[k-i]}{\sum_{i=-M}^{M} w[i]}$$

### 2.4 미디언 / 중앙값 필터 (Median Filter)
- **알고리즘**: 인라인 삽입 정렬(Insertion Sort) 기반 중앙값 추출로 임펄스/스파이크 노이즈 제거.
- **수학식**:
  $$y[k] = \text{median}\Big(x[k], x[k-1], \dots, x[k-N+1]\Big)$$

---

## 3. 카테고리 2: 주파수 선택 필터 (Frequency Selective Biquad IIR)

Robert Bristow-Johnson의 **Audio EQ Cookbook** 공식 기반 2차 IIR 바이쿼드 필터입니다. FPU 오차를 최소화하고 지연을 줄이기 위해 **Direct Form II Transposed** 구조를 적용했습니다.

### 3.1 차분 방정식 (Direct Form II Transposed)
- **전달함수**:
  $$H(z) = \frac{Y(z)}{X(z)} = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}{1 + a_1 z^{-1} + a_2 z^{-2}}$$
- **상태 방정식** (상태 변수 $d_1, d_2$ 2개만 유지, 약 15 CPU 사이클 소요):
  $$y[n] = b_0 \cdot x[n] + d_1[n-1]$$
  $$d_1[n] = b_1 \cdot x[n] - a_1 \cdot y[n] + d_2[n-1]$$
  $$d_2[n] = b_2 \cdot x[n] - a_2 \cdot y[n]$$

### 3.2 계수 계산 공식 ($\omega_0 = 2\pi f_0 / f_s$, $\alpha = \sin\omega_0 / (2Q)$)

| 필터 타입 | $b_0$ | $b_1$ | $b_2$ | $a_0$ | $a_1$ | $a_2$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **저역 통과 (LPF)** | $\frac{1 - \cos\omega_0}{2}$ | $1 - \cos\omega_0$ | $\frac{1 - \cos\omega_0}{2}$ | $1 + \alpha$ | $-2\cos\omega_0$ | $1 - \alpha$ |
| **고역 통과 (HPF)** | $\frac{1 + \cos\omega_0}{2}$ | $-(1 + \cos\omega_0)$ | $\frac{1 + \cos\omega_0}{2}$ | $1 + \alpha$ | $-2\cos\omega_0$ | $1 - \alpha$ |
| **대역 통과 (BPF)** | $\alpha$ | $0$ | $-\alpha$ | $1 + \alpha$ | $-2\cos\omega_0$ | $1 - \alpha$ |
| **대역 차단 (BSF/Notch)** | $1$ | $-2\cos\omega_0$ | $1$ | $1 + \alpha$ | $-2\cos\omega_0$ | $1 - \alpha$ |

---

## 4. 카테고리 3: 상태 추정 및 예측 필터 (Estimation & Adaptive Filters)

### 4.1 1차원 스칼라 칼만 필터 (1D Kalman Filter)
측정 잡음 공분산($R$)과 프로세스 동적 잡음 공분산($Q$)을 융합하여 센서 참값을 최적 추정 ($O(1)$, 행렬 연산 배제로 약 20 사이클 소요).

- **1) 시간 갱신 (예측 단계)**:
  $$\hat{x}_{k|k-1} = \hat{x}_{k-1}, \quad P_{k|k-1} = P_{k-1} + Q$$
- **2) 측정 갱신 (보정 단계)**:
  $$K_k = \frac{P_{k|k-1}}{P_{k|k-1} + R} \quad (\text{Kalman Gain})$$
  $$\hat{x}_k = \hat{x}_{k|k-1} + K_k \cdot (z_k - \hat{x}_{k|k-1})$$
  $$P_k = (1 - K_k) \cdot P_{k|k-1}$$

### 4.2 정규화 LMS 적응형 필터 (Normalized LMS, NLMS)
시변 노이즈 환경에서 입력 신호 파워로 가중치 갱신폭을 정규화하여 발산 없이 최적 계수를 자가 학습.

- **출력 및 오차**:
  $$y[n] = \sum_{i=0}^{M-1} w_i[n] \cdot x[n-i], \quad e[n] = d[n] - y[n]$$
- **가중치 적응 (Adaptation)**:
  $$\|\mathbf{x}[n]\|^2 = \sum_{i=0}^{M-1} x^2[n-i] + \epsilon, \quad \mathbf{w}[n+1] = \mathbf{w}[n] + \frac{\mu}{\|\mathbf{x}[n]\|^2} \cdot e[n] \cdot \mathbf{x}[n]$$

---

## 5. 카테고리 4: 비선형 불감대 및 검출 필터 (Nonlinear & Threshold Filters)

### 5.1 부호 함수 (Sign / Signum Function)
$$\text{sgn}(x) = \begin{cases} +1.0 & (x > 0) \\ 0.0 & (x = 0) \\ -1.0 & (x < 0) \end{cases}$$

### 5.2 데드존 필터 (Deadband Filter)
0 부근 정지 상태의 미세 진동/헌팅(Hunting)을 마스킹합니다. C11 `_Generic` 및 C++ 오버로딩을 통해 대칭/불평형을 동일 함수명으로 지원합니다.

- **1) 대칭형 데드존 ($[-V_{th}, +V_{th}]$)**:
  - **단순 절삭 (Step-Cut)**:
    $$y = \begin{cases} 0 & (|x| \le V_{th}) \\ x & (|x| > V_{th}) \end{cases}$$
  - **선형 연속 (Linear Rescaled - 모터 단차 충격 방지)**:
    $$y = \begin{cases} 0 & (|x| \le V_{th}) \\ \text{sgn}(x) \cdot (|x| - V_{th}) & (|x| > V_{th}) \end{cases}$$

- **2) 불평형/비대칭형 데드존 ($[V_{th,neg}, V_{th,pos}]$)**:
  - **선형 연속 (Linear Rescaled)**:
    $$y = \begin{cases} 0 & (V_{th,neg} \le x \le V_{th,pos}) \\ x - V_{th,pos} & (x > V_{th,pos}) \\ x - V_{th,neg} & (x < V_{th,neg}) \end{cases}$$

```c
DeadbandFilter_Init(&db, 0.05f);          // 대칭형: [-0.05, +0.05]
DeadbandFilter_Init(&db, -0.02f, 0.08f);  // 불평형: [-0.02, +0.08]
```

### 5.3 히스테리시스 제로 크로싱 디텍터 (Hysteresis ZCD)
슈미트 트리거 2중 임계값으로 0점 통과 채터링(False Crossing) 방지.

- **상태 천이**:
  $$\text{State}[k] = \begin{cases} +1 & (x[k] \ge \text{UpperThreshold}) \\ -1 & (x[k] \le \text{LowerThreshold}) \\ \text{State}[k-1] & (\text{LowerThreshold} < x[k] < \text{UpperThreshold}) \end{cases}$$
- **이벤트 반환**: `ZERO_CROSS_RISING` (+방향 교차), `ZERO_CROSS_FALLING` (-방향 교차), `ZERO_CROSS_NONE`

```c
HysteresisZcd_Init(&zcd, 0.1f);          // 대칭형: 상한 +0.1, 하한 -0.1
HysteresisZcd_Init(&zcd, -0.05f, 0.15f); // 불평형: 하한 -0.05, 상한 +0.15
```

---

## 6. 성능 벤치마크 및 시간 복잡도 (Benchmark)

*(ARM Cortex-M4 @ 240MHz, 단정밀도 FPU 가속, GCC -O3 기준)*

| 필터 알고리즘 | 시간 복잡도 | 메모리 (RAM) | 소요 시간 (평균) | 주요 적용처 |
| :--- | :---: | :---: | :---: | :--- |
| **Moving Average** | $O(1)$ | $4N + 12$ Bytes | **~0.05 µs** | 센서 평균화, 저주파 트렌드 |
| **EMA** | $O(1)$ | 12 Bytes | **~0.02 µs** | 초경량 1차 저역 감쇠 |
| **Median Filter (N=9)** | $O(N^2)$ (삽입정렬) | $8N + 8$ Bytes | **~0.25 µs** | 통신 패킷 결함, 이상치 제거 |
| **Gaussian (Kernel=7)** | $O(K)$ | $8K + 8$ Bytes | **~0.15 µs** | 위상 보존 스무딩 |
| **Biquad IIR (LPF/HPF/Notch)** | $O(1)$ | 28 Bytes | **~0.06 µs** | 전원 험, 기계 공진 주파수 차단 |
| **1D Kalman Filter** | $O(1)$ | 20 Bytes | **~0.08 µs** | IMU 각도, 가속도 센서 노이즈 |
| **NLMS (Taps=16)** | $O(M)$ | $8M + 16$ Bytes | **~0.45 µs** | 적응형 노이즈 캔슬링 |
| **Deadband (Linear)** | $O(1)$ | 8 Bytes | **~0.03 µs** | 조이스틱 0점 떨림, 모터 헌팅 방지 |
| **Hysteresis ZCD** | $O(1)$ | 12 Bytes | **~0.03 µs** | 주기 측정, 엔코더 인덱스 |

---

## 7. 실시간 제어 루프 예제 (Example Pipeline)

```c
#include "DigitalFilters/DigitalFilters.h"

static KalmanFilter1D_t s_kalman;
static BiquadFilter_t   s_notch;
static DeadbandFilter_t s_deadband;

void Control_Init(void)
{
    // 1. 센서 백색 노이즈 제거용 칼만 필터
    KalmanFilter1D_Init(&s_kalman, 0.005f, 0.05f, 0.0f);

    // 2. 25Hz 기계 공진 제거 노치 필터 (샘플링 100Hz 기준)
    BandStopFilter_Init(&s_notch, 100.0f, 25.0f, 5.0f);

    // 3. 서보 지터 방지용 불평형 데드존 (오버로딩)
    DeadbandFilter_Init(&s_deadband, -0.5f, 0.5f);
}

float Control_Process(float rawSensorInput)
{
    // Step 1: 칼만 필터링
    float est = KalmanFilter1D_Update(&s_kalman, rawSensorInput);

    // Step 2: 특정 주파수 공진 차단
    float notchOut = BandStopFilter_Update(&s_notch, est);

    // Step 3: 선형 연속 데드존 (원점 떨림 제거)
    float targetOut = DeadbandFilter_UpdateLinear(&s_deadband, notchOut);

    return targetOut;
}
```

---

## 8. 라이선스 (License)

MIT License 또는 자유로운 상업적/비상업적 프로젝트 임베딩 허용.
