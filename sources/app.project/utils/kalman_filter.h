/*
 * kalman_filter.h - 1D 칼만 필터 (스칼라)
 *
 * Sonar, 거리 센서 등 단일 측정값 노이즈 제거용.
 * 상태 모델: x_k = x_{k-1} (랜덤 워크, 속도 모델 생략)
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>

/* 1D 칼만 필터 상태 구조체 */
typedef struct {
    float x;   /* 상태 추정값 (필터 출력) */
    float P;   /* 추정 오차 공분산 */
    float Q;   /* 프로세스 노이즈 (작을수록 추정값이 천천히 변함) */
    float R;   /* 측정 노이즈 (클수록 센서를 덜 신뢰) */
} KalmanFilter1D_t;

/**
 * @brief 1D 칼만 필터 초기화
 *
 * @param kf     필터 구조체 포인터
 * @param x0     초기 추정값
 * @param P0     초기 오차 공분산 (불확실성, 클수록 첫 측정을 더 신뢰)
 * @param Q      프로세스 노이즈 (거리 변화 속도에 따른 튜닝)
 * @param R      측정 노이즈 (센서 노이즈 수준)
 */
void KalmanFilter1D_Init(KalmanFilter1D_t *kf, float x0, float P0, float Q, float R);

/**
 * @brief 측정값 입력 및 필터 업데이트 (추정값 반환)
 *
 * @param kf          필터 구조체 포인터
 * @param measurement 측정값 (예: sonar 거리 cm)
 * @return float      필터된 추정값
 */
float KalmanFilter1D_Update(KalmanFilter1D_t *kf, float measurement);

/**
 * @brief 현재 추정값만 반환 (업데이트 없음)
 */
float KalmanFilter1D_Get(const KalmanFilter1D_t *kf);

#endif /* KALMAN_FILTER_H */
