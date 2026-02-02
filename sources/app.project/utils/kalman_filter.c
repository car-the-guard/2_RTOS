/*
 * kalman_filter.c - 1D 칼만 필터 구현
 *
 * 예측: x_pred = x, P_pred = P + Q
 * 갱신: K = P_pred / (P_pred + R), x = x_pred + K*(z - x_pred), P = (1-K)*P_pred
 */

#include "kalman_filter.h"

void KalmanFilter1D_Init(KalmanFilter1D_t *kf, float x0, float P0, float Q, float R)
{
    if (kf == (KalmanFilter1D_t *)0) return;

    kf->x = x0;
    kf->P = P0;
    kf->Q = Q;
    kf->R = R;
}

float KalmanFilter1D_Update(KalmanFilter1D_t *kf, float measurement)
{
    if (kf == (KalmanFilter1D_t *)0) return measurement;

    /* 예측 단계 (상태 불변 모델) */
    float P_pred = kf->P + kf->Q;

    /* 칼만 이득 */
    float K = P_pred / (P_pred + kf->R);

    /* 갱신 */
    kf->x = kf->x + K * (measurement - kf->x);
    kf->P = (1.0f - K) * P_pred;

    return kf->x;
}

float KalmanFilter1D_Get(const KalmanFilter1D_t *kf)
{
    if (kf == (KalmanFilter1D_t *)0) return 0.0f;
    return kf->x;
}
