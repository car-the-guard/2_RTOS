/*
 * relative_velocity.c - 거리 시계열 기반 상대 속도 추정 구현
 *
 * velocity = (d2 - d1) / (t2 - t1) [cm/s]
 */

#include "relative_velocity.h"

/* 최소 Δt [ms] - 너무 짧으면 노이즈 증폭 */
#define RELVEL_MIN_DT_MS      50U

static struct {
    float    d_prev;
    uint32_t t_prev;
    float    velocity;   /* cm/s */
    uint8_t  have_prev;  /* 1 = 이전 샘플 있음 */
    uint8_t  valid;      /* 1 = 속도 계산 완료 (2회 이상 측정) */
} s_relvel;

void RelativeVel_Init(void)
{
    s_relvel.have_prev = 0;
    s_relvel.valid     = 0;
}

void RelativeVel_Update(float distance_cm, uint32_t tick_ms)
{
    if (!s_relvel.have_prev)
    {
        /* 첫 측정: 저장만 */
        s_relvel.d_prev   = distance_cm;
        s_relvel.t_prev   = tick_ms;
        s_relvel.have_prev = 1;
        return;
    }

    /* Δt 계산 (오버플로우 고려) */
    uint32_t dt = tick_ms - s_relvel.t_prev;
    if (dt < RELVEL_MIN_DT_MS)
        return;  /* Δt가 너무 짧으면 스킵 */

    float dd = distance_cm - s_relvel.d_prev;
    float dt_sec = (float)dt / 1000.0f;
    s_relvel.velocity = dd / dt_sec;
    s_relvel.valid    = 1;

    s_relvel.d_prev   = distance_cm;
    s_relvel.t_prev   = tick_ms;
}

float RelativeVel_Get(void)
{
    return s_relvel.velocity;
}

uint8_t RelativeVel_IsValid(void)
{
    return s_relvel.valid;
}
