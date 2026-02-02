/*
 * relative_velocity.h - 거리 시계열로부터 상대 속도 추정
 *
 * (거리, 시간) 쌍을 입력받아 d(distance)/dt로 접근/이탈 속도를 계산.
 * 센서 모듈과 독립적인 순수 알고리즘 (utils 계층).
 *
 * 호출 예: sonar에서 거리 갱신 시 RelativeVel_Update(distance_cm, tick_ms)
 */

#ifndef RELATIVE_VELOCITY_H
#define RELATIVE_VELOCITY_H

#include <stdint.h>

/**
 * @brief 상대 속도 추정기 초기화
 */
void RelativeVel_Init(void);

/**
 * @brief 거리와 시간 입력 (매 측정마다 호출)
 *
 * @param distance_cm  현재 거리 [cm]
 * @param tick_ms      현재 시각 [ms] (SAL_GetTickCount 등)
 */
void RelativeVel_Update(float distance_cm, uint32_t tick_ms);

/**
 * @brief 현재 추정 상대 속도 반환
 *
 * @return float  [cm/s] 양수=멀어짐, 음수=다가옴
 */
float RelativeVel_Get(void);

/**
 * @brief 유효한 속도 추정값 존재 여부 (2회 이상 측정 필요)
 */
uint8_t RelativeVel_IsValid(void);

#endif /* RELATIVE_VELOCITY_H */
