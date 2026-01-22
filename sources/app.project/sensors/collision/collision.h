#ifndef COLLISION_H
#define COLLISION_H

#include <sal_api.h>

// 핀 설정
#define COLLISION_PIN       GPIO_GPA(6)
#define COLLISION_IRQ_ID    GIC_EXT2    // GIC 인터럽트 ID (EXT0~EXT9 사용 가능)

// 콜백 함수 타입
typedef void (*CollisionCallback_t)(void);

// 초기화 함수
void COLLISION_init(void);

// 태스크 시작 함수
void COLLISION_start_task(void);

// 콜백 등록 함수
void COLLISION_register_callback(CollisionCallback_t callback);

// g_collision 조회 (scheduler용)
void COLLISION_get_data(uint8_t *pVal);

#endif // COLLISION_H
