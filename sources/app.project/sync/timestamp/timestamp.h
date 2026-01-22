#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <sal_api.h>
#include <stdint.h>

// 핀 설정
#define TIMESTAMP_PIN       GPIO_GPB(11)
#define TIMESTAMP_IRQ_ID    GIC_EXT3    // GIC 인터럽트 ID (EXT0~EXT9 사용 가능)

// 초기화 함수
void TIMESTAMP_init(void);

// 태스크 시작 함수
void TIMESTAMP_start_task(void);

// 현재 타임스탬프 읽기 (ms 단위, critical section 보호)
void TIMESTAMP_get_ms(uint32_t *pTimestamp);

#endif // TIMESTAMP_H
