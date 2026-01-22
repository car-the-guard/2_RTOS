/*
 * can.h
 *
 *  Created on: Jan 2, 2026
 *      Author: mokta
 */

#ifndef CAN_APP_H
#define CAN_APP_H

#include <sal_api.h>
#include <stdint.h>
// dev.drivers의 CAN 헤더는 can.c에서 직접 인클루드

// CAN 메시지 타입 정의
#define CAN_type_break_led    0x048

#define CAN_type_collision    0x008
#define CAN_type_sonar        0x024
#define CAN_type_accel        0x02C
#define CAN_type_compass      0x030

// 전역 변수 선언
extern uint32 g_canTxQueueHandle; // Gatekeeper용 Tx 큐 핸들
extern uint32 g_canRxQueueHandle; // Rx 큐 핸들

#pragma pack(push, 1) // 패딩 방지 (필수)
typedef union {
    struct {
        union {
            uint32_t u32_all;    // 전체 4바이트 (디버깅용 또는 32비트 값)
            uint8_t  u8_val;     // [Case 1] 1 byte (Byte 0 사용)
            uint16_t u16_val;    // [Case 2] 2 bytes (Byte 0~1 사용)
            struct {             // [Case 3] 2 bytes x 2개 (Byte 0~1, Byte 2~3)
                int16_t val_A;   // Byte 0~1 (예: X축 가속도)
                int16_t val_B;   // Byte 2~3 (예: Y축 가속도)
            } dual_u16;
        } data;

        uint16_t time_ms;   // 2 bytes (공통)
        uint8_t  reserved;  // 1 byte  (공통)
        uint8_t  CRC_8;     // 1 byte  (공통)
    } field;

    uint8_t raw[8];
} CAN_payload_t;
#pragma pack(pop)

typedef struct {
    uint16_t 		id;
	CAN_payload_t 	body;
} CAN_queue_pkt_t;

/* 함수 원형 선언 */
void CAN_init(void);        				// 필터 설정 및 CAN 시작 함수
void CAN_start_task(void); 					// 태스크 시작 함수 (Tx + Rx 모두)
void CAN_TX_start_task(void); 					// Tx 태스ㄴㄴㄴ크 시작 함수
void CAN_RX_start_task(void); 					// Rx 태스크 시작 함수

// 메모리 풀 관리 함수 (can.c에서 정의)
CAN_queue_pkt_t* CAN_AllocPool(void);
void CAN_FreePool(CAN_queue_pkt_t *pPkt);

// 큐 메시지 개수 추적 함수 (디버깅용)
uint32 CAN_GetTxQueueCount(void);
void CAN_IncrementTxQueueCount(void);

#endif /* CAN_APP_H */
