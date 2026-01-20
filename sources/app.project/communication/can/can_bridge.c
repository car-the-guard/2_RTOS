/*
 * can_bridge.c
 *
 *  Created on: Jan 3, 2026
 *      Author: mokta
 */

#include <sal_api.h>
#include <debug.h>
#include "can_bridge.h"
#include "can_app.h"  // 프로젝트의 can_app.h
#include "can_config.h"
#include "can.h"  // dev.drivers의 CAN 헤더
#include "utils.h"
// grid_led.h는 필요시 추가
// #include "grid_led.h"

// can.c에서 extern으로 접근
extern uint32 g_canTxQueueHandle;
extern CAN_queue_pkt_t* CAN_AllocPool(void);
extern void CAN_FreePool(CAN_queue_pkt_t *pPkt);
// CAN_GetTxQueueCount, CAN_IncrementTxQueueCount는 can_app.h에 선언되어 있음

static void Print_Hex_8Bytes(uint8_t *data)
{
    // [DEBUG] HEX: 라는 문구와 함께 출력
    mcu_printf("HEX: ");

    for (int i = 0; i < 8; i++)
    {
        // %02X 의미:
        // 0: 빈 자리를 0으로 채움 (예: A -> 0A)
        // 2: 최소 2자리로 출력
        // X: 대문자 16진수 (x를 쓰면 소문자 출력)
        mcu_printf("%02X ", data[i]);
    }

    mcu_printf("\r\n");
}
/* -------------------------------------------------------------------------
   CAN 메시지 소비 함수
   ------------------------------------------------------------------------- */
void CAN_consume_rx_message(const void *pRxHeader, CAN_payload_t rxPayload)
{
    const CANMessage_t *pMsg = (const CANMessage_t *)pRxHeader;
    uint32_t id = pMsg->mId;
    mcu_printf("[CAN_BRIDGE] Received message: ");
    Print_Hex_8Bytes(rxPayload.raw);
    mcu_printf("[CAN_BRIDGE] Received message ID: 0x%03X\r\n", id);

    switch(id)
    {
        case CAN_type_break_led:
            CAN_receive_led_signal(rxPayload.field.data.u8_val);
            break;
        default:
            mcu_printf("[CAN_BRIDGE] Unknown message ID: 0x%03X\r\n", id);
            break;
    }
}

/* -------------------------------------------------------------------------
   수신 메시지 처리 함수들
   ------------------------------------------------------------------------- */
void CAN_receive_led_signal(uint8_t type)
{
    // TODO: grid_led.h의 GRIDLED_SetState 함수 호출
    // GRIDLED_SetState((GridLed_State_t) type);
    mcu_printf("[CAN_BRIDGE] LED signal received: 0x%02X\r\n", type);
}

/* -------------------------------------------------------------------------
   메시지 전송 함수들 (외부 API)
   ------------------------------------------------------------------------- */
void CAN_send_message(uint16_t id, CAN_queue_pkt_t *pPacket)
{
    if (pPacket == NULL_PTR)
    {
        mcu_printf("[CAN_BRIDGE] CAN_send_message: pPacket is NULL\r\n");
        return;
    }
    
    mcu_printf("[CAN_BRIDGE] CAN_send_message: id=0x%03X, queue_handle=%u, packet=%p\r\n", id, (unsigned int)g_canTxQueueHandle, pPacket);
    
    SALRetCode_t ret = SAL_QueuePut(g_canTxQueueHandle,
                                    &pPacket,
                                    sizeof(CAN_queue_pkt_t*),
                                    0,  // timeout
                                    0); // blocking option
    
    if (ret != SAL_RET_SUCCESS)
    {
        mcu_printf("[CAN_BRIDGE] Queue put failed: %d\r\n", ret);
        CAN_FreePool(pPacket);  // 실패 시 풀에 반환
    }
    else
    {
        // 큐에 메시지를 넣었으므로 카운트 증가
        CAN_IncrementTxQueueCount();
        
        // 큐에 담긴 메시지 개수 확인
        uint32 queueCount = CAN_GetTxQueueCount();
        mcu_printf("[CAN_BRIDGE] Queue put success: id=0x%03X, queue_count=%u\r\n", id, (unsigned int)queueCount);
    }
}

void CAN_send_collision(void)
{
    mcu_printf("[CAN_BRIDGE] Sending collision message\n");
    CAN_queue_pkt_t *pPacket = CAN_AllocPool();
    
    if (pPacket != NULL_PTR)
    {
        pPacket->id = CAN_type_break_led;
        pPacket->body.field.data.u8_val = 0xFF;
        CAN_send_message(CAN_type_break_led, pPacket);
    }
    else {
        mcu_printf("[CAN_BRIDGE] CAN_AllocPool failed\n");
    }
}

void CAN_send_sonar(uint16_t distance0, uint16_t distance1)
{
    CAN_queue_pkt_t *pPacket = CAN_AllocPool();
    
    if (pPacket != NULL_PTR)
    {
        pPacket->id = CAN_type_sonar;
        pPacket->body.field.data.dual_u16.val_A = distance0;
        pPacket->body.field.data.dual_u16.val_B = distance1;
        CAN_send_message(CAN_type_sonar, pPacket);
    }
}

void CAN_send_accel(uint16_t accel_moment, uint16_t accel_filtered)
{
    CAN_queue_pkt_t *pPacket = CAN_AllocPool();
    
    if (pPacket != NULL_PTR)
    {
        pPacket->id = CAN_type_accel;
        pPacket->body.field.data.dual_u16.val_A = accel_moment;
        pPacket->body.field.data.dual_u16.val_B = accel_filtered;
        CAN_send_message(CAN_type_accel, pPacket);
    }
}

void CAN_send_compass(uint16_t heading)
{
    CAN_queue_pkt_t *pPacket = CAN_AllocPool();
    
    if (pPacket != NULL_PTR)
    {
        pPacket->id = CAN_type_compass;
        pPacket->body.field.data.u16_val = heading;
        CAN_send_message(CAN_type_compass, pPacket);
    }
}
