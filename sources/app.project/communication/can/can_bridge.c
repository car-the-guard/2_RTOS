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
#include "matrix_led.h"

// can.c에서 extern으로 접근
extern uint32 g_canTxQueueHandle;
extern CAN_queue_pkt_t* CAN_AllocPool(void);
extern void CAN_FreePool(CAN_queue_pkt_t *pPkt);
// CAN_GetTxQueueCount, CAN_IncrementTxQueueCount는 can_app.h에 선언되어 있음

/* -------------------------------------------------------------------------
   CAN 메시지 소비 함수
   ------------------------------------------------------------------------- */
void CAN_consume_rx_message(const void *pRxHeader, CAN_payload_t rxPayload)
{
    const CANMessage_t *pMsg = (const CANMessage_t *)pRxHeader;
    uint32_t id = pMsg->mId;

    switch(id)
    {
        case CAN_type_break_led:
            CAN_receive_led_signal(rxPayload.field.data.u8_val);
            break;
        default:
            break;
    }
}

/* -------------------------------------------------------------------------
   수신 메시지 처리 함수들
   ------------------------------------------------------------------------- */
void CAN_receive_led_signal(uint8_t type)
{
    MATRIXLED_SetState((MatrixLed_State_t)type);
}

/* -------------------------------------------------------------------------
   메시지 전송 함수들 (외부 API)
   ------------------------------------------------------------------------- */
void CAN_send_message(uint16_t id, CAN_queue_pkt_t *pPacket)
{
    if (pPacket == NULL_PTR)
    {
        return;
    }
    
    SALRetCode_t ret = SAL_QueuePut(g_canTxQueueHandle,
                                    &pPacket,
                                    sizeof(CAN_queue_pkt_t*),
                                    0,  // timeout
                                    0); // blocking option
    
    if (ret != SAL_RET_SUCCESS)
    {
        CAN_FreePool(pPacket);  // 실패 시 풀에 반환
    }
    else
    {
        // 큐에 메시지를 넣었으므로 카운트 증가
        CAN_IncrementTxQueueCount();
    }
}

void CAN_send_collision(uint8_t val)
{
    CAN_queue_pkt_t *pPacket = CAN_AllocPool();
    
    if (pPacket != NULL_PTR)
    {
        pPacket->id = CAN_type_collision;
        pPacket->body.field.data.u8_val = val;
        CAN_send_message(CAN_type_collision, pPacket);
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

void CAN_send_rel_velocity(uint32_t velocity_cm_per_s)
{
    CAN_queue_pkt_t *pPacket = CAN_AllocPool();
    
    if (pPacket != NULL_PTR)
    {
        pPacket->id = CAN_type_rel_velocity;
        /* 레이아웃: [0-3] velocity uint32_t [cm/s] */
        pPacket->body.raw[0] = (uint8_t)(velocity_cm_per_s & 0xFFU);
        pPacket->body.raw[1] = (uint8_t)((velocity_cm_per_s >> 8) & 0xFFU);
        pPacket->body.raw[2] = (uint8_t)((velocity_cm_per_s >> 16) & 0xFFU);
        pPacket->body.raw[3] = (uint8_t)((velocity_cm_per_s >> 24) & 0xFFU);
        pPacket->body.raw[4] = 0U;
        pPacket->body.raw[5] = 0U;
        pPacket->body.raw[6] = 0U;
        CAN_send_message(CAN_type_rel_velocity, pPacket);
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
