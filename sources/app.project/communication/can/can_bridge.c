/*
 * can_bridge.c
 *
 *  Created on: Jan 3, 2026
 *      Author: mokta
 */

#include <sal_api.h>
#include <debug.h>
#include "can_bridge.h"
#include "can.h"  // 프로젝트의 can.h
#include "can_config.h"
#include <can.h>  // dev.drivers의 CAN 헤더 (전역 경로)
#include "utils.h"

// grid_led.h는 필요시 추가
// #include "grid_led.h"

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
