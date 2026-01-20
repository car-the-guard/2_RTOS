/*
 * can_bridge.c
 *
 * CAN 메시지 소비 함수 구현
 */

#include <sal_api.h>
#include <debug.h>
#include "can_bridge.h"
#include "can.h"  // 프로젝트의 can.h
#include "can_config.h"
#include <can.h>  // dev.drivers의 CAN 헤더 (전역 경로)

/* -------------------------------------------------------------------------
   CAN 메시지 소비 함수
   ------------------------------------------------------------------------- */
void CAN_consume_rx_message(const void *pRxHeader, CAN_payload_t rxPayload)
{
    const CANMessage_t *pMsg = (const CANMessage_t *)pRxHeader;
    
    (void)rxPayload;
    
    // TODO: 실제 메시지 처리 로직 구현
    // 예: 특정 ID에 따라 다른 처리 수행
    // 예: 센서 데이터 업데이트, 상태 변경 등
    
    mcu_printf("[CAN_BRIDGE] Received message ID: 0x%03X\r\n", pMsg->mId);
    
    // CRC 검증 (필요시)
    // if(calculate_CRC8(rxPayload.raw, 7) != rxPayload.field.CRC_8) {
    //     mcu_printf("[CAN_BRIDGE] CRC Doesn't Match\r\n");
    //     return;
    // }
}
