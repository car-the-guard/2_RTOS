#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include <stdint.h>
#include "can.h"
// dev.drivers의 CANMessage_t는 can_bridge.c에서 직접 인클루드

// CAN 메시지 소비 함수 선언
// RxHeader는 CAN 드라이버의 CANMessage_t 구조체를 사용
// 실제 구현은 can_bridge.c에서 (필요시 별도 파일로 분리 가능)
void CAN_consume_rx_message(const void *pRxHeader, CAN_payload_t rxPayload);

// 수신 메시지 처리 함수들
void CAN_receive_led_signal(uint8_t type);

#endif // CAN_BRIDGE_H
