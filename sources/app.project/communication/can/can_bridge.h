#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include <stdint.h>
#include "can_app.h"
// dev.drivers의 CANMessage_t는 can_bridge.c에서 직접 인클루드

// CAN 메시지 타입 정의
#define CAN_type_break_led    0x048

#define CAN_type_collision    0x008
#define CAN_type_safety_belt 0x009   /* 안전벨트 출력 제어 (raw[0]: 0xFF=ON, 0x00=OFF) */
#define CAN_type_sonar        0x024   /* 초음파 거리 [cm] uint16 x2 */
#define CAN_type_rel_velocity 0x02C   /* 상대 속도 [cm/s] uint32 */
#define CAN_type_compass      0x084

// CAN 메시지 소비 함수 선언
// RxHeader는 CAN 드라이버의 CANMessage_t 구조체를 사용
// 실제 구현은 can_bridge.c에서 (필요시 별도 파일로 분리 가능)
void CAN_consume_rx_message(const void *pRxHeader, CAN_payload_t rxPayload);

// 수신 메시지 처리 함수들
void CAN_receive_led_signal(uint8_t type);

// 메시지 전송 함수들
void CAN_send_message(uint16_t id, CAN_queue_pkt_t *pPacket);
void CAN_send_collision(uint8_t val);
void CAN_send_sonar(uint16_t distance0, uint16_t distance1);
void CAN_send_rel_velocity(uint32_t velocity_cm_per_s);
void CAN_send_compass(uint16_t heading);

#endif // CAN_BRIDGE_H
