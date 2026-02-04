/*
 * safety_belt.h
 *
 * Safety belt output: CAN ID 0x09 수신 시 첫 바이트에 따라 GPB2 제어.
 * 0xFF = ON (HIGH), 0x00 = OFF (LOW), 기본값 OFF.
 */

#ifndef SAFETY_BELT_H
#define SAFETY_BELT_H

#include <stdint.h>
#include <gpio.h>

/* 핀 설정: B[2] */
#define SAFETYBELT_PIN    GPIO_GPB(2UL)

/* CAN 명령: raw[0] 값 */
#define SAFETYBELT_CMD_OFF    (0x00U)
#define SAFETYBELT_CMD_ON     (0xFFU)

void SAFETYBELT_Init(void);
void SAFETYBELT_SetFromCan(uint8_t first_byte);
void SAFETYBELT_start_task(void);

#endif /* SAFETY_BELT_H */
