/*
 * matrix_led.h
 *
 *  Created on: Jan 1, 2026
 *  Author: mokta
 *
 *  MAX7219 Matrix LED 제어 (1 Strip x 4 Chip, GPSB/SPI).
 *  grid_led / MAX7219 계열 코드를 Telechips topst-rtos에 포팅.
 */

#ifndef MATRIX_LED_H_
#define MATRIX_LED_H_

#include <stdint.h>

/* 상태 (CAN type과 호환: 0~3) */
typedef enum {
    MATRIX_LED_OFF      = 0, /* 모두 끄기 */
    MATRIX_LED_ON       = 1, /* 모두 켜기 */
    MATRIX_LED_BLINK    = 2, /* 빠르게 점멸 */
    MATRIX_LED_ANIMATE  = 3  /* 좌우 화살표 애니메이션 */
} MatrixLed_State_t;

void MATRIXLED_Init(void);
void MATRIXLED_SetState(MatrixLed_State_t state);
void MATRIXLED_Process(void);
void MATRIXLED_CreateAppTask(void);
void MATRIXLED_Test(void);

#endif /* MATRIX_LED_H_ */
