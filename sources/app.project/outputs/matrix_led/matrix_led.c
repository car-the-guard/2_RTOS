/*
 * matrix_led.c
 *
 *  Created on: Jan 1, 2026
 *  Author: mokta
 *
 *  MAX7219 Matrix LED 제어 (1 Strip x 4 Chip). GPSB(SPI) 사용.
 *  grid_led / MAX7219 / MAX7219_matrix 계열을 topst-rtos에 포팅.
 *  참고: spi_led.c (GPSB, GPIO, CsActivate/Deactivate, Xfer), dot_matrix.c (MAX7219 레지스터, CsInit).
 */

#include <gpsb.h>
#include <gpio.h>
#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include "matrix_led.h"

/* -------------------------------------------------------------------------
 * 설정 (1 Strip x 4 Chip). GPB4~7: SCLK=B4, CS=B5, MOSI=B6, MISO=B7, GPSB CH0.
 * ------------------------------------------------------------------------- */
#define MATRIX_LED_GPSB_CH       (0U)
#define MATRIX_LED_CS_GPIO       GPIO_GPB(5UL)
#define MATRIX_LED_SCLK_GPIO     GPIO_GPB(4UL)
#define MATRIX_LED_MOSI_GPIO     GPIO_GPB(6UL)
#define MATRIX_LED_MISO_GPIO     GPIO_GPB(7UL)
#define MATRIX_LED_GPIO_FUNC     (1UL)
#define MATRIX_LED_IC_NUM        (4U)

/* MAX7219 레지스터 */
#define MAX7219_REG_NOOP         (0x00U)
#define MAX7219_REG_DECODE       (0x09U)
#define MAX7219_REG_INTENSITY    (0x0AU)
#define MAX7219_REG_SCANLIMIT    (0x0BU)
#define MAX7219_REG_SHUTDOWN     (0x0CU)
#define MAX7219_REG_TEST         (0x0FU)

/* Strip 1, Chip 4, Row 8 */
static volatile uint8_t FrameBuffer[1][4][8];

static volatile MatrixLed_State_t current_led_state = MATRIX_LED_OFF;

/* 화살표 패턴 (8x8) */
static const uint8_t arrow_bmp[8] = {
    0b00001111,
    0b00011110,
    0b00111100,
    0b01111000,
    0b01111000,
    0b00111100,
    0b00011110,
    0b00001111
};

/* -------------------------------------------------------------------------
 * MAX7219 로우레벨: 4칩 daisy-chain. chip_idx 0~3.
 * ------------------------------------------------------------------------- */
static int MAX7219_Write(uint8_t chip_idx, uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[8];
    uint8_t rx_buf[8] = { 0 };
    uint32_t xfer_mode = GPSB_XFER_MODE_PIO | GPSB_XFER_MODE_WITHOUT_INTERRUPT;
    uint32_t wi = 0;
    uint8_t i;

    if (chip_idx >= MATRIX_LED_IC_NUM)
        return -1;

    /* 타겟보다 뒤(인덱스 큰) 칩: NO-OP */
    for (i = (uint8_t)(chip_idx + 1U); i < MATRIX_LED_IC_NUM; i++) {
        tx_buf[wi++] = MAX7219_REG_NOOP;
        tx_buf[wi++] = 0x00U;
    }
    tx_buf[wi++] = reg;
    tx_buf[wi++] = data;
    /* 타겟보다 앞(인덱스 작은) 칩: NO-OP */
    for (i = 0; i < chip_idx; i++) {
        tx_buf[wi++] = MAX7219_REG_NOOP;
        tx_buf[wi++] = 0x00U;
    }

    (void)GPSB_CsActivate(MATRIX_LED_GPSB_CH, MATRIX_LED_CS_GPIO, 0U);
    (void)GPSB_Xfer(MATRIX_LED_GPSB_CH, tx_buf, rx_buf, 8, xfer_mode);
    (void)GPSB_CsDeactivate(MATRIX_LED_GPSB_CH, MATRIX_LED_CS_GPIO, 0U);
    return 0;
}

/* -------------------------------------------------------------------------
 * 매트릭스 레이어 (strip_idx는 0만 사용)
 * ------------------------------------------------------------------------- */
static void MatrixSetPixel(uint8_t strip_idx, uint8_t chip_idx, uint8_t row, uint8_t data)
{
    if (strip_idx == 0U && chip_idx < MATRIX_LED_IC_NUM && row < 8U)
        FrameBuffer[0][chip_idx][row] = data;
}

static void MatrixClearAll(void)
{
    uint8_t c, r;
    for (c = 0; c < MATRIX_LED_IC_NUM; c++)
        for (r = 0; r < 8U; r++)
            FrameBuffer[0][c][r] = 0U;
}

static void MatrixUpdate(void)
{
    uint8_t c, r;
    for (c = 0; c < MATRIX_LED_IC_NUM; c++)
        for (r = 0; r < 8U; r++)
            (void)MAX7219_Write(c, (uint8_t)(r + 1U), FrameBuffer[0][c][r]);
}

static void MatrixShutDown(uint8_t chip_idx, uint8_t on)
{
    (void)MAX7219_Write(chip_idx, MAX7219_REG_SHUTDOWN, on ? 1U : 0U);
}

/* -------------------------------------------------------------------------
 * 헬퍼
 * ------------------------------------------------------------------------- */
static void Fill_Strip_Pattern(const uint8_t *pattern)
{
    uint8_t c, r;
    for (c = 0; c < MATRIX_LED_IC_NUM; c++)
        for (r = 0; r < 8U; r++)
            MatrixSetPixel(0, c, r, pattern[r]);
}

static void Control_Strip_Power(uint8_t on)
{
    uint8_t c;
    for (c = 0; c < MATRIX_LED_IC_NUM; c++)
        MatrixShutDown(c, on);
}

static void Shift_Strip0_Left(void)
{
    uint8_t c, r;
    uint8_t msb;
    for (c = 0; c < MATRIX_LED_IC_NUM; c++) {
        for (r = 0; r < 8U; r++) {
            msb = (FrameBuffer[0][c][r] & 0x80U) ? 1U : 0U;
            FrameBuffer[0][c][r] = (uint8_t)((FrameBuffer[0][c][r] << 1) | msb);
        }
    }
}

/* -------------------------------------------------------------------------
 * 공개 API
 * ------------------------------------------------------------------------- */
void MATRIXLED_Init(void)
{
    GPSBOpenParam_t param;
    uint8_t c, r;

    /* GPB4~7: CS=GPIO, SCLK/MOSI=ALT(GPSB), dot_matrix 방식 */
    (void)GPIO_Config(MATRIX_LED_CS_GPIO, (GPIO_FUNC(0UL) | GPIO_OUTPUT));
    GPIO_Set(MATRIX_LED_CS_GPIO, 1U);
    (void)GPIO_Config(MATRIX_LED_SCLK_GPIO, GPIO_FUNC(MATRIX_LED_GPIO_FUNC));
    (void)GPIO_Config(MATRIX_LED_MOSI_GPIO, GPIO_FUNC(MATRIX_LED_GPIO_FUNC));

    param.uiSdo        = MATRIX_LED_MOSI_GPIO;
    param.uiSdi        = MATRIX_LED_MISO_GPIO;
    param.uiSclk       = MATRIX_LED_SCLK_GPIO;
    param.pDmaAddrTx   = NULL_PTR;
    param.pDmaAddrRx   = NULL_PTR;
    param.uiDmaBufSize = 0UL;
    param.fbCallback   = NULL_PTR;
    param.pArg         = NULL_PTR;
    param.uiIsSlave    = GPSB_MASTER_MODE;

    if (GPSB_Open(MATRIX_LED_GPSB_CH, param) != SAL_RET_SUCCESS) {
        mcu_printf("[MATRIXLED] GPSB_Open failed\n");
        return;
    }
    (void)GPSB_SetBpw(MATRIX_LED_GPSB_CH, 16UL);
    (void)GPSB_SetSpeed(MATRIX_LED_GPSB_CH, 1000000UL);
    (void)GPSB_CsInit(MATRIX_LED_GPSB_CH, MATRIX_LED_CS_GPIO, 0U);

    /* MAX7219 초기화: 4칩 공통 */
    for (c = 0; c < MATRIX_LED_IC_NUM; c++) {
        (void)MAX7219_Write(c, MAX7219_REG_TEST, 0U);
        (void)MAX7219_Write(c, MAX7219_REG_SHUTDOWN, 1U);
        (void)MAX7219_Write(c, MAX7219_REG_DECODE, 0U);  /* 0x09 Decode */
        (void)MAX7219_Write(c, MAX7219_REG_INTENSITY, 1U);
        (void)MAX7219_Write(c, MAX7219_REG_SCANLIMIT, 7U);
        for (r = 0; r < 8U; r++)
            (void)MAX7219_Write(c, (uint8_t)(r + 1U), 0U);
    }

    MatrixClearAll();
    MatrixUpdate();
}

void MATRIXLED_SetState(MatrixLed_State_t state)
{
    current_led_state = state;
}

void MATRIXLED_Test(void)
{
    current_led_state = (current_led_state + 1) % 4;
    mcu_printf("[MATRIXLED] Test: %d\n", current_led_state);
}

static void MATRIXLED_Task_Loop(void *pArg)
{
    (void)pArg;
    for (;;)
        MATRIXLED_Process();
}

void MATRIXLED_Process(void)
{
    static MatrixLed_State_t prev_state = (MatrixLed_State_t)-1;
    static uint8_t blink_flag = 0U;

    /* 상태 변경 시 초기화 */
    if (prev_state != current_led_state) {
        MatrixClearAll();
        Control_Strip_Power(1);
        if (current_led_state == MATRIX_LED_ANIMATE)
            Fill_Strip_Pattern(arrow_bmp);
        MatrixUpdate();
        prev_state = current_led_state;
    }

    switch (current_led_state) {
    case MATRIX_LED_OFF:
        (void)SAL_TaskSleep(200);
        break;

    case MATRIX_LED_ON: {
        uint8_t c, r;
        for (c = 0; c < MATRIX_LED_IC_NUM; c++)
            for (r = 0; r < 8U; r++)
                MatrixSetPixel(0, c, r, 0xFFU);
        MatrixUpdate();
        (void)SAL_TaskSleep(200);
        break;
    }

    case MATRIX_LED_BLINK:
        Control_Strip_Power(blink_flag ? 1 : 0);
        if (blink_flag) {
            Fill_Strip_Pattern(arrow_bmp);
            MatrixUpdate();
        }
        blink_flag = blink_flag ? 0U : 1U;
        (void)SAL_TaskSleep(200);
        break;

    case MATRIX_LED_ANIMATE:
        Shift_Strip0_Left();
        MatrixUpdate();
        (void)SAL_TaskSleep(50);
        break;
    }
}

void MATRIXLED_CreateAppTask(void)
{
    static uint32_t taskId;
    static uint32_t taskStk[ACFG_TASK_NORMAL_STK_SIZE];

    (void)SAL_TaskCreate(&taskId,
                         (const uint8 *)"MATRIXLED",
                         (SALTaskFunc)MATRIXLED_Task_Loop,
                         (void * const)&taskStk[0],
                         ACFG_TASK_NORMAL_STK_SIZE,
                         SAL_PRIO_APP_CFG,
                         NULL_PTR);
}
