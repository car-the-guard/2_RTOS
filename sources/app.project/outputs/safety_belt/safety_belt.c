/*
 * safety_belt.c
 *
 * Safety belt output: CAN ID 0x09 수신 시 첫 바이트(0xFF=ON, 0x00=OFF)에 따라
 * GPB2 GPIO 출력 제어. 기본 상태 OFF.
 */

#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <gpio.h>
#include "safety_belt.h"

#define SAFETYBELT_TASK_STK_SIZE   (256U)
#define SAFETYBELT_TASK_PRIO       (SAL_PRIO_APP_CFG)

static uint32_t g_safetybelt_task_id = 0;
static uint32_t g_safetybelt_task_stk[SAFETYBELT_TASK_STK_SIZE];

static void Task_SafetyBelt(void *pArg);

/* -------------------------------------------------------------------------
   초기화
   ------------------------------------------------------------------------- */
void SAFETYBELT_Init(void)
{
    mcu_printf("[SAFETYBELT] Initializing...\n");

    (void)GPIO_Config(SAFETYBELT_PIN, GPIO_OUTPUT | GPIO_FUNC(0) | GPIO_NOPULL);
    GPIO_Set(SAFETYBELT_PIN, 0U);  /* 기본 OFF */

    mcu_printf("[SAFETYBELT] Init done (Pin: GPB2, default OFF)\n");
}

/* -------------------------------------------------------------------------
   CAN 수신 첫 바이트 반영: 0xFF=ON, 그 외=OFF
   ------------------------------------------------------------------------- */
void SAFETYBELT_SetFromCan(uint8_t first_byte)
{
    if (first_byte == SAFETYBELT_CMD_ON)
    {
        GPIO_Set(SAFETYBELT_PIN, 1U);
    }
    else
    {
        GPIO_Set(SAFETYBELT_PIN, 0U);
    }
}

/* -------------------------------------------------------------------------
   태스크 루프 (상태는 CAN 콜백에서만 갱신, 여기서는 주기 sleep)
   ------------------------------------------------------------------------- */
static void Task_SafetyBelt(void *pArg)
{
    (void)pArg;

    mcu_printf("[SAFETYBELT] Task started\n");

    for (;;)
    {
        SAL_TaskSleep(100);
    }
}

/* -------------------------------------------------------------------------
   태스크 시작
   ------------------------------------------------------------------------- */
void SAFETYBELT_start_task(void)
{
    SALRetCode_t ret;

    SAFETYBELT_Init();

    ret = (SALRetCode_t)SAL_TaskCreate(
        &g_safetybelt_task_id,
        (const uint8 *)"SafetyBelt Task",
        (SALTaskFunc)Task_SafetyBelt,
        &g_safetybelt_task_stk[0],
        SAFETYBELT_TASK_STK_SIZE,
        SAFETYBELT_TASK_PRIO,
        NULL
    );

    if (ret != SAL_RET_SUCCESS)
    {
        mcu_printf("[SAFETYBELT] Task create failed: %d\n", ret);
        return;
    }

    mcu_printf("[SAFETYBELT] Task created successfully\n");
}
