/*
 * scheduler.c - 센서 CAN 전송 스케줄러
 *
 * [동작]
 * - RTOS Task 1개로 동작. SCHED_TICK_MS(10ms)마다 루프 실행.
 * - 각 센서마다 "마지막 전송 시각(g_last_sent_*)"과 DEFINE 매크로 주기를 비교.
 * - 주기 도래 시: XXX_get_data로 g_XXX 값을 읽고 → CAN_send_XXX(bridge)로 전송.
 * - 전송 주기는 SCHED_PERIOD_*_MS 매크로로 변경 가능.
 *
 * [데이터 흐름]
 *   센서 Task(g_XXX 갱신) → get_data(읽기) → CAN bridge(전송) → CAN TX Task
 */

#include <sal_api.h>
#include <app_cfg.h>
#include <stdint.h>
#include <debug.h>
#include "scheduler.h"
#include "can_bridge.h"
#include "accel.h"
#include "compass.h"
#include "collision.h"
#include "sonar.h"

/* =========================================================================
 * 전송 주기 (ms) - DEFINE 매크로로 변경 가능
 * ========================================================================= */
#define SCHED_PERIOD_COLLISION_MS    1000U
#define SCHED_PERIOD_SONAR_MS        2000U
#define SCHED_PERIOD_ACCEL_MS        2000U
#define SCHED_PERIOD_COMPASS_MS      1000U

/* =========================================================================
 * Task 설정
 * ========================================================================= */
#define SCHED_TASK_STK_SIZE    (1024)
#define SCHED_TASK_PRIO        (SAL_PRIO_APP_CFG)
#define SCHED_TICK_MS          10U   /* 루프 주기: 주기 체크 기준 */

/* =========================================================================
 * 로컬 변수
 * ========================================================================= */
static uint32 g_sched_task_id = 0;
static uint32 g_sched_task_stk[SCHED_TASK_STK_SIZE];

/* 각 센서별 "마지막 전송 시각" (SAL tick) */
static uint32 g_last_sent_collision = 0;
static uint32 g_last_sent_sonar     = 0;
static uint32 g_last_sent_accel     = 0;
static uint32 g_last_sent_compass   = 0;

/* =========================================================================
 * Task 루프
 * ========================================================================= */
static void Task_Scheduler(void *pArg)
{
    (void)pArg;
    uint32 now = 0;

    mcu_printf("[SCHEDULER] Task started. Periods: Collision=%u, Sonar=%u, Accel=%u, Compass=%u ms\n",
               (unsigned)SCHED_PERIOD_COLLISION_MS, (unsigned)SCHED_PERIOD_SONAR_MS,
               (unsigned)SCHED_PERIOD_ACCEL_MS, (unsigned)SCHED_PERIOD_COMPASS_MS);

    for (;;)
    {
        SAL_GetTickCount(&now);

        /* Collision: 주기 도래 시 get_data → CAN_send_collision */
        if ((now - g_last_sent_collision) >= SCHED_PERIOD_COLLISION_MS)
        {
            uint8_t val = 0;
            COLLISION_get_data(&val);
            CAN_send_collision(val);
            g_last_sent_collision = now;
        }

        /* Sonar: 주기 도래 시 get_data → CAN_send_sonar */
        if ((now - g_last_sent_sonar) >= SCHED_PERIOD_SONAR_MS)
        {
            uint16_t d0 = 0, d1 = 0;
            SONAR_get_data(&d0, &d1);
            CAN_send_sonar(d0, d1);
            g_last_sent_sonar = now;
        }

        /* Accel: 주기 도래 시 get_data → CAN_send_accel */
        if ((now - g_last_sent_accel) >= SCHED_PERIOD_ACCEL_MS)
        {
            MPU6050_Data_t data;
            MPU6050_get_data(&data);
            uint16_t moment  = (uint16_t)(data.Raw_X & 0xFFFF);
            uint16_t filtered = (uint16_t)(data.Raw_Y & 0xFFFF);
            CAN_send_accel(moment, filtered);
            g_last_sent_accel = now;
        }

        /* Compass: 주기 도래 시 get_data → CAN_send_compass */
        if ((now - g_last_sent_compass) >= SCHED_PERIOD_COMPASS_MS)
        {
            uint16_t heading = 0;
            COMPASS_get_heading(&heading);
            CAN_send_compass(heading);
            g_last_sent_compass = now;
        }

        SAL_TaskSleep(SCHED_TICK_MS);
    }
}

/* =========================================================================
 * 외부 API
 * ========================================================================= */
void SCHEDULER_start_task(void)
{
    SALRetCode_t ret;

    ret = (SALRetCode_t)SAL_TaskCreate(
        &g_sched_task_id,
        (const uint8 *)"Task_Scheduler",
        (SALTaskFunc)Task_Scheduler,
        &g_sched_task_stk[0],
        SCHED_TASK_STK_SIZE,
        SCHED_TASK_PRIO,
        NULL
    );

    if (ret != SAL_RET_SUCCESS)
    {
        mcu_printf("[SCHEDULER] Task create failed: %d\n", (int)ret);
        return;
    }
    mcu_printf("[SCHEDULER] Task created\n");
}
