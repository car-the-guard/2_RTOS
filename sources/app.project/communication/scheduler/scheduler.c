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
#include "compass.h"
#include "sonar.h"
#include "relative_velocity.h"
/* Note: collision.h는 collision 태스크에서 직접 CAN 전송하므로 scheduler에서 불필요 */

/* =========================================================================
 * 전송 주기 (ms) - DEFINE 매크로로 변경 가능
 * ========================================================================= */
/* Note: Collision은 충돌 발생 시 즉시 전송하므로 주기 없음 */
#define SCHED_PERIOD_SONAR_MS          200U   /* 0x24 거리 */
#define SCHED_PERIOD_REL_VELOCITY_MS   200U   /* 0x2C 상대속도 */
#define SCHED_PERIOD_COMPASS_MS        1000U

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
/* Note: Collision은 충돌 발생 시 즉시 전송하므로 scheduler에서 관리하지 않음 */
static uint32 g_last_sent_sonar         = 0;
static uint32 g_last_sent_rel_velocity  = 0;
static uint32 g_last_sent_compass       = 0;

/* =========================================================================
 * Task 루프
 * ========================================================================= */
static void Task_Scheduler(void *pArg)
{
    (void)pArg;
    uint32 now = 0;

    mcu_printf("[SCHEDULER] Task started. Periods: Collision=immediate, Sonar(0x24)=%u, RelVel(0x2C)=%u, Compass=%u ms\n",
               (unsigned)SCHED_PERIOD_SONAR_MS,
               (unsigned)SCHED_PERIOD_REL_VELOCITY_MS,
               (unsigned)SCHED_PERIOD_COMPASS_MS);

    for (;;)
    {
        SAL_GetTickCount(&now);

        /* Collision: 충돌 발생 시 즉시 전송 (scheduler에서 주기 전송하지 않음) */

        /* Sonar: 주기 도래 시 get_data → CAN_send_sonar */
        if ((now - g_last_sent_sonar) >= SCHED_PERIOD_SONAR_MS)
        {
            uint16_t d0 = 0, d1 = 0;
            SONAR_get_data(&d0, &d1);
            CAN_send_sonar(d0, d1);
            g_last_sent_sonar = now;
        }

        /* RelVelocity (0x2C): 상대 속도 [cm/s] uint32만 전송 */
        if ((now - g_last_sent_rel_velocity) >= SCHED_PERIOD_REL_VELOCITY_MS)
        {
            float vel_f = RelativeVel_Get();
            int32_t vel_i = (vel_f < -2147483648.0f) ? (int32_t)0x80000000
                : (vel_f > 2147483647.0f) ? 2147483647 : (int32_t)vel_f;
            uint32_t vel_u = (uint32_t)vel_i;
            CAN_send_rel_velocity(vel_u);
            g_last_sent_rel_velocity = now;
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
