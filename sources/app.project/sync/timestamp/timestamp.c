/*
 * timestamp.c - 보드 간 시간 동기화 타임스탬프 Task
 *
 * 기능: 외부 보드로부터 시작 신호를 받아 ms 단위로 시간을 카운트
 * 동작: GPIO Rising Edge 신호 수신 시 카운터를 0으로 초기화하고 1ms 단위로 증가
 */

#include <bsp.h>
#include <stdint.h>
#include <debug.h>
#include <app_cfg.h>
#include <sal_api.h>

#include "gpio.h"
#include "gic.h"
#include "timestamp.h"

// =========================================================
// 설정 및 전역 변수
// =========================================================
#define TIMESTAMP_TASK_STK_SIZE     (ACFG_TASK_NORMAL_STK_SIZE)
#define TIMESTAMP_TASK_PRIO         (SAL_PRIO_POWER_MANAGER)  // 높은 우선순위로 1ms 정확도 보장

// 태스크 관련 변수
static uint32 g_timestamp_task_id = 0;
static uint32 g_timestamp_task_stk[TIMESTAMP_TASK_STK_SIZE];

// 타임스탬프 변수 (ms 단위)
static volatile uint32_t g_timestamp_ms = 0;

// 인터럽트 플래그 (시작 신호 수신 시 카운터 초기화)
static volatile uint8 g_reset_flag = 0;

// ISR 핸들러 프로토타입
static void TIMESTAMP_ISR_handler(void *pArg);

// 태스크 함수 프로토타입
static void Task_Timestamp(void *pArg);

// =========================================================
// ISR 핸들러
// =========================================================
static void TIMESTAMP_ISR_handler(void *pArg)
{
    (void)pArg;

    // Rising edge 신호 수신 시 플래그만 설정 (최소 처리)
    // 핀이 HIGH인지 확인 (Rising edge이므로 HIGH 상태여야 함)
    if (GPIO_Get(TIMESTAMP_PIN) != 0)
    {
        g_reset_flag = 1;  // 플래그 설정
    }
}

// =========================================================
// 태스크 함수
// =========================================================
static void Task_Timestamp(void *pArg)
{
    (void)pArg;

    mcu_printf("[TIMESTAMP] Task started, waiting for sync signal...\n");

    for (;;)
    {
        // 인터럽트 플래그 확인 (시작 신호 수신)
        if (g_reset_flag != 0)
        {
            g_reset_flag = 0;  // 플래그 클리어
            
            // Critical section으로 카운터 초기화
            SAL_CoreCriticalEnter();
            g_timestamp_ms = 0;
            SAL_CoreCriticalExit();
            
            mcu_printf("[TIMESTAMP] Sync signal received, counter reset to 0\n");
        }

        // Critical section으로 카운터 증가
        SAL_CoreCriticalEnter();
        g_timestamp_ms++;
        SAL_CoreCriticalExit();
        // mcu_printf("[TIMESTAMP] Timestamp: %d ms\n", g_timestamp_ms);
        // 1ms 대기
        SAL_TaskSleep(1);
    }
}

// =========================================================
// 초기화 함수
// =========================================================
void TIMESTAMP_init(void)
{
    mcu_printf("[TIMESTAMP] Initializing...\n");

    // 1. GPIO 입력 설정 (풀다운, 입력버퍼 활성화)
    //    Rising edge 감지를 위해 풀다운 설정 (기본 LOW, 신호 시 HIGH)
    GPIO_Config(TIMESTAMP_PIN, GPIO_INPUT | GPIO_FUNC(0) | GPIO_INPUTBUF_EN | GPIO_PULLDN);

    // 2. 외부 인터럽트 연결 (GIC_EXT3에 GPIO 핀 연결)
    GPIO_IntExtSet(TIMESTAMP_IRQ_ID, TIMESTAMP_PIN);

    // 3. GIC 핸들러 등록 (Rising edge 감지)
    GIC_IntVectSet(TIMESTAMP_IRQ_ID, 10, GIC_INT_TYPE_EDGE_RISING, TIMESTAMP_ISR_handler, NULL);

    // 4. 인터럽트 활성화
    GIC_IntSrcEn(TIMESTAMP_IRQ_ID);

    // 5. 초기값 설정
    SAL_CoreCriticalEnter();
    g_timestamp_ms = 0;
    SAL_CoreCriticalExit();

    mcu_printf("[TIMESTAMP] Init done (Pin: GPB11, IRQ: %d)\n", TIMESTAMP_IRQ_ID);
}

// =========================================================
// 태스크 시작 함수
// =========================================================
void TIMESTAMP_start_task(void)
{
    SALRetCode_t ret;

    TIMESTAMP_init();

    ret = (SALRetCode_t)SAL_TaskCreate(
        &g_timestamp_task_id,
        (const uint8 *)"Timestamp Task",
        (SALTaskFunc)Task_Timestamp,
        &g_timestamp_task_stk[0],
        TIMESTAMP_TASK_STK_SIZE,
        TIMESTAMP_TASK_PRIO,
        NULL
    );

    if (ret != SAL_RET_SUCCESS)
    {
        mcu_printf("[TIMESTAMP] Task create failed: %d\n", ret);
        return;
    }

    mcu_printf("[TIMESTAMP] Task created successfully\n");
}

// =========================================================
// 타임스탬프 읽기 함수
// =========================================================
void TIMESTAMP_get_ms(uint32_t *pTimestamp)
{
    if (pTimestamp == NULL)
    {
        return;
    }

    // Critical section으로 안전하게 읽기
    SAL_CoreCriticalEnter();
    *pTimestamp = g_timestamp_ms;
    SAL_CoreCriticalExit();
}
