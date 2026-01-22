/*
 * collision.c - YWRBOT 충격 감지 센서 드라이버
 *
 * 센서 동작: HIGH 유지 → 충격 시 LOW (Falling edge 감지)
 * 구현 방식: EXTI + 플래그 기반 (디바운싱 포함)
 */

#include <bsp.h>
#include <stdint.h>
#include <debug.h>
#include <app_cfg.h>
#include <sal_api.h>

#include "gpio.h"
#include "gic.h"
#include "collision.h"
#include "matrix_led.h"
#include "can_bridge.h"

// =========================================================
// 설정 및 전역 변수
// =========================================================
#define COLLISION_TASK_STK_SIZE     (2048)
#define COLLISION_TASK_PRIO         (SAL_PRIO_APP_CFG)

// 태스크 관련 변수
static uint32 g_collision_task_id = 0;
static uint32 g_collision_task_stk[COLLISION_TASK_STK_SIZE];

// 콜백 함수 포인터
static CollisionCallback_t g_collision_callback = NULL;

// 디버그용 ISR 호출 카운터
static volatile uint32 g_isr_count = 0;

// 채터링 방지용 변수
#define DEBOUNCE_TIME_MS    300     // 300ms 디바운스 시간
static volatile uint32 g_last_isr_tick = 0;
static volatile uint8 g_collision_flag = 0;  // 충돌 감지 플래그

// 측정값 저장 (0: 정상, 0xFF: 충돌 감지). CAN 전송은 scheduler에서 주기 수행
static uint8_t g_collision = 0;

// ISR 핸들러 프로토타입
static void COLLISION_ISR_handler(void *pArg);

// 태스크 함수 프로토타입
static void Task_Collision(void *pArg);

// =========================================================
// ISR 핸들러
// =========================================================
static void COLLISION_ISR_handler(void *pArg)
{
    (void)pArg;
    uint32 current_tick = 0;

    g_isr_count++;

    // 핀이 LOW(눌림)일 때만 처리, HIGH(뗌)일 때는 무시
    if (GPIO_Get(COLLISION_PIN) != 0)
    {
        return;
    }

    // 현재 시간 가져오기
    SAL_GetTickCount(&current_tick);

    // 채터링 방지: 마지막 인터럽트로부터 DEBOUNCE_TIME_MS 이내면 무시
    if ((current_tick - g_last_isr_tick) >= DEBOUNCE_TIME_MS)
    {
        g_last_isr_tick = current_tick;
        g_collision_flag = 1;  // 플래그만 설정
    }
}

// =========================================================
// 태스크 함수
// =========================================================
static void Task_Collision(void *pArg)
{
    (void)pArg;

    mcu_printf("[COLLISION] Task started, waiting for collision events...\n");

    for (;;)
    {
        // 충돌 감지 플래그 확인
        if (g_collision_flag != 0)
        {
            g_collision_flag = 0;  // 플래그 클리어
            SAL_CoreCriticalEnter();
            g_collision = 0xFF;    // 측정값 저장
            SAL_CoreCriticalExit();

            mcu_printf("[COLLISION] Collision detected! (ISR count: %d)\n", g_isr_count);
            
            // 충돌 발생 시 즉시 CAN 메시지 전송
            CAN_send_collision(0xFF);
            
            if (g_collision_callback != NULL)
            {
                g_collision_callback();
            }
        }

        // 10ms 폴링 주기
        SAL_TaskSleep(10);
    }
}

// =========================================================
// 초기화 함수
// =========================================================
void COLLISION_init(void)
{
    mcu_printf("[COLLISION] Initializing...\n");

    // 1. GPIO 입력 설정 (풀업, 입력버퍼 활성화)
    //    센서가 HIGH 유지 → 충격 시 LOW이므로 풀업 설정
    GPIO_Config(COLLISION_PIN, GPIO_INPUT | GPIO_FUNC(0) | GPIO_INPUTBUF_EN | GPIO_PULLUP);

    // 2. 외부 인터럽트 연결 (GIC_EXT2에 GPIO 핀 연결)
    GPIO_IntExtSet(COLLISION_IRQ_ID, COLLISION_PIN);

    // 3. GIC 핸들러 등록 (Falling edge 감지)
    GIC_IntVectSet(COLLISION_IRQ_ID, 10, GIC_INT_TYPE_EDGE_FALLING, COLLISION_ISR_handler, NULL);

    // 4. 인터럽트 활성화
    GIC_IntSrcEn(COLLISION_IRQ_ID);

    mcu_printf("[COLLISION] Init done (Pin: GPA6, IRQ: %d)\n", COLLISION_IRQ_ID);
}

// =========================================================
// 태스크 시작 함수
// =========================================================
void COLLISION_start_task(void)
{
    SALRetCode_t ret;

    COLLISION_init();

    ret = (SALRetCode_t)SAL_TaskCreate(
        &g_collision_task_id,
        (const uint8 *)"Collision Task",
        (SALTaskFunc)Task_Collision,
        &g_collision_task_stk[0],
        COLLISION_TASK_STK_SIZE,
        COLLISION_TASK_PRIO,
        NULL
    );

    if (ret != SAL_RET_SUCCESS)
    {
        mcu_printf("[COLLISION] Task create failed: %d\n", ret);
        return;
    }

    mcu_printf("[COLLISION] Task created successfully\n");
}

// =========================================================
// 콜백 등록 함수
// =========================================================
void COLLISION_register_callback(CollisionCallback_t callback)
{
    g_collision_callback = callback;
    mcu_printf("[COLLISION] Callback registered\n");
}

// g_collision 조회 (critical section 보호, scheduler용)
void COLLISION_get_data(uint8_t *pVal)
{
    if (pVal == NULL)
        return;
    SAL_CoreCriticalEnter();
    *pVal = g_collision;
    SAL_CoreCriticalExit();
}
