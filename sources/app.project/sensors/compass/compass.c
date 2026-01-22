/*
 * compass.c - 나침반 센서 시뮬레이션 드라이버
 *
 * 0도를 기준으로 -10도 ~ 10도 범위 내에서 천천히 흔들리는 값을 생성
 * 변화 속도: 1초당 최대 0.5도
 */

#include <sal_api.h>
#include <app_cfg.h>
#include <bsp.h>
#include <stdint.h>
#include <debug.h>
#include "compass.h"

/* ========================================================================= */
/* DEFINITIONS                                                               */
/* ========================================================================= */

#define COMPASS_TASK_STK_SIZE     (1024)
#define COMPASS_TASK_PRIO         (SAL_PRIO_APP_CFG)

// 나침반 설정
#define COMPASS_BASE_HEADING       0      // 기준 각도 (0도)
#define COMPASS_MIN_OFFSET        -10     // 최소 오프셋 (-10도)
#define COMPASS_MAX_OFFSET         10     // 최대 오프셋 (+10도)
#define COMPASS_UPDATE_INTERVAL_MS 100    // 업데이트 주기 (100ms)
// 각도 변화: 매 업데이트마다 -1도, 0도, +1도 중 하나 (1도 단위)

/* ========================================================================= */
/* LOCAL VARIABLES                                                           */
/* ========================================================================= */

// 태스크 스택
static uint32 g_compass_task_stk[COMPASS_TASK_STK_SIZE];
static uint32 g_compass_task_id = 0;

// 현재 heading 값 (0도 기준 오프셋, -10 ~ +10도 범위, 1도 단위)
static int16_t g_current_offset = 0;

// 간단한 랜덤 시드 (시간 기반)
static uint32 g_random_seed = 0;

/* ========================================================================= */
/* INTERNAL FUNCTIONS                                                        */
/* ========================================================================= */

// 간단한 랜덤 생성기 (선형 합동 생성기)
static uint32 simple_random(void)
{
    g_random_seed = (g_random_seed * 19990615 + 12345) & 0x7FFFFFFF;
    return g_random_seed;
}

// random_float와 clamp 함수는 더 이상 사용하지 않음 (1도 단위로 변경)

/* ========================================================================= */
/* TASK LOOP                                                                 */
/* ========================================================================= */

static void COMPASS_Task_Loop(void *pArg)
{
    (void)pArg;
    uint32 current_tick = 0;
    
    mcu_printf("[COMPASS] Task Loop Started...\n");
    
    // 랜덤 시드 초기화 (현재 시간 기반)
    SAL_GetTickCount(&current_tick);
    g_random_seed = current_tick;
    
    // 초기 heading 값 설정
    g_current_offset = 0;
    
    mcu_printf("[COMPASS] Initialized (Range: -10 ~ +10 degrees, Change: 1 deg per step)\n");
    
    for (;;)
    {
        // 무작위 변화량 생성 (-1, 0, +1 중 하나)
        // 랜덤 값에 따라 -1도, 0도, +1도 중 하나 선택
        int32_t random_val = (int32_t)simple_random();
        int16_t change = 0;
        
        // 랜덤 값에 따라 -1, 0, +1 중 하나 선택 (각각 33% 확률)
        int32_t mod = random_val % 3;
        if (mod == 0)
            change = -1;  // -1도
        else if (mod == 1)
            change = 0;   // 변화 없음
        else
            change = 1;   // +1도
        
        SAL_CoreCriticalEnter();
        g_current_offset += change;
        // 범위 제한 (-10 ~ +10도)
        if (g_current_offset < COMPASS_MIN_OFFSET)
            g_current_offset = COMPASS_MIN_OFFSET;
        else if (g_current_offset > COMPASS_MAX_OFFSET)
            g_current_offset = COMPASS_MAX_OFFSET;
        SAL_CoreCriticalExit();
        /* 측정값은 g_current_offset에만 저장. CAN 전송은 scheduler에서 주기 수행 */
        
        // 디버그 출력 (선택적)
        // mcu_printf("[COMPASS] Heading: %d deg\n", (int)g_current_offset);
        
        // 업데이트 주기 대기
        SAL_TaskSleep(COMPASS_UPDATE_INTERVAL_MS);
    }
}

/* ========================================================================= */
/* EXTERNAL FUNCTIONS (API)                                                  */
/* ========================================================================= */

// 태스크 생성 및 시작
void COMPASS_start_task(void)
{
    SALRetCode_t ret;
    
    ret = (SALRetCode_t)SAL_TaskCreate(
        &g_compass_task_id,
        (const uint8 *)"Task_COMPASS",
        (SALTaskFunc)COMPASS_Task_Loop,
        &g_compass_task_stk[0],
        COMPASS_TASK_STK_SIZE,
        COMPASS_TASK_PRIO,
        NULL
    );
    
    if (ret != SAL_RET_SUCCESS)
    {
        mcu_printf("[COMPASS] Task create failed: %d\n", ret);
        return;
    }
    
    mcu_printf("[COMPASS] Task created successfully\n");
}

// 현재 heading 값 조회
void COMPASS_get_heading(uint16_t *pHeading)
{
    if (pHeading == NULL) return;
    
    int16_t offset;
    
    // 크리티컬 섹션으로 값 읽기
    SAL_CoreCriticalEnter();
    offset = g_current_offset;
    SAL_CoreCriticalExit();
    
    // heading 값 계산 (0도 기준 오프셋을 0~360도 범위로 변환)
    if (offset < 0)
    {
        *pHeading = (uint16_t)(360 + offset);
    }
    else
    {
        *pHeading = (uint16_t)offset;
    }
}
