#include <main.h>

#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>
#include <bsp.h>
#include <gpio.h>

#include <stdio.h>
#include "app_cfg.h"
#include "timestamp.h"
#include "sonar.h"
#include "collision.h"
#include "matrix_led.h"
#include "safety_belt.h"
#include "scheduler.h"
#include "can_app.h"
#include "compass.h"

static void Main_StartTask(void * pArg);

void cmain (void)
{
    static uint32           AppTaskStartID = 0;
    static uint32           AppTaskStartStk[ACFG_TASK_MEDIUM_STK_SIZE];
    SALRetCode_t            err;

    (void)SAL_Init();

    BSP_PreInit(); /* Initialize basic BSP functions */

#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )
    (void)CAN_DemoInitialize();
#endif  // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

    BSP_Init(); /* Initialize BSP functions */

    
    // create the first app task...
    err = (SALRetCode_t)SAL_TaskCreate(&AppTaskStartID,
                         (const uint8 *)"App Task Start",
                         (SALTaskFunc) &Main_StartTask,
                         &AppTaskStartStk[0],
                         ACFG_TASK_MEDIUM_STK_SIZE,
                         SAL_PRIO_APP_CFG,
                         NULL);
    if (err == SAL_RET_SUCCESS)
    {
        // start woring os.... never return from this function
        (void)SAL_OsStart();
    }
}

static void Main_StartTask(void * pArg)
{
    (void)pArg;
    (void)SAL_OsInitFuncs();

    mcu_printf("[Main] Main_StartTask started\n");

    // 시간 동기화 타임스탬프 Task 시작 (다른 Task들보다 먼저 시작)
    TIMESTAMP_start_task();

    CAN_start_task();

    COLLISION_start_task();

    SONAR_start_task();

    COMPASS_start_task();
    
    MATRIXLED_start_task();

    SAFETYBELT_start_task();

    SCHEDULER_start_task();

    mcu_printf("[Main] Entering main loop\n");
    for(;;)
    {
        SAL_TaskSleep(1000);
    }
}