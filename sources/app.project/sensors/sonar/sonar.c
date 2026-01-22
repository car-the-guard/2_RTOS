#include <bsp.h>
#include <stdint.h>
#include <stdio.h>
#include <sal_api.h>
#include <app_cfg.h>

#include "gpio.h"
#include "gic.h"     
#include "sonar.h"
#include "utils.h"

#define SONAR_IRQ_ID        GIC_EXT1  

// =========================================================
// 설정 및 전역 변수
// =========================================================
#define SONAR_TASK_STK_SIZE     (2048)
#define SONAR_TASK_PRIO         (SAL_PRIO_APP_CFG)

// 태스크 관련 변수
static uint32 g_sonar_task_id = 0;
static uint32 g_sonar_task_stk[SONAR_TASK_STK_SIZE];

// 전역 변수 (인터럽트 방식용)
volatile uint32_t g_echo_start_time = 0;
volatile uint32_t g_echo_end_time = 0;
volatile uint32_t g_pulse_width = 0;
volatile uint8_t  g_capture_done = 0;

// 측정값 저장 (cm). CAN 전송은 scheduler에서 주기 수행. distance1은 예비(2채널 확장용)
static uint16_t g_sonar_distance0 = 0;
static uint16_t g_sonar_distance1 = 0;


uint32_t BSP_GetMicros(void)
{
    // SAL_GetTickCount는 매크로이며, 값을 담을 변수의 '주소'를 인자로 받습니다.
    // 반환값은 성공/실패 코드이고, 실제 시간 값은 인자로 넘긴 변수에 담깁니다.
    
    uint32_t tick_val = 0;
    
    // extern 선언 제거함 (헤더에 이미 매크로로 있음)
    SAL_GetTickCount(&tick_val); 

    // ms 단위를 us로 변환 (정밀도는 떨어짐)
    return (uint32_t)(tick_val * 1000); 
}

// ISR 함수 프로토타입
static void SONAR_ISR_handler(void *pArg);

// 태스크 함수 프로토타입
static void Task_Sonar(void *pArg);

// -----------------------------------------------------------
// 함수 구현
// -----------------------------------------------------------

void SONAR_init(void)
{
    mcu_printf("SONAR_INIT_START\n");

    // 1. Trig: 출력
    GPIO_Config(SONAR_TRIG_PIN, GPIO_OUTPUT | GPIO_FUNC(0) | GPIO_NOPULL);
    GPIO_Set(SONAR_TRIG_PIN, 0);

    // 2. Echo: 입력 (입력버퍼 활성화, 풀업은 끔 - 센서가 5V 구동 시 풀업 충돌 방지)
    // *주의: HC-SR04는 대기 시 Low, 신호 올 때 High를 줍니다. 
    //        센서가 연결되어 있다면 PULLUP을 끄는 게(NOPULL) 더 정확할 수 있습니다.
    //        하지만 안전하게 일단 PULLUP 유지해도 동작은 합니다.
    GPIO_Config(SONAR_ECHO_PIN, GPIO_INPUT | GPIO_FUNC(0) | GPIO_INPUTBUF_EN | GPIO_PULLUP);
    
    // 3. 인터럽트 연결 (GIC ID와 GPIO 핀 연결)
    GPIO_IntExtSet(SONAR_IRQ_ID, SONAR_ECHO_PIN);

    // 4. GIC 핸들러 등록
    // 여기는 GIC 번호(SONAR_IRQ_ID)를 쓰는 것이 맞습니다.
    GIC_IntVectSet(SONAR_IRQ_ID, 10, GIC_INT_TYPE_EDGE_BOTH, SONAR_ISR_handler, (void *)SONAR_ECHO_PIN);
    
    // 5. 인터럽트 활성화
    GIC_IntSrcEn(SONAR_IRQ_ID);

    mcu_printf("SONAR_INIT_DONE (TRIG: GPB10, ECHO: GPB27, IRQ: %d)\n", SONAR_IRQ_ID);
}

void SONAR_read_sensor(void)
{
    // mcu_printf("DEBUG: SEND Message\n");
    g_capture_done = 0;
    GPIO_Set(SONAR_TRIG_PIN, 1U);
    SoftwareDelay_us(10);
    GPIO_Set(SONAR_TRIG_PIN, 0U);
}

// =========================================================
// 태스크 함수
// =========================================================
static void Task_Sonar(void *pArg)
{
    (void)pArg;

    mcu_printf("SONAR Polling Mode Start\n");

    // 1. 핀 설정 (입력 버퍼 필수!)
    GPIO_Config(SONAR_TRIG_PIN, GPIO_OUTPUT | GPIO_FUNC(0) | GPIO_NOPULL);
    GPIO_Config(SONAR_ECHO_PIN, GPIO_INPUT  | GPIO_FUNC(0) | GPIO_INPUTBUF_EN | GPIO_PULLUP);
    
    GPIO_Set(SONAR_TRIG_PIN, 0);

    mcu_printf("ECHO pin state: %d\n", GPIO_Get(SONAR_ECHO_PIN));

    for(;;)
    {
        // -----------------------------------------------------
        // 1. Trigger 발송 (시간 넉넉하게)
        // -----------------------------------------------------
        // mcu_printf("TRIG...\n");
        GPIO_Set(SONAR_TRIG_PIN, 1U);
        SoftwareDelay_us(500); // 10us -> 500us로 대폭 증가
        GPIO_Set(SONAR_TRIG_PIN, 0U);
        // mcu_printf("Waiting ECHO...\n");

        // -----------------------------------------------------
        // 2. Echo 신호 대기 (LOW -> HIGH 될 때까지 대기)
        // -----------------------------------------------------
        // 200MHz 클럭 기준: 1us ≈ 200 사이클
        // 루프 한 번 (GPIO_Get + 비교 + 증가 + 점프) ≈ 40-50 사이클
        // 1us ≈ 4-5 루프 (보수적으로 4로 계산)
        #define LOOPS_PER_US 4  // 200MHz 기준: 루프당 약 50 사이클 가정
        
        uint32_t wait_timeout_loops = 30000 * LOOPS_PER_US; // 30ms 타임아웃
        uint32_t wait_count = 0;
        while(GPIO_Get(SONAR_ECHO_PIN) == 0) {
            wait_count++;
            if(wait_count > wait_timeout_loops) {
                wait_count = 0; // 타임아웃 플래그
                break;
            }
        }

        if(wait_count != 0) {
            // -----------------------------------------------------
            // 3. Echo 지속 시간 측정 (HIGH -> LOW 될 때까지)
            // 200MHz 클럭 기준으로 루프 횟수를 마이크로초로 변환
            // -----------------------------------------------------
            uint32_t pulse_len = 0;
            uint32_t pulse_timeout_loops = 30000 * LOOPS_PER_US; // 30ms 타임아웃 (약 5m 거리)
            
            while(GPIO_Get(SONAR_ECHO_PIN) == 1) {
                pulse_len++;
                // 무한루프 방지
                if(pulse_len > pulse_timeout_loops) break;
            }
            
            // 루프 횟수를 마이크로초로 변환
            // 200MHz에서 1us = 200 사이클, 루프 한 번 ≈ 50 사이클
            uint32_t pulse_width_us = pulse_len / LOOPS_PER_US;
            
            // HC-SR04 거리 계산: 거리(cm) = 펄스폭(us) / 58
            // 음속 343m/s = 34300cm/s = 0.0343cm/us
            // 왕복 거리이므로: 거리 = (펄스폭 * 0.0343) / 2 = 펄스폭 / 58.3 ≈ 펄스폭 / 58
            // uint32_t distance_cm = pulse_width_us / 58;
            uint32_t distance_cm = pulse_width_us / 7.2;
            uint16_t d0 = (distance_cm > 0xFFFF) ? 0xFFFF : (uint16_t)distance_cm;
            SAL_CoreCriticalEnter();
            g_sonar_distance0 = d0;
            g_sonar_distance1 = 0;  /* 예비 */
            SAL_CoreCriticalExit();

            // mcu_printf("Dist: %d cm (Pulse: %d us, Loops: %d)\n", (int)distance_cm, (int)pulse_width_us, (int)pulse_len);
        }
        else
        {
            mcu_printf("No Echo Signal (Sensor Fault or Wiring)\n");
        }

        SAL_TaskSleep(100);
    }
}

// =========================================================
// 태스크 시작 함수
// =========================================================
void SONAR_start_task(void)
{
    SALRetCode_t ret;

    mcu_printf("[SONAR] start_task called, calling SONAR_init()...\n");
    SONAR_init();
    mcu_printf("[SONAR] SONAR_init() returned\n");

    ret = (SALRetCode_t)SAL_TaskCreate(
        &g_sonar_task_id,
        (const uint8 *)"Sonar Task",
        (SALTaskFunc)Task_Sonar,
        &g_sonar_task_stk[0],
        SONAR_TASK_STK_SIZE,
        SONAR_TASK_PRIO,
        NULL
    );

    if (ret != SAL_RET_SUCCESS)
    {
        mcu_printf("[SONAR] Task create failed: %d\n", ret);
        return;
    }

    mcu_printf("[SONAR] Task created successfully\n");
}


int32_t SONAR_get_distance(void)
{
    if (g_capture_done) {
        // 거리(cm) = 시간(us) / 58
        return (int32_t)(g_pulse_width / 58);
    }
    return -1;
}

void SONAR_get_data(uint16_t *pDistance0, uint16_t *pDistance1)
{
    if (pDistance0 == NULL || pDistance1 == NULL)
        return;
    SAL_CoreCriticalEnter();
    *pDistance0 = g_sonar_distance0;
    *pDistance1 = g_sonar_distance1;
    SAL_CoreCriticalExit();
}

// 우선 Polling 방식으로 구현되어있음.
// 이후에 시간이 되면 이거 인터럽트 방식으로 전환하기
static void SONAR_ISR_handler(void *pArg)
{
    uint32_t pin = (uint32_t)pArg;
    
    // [수정] GIC_IntClr 함수가 SDK 헤더에 없으므로 제거했습니다.
    // 보통 GIC 핸들러가 ISR 호출 전후로 Ack/EOI 처리를 자동으로 수행합니다.
    // 만약 인터럽트가 한 번만 발생하고 멈춘다면, GIC_IntSGI가 아닌
    // 레지스터 직접 제어가 필요할 수 있습니다. (일단 주석 처리)
    // GIC_IntClr(SONAR_IRQ_ID); 
    mcu_printf("ISR IN\n");
    if(GPIO_Get(pin) == 1) { // Rising
        g_echo_start_time = BSP_GetMicros();
        g_capture_done = 0;
    }
    else { // Falling
        g_echo_end_time = BSP_GetMicros();
        
        // 오버플로우 방어 로직
        if (g_echo_end_time >= g_echo_start_time) {
            g_pulse_width = g_echo_end_time - g_echo_start_time;
        } else {
            g_pulse_width = (0xFFFFFFFF - g_echo_start_time) + g_echo_end_time;
        }
        g_capture_done = 1;
    }
}