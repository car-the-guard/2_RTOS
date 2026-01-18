#include <bsp.h>
#include <stdint.h>
#include <stdio.h>

#include "gpio.h"
#include "gic.h"     
#include "sonar.h"
#include "utils.h"

// -----------------------------------------------------------
// [설정] 인터럽트 ID 및 트리거 타입
// -----------------------------------------------------------
// 주의: IRQ ID 54번은 예시입니다. 동작하지 않으면 회로도/매뉴얼 확인 필요.
#define SONAR_IRQ_ID        GIC_EXT1  

// 전역 변수
volatile uint32_t g_echo_start_time = 0;
volatile uint32_t g_echo_end_time = 0;
volatile uint32_t g_pulse_width = 0;
volatile uint8_t  g_capture_done = 0;

// -----------------------------------------------------------
// [에러 해결 1] SAL_GetTickCount 매크로 올바르게 사용하기
// -----------------------------------------------------------
uint32_t BSP_GetMicros(void)
{
    // SAL_GetTickCount는 매크로이며, 값을 담을 변수의 '주소'를 인자로 받습니다.
    // 반환값은 성공/실패 코드이고, 실제 시간 값은 인자로 넘긴 변수에 담깁니다.
    
    uint64_t tick_val = 0; // 64비트 혹은 32비트 (SDK 버전에 따라 다름, 넉넉히 64 잡음)
    
    // extern 선언 제거함 (헤더에 이미 매크로로 있음)
    SAL_GetTickCount(&tick_val); 

    // ms 단위를 us로 변환 (정밀도는 떨어짐)
    return (uint32_t)(tick_val * 1000); 
}

uint32_t BSP_GetMicros_Simulated(void) {
    // Topst SDK에 us 단위 타이머가 없다면, 
    // 반복문 횟수로 시간을 짐작해야 합니다. (임시 방편)
    static volatile uint32_t loop_cnt = 0;
    return loop_cnt++;
}

// ISR 함수 프로토타입
static void SONAR_ISR_handler(void *pArg);

// -----------------------------------------------------------
// 함수 구현
// -----------------------------------------------------------

void SONAR_init(void)
{
    mcu_printf("SONAR_INIT_START\n");

    // 1. Trig: 출력 (Low 초기화)
    GPIO_Config(SONAR_TRIG_PIN, GPIO_OUTPUT | GPIO_FUNC(0) | GPIO_NOPULL);
    GPIO_Set(SONAR_TRIG_PIN, 0);

    // 2. Echo: 입력 (입력버퍼 활성화, 풀업은 끔 - 센서가 5V 구동 시 풀업 충돌 방지)
    // *주의: HC-SR04는 대기 시 Low, 신호 올 때 High를 줍니다. 
    //        센서가 연결되어 있다면 PULLUP을 끄는 게(NOPULL) 더 정확할 수 있습니다.
    //        하지만 안전하게 일단 PULLUP 유지해도 동작은 합니다.
    GPIO_Config(SONAR_ECHO_PIN, GPIO_INPUT | GPIO_FUNC(0) | GPIO_INPUTBUF_EN | GPIO_PULLUP);
    
    // 3. 인터럽트 연결 [핵심 수정 부분!]
    // SONAR_IRQ_ID(GIC번호)가 아니라, 채널 번호 '1'을 넣어야 합니다.
    // GPB_01 핀은 보통 EXT_INT_1 채널에 연결됩니다.
    GPIO_IntExtSet(1, SONAR_ECHO_PIN); 

    // 4. GIC 핸들러 등록
    // 여기는 GIC 번호(SONAR_IRQ_ID)를 쓰는 것이 맞습니다.
    GIC_IntVectSet(SONAR_IRQ_ID, 10, GIC_INT_TYPE_EDGE_BOTH, SONAR_ISR_handler, (void *)SONAR_ECHO_PIN);
    
    // 5. 인터럽트 활성화
    GIC_IntSrcEn(SONAR_IRQ_ID);

    mcu_printf("SONAR_INIT_DONE (Channel: 1, IRQ: %d)\n", SONAR_IRQ_ID);
}

void SONAR_read_sensor(void)
{
    // mcu_printf("DEBUG: SEND Message\n");
    g_capture_done = 0;
    GPIO_Set(SONAR_TRIG_PIN, 1U);
    SoftwareDelay_us(10);
    GPIO_Set(SONAR_TRIG_PIN, 0U);
}

// void SONAR_start_task(void)
// {
//     SONAR_init(); // 초기화

//     for(;;)
//     {
//         SONAR_read_sensor();
        
//         // 100ms 대기 (초음파 왕복 대기)
//         SAL_TaskSleep(100);

//         // 거리 계산 및 출력
//         int32_t dist = SONAR_get_distance();
//         // if(dist >= 0) {
//             // 디버그용 로그 (필요시 주석 해제)
//             mcu_printf("Sonar Dist: %d cm\n", (int)dist);
//         // }
//     }
// }

void SONAR_start_task(void)
{
    mcu_printf("SONAR Polling Mode Start\n");

    // 1. 핀 설정 (입력 버퍼 필수!)
    GPIO_Config(SONAR_TRIG_PIN, GPIO_OUTPUT | GPIO_FUNC(0) | GPIO_NOPULL);
    GPIO_Config(SONAR_ECHO_PIN, GPIO_INPUT  | GPIO_FUNC(0) | GPIO_INPUTBUF_EN | GPIO_PULLUP);
    
    GPIO_Set(SONAR_TRIG_PIN, 0);

    for(;;)
    {
        // -----------------------------------------------------
        // 1. Trigger 발송 (시간 넉넉하게)
        // -----------------------------------------------------
        GPIO_Set(SONAR_TRIG_PIN, 1U);
        SoftwareDelay_us(500); // 10us -> 500us로 대폭 증가
        GPIO_Set(SONAR_TRIG_PIN, 0U);

        // -----------------------------------------------------
        // 2. Echo 신호 대기 (LOW -> HIGH 될 때까지 대기)
        // -----------------------------------------------------
        uint32_t wait_timeout = 0;
        while(GPIO_Get(SONAR_ECHO_PIN) == 0) {
            wait_timeout++;
            if(wait_timeout > 200000) break; // 타임아웃
        }

        if(wait_timeout <= 200000) {
            // -----------------------------------------------------
            // 3. Echo 지속 시간 측정 (HIGH -> LOW 될 때까지)
            // -----------------------------------------------------
            uint32_t pulse_len = 0;
            while(GPIO_Get(SONAR_ECHO_PIN) == 1) {
                pulse_len++;
                // 무한루프 방지
                if(pulse_len > 200000) break; 
            }
            
            // pulse_len은 '시간'이 아니라 '루프 횟수'입니다.
            // 정확한 cm는 아니지만, 거리에 비례해서 숫자가 커져야 합니다.
            // 대략적인 보정치(캘리브레이션)를 곱해서 cm로 만듭니다.
            // (보드 속도에 따라 이 나누기 값을 조절해야 함)
            uint32_t estimated_dist = pulse_len / 60; 

            mcu_printf("Dist: %d cm (Raw: %d)\n", (int)estimated_dist, (int)pulse_len);
        } 
        else {
            mcu_printf("No Echo Signal (Sensor Fault or Wiring)\n");
        }

        SAL_TaskSleep(100);
    }
}


int32_t SONAR_get_distance(void)
{
    if (g_capture_done) {
        // 거리(cm) = 시간(us) / 58
        return (int32_t)(g_pulse_width / 58);
    }
    return -1;
}

// -----------------------------------------------------------
// [에러 해결 2] GIC_IntClr 제거 (SDK가 자동 처리)
// -----------------------------------------------------------
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