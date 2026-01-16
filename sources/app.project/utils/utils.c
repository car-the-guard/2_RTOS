#include "utils.h"

static void SoftwareDelay_us(uint32 us)
{
    // Cortex-R5 클럭 속도에 따라 반복 횟수 조정 필요
    // 최적화 방지를 위해 volatile 사용
    volatile uint32 count; 
    for (uint32 i = 0; i < us; i++)
    {
        // 1us를 만들기 위한 루프 (보드 속도에 따라 값 10~100 사이 조정 필요)
        for (count = 0; count < 20; count++) 
        {
            __asm__("nop"); // 아무것도 안 하는 명령어
        }
    }
}