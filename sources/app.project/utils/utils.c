#include "utils.h"

void SoftwareDelay_us(uint32_t us)
{
    // Cortex-R5 클럭 속도에 따라 반복 횟수 조정 필요
    volatile uint32_t count; 
    for (uint32_t i = 0; i < us; i++)
    {
        // 1us를 만들기 위한 루프 (보드 속도에 따라 값 10~100 사이 조정 필요)
        // 현재 CPU는 약 200MHz로 처리한다고 가정
        // (nop + count 증가 + count 비교 + 점프 1~2) 반복마다 4~5 사이클
        for (count = 0; count < 20; count++) 
        {
            __asm__("nop");
        }
    }
}