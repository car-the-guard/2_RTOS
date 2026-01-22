#ifndef COMPASS_H
#define COMPASS_H

#include <sal_api.h>
#include <stdint.h>

// 태스크 시작 함수
void COMPASS_start_task(void);

// 현재 heading 값 조회 함수
void COMPASS_get_heading(uint16_t *pHeading);

#endif // COMPASS_H
