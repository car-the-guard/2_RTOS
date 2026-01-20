#ifndef SONAR_H
#define SONAR_H

#include "gpio.h"  // <gpio.h> 대신 "gpio.h" 권장 (경로 설정에 따라 다름)
#include <stdint.h>

// 핀 설정
#define SONAR_TRIG_PIN    GPIO_GPB(10)
#define SONAR_ECHO_PIN    GPIO_GPB(27)

// 초기화 함수
void SONAR_init(void);

// 메인 루프에서 실행할 태스크 함수
void SONAR_start_task(void);

// 센서 트리거 발생 함수
void SONAR_read_sensor(void);

// 계산된 거리값을 가져오는 함수 (이름 변경: get_data -> get_distance)
int32_t SONAR_get_distance(void);

#endif // SONAR_H