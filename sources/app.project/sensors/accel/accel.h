#ifndef ACCEL_H
#define ACCEL_H

#include <sal_api.h> // 기본 타입(uint32, int16 등) 사용을 위해

typedef struct {
    // 가속도 데이터
    int16 Raw_X;  // int16_t 대신 SDK 정의 타입 사용 (보통 int16)
    int16 Raw_Y;
    int16 Raw_Z;
    double Ax;
    double Ay;
    double Az;
    // 자이로 데이터
    int16 Raw_Gx;
    int16 Raw_Gy;
    int16 Raw_Gz;
    double Gx;
    double Gy;
    double Gz;
} ACCEL_Data_t;

// 태스크 시작 함수
void ACCEL_start_task(void);

// 데이터 조회 함수
void ACCEL_get_data(ACCEL_Data_t *pOutData);
#endif // ACCEL_H