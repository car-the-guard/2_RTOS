// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
* FileName : accel.c
* Description : ACCEL I2C Driver & Task Implementation (Final Fixed Version)
***************************************************************************************************
*/

#include <sal_api.h>
#include "accel.h"
#include "i2c.h"
#include "gpio.h"
#include "app_cfg.h"
#include <bsp.h>
#include "sal_internal.h"
#include <stdint.h>
#include <debug.h> 

/* ========================================================================= */
/* DEFINITIONS                                                               */
/* ========================================================================= */

// I2C 설정 (SCL: B0, SDA: B1 -> Port Index 0 사용)
#define MPU_I2C_CH          I2C_CH_MASTER0
#define MPU_I2C_PORT_IDX    0UL
#define MPU_I2C_SPEED       100UL   // 100kHz (안정성 최우선)

// ACCEL 레지스터 및 주소
#define ACCEL_ADDR        0x68    // 7-bit Address
#define REG_PWR_MGMT_1      0x6B    // 전원 관리 (Sleep 해제용)
#define REG_ACCEL_CONFIG    0x1C    // 가속도 범위 설정
#define REG_ACCEL_XOUT_H    0x3B    // 데이터 시작 주소
#define REG_WHO_AM_I        0x75    // ID 확인

// 가속도 감도 (기본값 +/- 2g 설정 시 16384 LSB/g)
#define ACCEL_SENSITIVITY   16384.0

/* ========================================================================= */
/* LOCAL VARIABLES                                                           */
/* ========================================================================= */

// 스택 크기 1024 (printf 및 실수 연산 보호)
static uint32 g_AccelTaskStk[1024]; 

// 최신 센서 데이터를 저장하는 공유 변수
static ACCEL_Data_t g_SensorData;

/* ========================================================================= */
/* INTERNAL FUNCTIONS (FIXED DRIVER)                                         */
/* ========================================================================= */

// [수정 1] 쓰기 함수 개선 (버퍼 합치기 + 주소 시프트)
// 호환성을 위해 [레지스터주소 + 데이터]를 하나의 배열로 묶어서 보내며,
// I2C 주소에 << 1을 적용합니다.
static SALRetCode_t ACCEL_WriteReg(uint8 reg, uint8 data)
{
    uint8 buf[2];
    buf[0] = reg;
    buf[1] = data;

    I2CXfer_t xfer;
    xfer.xCmdBuf = buf;      // [주소, 데이터]를 통째로 전송
    xfer.xCmdLen = 2;        // 길이 2바이트
    xfer.xOutBuf = NULL;
    xfer.xOutLen = 0;
    xfer.xInBuf  = NULL;
    xfer.xInLen  = 0;
    xfer.xOpt    = 0;

    // [중요] ACCEL_ADDR << 1 적용
    return I2C_Xfer(MPU_I2C_CH, (uint8)(ACCEL_ADDR << 1), xfer, 0); 
}

// [수정 2] 읽기 함수 개선 (주소 시프트)
static SALRetCode_t ACCEL_ReadRegs(uint8 startReg, uint8 *pBuf, uint8 len)
{
    I2CXfer_t xfer;
    xfer.xCmdBuf = &startReg;
    xfer.xCmdLen = 1;
    xfer.xOutBuf = NULL;
    xfer.xOutLen = 0;
    xfer.xInBuf  = pBuf;
    xfer.xInLen  = len;
    xfer.xOpt    = 0;

    // [중요] ACCEL_ADDR << 1 적용
    return I2C_Xfer(MPU_I2C_CH, (uint8)(ACCEL_ADDR << 1), xfer, 0); 
}

// 1바이트 읽기 (진단용 Wrapper)
static SALRetCode_t ACCEL_ReadOneByte(uint8 reg, uint8 *pData)
{
    return ACCEL_ReadRegs(reg, pData, 1);
}

/* ========================================================================= */
/* TASK LOOP                                                                 */
/* ========================================================================= */

static void ACCEL_Task_Loop(void *pArg)
{
    (void)pArg;
    SALRetCode_t ret;
    uint8 rawData[6]; 
    uint8 reg_val = 0;

    mcu_printf("[ACCEL] Task Loop Started...\n");

    // 1. I2C 오픈 (I2C_Init은 start_task에서 이미 호출됨)
    ret = I2C_Open(MPU_I2C_CH, MPU_I2C_PORT_IDX, MPU_I2C_SPEED, NULL, NULL);
    
    if (ret != SAL_RET_SUCCESS) {
        mcu_printf("[ACCEL] I2C Open Failed! Error: %d\n", ret);
        while(1) { SAL_TaskSleep(1000); }
    } else {
        mcu_printf("[ACCEL] I2C Open Success.\n");
    }

    // 2. 연결 확인 (WHO_AM_I)
    // 여기서 0x68이 나와야 정상 연결입니다.
    ACCEL_ReadOneByte(REG_WHO_AM_I, &reg_val);
    mcu_printf("[Diag] WHO_AM_I: 0x%02X (Expected: 0x68)\n", reg_val);

    // 3. 센서 깨우기 (가장 중요한 단계)
    mcu_printf("[Setup] Waking up sensor...\n");
    // PWR_MGMT_1(0x6B) 레지스터에 0x00을 씁니다.
    ACCEL_WriteReg(REG_PWR_MGMT_1, 0x00);
    SAL_TaskSleep(100); 

    // 4. 깨어났는지 확인 (검증)
    // 0x00이어야 합니다. 0x40이면 여전히 자고 있는 것입니다.
    ACCEL_ReadOneByte(REG_PWR_MGMT_1, &reg_val);
    mcu_printf("[Check] PWR_MGMT_1: 0x%02X (Expected: 0x00)\n", reg_val);
    
    if (reg_val & 0x40) {
        mcu_printf("[Warning] Sensor still sleeping. Retrying...\n");
        ACCEL_WriteReg(REG_PWR_MGMT_1, 0x00);
        SAL_TaskSleep(100);
    }

    // 5. 가속도 범위 설정 (+/- 2g)
    ACCEL_WriteReg(REG_ACCEL_CONFIG, 0x00); 
    SAL_TaskSleep(10);

    // 6. 데이터 수집 루프
    for (;;)
    {
        // 버퍼 초기화
        for(int i=0; i<6; i++) rawData[i] = 0;

        ret = ACCEL_ReadRegs(REG_ACCEL_XOUT_H, rawData, 6);

        if (ret == SAL_RET_SUCCESS)
        {
            // 상위/하위 바이트 결합
            int16 temp_raw_x = (int16)((rawData[0] << 8) | rawData[1]);
            int16 temp_raw_y = (int16)((rawData[2] << 8) | rawData[3]);
            int16 temp_raw_z = (int16)((rawData[4] << 8) | rawData[5]);

            // 전역 변수 업데이트
            SAL_CoreCriticalEnter();
            g_SensorData.Raw_X = temp_raw_x;
            g_SensorData.Raw_Y = temp_raw_y;
            g_SensorData.Raw_Z = temp_raw_z;

            g_SensorData.Ax = (double)temp_raw_x / ACCEL_SENSITIVITY;
            g_SensorData.Ay = (double)temp_raw_y / ACCEL_SENSITIVITY;
            g_SensorData.Az = (double)temp_raw_z / ACCEL_SENSITIVITY;
            SAL_CoreCriticalExit();

            // 실수형 출력 (정수.소수 형태로 변환)
            int z_int = (int)g_SensorData.Az;
            int z_dec = (int)((g_SensorData.Az - z_int) * 100);
            if(z_dec < 0) z_dec = -z_dec;

            // [결과 확인]
            // 정상: RAW 값이 0이 아니고 변동이 있어야 함. Az는 약 1.00 근처.
            // mcu_printf("RAW: %d %d %d || Az: %d.%02d\n", 
            //            temp_raw_x, temp_raw_y, temp_raw_z, z_int, z_dec);
        }
        else
        {
            mcu_printf("[Error] I2C Read Failed\n");
        }

        SAL_TaskSleep(200); // 0.2초마다 갱신
    }
}

/* ========================================================================= */
/* EXTERNAL FUNCTIONS (API)                                                  */
/* ========================================================================= */

// [API 1] 태스크 생성 및 시작
void ACCEL_start_task(void)
{
    static uint32_t AppTaskStartID = 0;
    SALRetCode_t ret;

    // I2C 초기화 (BSP_Init에서 이미 호출되지만, 안전을 위해 중복 호출)
    I2C_Init();

    ret = SAL_TaskCreate(
        &AppTaskStartID,
        (const uint8 *)"Task_ACCEL",     
        ACCEL_Task_Loop,  
        &g_AccelTaskStk[0],   
        1024,               // 스택 크기
        SAL_PRIO_APP_CFG,   // 다른 태스크들과 동일한 우선순위
        NULL                
    );
    
    if (ret != SAL_RET_SUCCESS)
    {
        mcu_printf("[ACCEL] Task create failed: %d\n", ret);
        return;
    }
    
    mcu_printf("[ACCEL] Task created successfully\n");
}

// [API 2] 최신 데이터 가져오기 (Getter)
void ACCEL_get_data(ACCEL_Data_t *pOutData)
{
    if (pOutData == NULL) return;

    SAL_CoreCriticalEnter(); 
    pOutData->Raw_X = g_SensorData.Raw_X;
    pOutData->Raw_Y = g_SensorData.Raw_Y;
    pOutData->Raw_Z = g_SensorData.Raw_Z;
    pOutData->Ax    = g_SensorData.Ax;
    pOutData->Ay    = g_SensorData.Ay;
    pOutData->Az    = g_SensorData.Az;
    SAL_CoreCriticalExit();
}