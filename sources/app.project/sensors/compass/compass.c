/*
 * compass.c - MPU-9250 / AK8963 지자기 센서 드라이버
 *
 * MPU-9250 I2C bypass 모드로 AK8963에 직접 접근하여 heading 계산
 */

#include <sal_api.h>
#include "compass.h"
#include "i2c.h"
#include "app_cfg.h"
#include <bsp.h>
#include <stdint.h>
#include <debug.h>
#include <math.h>

/* ========================================================================= */
/* DEFINITIONS                                                               */
/* ========================================================================= */

#define COMPASS_TASK_STK_SIZE     (1024)
#define COMPASS_TASK_PRIO         (SAL_PRIO_APP_CFG)
#define COMPASS_UPDATE_INTERVAL_MS 100

// I2C 설정 (accel과 동일: SCL B0, SDA B1)
#define MPU_I2C_CH          I2C_CH_MASTER0
#define MPU_I2C_PORT_IDX    0UL
#define MPU_I2C_SPEED       100UL

// MPU-9250
#define MPU_ADDR            0x68
#define MPU_REG_WHO_AM_I    0x75   /* 0x71 반환 */
#define MPU_REG_PWR_MGMT_1  0x6B
#define MPU_REG_USER_CTRL   0x6A
#define MPU_REG_INT_PIN_CFG 0x37

// AK8963 (bypass 후 0x0C로 직접 접근)
#define AK8963_ADDR         0x0C
#define AK8963_REG_WHO_AM_I 0x00   /* 0x48 반환 */
#define AK8963_REG_CNTL1    0x0A   /* 0x16: 16bit, 100Hz 연속 */
#define AK8963_REG_ST1      0x02   /* Data ready */
#define AK8963_REG_HXL      0x03   /* HXL~HZH 6바이트 */
#define AK8963_REG_ST2      0x09   /* ST2 읽어야 다음 측정 가능 (필수) */

/* 하드 아이론 오프셋: 한 바퀴 회전 시 min/max의 중심값. 보정 시 (mx_max+mx_min)/2, (my_max+my_min)/2 */
#define COMPASS_OFFSET_X    130
#define COMPASS_OFFSET_Y    260

/* 북(North) 보정: 센서값이 이 각도일 때 실제 나침판 0도(북). 예: 219일 때 북이면 219 */
#define COMPASS_NORTH_OFFSET_DEG  219

/* ========================================================================= */
/* LOCAL VARIABLES                                                           */
/* ========================================================================= */

static uint32 g_compass_task_stk[COMPASS_TASK_STK_SIZE];
static uint32 g_compass_task_id = 0;

/* heading 0~360 [deg] */
static uint16_t g_heading = 0;

/* ========================================================================= */
/* I2C HELPERS                                                               */
/* ========================================================================= */

static SALRetCode_t MPU_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    I2CXfer_t xfer;
    xfer.xCmdBuf = buf;
    xfer.xCmdLen = 2;
    xfer.xOutBuf = NULL;
    xfer.xOutLen = 0;
    xfer.xInBuf  = NULL;
    xfer.xInLen  = 0;
    xfer.xOpt    = 0;
    return I2C_Xfer(MPU_I2C_CH, (uint8_t)(MPU_ADDR << 1), xfer, 0);
}

static SALRetCode_t MPU_ReadRegs(uint8_t startReg, uint8_t *pBuf, uint8_t len)
{
    I2CXfer_t xfer;
    xfer.xCmdBuf = &startReg;
    xfer.xCmdLen = 1;
    xfer.xOutBuf = NULL;
    xfer.xOutLen = 0;
    xfer.xInBuf  = pBuf;
    xfer.xInLen  = len;
    xfer.xOpt    = 0;
    return I2C_Xfer(MPU_I2C_CH, (uint8_t)(MPU_ADDR << 1), xfer, 0);
}

static SALRetCode_t AK8963_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    I2CXfer_t xfer;
    xfer.xCmdBuf = buf;
    xfer.xCmdLen = 2;
    xfer.xOutBuf = NULL;
    xfer.xOutLen = 0;
    xfer.xInBuf  = NULL;
    xfer.xInLen  = 0;
    xfer.xOpt    = 0;
    return I2C_Xfer(MPU_I2C_CH, (uint8_t)(AK8963_ADDR << 1), xfer, 0);
}

static SALRetCode_t AK8963_ReadRegs(uint8_t startReg, uint8_t *pBuf, uint8_t len)
{
    I2CXfer_t xfer;
    xfer.xCmdBuf = &startReg;
    xfer.xCmdLen = 1;
    xfer.xOutBuf = NULL;
    xfer.xOutLen = 0;
    xfer.xInBuf  = pBuf;
    xfer.xInLen  = len;
    xfer.xOpt    = 0;
    return I2C_Xfer(MPU_I2C_CH, (uint8_t)(AK8963_ADDR << 1), xfer, 0);
}

/* ========================================================================= */
/* TASK LOOP                                                                 */
/* ========================================================================= */

static void COMPASS_Task_Loop(void *pArg)
{
    (void)pArg;
    SALRetCode_t ret;
    uint8_t reg_val = 0;
    uint8_t raw[8];  /* ST1 + HXL~HZH 6바이트 + ST2 (ST2 읽어야 다음 측정 가능) */
    int16_t mx, my, mz;
    float heading_deg;

    mcu_printf("[COMPASS] Task Loop Started (MPU-9250 / AK8963)\n");

    ret = I2C_Open(MPU_I2C_CH, MPU_I2C_PORT_IDX, MPU_I2C_SPEED, NULL, NULL);
    if (ret != SAL_RET_SUCCESS) {
        mcu_printf("[COMPASS] I2C Open Failed: %d\n", (int)ret);
        for (;;) { SAL_TaskSleep(1000); }
    }
    mcu_printf("[COMPASS] I2C Open Success\n");

    /* MPU-9250 WHO_AM_I */
    ret = MPU_ReadRegs(MPU_REG_WHO_AM_I, &reg_val, 1);
    if (ret != SAL_RET_SUCCESS || reg_val != 0x71) {
        mcu_printf("[COMPASS] MPU-9250 WHO_AM_I: 0x%02X (Expected: 0x71)\n", reg_val);
    } else {
        mcu_printf("[COMPASS] MPU-9250 OK (WHO_AM_I=0x71)\n");
    }

    /* Wake MPU-9250 */
    MPU_WriteReg(MPU_REG_PWR_MGMT_1, 0x00);
    SAL_TaskSleep(100);

    /* USER_CTRL: I2C_MST_EN = 0 */
    ret = MPU_ReadRegs(MPU_REG_USER_CTRL, &reg_val, 1);
    if (ret == SAL_RET_SUCCESS) {
        reg_val &= (uint8_t)~0x20;  /* I2C_MST_EN clear */
        MPU_WriteReg(MPU_REG_USER_CTRL, reg_val);
        SAL_TaskSleep(10);
    }

    /* INT_PIN_CFG: BYPASS_EN = 1 (0x02) */
    MPU_WriteReg(MPU_REG_INT_PIN_CFG, 0x02);
    SAL_TaskSleep(10);

    /* AK8963 WHO_AM_I */
    ret = AK8963_ReadRegs(AK8963_REG_WHO_AM_I, &reg_val, 1);
    if (ret != SAL_RET_SUCCESS || reg_val != 0x48) {
        mcu_printf("[COMPASS] AK8963 WHO_AM_I: 0x%02X (Expected: 0x48)\n", reg_val);
    } else {
        mcu_printf("[COMPASS] AK8963 OK (WHO_AM_I=0x48)\n");
    }

    /* AK8963 CNTL1: 16bit, 100Hz 연속 측정 */
    AK8963_WriteReg(AK8963_REG_CNTL1, 0x16);
    SAL_TaskSleep(10);

    mcu_printf("[COMPASS] Init done. Reading magnetometer...\n");

    for (;;)
    {
        /* ST1~ST2 읽기 (ST2까지 읽어야 다음 측정 가능) */
        ret = AK8963_ReadRegs(AK8963_REG_ST1, raw, 8);
        if (ret != SAL_RET_SUCCESS) {
            SAL_TaskSleep(COMPASS_UPDATE_INTERVAL_MS);
            continue;
        }

        if ((raw[0] & 0x01) == 0) {
            /* Data not ready */
            SAL_TaskSleep(10);
            continue;
        }

        /* HXL~HZH: Little Endian (HXL=LSB) */
        mx = (int16_t)((raw[2] << 8) | raw[1]);
        my = (int16_t)((raw[4] << 8) | raw[3]);
        mz = (int16_t)((raw[6] << 8) | raw[5]);

        /* 하드 아이론 보정 (오프셋 제거) */
        float mx_cal = (float)mx - (float)COMPASS_OFFSET_X;
        float my_cal = (float)my - (float)COMPASS_OFFSET_Y;

        /* heading: atan2(-my_cal, mx_cal) * 180/pi, 0~360 */
        heading_deg = (float)atan2((double)(-my_cal), (double)mx_cal) * (180.0f / 3.14159265f);
        if (heading_deg < 0.0f)
            heading_deg += 360.0f;

        /* 북(North) 보정: 센서 219도 = 실제 북(0도) */
        heading_deg -= (float)COMPASS_NORTH_OFFSET_DEG;
        if (heading_deg < 0.0f)
            heading_deg += 360.0f;

        g_heading = (uint16_t)(heading_deg + 0.5f);
        if (g_heading >= 360U)
            g_heading = 0U;

        /* 디버그 */
        {
            int heading_val = (int)g_heading;
            // mcu_printf("[COMPASS] mx=%d my=%d mz=%d heading=%d deg\n",
            //            (int)mx, (int)my, (int)mz, heading_val);
        }

        SAL_TaskSleep(COMPASS_UPDATE_INTERVAL_MS);
    }
}

/* ========================================================================= */
/* EXTERNAL FUNCTIONS                                                        */
/* ========================================================================= */

void COMPASS_start_task(void)
{
    SALRetCode_t ret;

    I2C_Init();

    ret = (SALRetCode_t)SAL_TaskCreate(
        &g_compass_task_id,
        (const uint8_t *)"Task_COMPASS",
        (SALTaskFunc)COMPASS_Task_Loop,
        &g_compass_task_stk[0],
        COMPASS_TASK_STK_SIZE,
        COMPASS_TASK_PRIO,
        NULL
    );

    if (ret != SAL_RET_SUCCESS) {
        mcu_printf("[COMPASS] Task create failed: %d\n", (int)ret);
        return;
    }
    mcu_printf("[COMPASS] Task created\n");
}

void COMPASS_get_heading(uint16_t *pHeading)
{
    if (pHeading == NULL) return;

    SAL_CoreCriticalEnter();
    *pHeading = g_heading;
    SAL_CoreCriticalExit();
}
