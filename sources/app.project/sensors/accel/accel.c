/*
 * mpu6050_task.c
 */

 #include "accel.h"
 #include <sal_api.h>
 #include <app_cfg.h>
 #include <debug.h>
 #include <bsp.h>
 #include <stdint.h>

 #include "i2c.h"
 #include "gpio.h"

 #define I2C_CH_ACCEL        I2C_CH_MASTER0
 #define I2C_PORT_SEL        1   // 핀 mux 설정 (보통 0번, 안 되면 1, 2 변경 시도)
 #define I2C_SPEED_HZ        400 // 400kHz /* [TODO] I2C 드라이버 헤더 추가 (예: tcc_i2c.h) */

 // #include "tcc_i2c.h"
 
 // =========================================================
 // 설정 및 전역 변수
 // =========================================================
 #define MPU_TASK_STK_SIZE   (4096)       // 스택 사이즈 (상황에 맞춰 조정)
 #define MPU_TASK_PRIO       (SAL_PRIO_APP_CFG) // 우선순위 (Main과 동일하게 설정 예시)
 #define MPU6050_ADDR        0x68
 
 // 태스크 관리를 위한 정적 변수 (SAL 필수)
 static uint32 MpuTaskID = 0;
 static uint32 MpuTaskStk[MPU_TASK_STK_SIZE];
 
 // 데이터 공유용 구조체 및 뮤텍스
 static MPU6050_Data_t g_mpu_data = {0};
 // static SALMutexId_t g_mpu_mutex; // (만약 SAL에 뮤텍스가 있다면 사용)
 
 // =========================================================
// I2C 래퍼 함수 구현 (SDK: i2c.h 사용)
// =========================================================

/* I2C 쓰기: [레지스터 주소] -> [데이터] */
static int8_t VCPG_I2C_Write(uint8_t slave_addr, uint8_t reg, uint8_t data)
{
    SALRetCode_t ret;
    I2CXfer_t xfer = {0};
    
    // SDK 구조체 설정
    uint8_t cmd_buf[1] = {reg};
    uint8_t out_buf[1] = {data};

    xfer.xCmdBuf = cmd_buf; // 레지스터 주소
    xfer.xCmdLen = 1;
    xfer.xOutBuf = out_buf; // 쓸 데이터
    xfer.xOutLen = 1;
    xfer.xOpt    = 0;       // 옵션 없음

    // I2C_XferCmd(채널, 주소, 구조체, 비동기여부)
    // ucAsync = 0 (Sync 모드)으로 호출하여 완료될 때까지 대기
    ret = I2C_XferCmd(I2C_CH_ACCEL, slave_addr, xfer, 0);

    if (ret != SAL_RET_SUCCESS) {
        mcu_printf("[I2C Write Error] Ret: %d\n\r", ret);
        return -1;
    }
    return 0;
}

/* I2C 읽기: [레지스터 주소] -> (Restart) -> [데이터 수신] */
static int8_t VCPG_I2C_Read(uint8_t slave_addr, uint8_t reg, uint8_t *pBuf, uint16_t len)
{
    SALRetCode_t ret;
    I2CXfer_t xfer = {0};
    
    uint8_t cmd_buf[1] = {reg};

    xfer.xCmdBuf = cmd_buf; // 레지스터 주소
    xfer.xCmdLen = 1;
    xfer.xInBuf  = pBuf;    // 읽은 데이터 저장할 곳
    xfer.xInLen  = len;     // 읽을 길이
    xfer.xOpt    = 0;

    // ucAsync = 0 (Sync 모드)
    ret = I2C_XferCmd(I2C_CH_ACCEL, slave_addr, xfer, 0);

    if (ret != SAL_RET_SUCCESS) {
        // 읽기 실패 시 로그 출력
        // mcu_printf("[I2C Read Error] Ret: %d\n\r", ret);
        return -1;
    }
    return 0;
}
 
 // =========================================================
 // 태스크 본체 (Task Function)
 // =========================================================
 static void Task_MPU6050(void * pArg)
 {
     (void)pArg;
     uint8_t buffer[6];
     uint8_t chip_id = 0;

     SALRetCode_t i2c_ret;

     int found_ch = -1;

    SAL_TaskSleep(500); 
    mcu_printf("\n\r=== I2C Channel Scanner Start ===\n\r");

    // 채널 0번부터 2번까지 돌면서 MPU6050(0x68)을 찾아봅니다.
    for (int ch = 0; ch <= 2; ch++) 
    {
        // 1. 채널 열기 (포트 0)
        SALRetCode_t ret = I2C_Open(ch, 0, 400, NULL, NULL); 
        
        if (ret == 0) // 열기 성공!
        {
            mcu_printf("Checking CH %d... ", ch);
            
            // 2. 0x68 주소로 '노크' 해보기 (SDK 스캔 함수)
            uint32_t detected = I2C_ScanSlave(ch);
            
            if (detected == 0x68) {
                mcu_printf("FOUND! (Sensor is here)\n\r");
                found_ch = ch; // 찾았다!
                break;         // 더 찾을 필요 없음
            } else {
                mcu_printf("Empty (Result: 0x%X)\n\r", detected);
                I2C_Close(ch); // 아니면 닫고 다음 채널로
            }
        }
        else {
            mcu_printf("CH %d Open Fail\n\r", ch);
        }
    }

    if (found_ch == -1) {
        mcu_printf("ERROR: Sensor NOT found. Check Wiring (SDA/SCL)!\n\r");
        while(1) SAL_TaskSleep(1000); 
    }

    mcu_printf("=== Success! Using CH %d ===\n\r", found_ch);

     // =========================================================
    // [추가] I2C 채널 열기 (속도 400kHz)
    // =========================================================
    // 파라미터: 채널, 포트선택(0), 속도(kHz), 콜백(NULL), 인자(NULL)
    i2c_ret = I2C_Open(I2C_CH_ACCEL, I2C_PORT_SEL, I2C_SPEED_HZ, NULL, NULL);
    
    if (i2c_ret != SAL_RET_SUCCESS) {
        mcu_printf("I2C Open Failed! Error: %d\n\r", i2c_ret);
        // 실패하면 더 이상 진행하지 않고 무한 대기 (또는 리턴)
        while(1) SAL_TaskSleep(1000);
    }
    mcu_printf("I2C Open Success\n\r");
    
     // 1. 초기화 대기 
    SAL_TaskSleep(100);

     // 2. WHO_AM_I 확인
     VCPG_I2C_Read(MPU6050_ADDR, 0x75, &chip_id, 1);

     // 3. Wake Up
     VCPG_I2C_Write(MPU6050_ADDR, 0x6B, 0x00);

     

     for(;;)
     {
         // 데이터 읽기
         if (VCPG_I2C_Read(MPU6050_ADDR, 0x3B, buffer, 6) == 0)
         {
             int16_t raw_x = (int16_t)(buffer[0] << 8 | buffer[1]);
             int16_t raw_y = (int16_t)(buffer[2] << 8 | buffer[3]);
             int16_t raw_z = (int16_t)(buffer[4] << 8 | buffer[5]);
 
             // 전역 데이터 업데이트 (Critical Section 필요 시 SAL_EnterCritical 등 사용)
             g_mpu_data.Raw_X = raw_x;
             g_mpu_data.Raw_Y = raw_y;
             g_mpu_data.Raw_Z = raw_z;
             g_mpu_data.Ax = raw_x / 16384.0;
             g_mpu_data.Ay = raw_y / 16384.0;
             g_mpu_data.Az = raw_z / 16384.0;
 
             // 디버그 출력 (필요시 주석 해제)
             mcu_printf("Ax: %d, Ay: %d, Az: %d\n", (int32)raw_x, (int32)raw_y, (int32)raw_z);
         }

         // 100ms 지연 (10Hz)
         SAL_TaskSleep(100); 

     }
 }
 
 // =========================================================
 // 외부 공개 함수 (시작 함수)
 // =========================================================
 void MPU6050_start_task(void)
 {
     SALRetCode_t err;
     SAL_TaskSleep(500);
     err = (SALRetCode_t)SAL_TaskCreate(
         &MpuTaskID,                  // Task ID 저장 변수
         (const uint8 *)"MPU Task",   // Task 이름
         (SALTaskFunc)Task_MPU6050,   // Task 함수 포인터
         &MpuTaskStk[0],              // 스택 배열 시작 주소
         MPU_TASK_STK_SIZE,           // 스택 사이즈
         MPU_TASK_PRIO,               // 우선순위
         NULL                         // 파라미터
     );
     
     if (err != SAL_RET_SUCCESS) {
         mcu_printf("MPU Task Create Failed: %d\n\r", err);
     }
     mcu_printf("MPU Task Create Success: %d\n\r", err);
 }
 
 void MPU6050_get_data(MPU6050_Data_t *pOutData)
 {
     // 필요 시 Mutex 보호 추가
     if(pOutData != NULL) {
         *pOutData = g_mpu_data;
     }
 }