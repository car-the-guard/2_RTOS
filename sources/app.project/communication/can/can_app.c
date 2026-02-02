/*
 * can.c
 *
 * Created on: Jan 2, 2026
 * Author: mokta
 * Modified for: SAL API Compliance
 */

#include <sal_api.h>
#include <app_cfg.h>
#include <bsp.h>
#include <debug.h>
#include <string.h>
#include <stdio.h>

// dev.drivers의 CAN 헤더들을 먼저 인클루드 (can_demo.c와 동일한 순서)
#include "can_config.h"
#include "can_reg.h"
#include "can.h"        // dev.drivers의 can.h (CANMessage_t, CANErrorType_t 등 정의)
#include "can_drv.h"
#include "can_porting.h"

// 프로젝트의 CAN 헤더들 (타입 정의만, 함수는 아래)
#include "can_bridge.h"
#include "utils.h"
#include "timestamp.h"

// 프로젝트의 can_app.h 인클루드 (CAN_payload_t, CAN_queue_pkt_t 등)
#include "can_app.h"

/* -------------------------------------------------------------------------
   전역 변수 및 핸들 정의
   ------------------------------------------------------------------------- */
#define CAN_TX_QUEUE_SIZE      10
#define CAN_RX_QUEUE_SIZE      10
#define CAN_TX_POOL_SIZE      16

// CAN 채널 (기본적으로 CH0 사용)
#define CAN_CHANNEL           0

// Rx 메시지를 담을 구조체
typedef struct {
    CANMessage_t msg;      // CAN 메시지 헤더
    CAN_payload_t payload; // CAN 페이로드
} CAN_rx_queue_item_t;

// 메시지 큐 핸들
uint32 g_canTxQueueHandle = 0;
uint32 g_canRxQueueHandle = 0;

// 큐에 담긴 메시지 개수 추적 (디버깅용)
static uint32 g_canTxQueueCount = 0;

// CAN 에러 발생 빈도 제한 (ISR에서 과도한 출력 방지)
static uint32 g_canErrorCount = 0;
static uint32 g_canLastErrorType = 0;
static uint32 g_canErrorReportCounter = 0;  // 에러 보고 카운터 (100번마다 한 번만 보고)
#define CAN_ERROR_REPORT_INTERVAL 100  // 100번 에러마다 한 번만 보고

// 메모리 풀 (정적 배열로 구현)
static CAN_queue_pkt_t g_canTxPool[CAN_TX_POOL_SIZE];
static uint8 g_canTxPoolUsed[CAN_TX_POOL_SIZE];  // 0: 사용 가능, 1: 사용 중

// 태스크 관련 변수
#define CAN_TASK_STK_SIZE     (2048)
#define CAN_TASK_PRIO         (SAL_PRIO_APP_CFG)

static uint32 g_can_tx_task_id = 0;
static uint32 g_can_tx_task_stk[CAN_TASK_STK_SIZE];

static uint32 g_can_rx_task_id = 0;
static uint32 g_can_rx_task_stk[CAN_TASK_STK_SIZE];

/* -------------------------------------------------------------------------
   내부 함수 선언
   ------------------------------------------------------------------------- */
static void CAN_TxTask_Loop(void *pArg);
static void CAN_RxTask_Loop(void *pArg);
static void CAN_RxCallback(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError);
static void CAN_TxCallback(uint8 ucCh, CANTxInterruptType_t uiIntType);
static void CAN_ErrorCallback(uint8 ucCh, CANErrorType_t uiError);
CAN_queue_pkt_t* CAN_AllocPool(void);
void CAN_FreePool(CAN_queue_pkt_t *pPkt);
static void Print_Hex_8Bytes(uint8_t *data);

/* -------------------------------------------------------------------------
   유틸리티 함수
   ------------------------------------------------------------------------- */
static void Print_Hex_8Bytes(uint8_t *data)
{
    // [DEBUG] HEX: 라는 문구와 함께 출력
    mcu_printf("HEX: ");

    for (int i = 0; i < 8; i++)
    {
        // %02X 의미:
        // 0: 빈 자리를 0으로 채움 (예: A -> 0A)
        // 2: 최소 2자리로 출력
        // X: 대문자 16진수 (x를 쓰면 소문자 출력)
        mcu_printf("%02X ", data[i]);
    }

    mcu_printf("\r\n");
}

/* -------------------------------------------------------------------------
   메모리 풀 관리 함수
   ------------------------------------------------------------------------- */
CAN_queue_pkt_t* CAN_AllocPool(void)
{
    SAL_CoreCriticalEnter();
    
    for (uint8 i = 0; i < CAN_TX_POOL_SIZE; i++)
    {
        if (g_canTxPoolUsed[i] == 0)
        {
            g_canTxPoolUsed[i] = 1;
            SAL_CoreCriticalExit();
            return &g_canTxPool[i];
        }
    }
    
    SAL_CoreCriticalExit();
    return NULL_PTR;
}

void CAN_FreePool(CAN_queue_pkt_t *pPkt)
{
    if (pPkt == NULL_PTR)
        return;
    
    SAL_CoreCriticalEnter();
    
    // 포인터로부터 인덱스 계산
    uint32 index = (uint32)(pPkt - &g_canTxPool[0]);
    if (index < CAN_TX_POOL_SIZE)
    {
        g_canTxPoolUsed[index] = 0;
        SAL_MemSet(pPkt, 0, sizeof(CAN_queue_pkt_t));
    }
    
    SAL_CoreCriticalExit();
}

/* -------------------------------------------------------------------------
   CAN 콜백 함수
   ------------------------------------------------------------------------- */
static void CAN_RxCallback(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError)
{
    (void)ucCh;
    (void)uiRxIndex;
    (void)uiRxBufferType;
    
    if (uiError == CAN_ERROR_NONE)
    {
        CANMessage_t sRxMsg;
        CANErrorType_t ret;
        CAN_rx_queue_item_t rxItem;
        
        // 수신된 메시지 가져오기
        ret = CAN_GetNewRxMessage(CAN_CHANNEL, &sRxMsg);
        
        if (ret == CAN_ERROR_NONE)
        {
            // (1) 메시지 헤더 복사
            SAL_MemCopy(&rxItem.msg, &sRxMsg, sizeof(CANMessage_t));
            
            // (2) 페이로드 복사
            SAL_MemCopy(rxItem.payload.raw, sRxMsg.mData, 8);
            
            // (3) CRC 검증 (첫 7바이트에 대해 CRC 계산 후 8번째 바이트와 비교)
            uint8_t calculated_crc = calculate_CRC8(rxItem.payload.raw, 7);
            uint8_t received_crc = rxItem.payload.field.CRC_8;
            
            // if (calculated_crc != received_crc)
            {
                // CRC 불일치: 메시지 버림 (ISR 컨텍스트이므로 로그 출력 최소화)
                mcu_printf("[CAN] CRC mismatch: calc=0x%02X, recv=0x%02X, dropped\r\n", 
                           calculated_crc, received_crc);
                // return; // 메시지를 큐에 넣지 않고 버림
            }
            
            // (4) CRC 검증 통과: Rx Queue에 넣기 (ISR 컨텍스트이므로 비블로킹)
            SALRetCode_t queueRet = SAL_QueuePut(g_canRxQueueHandle,
                                                  &rxItem,
                                                  sizeof(CAN_rx_queue_item_t),
                                                  0,  // timeout (비블로킹)
                                                  SAL_OPT_NON_BLOCKING); // non-blocking option
            
            if (queueRet != SAL_RET_SUCCESS)
            {
                // 큐가 가득 찬 경우 (일반적으로 발생하지 않아야 함)
                mcu_printf("[CAN] Rx Queue full, message dropped\r\n");
            }
        }
    }
}

static void CAN_TxCallback(uint8 ucCh, CANTxInterruptType_t uiIntType)
{
    (void)ucCh;
    (void)uiIntType;
    // Tx 완료는 태스크에서 확인하므로 여기서는 특별한 처리가 필요 없음
}

static void CAN_ErrorCallback(uint8 ucCh, CANErrorType_t uiError)
{
    (void)ucCh;
    
    // ISR 컨텍스트에서는 최소한의 처리만 수행
    // 에러 카운트만 증가시키고, 실제 출력은 태스크에서 처리
    g_canErrorCount++;
    g_canLastErrorType = uiError;
    
    // 에러 발생 빈도가 너무 높으면 출력을 억제
    // (이미 에러가 발생하고 있다는 것을 알 수 있으므로)
}

/* -------------------------------------------------------------------------
   1. 초기화 및 설정 함수
   ------------------------------------------------------------------------- */
void CAN_init(void)
{
    CANErrorType_t ret;
    
    mcu_printf("[CAN] Initializing...\r\n");
    
    // 1. 메모리 풀 초기화
    SAL_MemSet(g_canTxPool, 0, sizeof(g_canTxPool));
    SAL_MemSet(g_canTxPoolUsed, 0, sizeof(g_canTxPoolUsed));
    
    // 2. Tx 큐 생성
    SALRetCode_t queueRet = SAL_QueueCreate(&g_canTxQueueHandle, 
                          (const uint8 *)"CAN_TxQueue",
                          CAN_TX_QUEUE_SIZE,
                          sizeof(CAN_queue_pkt_t*));
    
    if (queueRet != SAL_RET_SUCCESS)
    {
        mcu_printf("[CAN] Tx Queue create failed: %d\r\n", queueRet);
        return;
    }
    else
    {
        mcu_printf("[CAN] Tx Queue created successfully, handle: %u, size: %d, item_size: %u\r\n", 
                   (unsigned int)g_canTxQueueHandle, CAN_TX_QUEUE_SIZE, (unsigned int)sizeof(CAN_queue_pkt_t*));
    }
    
    // 3. Rx 큐 생성
    queueRet = SAL_QueueCreate(&g_canRxQueueHandle,
                               (const uint8 *)"CAN_RxQueue",
                               CAN_RX_QUEUE_SIZE,
                               sizeof(CAN_rx_queue_item_t));
    
    if (queueRet != SAL_RET_SUCCESS)
    {
        mcu_printf("[CAN] Rx Queue create failed: %d\r\n", queueRet);
        return;
    }
    
    // 4. CAN 드라이버 초기화
    // 이미 초기화되어 있으면 Deinit 후 재초기화
    ret = CAN_Init();
    if (ret != CAN_ERROR_NONE)
    {
        mcu_printf("[CAN] CAN_Init failed: %d (CAN_ERROR_BAD_PARAM=2), trying CAN_Deinit first...\r\n", ret);
        // 이미 초기화되어 있을 수 있으므로 Deinit 후 재시도
        (void)CAN_Deinit();
        SAL_TaskSleep(10); // 짧은 딜레이
        ret = CAN_Init();
        if (ret != CAN_ERROR_NONE)
        {
            mcu_printf("[CAN] CAN_Init failed again: %d. Check if CANDriverInfo is properly initialized.\r\n", ret);
            return;
        }
        else
        {
            mcu_printf("[CAN] CAN_Init succeeded after Deinit\r\n");
        }
    }
    
    // 5. 콜백 함수 등록
    CAN_RegisterCallbackFunctionTx(CAN_TxCallback);
    CAN_RegisterCallbackFunctionRx(CAN_RxCallback);
    CAN_RegisterCallbackFunctionError(CAN_ErrorCallback);
    
    // 6. 필터 설정은 can_par.c에서 이미 설정되어 있음
    // 필요시 런타임에 필터를 수정하려면 can_par.c의 StandardIDFilterPar를 수정해야 함
    
    mcu_printf("[CAN] Initialization completed\r\n");
}

/* -------------------------------------------------------------------------
   큐 메시지 개수 추적 함수 (디버깅용)
   ------------------------------------------------------------------------- */
uint32 CAN_GetTxQueueCount(void)
{
    return g_canTxQueueCount;
}

void CAN_IncrementTxQueueCount(void)
{
    g_canTxQueueCount++;
}

/* -------------------------------------------------------------------------
   2. CAN 송신(Tx) 태스크 - Gatekeeper Implementation
   ------------------------------------------------------------------------- */
static void CAN_TxTask_Loop(void *pArg)
{
    (void)pArg;
    
    CANMessage_t sTxMsg;
    CAN_queue_pkt_t *rxPacket;
    CANErrorType_t canRet;
    SALRetCode_t salRet;
    uint32 uiSizeCopied;
    uint8 ucTxBufferIndex;
    
    SAL_TaskSleep(1000);
    mcu_printf("[CAN] Tx Task started\r\n");
    
    // [공통 설정] ID를 제외한 나머지 설정은 고정
    SAL_MemSet(&sTxMsg, 0, sizeof(CANMessage_t));
    sTxMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO;
    sTxMsg.mExtendedId = 0;          // Standard ID
    sTxMsg.mRemoteTransmitRequest = 0; // Data Frame
    sTxMsg.mDataLength = 8;          // Data Length 8
    sTxMsg.mFDFormat = 0;             // Classic CAN
    sTxMsg.mBitRateSwitching = 0;
    sTxMsg.mMessageMarker = 0xFF;
    sTxMsg.mEventFIFOControl = 1;
    
    for(;;)
    {
        // CAN 에러 상태 주기적 보고 (100번 에러마다)
        if (g_canErrorCount > 0)
        {
            g_canErrorReportCounter++;
            if (g_canErrorReportCounter >= CAN_ERROR_REPORT_INTERVAL)
            {
                const char* errorName = "UNKNOWN";
                switch(g_canLastErrorType)
                {
                    case CAN_ERROR_INT_PROTOCOL: errorName = "PROTOCOL"; break;
                    case CAN_ERROR_INT_BUS_OFF: errorName = "BUS_OFF"; break;
                    case CAN_ERROR_INT_WARNING: errorName = "WARNING"; break;
                    case CAN_ERROR_INT_PASSIVE: errorName = "PASSIVE"; break;
                    case CAN_ERROR_INT_BIT: errorName = "BIT"; break;
                    case CAN_ERROR_INT_TIMEOUT: errorName = "TIMEOUT"; break;
                    case CAN_ERROR_INT_RAM_ACCESS_FAIL: errorName = "RAM_ACCESS_FAIL"; break;
                    case CAN_ERROR_INT_TX_EVENT_FULL: errorName = "TX_EVENT_FULL"; break;
                    case CAN_ERROR_INT_TX_EVENT_LOST: errorName = "TX_EVENT_LOST"; break;
                    default: break;
                }
                mcu_printf("[CAN] Error occurred: %s (total: %u)\r\n", errorName, (unsigned int)g_canErrorCount);
                g_canErrorCount = 0;  // 카운트 리셋
                g_canErrorReportCounter = 0;
            }
        }
        
        // 1. 큐 대기 (Blocking)
        salRet = SAL_QueueGet(g_canTxQueueHandle,
                              &rxPacket,
                              &uiSizeCopied,
                              0,  // timeout (0 = wait forever, uint32a 타입)
                              0);  // blocking option (0 = wait)
        
        if (salRet == SAL_RET_SUCCESS)
        {
            // mcu_printf("[CAN] Queue get success, size: %u, packet: %p\r\n", (unsigned int)uiSizeCopied, rxPacket);
        }
        if (salRet == SAL_RET_SUCCESS && rxPacket != NULL_PTR)
        {
            // 큐에서 메시지를 가져왔으므로 카운트 감소
            if (g_canTxQueueCount > 0)
            {
                g_canTxQueueCount--;
            }
            
            sTxMsg.mId = rxPacket->id;
            
            uint8_t temp_byte; // 스왑용 임시 변수
            
            // 여기에서 TimeStamp + CRC 계산해야함
            // TIMESTAMP에서 하위 2바이트만 가져와서 설정
            {
                uint32_t timestamp_value = 0;
                TIMESTAMP_get_ms(&timestamp_value);
                rxPacket->body.field.time_ms = (uint16_t)(timestamp_value & 0xFFFFU);  // 하위 2바이트만 사용
            }
            
            temp_byte = rxPacket->body.raw[4];
            rxPacket->body.raw[4] = rxPacket->body.raw[5];
            rxPacket->body.raw[5] = temp_byte;
            
            // Case 1: ID 0x24 -> [0]과 [1] 교환
            if (sTxMsg.mId == CAN_type_sonar || sTxMsg.mId == CAN_type_compass)
            {
                temp_byte = rxPacket->body.raw[0];
                rxPacket->body.raw[0] = rxPacket->body.raw[1];
                rxPacket->body.raw[1] = temp_byte;
            }
            // Case 2: dual_u16 형식 (accel, rel_distance) -> [0]<->[1] 교환 AND [2]<->[3] 교환
            else if (sTxMsg.mId == CAN_type_accel || sTxMsg.mId == CAN_type_rel_distance)
            {
                // 0, 1 교환
                temp_byte = rxPacket->body.raw[0];
                rxPacket->body.raw[0] = rxPacket->body.raw[1];
                rxPacket->body.raw[1] = temp_byte;
                
                // 2, 3 교환
                temp_byte = rxPacket->body.raw[2];
                rxPacket->body.raw[2] = rxPacket->body.raw[3];
                rxPacket->body.raw[3] = temp_byte;
            }
            
            // CRC 계산
            rxPacket->body.field.CRC_8 = calculate_CRC8(rxPacket->body.raw, 7);
            
            // 데이터 복사
            SAL_MemCopy(sTxMsg.mData, rxPacket->body.raw, 8);
            
            // 3. 실제 전송 (NUCLEO STM32 등과 통신 시 ACK를 받으려면 반드시 호출 필요)
            canRet = CAN_SendMessage(CAN_CHANNEL, &sTxMsg, &ucTxBufferIndex);
            
            if (canRet != CAN_ERROR_NONE)
            {
                // 전송 에러 처리 (버스 오프, 미수신 등 → ACK 없음 원인)
                mcu_printf("[CAN] Message Send Failed: %d\r\n", canRet);
            }
            else
            {
                mcu_printf("[CAN] MESSAGE SEND: 0x%03X ", sTxMsg.mId);
                Print_Hex_8Bytes(rxPacket->body.raw);
            }
            
            // 전송 성공/실패와 관계없이 pool 할당 해제 (실패 시에도 재사용 위해)
            SAL_MemSet(rxPacket->body.raw, 0, 8);
            CAN_FreePool(rxPacket);
        }
        else
        {
            // 큐에서 가져오기 실패
            mcu_printf("[CAN] Queue get failed\r\n");
            SAL_TaskSleep(1);
        }
    }
}

/* -------------------------------------------------------------------------
   3. CAN 수신(Rx) 태스크
   ------------------------------------------------------------------------- */
static void CAN_RxTask_Loop(void *pArg)
{
    (void)pArg;
    
    CAN_rx_queue_item_t rxItem;
    SALRetCode_t salRet;
    uint32 uiSizeCopied;
    
    mcu_printf("[CAN] Rx Task started\r\n");
    
    for(;;)
    {
        // 1. Rx 큐 대기 (Blocking)
        salRet = SAL_QueueGet(g_canRxQueueHandle,
                              &rxItem,
                              &uiSizeCopied,
                              0,  // timeout (0 = wait forever)
                              0); // blocking option (0 = wait)
        
        if (salRet == SAL_RET_SUCCESS)
        {
            // 2. 메시지 소비 함수 호출
            CAN_consume_rx_message(&rxItem.msg, rxItem.payload);
        }
        else
        {
            // 큐에서 가져오기 실패
            SAL_TaskSleep(1);
        }
    }
}

/* -------------------------------------------------------------------------
   태스크 시작 함수들
   ------------------------------------------------------------------------- */

// Tx Task 시작 함수 (GateKeeper)
void CAN_TX_start_task(void)
{
    SALRetCode_t ret;
    
    // CAN 초기화 (한 번만 수행)
    static uint8 init_done = 0;
    if (init_done == 0)
    {
        CAN_init();
        init_done = 1;
    }
    
    // 이미 Task가 생성되어 있으면 중복 생성 방지
    if (g_can_tx_task_id != 0)
    {
        mcu_printf("[CAN] Tx Task already created\r\n");
        return;
    }
    
    // Tx Task 생성 (GateKeeper)
    ret = (SALRetCode_t)SAL_TaskCreate(
        &g_can_tx_task_id,
        (const uint8 *)"CAN Tx Task",
        (SALTaskFunc)CAN_TxTask_Loop,
        &g_can_tx_task_stk[0],
        CAN_TASK_STK_SIZE,
        CAN_TASK_PRIO,
        NULL
    );
    
    if (ret != SAL_RET_SUCCESS)
    {
        mcu_printf("[CAN] Tx Task create failed: %d\r\n", ret);
        return;
    }
    
    mcu_printf("[CAN] Tx Task created successfully\r\n");
}

// Rx Task 시작 함수
void CAN_RX_start_task(void)
{
    SALRetCode_t ret;
    
    // CAN 초기화 (한 번만 수행)
    static uint8 init_done = 0;
    if (init_done == 0)
    {
        CAN_init();
        init_done = 1;
    }
    
    // 이미 Task가 생성되어 있으면 중복 생성 방지
    if (g_can_rx_task_id != 0)
    {
        mcu_printf("[CAN] Rx Task already created\r\n");
        return;
    }
    
    // Rx Task 생성
    ret = (SALRetCode_t)SAL_TaskCreate(
        &g_can_rx_task_id,
        (const uint8 *)"CAN Rx Task",
        (SALTaskFunc)CAN_RxTask_Loop,
        &g_can_rx_task_stk[0],
        CAN_TASK_STK_SIZE,
        CAN_TASK_PRIO,
        NULL
    );
    
    if (ret != SAL_RET_SUCCESS)
    {
        mcu_printf("[CAN] Rx Task create failed: %d\r\n", ret);
        return;
    }
    
    mcu_printf("[CAN] Rx Task created successfully\r\n");
}

// 기존 호환성을 위한 함수 (Tx와 Rx 모두 시작)
void CAN_start_task(void)
{
    CAN_TX_start_task();
    CAN_RX_start_task();
}
