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

#include "can.h"
#include "can_bridge.h"
#include "utils.h"
#include "can_config.h"
#include "can_drv.h"
#include "can_par.h"

/* -------------------------------------------------------------------------
   전역 변수 및 핸들 정의
   ------------------------------------------------------------------------- */
#define CAN_TX_QUEUE_SIZE      10
#define CAN_TX_POOL_SIZE      16

// CAN 채널 (기본적으로 CH0 사용)
#define CAN_CHANNEL           0

// 메시지 큐 핸들
uint32 g_canTxQueueHandle = 0;

// 메모리 풀 (정적 배열로 구현)
static CAN_queue_pkt_t g_canTxPool[CAN_TX_POOL_SIZE];
static uint8 g_canTxPoolUsed[CAN_TX_POOL_SIZE];  // 0: 사용 가능, 1: 사용 중

// 태스크 관련 변수
#define CAN_TASK_STK_SIZE     (2048)
#define CAN_TASK_PRIO         (SAL_PRIO_APP_CFG)

static uint32 g_can_task_id = 0;
static uint32 g_can_task_stk[CAN_TASK_STK_SIZE];

// Rx 메시지 디버그용
CAN_payload_t rxMsgDebug;

/* -------------------------------------------------------------------------
   내부 함수 선언
   ------------------------------------------------------------------------- */
static void CAN_Task_Loop(void *pArg);
static void CAN_RxCallback(uint8 ucCh, uint32 uiRxIndex, CANMessageBufferType_t uiRxBufferType, CANErrorType_t uiError);
static void CAN_TxCallback(uint8 ucCh, CANTxInterruptType_t uiIntType);
static void CAN_ErrorCallback(uint8 ucCh, CANErrorType_t uiError);
static CAN_queue_pkt_t* CAN_AllocPool(void);
static void CAN_FreePool(CAN_queue_pkt_t *pPkt);

/* -------------------------------------------------------------------------
   유틸리티 함수
   ------------------------------------------------------------------------- */
void Print_Hex_8Bytes(uint8_t *data)
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
static CAN_queue_pkt_t* CAN_AllocPool(void)
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

static void CAN_FreePool(CAN_queue_pkt_t *pPkt)
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
        
        // 수신된 메시지 가져오기
        ret = CAN_GetNewRxMessage(CAN_CHANNEL, &sRxMsg);
        
        if (ret == CAN_ERROR_NONE)
        {
            // (1) Union 변수 선언
            CAN_payload_t rxPayload;

            // (2) 배열 -> Union 복사 (memcpy가 가장 깔끔함)
            // sRxMsg.mData의 8바이트를 rxPayload.raw에 그대로 복사하면,
            // 자동으로 rxPayload.field... 로 접근 가능해짐
            SAL_MemCopy(rxPayload.raw, sRxMsg.mData, 8);

            // (3) 소비(Consume) 함수 호출
            // *주의: 호출할 때는 타입명(CANMessage_t)을 적지 않습니다.
            // sRxMsg는 구조체이므로 주소(&)를 넘기는 것이 성능상 유리합니다.
            CAN_consume_rx_message(&sRxMsg, rxPayload);
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
    mcu_printf("[CAN] Error on channel %d: %d\r\n", ucCh, uiError);
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
    
    // 2. 큐 생성
    SALRetCode_t queueRet = SAL_QueueCreate(&g_canTxQueueHandle, 
                          (const uint8 *)"CAN_TxQueue",
                          CAN_TX_QUEUE_SIZE,
                          sizeof(CAN_queue_pkt_t*));
    
    if (queueRet != SAL_RET_SUCCESS)
    {
        mcu_printf("[CAN] Queue create failed: %d\r\n", ret);
        return;
    }
    
    // 3. CAN 드라이버 초기화
    ret = CAN_Init();
    if (ret != CAN_ERROR_NONE)
    {
        mcu_printf("[CAN] CAN_Init failed: %d\r\n", ret);
        return;
    }
    
    // 4. 콜백 함수 등록
    CAN_RegisterCallbackFunctionTx(CAN_TxCallback);
    CAN_RegisterCallbackFunctionRx(CAN_RxCallback);
    CAN_RegisterCallbackFunctionError(CAN_ErrorCallback);
    
    // 5. 필터 설정은 can_par.c에서 이미 설정되어 있음
    // 필요시 런타임에 필터를 수정하려면 can_par.c의 StandardIDFilterPar를 수정해야 함
    
    mcu_printf("[CAN] Initialization completed\r\n");
}

/* -------------------------------------------------------------------------
   2. CAN 송신(Tx) 태스크 - Gatekeeper Implementation
   ------------------------------------------------------------------------- */
static void CAN_Task_Loop(void *pArg)
{
    (void)pArg;
    
    CANMessage_t sTxMsg;
    CAN_queue_pkt_t *rxPacket;
    CANErrorType_t canRet;
    SALRetCode_t salRet;
    uint32 uiSizeCopied;
    uint8 ucTxBufferIndex;
    
    mcu_printf("[CAN] Task started\r\n");
    
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
        // 1. 큐 대기 (Blocking)
        salRet = SAL_QueueGet(g_canTxQueueHandle,
                              &rxPacket,
                              &uiSizeCopied,
                              0,  // timeout (0 = wait forever, uint32a 타입)
                              0);  // blocking option (0 = wait)
        
        if (salRet == SAL_RET_SUCCESS && rxPacket != NULL_PTR)
        {
            sTxMsg.mId = rxPacket->id;
            
            uint8_t temp_byte; // 스왑용 임시 변수
            
            // 여기에서 TimeStamp + CRC 계산해야함
            rxPacket->body.field.time_ms = 0;
            
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
            // Case 2: ID 0x2C -> [0]<->[1] 교환 AND [2]<->[3] 교환
            else if (sTxMsg.mId == CAN_type_accel)
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
            
            // 3. 실제 전송
            canRet = CAN_SendMessage(CAN_CHANNEL, &sTxMsg, &ucTxBufferIndex);
            
            if (canRet != CAN_ERROR_NONE)
            {
                // 전송 에러 처리
                mcu_printf("[CAN] Message Send Failed: %d\r\n", canRet);
            }
            else
            {
                mcu_printf("[CAN] MESSAGE SEND: 0x%03X ", sTxMsg.mId);
                Print_Hex_8Bytes(rxPacket->body.raw);
            }
            
            // 전송이 잘 진행되었다면 pool 할당 해제
            SAL_MemSet(rxPacket->body.raw, 0, 8);
            CAN_FreePool(rxPacket);
        }
        else
        {
            // 큐에서 가져오기 실패
            SAL_TaskSleep(1);
        }
    }
}

/* -------------------------------------------------------------------------
   태스크 시작 함수
   ------------------------------------------------------------------------- */
void CAN_start_task(void)
{
    SALRetCode_t ret;
    
    CAN_init();
    
    ret = (SALRetCode_t)SAL_TaskCreate(
        &g_can_task_id,
        (const uint8 *)"CAN Task",
        (SALTaskFunc)CAN_Task_Loop,
        &g_can_task_stk[0],
        CAN_TASK_STK_SIZE,
        CAN_TASK_PRIO,
        NULL
    );
    
    if (ret != SAL_RET_SUCCESS)
    {
        mcu_printf("[CAN] Task create failed: %d\r\n", ret);
        return;
    }
    
    mcu_printf("[CAN] Task created successfully\r\n");
}
