#ifndef __ESP32_H
#define __ESP32_H

#include <stdint.h>
#include <stdio.h>
#include "delay.h"

//#define DEBUG
#ifdef DEBUG
    #define DBG(x...)   printf(x)
#else
    #define DBG(x...)
#endif

#define  BLE_MODE   0
#define  WIFI_MODE  1

//ATָ�����к�
typedef enum
{
    AT_IDIE  = 0,
    AT,
    AT_CIPAPMAC,
    AT_CWSAP,
    AT_CWMODE,
    AT_CIPMUX,
    AT_CIPSERVER,
    AT_CIPSTO,
    AT_CIPSEND,

    AT_RST,
    AT_BLEINIT,
    AT_BLEADDR,
    AT_BLEADVDATA,
    AT_BLEGATTSSRVCRE,
    AT_BLEGATTSSRVSTART,
    AT_BLEADVSTART,
    AT_BLEGATTSNTFY,
    CMDSTR_NOUSE,

} ATCmdNum_t;

//����ؽ����״̬
typedef enum
{
    NO_RCV  = 0,
    RCV_SUCCESS,
    RCV_TIMEOUT,
    NO_CONNECT,
} ATCmdStatus_t;

typedef struct
{
    char *CmdString;                //���͵�����
    char *CmdEchoString;           //��ȷ���ذ������ַ���
    uint16_t TimeOut;               //��ʱ��ʱ��
    ATCmdStatus_t CmdStatus;        //����ص�״̬
} ATCmd_t;

typedef struct
{
    uint8_t DataLen;
    uint8_t RcvDataBuf  [255];
} Esp32_RcvBuf_t;


extern TimeDelay_t  ESP32_TimeDelay;

extern uint8_t ESP32_RCV_FLAG;
extern uint8_t NET_MODE ;
extern uint8_t WIFI_CONNECT_FLAG ;    //WIFI���ӱ�־λ
extern uint8_t BLE_CONNECT_FLAG ;     //BLE���ӱ�־λ

void ESP32_Init(void);
ATCmdStatus_t ESP32_Send_AppLayerData(uint8_t *SendBuf, uint8_t len);	/* ��������stm32 --->>>  ESP32 */
void ESP32_DeQueue_exeCmd(void);										/* ��������ESP32 --->>>   stm32 */

ATCmdStatus_t ESP32_Send_ATCmd(ATCmdNum_t ATCmdNum);
void ESP32_DeQueue_MatchATCmdEcho(ATCmdNum_t ATCmdNum);
void uart6_data_send(uint8_t *fmt, uint16_t len);

#endif /*__ESP32_H*/

