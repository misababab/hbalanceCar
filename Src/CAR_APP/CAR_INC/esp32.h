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

//AT指令序列号
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

//命令返回结果的状态
typedef enum
{
    NO_RCV  = 0,
    RCV_SUCCESS,
    RCV_TIMEOUT,
    NO_CONNECT,
} ATCmdStatus_t;

typedef struct
{
    char *CmdString;                //发送的命令
    char *CmdEchoString;           //正确返回包含的字符串
    uint16_t TimeOut;               //超时的时间
    ATCmdStatus_t CmdStatus;        //命令返回的状态
} ATCmd_t;

typedef struct
{
    uint8_t DataLen;
    uint8_t RcvDataBuf  [255];
} Esp32_RcvBuf_t;


extern TimeDelay_t  ESP32_TimeDelay;

extern uint8_t ESP32_RCV_FLAG;
extern uint8_t NET_MODE ;
extern uint8_t WIFI_CONNECT_FLAG ;    //WIFI连接标志位
extern uint8_t BLE_CONNECT_FLAG ;     //BLE连接标志位

void ESP32_Init(void);
ATCmdStatus_t ESP32_Send_AppLayerData(uint8_t *SendBuf, uint8_t len);	/* 数据流：stm32 --->>>  ESP32 */
void ESP32_DeQueue_exeCmd(void);										/* 数据流：ESP32 --->>>   stm32 */

ATCmdStatus_t ESP32_Send_ATCmd(ATCmdNum_t ATCmdNum);
void ESP32_DeQueue_MatchATCmdEcho(ATCmdNum_t ATCmdNum);
void uart6_data_send(uint8_t *fmt, uint16_t len);

#endif /*__ESP32_H*/

