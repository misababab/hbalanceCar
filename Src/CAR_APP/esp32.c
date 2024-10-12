#include "esp32.h"
#include "usart.h"
#include <stdarg.h>
#include <string.h>
#include "connect.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


#define AT_CWSAP_STRING  		"AT+CWSAP=\"FarsightESP32\",\"123456789\",5,3\r\n"
#define AT_BLEADVDATA_STRING    "AT+BLEADVDATA=\"0201060B09466172736967687420030302A0\"\r\n"



uint8_t NET_MODE = 0;            			 //0、蓝牙模式     1、wifi模式   默认蓝牙
uint8_t WIFI_CONNECT_FLAG = 0;   			 //WIFI连接标志位
uint8_t BLE_CONNECT_FLAG = 0;     			 //BLE连接标志位

extern 	QueueHandle_t Message_Queue;
Esp32_RcvBuf_t Esp32_RcvBufStruct;           //ESP数据接收缓冲区

TimeDelay_t    ESP32_TimeDelay;

volatile ATCmd_t  ATCmdStruct[20]=
{
	//*CmdString,         *CmdEchoString,    TimeOut,   CmdStatus，  
	{NULL,NULL,0,NO_RCV},
	{"AT\r\n",             		"OK",         5000,      NO_RCV, },   //检测AT指令       
	{"AT+CIPAPMAC?\r\n",   		"CIPAPMAC",   2000,      NO_RCV, },	  //获取MAC地址    
	{AT_CWSAP_STRING,       	"CWSAP" ,     2000,      NO_RCV, },   //建立MAC相关的AP名称  
	{"AT+CWMODE=3\r\n",    		"OK" ,        2000,      NO_RCV, },   //设置WIFI模式AP+Station
	{"AT+CIPMUX=1\r\n",    		"OK" ,        2000,      NO_RCV, },   //设置多连接
	{"AT+CIPSERVER=1\r\n", 		"OK" ,        2000,      NO_RCV, },   //初始化TCP服务器 默认IP（192.168.4.1）默认端口号（333）
	{"AT+CIPSTO=0\r\n",    		"OK" ,        2000,      NO_RCV, },   //设置TCP连接时间
	{"AT+CIPSEND=0\r\n",   		"OK" ,        500,       NO_RCV, },   //TCP发送数据

	{"AT+RST\r\n",         		"ready" ,     1000,      NO_RCV, },   //重启AT指令：
	{"AT+BLEINIT=2\r\n",   		"OK" ,        1000,      NO_RCV, },   //初始化为 BLE server：
	{"AT+BLEADDR?\r\n",    		"BLEADDR" ,   2000,      NO_RCV, },   //查询自身的 BLE 地址
	{AT_BLEADVDATA_STRING,  	"OK" ,        2000,      NO_RCV, },   //配置广播数据包
	{"AT+BLEGATTSSRVCRE\r\n", 	"OK",         1000,      NO_RCV, },   //创建服务：
	{"AT+BLEGATTSSRVSTART\r\n", "OK" ,        3000,      NO_RCV, },   //开启服务 
	{"AT+BLEADVSTART\r\n",   	"OK" ,        1000,      NO_RCV, },   //开始广播
	{"AT+BLEGATTSNTFY\r\n" , 	">" ,         500,       NO_RCV, },   //服务器发送数据

	{"CMDSTR_NOUSE",       		"OK" ,        2000,      NO_RCV, }, 
};

void ESP32_Init(void)
{
	ATCmdNum_t i = AT_IDIE;//枚举类型
	if(NET_MODE == BLE_MODE)
	{
		for(i = AT_BLEINIT; i<=AT_BLEADVSTART ; i++)
		{
			if( ESP32_Send_ATCmd(i) != RCV_SUCCESS)
			{				
				DBG("PES32 Init failed\n");
				return ;
			}

		}
	}
	else
	{
		for(i = AT; i<=AT_CIPSTO ; i++)
		{						
			if( ESP32_Send_ATCmd(i) != RCV_SUCCESS)
			{
				DBG("PES32 Init failed\n");
				return ;
			}
		}

		DBG("PES32 Init Success\n");
	}
}

void ESP32_DeQueue_exeCmd(void)//stm32接收
{
	memset(&Esp32_RcvBufStruct,0,sizeof(Esp32_RcvBufStruct));
	if(xQueueReceive(Message_Queue, &Esp32_RcvBufStruct,0 ))
	{
		DBG("%s", Esp32_RcvBufStruct.RcvDataBuf);
		 
		if(NET_MODE == BLE_MODE)   // 蓝牙模式 
		{
			if(strstr((char *)(Esp32_RcvBufStruct.RcvDataBuf),"WRITE") != NULL ) //收到客户端数据
			{
				DBG("收到上位机数据\n");

				BLE_CONNECT_FLAG = 1;   //对方打开读写特征值时，置连接标志
				
				EP32_RcvData_Extract(Esp32_RcvBufStruct.RcvDataBuf,Esp32_RcvBufStruct.DataLen);	
				return ;			          				
			}

			if(strstr((char *)(Esp32_RcvBufStruct.RcvDataBuf),"BLEDISCONN") != NULL) //客户端断开连接
			{
				DBG("蓝牙断开连接，重新广播\n");	

				BLE_CONNECT_FLAG = 0;    //清除连接标志位
				
				ESP32_Send_ATCmd(AT_BLEADVDATA);//重新广播
				ESP32_Send_ATCmd(AT_BLEADVSTART);
			}
		}
		
		else     // WIFI模式 
		{

			if((!WIFI_CONNECT_FLAG) && (strstr((char *)(Esp32_RcvBufStruct.RcvDataBuf),"CONNECT")!=NULL )) //收到客户端数据
			{
				DBG("WIFI已连接\n");	

				WIFI_CONNECT_FLAG = 1;   //置连接标志位
			}
			if(strstr((char *)(Esp32_RcvBufStruct.RcvDataBuf),"+IPD") != NULL  ) //收到客户端数据
			{
				DBG("WIFI收到上位机数据\n");	

				EP32_RcvData_Extract(Esp32_RcvBufStruct.RcvDataBuf,Esp32_RcvBufStruct.DataLen);	

				return ;						

			}	
			if(strstr((char *)(Esp32_RcvBufStruct.RcvDataBuf),"CLOSED") != NULL) //客户端断开连接
			{
				DBG("WIFI断开连接\n");	

				WIFI_CONNECT_FLAG = 0;    //清除连接标志位
			}	
		}
	}
}

ATCmdStatus_t ESP32_Send_AppLayerData(uint8_t *SendBuf,uint8_t len)//stm32发送
{
	uint8_t tempATCMDbuf[30] =  {0};
	ATCmdNum_t ATCmdNum;

	if(! (BLE_CONNECT_FLAG || WIFI_CONNECT_FLAG))  //未连接状态不能发送数据
	{
		DBG("未连接设备\n");
		return NO_CONNECT;
	}		
	if(NET_MODE == BLE_MODE)    
	{
		sprintf((char *)tempATCMDbuf,"AT+BLEGATTSNTFY=%d,%d,%d,%d\r\n",0,1,2,len);
		ATCmdNum = AT_BLEGATTSNTFY;		
	}
	else 						
	{
		sprintf((char *)tempATCMDbuf,"AT+CIPSEND=%d,%d\r\n",0,len);
		ATCmdNum = AT_CIPSEND;
	}
	uart6_data_send(tempATCMDbuf,strlen((char *)tempATCMDbuf));    
	ATCmdStruct[ATCmdNum].CmdStatus = NO_RCV;        			//清接收状态
	SetTime(&ESP32_TimeDelay, ATCmdStruct[ATCmdNum].TimeOut);	//打开超时定时器
	while(ATCmdStruct[ATCmdNum].CmdStatus != RCV_SUCCESS)
	{		 
		ESP32_DeQueue_MatchATCmdEcho(ATCmdNum);
		if(ATCmdStruct[ATCmdNum].CmdStatus == RCV_TIMEOUT)
		{
			return RCV_TIMEOUT;
		}
	}
	uart6_data_send( SendBuf,len);                
	DBG("send data ok\n");
	return RCV_SUCCESS;

}

ATCmdStatus_t ESP32_Send_ATCmd(ATCmdNum_t ATCmdNum)
{		
	uint8_t len;

	ATCmdStruct[ATCmdNum].CmdStatus = NO_RCV;
	len = strlen(ATCmdStruct[ATCmdNum].CmdString);
	uart6_data_send((uint8_t *)ATCmdStruct[ATCmdNum].CmdString, len);
	HAL_UART_Transmit(&huart1,(uint8_t *)ATCmdStruct[ATCmdNum].CmdString, len,100);
	SetTime(&ESP32_TimeDelay, ATCmdStruct[ATCmdNum].TimeOut);//设置超时
	while(ATCmdStruct[ATCmdNum].CmdStatus != RCV_SUCCESS)
	{
		ESP32_DeQueue_MatchATCmdEcho(ATCmdNum);
		if(ATCmdStruct[ATCmdNum].CmdStatus == RCV_TIMEOUT)
		return RCV_TIMEOUT;
	}

	return RCV_SUCCESS;
}


void ESP32_DeQueue_MatchATCmdEcho(ATCmdNum_t ATCmdNum)
{
	memset(&Esp32_RcvBufStruct,0,sizeof(Esp32_RcvBufStruct));

	if(xQueueReceive(Message_Queue, &Esp32_RcvBufStruct,0 ))
	{
		DBG("%s", Esp32_RcvBufStruct.RcvDataBuf);

		if(strstr((const char*)Esp32_RcvBufStruct.RcvDataBuf,ATCmdStruct[ATCmdNum].CmdEchoString) != NULL)
		{
			ATCmdStruct[ATCmdNum].CmdStatus = RCV_SUCCESS;						
		}			
	}
	else
	{
		if(CompareTime(&ESP32_TimeDelay))
		{
			ATCmdStruct[ATCmdNum].CmdStatus = RCV_TIMEOUT;
		}
	}	

}

void uart6_data_send(uint8_t *fmt, uint16_t len)
{
	taskENTER_CRITICAL();  
	HAL_UART_Transmit(&huart6, (uint8_t *)fmt, len,100);
	taskEXIT_CRITICAL(); 
}



