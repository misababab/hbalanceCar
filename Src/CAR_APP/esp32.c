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



uint8_t NET_MODE = 0;            			 //0������ģʽ     1��wifiģʽ   Ĭ������
uint8_t WIFI_CONNECT_FLAG = 0;   			 //WIFI���ӱ�־λ
uint8_t BLE_CONNECT_FLAG = 0;     			 //BLE���ӱ�־λ

extern 	QueueHandle_t Message_Queue;
Esp32_RcvBuf_t Esp32_RcvBufStruct;           //ESP���ݽ��ջ�����

TimeDelay_t    ESP32_TimeDelay;

volatile ATCmd_t  ATCmdStruct[20]=
{
	//*CmdString,         *CmdEchoString,    TimeOut,   CmdStatus��  
	{NULL,NULL,0,NO_RCV},
	{"AT\r\n",             		"OK",         5000,      NO_RCV, },   //���ATָ��       
	{"AT+CIPAPMAC?\r\n",   		"CIPAPMAC",   2000,      NO_RCV, },	  //��ȡMAC��ַ    
	{AT_CWSAP_STRING,       	"CWSAP" ,     2000,      NO_RCV, },   //����MAC��ص�AP����  
	{"AT+CWMODE=3\r\n",    		"OK" ,        2000,      NO_RCV, },   //����WIFIģʽAP+Station
	{"AT+CIPMUX=1\r\n",    		"OK" ,        2000,      NO_RCV, },   //���ö�����
	{"AT+CIPSERVER=1\r\n", 		"OK" ,        2000,      NO_RCV, },   //��ʼ��TCP������ Ĭ��IP��192.168.4.1��Ĭ�϶˿ںţ�333��
	{"AT+CIPSTO=0\r\n",    		"OK" ,        2000,      NO_RCV, },   //����TCP����ʱ��
	{"AT+CIPSEND=0\r\n",   		"OK" ,        500,       NO_RCV, },   //TCP��������

	{"AT+RST\r\n",         		"ready" ,     1000,      NO_RCV, },   //����ATָ�
	{"AT+BLEINIT=2\r\n",   		"OK" ,        1000,      NO_RCV, },   //��ʼ��Ϊ BLE server��
	{"AT+BLEADDR?\r\n",    		"BLEADDR" ,   2000,      NO_RCV, },   //��ѯ����� BLE ��ַ
	{AT_BLEADVDATA_STRING,  	"OK" ,        2000,      NO_RCV, },   //���ù㲥���ݰ�
	{"AT+BLEGATTSSRVCRE\r\n", 	"OK",         1000,      NO_RCV, },   //��������
	{"AT+BLEGATTSSRVSTART\r\n", "OK" ,        3000,      NO_RCV, },   //�������� 
	{"AT+BLEADVSTART\r\n",   	"OK" ,        1000,      NO_RCV, },   //��ʼ�㲥
	{"AT+BLEGATTSNTFY\r\n" , 	">" ,         500,       NO_RCV, },   //��������������

	{"CMDSTR_NOUSE",       		"OK" ,        2000,      NO_RCV, }, 
};

void ESP32_Init(void)
{
	ATCmdNum_t i = AT_IDIE;//ö������
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

void ESP32_DeQueue_exeCmd(void)//stm32����
{
	memset(&Esp32_RcvBufStruct,0,sizeof(Esp32_RcvBufStruct));
	if(xQueueReceive(Message_Queue, &Esp32_RcvBufStruct,0 ))
	{
		DBG("%s", Esp32_RcvBufStruct.RcvDataBuf);
		 
		if(NET_MODE == BLE_MODE)   // ����ģʽ 
		{
			if(strstr((char *)(Esp32_RcvBufStruct.RcvDataBuf),"WRITE") != NULL ) //�յ��ͻ�������
			{
				DBG("�յ���λ������\n");

				BLE_CONNECT_FLAG = 1;   //�Է��򿪶�д����ֵʱ�������ӱ�־
				
				EP32_RcvData_Extract(Esp32_RcvBufStruct.RcvDataBuf,Esp32_RcvBufStruct.DataLen);	
				return ;			          				
			}

			if(strstr((char *)(Esp32_RcvBufStruct.RcvDataBuf),"BLEDISCONN") != NULL) //�ͻ��˶Ͽ�����
			{
				DBG("�����Ͽ����ӣ����¹㲥\n");	

				BLE_CONNECT_FLAG = 0;    //������ӱ�־λ
				
				ESP32_Send_ATCmd(AT_BLEADVDATA);//���¹㲥
				ESP32_Send_ATCmd(AT_BLEADVSTART);
			}
		}
		
		else     // WIFIģʽ 
		{

			if((!WIFI_CONNECT_FLAG) && (strstr((char *)(Esp32_RcvBufStruct.RcvDataBuf),"CONNECT")!=NULL )) //�յ��ͻ�������
			{
				DBG("WIFI������\n");	

				WIFI_CONNECT_FLAG = 1;   //�����ӱ�־λ
			}
			if(strstr((char *)(Esp32_RcvBufStruct.RcvDataBuf),"+IPD") != NULL  ) //�յ��ͻ�������
			{
				DBG("WIFI�յ���λ������\n");	

				EP32_RcvData_Extract(Esp32_RcvBufStruct.RcvDataBuf,Esp32_RcvBufStruct.DataLen);	

				return ;						

			}	
			if(strstr((char *)(Esp32_RcvBufStruct.RcvDataBuf),"CLOSED") != NULL) //�ͻ��˶Ͽ�����
			{
				DBG("WIFI�Ͽ�����\n");	

				WIFI_CONNECT_FLAG = 0;    //������ӱ�־λ
			}	
		}
	}
}

ATCmdStatus_t ESP32_Send_AppLayerData(uint8_t *SendBuf,uint8_t len)//stm32����
{
	uint8_t tempATCMDbuf[30] =  {0};
	ATCmdNum_t ATCmdNum;

	if(! (BLE_CONNECT_FLAG || WIFI_CONNECT_FLAG))  //δ����״̬���ܷ�������
	{
		DBG("δ�����豸\n");
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
	ATCmdStruct[ATCmdNum].CmdStatus = NO_RCV;        			//�����״̬
	SetTime(&ESP32_TimeDelay, ATCmdStruct[ATCmdNum].TimeOut);	//�򿪳�ʱ��ʱ��
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
	SetTime(&ESP32_TimeDelay, ATCmdStruct[ATCmdNum].TimeOut);//���ó�ʱ
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



