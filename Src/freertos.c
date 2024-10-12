/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     

#include "main.h"
#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "car_task.h"
#include "inv_mpu_user.h"
#include "esp32.h"
#include "connect.h"
#include "oled.h"
#include "oled_show.h"
#include "car_system.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId Task_200HZHandle;
osThreadId Task_100HZHandle;
osThreadId Task_5HZHandle;
osThreadId Task_InteractioHandle;

/* USER CODE BEGIN Variables */

#define   Message_Q_NUM      5
#define   Message_Q_Length   sizeof(Esp32_RcvBuf_t)
xQueueHandle  Message_Queue;
Esp32_RcvBuf_t Uart6_RcvCacheBufStruct;


/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void TASK_MPU6050_200HZ(void const * argument);
void TASK_Pid_Control_100HZ(void const * argument);
void TASK_Oled_Menu_5HZ(void const * argument);
void TASK_Queue_Interaction_100HZ(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of Task_200HZ */
  osThreadDef(Task_200HZ, TASK_MPU6050_200HZ, osPriorityNormal, 0, 128);
  Task_200HZHandle = osThreadCreate(osThread(Task_200HZ), NULL);

  /* definition and creation of Task_100HZ */
  osThreadDef(Task_100HZ, TASK_Pid_Control_100HZ, osPriorityIdle, 0, 128);
  Task_100HZHandle = osThreadCreate(osThread(Task_100HZ), NULL);

  /* definition and creation of Task_5HZ */
  osThreadDef(Task_5HZ, TASK_Oled_Menu_5HZ, osPriorityIdle, 0, 128);
  Task_5HZHandle = osThreadCreate(osThread(Task_5HZ), NULL);

  /* definition and creation of Task_Interactio */
  osThreadDef(Task_Interactio, TASK_Queue_Interaction_100HZ, osPriorityIdle, 0, 128);
  Task_InteractioHandle = osThreadCreate(osThread(Task_Interactio), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* TASK_MPU6050_200HZ function */
void TASK_MPU6050_200HZ(void const * argument)
{

  /* USER CODE BEGIN TASK_MPU6050_200HZ */
	
	MPU6050_Init();
	
	while(mpu_dmp_init()){};
	
  /* Infinite loop */
  for(;;)
  {
		Get_MPU6050_Data_5ms();//传感器设置了100hz的转换速度
		
		osDelay(5);
  }
  /* USER CODE END TASK_MPU6050_200HZ */
}

/* TASK_Pid_Control_100HZ function */
void TASK_Pid_Control_100HZ(void const * argument)
{
  /* USER CODE BEGIN TASK_Pid_Control_100HZ */
	
  /* Infinite loop */
  for(;;)
  {
		Pid_Control_10ms();
	  
		osDelay(10);
  }
  /* USER CODE END TASK_Pid_Control_100HZ */
}

/* TASK_Oled_Menu_5HZ function */
void TASK_Oled_Menu_5HZ(void const * argument)
{
  /* USER CODE BEGIN TASK_Oled_Menu_5HZ */
	
	printf("菜单显示进程运行\n");
	
	oled_Init();	//0.96寸OLED显示屏初始化
	
	StaticPage();  	//系统首次显示静态界面
	
  /* Infinite loop */
  for(;;)
  {
		Oled_Show_Menu_200ms();
	  
		osDelay(200);
  }
  /* USER CODE END TASK_Oled_Menu_5HZ */
}

/* TASK_Queue_Interaction_100HZ function */
void TASK_Queue_Interaction_100HZ(void const * argument)
{
  /* USER CODE BEGIN TASK_Queue_Interaction_100HZ */
	
	
	uint8_t time = 0;
	printf("交互进程运行\n");

	Message_Queue = xQueueCreate( Message_Q_NUM, Message_Q_Length );
	
	HAL_UART_Receive_DMA(&huart6, Uart6_RcvCacheBufStruct.RcvDataBuf, 255);
	
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE );
	
	ESP32_Init();

  /* Infinite loop */
  for(;;)
  {
		ESP32_DeQueue_exeCmd();
		
		time++;
		if(time >= 50)
		{
			time = 0;
			Connect_Send_data(READ_ALL_ARG);
		}
		
		osDelay(10);
  }
  /* USER CODE END TASK_Queue_Interaction_100HZ */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
