#ifndef __CAR_TASK_H
#define __CAR_TASK_H
#include "stm32f4xx_hal.h"

struct mpu6050_data
{
	
	short acc_x;
	short acc_y;
	short acc_z;

	short gyro_x;
	short gyro_y;
	short gyro_z;

	float pitch;    //俯仰角
	float roll;     //翻滚角
	float yaw;      //偏航角
};

struct CCD_t
{
	uint16_t middle;      //中间位置值
	uint16_t threshold;   //像素ad阈值
	uint16_t left;        //左跳变的位置
	uint16_t right;       //右跳变的位置
};

extern int  FS_MODE  ;                      		//0、遥控模式   1、蔽障模式  2、巡线模式 

extern struct mpu6050_data mpu6050Struct;			//接收mpu6050传感器数据
/* 转向环 */
extern struct CCD_t  CCD;
extern int  rt_TrunAngle ;              //遥控模式，用于转向调节变量
/* 速度环 */
extern int  rt_EncoderLeft, rt_EncoderRight;     	//检测速度
extern float target_SpeedEncoder ;                  //速度调节 
/* 避障 */
extern int  rt_Distence ;                      		//小车和前方障碍物之间的距离
extern uint8_t   rt_Power;                      		//定义电池电量



//extern int  Balance_Pwm,Velocity_Out,Turn_Out;      //PID计算的PWM值
//extern int  motor1_pwm, motor2_pwm;                 //左右电机PWM值




void Get_MPU6050_Data_5ms(void);
void Pid_Control_10ms(void);
void Oled_Show_Menu_200ms(void);

void  HC_SRC04_Start(void);



#endif
