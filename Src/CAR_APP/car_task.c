#include "car_task.h"
#include "mpu6050.h"
#include "inv_mpu_user.h"
#include "contrl.h"
#include "oled.h"
#include "oled_show.h"
#include "car_system.h"

int  FS_MODE = 0 ;                          //0、蓝牙遥控模式   1、蔽障模式  2、巡线模式
										    
int  rt_EncoderLeft , rt_EncoderRight;      //实时速度，在PID控制任务维护
float target_SpeedEncoder = 0;              //目标速度，在菜单任务 和 交互任务维护

int  rt_Distence ;                       	//小车和障碍物的距离，在外部中断里维护
int  rt_TrunAngle = 64;         			//实时转向角，在交互任务维护
struct CCD_t  CCD;                          //摄像头的数据，在串口3的（RX通道）DMA1_Stream1中断里维护

int  Vertical_Out, Velocity_Out, Turn_Out;   //PID控制输出,中间变量
int  motor1_pwm, motor2_pwm;                 //最终PWM输出，中间变量											

void Get_MPU6050_Data_5ms(void)
{
	static struct mpu6050_data mpu6050Struct_Last;

	if(mpu_dmp_get_data() !=0 )			//获取欧拉角和加速度角速度数据
		mpu6050Struct = mpu6050Struct_Last;
	else
		mpu6050Struct_Last = mpu6050Struct;

}

void Pid_Control_10ms(void)
{
	
	HC_SRC04_Start();					// 启动超声波模块检测，然后再在中断里更新rt_Distence
	rt_EncoderLeft  = Read_Encoder(1);	//相当于10ms读取一次
	rt_EncoderRight = -Read_Encoder(2);
	Vertical_Out = Vertical_PD(mpu6050Struct.pitch, mpu6050Struct.gyro_x);
	Velocity_Out = Velocity_PI(rt_EncoderLeft,rt_EncoderRight,mpu6050Struct.pitch, target_SpeedEncoder );
	
	if(FS_MODE == 0)         	//遥控模式
	{
		Turn_Out = Turn_PD(rt_TrunAngle, mpu6050Struct.gyro_z);
	}
	else if(FS_MODE == 1)  		//避障模式
	{
		if(rt_Distence < 20)  Turn_Out = Turn_PD(20, mpu6050Struct.gyro_z);
		else  				  Turn_Out = 0;
	}
	else if(FS_MODE == 2)  		//巡线模式
	{
		Turn_Out = Turn_PD(CCD.middle, mpu6050Struct.gyro_z);
	}
	motor1_pwm = Vertical_Out + Velocity_Out + Turn_Out;
	motor2_pwm = Vertical_Out + Velocity_Out - Turn_Out;

	PWM_Limiting(&motor1_pwm,&motor2_pwm);
				
	if(Turn_off(mpu6050Struct.pitch)==0)	//判断小车是否出现特殊状况
	{
		Set_PWM(motor1_pwm,motor2_pwm);		//motor1_pwm,motor2_pwm范围是 0-5800   ,PWM的最大CCR为8000
	}

}

void Oled_Show_Menu_200ms(void)
{
	 Task_State();

	 oled_Show();
}

void  HC_SRC04_Start(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	delay_us(20);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	
}
