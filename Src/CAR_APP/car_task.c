#include "car_task.h"
#include "mpu6050.h"
#include "inv_mpu_user.h"
#include "contrl.h"
#include "oled.h"
#include "oled_show.h"
#include "car_system.h"

int  FS_MODE = 0 ;                          //0������ң��ģʽ   1������ģʽ  2��Ѳ��ģʽ
										    
int  rt_EncoderLeft , rt_EncoderRight;      //ʵʱ�ٶȣ���PID��������ά��
float target_SpeedEncoder = 0;              //Ŀ���ٶȣ��ڲ˵����� �� ��������ά��

int  rt_Distence ;                       	//С�����ϰ���ľ��룬���ⲿ�ж���ά��
int  rt_TrunAngle = 64;         			//ʵʱת��ǣ��ڽ�������ά��
struct CCD_t  CCD;                          //����ͷ�����ݣ��ڴ���3�ģ�RXͨ����DMA1_Stream1�ж���ά��

int  Vertical_Out, Velocity_Out, Turn_Out;   //PID�������,�м����
int  motor1_pwm, motor2_pwm;                 //����PWM������м����											

void Get_MPU6050_Data_5ms(void)
{
	static struct mpu6050_data mpu6050Struct_Last;

	if(mpu_dmp_get_data() !=0 )			//��ȡŷ���Ǻͼ��ٶȽ��ٶ�����
		mpu6050Struct = mpu6050Struct_Last;
	else
		mpu6050Struct_Last = mpu6050Struct;

}

void Pid_Control_10ms(void)
{
	
	HC_SRC04_Start();					// ����������ģ���⣬Ȼ�������ж������rt_Distence
	rt_EncoderLeft  = Read_Encoder(1);	//�൱��10ms��ȡһ��
	rt_EncoderRight = -Read_Encoder(2);
	Vertical_Out = Vertical_PD(mpu6050Struct.pitch, mpu6050Struct.gyro_x);
	Velocity_Out = Velocity_PI(rt_EncoderLeft,rt_EncoderRight,mpu6050Struct.pitch, target_SpeedEncoder );
	
	if(FS_MODE == 0)         	//ң��ģʽ
	{
		Turn_Out = Turn_PD(rt_TrunAngle, mpu6050Struct.gyro_z);
	}
	else if(FS_MODE == 1)  		//����ģʽ
	{
		if(rt_Distence < 20)  Turn_Out = Turn_PD(20, mpu6050Struct.gyro_z);
		else  				  Turn_Out = 0;
	}
	else if(FS_MODE == 2)  		//Ѳ��ģʽ
	{
		Turn_Out = Turn_PD(CCD.middle, mpu6050Struct.gyro_z);
	}
	motor1_pwm = Vertical_Out + Velocity_Out + Turn_Out;
	motor2_pwm = Vertical_Out + Velocity_Out - Turn_Out;

	PWM_Limiting(&motor1_pwm,&motor2_pwm);
				
	if(Turn_off(mpu6050Struct.pitch)==0)	//�ж�С���Ƿ��������״��
	{
		Set_PWM(motor1_pwm,motor2_pwm);		//motor1_pwm,motor2_pwm��Χ�� 0-5800   ,PWM�����CCRΪ8000
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
