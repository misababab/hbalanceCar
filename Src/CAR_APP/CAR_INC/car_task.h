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

	float pitch;    //������
	float roll;     //������
	float yaw;      //ƫ����
};

struct CCD_t
{
	uint16_t middle;      //�м�λ��ֵ
	uint16_t threshold;   //����ad��ֵ
	uint16_t left;        //�������λ��
	uint16_t right;       //�������λ��
};

extern int  FS_MODE  ;                      		//0��ң��ģʽ   1������ģʽ  2��Ѳ��ģʽ 

extern struct mpu6050_data mpu6050Struct;			//����mpu6050����������
/* ת�� */
extern struct CCD_t  CCD;
extern int  rt_TrunAngle ;              //ң��ģʽ������ת����ڱ���
/* �ٶȻ� */
extern int  rt_EncoderLeft, rt_EncoderRight;     	//����ٶ�
extern float target_SpeedEncoder ;                  //�ٶȵ��� 
/* ���� */
extern int  rt_Distence ;                      		//С����ǰ���ϰ���֮��ľ���
extern uint8_t   rt_Power;                      		//�����ص���



//extern int  Balance_Pwm,Velocity_Out,Turn_Out;      //PID�����PWMֵ
//extern int  motor1_pwm, motor2_pwm;                 //���ҵ��PWMֵ




void Get_MPU6050_Data_5ms(void);
void Pid_Control_10ms(void);
void Oled_Show_Menu_200ms(void);

void  HC_SRC04_Start(void);



#endif
