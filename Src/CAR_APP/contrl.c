#include "math.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "contrl.h"

int   Dead_Zone=1200;     //�������

struct PID_t PID = 		  //������һ��PID �ṹ�壬���ڲ���
{
	.Vertical_Kp= 200,
	.Vertical_Kd=1,
	
	.Velocity_Kp= -56,    
	.Velocity_Ki= -0.28,
	
	.Turn_Kp = 18,
	.Turn_Kd = 0.18,
};

/**************************************************************************************************************
*����:ֱ����PD����
*�β�:(float Angle):x��ĽǶ�/(float Gyro):x��Ľ��ٶ�
*����ֵ:����PIDת��֮���PWMֵ
**************************************************************************************************************/
int	Vertical_PD(float rt_Angle,float rt_Gyro)
{
	float err;
	int Output;
	err = rt_Angle - Mechanical_BALANCE;
	Output = err * ( PID.Vertical_Kp) + rt_Gyro * (PID.Vertical_Kd);

	return Output;

}

/**************************************************************************************************************
*���ܣ��ٶȻ�PI����
*�β�:(int encoder_left):���ֱ�����ֵ/(int encoder_right):���������ֵ�ֵ/(float Angle):x��Ƕ�ֵ
**************************************************************************************************************/

int Velocity_PI(int rt_Enc_Left,int rt_Enc_Right,float rt_Angle,float Target_SpeedEncoder )
{
	static float Output, Encoder_err, Encode_LPF_err;
	static float Encoder_acc;
	
	Encoder_err = (rt_Enc_Left + rt_Enc_Right) - 0;    //��ȡ�����ٶ�ƫ��=�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩
	Encode_LPF_err *= 0.8f;								  	//һ�׵�ͨ�˲��� ���ϴε��ٶ�ռ85%
	Encode_LPF_err += Encoder_err * 0.2f;                   //һ�׵�ͨ�˲����� ���ε��ٶ�ռ15% 
	Encoder_acc += Encode_LPF_err;                       	//���ֳ�λ�� ����ʱ�䣺10ms
	Encoder_acc = Encoder_acc - Target_SpeedEncoder; 

	if(Encode_LPF_err>10000)    		Encoder_acc = 10000;             //�����޷�
	if(Encoder_acc<-10000)	    		Encoder_acc = -10000;            //�����޷�acc
	
	Output = Encode_LPF_err * (PID.Velocity_Kp) + Encoder_acc * (PID.Velocity_Ki);      //�ٶȿ���

	if(Turn_off(rt_Angle)==1)           Encoder_acc=0;            //����رպ��������
	return Output;
}


/**************************************************************************************************************
*����:ת��PD
*�β�:��  CCDС��64��ת��CCD����64��ת�� yaw = z����������ֵ
*����ֵ:��
***************************************************************************************************************/
int Turn_PD(u8 rt_CCD_Middle,short rt_Yaw)
{
	float Output;     
	float err;	  
	err = rt_CCD_Middle - 64;
	Output  = -err * (PID.Turn_Kp) - rt_Yaw * (PID.Turn_Kd);
	return Output;
}

/**************************************************************************************************************
*����:��ȡ������ֵ(����С����ǰǰ�����ٶ�)
*�β�:(u8 TIMX):xΪ������1����2
*************************************************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int TIM_Counter;  
		
	switch(TIMX)
	{
		case 1:   TIM_Counter = (short)TIM1->CNT;  TIM1->CNT=0;break;
		case 2:   TIM_Counter = (short)TIM2->CNT;  TIM2->CNT=0;break;
		default:  TIM_Counter =0;
	}
	return TIM_Counter;
}


/**************************************************************************************************************
*����:PWM�޷�����
PWM�ķ�Χ��-5800-5800
***************************************************************************************************************/
void PWM_Limiting(int *motor1,int *motor2)
{
	int Amplitude=5800;
	
	if(*motor1<-Amplitude) *motor1=-Amplitude;	
	if(*motor1>Amplitude)  *motor1=Amplitude;
	
	if(*motor2<-Amplitude) *motor2=-Amplitude;	
	if(*motor2>Amplitude)  *motor2=Amplitude;		
}


/**************************************************************************************************************
*����:�رյ��
*�β�:(const float Angle):x��Ƕ�ֵ
*����ֵ:1:С����ǰ����ֹͣ״̬/0:С����ǰ��������״̬
***************************************************************************************************************/
u8 FS_state;

u8 Turn_off(const float Angle)
{
	u8 temp;
	if(fabs(Angle)>80)
	{
		FS_state=1;
		temp=1;
		AIN2(0),AIN1(0);
		BIN1(0),BIN2(0);
	}
	else temp=0;
		FS_state=0;
	return temp;
}

/**************************************************************************************************************
*����:���PWM���Ƶ��
*�βΣ�(int motor1):���1��Ӧ��PWMֵ/(int motor2):���2��Ӧ��PWMֵ
PWM�ķ�Χ��0-5800
*************************************************************************************************************/
void Set_PWM(int motor1,int motor2)
{
	if(motor1>0)			AIN2(1),			AIN1(0);//��ת
	else 	          		AIN2(0),			AIN1(1);//��ת
	PWMA = Dead_Zone + (abs(motor1))*1.17;
	
	if(motor2>0)			BIN1(1),			BIN2(0);//��ת
	else       		 		BIN1(0),			BIN2(1);//��ת
	PWMB = Dead_Zone + (abs(motor2))*1.17;	
	
}



