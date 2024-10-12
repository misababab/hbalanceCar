#include "math.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "contrl.h"

int   Dead_Zone=1200;     //电机死区

struct PID_t PID = 		  //定义了一个PID 结构体，调节参数
{
	.Vertical_Kp= 200,
	.Vertical_Kd=1,
	
	.Velocity_Kp= -56,    
	.Velocity_Ki= -0.28,
	
	.Turn_Kp = 18,
	.Turn_Kd = 0.18,
};

/**************************************************************************************************************
*功能:直立环PD控制
*形参:(float Angle):x轴的角度/(float Gyro):x轴的角速度
*返回值:经过PID转换之后的PWM值
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
*功能；速度环PI控制
*形参:(int encoder_left):左轮编码器值/(int encoder_right):编码器右轮的值/(float Angle):x轴角度值
**************************************************************************************************************/

int Velocity_PI(int rt_Enc_Left,int rt_Enc_Right,float rt_Angle,float Target_SpeedEncoder )
{
	static float Output, Encoder_err, Encode_LPF_err;
	static float Encoder_acc;
	
	Encoder_err = (rt_Enc_Left + rt_Enc_Right) - 0;    //获取最新速度偏差=测量速度（左右编码器之和）-目标速度（此处为零）
	Encode_LPF_err *= 0.8f;								  	//一阶低通滤波器 ，上次的速度占85%
	Encode_LPF_err += Encoder_err * 0.2f;                   //一阶低通滤波器， 本次的速度占15% 
	Encoder_acc += Encode_LPF_err;                       	//积分出位移 积分时间：10ms
	Encoder_acc = Encoder_acc - Target_SpeedEncoder; 

	if(Encode_LPF_err>10000)    		Encoder_acc = 10000;             //积分限幅
	if(Encoder_acc<-10000)	    		Encoder_acc = -10000;            //积分限幅acc
	
	Output = Encode_LPF_err * (PID.Velocity_Kp) + Encoder_acc * (PID.Velocity_Ki);      //速度控制

	if(Turn_off(rt_Angle)==1)           Encoder_acc=0;            //电机关闭后清除积分
	return Output;
}


/**************************************************************************************************************
*功能:转向环PD
*形参:无  CCD小于64左转、CCD大于64右转。 yaw = z轴陀螺仪数值
*返回值:无
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
*功能:读取编码器值(当作小车当前前进的速度)
*形参:(u8 TIMX):x为编码器1或者2
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
*功能:PWM限幅函数
PWM的范围是-5800-5800
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
*功能:关闭电机
*形参:(const float Angle):x轴角度值
*返回值:1:小车当前处于停止状态/0:小车当前处于正常状态
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
*功能:输出PWM控制电机
*形参；(int motor1):电机1对应的PWM值/(int motor2):电机2对应的PWM值
PWM的范围是0-5800
*************************************************************************************************************/
void Set_PWM(int motor1,int motor2)
{
	if(motor1>0)			AIN2(1),			AIN1(0);//正转
	else 	          		AIN2(0),			AIN1(1);//反转
	PWMA = Dead_Zone + (abs(motor1))*1.17;
	
	if(motor2>0)			BIN1(1),			BIN2(0);//正转
	else       		 		BIN1(0),			BIN2(1);//反转
	PWMB = Dead_Zone + (abs(motor2))*1.17;	
	
}



