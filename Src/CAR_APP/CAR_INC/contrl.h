#ifndef _CONTRIL_H_
#define _CONTRIL_H_

#include "sys.h"


#define Mechanical_BALANCE 0        //机械0点

#define AIN1(PinState)    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_13, (GPIO_PinState)PinState)
#define AIN2(PinState)    HAL_GPIO_WritePin( GPIOE, GPIO_PIN_15, (GPIO_PinState)PinState)

#define BIN1(PinState)    HAL_GPIO_WritePin( GPIOC, GPIO_PIN_3, (GPIO_PinState)PinState)
#define BIN2(PinState)    HAL_GPIO_WritePin( GPIOA, GPIO_PIN_3, (GPIO_PinState)PinState)

#define PWMA   TIM4->CCR1
#define PWMB   TIM5->CCR3

struct PID_t
{
    float Vertical_Kp;
    float Vertical_Ki;
    float Vertical_Kd;

    float Velocity_Kp;
    float Velocity_Ki;
    float Velocity_Kd;

    float  Turn_Kp;
    float  Turn_Ki;
    float  Turn_Kd;
};

extern struct PID_t PID;                              //定义在c文件

int Vertical_PD(float rt_Angle, float rt_Gyro);
int Velocity_PI(int rt_Encoder_Left, int rt_Encoder_Right, float rt_Angle, float target_SpeedEncoder);
int Turn_PD(u8 rt_CCD_Middle, short rt_Yaw);

int Read_Encoder(u8 TIMX);
void PWM_Limiting(int *motor1, int *motor2);
u8 Turn_off(const float Angle);
void Set_PWM(int motor1, int motor2);


#endif
