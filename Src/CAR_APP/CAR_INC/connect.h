#ifndef __CONNECT_H
#define __CONNECT_H

#include <stdint.h>

//命令种类，上位机发-------------------------------------------------------------------------------------
#define MOVEMENT               0X01         //小车动作
#define READ_INFO              0X02         //读取PID、模式、版本，恢复默认参数
#define SET_MODE               0X03         //设置小车工作模式
#define WRITE_PID1             0X10         //设置PID1
#define WRITE_PID2             0X11         //设置PID2

	/* MOVEMENT的具体命令 */
	#define CAR_STOP              0X00         //停止
	#define CAR_FORWARD           0X01         //前进
	#define CAR_BACK              0X02         //后退
	#define CAR_TURN_LEFT         0X03         //左转
	#define CAR_TURN_RIGHT        0X04         //右转

	/* SET_MODE的具体命令 */
	#define REMOTE_MODE           0X01        //遥控模式
	#define	LINE_TRACK_MODE       0X02        //巡线模式
	#define AVOID_MODE            0X03        //蔽障模式

	/* READ_INFO的具体命令 */
	#define READ_ALL_ARG          0X00        //读取所有的数据
	#define READ_PID              0X01        //读取PID数据
	#define	READ_WORK_MODE        0X02        //读取当前工作模式
	#define	READ_VER_INFO         0XA0        //读取版本信息
	#define	RESTORE_DEFAULT_ARG   0XA1        //恢复默认参数

//上位机的命令格式-------------------------------------------------------------------------------------

//小车上报数据的种类
#define TYPE_POST_VER								0x00					//版本信息
#define TYPE_POST_MPU6050_Angle						0x01					//姿态
#define TYPE_POST_MPU6050_RAWDATA					0x02					//传感器原始数据
#define TYPE_POST_CMD_ECHO						    0x03					//小车接收到的遥控数据
#define TYPE_POST_POWER								0x04					//小车电量
#define TYPE_POST_MOTO_ENCODER						0x05					//电机转速
#define TYPE_POST_HCSR04_DISTANCE					0x06					//超声波距离
#define TYPE_POST_MODE								0X07					//小车模式
#define TYPE_POST_PID_1								0x10					//PID1数据
#define TYPE_POST_PID_2								0x11					//PID2的数据
#define TYPE_POST_PID_3								0X12					//PID3的数据
#define TYPE_POST_CCD								0XF1					//CCD的数据 
#define TYPE_POST_User_Waveform						0xA1	

//定义版本相关信息
#define Hardware_Type           10
#define Hardware_VER            10
#define Software_VER            10
#define Protocol_VER            10

typedef struct 
{
	uint8_t send_check;
	uint8_t send_version;
	uint8_t send_status;
	uint8_t send_senser;
	uint8_t send_senser2;
	uint8_t send_pid1;
	uint8_t send_pid2;
	uint8_t send_pid3;
	uint8_t send_pid4;
	uint8_t send_pid5;
	uint8_t send_pid6;
	uint8_t send_rcdata;
	uint8_t send_offset;
	uint8_t send_motopwm;
	uint8_t send_power;
	uint8_t send_user;
	uint8_t send_speed;
	uint8_t send_location;

}dt_flag_t;

//发送数据给上位机
void Connect_Send_data(uint8_t CMD_Data);

//接收上位机的数据
uint8_t EP32_RcvData_Extract(const uint8_t *Buff,int len);


#endif

