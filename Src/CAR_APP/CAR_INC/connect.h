#ifndef __CONNECT_H
#define __CONNECT_H

#include <stdint.h>

//�������࣬��λ����-------------------------------------------------------------------------------------
#define MOVEMENT               0X01         //С������
#define READ_INFO              0X02         //��ȡPID��ģʽ���汾���ָ�Ĭ�ϲ���
#define SET_MODE               0X03         //����С������ģʽ
#define WRITE_PID1             0X10         //����PID1
#define WRITE_PID2             0X11         //����PID2

	/* MOVEMENT�ľ������� */
	#define CAR_STOP              0X00         //ֹͣ
	#define CAR_FORWARD           0X01         //ǰ��
	#define CAR_BACK              0X02         //����
	#define CAR_TURN_LEFT         0X03         //��ת
	#define CAR_TURN_RIGHT        0X04         //��ת

	/* SET_MODE�ľ������� */
	#define REMOTE_MODE           0X01        //ң��ģʽ
	#define	LINE_TRACK_MODE       0X02        //Ѳ��ģʽ
	#define AVOID_MODE            0X03        //����ģʽ

	/* READ_INFO�ľ������� */
	#define READ_ALL_ARG          0X00        //��ȡ���е�����
	#define READ_PID              0X01        //��ȡPID����
	#define	READ_WORK_MODE        0X02        //��ȡ��ǰ����ģʽ
	#define	READ_VER_INFO         0XA0        //��ȡ�汾��Ϣ
	#define	RESTORE_DEFAULT_ARG   0XA1        //�ָ�Ĭ�ϲ���

//��λ���������ʽ-------------------------------------------------------------------------------------

//С���ϱ����ݵ�����
#define TYPE_POST_VER								0x00					//�汾��Ϣ
#define TYPE_POST_MPU6050_Angle						0x01					//��̬
#define TYPE_POST_MPU6050_RAWDATA					0x02					//������ԭʼ����
#define TYPE_POST_CMD_ECHO						    0x03					//С�����յ���ң������
#define TYPE_POST_POWER								0x04					//С������
#define TYPE_POST_MOTO_ENCODER						0x05					//���ת��
#define TYPE_POST_HCSR04_DISTANCE					0x06					//����������
#define TYPE_POST_MODE								0X07					//С��ģʽ
#define TYPE_POST_PID_1								0x10					//PID1����
#define TYPE_POST_PID_2								0x11					//PID2������
#define TYPE_POST_PID_3								0X12					//PID3������
#define TYPE_POST_CCD								0XF1					//CCD������ 
#define TYPE_POST_User_Waveform						0xA1	

//����汾�����Ϣ
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

//�������ݸ���λ��
void Connect_Send_data(uint8_t CMD_Data);

//������λ��������
uint8_t EP32_RcvData_Extract(const uint8_t *Buff,int len);


#endif

