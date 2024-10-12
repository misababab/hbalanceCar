#include "inv_mpu_user.h"
#include "connect.h"
#include "string.h"
#include "stdio.h"
#include "esp32.h"
#include "car_task.h"
#include "contrl.h"

/* ���ļ�����Ҫ���ܾ��ǣ���װ���ݰ����������ݰ� */

void post_CCD(void);
void post_MOTO_Encoder(float PWM_MOTO);
static void post_VERSION(u8 HardwareType,u16 HardwareVER,u16 SoftwareVER,u16 ProtocolVER);
static void post_MODE(u8 data);
static void post_MPU6050_Angle(void);
static void post_MPU6050_RawData(void);
static void post_PID(u8 Function);

void Host_Data_Receive_Anl(u8 *data_buf,u8 num);

/**********************************************************************************************************
*����:��ȡESP32����Ч����,���ҽ��俽������������
*�β�:�Ӷ�����ȡ����һ����Ϣ��
*����ֵ:�ɹ�����0��ʧ�ܷ���1
ע�����ݸ�ʽ
							  ��ͷ  ��ͷ  ����  �غɳ���   �غ�  У��
����                          0xaa  0xaf  0x01  0x01      0x01  ����
***********************************************************************************************************/
u8 EP32_RcvData_Extract(const uint8_t *Buff,int len)
{	
	static u8 RxBuffer[50];					//����Ϣ��
	static int data_len = 0,data_cnt = 0;
	static u8 state = 0;
	u8 i, data;

	for(i = 0;i < len ;i++)
	{
		data = Buff[i];
		
		/* ����ת����Ϣ�������ݵ�ջ��RxBuffer+++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
		/* ��λ�� ��Ϣ��ͷ 0xaa  0xaf  */		
		if(state==0 && data==0xAA)
		{
			RxBuffer[0]=data;	
			state=1;
		}
		else if(state==1 && data==0xAF)
		{
			RxBuffer[1]=data;	
			state=2;
		}
		
		/* ��λ�� ��Ϣ���� < 0XF1 */
		else if(state==2 && data<0XF1)
		{
			RxBuffer[2]=data;	
			state=3;
		}
		
		/* �غ����� ���� < 50 */
		else if(state==3 && data<50)
		{
			RxBuffer[3]=data; 	
			data_len = data; 
			data_cnt = 0; 
			state = 4;
		}
		
		/* �غ����� -> RxBuffer */
		else if(state==4 && data_len>0)
		{
			data_len--;
			RxBuffer[4+data_cnt++]=data;
			if(data_len==0)
			state = 5;
		}
		
		/* �õ�У��ֵ */
		else if(state==5)
		{
			state = 0;
			RxBuffer[4+data_cnt]=data;
			Host_Data_Receive_Anl(RxBuffer,data_cnt+5);
			memset(RxBuffer,0,50);
			return 0;
		}
		/* ����ת����Ϣ�������ݵ�ջ��RxBuffer------------------------------------------------------ */
	}
	return 1;
}



/**************************************************************************************************************
*���ܣ��������ݸ���λ��
ע�����ݸ�ʽ
							  ��ͷ  ��ͷ  ����  �غɳ���   �غ�  У��
����                          0xaa  0xaa  0x01  0x01      0x01  ����
**************************************************************************************************************/
void Connect_Send_data(u8 CMD_Data)
{
	switch(CMD_Data)
	{
		case READ_ALL_ARG://DBG("\r�������ݵ���λ��\n");
			post_MPU6050_Angle();
			post_MPU6050_RawData();
			post_MOTO_Encoder(rt_EncoderRight);
			post_CCD();		
			//post_RCDATA(F_CMD,target_SpeedEncoder);
			//post_POWER(power);
			//post_HCSR04_Distance(Distance);
			//post_VERSION(Hardware_Type,Hardware_VER,Software_VER,Protocol_VER);
			//post_MODE(FS_MODE);
			break;
		
		case READ_PID:   			DBG("\r����PID����\n");     			 
			post_PID(1);
			break;
		
		case READ_WORK_MODE:		DBG("\r���͵�ǰģʽ\n");  
			post_MODE(FS_MODE);
			break;
		
		case READ_VER_INFO: 		DBG("\r���Ͱ汾��Ϣ\n");  
			post_VERSION(Hardware_Type,Hardware_VER,Software_VER,Protocol_VER);	
			break;
		
		default:
			break;
	}

}

/**************************************************************************************************************
*���ܣ���Ӧ��λ������������
*�β�:		(u8 *data_buf):������Ϣ��        (u8 num):��Ϣ������
ע�����ݸ�ʽ
							  ��ͷ  ��ͷ  ����  �غɳ���   �غ�  У��
����                          0xaa  0xaf  0x01  0x01      0x01  ����
**************************************************************************************************************/
void Host_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0,i;
	u8 function = *(data_buf+2);		//�õ���Ϣ����
	u8 cmd = *(data_buf+4);				//�õ��غ�
	
	DBG("�յ���Ч���ݣ�");									
	for(i=0;i<(num-1);i++)// ����У�� 
	{		
		DBG("%x  ",*(data_buf+i));
		sum += *(data_buf+i);
	}
	DBG("\r\n");
	
	/* У�� */
	if(sum != *(data_buf+num-1) )	return;						/* У�鲻��ֱ�ӷ��� */
	/* ������Ϣ��ͷ */
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))	return;		/* �ж�֡ͷ */

	switch(function)
	{
		case MOVEMENT: 	DBG("����С������\n");
			switch(cmd)
			{
				case CAR_STOP:	    DBG("С��ֹͣ\n");if(FS_MODE==0){rt_TrunAngle = 64;target_SpeedEncoder = 0;}break;
				case CAR_FORWARD:   DBG("С��ǰ��\n");if(FS_MODE==0){rt_TrunAngle = 64;target_SpeedEncoder = 50;}break;
				case CAR_BACK:      DBG("С������\n");if(FS_MODE==0){rt_TrunAngle = 64;target_SpeedEncoder = -50;}break;
				case CAR_TURN_LEFT: DBG("С����ת\n");if(FS_MODE==0){rt_TrunAngle = 30;target_SpeedEncoder = 0;}break;
				case CAR_TURN_RIGHT:DBG("С����ת\n");if(FS_MODE==0){rt_TrunAngle = 98;target_SpeedEncoder = 0;}break;
				default:break;
			}
			break;
			
		case READ_INFO: DBG("��ȡС��������Ϣ\n");
			switch(cmd)
			{
				case READ_PID:		DBG("��ȡPID����\n");	break;//
				case READ_WORK_MODE:DBG("��ȡ��ǰ����ģʽ\n");break;//
				case READ_VER_INFO: DBG("��ȡ�汾��Ϣ\n");	break;//
				case RESTORE_DEFAULT_ARG:
					DBG("�ָ�Ĭ�ϲ���\n");
					PID.Vertical_Kp=200;
					PID.Vertical_Kd=1;
					PID.Velocity_Kp=-60;
					PID.Velocity_Ki=-0.3;
					PID.Turn_Kp = 18;
					PID.Turn_Kd = 0.18;
					break;			
				default:
					break;
			}	
			break;
		case SET_MODE: DBG("����С������ģʽ\n");
			switch(cmd)
			{
				case REMOTE_MODE:    DBG("ң��ģʽ\n");    FS_MODE = 0;target_SpeedEncoder = 0; break;
				case AVOID_MODE:     DBG("����ģʽ\n");    FS_MODE = 1;target_SpeedEncoder = 10;break;
			    case LINE_TRACK_MODE:DBG("Ѳ��ģʽ\n");    FS_MODE = 2;target_SpeedEncoder = 10;break;
				default:break;
			}		
			break;
			
		case WRITE_PID1: DBG("����С��PID1\n");//��λ������9����16λ����*100���͹�����,�ܹ���Ҫ18���ֽ�
		
			PID.Vertical_Kp  = 0.01*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
			PID.Vertical_Ki  = 0.01*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
			PID.Vertical_Kd  = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );

			PID.Velocity_Kp = 0.01*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
			PID.Velocity_Ki = 0.01*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
			PID.Velocity_Kd = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );

			PID.Turn_Kp 	= 0.01*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
			PID.Turn_Ki 	= 0.01*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
			PID.Turn_Kd	    = 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
			break;
			
		case WRITE_PID2: DBG("����С��PID2\n");
			break;
		
		default:
			break;
	}
	
}

/**************************************************************************************************************
*����:���֡ͷ�����ּ�У��λ
*�β�:(u8 fun):��Ϣ����/			(u8*data):�غ�/       (u8 len):�غɳ���
*˵����fun��ɸѡ�������࣬data��ɸѡ��������
**************************************************************************************************************/
static void package_report_data(u8 fun,u8 * data,u8 len)
{
    static u8 send_buf[40] = {0};   //���static����ջ��Сһ��ѹ��
	u16 check_sum=0;				//���У��ֵ
    u8 i;
	
	/* ��װ�ϴ����ݰ�+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
    if(len>28)return;   
    send_buf[0]=0XAA;  					/* ��װ ��Ϣ��ͷ 0XAA 0XAA */
	send_buf[1]=0XAA;
	
    send_buf[2]=fun;					/* ��װ ��Ϣ���� */
    send_buf[3]=len; 					/* ��װ �غ����ݳ���len */

    for(i=0;i<len;i++)send_buf[4+i]=data[i];	/* ��װ �غ����� */

    for(i=0;i<len+4;i++)						/* ��װ У�� */
	check_sum += send_buf[i];
	send_buf[len+4]=((check_sum)&0xFF);
	/* ��װ�ϴ����ݰ�+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
	
	/* ����װ�õ��û���Ϣ���͸�ESP32 */
	ESP32_Send_AppLayerData(send_buf,len+5);
}

/*********************************************************************************************************
*���ܣ����Ͱ汾��Ϣ
*�β�:HardwareType,HardwareVER,SoftwareVER,ProtocolVER
*********************************************************************************************************/
static void post_VERSION(u8 HardwareType,u16 HardwareVER,u16 SoftwareVER,u16 ProtocolVER)
{
		static u8 tempbuf[7];   
		tempbuf[0]=(HardwareType)&0XFF;
	
		tempbuf[1]=((HardwareVER)>>8)&0XFF;
		tempbuf[2]=(HardwareVER)&0XFF;
	
		tempbuf[3]=((SoftwareVER)>>8)&0XFF;
		tempbuf[4]=(SoftwareVER)&0XFF;
	
		tempbuf[5]=((ProtocolVER)>>8)&0XFF;
		tempbuf[6]=(ProtocolVER)&0XFF;
		package_report_data(TYPE_POST_VER,tempbuf,7);
}

/*************************************************************************************************************
*����:���͵�ǰģʽ
*�β�:dataģʽ
*************************************************************************************************************/
static void post_MODE(u8 data)
{
	u8 tempbuf[1];
	tempbuf[0]=(data)&0XFF;  
	package_report_data(TYPE_POST_MODE,tempbuf,1);
}
/**********************************************************************************************************
*����:������̬
*�β�:angle(x����,y���,zƫ��)
***********************************************************************************************************/
static void post_MPU6050_Angle(void)/* float  */
{
	static u8 tempbuf[6]; 
	int32_t anglex,angley,anglez;
	anglex=((short)(mpu6050Struct.pitch))*100;
	angley=((short)(mpu6050Struct.roll))*100;
	anglez=((short)(mpu6050Struct.yaw))*100;
	
	tempbuf[0]=((anglex)>>8)&0XFF;      
	tempbuf[1]=(anglex)&0XFF;
	
	tempbuf[2]=((angley)>>8)&0XFF;
	tempbuf[3]=(angley)&0XFF;
	
	tempbuf[4]=((anglez)>>8)&0XFF;
	tempbuf[5]=(anglez)&0XFF;
	
	package_report_data(TYPE_POST_MPU6050_Angle,tempbuf,6);
}

/**************************************************************************************************************
*����:���ʹ�����ԭ����
*�β�:acc:������,gyro:���ٶȼ�,mag:��������
**************************************************************************************************************/
static void post_MPU6050_RawData(void)
{
	static u8 tempbuf[18];    
	u16 accx,accy,accz,gyrox,gyroy,gyroz;//magx,magy,magz
	
	accx=(u16)((mpu6050Struct.acc_x)*100);
	accy=(u16)((mpu6050Struct.acc_y)*100);
	accz=(u16)((mpu6050Struct.acc_z)*100);
	
	gyrox=(u16)((mpu6050Struct.gyro_x)*100);
	gyroy=(u16)((mpu6050Struct.gyro_y)*100);
	gyroz=(u16)((mpu6050Struct.gyro_z)*100);
	
//	magx=(u8)((mag->x)*100);
//	magy=(u8)((mag->y)*100);
//	magz=(u8)((mag->z)*100);
	
	tempbuf[0]=((accx)>>8)&0XFF;      
	tempbuf[1]=(accx)&0XFF;
	tempbuf[2]=((accy)>>8)&0XFF;
	tempbuf[3]=(accy)&0XFF;
	tempbuf[4]=((accz)>>8)&0XFF;
	tempbuf[5]=(accz)&0XFF;
	
	tempbuf[6]=((gyrox)>>8)&0XFF;      
	tempbuf[7]=(gyrox)&0XFF;
	tempbuf[8]=((gyroy)>>8)&0XFF;
	tempbuf[9]=(gyroy)&0XFF;
	tempbuf[10]=((gyroz)>>8)&0XFF;
	tempbuf[11]=(gyroz)&0XFF;
	
//	tempbuf[12]=((magx)>>8)&0XFF;      
//	tempbuf[13]=(magx)&0XFF;
//	tempbuf[14]=((magy)>>8)&0XFF;
//	tempbuf[15]=(magy)&0XFF;
//	tempbuf[16]=((magz)>>8)&0XFF;
//	tempbuf[17]=(magz)&0XFF;
	package_report_data(TYPE_POST_MPU6050_RAWDATA,tempbuf,18);
}
/*************************************************************************************************************
*����:����PID����
*�β�:PID1��PID2��PID3�Ĳ���
**************************************************************************************************************/
static void post_PID(u8 Function)
{
  static u8 tempbuf[18];
	int16_t	PID1_P,PID1_I,PID1_D,PID2_P,PID2_I,PID2_D,PID3_P,PID3_I,PID3_D;
	
	PID1_P=(u16)((PID.Vertical_Kp)*100);
	PID1_I=(u16)((PID.Vertical_Ki)*100);
	PID1_D=(u16)((PID.Vertical_Kd)*100);
	
	PID2_P=(u16)((PID.Velocity_Kp)*100);
	PID2_I=(u16)((PID.Velocity_Ki)*100);
	PID2_D=(u16)((PID.Velocity_Kd)*100);
	
	PID3_P=(u16)((PID.Turn_Kp)*100);
	PID3_I=(u16)((PID.Turn_Ki)*100);
	PID3_D=(u16)((PID.Turn_Kd)*100);
	
	tempbuf[0]=((PID1_P)>>8)&0XFF;      
	tempbuf[1]= (PID1_P)&0XFF;
	tempbuf[2]=((PID1_I)>>8)&0XFF;
	tempbuf[3]= (PID1_I)&0XFF;
	tempbuf[4]=((PID1_D)>>8)&0XFF;
	tempbuf[5]= (PID1_D)&0XFF;
		
	tempbuf[6]= ((PID2_P)>>8)&0XFF;      
	tempbuf[7]=  (PID2_P)&0XFF;
	tempbuf[8]= ((PID2_I)>>8)&0XFF;
	tempbuf[9]=  (PID2_I)&0XFF;
	tempbuf[10]=((PID2_D)>>8)&0XFF;
	tempbuf[11]= (PID2_D)&0XFF;
	
	tempbuf[12]=((PID3_P)>>8)&0XFF;      
	tempbuf[13]= (PID3_P)&0XFF;
	tempbuf[14]=((PID3_I)>>8)&0XFF;
	tempbuf[15]= (PID3_I)&0XFF;
	tempbuf[16]=((PID3_D)>>8)&0XFF;
	tempbuf[17]= (PID3_D)&0XFF;
	if(Function==1)
		package_report_data(TYPE_POST_PID_1,tempbuf,18);
	if(Function==2)
		package_report_data(TYPE_POST_PID_2,tempbuf,18);
	if(Function==3)
		package_report_data(TYPE_POST_PID_3,tempbuf,18);
}

/**************************************************************************************************************
*����:����CCD����
*�β�:ccd:CCD���ݰ���(��ֵ,��ֵ,������,������)
***************************************************************************************************************/
void post_CCD(void)
{
	static u8 tempbuf[8];
	u16 CCD_MIDDLE,CCD_THRESHOLD,CCD_LEFT,CCD_RIGHT;
	
	CCD_MIDDLE = CCD.middle;
	CCD_THRESHOLD = CCD.threshold;
	CCD_LEFT = CCD.left;
	CCD_RIGHT = CCD.right;
	
	tempbuf[0]=((CCD_MIDDLE)>>8)&0XFF;
	tempbuf[1]=(CCD_MIDDLE)&0XFF;
	
	tempbuf[2]=((CCD_THRESHOLD)>>8)&0XFF;
	tempbuf[3]=(CCD_THRESHOLD)&0XFF;
	
	tempbuf[4]=((CCD_LEFT)>>8)&0XFF;
	tempbuf[5]=(CCD_LEFT)&0XFF;
	
	tempbuf[6]=((CCD_RIGHT)>>8)&0XFF;
	tempbuf[7]=(CCD_RIGHT)&0XFF;
	
	package_report_data(TYPE_POST_CCD,tempbuf,8);
}

/*************************************************************************************************************
*����:���͵���
*�β�:data:����(0%,25%,50%,75%,100%)
**************************************************************************************************************/
void post_POWER(u16 data)
{
	static u8 tempbuf[2];
	tempbuf[0]=((data)>>8)&0XFF;      
	tempbuf[1]=(data)&0XFF;
	package_report_data(TYPE_POST_POWER,tempbuf,2);
}
/*************************************************************************************************************
*����:���͵��ת��
*�β�:PWM_MOTO:���ת��
**************************************************************************************************************/
void post_MOTO_Encoder(float PWM_MOTO)	/* int16_t rt_EncoderRight */
{
	u16 PWM_Percentage;
	static u8 tempbuf[2];
	
	PWM_Percentage = fabs(PWM_MOTO)*1.24;
	
	tempbuf[0]=((PWM_Percentage)>>8)&0XFF;      
	tempbuf[1]=(PWM_Percentage)&0XFF;
	package_report_data(TYPE_POST_MOTO_ENCODER,tempbuf,2);
}

