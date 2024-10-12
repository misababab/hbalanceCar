#include "oled_show.h"
#include "connect.h"
#include "car_task.h"
#include "stm32f4xx_hal.h"
#include "oled.h"
#include "inv_mpu_user.h"
#include "esp32.h"
#include "bmp.h"
#include "contrl.h"
#include "inv_mpu.h"


static void Submenu_Display_Parameter(void);
static void Submenu_Set_WorkMode(void);
static void Submenu_Set_Speed(void);
static void Submenu_Set_NetMode(void);

extern struct PID_t PID;
float mechanical_zero;          //��е0��
struct _PARMETER    PARMETER;   //����PID�ַ����ṹ�����

u8 key;
static int Location=0;                            		//��¼�˵�ѡ���λ��
static int Enter_flag=0, exit_flag=0, Write_flag=0; 	//�����¼���־λ

/*************************************************************************************************************
*����:������ʾ
*�βΣ�angle(�Ƕ�ֵ),LEFT\RIGHT(�������ٶ�ֵ),distance(��������ֵ),ccd(CCD��ֵ)
**************************************************************************************************************/
void oled_Show(void)
{
    OLED_ShowFloat(36, 3, mpu6050Struct.pitch, 2, 6, 1);   		//��ʾpitch(����)��ֵ
    OLED_ShowFloat(30, 4, mpu6050Struct.roll, 2, 6, 1);    		//��ʾroll(����)��ֵ
    OLED_ShowFloat(24, 5, mpu6050Struct.yaw, 2, 6, 1);   		 //��ʾyaw(ƫ��)��ֵ
    OLED_ShowFloat(80, 4, (rt_Distence*0.58), 2, 6, 1);  	 	 //������������ֵ��ʾ
    OLED_ShowFloat(102, 5, (float)rt_EncoderRight, 3, 6, 0);    //��ʾС�������ٶ�ֵ
	
    if ((WIFI_CONNECT_FLAG!=0)||(BLE_CONNECT_FLAG!=0))    //��ʾ��ǰWIFI����״̬
        OLED_DrawBMP(90, 0, 101, 1, BMP8); //����
    else
        OLED_DrawBMP(90, 0, 101, 1, BMP9); //�Ͽ�
	
    Show_Power();   //��ʾ����

    key = Key_Scan(0); /* ɨ�谴�� */

    if (key==KEY_CENTER)	
    {
        OLED_Clear();//����

        AIN2(0),            AIN1(0);        //С����ͣ��ת
        BIN1(0),            BIN2(0);        //С������ͣת
		
        while (1)		
        {
            key=Key_Scan(0);					/* ɨ�谴�� */
            Process_Key_Input(key);				/* ���������¼� */

            if (exit_flag==1) //�ȴ��˳������¼�
            {
                OLED_Clear();//����
                exit_flag=0;
                break;
            }
            MainMenu_Show();  /* һ���˵�or�����˵� */
        }
        StaticPage();    //�˳�while֮��Ҫ��ʾһ�ξ�̬����
    }
}

/*************************************************************************************************************
*����:����������
*�β�:(u8 mode):����ģʽ(0:����/1:����)
*����ֵ:1:����1/2:����2/3:����3/4:����4/5:����5
**************************************************************************************************************/
u8 Key_Scan(u8 mode)
{
    static u8 key_up=1;    //�����ɿ���־

    if (mode) key_up=1;    //������������£���ô������״̬��һֱ��Ч

    if (key_up&&(KEY0==0||KEY1==0||KEY2==0||KEY3==0||KEY4==0))
    {
        HAL_Delay(10);          //��ʱȥ����
        key_up=0;

        if (KEY0==0)       return 1;
        else if (KEY1==0)  return 2;
        else if (KEY2==0)  return 3;
        else if (KEY3==0)  return 4;
        else if (KEY4==0)  return 5;
    }
    else if (KEY0==1&&KEY1==1&&KEY2==1&&KEY3==1&&KEY4==1)
        key_up=1;

    return 0;
}

/*************************************************************************************************************
*����:��ȡ������Ϣ
*�β�:(u8 Keynum)
**************************************************************************************************************/
void Process_Key_Input(u8 Keynum)
{
    if (key)
    {
        switch (key)
        {
            case KEY_LEFT:	exit_flag=1;	break;	//��	�˳�
            case KEY_UP:	Location--;		break;	//��
            case KEY_DOWN:	Location++;		break;	//��
            case KEY_CENTER:Write_flag=1;	break;	//����	���в˵�/ȷ���޸�
            case KEY_RIGHT:	Enter_flag=1;	break;	//��	����
        }
    }
}

/**********************************************************************************************
*����:�����˵�
*����ֵ:��
*�β�:��
***********************************************************************************************/
void MainMenu_Show(void)
{
    switch (Location)
    {
        case 0: Submenu_Display_Parameter();break;     //������ʾ
        case 1: Submenu_Set_WorkMode();     break;      //ģʽ����
        case 2: Submenu_Set_Speed();		break;      //�ٶ��趨
        case 3: Submenu_Set_NetMode();		break;       //ͨ������
        default:
            if (Location>=4) Location=0;
            if (Location<=-1) Location=3;
            break;
    } 

}

/***************************************************************************************************************
*����:������ʾ�Ӳ˵�
***************************************************************************************************************/

static void Submenu_Display_Parameter(void)
{
    if (Enter_flag==0)
    {
        OLED_ShowC_NMKC(10, 0, 1, 1); //"������ʾ"************
        OLED_ShowC_NMKC(10, 1, 2, 0); //"ģʽ����"
        OLED_ShowC_NMKC(10, 2, 3, 0); //"�ٶ��趨"
        OLED_ShowC_NMKC(10, 3, 4, 0); //"ͨ������"
    }

    if (Enter_flag==1)
    {
        OLED_Clear();//����
       
        mechanical_zero=Mechanical_BALANCE;//��ȡ��е0��
        sprintf(PARMETER.upright_PD, "Stand P:%.0f D:%.1f", PID.Vertical_Kp, PID.Vertical_Kd);
        sprintf(PARMETER.speed_PI, "Speed P:%.0f D:%.2f", PID.Velocity_Kp, PID.Velocity_Ki);
        sprintf(PARMETER.turn_PD, "Turn P:%.0f D:%.2f", PID.Turn_Kp, PID.Turn_Kd);
        sprintf(PARMETER.Mechanical_Zero, "Mechanical:%.2f", mechanical_zero);
        OLED_ShowString(0, 0, PARMETER.upright_PD, 6); 		//ֱ��PDֵ
        OLED_ShowString(0, 2, PARMETER.speed_PI, 6);     	//�ٶ�PIֵ      
        OLED_ShowString(0, 4, PARMETER.turn_PD, 6); 		//ת��PDֵ      
        OLED_ShowString(0, 6, PARMETER.Mechanical_Zero, 6); //��ʾ��еƽ���ֵ

        while (1)
        {
            key=Key_Scan(0);			/* ��ȡ���� */
            Process_Key_Input(key);		/* ���������¼� */

            //�ȴ������˳��¼�
            if (exit_flag==1)
            {
                OLED_Clear();//����
                Enter_flag=0;
                exit_flag=0;
                break;
            }
        }
    }
}

/***************************************************************************************************************
*����:����ģʽ�����Ӳ˵�
***************************************************************************************************************/
static void Submenu_Set_WorkMode(void)
{
    if (Enter_flag==0)
    {
        OLED_ShowC_NMKC(10, 0, 1, 0); //"������ʾ"
        OLED_ShowC_NMKC(10, 1, 2, 1); //"ģʽ����"		//Location=1******************
        OLED_ShowC_NMKC(10, 2, 3, 0); //"�ٶ��趨"
        OLED_ShowC_NMKC(10, 3, 4, 0); //"ͨ������"
    }

    if (Enter_flag==1)
    {
        OLED_Clear();
        Location=0;

        while (1)
        {
            key=Key_Scan(0);		//��ⰴ��
            Process_Key_Input(key); //���������¼�

            if (Location>=3)     Location=0;
            if (Location<=-1)    Location=2;

            if (Location==0)
            {
                OLED_ShowC_NMKC(10, 0, 9, 1); //"ң��ģʽ"
                OLED_ShowC_NMKC(10, 1, 11, 0); //"����ģʽ"
                OLED_ShowC_NMKC(10, 2, 10, 0); //"Ѳ��ģʽ"
            }

            if (Location==1)
            {
                OLED_ShowC_NMKC(10, 0, 9, 0); //"ң��ģʽ"
                OLED_ShowC_NMKC(10, 1, 11, 1); //"����ģʽ"
                OLED_ShowC_NMKC(10, 2, 10, 0); //"Ѳ��ģʽ"
            }

            if (Location==2)
            {
                OLED_ShowC_NMKC(10, 0, 9, 0); //"ң��ģʽ"
                OLED_ShowC_NMKC(10, 1, 11, 0); //"����ģʽ"
                OLED_ShowC_NMKC(10, 2, 10, 1); //"Ѳ��ģʽ"
            }

            //��⵽д�밴������
            if (Write_flag==1)
            {
                Write_flag=0;

                FS_MODE = Location;//ģʽ�л�,����Ǹ��ٶ�Ҳ��ʼ��
                BUZZ=1;delay_ms(200);BUZZ=0;//������������
            }

            //��⵽�˳���������
            if (exit_flag==1)
            {
                OLED_Clear();	//����
                Location=0;		//����һ���˵���ָ��ָ���һ��
                
				Enter_flag=0;
                exit_flag=0;
                
				break;
            }
        }
    }
}


/***************************************************************************************************************
*����:�ٶ������Ӳ˵�
*�β�:��
*����ֵ:��
***************************************************************************************************************/
static void Submenu_Set_Speed(void)
{
    if (Enter_flag==0)
    {
        OLED_ShowC_NMKC(10, 0, 1, 0); //"������ʾ"
        OLED_ShowC_NMKC(10, 1, 2, 0); //"ģʽ����"
        OLED_ShowC_NMKC(10, 2, 3, 1); //"�ٶ��趨"		//Location=2******************
        OLED_ShowC_NMKC(10, 3, 4, 0); //"ͨ������"
    }

    if (Enter_flag==1)
    {
        OLED_Clear();//��������
        OLED_ShowC_NMKC(10, 0, 7, 0); //"�ٶ�ֵ"
        while (1)
        {
            key=Key_Scan(1);//������ȡ����
            Process_Key_Input(key);//���������¼�

            if (key==KEY_UP)
			{
                delay_ms(50);    			//�ٶ�ֵ+
                target_SpeedEncoder++;
            }

            if ((key==KEY_DOWN) && (target_SpeedEncoder>=1))
            {
                target_SpeedEncoder--;     //�ٶ�ֵ-
                delay_ms(50);
            }

            OLED_ShowNum(80, 0, target_SpeedEncoder, 3, 8);

            if (Write_flag==1)
            {
                Write_flag=0;    //��FLASH����������������
                BUZZ=1;
                delay_ms(200);
                BUZZ=0;
            }

            if (exit_flag==1)
            {
                OLED_Clear();    //����
                Enter_flag=0;
                exit_flag=0;
                break;
            }
        }
    }
}

/***************************************************************************************************************
*����:�ٶ���������ģʽ
*�β�:��
*����ֵ:��
***************************************************************************************************************/
static void Submenu_Set_NetMode(void)
{
    if (Enter_flag==0)
    {
        OLED_ShowC_NMKC(10, 0, 1, 0); //"������ʾ"
        OLED_ShowC_NMKC(10, 1, 2, 0); //"ģʽ����"
        OLED_ShowC_NMKC(10, 2, 3, 0); //"�ٶ�����"
        OLED_ShowC_NMKC(10, 3, 4, 1); //"ͨ������"		//Location=3******************
    }

    if (Enter_flag==1)
    {
        OLED_Clear();//����
        while (1)
        {
            key=Key_Scan(0);			//ɨ�谴��
            Process_Key_Input(key);		//���������¼�
         
            OLED_ShowString(0, 0, "SSID:", 6);   							/* "SSID" */
            OLED_ShowString(18, 1, (const char *)("FarsightESP32"), 6); 	/* WIFI���� */

            if (Location>=2)     Location=0;
            if (Location<=-1)    Location=2;

            if (Location==0)
            {
                OLED_ShowC_NMKC(10, 2, 13, 1); //"����ģʽ"
                OLED_ShowC_NMKC(10, 3, 12, 0); //"WIFIģʽ"
            }

            if (Location==1)
            {
                OLED_ShowC_NMKC(10, 2, 13, 0); //"����ģʽ"
                OLED_ShowC_NMKC(10, 3, 12, 1); //"WIFIģʽ"
            }

            if (Write_flag==1)
            {
                Write_flag=0;

                BUZZ=1;//������������
                delay_ms(200);
                BUZZ=0;
            }

            if (exit_flag==1)
            {
                OLED_Clear();//����
                Enter_flag=0;
                exit_flag=0;
                break;
            }
        }

    }

}



/***************************************************************************************************************
*����:������ʾ���㾲̬���档
*˵����  �˺������ᱻ��ѯ���ã���Ҫ��ʱ�����һ��
***************************************************************************************************************/
void Show_Power(void)
{
    switch (rt_Power)
    {
        case 100: OLED_DrawBMP(110, 0, 127, 1, BMP5);break;//����Ϊ100%    
        case 75:  OLED_DrawBMP(110, 0, 127, 1, BMP4);break;//����Ϊ75%
        case 50:  OLED_DrawBMP(110, 0, 127, 1, BMP3);break;//����Ϊ50%
        case 25:  OLED_DrawBMP(110, 0, 127, 1, BMP2);break; //����Ϊ25%
        case 0:   OLED_DrawBMP(110, 0, 127, 1, BMP1);break;    //����Ϊ0%
        default:break;
    }
}

/***************************************************************************************************************
*����:������ʾ���㾲̬���档
*�β�:��
*����ֵ:��
*˵����  �˺������ᱻ��ѯ���ã���Ҫ��ʱ�����һ��
***************************************************************************************************************/
void StaticPage(void)
{
    //����Ļ����ʾPitch
    OLED_ShowString(0, 3, "Pitch:", 6);
    //����Ļ����ʾRoll
    OLED_ShowString(0, 4, "Roll:", 6);
    //����Ļ����ʾYaw
    OLED_ShowString(0, 5, "Yaw:", 6);
    //����Ļ����ʾmm
    OLED_ShowString(112, 4, "mm", 6);
    //����Ļ����ʾSPED
    OLED_ShowString(66, 5, "SPEED:", 6);

    //��ʼ�����棬��OLED����ʾ��ǰģʽ
    OLED_ShowString(83, 3, "MOD:", 6);
    OLED_ShowNum(107, 3, FS_MODE, 1, 6);

    //��ʾ"huaqing_logo"
    OLED_ShowC_NMKC(0, 0, 5, 0);

    //ͨ�ŵ�ģʽ
    if (NET_MODE == BLE_MODE)
        OLED_DrawBMP(78, 0, 89, 1, BMP10); //��ʾ������ͼ��
    else
        OLED_DrawBMP(78, 0, 89, 1, BMP6); //��ʾWIFI��ͼ��
}

