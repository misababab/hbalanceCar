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
float mechanical_zero;          //机械0点
struct _PARMETER    PARMETER;   //定义PID字符串结构体变量

u8 key;
static int Location=0;                            		//记录菜单选项的位置
static int Enter_flag=0, exit_flag=0, Write_flag=0; 	//按键事件标志位

/*************************************************************************************************************
*功能:数据显示
*形参：angle(角度值),LEFT\RIGHT(编码器速度值),distance(超声波数值),ccd(CCD数值)
**************************************************************************************************************/
void oled_Show(void)
{
    OLED_ShowFloat(36, 3, mpu6050Struct.pitch, 2, 6, 1);   		//显示pitch(俯仰)数值
    OLED_ShowFloat(30, 4, mpu6050Struct.roll, 2, 6, 1);    		//显示roll(翻滚)数值
    OLED_ShowFloat(24, 5, mpu6050Struct.yaw, 2, 6, 1);   		 //显示yaw(偏航)数值
    OLED_ShowFloat(80, 4, (rt_Distence*0.58), 2, 6, 1);  	 	 //超声波数据数值显示
    OLED_ShowFloat(102, 5, (float)rt_EncoderRight, 3, 6, 0);    //显示小车右轮速度值
	
    if ((WIFI_CONNECT_FLAG!=0)||(BLE_CONNECT_FLAG!=0))    //显示当前WIFI连接状态
        OLED_DrawBMP(90, 0, 101, 1, BMP8); //连接
    else
        OLED_DrawBMP(90, 0, 101, 1, BMP9); //断开
	
    Show_Power();   //显示电量

    key = Key_Scan(0); /* 扫描按键 */

    if (key==KEY_CENTER)	
    {
        OLED_Clear();//清屏

        AIN2(0),            AIN1(0);        //小车左停轮转
        BIN1(0),            BIN2(0);        //小车右轮停转
		
        while (1)		
        {
            key=Key_Scan(0);					/* 扫描按键 */
            Process_Key_Input(key);				/* 产生按键事件 */

            if (exit_flag==1) //等待退出按键事件
            {
                OLED_Clear();//清屏
                exit_flag=0;
                break;
            }
            MainMenu_Show();  /* 一级菜单or二级菜单 */
        }
        StaticPage();    //退出while之后要显示一次静态界面
    }
}

/*************************************************************************************************************
*功能:按键处理函数
*形参:(u8 mode):按键模式(0:单次/1:连续)
*返回值:1:按键1/2:按键2/3:按键3/4:按键4/5:按键5
**************************************************************************************************************/
u8 Key_Scan(u8 mode)
{
    static u8 key_up=1;    //按键松开标志

    if (mode) key_up=1;    //如果是连续按下，那么按键的状态会一直有效

    if (key_up&&(KEY0==0||KEY1==0||KEY2==0||KEY3==0||KEY4==0))
    {
        HAL_Delay(10);          //延时去抖动
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
*功能:读取按键信息
*形参:(u8 Keynum)
**************************************************************************************************************/
void Process_Key_Input(u8 Keynum)
{
    if (key)
    {
        switch (key)
        {
            case KEY_LEFT:	exit_flag=1;	break;	//左	退出
            case KEY_UP:	Location--;		break;	//上
            case KEY_DOWN:	Location++;		break;	//下
            case KEY_CENTER:Write_flag=1;	break;	//中央	呼叫菜单/确认修改
            case KEY_RIGHT:	Enter_flag=1;	break;	//右	进入
        }
    }
}

/**********************************************************************************************
*功能:按键菜单
*返回值:无
*形参:无
***********************************************************************************************/
void MainMenu_Show(void)
{
    switch (Location)
    {
        case 0: Submenu_Display_Parameter();break;     //参数显示
        case 1: Submenu_Set_WorkMode();     break;      //模式设置
        case 2: Submenu_Set_Speed();		break;      //速度设定
        case 3: Submenu_Set_NetMode();		break;       //通信设置
        default:
            if (Location>=4) Location=0;
            if (Location<=-1) Location=3;
            break;
    } 

}

/***************************************************************************************************************
*功能:参数显示子菜单
***************************************************************************************************************/

static void Submenu_Display_Parameter(void)
{
    if (Enter_flag==0)
    {
        OLED_ShowC_NMKC(10, 0, 1, 1); //"参数显示"************
        OLED_ShowC_NMKC(10, 1, 2, 0); //"模式设置"
        OLED_ShowC_NMKC(10, 2, 3, 0); //"速度设定"
        OLED_ShowC_NMKC(10, 3, 4, 0); //"通信设置"
    }

    if (Enter_flag==1)
    {
        OLED_Clear();//清屏
       
        mechanical_zero=Mechanical_BALANCE;//获取机械0点
        sprintf(PARMETER.upright_PD, "Stand P:%.0f D:%.1f", PID.Vertical_Kp, PID.Vertical_Kd);
        sprintf(PARMETER.speed_PI, "Speed P:%.0f D:%.2f", PID.Velocity_Kp, PID.Velocity_Ki);
        sprintf(PARMETER.turn_PD, "Turn P:%.0f D:%.2f", PID.Turn_Kp, PID.Turn_Kd);
        sprintf(PARMETER.Mechanical_Zero, "Mechanical:%.2f", mechanical_zero);
        OLED_ShowString(0, 0, PARMETER.upright_PD, 6); 		//直立PD值
        OLED_ShowString(0, 2, PARMETER.speed_PI, 6);     	//速度PI值      
        OLED_ShowString(0, 4, PARMETER.turn_PD, 6); 		//转向PD值      
        OLED_ShowString(0, 6, PARMETER.Mechanical_Zero, 6); //显示机械平衡点值

        while (1)
        {
            key=Key_Scan(0);			/* 读取按键 */
            Process_Key_Input(key);		/* 产生按键事件 */

            //等待按键退出事件
            if (exit_flag==1)
            {
                OLED_Clear();//清屏
                Enter_flag=0;
                exit_flag=0;
                break;
            }
        }
    }
}

/***************************************************************************************************************
*功能:工作模式设置子菜单
***************************************************************************************************************/
static void Submenu_Set_WorkMode(void)
{
    if (Enter_flag==0)
    {
        OLED_ShowC_NMKC(10, 0, 1, 0); //"参数显示"
        OLED_ShowC_NMKC(10, 1, 2, 1); //"模式设置"		//Location=1******************
        OLED_ShowC_NMKC(10, 2, 3, 0); //"速度设定"
        OLED_ShowC_NMKC(10, 3, 4, 0); //"通信设置"
    }

    if (Enter_flag==1)
    {
        OLED_Clear();
        Location=0;

        while (1)
        {
            key=Key_Scan(0);		//检测按键
            Process_Key_Input(key); //产生按键事件

            if (Location>=3)     Location=0;
            if (Location<=-1)    Location=2;

            if (Location==0)
            {
                OLED_ShowC_NMKC(10, 0, 9, 1); //"遥控模式"
                OLED_ShowC_NMKC(10, 1, 11, 0); //"避障模式"
                OLED_ShowC_NMKC(10, 2, 10, 0); //"巡线模式"
            }

            if (Location==1)
            {
                OLED_ShowC_NMKC(10, 0, 9, 0); //"遥控模式"
                OLED_ShowC_NMKC(10, 1, 11, 1); //"避障模式"
                OLED_ShowC_NMKC(10, 2, 10, 0); //"巡线模式"
            }

            if (Location==2)
            {
                OLED_ShowC_NMKC(10, 0, 9, 0); //"遥控模式"
                OLED_ShowC_NMKC(10, 1, 11, 0); //"避障模式"
                OLED_ShowC_NMKC(10, 2, 10, 1); //"巡线模式"
            }

            //检测到写入按键按下
            if (Write_flag==1)
            {
                Write_flag=0;

                FS_MODE = Location;//模式切换,最好是给速度也初始化
                BUZZ=1;delay_ms(200);BUZZ=0;//蜂鸣器哔哔响
            }

            //检测到退出按键按下
            if (exit_flag==1)
            {
                OLED_Clear();	//清屏
                Location=0;		//返回一级菜单，指针指向第一行
                
				Enter_flag=0;
                exit_flag=0;
                
				break;
            }
        }
    }
}


/***************************************************************************************************************
*功能:速度设置子菜单
*形参:无
*返回值:无
***************************************************************************************************************/
static void Submenu_Set_Speed(void)
{
    if (Enter_flag==0)
    {
        OLED_ShowC_NMKC(10, 0, 1, 0); //"参数显示"
        OLED_ShowC_NMKC(10, 1, 2, 0); //"模式设置"
        OLED_ShowC_NMKC(10, 2, 3, 1); //"速度设定"		//Location=2******************
        OLED_ShowC_NMKC(10, 3, 4, 0); //"通信设置"
    }

    if (Enter_flag==1)
    {
        OLED_Clear();//清屏函数
        OLED_ShowC_NMKC(10, 0, 7, 0); //"速度值"
        while (1)
        {
            key=Key_Scan(1);//连续读取按键
            Process_Key_Input(key);//产生按键事件

            if (key==KEY_UP)
			{
                delay_ms(50);    			//速度值+
                target_SpeedEncoder++;
            }

            if ((key==KEY_DOWN) && (target_SpeedEncoder>=1))
            {
                target_SpeedEncoder--;     //速度值-
                delay_ms(50);
            }

            OLED_ShowNum(80, 0, target_SpeedEncoder, 3, 8);

            if (Write_flag==1)
            {
                Write_flag=0;    //向FLASH发送数组填入数据
                BUZZ=1;
                delay_ms(200);
                BUZZ=0;
            }

            if (exit_flag==1)
            {
                OLED_Clear();    //清屏
                Enter_flag=0;
                exit_flag=0;
                break;
            }
        }
    }
}

/***************************************************************************************************************
*功能:速度设置网络模式
*形参:无
*返回值:无
***************************************************************************************************************/
static void Submenu_Set_NetMode(void)
{
    if (Enter_flag==0)
    {
        OLED_ShowC_NMKC(10, 0, 1, 0); //"参数显示"
        OLED_ShowC_NMKC(10, 1, 2, 0); //"模式设置"
        OLED_ShowC_NMKC(10, 2, 3, 0); //"速度设置"
        OLED_ShowC_NMKC(10, 3, 4, 1); //"通信设置"		//Location=3******************
    }

    if (Enter_flag==1)
    {
        OLED_Clear();//清屏
        while (1)
        {
            key=Key_Scan(0);			//扫描按键
            Process_Key_Input(key);		//产生按键事件
         
            OLED_ShowString(0, 0, "SSID:", 6);   							/* "SSID" */
            OLED_ShowString(18, 1, (const char *)("FarsightESP32"), 6); 	/* WIFI名字 */

            if (Location>=2)     Location=0;
            if (Location<=-1)    Location=2;

            if (Location==0)
            {
                OLED_ShowC_NMKC(10, 2, 13, 1); //"蓝牙模式"
                OLED_ShowC_NMKC(10, 3, 12, 0); //"WIFI模式"
            }

            if (Location==1)
            {
                OLED_ShowC_NMKC(10, 2, 13, 0); //"蓝牙模式"
                OLED_ShowC_NMKC(10, 3, 12, 1); //"WIFI模式"
            }

            if (Write_flag==1)
            {
                Write_flag=0;

                BUZZ=1;//蜂鸣器哔哔响
                delay_ms(200);
                BUZZ=0;
            }

            if (exit_flag==1)
            {
                OLED_Clear();//清屏
                Enter_flag=0;
                exit_flag=0;
                break;
            }
        }

    }

}



/***************************************************************************************************************
*功能:用来显示顶层静态界面。
*说明：  此函数不会被轮询调用，需要的时候调用一次
***************************************************************************************************************/
void Show_Power(void)
{
    switch (rt_Power)
    {
        case 100: OLED_DrawBMP(110, 0, 127, 1, BMP5);break;//电量为100%    
        case 75:  OLED_DrawBMP(110, 0, 127, 1, BMP4);break;//电量为75%
        case 50:  OLED_DrawBMP(110, 0, 127, 1, BMP3);break;//电量为50%
        case 25:  OLED_DrawBMP(110, 0, 127, 1, BMP2);break; //电量为25%
        case 0:   OLED_DrawBMP(110, 0, 127, 1, BMP1);break;    //电量为0%
        default:break;
    }
}

/***************************************************************************************************************
*功能:用来显示顶层静态界面。
*形参:无
*返回值:无
*说明：  此函数不会被轮询调用，需要的时候调用一次
***************************************************************************************************************/
void StaticPage(void)
{
    //在屏幕上显示Pitch
    OLED_ShowString(0, 3, "Pitch:", 6);
    //在屏幕上显示Roll
    OLED_ShowString(0, 4, "Roll:", 6);
    //在屏幕上显示Yaw
    OLED_ShowString(0, 5, "Yaw:", 6);
    //在屏幕上显示mm
    OLED_ShowString(112, 4, "mm", 6);
    //在屏幕上显示SPED
    OLED_ShowString(66, 5, "SPEED:", 6);

    //初始化界面，在OLED上显示当前模式
    OLED_ShowString(83, 3, "MOD:", 6);
    OLED_ShowNum(107, 3, FS_MODE, 1, 6);

    //显示"huaqing_logo"
    OLED_ShowC_NMKC(0, 0, 5, 0);

    //通信的模式
    if (NET_MODE == BLE_MODE)
        OLED_DrawBMP(78, 0, 89, 1, BMP10); //显示蓝牙的图标
    else
        OLED_DrawBMP(78, 0, 89, 1, BMP6); //显示WIFI的图标
}

