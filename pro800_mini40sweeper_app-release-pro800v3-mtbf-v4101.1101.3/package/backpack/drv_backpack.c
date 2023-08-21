/********************************************************************************
*文件名			: drv_backpack.c
*简  介			: 消杀、香薰背包控制 
*日	 期			: 2022/04/07
*作  者			: 高仙/刘德明 
********************************************************************************
* 备注：  消杀3.0功能，具体软硬件设计查看jira PLATFORM-787
* 消杀3.0 用于40的PRO800V1.0和PRO800V2.0控制盒整车
* 消杀3.0 只用于50的PRO800V2.0控制盒整车
* 因为疫情导致无法调试整车，暂时只聚焦40消杀3.0背包功能，不保证兼容旧的消杀逻辑。
********************************************************************************/
 
/******************************头文件*******************************************/
#include "drv_backpack.h"
#include "normal_io.h"
#include "nanopb_tcpip.h"

/****************************外部全局变量**************************************/
extern struct DEVICECOMMAND_CMD  DeviceCommandCMD ;
extern struct OTHER_DEVICE_DATA  Other_DeviceData;
extern struct CONFIGCOMMAND_CMD  ConfigCommandCMD;

extern uint8_t g_using_disinfect_flag;

/******************************本地变量****************************************/
static BackPackInfo_t BackPackInfo = {0};
static BackPackCtrl_t BackPackCtrl = {0};

/***************************私有函数原形*************************************/
static void backpack_parameter_init(void);
static void updata_device_command(void);
static void updata_device_data(void);
static void disfection_ctrl(uint8_t cmd_1, uint8_t cmd_2);
static void incense_ctrl(uint8_t cmd);
static void backpack_task(void *pvParameters); 


 
/****************************************************************************
*函数名	: BackpackTaskInit
*介	 绍	：背包任务初始化
*****************************************************************************/
void BackpackTaskInit(void)
{	
    /*参数初始化*/
	backpack_parameter_init();
    /*创建背包任务*/
	xTaskCreate( backpack_task, "back_pack", 256, 0, 5, NULL );
}

/****************************************************************************
*函数名	: backpack_task
*介	 绍	：香薰、消杀包信息初始化
*形  参 : 无
*返回值 : 无
*****************************************************************************/
static void backpack_task(void *pvParameters)
{ 
    while(1)
    {
		while( g_using_disinfect_flag!=2 )
		{
			vTaskDelay(100);
		}
        updata_device_command();                                                       /*更新上位机命令*/
        updata_device_data();                                                          /*更新消杀包状态*/ 
        disfection_ctrl(BackPackCtrl.disinfect_cmd_1,BackPackCtrl.disinfect_cmd_2);    /*消杀包控制*/ 
        incense_ctrl(BackPackCtrl.incense_cmd);                                        /*香薰包控制*/ 
        vTaskDelay(100);    
    }
}

/****************************************************************************
*函数名	: backpack_info_init
*介	 绍	：香薰、消杀包信息初始化
*形  参 : 无
*返回值 : 无
*****************************************************************************/
static void backpack_parameter_init(void)
{
	BackPackInfo.device_box_type = 1;		/*默认为2.0主板*/
}

/****************************************************************************
*函数名	: updata_device_command
*介	 绍	：更新控制命令
*形  参 : 无
*返回值 : 无
*****************************************************************************/
static void updata_device_command(void)
{
	BackPackInfo.device_box_type = ConfigCommandCMD.board_type; // 1 PRO800V1.0 ，2：PRO800V2.0
    BackPackCtrl.disinfect_cmd_1 = DeviceCommandCMD.mist_relay_1;
    BackPackCtrl.disinfect_cmd_2 = DeviceCommandCMD.mist_relay_2;
    BackPackCtrl.incense_cmd = DeviceCommandCMD.incense_relay;
    
}
/****************************************************************************
*函数名	: updata_device_data
*介	 绍	：更新背包状态
*形  参 : 无
*返回值 : 无
*****************************************************************************/
static void updata_device_data(void)
{
    /*更新背包状态数据上报*/
    Other_DeviceData.disfection_level = BackPackInfo.disfection_level;
    Other_DeviceData.atomizer_level = BackPackInfo.atomizer_level;
    Other_DeviceData.incense_level = BackPackInfo.incense_level;
    Other_DeviceData.disinfect_detect = BackPackInfo.disinfect_detect;
    Other_DeviceData.incense_detect = BackPackInfo.incense_detect;
    
	/*下位机使用PRO800V2.0控制盒*/
	if(BackPackInfo.device_box_type == 2)                            
	{
		/*背包安装检测*/
		BackPackInfo.disinfect_detect = m_allKeysSt.in[14].state ;  //香薰    
		BackPackInfo.incense_detect = m_allKeysSt.in[12].state ;  //消杀    
		
		/*背包低液位检测*/
		BackPackInfo.disfection_level = m_allKeysSt.in[10].state ; //背包液位
		//BackPackInfo.atomizer_level = m_allKeysSt.in[11].state ; //雾化箱液位
		if(1 == m_allKeysSt.in[11].state)//雾化箱液位
		{
			BackPackInfo.atomizer_level = 0;
		}
		else
		{
			BackPackInfo.atomizer_level = 1;
		}

		BackPackInfo.incense_level = m_allKeysSt.in[13].state ; //香薰液位		
	}
	/*下位机使用PRO800V1.0控制盒,只针对40机型*/
	else if(BackPackInfo.device_box_type < 2 && CAR_TYPE != 50 )
	{
		/*背包安装检测*/
		if(1 == m_allKeysSt.in[6].state)
        {
			BackPackInfo.disinfect_detect = 0;//香薰 
        }
		else
        {
			BackPackInfo.disinfect_detect = 1;
        }

		if(m_allKeysSt.in[8].state)
        {
			BackPackInfo.incense_detect = 0;//消杀
        }
		else
        {
			BackPackInfo.incense_detect = 1;    
        }  
		/*背包低液位检测*/
		BackPackInfo.disfection_level = m_allKeysSt.in[4].state; 
		BackPackInfo.atomizer_level = m_allKeysSt.in[5].state = 0 ? 1 : 0; 
		BackPackInfo.incense_level = m_allKeysSt.in[7].state;
	}
}

/****************************************************************************
*函数名	: disfection_ctrl
*介	 绍	：消杀包控制
*形  参 : cmd_1 继电器1， 0:关闭 1:打开 、 cmd_2 继电器2， 0:关闭 1:打开 
*返回值 : 无
******************************************************************************/
static void disfection_ctrl(uint8_t cmd_1, uint8_t cmd_2)
{
	if(BackPackInfo.device_box_type < 2 && CAR_TYPE == 50 )
	{
		return;             /*50机型PRO800V1.0控制盒,不适配消杀3.0*/
	}
	
	if(cmd_1)
	{
		OUTPUT1_24V_ON;
	}
	else
	{
		OUTPUT1_24V_OFF;
	}    
    
	if(cmd_2)
	{
		OUTPUT2_24V_ON;	   
	}
	else
	{
		OUTPUT2_24V_OFF;
	}
}
/****************************************************************************
*函数名	: incense_ctrl
*介	 绍	：香薰包控制
*形  参 : cmd: 0:关闭香薰 1:打开香薰(打开电气图中K2)
*返回值 : 无
******************************************************************************/
static void incense_ctrl(uint8_t cmd)
{
	if(cmd == 1)
	{
		CO24V_1_OFF;//实测是反的
	}
	else
	{
		CO24V_1_ON;
	}
}





