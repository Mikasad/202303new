/********************************************************************************
*文件名			: app_hub_motor.c
*简  介			: 轮毂电机控制驱动器   
*日	 期			: 2021/7/13
*作  者			: 刘德明/高仙     
********************************************************************************
*备注：  目前CAN线断连驱动器不刹车，后期需要看如何开启断连刹车功能，或断连时关闭
*		  驱动器电源。                                  
********************************************************************************/

#include "hub_motor/app_hub_motor.h"
#include "hub_motor/drv_zlac8015.h"
#include "ctr_move_task.h"
#include "normal_io.h"
#include "pc_correspond.h"


Motor_Control_Info_t Hub_Info={0};
/****************************************
* 函数介绍 :创建轮毂电机任务
*****************************************/
void Hub_Motor_Task_Create(void)
{	
	xTaskCreate( ZLAC_Motor_Task, "zlac_motor", 256, 0, 6, NULL );
}
#include "nanopb_tcpip.h"
/****************************************
* 函数介绍 :轮毂电机任务
*****************************************/
void ZLAC_Motor_Task(void *pvParameters )
{ 	
	static uint32_t time_cnt=0;
	static int16_t last_left_speed = 0;
	static int16_t last_right_speed = 0;
	vTaskDelay(7000);				//给驱动器5s上电时间。
	
	ZLAC_Motor.link_status =1;
    while(ConfigCommandCMD.motor_driver_type != 4)
	{
		vTaskDelay(50);
	}
	ZLAC_Obj_Load();               //加载中菱驱动器对象字典
	DeviceCommandCMD.zlac_current_limit = 32000;
	ZALC_Motor_Init();				//初始化驱动器为速度模式并设置参数
	set_zlac_pdo_state(ENABLE);
	TickType_t last_time= xTaskGetTickCount();
	Updata_ZALC_Link_Tick();	

	for(;;)
	{
		time_cnt++;
		Hub_Motor_Info_Read();		//读取轮毂电机相关信息*/		
		ZALC_Get_Link_Status();		//判断驱动器连接状态
		Hub_Motor_Logic();			//电机控制逻辑
		Hub_Motor_Param_Update();   //更新电机参数
		
		if((Hub_Info.left_speed != last_left_speed) || (Hub_Info.right_speed != last_right_speed))		//下发速度不一样时，给驱动器立即更新速度
		{
			ZALC_Set_Speed(Hub_Info.left_speed,Hub_Info.right_speed);	
			last_left_speed = Hub_Info.left_speed;
			last_right_speed = Hub_Info.right_speed;
		}
		else if(time_cnt%20==0)			//100ms下发一次驱动器速度
		{
			ZALC_Set_Speed(Hub_Info.left_speed,Hub_Info.right_speed);
		}
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}
extern uint32_t otaing_tick;

/**********************************************************************************************
*函数名	: Hub_Motor_Logic
*介	 绍	：电机控制逻辑
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void Hub_Motor_Logic(void)
{
	static uint8_t mode_change_status=0; //0未进行模式切换 1、2切换中
	static uint32_t mode_change_tick=0;    //用于避免频繁下发切换指令
	uint8_t error_status=0;
	uint8_t mode_ckeck_status=0;
	static uint32_t send_heart_tick = 0;
	/*获取上位机速度*/
	if(ZLAC_Motor.work_mode == 4)
	{
		Hub_Info.left_speed = Hub_Info.left_rx_speed*100;
		Hub_Info.right_speed = -Hub_Info.right_rx_speed*100;
		/*轮毂电机限速*/
		Hub_Info.left_speed=Hub_Info.left_speed>3000?3000:Hub_Info.left_speed;
		Hub_Info.left_speed=Hub_Info.left_speed<-3000?-3000:Hub_Info.left_speed;
		Hub_Info.right_speed=Hub_Info.right_speed>3000?3000:Hub_Info.right_speed;
		Hub_Info.right_speed=Hub_Info.right_speed<-3000?-3000:Hub_Info.right_speed;		
	}
	else if(ZLAC_Motor.work_mode == 3)
	{
		Hub_Info.left_speed=-Hub_Info.left_rx_speed*10;
		Hub_Info.right_speed=Hub_Info.right_rx_speed*10;
		/*轮毂电机限速*/
		Hub_Info.left_speed=Hub_Info.left_speed>1500?1500:Hub_Info.left_speed;
		Hub_Info.left_speed=Hub_Info.left_speed<-1500?-1500:Hub_Info.left_speed;
		Hub_Info.right_speed=Hub_Info.right_speed>1500?1500:Hub_Info.right_speed;
		Hub_Info.right_speed=Hub_Info.right_speed<-1500?-1500:Hub_Info.right_speed;		
	}

	mode_ckeck_status = Hub_Motor_Mode_Check();
	
	/*上位机断连、急停触发、模式切换时速度清零*/	 
	error_status=!GetPcConnectStatus()|GetEmergencyStopState()|mode_ckeck_status;
	
	if(error_status)
	{
		Hub_Info.left_speed=0;
		Hub_Info.right_speed=0;
		Hub_Info.left_rx_speed =0;
		Hub_Info.right_speed = 0;
	}
 	/*每500ms发送一次主站心跳，用于驱动器在通讯断联时停止电机*/	 
	if(xTaskGetTickCount() - send_heart_tick > 150) //
	{   
		uint8_t cmd[8] = {5,0,0,0,0,0,0,0};
		send_heart_tick = xTaskGetTickCount();
		vTaskDelay(1);   //驱控说有收不到的情况，加个延时
		if(xTaskGetTickCount()-otaing_tick>7000)
			can_standard_send(CM_CAN1, 0x77F, &cmd, 0x08); 
		vTaskDelay(1);
	}

	if(Hub_Info.clean_error_flag)
	{
		ZALC_Set_Claern_Error(1);
		Hub_Info.clean_error_flag = 0;
	}
	else if((HAL_GetTick()- mode_change_tick >200)&&(Hub_Info.tx_mode != Hub_Info.real_mode )) //最多200ms下发一次
	{
		mode_change_tick = HAL_GetTick();
		ZALC_Driver_Enable(Hub_Info.tx_mode);
 	}
	
}
/**********************************************************************************************
*函数名	: Hub_Motor_Mode_Check
*介	 绍	：检查模式切换状态
*形  参 : 无
*返回值 : 0：切换完成或无需切换 1：模式切换中
***********************************************************************************************/
uint8_t Hub_Motor_Mode_Check(void)
{
	uint8_t result = 1;
	
	Hub_Info.real_mode = (ZLAC_Motor.control_word == 0x0F)?1:0; //根据驱动器状态字获取当前真实模式状态

	if(GetEmergencyStopState()) //急停触发时強制切速度模式并使能，即自動模式
	{
		Hub_Info.rx_mode = 1;     //切自动模式
		Hub_Info.rx_ctrl_mode = 3;//切速度模式
	}
	else
	{
		Hub_Info.rx_mode = GetCurCtrMode();//GetCurCtrMode() 1自动 0手动
		Hub_Info.rx_ctrl_mode = ConfigCommandCMD.control_mode?4:3;//更新上位机控制模式命令 0速度模式 1转矩模式
		
		if(Hub_Info.rx_ctrl_mode == 4) //力矩模式需求使能驱动器，驱动器才能助力.此时使能后驱动器不会锁住轮子
		{
			Hub_Info.rx_mode = 1; //切自动模式
		}
	}

	if(ZLAC_Motor.work_mode !=4 && ZLAC_Motor.work_mode !=3 )
	{
		ZLAC_Work_Mode_Set(3);//切换到速度 
	}
	else if(ZLAC_Motor.work_mode !=Hub_Info.rx_ctrl_mode )//ZLAC_Motor.work_mode 1:位置模式  3:速度模式 4:力矩模式
	{
		ZLAC_Work_Mode_Set(Hub_Info.rx_ctrl_mode); 	
	}
	else
	{
		result = 0;
	}
	
	if(Hub_Info.rx_mode!=Hub_Info.real_mode)
	{
		Hub_Info.tx_mode = Hub_Info.rx_mode;
	}

	return result;
}
/**********************************************************************************************
*函数名	: Hub_Motor_Info_Read
*介	 绍	：读取轮毂电机相关信息
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void Hub_Motor_Info_Read(void)
{
	static uint32_t odom_tick=0;
	static uint32_t read_dict_tick=0;
	static uint32_t info_tick=0;
	static uint32_t control_word_tick = 0;
	

	/*25ms读取一次驱动器odom计数值*/
	if(HAL_GetTick()-odom_tick>=25)
	{
		odom_tick=HAL_GetTick();
		ZALC_Read_Odom_Count();
	}
	
	//80ms读取一次控制字
	if(HAL_GetTick()-control_word_tick>=79)
	{
		control_word_tick=HAL_GetTick();

//		ZALC_Read_Status();				//读状态字 
		ZALC_Read_Control_Word_Status();//读控制字
		vTaskDelay(1);
		ZALC_Read_Work_Mode();         //读工作模式
	}
	/*200ms读取驱动器字典一个成员，主要用来读取要求频率低的数据*/
	if(HAL_GetTick()-read_dict_tick>=200)
	{ 
		read_dict_tick=HAL_GetTick();
		ZALC_Cycle_Read_Driver_Info();
	}
	/*50ms读取一次驱动器相对需要高频读取的状态信息*/
	if(HAL_GetTick()-info_tick>=49)
	{
		info_tick=HAL_GetTick();
		ZALC_Read_Driver_Info();
	}
}

/**********************************************************************************************
*函数名	: Hub_Motor_error_Clear
*介	 绍	：清除轮毂电机告警
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void Hub_Motor_error_Clear(void)
{
	Hub_Info.clean_error_flag =1;
}

/**********************************************************************************************
*介	 绍	：更新轮毂驱动器参数
***********************************************************************************************/
void Hub_Motor_Param_Update(void)
{
	Other_DeviceData.zlac_current_limit = ZLAC_Motor.speed_current_limit;

	static uint32_t update_tick=0;
	/*200ms更新一下轮毂速度限制电流*/
	if(HAL_GetTick()-update_tick>=200)
	{
		update_tick=HAL_GetTick();
		if(DeviceCommandCMD.zlac_current_limit&&(DeviceCommandCMD.zlac_current_limit != ZLAC_Motor.speed_current_limit))
		{
			ZALC_Read_Current_Limit_Set(DeviceCommandCMD.zlac_current_limit);	
		}
 	}
}


















