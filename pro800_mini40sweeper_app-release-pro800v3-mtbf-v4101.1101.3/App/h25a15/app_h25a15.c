/********************************************************************************
*文件名			: app_h25a15.c
*简  介			: 自研驱动控制   
*日	 期			: 2021/7/8
*作  者			: 刘德明/高仙     
********************************************************************************
*备注： 1、驱动器断连、心跳功能未添加  
*		2、/后期要优化确认驱动器收到数据后不再重复发，减少数据量
*		3、驱动器程序不完善，很多信息读取不到
********************************************************************************/
#include "h25a15/app_h25a15.h"
#include "h25a15/drv_h25a15.h"
#include "normal_io.h"
#include "pc_correspond.h"
#include "nanopb_tcpip.h"


Motor_Info_t Squeegee_Lift_Info={0};
Motor_Info_t Roller_Brush_Lift_Info={0};
Motor_Info_t Side_Brush_Lift_Info={0};
Motor_Info_t Water_Pump_Info={0};
Motor_Info_t Fan_Motor_Info={0};
Motor_Info_t Roller_motor_Info={0};
Motor_Info_t Side_motor_Info[2]={0};
Lift_Self_Adaption_t  Self_Adaption={0};  //滚刷推杆自适应信息

/****************************************
* 函数介绍 :创建自研驱动器电机任务
*****************************************/
void H25A_Motor_Task_Create(void)
{	
	Side_Brush_Lift_Info.max_current=2000;            //设置边刷过流阈值电流
	Side_Brush_Lift_Info.over_current_time=1000;      //设置边刷过流阈值时间
	Squeegee_Lift_Info.max_current=2000;
    Squeegee_Lift_Info.over_current_time=1000;         
	Side_Brush_Lift_Info.operate_time = 9000;
	Squeegee_Lift_Info.operate_time = 9000;
	H25a_Info.link_status = 1;

 	xTaskCreate( H25A_Motor_Task, "h25a_motor", 512, 0, 4, NULL );
}

/****************************************
* 函数介绍 ：自研驱动器任务
*****************************************/
void H25A_Motor_Task(void *pvParameters )
{ 	
    uint8_t task_times = 0;/* 逻辑执行频率控制 */
	static uint8_t first_init_flag=1;       //切换驱动器类型后要更新字典，因为两种驱动器ID相同

	vTaskDelay(7000);					     //给驱动器5s上电时间。
	//	H25A_Push_Motor_Calibration();		     //滚刷推杆的初始化标定 40混合驱控是自己标定不需要控制标定
	vTaskDelay(2000);	
	TickType_t last_time= xTaskGetTickCount();

	H25a_Info.heart_time=HAL_GetTick();
	LoadH25aObj();	
    set_h4aa30_pdo_state(ENABLE);
	first_init_flag =0;	
//	H25A_Set_Heartbeat_Time(200);   		//设置驱动器心跳
	vTaskDelay(1);
	H25A_Set_Disconnect_Time(3000);				
 	for(;;)
	{
		while(ConfigCommandCMD.hardware_type != 0)   //hardware_type=1，则不运行混合驱控逻辑
		{	
			vTaskDelay(100);			
		}

        if(task_times++ >= 10)
        {
            task_times = 0;
    		Motor_Control_Task();		//电机控制
    		vTaskDelay(1);
    		H25A_Get_Link_Status();		//获取驱动器连接状态
    		vTaskDelay(1);
    		H25A_Driver_Info_Read();	//读取驱动器信息
    		vTaskDelay(1);
    		H25_Disconnetion_Logic();   //处理驱动器异常
        }
        H4AA_Driver_Info_Read_high_freq_data();
        
		vTaskDelay(pdMS_TO_TICKS(5)) ;
	}
}
/**********************************************************************************************
*函数名	: Motor_Control_Task
*介	 绍	：自研驱动器电机控制任务
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void Motor_Control_Task(void )
{ 	
	static uint8_t index=0;

	switch (index++)
	{
		case 0:
			Brush_Motor_Logic();									//滚刷、边刷控制
			break;
		case 1:
 			Roller_Brush_Lift_Logic(Roller_Brush_Lift_Info.rx_cmd,Self_Adaption.target_current); //滚刷升降控制
			break;
		case 2:
 			Side_Brush_Lift_Logic(Side_Brush_Lift_Info.rx_cmd);		//边刷升降控制 
			break;
		case 3:
			Squeegee_Lift_Logic(Squeegee_Lift_Info.rx_cmd); 		//吸水趴升降控制 
			index=0;
			break;
		default:
				index=0;
			break;
	}
}


/**********************************************************************************************
*函数名	: Brush_Motor_Logic
*介	 绍	：滚刷边刷控制
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void Brush_Motor_Logic(void) 
{

	if( !GetPcConnectStatus()|(GetKeyState() == 0x02) |GetEmergencyStopState()) //急停、钥匙开关关闭、断连
	{
		Roller_motor_Info.tx_cmd=0;
		Side_motor_Info[0].tx_cmd=0;
		Side_motor_Info[1].tx_cmd=0;
	}
	else
	{
		Roller_motor_Info.tx_cmd=Roller_motor_Info.rx_cmd;
		Side_motor_Info[0].tx_cmd=Side_motor_Info[0].rx_cmd;
		Side_motor_Info[1].tx_cmd=Side_motor_Info[1].rx_cmd;
	}
	
	Set_Roller_Brush_Speed(2,Roller_motor_Info.tx_cmd);	      //下发滚刷速度
	Set_Side_Brush_Speed(4,Side_motor_Info[0].tx_cmd);        //左边刷速度设置
	Set_Side_Brush_Speed(3,Side_motor_Info[1].tx_cmd);        //右边刷速度设置
}


/**********************************************************************************************
*函数名	: Squeegee_Lift_Logic
*介	 绍	：吸水趴推杆控制
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void Squeegee_Lift_Logic(uint8_t current_cmd) 
{
	static uint8_t last_operate_finish=0;	//上次操作是否完成，非零表示完成
	static uint8_t last_cmd=1;				//上次执行的命令，0零表示抬起
	static TickType_t start_tick = 0;

	static uint32_t last_over_current_tick;
	
	Squeegee_Lift_Info.current=(PushRodMotor1.current.ActualUnitValue*1000)/200;   //单位mA
	
	if(Squeegee_Lift_Info.current>Squeegee_Lift_Info.max_current)   //检测过流
	{
		if(HAL_GetTick()-last_over_current_tick>Squeegee_Lift_Info.over_current_time)
		{
			Squeegee_Lift_Info.over_current_status=1;
		}
	}
	else
	{
		last_over_current_tick=HAL_GetTick() ;
	}
	
	if( GetKeyState() == 0x02 && !GetEmergencyStopState()) //非急停且钥匙开关关机抬起吸水趴
	{
		current_cmd=0;
	}

    if( last_operate_finish && last_cmd != current_cmd )//上次操作已完成，收到新命令
    {
        last_operate_finish = 0;
        last_cmd = current_cmd;
        start_tick = xTaskGetTickCount();//更新升降电机启动时间
        Squeegee_Lift_Info.run_status = 2;
    }

	if(!last_operate_finish&&(xTaskGetTickCount() - start_tick <= Squeegee_Lift_Info.operate_time))//未完成操作
	{
		if( GetEmergencyStopState())
		{
			if(Squeegee_Lift_Info.tx_cmd) //之前在运动
			{
				last_cmd=0;
			}
			Squeegee_Lift_Info.tx_cmd = 0;
			start_tick = xTaskGetTickCount();
		}
		else
		{			
			Squeegee_Lift_Info.tx_cmd=last_cmd?2:1;
		}
		if((xTaskGetTickCount() - start_tick > 2000))
		{
			Squeegee_Lift_Info.run_status = last_cmd;
		}	}
	else                                              //操作完成
	{

		last_operate_finish = 1;
		Squeegee_Lift_Info.tx_cmd= 0;
		Squeegee_Lift_Info.run_status = last_cmd;
	}
	
	H25_Push_Rod_Ctr(1,Squeegee_Lift_Info.tx_cmd);   //给驱动器下发速度
}

/**********************************************************************************************
*函数名	: Side_Brush_Lift_Logic
*介	 绍	：边刷升降控制
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void Side_Brush_Lift_Logic(uint8_t current_cmd) 
{

	static uint8_t last_operate_finish=0;	//上次操作是否完成，非零表示完成
	static uint8_t last_cmd=1;				//上次执行的命令，0零表示抬起
	static TickType_t start_tick = 0;

	static uint32_t last_over_current_tick;
	
	Side_Brush_Lift_Info.current=(PushRodMotor3.current.ActualUnitValue*1000)/200;   //单位mA
	
	if(Side_Brush_Lift_Info.current>Side_Brush_Lift_Info.max_current)   //检测过流
	{
		if(HAL_GetTick()-last_over_current_tick>Side_Brush_Lift_Info.over_current_time)
		{
			Side_Brush_Lift_Info.over_current_status=1;
		}
	}
	else
	{
		last_over_current_tick = HAL_GetTick() ;
	}
	
	if( GetKeyState() == 0x02 && !GetEmergencyStopState()) //非急停且钥匙开关关机抬起吸水趴
	{
		current_cmd=0;
	}

    if( last_operate_finish && last_cmd != current_cmd )//上次操作已完成，收到新命令
    {
        last_operate_finish = 0;
        last_cmd = current_cmd;
        start_tick = xTaskGetTickCount();//更新升降电机启动时间
        Side_Brush_Lift_Info.run_status = 2;
    }

	if(!last_operate_finish&&(xTaskGetTickCount() - start_tick <= Side_Brush_Lift_Info.operate_time))//未完成操作
	{
		if( GetEmergencyStopState())
		{
			if(Side_Brush_Lift_Info.tx_cmd) //之前在运动
			{
				last_cmd=0;
			}
			Side_Brush_Lift_Info.tx_cmd = 0;
			start_tick = xTaskGetTickCount();
		}
		else
		{			
			Side_Brush_Lift_Info.tx_cmd=last_cmd?1:2; 	//边刷逻辑反了需要调整一下
		}
		if((xTaskGetTickCount() - start_tick > 2000))
		{
			Squeegee_Lift_Info.run_status = last_cmd;
		}
	}
	else                                              //操作完成
	{
		last_operate_finish = 1;
		Side_Brush_Lift_Info.tx_cmd= 0;
		Side_Brush_Lift_Info.run_status = last_cmd;
	}

	H25_Push_Rod_Ctr(3,Side_Brush_Lift_Info.tx_cmd);   //给驱动器下发速度
//	H25A_Brush_Push_Rod_Ctr(3,Side_Brush_Lift_Info.tx_cmd);   //边刷具体行程参数未提供

}

/**********************************************************************************************
*函数名	: Roller_Brush_Lift_Logic
*介	 绍	：滚刷升降控制
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void Roller_Brush_Lift_Logic(uint8_t current_cmd,uint8_t target_current) 
{
	static uint16_t last_target_current=0;   //保存上次下发的自适应电流值
	static uint32_t last_self_control_tick=0;  
	uint8_t target_self_current= target_current;
	Roller_Brush_Lift_Info.tx_cmd = current_cmd;
	Self_Adaption.real_position = PushRodMotor2.targetPWM/14;

	
	if(  GetKeyState() == 0x02 &&!GetEmergencyStopState())              //非急停且钥匙开关关机抬起
	{
		H25_Motor_Enable(6,0);	          //失能推杆电机
		H25A_Set_Brush_Push_Current(0);
		target_self_current=0;
	}  	

	if( target_self_current == 0 )                        //位置控制
	{
		if(last_target_current==0)            //已经将自适应电流置0
		{
			H25A_Brush_Push_Rod_Ctr(2,Roller_Brush_Lift_Info.tx_cmd);       //给驱动器下发速度	
		}
		else
		{
			last_target_current=0;
			H25A_Set_Brush_Push_Current(0); //将自适应电流置0
		}
		last_self_control_tick=HAL_GetTick();
	}
	else
	{
		H25A_Set_Brush_Push_Current(target_self_current*10);               //下发目标电流单位0.1A
		
		if(last_target_current!=target_self_current*10)
		{
			last_self_control_tick=HAL_GetTick();
		}
		last_target_current=target_self_current*10;
	}

	if(target_self_current!=0&&(HAL_GetTick()-last_self_control_tick<4000))
	{
		Roller_Brush_Lift_Info.run_status=12;	
	}
	else if(target_self_current!=0&&(HAL_GetTick()-last_self_control_tick>=4000))
	{
		Roller_Brush_Lift_Info.run_status=1;
	}
	else if( Self_Adaption.real_position >10)  
	{
		Roller_Brush_Lift_Info.run_status=1;
	}
	else if( Self_Adaption.real_position <=10)
	{
		Roller_Brush_Lift_Info.run_status=0;
	}

		
	/*配置自适应相关参数*/
	if(Self_Adaption.target_current && H25a_Info.brush_self_lift_deviation != (Self_Adaption.target_current*10))	
	{
		H25A_Set_Brush_Push_Current_Deviation(Self_Adaption.target_current*10);       
	}
	if(Self_Adaption.deviation && H25a_Info.brush_self_lift_time != Self_Adaption.deviation)	
	{
		H25A_Set_Brush_Push_Filter_Time(Self_Adaption.deviation);
	}
}
extern uint32_t otaing_tick;



/**********************************************************************************************
*函数名	: H25_Disconnetion_Logic
*介	 绍	：驱动器断连处理逻辑
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void H25_Disconnetion_Logic(void) 
{
	static uint32_t start_tick=0;
	static uint32_t send_heart_tick=0;
 	
	if(H25a_Info.heart_time&&H25a_Info.link_status)  //通讯正常更新异常出现时间
	{
		start_tick= xTaskGetTickCount();
	}
	else if(xTaskGetTickCount()-start_tick>5000)     //出现断连超过5s的情况，尝试重新设置心跳  
	{
		start_tick= xTaskGetTickCount();
//		H25A_Set_Heartbeat_Time(200);   		//设置驱动器心跳	
		vTaskDelay(1);
		H25A_Set_Disconnect_Time(3000);		
	}
	
	if(xTaskGetTickCount()-send_heart_tick>=150) //每500ms发送一次主站心跳，用于驱动器在通讯断联时停止电机
	{   
		uint8_t cmd[8] = {5,0,0,0,0,0,0,0};
		send_heart_tick = xTaskGetTickCount();
		vTaskDelay(1);
		if(xTaskGetTickCount()-otaing_tick>7000)
			can_standard_send(CM_CAN1, 0x77F, &cmd, 0x08);	
	}

}

/**********************************************************************************************
*函数名	: Water_Motor_Logic
*介	 绍	：喷水控制
*形  参 : speed:水量
*返回值 : 无
***********************************************************************************************/
void Water_Motor_Logic(uint8_t speed) 
{
	Water_Pump_Info.tx_cmd=speed;
//	if( !Get_Pc_Link_Status()|!InputStatus.power_key |InputStatus.stop_key) //急停、钥匙开关关闭、断连
//	{
//		Water_Pump_Info.tx_cmd=0;
//	}
	Set_Water_Pump_Speed(Water_Pump_Info.tx_cmd);
}
/**********************************************************************************************
*函数名	: Fan_Motor_Logic
*介	 绍	：风机控制
*形  参 : speed:转速 0~100
*返回值 : 无
***********************************************************************************************/
void Fan_Motor_Logic(uint8_t speed) 
{
	Fan_Motor_Info.tx_cmd=speed;
//	if( !Get_Pc_Link_Status()|!InputStatus.power_key |InputStatus.stop_key) //急停、钥匙开关关闭、断连
//	{
//		Fan_Motor_Info.tx_cmd=0;
//	}
	Set_Fan_Motor_Speed(Fan_Motor_Info.tx_cmd );
}	
