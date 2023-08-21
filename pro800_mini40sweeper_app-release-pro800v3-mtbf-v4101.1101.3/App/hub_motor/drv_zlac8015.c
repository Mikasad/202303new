/********************************************************************************
*文件名			: drv_zlac8015.c
*简  介			: 中菱轮毂电机驱动器   
*日	 期			: 2021/7/13
*作  者			: 刘德明/高仙   
********************************************************************************
*备注：
*                                                
********************************************************************************/
#include "hub_motor/drv_zlac8015.h"
#include "cmsis_os.h"
#include "hc_can.h"

ZLAC_MOTOR_T ZLAC_Motor={
	.work_mode = 3, //电机默认速度模式，上报给上位机
};

uint8_t s_zlac_pdo_data[ZLAC_PDO_NUM][CAN_DATA_LEN] = {0};
uint32_t s_zlac_pdo_id_num[ZLAC_PDO_NUM] = {0};

static uint32_t s_zlac_pdo_id_tbl[] = 
{
	0x180+ZLAC_ID,
	0x280+ZLAC_ID,
	0x380+ZLAC_ID,
	0x480+ZLAC_ID
};

/*中菱驱动器对象字典*/
const Objdict_t ZLAC_objdict[] =
{

	//软件版本
    {ZLAC_ID, 0x2016, 0x05, uint32, sizeof(uint32_t), RW, &ZLAC_Motor.soft_version , NULL },
	//同步、异步通讯模式
    {ZLAC_ID, 0x200F, 0x00, uint16, sizeof(uint16_t), RW, &ZLAC_Motor.communication_mode , NULL },
	//温度信息
    {ZLAC_ID, 0x2018, 0x01, uint32, sizeof(uint32_t), RW, &ZLAC_Motor.left_temperature , (void*)Updata_ZALC_Link_Tick},
    {ZLAC_ID, 0x2018, 0x02, uint16, sizeof(uint32_t), RW, &ZLAC_Motor.right_temperature , NULL },
	//故障状态
    {ZLAC_ID, 0x603F, 0x01, uint32, sizeof(uint32_t), RW, &ZLAC_Motor.left_error_code, NULL },
	{ZLAC_ID, 0x603F, 0x02, uint32, sizeof(uint32_t), RW, &ZLAC_Motor.right_error_code, NULL },
	//目标力矩
    {ZLAC_ID, 0x6071, 0x01, int32, sizeof(int32_t), RW, &ZLAC_Motor.left_target_torque, NULL },
	{ZLAC_ID, 0x6071, 0x02, int32, sizeof(int32_t), RW, &ZLAC_Motor.right_target_torque, NULL },
	//控制字
    {ZLAC_ID, 0x6040, 0x00, uint16, sizeof(uint16_t), RW, &ZLAC_Motor.control_word , NULL },
    {ZLAC_ID, 0x6040, 0x01, uint16, sizeof(uint16_t), RW, &ZLAC_Motor.right_control_word , NULL },
	//状态字
    {ZLAC_ID, 0x6041, 0x01, uint16, sizeof(uint16_t), RW, &ZLAC_Motor.status_word , (void*)Updata_ZALC_Link_Tick },	
	{ZLAC_ID, 0x6041, 0x02, uint16, sizeof(uint16_t), RW, &ZLAC_Motor.right_status_word , NULL },	
    //当前操作模式
    {ZLAC_ID, 0x6061, 0x01, uint8, sizeof(uint8_t), RW, &ZLAC_Motor.left_actual_mode , NULL },
    {ZLAC_ID, 0x6061, 0x02, uint8, sizeof(uint8_t), RW, &ZLAC_Motor.right_actual_mode , NULL },
	//工作模式
    {ZLAC_ID, 0x6060, 0x00, uint8, sizeof(uint8_t), RW, &ZLAC_Motor.work_mode , NULL },
	//编码器位置反馈
    {ZLAC_ID, 0x6064, 0x01, uint32, sizeof(uint32_t), RW, &ZLAC_Motor.left_odom_count , ZALC_Left_Odom_Difference_Callback },
    {ZLAC_ID, 0x6064, 0x02, uint32, sizeof(uint32_t), RW, &ZLAC_Motor.right_odom_count , ZALC_Right_Odom_Difference_Callback },
	//速度反馈
//    {ZLAC_ID, 0x606C, 0x01, uint32, sizeof(uint32_t), RW, &ZLAC_Motor.left_real_speed , NULL },
//    {ZLAC_ID, 0x606C, 0x02, uint32, sizeof(uint32_t), RW, &ZLAC_Motor.right_real_speed , ZALC_Right_Odom_Difference_Callback },
	//速度环最大电流限制
    {ZLAC_ID, 0x3001, 0x00, int16, sizeof(uint16_t), RW, &ZLAC_Motor.speed_current_limit , NULL },
	//目标力矩
//	{ZLAC_ID, 0x6071, 0x01, int32, sizeof(int32_t), RO, &ZLAC_Motor.left_target_power , NULL },
//	{ZLAC_ID, 0x6071, 0x02, int32, sizeof(int32_t), RO, &ZLAC_Motor.right_target_power, (void*)Updata_ZALC_Link_Tick },
	//左右轮电机电流
	{ZLAC_ID, 0x6078, 0x01, int32, sizeof(int32_t), RO, &ZLAC_Motor.left_motor_current , NULL },
	{ZLAC_ID, 0x6078, 0x02, int32, sizeof(int32_t), RO, &ZLAC_Motor.right_motor_current, (void*)Updata_ZALC_Link_Tick },
	//母线电压
//	{ZLAC_ID, 0x6079, 0x01, int32, sizeof(int32_t), RO, &ZLAC_Motor.driver_vol , NULL },	
	
	//目标速度
//	{ZLAC_ID, 0x60FF, 0x01, int32, sizeof(int32_t), RO, &ZLAC_Motor.left_target_speed , NULL },
//	{ZLAC_ID, 0x60FF, 0x02, int32, sizeof(int32_t), RO, &ZLAC_Motor.right_target_speed, (void*)Updata_ZALC_Link_Tick },
	//设置掉线保护时间
	{ZLAC_ID, 0x2000, 0x00, uint16, sizeof(uint16_t), RW, &ZLAC_Motor.offline_time, NULL },
	//保存所有RW参数
	{ZLAC_ID, 0x2010, 0x00, uint16, sizeof(uint16_t), RW, &ZLAC_Motor.save_all_param, NULL },
};
const uint16_t zlac_objdict_size = sizeof(ZLAC_objdict)/sizeof(Objdict_t);


/**********************************************************************************************
*函数名	: ZALC_Set_Work_Mode
*介	 绍	：设置驱动工作模式
*形  参 : mode 0：未定义;1：位置模式;3：速度模式;4：转矩模式
*返回值 : 无
***********************************************************************************************/
void ZALC_Set_Work_Mode(uint8_t mode)
{
	CanWriteSDO( ZLAC_ID, 0x6060, 0x00,  uint8,&mode);
}

/**********************************************************************************************
*函数名	: ZALC_Read_Work_Mode
*介	 绍	：读驱动工作模式
***********************************************************************************************/
void ZALC_Read_Work_Mode(void)
{
	CanReadSDO( ZLAC_ID, 0x6060, 0x00,  uint8,&ZLAC_Motor.work_mode);
}

/**********************************************************************************************
*函数名	: ZALC_Set_Synchronous
*介	 绍	：设置通讯模式
*形  参 : mode 0：异步控制;1：同步控制
*返回值 : 无
***********************************************************************************************/
void ZALC_Set_Synchronous(uint8_t mode)
{
	CanWriteSDO( ZLAC_ID, 0x200F, 0x00,  uint16,&mode);
}
/**********************************************************************************************
*函数名	: ZALC_Set_Acceleration
*介	 绍	：设置加速度时间
*形  参 : time：0~32767ms
*返回值 : 无
***********************************************************************************************/
void ZALC_Set_Acceleration(uint32_t time)
{
	CanWriteSDO( ZLAC_ID, 0x6083, 0x01,  uint32,&time);
	CanWriteSDO( ZLAC_ID, 0x6083, 0x02,  uint32,&time);
	
}
/**********************************************************************************************
*函数名	: ZALC_Set_Dcceleration
*介	 绍	：设置减速度时间
*形  参 : time：0~32767ms
*返回值 : 无
***********************************************************************************************/
void ZALC_Set_Dcceleration(uint32_t time)
{
	CanWriteSDO( ZLAC_ID, 0x6084, 0x01,  uint32,&time);
	CanWriteSDO( ZLAC_ID, 0x6084, 0x02,  uint32,&time);
}

/**********************************************************************************************
*函数名	: ZALC_Set_Dcceleration
*介	 绍	：设置驱动器掉线保护时间
*形  参 : time：0~32767ms
*返回值 : 无
***********************************************************************************************/
void ZALC_Set_Offline_Time(uint16_t time)
{
	CanWriteSDO( ZLAC_ID, 0x1016, 0x01,  uint32,&time);	
}
/**********************************************************************************************
*函数名	: ZALC_Driver_Enable
*介	 绍	：驱动器使能失能控制
*形  参 : enable：0失能；1：使能
*返回值 : 无
***********************************************************************************************/
void ZALC_Driver_Enable(uint8_t enable)
{
	uint16_t control_word=0x00;  //控制字写0x80清除告警

	if(enable)
	{
		control_word=0x06;	
		CanWriteSDO( ZLAC_ID, 0x6040, 0x00,  uint16,&control_word);
		vTaskDelay(1);	
		control_word=0x07;	
		CanWriteSDO( ZLAC_ID, 0x6040, 0x00,  uint16,&control_word);
		vTaskDelay(1);
		control_word=0x0F;	
		CanWriteSDO( ZLAC_ID, 0x6040, 0x00,  uint16,&control_word);
	}
	else
	{   
		control_word=0x02;   
		CanWriteSDO( ZLAC_ID, 0x6040, 0x00,  uint16,&control_word);		
	}
}
/**********************************************************************************************
*函数名	: ZALC_Set_Claern_Error
*介	 绍	：清除驱动器错误
*形  参 : clran_flag：1 - 清除告警 0 - 不做处理
*返回值 : 无
***********************************************************************************************/
void ZALC_Set_Claern_Error( uint8_t clean_flag)
{
	uint16_t control_word=0x80;  //控制字写0x80清除告警

	if(clean_flag)
	{
		CanWriteSDO( ZLAC_ID, 0x6040, 0x00,  uint16,&control_word);
		vTaskDelay(1);	
	}
}
/**********************************************************************************************
*函数名	: ZALC_Set_Stop_Enable
*介	 绍	：设置驱动器急停指令
*形  参 : enable：1 - 急停触发 0 - 急停解除
*返回值 : 无
***********************************************************************************************/
void ZALC_Set_Stop_Enable(uint8_t enable)
{
	static uint16_t stop_ctrl = 0;
	
	if(enable)
	{
//		stop_ctrl = 0x02;
		stop_ctrl = 0x00;
		CanWriteSDO( ZLAC_ID, 0x6040, 0x00,  uint16,&stop_ctrl);
		
	}
	else
	{
//		stop_ctrl = 0x0f;		
		CanWriteSDO( ZLAC_ID, 0x6040, 0x00,  uint16,&stop_ctrl);	
	}
}

/**********************************************************************************************
*函数名	: ZALC_Set_Heartbeat_Time
*介	 绍	：设置驱动器心跳发送时间间隔
*形  参 : time：心跳间隔ms
*返回值 : 无
***********************************************************************************************/
void ZALC_Set_Heartbeat_Time(uint32_t time)
{
	CanWriteSDO( ZLAC_ID, 0x1017, 0x00,  uint16,&time);
}

/**********************************************************************************************
*函数名	: ZALC_Motor_Init
*介	 绍	：初始化驱动器为速度模式
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void ZALC_Motor_Init(void)
{
	CanReadSDO( ZLAC_ID, 0x2016, 0x05,  uint32,&ZLAC_Motor.soft_version);
	vTaskDelay(5);	
	ZALC_Set_Synchronous(1);	//0异步模式 1同步模式
	ZALC_Set_Work_Mode(3);		//速度模式
//	ZALC_Set_Acceleration(100);	//S形加速时间10ms
//	ZALC_Set_Dcceleration(100);	//S形减速时间10ms	
//	ZALC_Set_Offline_Time(500);	//设置掉线保护时间为500ms 主站心跳有同样效果
	ZALC_Set_Heartbeat_Time(200);//设置驱动器心跳200ms
	ZALC_Driver_Enable(0);		 //失能驱动器默认手动模式
	ZLAC_Save_All_Param(1);		 //保存所有RW参数到EEPROM
}
/**********************************************************************************************
*函数名	: ZALC_Set_Speed
*介	 绍	：设置左右轮电机速度
*形  参 : 左右轮速度，单位rpm
*返回值 : 无
***********************************************************************************************/
void ZALC_Set_Speed(uint32_t left_speed,uint32_t right_speed)
{
	int32_t speed = 0;
	//异步模式下速度控制指令
//	CanWriteSDO( ZLAC_ID, 0x60FF, 0x01,  uint32,&left_speed);
//	CanWriteSDO( ZLAC_ID, 0x60FF, 0x02,  uint32,&right_speed);
	
	//同步模式下速度控制指令
	speed = (uint16_t)left_speed  | ((uint16_t)right_speed << 16);
	CanWriteSDO( ZLAC_ID, 0x60FF, 0x03,  uint32,&speed);
	
}
/**********************************************************************************************
*函数名	: ZALC_Error_Clear
*介	 绍	：清除0x603F的告警信息
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void ZALC_Error_Clear(void)
{
	uint16_t data=0x80;
	CanWriteSDO( ZLAC_ID, 0x6040, 0x00,  uint16,&data);
	
}
/**********************************************************************************************
*函数名	: ZALC_Read_Error
*介	 绍	：读故障信息
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void ZALC_Read_Error(void)
{
	CanReadSDO( ZLAC_ID, 0x603F, 0x01,  uint32,&ZLAC_Motor.left_error_code);	
	CanReadSDO( ZLAC_ID, 0x603F, 0x02,  uint32,&ZLAC_Motor.right_error_code);
	
}
/**********************************************************************************************
*函数名	: ZALC_Read_Versions
*介	 绍	：读驱动器软件版本
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void ZALC_Read_Versions(void)
{
	if(!ZLAC_Motor.soft_version)
	CanReadSDO( ZLAC_ID, 0x2016, 0x05,  uint32,&ZLAC_Motor.soft_version);
	
}

/**********************************************************************************************
*函数名	: ZALC_Read_Status
*介	 绍	：读驱动器控制字和状态字
***********************************************************************************************/
void ZALC_Read_Status(void)
{
	CanReadSDO( ZLAC_ID, 0x6041, 0x01,  uint32,&ZLAC_Motor.status_word );
}

/**********************************************************************************************
*函数名	: ZALC_Read_Control_Word_Status
*介	 绍	：读驱动器控制字
***********************************************************************************************/
void ZALC_Read_Control_Word_Status(void)
{
	CanReadSDO( ZLAC_ID, 0x6040, 0x00,  uint16,&ZLAC_Motor.control_word);
	
}
/**********************************************************************************************
*函数名	: ZALC_Read_Status
*介	 绍	：读驱动器通讯模式
***********************************************************************************************/
void ZALC_Read_Communicate_mode(void)
{
	CanReadSDO( ZLAC_ID, 0x200F, 0x00,  uint16,&ZLAC_Motor.communication_mode );
	
}
/**********************************************************************************************
*介	 绍	：读电机速度环最大电流限制参数
***********************************************************************************************/
void ZALC_Read_Current_Limit(void)
{
	CanReadSDO( ZLAC_ID, 0x3001, 0x00,  uint16,&ZLAC_Motor.speed_current_limit );
}
/**********************************************************************************************
*函数名	: ZALC_Read_Real_Speed
*介	 绍	：读电机实时速度
***********************************************************************************************/
void ZALC_Read_Real_Speed(void)
{
//	CanReadSDO( ZLAC_ID, 0x606C, 0x01,  uint32,&ZLAC_Motor.left_real_speed);	
//	CanReadSDO( ZLAC_ID, 0x606C, 0x02,  uint32,&ZLAC_Motor.right_real_speed);
	
}
/**********************************************************************************************
*函数名	: ZALC_Read_Real_Speed
*介	 绍	：读左右轮编码器值
***********************************************************************************************/
void ZALC_Read_Odom_Count(void)
{
	CanReadSDO( ZLAC_ID, 0x6064, 0x01,  uint32,&ZLAC_Motor.left_odom_count);
	CanReadSDO( ZLAC_ID, 0x6064, 0x02,  uint32,&ZLAC_Motor.right_odom_count);
	
}

/**********************************************************************************************
*函数名	: ZALC_Read_Temperature
*介	 绍	：读驱动器温度和轮毂电机温度
***********************************************************************************************/
void ZALC_Read_Temperature(void)
{
	static uint8_t index=0;

	switch (index++)
	{
		case 0:
			CanReadSDO( ZLAC_ID, 0x2032, 0x01,  uint16,&ZLAC_Motor.left_temperature);
			break;
		case 1:
			CanReadSDO( ZLAC_ID, 0x2032, 0x02,  uint16,&ZLAC_Motor.right_temperature);			
			break;
		case 2:
			CanReadSDO( ZLAC_ID, 0x2032, 0x03,  uint16,&ZLAC_Motor.driver_temperature);			
			index=0;
			break;
		default:
				index=0;
			break;
	}
}

/**********************************************************************************************
*函数名	: ZALC_Get_Link_Status
*介	 绍	：获取驱动器连接状态
*形  参 : 无
*返回值 : 1：连接正常 0：驱动器断连
***********************************************************************************************/
uint8_t ZALC_Get_Link_Status(void)
{
	ZLAC_Motor.link_status=HAL_GetTick()-ZLAC_Motor.heart_time>2000?0:1;
	return ZLAC_Motor.link_status;
}
/**********************************************************************************************
*函数名	: Updata_ZALC_Link_Tick
*介	 绍	：刷新中菱驱动器连接计数时间
*形  参 : 无
*返回值 : 1：连接正常 0：驱动器断连
***********************************************************************************************/
void Updata_ZALC_Link_Tick(void)
{
	 ZLAC_Motor.heart_time=HAL_GetTick();
}
/**********************************************************************************************
*函数名	: ZALC_Cycle_Read_Driver_Info
*介	 绍	：读取驱动器信息，不重要的数据较长时间读一次，需要频率较高读取的单独读取
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
#define ZALC_READ_INFO_OFFSET (12) /* 低频数据读取从can字典的12个开始 */
void ZALC_Cycle_Read_Driver_Info(void)
{
	static uint8_t index = ZALC_READ_INFO_OFFSET;
	CanReadSDO( ZLAC_objdict[index].id,ZLAC_objdict[index].index, ZLAC_objdict[index].sub_index,ZLAC_objdict[index].data_type,ZLAC_objdict[index].p_data);

	if(index++ >= (zlac_objdict_size-1))
	{
		index = ZALC_READ_INFO_OFFSET;
	}
}
/**
 * @brief 轮毂驱动器读取高频数据
 */
void ZALC_Cycle_Read_Driver_Info_high_data(void)
{
	static uint8_t index = 0;
	CanReadSDO( ZLAC_objdict[index].id,ZLAC_objdict[index].index, ZLAC_objdict[index].sub_index,ZLAC_objdict[index].data_type,ZLAC_objdict[index].p_data);

	if(index++ >= (ZALC_READ_INFO_OFFSET-1))
	{
		index = 0;
	}
}

/**********************************************************************************************
*函数名	: ZALC_Read_Driver_Info
*介	 绍	：读取驱动器信息
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void ZALC_Read_Driver_Info(void)
{
	static uint8_t index=0;

	switch (index++)
	{
		case 1:
				ZALC_Read_Error();				//读故障信息
			break;
		case 2:
				ZALC_Read_Current_Limit();	   //电流限制参数
			break;
		case 3:									//读左右电机的电流
				ZLAC_Read_Motor_Current();
				index=1;
			break;
		case 4:									//读通讯模式
				CanReadSDO( ZLAC_ID, 0x200F, 0x00,  uint16,&ZLAC_Motor.communication_mode );
				index=1;
			break;		
		default:
			index=1;
			break;
	}
}

/**********************************************************************************************
*函数名	: ZALC_Left_Odom_Difference_Callback
*介	 绍	：计算左轮编码器每50ms的差值,每次收到编码器数据后调用
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void ZALC_Left_Odom_Difference_Callback(void* param)
{
	static int32_t last_count=0;		//上次计数值
	static uint32_t last_time=0; 
	int16_t dcount,dtime;  
	int16_t tem;               
	if(HAL_GetTick()-last_time>40) 
	{
		/*计算两次编码器数值的差值和获取时间的差值*/
		dcount=ZLAC_Motor.left_odom_count-last_count;
		dtime =HAL_GetTick()-last_time;
		last_count=ZLAC_Motor.left_odom_count;
		last_time=HAL_GetTick();
		/*不可能产生这么大差值，计数值溢出*/
		if(dcount>30000)
		{
			dcount-=65535;
		}
		else if(dcount<-30000)
		{
			dcount+=65535;
		}
		/*补偿产生的误差*/
		tem=-dcount*50/dtime; //上位机要求uint16类型前进差值为正
		ZLAC_Motor.left_odom_dcount=tem; 
	}
	
	Updata_ZALC_Link_Tick();
}
/**********************************************************************************************
*函数名	: ZALC_Right_Odom_Difference_Callback
*介	 绍	：计算右轮编码器每50ms的差值,每次收到编码器数据后调用
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void ZALC_Right_Odom_Difference_Callback(void* param)
{
	static int32_t last_count=0;		//上次计数值
	static uint32_t last_time=0; 
	int16_t dcount,dtime;                //上位机要求uint16类型
	int16_t tem; 
	if(HAL_GetTick()-last_time>40) 
	{
		/*计算两次编码器数值的差值和获取时间的差值*/
		dcount=ZLAC_Motor.right_odom_count-last_count;
		dtime =HAL_GetTick()-last_time;
		last_count=ZLAC_Motor.right_odom_count;
		last_time=HAL_GetTick();
		/*不可能产生这么大差值，计数值溢出*/
		if(dcount>30000)
		{
			dcount-=65535;
		}
		else if(dcount<-30000)
		{
			dcount+=65535;
		}
		/*补偿产生的误差*/
		tem=dcount*50/dtime;
		ZLAC_Motor.right_odom_dcount=tem; 
	}
	Updata_ZALC_Link_Tick();
}

/**********************************************************************************************
*函数名	: ZLAC_Obj_Load
*介	 绍	：添加对象字典
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void ZLAC_Obj_Load(void)
{
	ObjdictLoad(ZLAC_objdict,zlac_objdict_size);
}

/**********************************************************************************************
*函数名	: ZLAC_Read_Motor_Current
*介	 绍	：读取左右轮电机的电流
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void ZLAC_Read_Motor_Current(void)
{
	CanReadSDO( ZLAC_ID, 0x6078, 0x01,  int32,&ZLAC_Motor.left_motor_current);
	
	CanReadSDO( ZLAC_ID, 0x6078, 0x02,  int32,&ZLAC_Motor.right_motor_current);
	
}

/**********************************************************************************************
*函数名	: ZLAC_Save_All_Param
*介	 绍	：保存所有RW属性参数到EEPROM中
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void ZLAC_Save_All_Param(uint8_t enable)
{
	CanWriteSDO( ZLAC_ID, 0x2010, 0x00,  uint16,&enable);
	
}

/**********************************************************************************************
*函数名	: GetMotorErrorCode
*介	 绍	：给上位机传驱动器左轮错误码
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
uint32_t GetMotorLeftErrorCode(void)
{
    return ZLAC_Motor.left_error_code;
}

/**********************************************************************************************
*函数名	: GetMotorErrorCode
*介	 绍	：给上位机传驱动器左轮错误码
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
uint32_t GetMotorRightErrorCode(void)
{
    return ZLAC_Motor.right_error_code;
}


/**********************************************************************************************
*函数名	: Get_Homemade_LeftHealthState
*介	 绍	：返回左轮健康状态	1健康 0异常
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
uint8_t Get_Homemade_LeftHealthState(void)
{
	uint8_t ret = 0;//1=健康
	ret = ((!ZLAC_Motor.link_status)==0)?1:0;
	return ret;
}
/**********************************************************************************************
*函数名	: Get_Homemade_RightHealthState
*介	 绍	：返回右轮健康状态	1健康 0异常
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
uint8_t Get_Homemade_RightHealthState(void)
{
	uint8_t ret = 0;//1=健康
	ret = ((!ZLAC_Motor.link_status)==0)?1:0;
	return ret;
}

/**********************************************************************************************
*函数名	: ZLAC_Damping_Mode_Switch
*介	 绍	：阻尼模式开关	 
*形  参 : operate 0 关闭 1打开
*返回值 : 无
***********************************************************************************************/
void ZLAC_Damping_Mode_Switch(uint8_t operate)
{
	uint8_t value = operate;
	CanWriteSDO( ZLAC_ID, 0x2017, 0x00,  uint8,&value);
	return  ;
}
/**********************************************************************************************
*函数名	: ZLAC_Work_Mode_Set
*介	 绍	：设置电机工作模式	 
*形  参 : operate 1:位置模式  3:速度模式 4:力矩模式
*返回值 : 无
***********************************************************************************************/
void ZLAC_Work_Mode_Set(uint8_t operate)
{
	uint8_t value = operate;
	CanWriteSDO( ZLAC_ID, 0x6060, 0x00,  uint8,&value);
	CanWriteSDO( ZLAC_ID, 0x6060, 0x01,  uint8,&value);
	return  ;
}

/**********************************************************************************************
*介	 绍	配置电机速度环最大电流限制参数
***********************************************************************************************/
void ZALC_Read_Current_Limit_Set(int value)
{
	uint16_t data = value;
	CanWriteSDO( ZLAC_ID, 0x3001, 0x00,  int16,&data);
}

void set_zlac_pdo_state(uint8_t enable)
{
	uint8_t data[] = {0x80, ZLAC_ID};
	if (enable == 0)
		data[0] = 0x80;
	else
		data[0] = 0x01;
	can_standard_send(CAN1_UNIT, 0x00, &data, sizeof(data)/sizeof(data[0]));
}

void handle_zlac_pdo_data(uint32_t id, uint8_t *pData, uint8_t len)
{
	uint8_t i = 0;

	for (i = 0; i < ZLAC_PDO_NUM; i++)
	{
		if (id == s_zlac_pdo_id_tbl[i])
		{
			s_zlac_pdo_id_num[i]++;
			break;
		}
	}
	
	if (i >= ZLAC_PDO_NUM)
	{
		//printf("invalid zlac pdo id.\r\n");
		return;
	}

	memcpy(&s_zlac_pdo_data[i][0], pData, len);
}

uint32_t get_zlac_pdo_data(uint32_t key)
{
	/* key/value中的value是uint32_t类型的，使用两个key/value传一个PDO数据，offset代表偏移0还是4 */
	uint8_t index = key/2;
	uint8_t offset = (key%(CAN_DATA_LEN/sizeof(uint32_t)))*sizeof(uint32_t);
	
	if (key > CAN_DATA_LEN/sizeof(uint32_t)*ZLAC_PDO_NUM)
		return 0;

	#if 0
	return s_zlac_pdo_data[key][key%2];
	#else
	return s_zlac_pdo_data[index][offset] << 24 | s_zlac_pdo_data[index][offset+1] << 16 | 
			s_zlac_pdo_data[index][offset+2] << 8 | s_zlac_pdo_data[index][offset+3];
	#endif
}


