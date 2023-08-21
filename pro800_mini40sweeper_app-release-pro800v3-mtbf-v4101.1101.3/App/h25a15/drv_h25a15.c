/********************************************************************************
*文件名			: drv_h25a15.c
*简  介			: 自研驱动控制基本控制驱动  
*日	 期			: 2021/7/8
*作  者			: 刘德明/高仙   
********************************************************************************
*备注：未使用标志CANopen协议，仅使用通讯CANopen格式 
*                                                
********************************************************************************/
#include "h25a15/drv_h25a15.h"
#include "hc_can.h"


H25A_MOTOR_T H25a_Info={0};
/*声明驱动器信息结构体变量*/
MOTOR_PARAMETER_T RollerBrushMotor1;		//J6端口无刷驱动器1，前滚刷
MOTOR_PARAMETER_T RollerBrushMotor2;		//J7端口无刷驱动器2，后滚刷
MOTOR_PARAMETER_T PushRodMotor1;			//J1端口有刷H桥1，边刷推杆 
MOTOR_PARAMETER_T PushRodMotor2;			//J2端口有刷H桥2，滚刷推杆
MOTOR_PARAMETER_T PushRodMotor3;			//J3端口有刷H桥3，吸水趴推杆



//ROBOTPLAN-2999 40电气相关运行数据上传需求要传很多数据 ，
//决定将字典所有索引用做key
//int32_t H4AA30_driver_info[80]={0};
H4AA30_INFO_T g_h4aa30_sdo_data = {0};
uint8_t s_h4aa30_pdo_data[H4AA30_PDO_NUM][CAN_DATA_LEN] = {0};
uint32_t s_h4aa30_pdo_id_num[H4AA30_PDO_NUM] = {0};

static uint32_t s_h4aa30_pdo_id_tbl[] = 
{
	0x180+H25A_ID,
	0x280+H25A_ID,
	0x380+H25A_ID,
	0x480+H25A_ID
};

/*自研驱动器对象字典*/
const Objdict_t H25A_objdict[] =
{
    //混合驱控需要通过SDO上传的数据
	//当前电流
    {H25A_ID, 0x2007, 0x01, int16, sizeof(int16_t), RW, &PushRodMotor1.current.ActualUnitValue, NULL },//吸水趴推杆
    //母线电压
    {H25A_ID, 0x200A, 0x01, int16, sizeof(int16_t), RW, &g_h4aa30_sdo_data.bus_voltage, NULL },
    //当前电流
    {H25A_ID, 0x3004, 0x02, int16, sizeof(int16_t), RW, &RollerBrushMotor2.current.ActualUnitValue, NULL },
	{H25A_ID, 0x3004, 0x04, int16, sizeof(int16_t), RW, &RollerBrushMotor1.current.ActualUnitValue , NULL },
	//驱动器全局错误
    {H25A_ID, 0x300D, 0x00, int32, sizeof(int32_t), RW, &H25a_Info.error_code, NULL },
	{H25A_ID, 0x300D, 0x02, int32, sizeof(int32_t), RW, &H25a_Info.error_code_sub, NULL },
	{H25A_ID, 0x3011, 0x01, uint8, sizeof(uint8_t), RW, &g_h4aa30_sdo_data.roller_lift_position_loop, NULL },
	{H25A_ID, 0x3011, 0x07, uint16, sizeof(uint16_t), RW, &g_h4aa30_sdo_data.dust_lift_positon, NULL },
	//推杆标定状态
	{H25A_ID, 0x3017, 0x01, uint8, sizeof(uint8_t), RW, &g_h4aa30_sdo_data.dust_lift_calib , NULL },
    {H25A_ID, 0x3017, 0x02, uint8, sizeof(uint8_t), RW, &PushRodMotor2.calibration , NULL },    //滚刷推杆标定状态
	//自适应偏差值
    {H25A_ID, 0x3009, 0x01, uint16, sizeof(uint16_t), RW, &H25a_Info.brush_self_lift_deviation , NULL },     
	//自适应过滤时间
    {H25A_ID, 0x3009, 0x02, uint16, sizeof(uint16_t), RW, &H25a_Info.brush_self_lift_time , NULL },  
    {H25A_ID, 0x3003, 0x02, int16, sizeof(int16_t), RW, &g_h4aa30_sdo_data.motor2_current , NULL },
    {H25A_ID, 0x2003, 0x04, uint16, sizeof(uint16_t), RW, &g_h4aa30_sdo_data.roller_brush_pwm , NULL },
    {H25A_ID, 0x2003, 0x05, int8, sizeof(int8_t), RW, &g_h4aa30_sdo_data.brush_type , NULL },
	//软件版本
    {H25A_ID, 0x3000, 0x02, uint32, sizeof(uint32_t), RO, &H25a_Info.soft_version , NULL },
    {H25A_ID, 0x3000, 0x03, uint32, sizeof(uint32_t), RO, & H25a_Info.hard_version  , NULL },
    
    {H25A_ID, 0x2007, 0x02, int16, sizeof(int16_t), RW, &PushRodMotor2.current.ActualUnitValue, NULL },//滚刷推杆
    {H25A_ID, 0x2007, 0x03, int16, sizeof(int16_t), RW, &PushRodMotor3.current.ActualUnitValue, NULL },//边刷推杆 1A=200
	//实际位置
	{H25A_ID, 0x3011, 0x04, uint16, sizeof(uint16_t), RW, &PushRodMotor2.targetPWM, NULL },     //滚刷推杆	
	/*心跳接收时间*/
    {H25A_ID, 0xFFFF, 0, uint32, sizeof(uint32_t), RO, &H25a_Info.heart_time , NULL },
};
const uint16_t h25_objdict_size = sizeof(H25A_objdict)/sizeof(Objdict_t);

/**********************************************************************************************
*函数名	: InitH25aObj
*介	 绍	：添加对象字典
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void LoadH25aObj(void)
{
	ObjdictLoad(H25A_objdict,h25_objdict_size);
}


/**********************************************************************************************
*函数名	: H25_Motor_Enable
*介	 绍	：电机使能、失能控制
*形  参 : id:电机编号 enable：0失能 1使能
*返回值 : 无
***********************************************************************************************/
void H25_Motor_Enable(uint8_t id,uint8_t enable) 
{
	CanWriteSDO( H25A_ID, 0x2001, id,  uint8,&enable); 		//使能推杆
}
/**********************************************************************************************
*函数名	: H25_Set_Motor_Pwm
*介	 绍	：电机PWM输出控制
*形  参 : id:电机编号 PWM：
*返回值 : 无
***********************************************************************************************/
void H25_Set_Motor_Pwm(uint8_t id,uint16_t pwm) 
{
	CanWriteSDO( H25A_ID, 0x2002, id,  uint16,&pwm); 		
}
/**********************************************************************************************
*函数名	: H25_Set_Motor_Pwm
*介	 绍	：电机PWM输出控制
*形  参 : id:电机编号 PWM：
*返回值 : 无
***********************************************************************************************/
void H4AA30_Set_Motor_Pwm(uint8_t id,uint16_t pwm)
{
	CanWriteSDO( H25A_ID, 0x3011, id,  uint16,&pwm); 	
}
/**********************************************************************************************
*函数名	: H25_Set_Motor_Speed
*介	 绍	：设置驱动器无刷电机速度
*形  参 : id:电机编号 speed：转速
*返回值 : 无
***********************************************************************************************/
void H25_Set_Motor_Speed(uint8_t id,int16_t speed) 
{
	CanWriteSDO( H25A_ID, 0x3001, id,  uint16,&speed); 		
}
/**********************************************************************************************
*函数名	: H25_Push_Rod_Ctr
*介	 绍	：滚刷推杆电机控制，有位置控制
*形  参 : id:2滚刷推杆 3边刷边刷推杆     location：位置
*返回值 : 无
***********************************************************************************************/
void H25A_Brush_Push_Rod_Ctr(uint8_t id , uint16_t location)
{	
	location=location*2.5f; //上位机需要响应0，实际下发范围为0~40而不是0~100。   

	H4AA30_Set_Motor_Pwm(id,location*6);  //0-593   驱动组给的最新行程数据为593
	H25_Motor_Enable(id+4,1);	//使能推杆电机
}
/**********************************************************************************************
*函数名	: H25_Push_Rod_Ctr
*介	 绍	：推杆电机控制
*形  参 : id:1:吸水趴 ；2滚刷；3:边刷 status：0关闭 1收缩 2升长
*返回值 : 无
***********************************************************************************************/
void H25_Push_Rod_Ctr(uint8_t id,uint8_t status)
{
	int16_t pwm=0;
	uint8_t enable=1;
	if(status==0)
	{
		enable=0;
	}
	else if(status==1)
	{
		pwm=8000;
	}
	else if(status==2)
	{
		pwm=-8000;
	}
	H25_Set_Motor_Pwm(id,pwm);	//设置推杆电机PWM
	H25_Motor_Enable(id+4,enable);	//使能推杆电机
}

/**********************************************************************************************
*函数名	: H25_Push_Rod_Ctr
*介	 绍	：滚刷边刷电机控制
*形  参 : id:1前滚刷 2后滚刷  3:右边刷 4:左边刷 ； speed：速度 
*返回值 : 无
***********************************************************************************************/
void H25_Roller_Brush_Ctr(uint8_t id,int16_t speed) 
{
	H25_Set_Motor_Speed(id,speed);         	//滚刷边刷速度
 	H25_Motor_Enable(id,speed?1:0);			//使能滚刷、边刷电机
}
/**********************************************************************************************
*函数名	: H25_Water_Pump_Ctr
*介	 绍	：水泵控制
*形  参 : pwm
*返回值 : 无
***********************************************************************************************/
void H25_Water_Pump_Ctr(uint16_t pwm) 
{
	H25_Set_Motor_Pwm(0x09,pwm);	//设置PWM值
	if(pwm==0)
	H25_Motor_Enable(0x0d,0);		//失能水泵
	else
	H25_Motor_Enable(0x0d,1);		//使能水泵
}

/**********************************************************************************************
*函数名	: H25_Fan_Motor_Ctr
*介	 绍	：风机控制
*形  参 : pwm
*返回值 : 无
***********************************************************************************************/
void H25_Fan_Motor_Ctr(uint16_t speed) 
{
	H25_Set_Motor_Speed(0x04,speed);	//设置PWM值
	if(speed==0)
	H25_Motor_Enable(0x04,0);		//失能风机
	else
	H25_Motor_Enable(0x04,1);		//使能风机
}

/**********************************************************************************************
*函数名	: Set_Fan_Motor_Speed
*介	 绍	：风机控制
*形  参 : speed : 0~100
*返回值 : 无
***********************************************************************************************/
void Set_Fan_Motor_Speed(uint8_t  speed) 
{
	if(speed>100) 
	{
		speed=100;
	}
	H25_Fan_Motor_Ctr(speed*84);   //0~8400
}

/**********************************************************************************************
*函数名	: Set_Water_Pump_Speed
*介	 绍	：水泵控制
*形  参 : speed : 0~100
*返回值 : 无
***********************************************************************************************/
void Set_Water_Pump_Speed(uint8_t  speed) 
{
	if(speed>100) 
	{
		speed=100;
	}
	H25_Water_Pump_Ctr(speed*40);   //0~4000
}

/**********************************************************************************************
*函数名	: Set_Roller_Brush_Speed
*介	 绍	：滚刷控制
*形  参 : id:1前滚刷 2后滚刷  speed : -100~100
*返回值 : 无
***********************************************************************************************/
void Set_Roller_Brush_Speed( uint8_t id,int8_t speed) 
{
	if(speed>100) 
	{
		speed=100;
	}
	else if(speed<-100)
	{
		speed=-100;
	}	
	if(id<3)
	{
		H25_Roller_Brush_Ctr(id,speed*30); 
	}		  
}
/**********************************************************************************************
*函数名	: Set_Side_Brush_Speed
*介	 绍	：边刷控制 
*形  参 : id: 3右边刷 4 左边刷  speed : -100~100
*返回值 : 无
***********************************************************************************************/
void Set_Side_Brush_Speed( uint8_t id,int8_t speed) 
{
	if(speed>100) 
	{
		speed=100;
	}
	else if(speed<-100)
	{
		speed=-100;
	}	
	H25_Roller_Brush_Ctr(id,speed*84);   
}
/**********************************************************************************************
*函数名	: H25A_Set_Heartbeat_Time
*介	 绍	：设置驱动器心跳发送时间间隔
*形  参 : time：心跳间隔ms
*返回值 : 无
***********************************************************************************************/
void H25A_Set_Heartbeat_Time(uint32_t time)
{
	CanWriteSDO( H25A_ID, 0x1017, 0x00,  uint32,&time);
}
/**********************************************************************************************
*函数名	: H25A_Set_Disconnect_Time
*介	 绍	：设置驱动器断联告警时间
*形  参 : time：心跳间隔ms
*返回值 : 无
***********************************************************************************************/
void H25A_Set_Disconnect_Time(uint32_t time)
{
	time = time|0x7F0000;
	CanWriteSDO( H25A_ID, 0x1016, 0x01,  uint32,&time);
}
/**********************************************************************************************
*函数名	: H25A_Get_Heartbeat_Time
*介	 绍	：获取驱动器驱动器心跳时间
*形  参 : 无
*返回值 : 驱动器心跳时间
***********************************************************************************************/
uint32_t H25A_Get_Heartbeat_Time(void)
{
	return H25a_Info.heart_time;
}

/**********************************************************************************************
*函数名	: H25A_Get_Link_Status
*介	 绍	：获取驱动器连接状态
*形  参 : 无
*返回值 : 1：连接正常 0：驱动器断连
***********************************************************************************************/
uint8_t H25A_Get_Link_Status(void)
{
	return ((HAL_GetTick() - H25a_Info.data_recv_time) > HEARTBEAT_TIMEOUT_MS) ? 0 : 1;
//	return H25a_Info.link_status;
}

/**********************************************************************************************
*函数名	: H25A_Driver_Info_Read
*介	 绍	：读取驱动器信息
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
#define H4AA_READ_INFO_OFFSET (14) /* 低频数据读取从can字典的15个开始 */ 
void H25A_Driver_Info_Read(void )
{ 	
	static uint8_t index = H4AA_READ_INFO_OFFSET;
	CanReadSDO( H25A_objdict[index].id,H25A_objdict[index].index, H25A_objdict[index].sub_index,H25A_objdict[index].data_type,H25A_objdict[index].p_data);

	if(index++ >= (h25_objdict_size - 1))
	{
		index = H4AA_READ_INFO_OFFSET;
	}
}

/**
 * @brief 读取高频数据
 */
void H4AA_Driver_Info_Read_high_freq_data(void )
{ 	
	static uint8_t index = 0;
	
	CanReadSDO( H25A_objdict[index].id,H25A_objdict[index].index, H25A_objdict[index].sub_index,H25A_objdict[index].data_type,H25A_objdict[index].p_data);

	if(index++ >= (H4AA_READ_INFO_OFFSET - 1))
	{
		index = 0;
	}
}

/**********************************************************************************************
*函数名	: H25A_Push_Motor_Calibration
*介	 绍	：滚刷推杆标定
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void H25A_Push_Motor_Calibration(void)
{
	uint32_t data=0x01;
	CanWriteSDO( H25A_ID, 0x3017, 0x02,  uint8,&data);
}
/**********************************************************************************************
*函数名	: H4AA30_Set_Brush_Push_Current
*介	 绍	：推杆电机电流设置
*形  参 : id:电机编号 current:设置的电流值 单位0.01A
*返回值 : 无
***********************************************************************************************/
void H25A_Set_Brush_Push_Current(uint16_t current)
{
	CanWriteSDO( H25A_ID, 0x3003, 2,  uint16,&current); 	
}

/**********************************************************************************************
*函数名	: H4AA30_Set_Brush_Push_Current
*介	 绍	：推杆电机电流偏差设置
*形  参 : id:电机编号 deviation:设置的偏差值
*返回值 : 无
***********************************************************************************************/
void H25A_Set_Brush_Push_Current_Deviation(uint8_t deviation)
{
	CanWriteSDO( H25A_ID, 0x3009, 1,  uint8,&deviation); 	
}
/**********************************************************************************************
*函数名	: H25A_Set_Brush_Push_Filter_Time
*介	 绍	：推杆电机电流过滤时间设置
*形  参 : id:电机编号 deviation:过滤时间 单位ms
*返回值 : 无
***********************************************************************************************/
void H25A_Set_Brush_Push_Filter_Time(uint16_t time)
{
	CanWriteSDO( H25A_ID, 0x3009, 2,  uint16,&time); 	
}
/**********************************************************************************************
*函数名	: H25A_Error_Clear
*介	 绍	：清除告警
*形  参 : 无
*返回值 : 无
***********************************************************************************************/
void H25A_Error_Clear(void)
{
	uint32_t data=0;
	CanWriteSDO( H25A_ID, 0x300D, 0,  uint32,&data); 	
}

void set_h4aa30_pdo_state(uint8_t enable)
{
	uint8_t data[] = {0x80, H25A_ID};
	if (enable == 0)
		data[0] = 0x80;
	else
		data[0] = 0x01;
	can_standard_send(CAN1_UNIT, 0x00, &data, sizeof(data)/sizeof(data[0]));
}

void handle_h4aa30_pdo_data(uint32_t id, uint8_t *pData, uint8_t len)
{
	uint8_t i = 0;

	for (i = 0; i < H4AA30_PDO_NUM; i++)
	{
		if (id == s_h4aa30_pdo_id_tbl[i])
		{
			s_h4aa30_pdo_id_num[i]++;
			break;
		}
	}
	
	if (i >= H4AA30_PDO_NUM)
	{
		//printf("invalid h4aa30 pdo id.\r\n");
		return;
	}

	memcpy(&s_h4aa30_pdo_data[i][0], pData, len);
}

uint32_t get_h4aa30_pdo_data(uint32_t key)
{
	/* key/value中的value是uint32_t类型的，使用两个key/value传一个PDO数据，offset代表偏移0还是4 */
	uint8_t index = key/2;
	uint8_t offset = (key%(CAN_DATA_LEN/sizeof(uint32_t)))*sizeof(uint32_t);
	
	if (key > CAN_DATA_LEN/sizeof(uint32_t)*H4AA30_PDO_NUM)
		return 0;

	return s_h4aa30_pdo_data[index][offset] << 24 | s_h4aa30_pdo_data[index][offset+1] << 16 | 
			s_h4aa30_pdo_data[index][offset+2] << 8 | s_h4aa30_pdo_data[index][offset+3];
}

