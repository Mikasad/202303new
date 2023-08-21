#ifndef __DRV_ZLAC8015_H__
#define __DRV_ZLAC8015_H__

//#include "stm32f4xx_hal.h"
#include "hc32_ll.h"
#include "objdict.h"

#define ZLAC_ID			0x05	//中菱驱动器ID

#define ZLAC_PDO_NUM  4
#define CAN_DATA_LEN  8

typedef struct
{
	uint16_t communication_mode;	//通讯模式
	uint32_t left_accelerate_time;	//左轮加速时间
	uint32_t right_accelerate_time;	//右轮速度时间
	uint32_t left_dccelerate_time;	//左轮减速时间
	uint32_t right_dccelerate_time;	//右轮减速时间
	uint32_t soft_version;			//软件版本
	uint16_t control_word; 	 		//控制字
	uint16_t status_word;			//状态字
	uint16_t status_word_m2;
	uint8_t  work_mode;				//工作模式
	int32_t left_odom_count;		//左轮位置反馈   
	int32_t right_odom_count;		//右轮位置反馈
	int32_t left_odom_dcount;		//左轮位置反馈   
	int32_t right_odom_dcount;		//右轮位置反馈
	int32_t left_real_speed;		// 
	int32_t right_real_speed;		// 
	int32_t left_target_speed;		// 
	int32_t right_target_speed;		// 
	int32_t left_target_power;		// 
	int32_t right_target_power;		// 
	int32_t driver_vol;		// 
	int32_t left_temperature;		//左轮温度   
	int32_t right_temperature;		//右轮温度
	int32_t driver_temperature;		//驱动器温度
	uint32_t left_error_code;		//左轮告警码
	uint32_t right_error_code;		//右轮告警码
	uint32_t heart_time;			//用于记录收到心跳包的时间，用于判断断连
	uint32_t link_status;			//连接状态0通讯断连 1通讯正常
	int16_t left_motor_current;		//左轮电机电流
	int16_t right_motor_current;	//右轮电机电流
	uint16_t offline_time;			//掉线保护时间
	uint16_t save_all_param;		//保存所有参数
	int16_t speed_current_limit;    //速度环最大电流限制 
	int32_t left_target_torque;
	int32_t right_target_torque;
	uint16_t right_control_word;
	uint16_t right_status_word;
	uint8_t left_actual_mode;
	uint8_t right_actual_mode;
 }ZLAC_MOTOR_T;

extern ZLAC_MOTOR_T ZLAC_Motor;
extern const Objdict_t ZLAC_objdict[];
extern const uint16_t zlac_objdict_size;

/*设置驱动工作模式*/
void ZALC_Set_Work_Mode(uint8_t mode);
/*读驱动工作模式*/
void ZALC_Read_Work_Mode(void);
/*设置通讯模式*/
void ZALC_Set_Synchronous(uint8_t mode);
/*设置加速度时间*/
void ZALC_Set_Acceleration(uint32_t time);
/*设置减速度时间;*/
void ZALC_Set_Dcceleration(uint32_t time);
/*设置驱动器心跳发送时间间隔*/
void ZALC_Set_Heartbeat_Time(uint32_t time);
/*驱动器使能失能控制*/
void ZALC_Driver_Enable(uint8_t enable);
/*清除驱动器错误*/
void ZALC_Set_Claern_Error( uint8_t clean_flag);
/*初始化驱动器为速度模式*/
void ZALC_Motor_Init(void);
/*设置左右轮电机速度*/
void ZALC_Set_Speed(uint32_t left_speed,uint32_t right_speed);
/*清除0x603F的告警信息*/
void ZALC_Error_Clear(void);
/*读驱动器软硬件版本*/
void ZALC_Read_Versions(void);
/*读驱动器编码器计数*/
void ZALC_Read_Real_Speed(void);
/*读左右轮编码器值*/
void ZALC_Read_Odom_Count(void);
/*读驱动器温度和轮毂电机温度*/
void ZALC_Read_Temperature(void);
/*读驱动器控制字和状态字*/
void ZALC_Read_Status(void);
/*读驱动器通讯模式*/
void ZALC_Read_Communicate_mode(void);
/*读电机速度环最大电流限制参数*/
void ZALC_Read_Current_Limit(void);
/*读故障信息*/
void ZALC_Read_Error(void);
/*读取驱动器信息*/
void ZALC_Read_Driver_Info(void);
/*读取驱动器信息，不重要的数据较长时间读一次，需要频率较高读取的单独读取*/
void ZALC_Cycle_Read_Driver_Info(void);
/*刷新中菱驱动器连接计数时间*/
void Updata_ZALC_Link_Tick(void);
/*获取驱动器连接状态*/
uint8_t ZALC_Get_Link_Status(void);
/*计算左轮编码器每50ms的差值,每次收到编码器数据后调用*/
void ZALC_Left_Odom_Difference_Callback(void* param);
/*计算右轮编码器每50ms的差值,每次收到编码器数据后调用*/
void ZALC_Right_Odom_Difference_Callback(void* param);

/*添加中菱驱动器对象字典*/
void ZLAC_Obj_Load(void);

void ZALC_Read_Control_Word_Status(void);

void ZALC_Set_Stop_Enable(uint8_t enable);

void ZALC_Set_Offline_Time(uint16_t time);

void ZLAC_Read_Motor_Current(void);

void ZLAC_Save_All_Param(uint8_t enable);

uint32_t GetMotorLeftErrorCode(void);

uint32_t GetMotorRightErrorCode(void);

uint8_t Get_Homemade_LeftHealthState(void);

uint8_t Get_Homemade_RightHealthState(void);
/*阻尼模式开关 0 关闭 1打开*/
void ZLAC_Damping_Mode_Switch(uint8_t operate);
/*设置电机工作模式	1:位置模式  3:速度模式 4:力矩模式*/
void ZLAC_Work_Mode_Set(uint8_t operate);
/*配置电机速度环最大电流限制参数*/
void ZALC_Read_Current_Limit_Set(int value);

void set_zlac_pdo_state(uint8_t enable);
void handle_zlac_pdo_data(uint32_t id, uint8_t *pData, uint8_t len);
uint32_t get_zlac_pdo_data(uint32_t key);
void ZALC_Cycle_Read_Driver_Info_high_data(void);

#endif

