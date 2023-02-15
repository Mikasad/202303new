#ifndef __DS402_H__
#define __DS402_H__
//#include "public_h.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
//#include "public_global.h"
#include "canopen_od.h"

typedef struct struct_Drive_402_Data  Drive_Data;

/* Condition Define */
#define TRUE  0x1
#define FALSE 0x0

/* StatuWord Bit Define[0x6041] */
#define Bit_Not_Ready_To_Switch_On (0<<0)
#define Bit_Ready_to_switch_on     (1<<0)
#define Bit_Switched_on            (1<<1)
#define Bit_Operation_enabled      (1<<2)
#define Bit_Fault                  (1<<3)
#define Bit_Voltage_enabled        (1<<4)
#define Bit_Quick_stop             (1<<5)
#define Bit_Switch_on_disabled     (1<<6)
#define Bit_Warning                (1<<7)
#define Bit_Manufacturer_specific  (1<<8)
#define Bit_Remote                 (1<<9)
#define Bit_Target_reached         (1<<10)
#define Bit_Internal_limit_active  (1<<11)
/* Bit12-13 Operation Mode Specific */
/*  
                vl        pp                    pv                    tq        hm                ip
    Bit12    reserved  SetPoint-Acknowledege  Speed                reserved   Homing-Attained    Ip-Mode-Active 
    Bit13    reserved  Fllowing-Error         Max-Slippage-error   reserved   Homing-Error       reserved
*/
/*PP*/
#define Bit_Set_Point_Acknowledge (1<<12)
#define Bit_Following_Error       (1<<13)
/*pv*/
#define Bit_Speed                 (1<<12)
#define Bit_Max_Slippage_Error    (1<<13)
/*hm*/
#define Bit_Homing_Attained       (1<<12)
#define Bit_Homing_Error          (1<<13)
/*ip*/
#define Bit_Ip_Mode_Active        (1<<12)
/* Specific define */
#define Bit_Motor_En              (1<<13)
/* Bit14-15 Manufacturer Specific */

/* ContrlWord Bit Define[0x6040] */
#define Bit_Switch_On               (1<<0)
#define Bit_Enable_Voltage          (1<<1)
#define Bit_Quick_Stop              (1<<2)
#define Bit_Enable_Operation        (1<<3)
#define Bit_Operation_Mode_Specific (1<<4)
#define Bit_Fault_Reset             (1<<7)
#define Bit_Halt                    (1<<8)
#define Bit_Reserved                (0<<0)
#define Bit_Manufacturer_Specific   (0<<0)

/* ControlWord Commands Mask[0x6040] */
#define CONTROLWORD_COMMAND_SHUTDOWN_MASK                    0x0087
#define CONTROLWORD_COMMAND_SWITCHON_MASK                    0x00C7 
#define CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION_MASK    0x008F 
#define CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK              0x0082 
#define CONTROLWORD_COMMAND_QUICKSTOP_MASK                   0x0086 
#define CONTROLWORD_COMMAND_DISABLEOPERATION_MASK            0x008F
#define CONTROLWORD_COMMAND_ENABLEOPERATION_MASK             0x008F 
#define CONTROLWORD_COMMAND_FAULTRESET_MASK                  0x0080
#define CONTROLWORD_COMMAND_NEWSETPOINT_MASK		             0x009f
#define CONTROLWORD_COMMAND_HALT_MASK                        0x0100

/* Controlword Command Specifier  */
#define CONTROLWORD_COMMAND_SHUTDOWN                         0x0006
#define CONTROLWORD_COMMAND_SWITCHON                         0x0007
#define CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION         0x000f
#define CONTROLWORD_COMMAND_DISABLEVOLTAGE                   0x0000
#define CONTROLWORD_COMMAND_QUICKSTOP                        0x0002
#define CONTROLWORD_COMMAND_DISABLEOPERATION                 0x0007
#define CONTROLWORD_COMMAND_ENABLEOPERATION                  0X000f
#define CONTROLWORD_COMMAND_FAULTRESET                       0x0080
#define CONTROLWORD_COMMAND_ENABLE_MOTOR_START_OPERATION     0X001f
#define CONTROLWORD_COMMAND_NEWSETPOINT                      0X001f
#define CONTROLWORD_COMMAND_HALT                             0x0100

/* StatusWord In Diffrent State */
/* Bit0-3,5,6 */
#define STATUSWORD_STATE_NOTREADYTOSWITCHON                  0x0000 
#define STATUSWORD_STATE_SWITCHEDONDISABLED                  0x0040 
#define STATUSWORD_STATE_READYTOSWITCHON                     0x0021
#define STATUSWORD_STATE_SWITCHEDON                          0x0023
#define STATUSWORD_STATE_OPERATIONENABLED                    0x0027
#define STATUSWORD_STATE_QUICKSTOPACTIVE                     0x0007
#define STATUSWORD_STATE_FAULTREACTIONACTIVE                 0x000F 
#define STATUSWORD_STATE_FAULT                               0x0008

/* StatusWord Masks and Flags */
#define STATUSWORD_STATE_MASK                                0x006F
/* Bit4 当高压应用于驱动器时,将该位置1 */
#define STATUSWORD_VOLTAGE_ENABLED                           0x0010
/* Bit7 驱动器警告时将该位置1，E.G.温度限制.驱动器状态不会改变 */
#define STATUSWORD_WARNING                                   0x0080
/* Bit8 制造商定义 */
#define STATUSWORD_MANUFACTORSPECIFIC                        0x0100
#define STATUSWORD_INTERNAL_LIMIT                            0x0800
/* Bit9 该位置起，可通过CAN网络对参数进行修改 */
#define STATUSWORD_REMOTE                                    0x0200
/* Bit10 当该位置起时，表示驱动器达到了一个设定值。取决于工作状态。当在急停状态下，当驱动器已经停止，该位也被置起 */
#define STATUSWORD_TARGET_REACHED                            0x0400 
/* Bit11 驱动器内部定义，该位在某种限制状态下被置起，E.G.位置正反限位 */
#define STATUSWORD_INTERNALLIMITACTIVE                       0x0800
#define STATUSWORD_DRIVE_FOLLOWS_COMMAND                     0x1000 

#define DISABLE_DRIVE                    0 /**< \brief Disable drive (options: 0x605B; 0x605C; 0x605E)*/
#define SLOW_DOWN_RAMP                   1 /**< \brief Slow down ramp (options: 0x605B; 0x605C; 0x605E)*/
#define QUICKSTOP_RAMP                   2 /**< \brief Quick stop ramp (options: 0x605E)*/
#define STOP_ON_CURRENT_LIMIT            3 /**< \brief Stop on current limit (options: 0x605E)*/
#define STOP_ON_VOLTAGE_LIMIT            4 /**< \brief Stop on voltage limit (options: 0x605E)*/
#define SLOW_DOWN_RAMP_STAY              5
#define QUICKSTOP_RAMP_STAY              6
#define STOP_ON_CURRENT_LIMIT_STAY       7
#define STOP_ON_VOLTAGE_LIMIT_STAY       8
/* Opcode Define */
#define QUICK_STOP_CODE         0x605A
#define SHUTDOWN_CODE           0x605B
#define DISABLE_OPERATION_CODE  0x605C
#define FAULT_REACT_CODE        0x605E
/* EMCY Stop */
#define QUICK_STOP_ENABLE  0x8
#define QUICK_STOP_DISABLE 0x0

/* State machine */
enum enum_Drive_System_State 
{
	State_Start							= 0x00, 
	State_Not_ready_to_switch_on		= 0x01,
	State_Switch_on_disabled			= 0x02,
	State_Ready_to_switch_on			= 0x03,
	State_Switched_on					= 0x04,
	State_Operation_enable		        = 0x05,	
	State_Quick_stop_active		        = 0x06,
	State_Fault_reaction_active			= 0x07,
	State_Fault							= 0x08
};
typedef enum enum_Drive_System_State e_drivesytemstate;

/*Modes of operation*/
/*Object dictionary is 0x6060h*/
enum enum_Modes_Operation
{
	reserved1							= 0,			
	Profile_Position_Mode				= 1,
	Velocity_Mode						= 2,
	Profile_Velocity_Mode				= 3,
	Torque_Profile_Mode					= 4, 
	reserved2							= 5,
	Homing_Mode						= 6,
	Interpolated_Position_Mode			= 7
};
typedef enum enum_Modes_Operation e_modes_operation;

/* common data */
struct struct_Common_Data
{
	uint8_t commnomdata;
};
typedef struct struct_Common_Data s_common_data;

/* device control data  */
struct struct_Device_control_Data
{
	uint16_t Conrtolword; 							    //控制字6040h
	uint16_t Statusword;							      //状态字6041h
	int16_t Shutdown_option_code;				    //关机选项代码605Bh 
	int16_t Disable_operation_option_code;	//关失能选项代码605Ch 
	int16_t Quick_stop_option_code;				  //急停选项代码605Ah 
	int16_t Halt_option_code;						    //暂停选项代码605Dh 
	int16_t Fault_reaction_option_code;			//错误应对选项代码O605Eh 
	int8_t Modes_of_operation;					    //操作模式6060h 
	int8_t Modes_of_operation_display;			//操作模式显示6061h
	int8_t last_Modes_of_operation;	        //上一次操作模式
	uint16_t Pending_Option_code;	          //即将执行的选项代码
	uint8_t bAxisIsActive;                  //
	uint8_t bBrakeApplied;                  //急停被应用
	uint8_t bLowLevelPowerApplied;          //低压被应用
	uint8_t bHighLevelPowerApplied;         //高压被应用
	uint8_t bAxisFunctionEnabled;           //功能被使能
	uint8_t bConfigurationAllowed;          //参数允许配置
};
typedef struct struct_Device_control_Data s_device_control_data;

/* Profile position mode */
struct struct_PP_Mode_Data
{
	uint8_t Polarity;														//607eh
	uint32_t Position_factor;										//6093h
	uint32_t Velocity_encoder_factor;						//6094h
	uint32_t Velocity_factor;										//6095h 
	uint32_t Acceleration_factor;								//6097h
	
	uint32_t Position_demand_value;							//6062h
	uint32_t Position_actual_internal_value;		//6063h
	uint32_t Position_actual_value;							//6064h
	uint32_t Following_error_window;						//6065h
	uint32_t Position_window;										//6067h
	uint16_t Position_window_time;							//6068h
	
	
	int32_t Target_position;										//607ah
	int32_t Position_range_limit;								//607bh
	int32_t Software_position_limit;						//607dh
	uint32_t Max_profile_velocity;							//607fh
	uint32_t Max_motor_speed;										//6080h
	uint32_t Profile_velocity;									//6081h
	uint32_t Profile_acceleration;							//6083h
	uint32_t Profile_deceleration;							//6084h
	uint32_t Quick_stop_deceleration;						//6085h
	uint16_t Motion_profile_type;								//6086h
	uint32_t Max_acceleration;									//60c5h
	uint32_t Max_deceleration;									//60c6h
};
typedef struct struct_PP_Mode_Data s_pp_mode_data;

/* Homing mode */
struct struct_Homing_Mode_Data
{
	int32_t Home_offset;												//6062h
	int8_t Homing_method;												//6063h
	uint32_t Homing_speeds;											//6064h
	uint32_t Homing_acceleration;								//6065h
};
typedef struct struct_Homing_Mode_Data s_homing_mode_data;

/* Velocity mode */
struct struct_Velocity_Mode_Data
{	
	int16_t target_velocity;										//6042h
	int16_t velocity_demand;										//6043h
	int16_t velocity_actual_value;							//6044h
	int16_t manipulated_velocity;								//6045h
	uint32_t velocity_min_max_amount;						//6046h
	uint32_t velocity_min_max;									//6047h
	uint16_t velocity_acceleration;							//6048h
	uint16_t velocity_deceleration;							//6049h
};
typedef struct struct_Velocity_Mode_Data s_velocity_mode_data;

/*Drive about Cia dsp-402 profile data*/
struct struct_Drive_402_Data
{
	/* State machine */
	e_drivesytemstate drivesytemState;
	
	/* modes of operation */
	e_modes_operation modes_operation;
	
	/* common data */
	s_common_data common_data;
	
	/* device control data */
	s_device_control_data device_control_data;
	
	/* pp_mode_data */
	s_pp_mode_data pp_mode_data;
	
	/* homing_mode_data */
	s_homing_mode_data homing_mode_data;
	
	/* velocity_mode_data */
	s_velocity_mode_data velocity_mode_data;
};

extern Drive_Data Driver_Data;


uint8_t SetStateWord(Drive_Data* dd,uint16_t setState);
uint8_t ClrStateWord(Drive_Data* dd,uint16_t clrState);
void ProceedDriveStateChange(Drive_Data* dd);
void ProceedDriverHandler(Drive_Data* dd);
uint8_t CiA402_TransitionAction(uint16_t Characteristic);
void PendingOptionCode(Drive_Data *dd);

#endif
