#ifndef __DS402_H__
#define __DS402_H__
//#include "public_h.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
//#include "public_global.h"
#include "canopen_od.h"
#include "mc_tasks.h"
#include "mc_config.h"
typedef struct struct_Drive_402_Data  Drive_Data;

/* Condition Define */
#define TRUE  0x1
#define FALSE 0x0
#define MAX_AXES 2 //轴个数
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
//=========================================================================================================
//
//	Status register (glStatReg) definitions and bits
//
#define	STAT_REG_COMMUTATION_BIT							0
#define	STAT_REG_COMMUTATION_SET							0x00000001
#define	STAT_REG_COMMUTATION_CLEAR						0xFFFFFFFE

#define	STAT_REG_REGENERATION_BIT							1
#define	STAT_REG_REGENRATION_SET							0x00000002
#define	STAT_REG_GENERATION_CLEAR							0xFFFFFFFD

#define	STAT_REG_REGENERATION_FEEDBACK_BIT		2
#define	STAT_REG_REGENERATION_FEEDBACK_SET		0x00000004
#define	STAT_REG_REGENERATION_FEEDBACK_CLEAR	0xFFFFFFFB

#define	STAT_REG_OVER_MAX_VBUS_BIT						3
#define	STAT_REG_OVER_MAX_VBUS_SET						0x00000008
#define	STAT_REG_OVER_MAX_VBUS_CLEAR					0xFFFFFFF7

#define	STAT_REG_UNDER_MIN_VBUS_BIT						4
#define	STAT_REG_UNDER_MIN_VBUS_SET						0x00000010
#define	STAT_REG_UNDER_MIN_VBUS_CLEAR					0xFFFFFFEF

#define	STAT_REG_AMPLIFIER_MAIN_RELAY_BIT			5
#define	STAT_REG_AMPLIFIER_MAIN_RELAY_SET			0x00000020
#define	STAT_REG_AMPLIFIER_MAIN_RELAY_CLEAR		0xFFFFFFDF


#define	STAT_REG_TORMOD_VEL_LIMITING_BIT0_BIT		13
#define	STAT_REG_TORMOD_VEL_LIMITING_BIT0_SET		0x00002000
#define	STAT_REG_TORMOD_VEL_LIMITING_BIT0_CLEAR	0xFFFFDFFF


#define	STAT_REG_RLS_BIT											17
#define	STAT_REG_RLS_SET											0x00020000
#define	STAT_REG_RLS_CLEAR										0xFFFDFFFF

#define	STAT_REG_FLS_BIT											18
#define	STAT_REG_FLS_SET											0x00040000
#define	STAT_REG_FLS_CLEAR										0xFFFBFFFF

#define	STAT_REG_BOTH_LIMITS_CLEAR						0xFFF9FFFF

#define	STAT_REG_REVPLIM_BIT									19
#define	STAT_REG_REVPLIM_SET									0x00080000
#define	STAT_REG_REVPLIM_CLEAR								0xFFF7FFFF

#define	STAT_REG_FWDPLIM_BIT									20
#define	STAT_REG_FWDPLIM_SET									0x00100000
#define	STAT_REG_FWDPLIM_CLEAR								0xFFEFFFFF
#define	STAT_REG_BOTH_PLIM_CLEAR							0xFFE7FFFF

#define	STAT_REG_CURR_SAT_BIT									21
#define	STAT_REG_CURR_SAT_SET									0x00200000
#define	STAT_REG_CURR_SAT_CLEAR								0xFFDFFFFF

#define	STAT_REG_VOLTAGE_SAT_BIT							22
#define	STAT_REG_VOLTAGE_SAT_SET							0x00400000
#define	STAT_REG_VOLTAGE_SAT_CLEAR						0xFFBFFFFF

#define	STAT_REG_VEL_SAT_BIT									23
#define	STAT_REG_VEL_SAT_SET									0x00800000
#define	STAT_REG_VEL_SAT_CLEAR								0xFF7FFFFF


#define	STAT_REG_FILTERS_MODIFIED_BIT					26
#define	STAT_REG_FILTERS_MODIFIED_SET					0x04000000
#define	STAT_REG_FILTERS_MODIFIED_CLEAR				0xFBFFFFFF


#define	STAT_REG_CALC_FILTERS_FAILED_BIT			27
#define	STAT_REG_CALC_FILTERS_FAILED_SET			0x08000000
#define	STAT_REG_CALC_FILTERS_FAILED_CLEAR		0xF7FFFFFF


#define	STAT_REG_DYNAMIC_BRAKE_STATUS_BIT			28					//todo:20190905
#define	STAT_REG_DYNAMIC_BRAKE_STATUS_SET			0x10000000
#define	STAT_REG_DYNAMIC_BRAKE_STATUS_CLEAR		0xEFFFFFFF


#define	STAT_REG_ENCODER_READY_STATUS_BIT			30
#define	STAT_REG_ENCODER_READY_STATUS_SET			0x40000000		//todo:20190905 编码器准备好状态
#define	STAT_REG_ENCODER_READY_STATUS_CLEAR		0xBFFFFFFF
//
#define	STAT_REG_MANY_BITS_CLEAR							0xFF07E027			// 0xFFE7E027
//==========================================================================
//
//	Motion status (of glMotionStat)
//
#define	NOT_IN_MOTION									0
//
#define IN_MOTION_BIT									0
#define	IN_MOTION_BIT_SET							0x00000001
#define	IN_MOTION_BIT_CLEAR						0xFFFFFFFE

#define IN_WAITING_BIT								1
#define	IN_WAITING_BIT_SET						0x00000002
#define	IN_WAITING_BIT_CLEAR					0xFFFFFFFD

#define IN_REPETITIVE_STOP_BIT				2
#define	IN_REPETITIVE_STOP_BIT_SET		0x00000004
#define	IN_REPETITIVE_STOP_BIT_CLEAR	0xFFFFFFFB

#define IN_STOP_REQUEST_BIT						3
#define	IN_STOP_REQUEST_BIT_SET				0x00000008
#define	IN_STOP_REQUEST_BIT_CLEAR			0xFFFFFFF7

#define	IN_ACCELERATION_BIT						4
#define	IN_ACCELERATION_BIT_SET				0x00000010
#define	IN_ACCELERATION_BIT_CLEAR			0xFFFFFFEF

#define	IN_DECELERATION_BIT						5
#define	IN_DECELERATION_BIT_SET				0x00000020
#define	IN_DECELERATION_BIT_CLEAR			0xFFFFFFDF

#define	IN_ACC_DEC_BITS_SET						0x00000030
#define	IN_ACC_DEC_BITS_CLEAR					0xFFFFFFCF

#define	IN_WAIT_END_SMOOTH_BIT				6
#define	IN_WAIT_END_SMOOTH_BIT_SET		0x00000040
#define	IN_WAIT_END_SMOOTH_BIT_CLEAR	0xFFFFFFBF

#define IN_ECAM_STOP_BIT							7
#define	IN_ECAM_STOP_BIT_SET					0x00000080
#define	IN_ECAM_STOP_BIT_CLEAR				0xFFFFFF7F

#define IN_FIFO_STOP_BIT							8
#define	IN_FIFO_STOP_BIT_SET					0x00000100
#define	IN_FIFO_STOP_BIT_CLEAR				0xFFFFFEFF

#define IN_WAIT_FOR_INPUT_BIT					9
#define	IN_WAIT_FOR_INPUT_BIT_SET			0x00000200
#define	IN_WAIT_FOR_INPUT_BIT_CLEAR		0xFFFFFDFF
//
#define	ALL_IN_MOTION_BITS_CLEAR			0xFFFFFC00			// All motion related bits are cleared
#define	IN_MOTION_AND_STOP_BITS_SET		0x00000009
//
//	End of motion reasons (of glMotionReason)
/* ControlWord Commands Mask[0x6040] */
#define CONTROLWORD_COMMAND_SHUTDOWN_MASK                    0x0087
#define CONTROLWORD_COMMAND_SWITCHON_MASK                    0x00C7
#define CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION_MASK    0x008F
#define CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK              0x0082
#define CONTROLWORD_COMMAND_QUICKSTOP_MASK                   0x0086
#define CONTROLWORD_COMMAND_DISABLEOPERATION_MASK            0x008F
#define CONTROLWORD_COMMAND_ENABLEOPERATION_MASK             0x008F
#define CONTROLWORD_COMMAND_FAULTRESET_MASK                  0x0080
#define CONTROLWORD_COMMAND_ABS_MASK		         				     0XFFEF
#define CONTROLWORD_COMMAND_RELATIVE_MASK                    0XFFBF  //掩码设置为多少是个问题？
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
#define CONTROLWORD_COMMAND_ABS         				             0X001f   //绝对位置指令
#define CONTROLWORD_COMMAND_RELATIVE    										 0X004f   //相对位置指令
#define CONTROLWORD_COMMAND_HALT                             0x0100   //厂商自定义
#define CONTROLWORD_EMPTY																		 0x1111


/* StatusWord In Diffrent State */
/* Bit0-3,5,6 */
#define STATUSWORD_STATE_NOTREADYTOSWITCHON                  0x0000 //初始化状态
#define STATUSWORD_STATE_SWITCHEDONDISABLED                  0x0040 //伺服无故障状态
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
    Velocity_Mode						= 3,
    Profile_Velocity_Mode				= 2,
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
    uint8_t bAxisIsActive;                  //该轴需要执行402状态机
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
    uint8_t  Polarity;														//607eh
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
void Par_402_Init(void);
extern Drive_Data Driver_Data[MAX_AXES];
extern STM_Handle_t STM[NBR_OF_MOTORS];
uint8_t SetStateWord(Drive_Data* dd,uint16_t setState);
uint8_t ClrStateWord(Drive_Data* dd,uint16_t clrState);
void ProceedDriveStateChange(void);
void ProceedDriverHandler(void);
u8 CiA402_TransitionAction(u16 Characteristic,u8 Axis);
void PendingOptionCode(void);

#endif
