#ifndef __DRV_H25A15_H__
#define __DRV_H25A15_H__

#include "main.h"
#include "stdio.h"
#include "objdict.h"

#define H25A_ID 1

#define H4AA30_PDO_NUM  4
#define CAN_DATA_LEN    8
#define H4AA30_SDO_NUM  16 //SDO上传数据个数，由混合驱控提供

#define HEARTBEAT_TIMEOUT_MS    1000

//电压信息结构体
typedef struct
{
  uint16_t GetADCValue;//获取的ADC值
  uint16_t FilterValue;//滤波后的ADC值
  uint16_t ConversionFactor; //转化系数
  uint16_t ActualUnitValue;  //转换为实际单位值
   
}ADC_ValueParameters_t;

//电机结构体
typedef struct  BLDC_Motor
{
  uint8_t enable;
   int32_t targetPWM;
  ADC_ValueParameters_t current;
  int16_t temperature;
  uint8_t states;
  uint8_t fault;
  uint8_t calibration;

}MOTOR_PARAMETER_T;

typedef struct
{
	uint32_t soft_version;			//软件版本
	uint32_t hard_version;			//软件版本
	uint32_t data_recv_time;        //数据接收时间，用于判断断连
	uint32_t heart_time;			//用于记录收到心跳包的时间
	uint32_t link_status;			//连接状态0通讯断连 1通讯正常
	uint32_t error_code;            //驱动器告警
	uint32_t error_code_sub;        //驱动器告警
	uint16_t  brush_self_lift_deviation;
	uint16_t  brush_self_lift_time;

}H25A_MOTOR_T;

typedef struct
{
	int16_t bus_voltage;
	uint8_t roller_lift_position_loop;
    uint16_t dust_lift_positon;
	uint8_t dust_lift_calib;
	int8_t brush_type;
	int16_t motor2_current;
    int16_t roller_brush_pwm;
}H4AA30_INFO_T;

extern const Objdict_t H25A_objdict[];
extern const uint16_t h25_objdict_size;
extern H25A_MOTOR_T H25a_Info;
extern H4AA30_INFO_T g_h4aa30_sdo_data;

extern MOTOR_PARAMETER_T RollerBrushMotor1;		//J6端口无刷驱动器1，前滚刷
extern MOTOR_PARAMETER_T RollerBrushMotor2;		//J7端口无刷驱动器2，后滚刷
extern MOTOR_PARAMETER_T PushRodMotor1;			//J1端口有刷H桥1，边刷推杆 
extern MOTOR_PARAMETER_T PushRodMotor2;			//J2端口有刷H桥2，滚刷推杆
extern MOTOR_PARAMETER_T PushRodMotor3;			//J3端口有刷H桥3，吸水趴推杆
/*添加对象字典*/
void LoadH25aObj(void);
/*电机使能、失能控制*/
void H25_Motor_Enable(uint8_t id,uint8_t enable);
/*电机PWM输出控制*/
void H25_Set_Motor_Pwm(uint8_t id,uint16_t pwm) ;
/*设置驱动器无刷电机速度*/
void H25_Set_Motor_Speed(uint8_t id,int16_t speed) ;
/*电机PWM输出控制*/
void H4AA30_Set_Motor_Pwm(uint8_t id,uint16_t pwm);
/*滚刷推杆电机控制，有位置控制*/
void H25A_Brush_Push_Rod_Ctr(uint8_t id , uint16_t location);
/*推杆电机控制*/
void H25_Push_Rod_Ctr(uint8_t id,uint8_t status);
/*滚刷边刷电机控制*/
void H25_Roller_Brush_Ctr(uint8_t id,int16_t speed) ;
/*水泵控制*/
void H25_Water_Pump_Ctr(uint16_t pwm) ;
/*风机控制*/
void H25_Fan_Motor_Ctr(uint16_t speed) ;
/*风机控制*/
void Set_Fan_Motor_Speed(uint8_t  speed) ;
/*水泵控制*/
void Set_Water_Pump_Speed(uint8_t  speed) ;
/*滚刷控制*/
void Set_Roller_Brush_Speed( uint8_t id,int8_t speed) ;
/*边刷控制*/
void Set_Side_Brush_Speed( uint8_t id,int8_t speed) ;
/*设置驱动器心跳发送时间间隔*/
void H25A_Set_Heartbeat_Time(uint32_t time);
/*设置驱动器断联告警时间*/
void H25A_Set_Disconnect_Time(uint32_t time);
/*获取连接状态*/
uint8_t H25A_Get_Link_Status(void);
/*读取驱动器信息*/
void H25A_Driver_Info_Read(void );
/*滚刷推杆标定*/
void H25A_Push_Motor_Calibration(void);
/*推杆电机电流设置*/
void H25A_Set_Brush_Push_Current(uint16_t current);
/*推杆电机电流偏差设置*/
void H25A_Set_Brush_Push_Current_Deviation(uint8_t deviation);
/*推杆电机电流过滤时间设置*/
void H25A_Set_Brush_Push_Filter_Time(uint16_t time);
/*清除告警*/
void H25A_Error_Clear(void);

void set_h4aa30_pdo_state(uint8_t enable);
void handle_h4aa30_pdo_data(uint32_t id, uint8_t *pData, uint8_t len);
uint32_t get_h4aa30_pdo_data(uint32_t key);
void H4AA_Driver_Info_Read_high_freq_data(void );


#endif

