#ifndef _NANOPB_TCPIP_H
#define _NANOPB_TCPIP_H
//#include "w5500.h"
#include <stdint.h>

#define _GLOBALS


#define NONOPD_BUF_LEN 512




//extern DeviceData DeviceDataMsg;
extern struct CONFIGCOMMAND_CMD ConfigCommandCMD;
extern struct DEVICECOMMAND_CMD DeviceCommandCMD;
extern struct DEVICE_DATA      DeviceDATA;
extern struct SHUTDOWN_CMD    ShutDown_CMD;
extern struct OTHER_DEVICE_DATA  Other_DeviceData;
extern uint8_t speed_counts;
extern uint32_t tcpic_connect_count;
extern uint32_t w5500_reset_count,w5500_reset_stepa_flg;

extern uint8_t nanopb_buf[NONOPD_BUF_LEN];
extern uint16_t g_nanopb_buf_len;

extern int32_t mySetSpdLeft;
extern int32_t mySetSpdRight;

extern uint8_t is_hardware_type_configed;
extern uint8_t is_motor_driver_type_configed;

enum State
{
  RX_HEAD_MSG =1,
  RX_VERIFY_MSG,
  RX_SHUTDOWN_MSG,
  RX_CONFIG_CMD,
  RX_MCM_CMD,
  RX_MMD_CMD,
  RX_DEVICE_CMD,
  RX_REMOTE_CMD433,
  RX_HEALTH_DATA,
  RX_VERSION_DATA,
  RX_DEVICE_DATA,
  RX_ODOMETRY_DATA,
  RX_IMU_DATA,
  RX_ULATRASONIC_DATA,
  RX_UWB_DATA,
  RX_REMOTE_DATA433,
  RX_PROTECTER_DATA,
  RX_RFID_DATA,
  RX_ANTIDROP_DATA,
  RX_CLEAR_ERROR_DATA,
  RX_DRIVER_CONFIG_DATA,
  RX_MCU_UPDATE_CMD,
  RX_CANBUS_COMMAND = 42,
  RX_PHANTAS_MCU_UPGRADE_COMMAND = 45,
  RX_PHANTAS_MCU_UPGRADE_PACKAGE = 46,
  RX_CANBUS2_COMMAND = 50,  
};

enum MotionControlMotorCommand_ID{
  
  MotionControlMotorCommand_Low = 0,
  MotionControlMotorCommand_High,
  AKM_LINE_SPEED = 10,//阿克曼结构的线速度，即后轮电机速度  
  AKM_ANGLE = 11,//阿克曼结构的角度，即前轮的转向角度
  AKM_ANGLE_SPEED = 12,//阿克曼结构的角速度，即前轮的转向速度

    
};
struct DEVICECOMMAND_CMD{
  uint32_t  Do;     
  uint32_t  Relay;
  uint32_t  Braker_Down;   //ִ执行此指令时，放下所有刹车，所有。。
  uint32_t  Auto_Mode;
//  uint32_t  Track_Motor;
//  uint32_t  Steering_Motor;
  uint32_t  Brush_Down;
  uint32_t  Brush_Motor;
  uint32_t  Left_Brush_Motor;
  uint32_t  Right_Brush_Motor;
  uint32_t  Squeegee_Down;
  uint32_t  Squeegee_Motor;
  uint32_t  Spray_Motor;
  uint32_t  Valve;
  uint32_t  Water_Level;
  uint32_t  Fan_Levle;
  uint32_t  keep_vaccum;
  uint32_t  brush_spin_level;
  uint32_t  brush_pressure_level;
  uint16_t  brush_lift_poisition;
  uint32_t  filter_motor_pwm;
  uint32_t  outlet_cmd;
  uint32_t  work_status;
  uint32_t  motor_mode;
  uint32_t  pc_current_time;
  uint32_t  side_brush_down;
  uint32_t  left_brush_pwm;
  uint32_t  ritht_brush_pwm;
  uint32_t  side_brush_enable;
  uint32_t  dust_push_spin_level;
  uint32_t  dust_push_clean_spin_level;
  uint8_t   anti_drop_switch;
  uint8_t   clear_emergency;
  uint8_t   set_emergency;
  uint8_t   disinfect_mist_spray_level;
  uint8_t   disinfect_spray_distance;
  uint8_t   disinfect_box_beep;
  uint8_t   disinfect_disinfectant_pump;
  uint8_t   side_brush_level;
  uint8_t   self_adapt_target_current;   //自适应电流
  uint8_t   brush_lift_taget_position; 
  uint8_t   clean_error;
  uint8_t   roller_dir;
  uint8_t   reset_imu;
  uint8_t   mist_relay_1;   
  uint8_t   mist_relay_2;    
  uint8_t   incense_relay;   
  int16_t   zlac_current_limit;   // 中菱电机速度环最大电流限制
  uint8_t   hub_motor_damping;
  uint8_t   hub_motor_power_on;
};

struct CONFIGCOMMAND_CMD{
  uint32_t  Device_Id;
  uint32_t  Speed_Level;
  uint32_t  Bumper_Front;         // 0-15位有效  哪些位为1，则说明是在这个方向上。
  uint32_t  Bumper_Left;          //同上
  uint32_t  Bumper_Right;         //同上
  uint32_t  Bumper_Rear;          //同上
  uint32_t  Bumper_Left_Front;        //同上
  uint32_t  Bumper_Right_Front;       //同上
  uint32_t  Bumper_Left_Rear;       //同上
  uint32_t  Bumper_Right_Rear;        //同上  
  uint32_t  Ultrasonic_Front;       //0-11位有效   哪些位为1，则说明是在这个方向上。
  uint32_t  Ultrasonic_Left;        //同上
  uint32_t  Ultrasonic_Right;       //同上
  uint32_t  Ultrasonic_Rear;        //同上
  uint32_t  Ultrasonic_Left_Front;      //同上
  uint32_t  Ultrasonic_Right_Front;     //同上
  uint32_t  Ultrasonic_Left_Rear;     //同上
  uint32_t  Ultrasonic_Right_Rear;      //同上
  uint32_t  Ultrasonic_Enable;        //超声数据收发使能，1为使能
  uint32_t  Charge_Full_Voltage;
  uint32_t  Charge_Full_Current;
  uint32_t  Charge_Short_Current;
  uint32_t  Charge_Touched_Voltage_Low;
  uint32_t  Charge_Touched_Voltage_High;
  uint32_t  Additional_Braker;
  uint32_t  Filter_Motor;
  uint32_t  Anti_Drop_Front;
  uint32_t  Anti_Drop_Left;
  uint32_t  Anti_Drop_Right;
  uint32_t  Anti_Drop_Rear;
  uint32_t  Anti_Drop_Left_Front;
  uint32_t  Anti_Drop_Right_Front;
  uint32_t  Anti_Drop_Left_Rear;
  uint32_t  Anti_Drop_Right_Rear;

  uint32_t  Bumper_Enable;// add by shaolei
  uint32_t  Filter_Motor_Level;
  uint32_t  Brush_Up_PWM;
  uint32_t  Brush_Up_Current_Threshold;
  uint32_t  Brush_Down_PWM;
  uint32_t  Brush_Down_Current_Threshold;
  uint32_t  Spread_PWM;
  uint32_t  Spread_Current_Threshold;
  uint32_t  Suppress_PWM;
  uint32_t  Suppress_Current_Threshold;
  uint32_t  Spread_Reopen_Delay;
  uint32_t  Brush_Down_Half_Time;
  uint32_t  Brush_Up_Half_Time;
  uint32_t  Brush_Down_Full_Time;
  uint32_t  Enable_Debug;
  uint32_t  Debug_Channel;

  uint32_t  Battery_Empty_Voltage_adc;
  uint32_t  Charge_Empty_Voltage_adc;
  uint32_t  Charge_Empty_Current_adc;
  uint32_t  Max_Charge_Time;// add end

  uint32_t  Anti_Drop_Threshold[8];

  uint32_t  Anti_Collision_Front;
  uint32_t  Anti_Collision_Left;
  uint32_t  Anti_Collision_Right;
  uint32_t  Anti_Collision_Rear;
  uint32_t  Anti_Collision_Left_Front;
  uint32_t  Anti_Collision_Right_Front;
  uint32_t  Anti_Collision_Left_Rear;
  uint32_t  Anti_Collision_Right_Rear;
  uint32_t  Anti_Collision_Threshold[8];
  uint32_t  anti_collision_period_time[8];
  uint32_t  rfidPower;
  
  uint32_t left_motor_max_temperature;
  uint32_t right_motor_max_temperature;
  uint32_t xd510_max_temperatrue;
  uint32_t brush_motor_max_current;
  uint32_t spray_motor_max_current;
  uint32_t fliter_motor_max_current;
  uint32_t left_motor_max_current;
  uint32_t right_motor_max_current;
  uint32_t brush_lift_motor_max_current;
  uint32_t squeegee_lift_motor_max_current;
  uint32_t squeegee_lift_max_current_time;
  uint32_t vacuum_motor_max_current;
  uint32_t run_motor_max_current_in_charge;
  uint32_t bms_max_disconnect_time;
  uint32_t bms_recover_time;
  uint32_t motor_driver_type;
  uint16_t hull_lift_travel_low;
  uint16_t hull_lift_travel_median;
  uint16_t hull_lift_travel_high;
  uint16_t side_lift_travel_low;
  uint16_t side_lift_travel_median;
  uint16_t side_lift_travel_high;
  uint8_t  wheel_type;
  uint32_t velocity_1P;
  uint32_t velocity_1I;
  uint16_t touch_sensor_enable;
  uint16_t anticollision_valid_voltage;
  uint8_t  control_mode;
  uint8_t  emergency_type;
  uint32_t emergency_status_min_time;
  uint8_t disinfect_enable;
  uint8_t cancel_485;
  uint8_t rx_cancel_485_flag;
  uint8_t dust_bag_error_vol;
  uint8_t dust_bag_error_distance;
  uint16_t  dust_full_threshold;
  uint16_t Brush_Down;
  uint16_t side_motor_max_current;
  uint16_t side_motor_max_current_time;
  uint8_t side_motor_type;
  uint8_t hardware_type;                       //硬件类型 0：50 E4.1混合驱控整车  1： 50旧非自研驱控整车
  uint8_t board_type;
  uint8_t self_adaption_enable;               //0 不使用推杆自适应 1使用
  uint8_t self_adaption_deviation;
  uint16_t self_adaption_filter_time;          //刷盘推杆自适应控制电压去抖时间
  uint16_t bms_type;                           //电池类型 0：旧485电池；1：比亚迪电池
  uint16_t squeegee_lift_operate_time;         //尘推动作时间
  uint16_t side_lift_operate_time;             //边刷动作时间
  uint8_t odom_bit_len;
  uint8_t side_door_sensor_type;
};  

struct DEVICE_DATA{
  uint32_t  Di;
  uint32_t  Do;
  uint32_t  Relay;
  uint32_t  Emergency;
  uint32_t  Braker_Down;
  uint32_t  Battery_Voltage_ADC;
  uint32_t  Charger_Voltage_ADC;
  uint32_t  Charge_Current_ADC;
  uint32_t  Remote_Controller;
  uint32_t  Auto_Mode;
  uint32_t  Left_Bumper;
  uint32_t  Right_Bumper;
  uint32_t  Sewage_Level;
  uint32_t  Sewage_Foam;
  uint32_t  Clean_Water_Level;
  uint32_t  Clean_Water_Foam;
  uint32_t  Left_Spread;
  uint32_t  Right_Spread;
  uint32_t  Brush_Down;
  uint32_t  Brush_Motor_Working;
  uint32_t  Filter_Motor_Working;
  uint32_t  Squeegee_Down;
  uint32_t  Squeegee_Motor_Working;
  uint32_t  Valve;
  uint32_t  Spray_Motor_Working;
  uint32_t  Track_Motor_Working;
//  uint32_t  Track_Motor;
  uint32_t  Steering_Motor_Working;
//  uint32_t  Steering_Motor;
  uint32_t  Power_Switch;
  uint32_t  BatterySoc;
  uint32_t  mcu_error_code[28];
  uint32_t  spray_motor_current;
  uint32_t  filter_motor_current;
  uint32_t  vacuum_motor_current;
  uint32_t  water_level_sensor_status;
  uint32_t  left_motor_load_factor;
  uint32_t  right_motor_load_factor;
  uint32_t  mode_button_status;
  uint32_t  side_brush_down;
  uint32_t  left_brush_motor_working;
  uint32_t  right_brush_motor_working;
  uint8_t   imu_reset_status;
  uint16_t  dust_bag_voltage1;
  uint16_t  dust_bag_voltage2;
  uint16_t  dust_bag_status;	
  uint16_t  dust_bag_error_code;
  uint32_t  h25a_error_code;
  uint32_t  h25a_error_code_sub;
  uint32_t imu_data_update_tick;   //imu最近一次正常解析的时刻,用于排查imu数据异常问题
  uint32_t imu_uart_recieve_cnt;   //imu所用串口接收到数据的计数
  uint32_t imu_task_run_cnt;       //任务循环运行次数
  uint8_t  hull_disconnect_flag;
  uint32_t zlac_error_code1;  //自研轮毂电机左右轮告警码
  uint32_t zlac_error_code2;
  uint32_t zlac_version;
  uint16_t xds_motor_temperature;
};  

struct OTHER_DEVICE_DATA{
  uint32_t  run_motor_mode;
  uint32_t  xd510_temperature;
  uint32_t  left_motor_temperature;
  uint32_t  right_motor_temperature;
  uint32_t  brush_lift_motor_current;
  uint32_t  brush_motor_current;
  uint32_t  squeegee_lift_motor_current;
  uint32_t  left_motor_current;
  uint32_t  right_motor_current;
  uint32_t  starup_type;
  uint32_t  imu_delta_yaw;
  uint32_t  dust_push_spin;
  uint32_t  dust_push_clean_spin;
  uint32_t  dust_push_current;
  uint32_t  dust_push_clean_current;
  uint8_t   anti_drop_switch;
  uint8_t  motor_mode;
  uint16_t  torque_voltage1;
  uint16_t  torque_voltage2;
  uint8_t  emergency_button;
  uint8_t  disinfect_mist_spray_level;
  uint8_t  disinfect_spray_distance;
  uint8_t  disinfect_atomization_box_level;
  uint8_t  disinfect_beep_status;
  uint8_t  disinfect_pump_status;
  uint16_t disinfect_temperature;
  uint16_t side_lift_position;
  uint16_t Brush_Down;
  uint16_t side_lift_motor_current;
  int16_t   left_side_motor_current;
  int16_t   right_side_motor_current;
  int16_t   roller_motor_position;
  uint16_t  roller_speed;
  uint8_t   disfection_level;  
  uint8_t   atomizer_level;   
  uint8_t   incense_level;    
  uint8_t disinfect_detect;  
  uint8_t incense_detect;   
  uint8_t side_door_status;   
  int16_t   zlac_current_limit;   // 中菱电机速度环最大电流限制
  uint32_t xds_driver_info ;      //星德胜风机相关信息
  uint32_t driver_heartbeat_time; //驱动器心跳时间
};

struct HEALTH_DATA{
  uint8_t  Avr;
  uint8_t  Ultrasonic_Board;
  uint8_t  Imu_Board;
  uint8_t  Motor_Driver;
  uint8_t  Touch_Screen;
  uint8_t  Left_Encoder;
  uint8_t  Right_Encoder;
  uint8_t  Uwb;
  uint8_t  Gps;
  uint8_t  Rfid;
  uint8_t  Left_Motor;
  uint8_t  Right_Motor;
//  uint8_t  Track_Motor;
//  uint8_t  Steering_Motor;
  uint8_t  Squeegee_Motor;
  uint8_t  Left_Spread_Motor;
  uint8_t  Right_Spread_Motor;
  uint8_t  Brush_Lift_Motor;
  uint8_t  Brush_Motor;
  uint8_t  Spray_Motor;
  uint8_t  battery_connect;
  uint8_t  left_brush_motor;
  uint8_t  right_brush_motor;
};
struct RFID_DATA
{
  uint32_t Data;
  uint32_t Strength;
};
struct ULTRASONICDATA
{
  uint32_t ks103_data0;
  uint32_t ks103_data1;
  uint32_t ks103_data2;
  uint32_t ks103_data3;
  uint32_t ks103_data4;
  uint32_t ks103_data5;
  uint32_t ks103_data6;
  uint32_t ks103_data7;
  uint32_t ks103_data8;
  uint32_t ks103_data9;
  uint32_t ks103_data10;
  uint32_t ks103_data11;
//  uint32_t ks103_data12;
};

typedef struct {
  uint32_t  type;
  uint32_t  major;
  uint32_t  minor;
  uint32_t  build;
}VERSION_DATA_Typedef;

typedef struct {
  uint8_t anti_drop_0;
  uint8_t anti_drop_1;
}ANTIDROP_DATA_Typedef;

struct MOTIONCONTROLMOTORCOMMAND_CMD{

	int32_t   MotionControlMotorCommand_Low;
	int32_t   MotionControlMotorCommand_High;
	int32_t   MotionControlMotorCommand_AKMLineSpeed;
	int32_t   MotionControlMotorCommand_AKMAngleSpeed;
	int32_t   MotionControlMotorCommand_AKMAngle;
};

struct SHUTDOWN_CMD{
  uint32_t id;
  uint32_t type;
};

struct PROTECTOR_DATA{
	uint32_t left_bump;
	uint32_t left_front_bump;
	uint32_t right_front_bump;
	uint32_t right_bump;
	uint32_t left_presser_foot;
	uint32_t right_presser_foot;
	uint32_t rfid_alarm;
};






void InitTcpip(void);
void InitTcpipresetA(void);
void InitTcpipresetB(void);
//void Process_Socket_Data(SOCKET s);
void SetTcpCurrentState(enum State f_state);
uint8_t GetMotorCmdState(void);
void SetMotorCmdState(uint8_t f_newState);
uint8_t GetHasGotCfgParaFlag(void);
void SetHasGotCfgParaFlag(uint8_t f_newState);

uint8_t RunNanopb(uint8_t buf[],uint16_t *msg_len,const uint16_t buf_len);
uint8_t nanopb_fill_ack_msg(uint32_t msg_id, uint8_t* buf, uint16_t* msg_len, const uint16_t buf_len);
uint8_t nanopb_fill_canbus_status_ack_msg(uint32_t msg_id, uint8_t* buf, uint16_t* msg_len, const uint16_t buf_len);

#endif
