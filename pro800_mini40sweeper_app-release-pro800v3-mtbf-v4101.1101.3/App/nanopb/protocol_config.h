/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file protocol_config.h
 * @brief 定义上下位机通行的数据结构，nanopb流程为
 *        发送：  约定结构Message--结构赋值--压缩结构--发送压缩数据
 *        接收：  收到压缩数据--接压缩数据--用约定的结构Message接受数据
 *
 *        本文件与nanopb.pb.h nanopb.pb.c为上下位机通用，有一方要更改此三个文件，更改后必须将此三个文件重新提交给另一方
 *        且要在本文件中写上更改说明；
 *
 *        说明：电机控制对应bit置1为运行，置0为停止命令；
 *              手自动切换0：不做切换 1：切换手动  2：切换自动
 *              刹车命令  0：抬起刹车  1：放下刹车
 * @author Yipeng<yp@gs-robot.com>
 * @version 1.0.0.0
 * @date 2016-10-24
 */

#ifndef _PROTOCOL_CONFIG_H
#define _PROTOCOL_CONFIG_H

enum MessageType {
  HEADER_MESSAGE = 1,
  VERIFY_MESSAGE,

  SHUTDOWN_COMMAND,
  CONFIG_COMMAND,
  MOTION_CONTROL_MOTOR_COMMAND,
  DEVICE_COMMAND,
  MANUAL_MODE_DEVICE_COMMAND,
  REMOTE_COMMAND_433,

  VERSION_DATA,
  HEALTH_DATA,
  DEVICE_DATA,
  ODOMETRY_DATA,
  IMU_DATA,
  ULTRASONIC_DATA,
  PROTECTOR_DATA,
  ANTIDROP_DATA,
  UWB_DATA,
  RFID_DATA,
  REMOTE_DATA_433,

  CLEAR_ERROR_COMMAND,
  MOTOR_DRIVER_CONFIG_COMMAND,
  MOTOR_DRIVER_CONFIG_DATA,
  SAVE_MOTOR_DRIVER_CONFIG_COMMAND,
  ANALOG_ANTI_COLLISION_DATA,

  DRIVER_CONFIG_COMMOD,
  HARDWARE_DRIVER_DATA,
  SABER_IMU_DATA,
  MCU_UPGRADE_COMMOD,
  MCU_UPGRADE_STATUS,
  UWB_DISTANCE_DATA,
  ODOM_RECORDER_DATA,
  MCU_DEBUG_DATA                        = 32, 
  HI216_IMU_DATA                        = 33, 
  CAMERA_PTZ_DATA                       = 34, 
  BATTERY_DATA                          = 35, 
  SHAO_TEST_DATA 						= 36, 
  MOTION_POSITION_CAMMAND 				= 37, 
  OTHER_DEVICE_DATA 					= 38,
  CANBUS_COMMAND = 42,
  CANBUS_STATUS = 43,
  DRIVER_DATA                           = 44,
  PHANTAS_MCU_UPGRADE_COMMAND = 45,
  PHANTAS_MCU_UPGRADE_PACKAGE = 46,
  PHANTAS_MCU_UPGRADE_COMMAND_STATUS = 47,
  PHANTAS_MCU_UPGRADE_PACKAGE_STATUS = 48,
  PHANTAS_MCU_TYPE_DATA = 49,
  CANBUS2_COMMAND = 50,
  CANBUS2_STATUS = 51,  
};

enum DeviceCommandID {
  DEVICE_COMMAND_DO = 0,
  DEVICE_COMMAND_RELAY = 1,
  DEVICE_COMMAND_BRAKER_DOWN = 2,
  DEVICE_COMMAND_AUTO_MODE = 3,
  DEVICE_COMMAND_TRACK_MOTOR = 4,
  DEVICE_COMMAND_STEERING_MOTOR = 5,
  DEVICE_COMMAND_BRUSH_DOWN = 6,
  DEVICE_COMMAND_BRUSH_MOTOR = 7,
  DEVICE_COMMAND_LEFT_BRUSH_MOTOR = 8,
  DEVICE_COMMAND_RIGHT_BRUSH_MOTOR = 9,
  DEVICE_COMMAND_SQUEEGEE_DOWN = 10,
  DEVICE_COMMAND_SQUEEGEE_MOTOR = 11,
  DEVICE_COMMAND_SPRAY_MOTOR = 12,
  DEVICE_COMMAND_VALVE = 13,
  DEVICE_COMMAND_WATER_LEVEL = 14,
  DEVICE_COMMAND_FAN_LEVEL = 15,
  DEVICE_COMMAND_BRUSH_SPIN_LEVEL = 16,
  DEVICE_COMMAND_BRUSH_PRESSURE_LEVEL = 17,
  DEVICE_COMMAND_SPEED_LEVEL = 18,
  DEVICE_COMMAND_DETERGENT_LEVEL = 19,
  DEVICE_COMMAND_DETERGENT_MOTOR = 20,
  DEVICE_COMMAND_LED_LIGHT = 21,
  DEVICE_COMMAND_LEFT_VACUUM = 22,
  DEVICE_COMMAND_RIGHT_VACUUM = 23,
  DEVICE_COMMAND_PUSH_POD_MOTOR = 24,		
  DEVICE_COMMAND_LED_CMD = 25,
  DEVICE_COMMAND_LED_CMD1 = 26,
  DEVICE_COMMAND_FILTRE_PWM = 27,
  DEVICE_COMMAND_MCU_DEBUG_CONFIG = 28,
  DEVICE_COMMAND_INLET_VALVE			= 29,
  DEVICE_COMMAND_OUTLET_VALVE 		= 30,
  DEVICE_COMMAND_WORK_STATUS			= 31, //0：正常	1：执行扫地任务	2：执行充电任务	 3：执行加水任务 
  DEVICE_COMMAND_HEART_STATUS 		= 32, //心跳，充电桩心跳回应
  DEVICE_COMMAND_ALONE_BRAKE			= 33, //0 : 正常	1 : 左轮刹车  2 : 右轮刹车 3都刹车
  DEVICE_COMMAND_ORDER_CHARGER		= 34, //1 预约充电桩
  DEVICE_COMMAND_MOTOR_MODE			  = 35, //1 位置模式 0速度模式
  DEVICE_COMMAND_BRUSH_MOTOR_CURRENT	= 36, //刷盘电机的工作电流 单位：A 电流模式下使用
  DEVICE_COMMAND_EMERGENCY            =38,
  DEVICE_COMMAND_CHECK_CHARGER = 39, //充电桩查询命令
  DEVICE_COMMAND_TIME = 40,//上位机实时时间
  DEVICE_COMMAND_SIDE_BRUSH_DOWN       = 42,
  DEVICE_COMMAND_LEFT_BRUSH_SPIN_LEVEL = 43, // 左刷盘转速
  DEVICE_COMMAND_RIGHT_BRUSH_SPIN_LEVEL = 44, //右刷盘转速
  DEVICE_COMMAND_MANUAL_DRAIN_MODE = 45, // 手动排水开关
  DEVICE_COMMAND_AUTO_FILLER_MODE = 46, // 自动加水开关
  DEVICE_COMMAND_SIDE_BRUSH_MOTOR = 47, // 左右边刷电机开关
  DEVICE_COMMAND_CLEAR_PROTECTOR_TRIGGET = 48,
  DEVICE_COMMAND_RESTART_TIME = 49,
  DEVICE_COMMAND_KEEP_VACCUM = 50,
  DEVICE_COMMAND_DUST_PUSH_SPIN = 68,
  DEVICE_COMMAND_DUST_PUSH_CLEAN_SPIN = 69,
  DEVICE_COMMAND_ANTI_DROP_SWITCH = 74,
  DEVICE_COMMAND_CLEAR_EMERGENCY = 80,
  DEVICE_COMMAND_MIST_SPRAY_LEVEL = 87,
  DEVICE_COMMAND_SPRAY_DISTANCE = 88,
  DEVICE_COMMAND_DISINFECT_BOX_BEEP = 90,
  DEVICE_COMMAND_DISINFECT_DISINFECTANT_PUMP = 91,
  DEVICE_COMMAND_HUB_MOTOR_DAMPING = 136,
  DEVICE_COMMAND_HUB_MOTOR_POWER_ON = 137,
};

enum ConfigCommandID {
  CONFIG_COMMAND_DEVICE_ID = 0,
  CONFIG_COMMAND_SPEED_LEVEL,
  CONFIG_COMMAND_BUMPER_FRONT,
  CONFIG_COMMAND_BUMPER_LEFT,
  CONFIG_COMMAND_BUMPER_RIGHT,
  CONFIG_COMMAND_BUMPER_REAR,
  CONFIG_COMMAND_BUMPER_LEFT_FRONT,
  CONFIG_COMMAND_BUMPER_RIGHT_FRONT,
  CONFIG_COMMAND_BUMPER_LEFT_REAR,
  CONFIG_COMMAND_BUMPER_RIGHT_REAR,
  CONFIG_COMMAND_ULTRASONIC_FRONT =10,
  CONFIG_COMMAND_ULTRASONIC_LEFT,
  CONFIG_COMMAND_ULTRASONIC_RIGHT,
  CONFIG_COMMAND_ULTRASONIC_REAR,
  CONFIG_COMMAND_ULTRASONIC_LEFT_FRONT,
  CONFIG_COMMAND_ULTRASONIC_RIGHT_FRONT,
  CONFIG_COMMAND_ULTRASONIC_LEFT_REAR,
  CONFIG_COMMAND_ULTRASONIC_RIGHT_REAR,
  CONFIG_COMMAND_ULTRASONIC_ENABLE,
  CONFIG_COMMAND_CHARGE_FULL_VOLTAGE_ADC,
  CONFIG_COMMAND_CHARGE_FULL_CURRENT_ADC =20,
  CONFIG_COMMAND_CHARGE_SHORT_CIRCUIT_CURRENT_ADC,
  CONFIG_COMMAND_CHARGE_TOUCHED_VOLTAGE_ADC_LOWER_BOUNDS,
  CONFIG_COMMAND_CHARGE_TOUCHED_VOLTAGE_ADC_UPPER_BOUNDS,
  CONFIG_COMMAND_ADDITIONAL_BRAKER,
  CONFIG_COMMAND_FILTER_MOTOR,
  CONFIG_COMMAND_ANTI_DROP_FRONT,
  CONFIG_COMMAND_ANTI_DROP_LEFT,
  CONFIG_COMMAND_ANTI_DROP_RIGHT,
  CONFIG_COMMAND_ANTI_DROP_REAR,
  CONFIG_COMMAND_ANTI_DROP_LEFT_FRONT = 30 ,
  CONFIG_COMMAND_ANTI_DROP_RIGHT_FRONT,
  CONFIG_COMMAND_ANTI_DROP_LEFT_REAR,
  CONFIG_COMMAND_ANTI_DROP_RIGHT_REAR,

  CONFIG_COMMAND_BUMPER_ENABLE,
  CONFIG_COMMAND_FILTER_MOTOR_LEVEL,
  CONFIG_COMMAND_BRUSH_UP_PWM,
  CONFIG_COMMAND_BRUSH_UP_CURRENT_THRESHOLD,
  CONFIG_COMMAND_BRUSH_DOWN_PWM,
  CONFIG_COMMAND_BRUSH_DOWN_CURRENT_THRESHOLD,
  CONFIG_COMMAND_SPREAD_PWM = 40,
  CONFIG_COMMAND_SPREAD_CURRENT_THRESHOLD,
  CONFIG_COMMAND_SUPPRESS_PWM,
  CONFIG_COMMAND_SUPPRESS_CURRENT_THRESHOLD,
  CONFIG_COMMAND_SPREAD_REOPEN_DELAY,
  CONFIG_COMMAND_BRUSH_DOWN_HALF_TIME,
  CONFIG_COMMAND_BRUSH_UP_HALF_TIME,
  CONFIG_COMMAND_BRUSH_DOWN_FULL_TIME,
  CONFIG_COMMAND_ENABLE_DEBUG,
  CONFIG_COMMAND_DEBUG_CHANNEL,

  CONFIG_COMMAND_BATTERY_EMPTY_VOLTAGE_ADC =50 ,
  CONFIG_COMMAND_CHARGE_EMPTY_VOLTAGE_ADC,
  CONFIG_COMMAND_CHARGE_EMPTY_CURRENT_ADC,
  CONFIG_COMMAND_MAX_CHARGE_TIME,

  CONFIG_COMMAND_FORWARD_DEC,
  CONFIG_COMMAND_BACKWARD_DEC,

  CONFIG_COMMAND_ANALOG_ANTI_COLLISION_FRONT,
  CONFIG_COMMAND_ANALOG_ANTI_COLLISION_LEFT,
  CONFIG_COMMAND_ANALOG_ANTI_COLLISION_RIGHT,
  CONFIG_COMMAND_ANALOG_ANTI_COLLISION_REAR,
  CONFIG_COMMAND_ANALOG_ANTI_COLLISION_LEFT_FRONT =  60,
  CONFIG_COMMAND_ANALOG_ANTI_COLLISION_RIGHT_FRONT,
  CONFIG_COMMAND_ANALOG_ANTI_COLLISION_LEFT_REAR,
  CONFIG_COMMAND_ANALOG_ANTI_COLLISION_RIGHT_REAR,

  CONFIG_COMMAND_RFID_POWER,
  CONFIG_COMMAND_ENABLE_SEAT_SWITCH,
  CONFIG_COMMAND_KEEP_BEEPING,
    
  CONFIG_COMMAND_LEFT_MOTOR_MAX_TEMPERATURE	= 76,//左轮温度上限 单位：C
  CONFIG_COMMAND_RIGHT_MOTOR_MAX_TEMPERATURE	= 77,//右轮温度上限 单位：C
  CONFIG_COMMAND_XD510_MAX_TEMPERATURE = 78,
  CONFIG_COMMAND_BRUSH_MOTOR_CURRENT_MAX = 79,
  CONFIG_COMMAND_SPRAY_MOTOR_CURRENT_MAX = 80,
  CONFIG_COMMAND_FILTER_MOTOR_CURRENT_MAX = 81,
  CONFIG_COMMAND_LEFT_MOTOR_CURRENT_MAX = 82,
  CONFIG_COMMAND_RIGHT_MOTOR_CURRENT_MAX = 83,
  CONFIG_COMMAND_BRUSH_LIFT_MOTOR_CURRENT_MAX = 84,
  CONFIG_COMMAND_SQUEEGEE_LIFT_MOTOR_CURRENT_MAX = 85,
  CONFIG_COMMAND_VACUUM_MOTOR_CURRENT_MAX = 86,
  CONFIG_COMMAND_MOTOR_DRIVER_MAX_CURRENT_CHARGING	= 87,//充电时电机驱动器的最大输出电流 单位：mA 
  CONFIG_COMMAND_BMS_MAX_DISCONNECT_TIME = 91,
  CONFIG_COMMAND_BMS_RECOVER_TIME = 92,
  CONFIG_COMMAND_MOTOR_DRIVER_TYPE = 93,
  CONFIG_COMMAND_HULL_LIFT_TRAVEL_LOW = 94,
  CONFIG_COMMAND_HULL_LIFT_TRAVEL_MDDIAN = 95,
  CONFIG_COMMAND_HULL_LIFT_TRAVEL_HIGH = 96,
  CONFIG_COMMAND_SIDE_LIFT_TRAVEL_LOW = 97,
  CONFIG_COMMAND_SIDE_LIFT_TRAVEL_MDDIAN = 98,
  CONFIG_COMMAND_SIDE_LIFT_TRAVEL_HIGH = 99,
  CONFIG_COMMAND_WHEEL_TYPE = 100,
  CONFIG_COMMAND_VELOCITY_1P = 101,
  CONFIG_COMMAND_VELOCITY_1I = 102,
  CONFIG_COMMAND_TOUCH_SENSOR_ENABLE = 103,
  CONFIG_COMMAND_ANTICOLLISION_VALID_VOLTAGE = 104,
  CONFIG_COMMAND_EMERGENCY_TYPE = 105,
  CONFIG_COMMAND_EMERGENCY_STATUS_MIN_TIME = 106,
  CONFIG_COMMAND_CONTROL_MODE = 109,
  CONFIG_COMMAND_DISINFECT_ENABLE = 114,

  CONFIG_COMMAND_ANTI_DROP_THRESHOLD = 0x1 << 10,
  CONFIG_COMMAND_ULTRASONIC_ACCESS_PRECEDENCE = 0x2 << 10,
  CONFIG_COMMAND_ANALOG_ANTI_COLLISION_THRESHOLD = 0x3 << 10,
  CONFIG_COMMAND_PROTECTOR_ANALOG_DELTA_T = 0x4 << 10,
};

enum HardwareDriverConfidCommandId {
  HARDWARE_VERSION = 0,
  IMU_DRIVER,
  ULTRASONIC_DRIVER,
  RFID_DRIVER,
  ANTIDROP_DRIVER,
  ANTICOLLISION_DRIVER,
  MOTOR_DRIVER,
};

enum ImuDriverDataId {
  IMU_DRIVER_CLOSE = 0,
  IMU_DRIVER_9250_IMU,
  IMU_DRIVER_MPU6050_IMU,
  IMU_DRIVER_SABER_UART4,
};

enum UltrasonicDriverDataId {
  ULTRASONIC_DRIVER_CLOSE = 0,
  ULTRASONIC_DRIVER_KS103_CAN,
  ULTRASONIC_DRIVER_KS136_USART6,
  ULTRASONIC_DRIVER_TANK_ULTRASONIC,
  ULTRASONIC_DRIVER_KS136_UART5_TANK,
  ULTRASONIC_DRIVER_KS136II_USART6,
  ULTRASONIC_DRIVER_GP2D120X_ULTRASONIC,
  ULTRASONIC_DRIVER_MUYE01_ULTRASONIC,
};

enum RfidDriverDataId {
  RFID_DRIVER_CLOSE = 0,
  RFID_DRIVER_UART4,
  RFID_DRIVER_UWB_UART5,
  RFID_DRIVER_JUNWANG_UART4,
};

enum AntidropDriverDataId {
  ANTIDROP_DRIVER_CLOSE = 0,
  ANTIDROP_DRIVER_CAN,
  ANTIDROP_DRIVER_ADC,
  ANTIDROP_DRIVER_GP2D120X_ADC,
};

enum AntiCollisionDriverDataId {
  ANTI_COLLISION_DRIVER_CLOSE = 0,
  ANTI_COLLISION_DRIVER_IO,
  ANTI_COLLISION_DRIVER_ADC,
};

enum MotorDriverDataId {
  MOTOR_DRIVER_CLOSE = 0,
  MOTOR_DRIVER_NORMAL,
  MOTOR_DRIVER_MANIS_PWM,
  MOTOR_DRIVER_ANZE_UART4,
  MOTOR_DRIVER_ZLAC_UART4_UART6,
  MOTOR_DRIVER_TRUMMAN_UART5,
  MOTOR_DRIVER_TANK_UART4,
  MOTOR_DRIVER_MUYE01_UART4_UART6,
  MOTOR_DRIVER_BALANCECAR_UART4,
};

enum MotorDriverConfigCommandId {
  MOTOR_DRIVER_CONFIG_COMMAND_LOW_SPEED = 0,
  MOTOR_DRIVER_CONFIG_COMMAND_HIGH_SPEED,
  MOTOR_DRIVER_CONFIG_COMMAND_LIFT_MOTOR_OVERLOAD_CURRENT_THRESHOLD,
  MOTOR_DRIVER_CONFIG_COMMAND_LIFT_MOTOR_OVERLOAD_TIME_THRESHOLD,
  MOTOR_DRIVER_CONFIG_COMMAND_LEFT_SPREAD_MOTOR_OVERLOAD_CURRENT_THRESHOLD,
  MOTOR_DRIVER_CONFIG_COMMAND_LEFT_SPREAD_MOTOR_OVERLOAD_TIME_THRESHOLD,
  MOTOR_DRIVER_CONFIG_COMMAND_RIGHT_SPREAD_MOTOR_OVERLOAD_CURRENT_THRESHOLD,
  MOTOR_DRIVER_CONFIG_COMMAND_RIGHT_SPREAD_MOTOR_OVERLOAD_TIME_THRESHOLD,
  MOTOR_DRIVER_CONFIG_COMMAND_SQUEEGEE_MOTOR_OVERLOAD_CURRENT_THRESHOLD,
  MOTOR_DRIVER_CONFIG_COMMAND_SQUEEGEE_MOTOR_OVERLOAD_TIME_THRESHOLD,
  MOTOR_DRIVER_CONFIG_COMMAND_SPRAY_MOTOR_OVERLOAD_CURRENT_THRESHOLD,
  MOTOR_DRIVER_CONFIG_COMMAND_SPRAY_MOTOR_OVERLOAD_TIME_THRESHOLD,
  MOTOR_DRIVER_CONFIG_COMMAND_FILTER_MOTOR_OVERLOAD_CURRENT_THRESHOLD,
  MOTOR_DRIVER_CONFIG_COMMAND_FILTER_MOTOR_OVERLOAD_TIME_THRESHOLD,
  MOTOR_DRIVER_CONFIG_COMMAND_BRUSH_MOTOR_OVERLOAD_CURRENT_THRESHOLD,
  MOTOR_DRIVER_CONFIG_COMMAND_BRUSH_MOTOR_OVERLOAD_TIME_THRESHOLD,
  MOTOR_DRIVER_CONFIG_COMMAND_CURRENT_0_P,
  MOTOR_DRIVER_CONFIG_COMMAND_CURRENT_0_I,
  MOTOR_DRIVER_CONFIG_COMMAND_VELOCITY_0_P,
  MOTOR_DRIVER_CONFIG_COMMAND_VELOCITY_0_I,
  MOTOR_DRIVER_CONFIG_COMMAND_POSITION_0_P,
  MOTOR_DRIVER_CONFIG_COMMAND_POSITION_0_I,
  MOTOR_DRIVER_CONFIG_COMMAND_POSITION_0_D,
  MOTOR_DRIVER_CONFIG_COMMAND_CURRENT_1_P,
  MOTOR_DRIVER_CONFIG_COMMAND_CURRENT_1_I,
  MOTOR_DRIVER_CONFIG_COMMAND_VELOCITY_1_P,
  MOTOR_DRIVER_CONFIG_COMMAND_VELOCITY_1_I,
  MOTOR_DRIVER_CONFIG_COMMAND_POSITION_1_P,
  MOTOR_DRIVER_CONFIG_COMMAND_POSITION_1_I,
  MOTOR_DRIVER_CONFIG_COMMAND_POSITION_1_D,
  MOTOR_DRIVER_CONFIG_COMMAND_CURRENT_2_P,
  MOTOR_DRIVER_CONFIG_COMMAND_CURRENT_2_I,
  MOTOR_DRIVER_CONFIG_COMMAND_VELOCITY_2_P,
  MOTOR_DRIVER_CONFIG_COMMAND_VELOCITY_2_I,
  MOTOR_DRIVER_CONFIG_COMMAND_POSITION_2_P,
  MOTOR_DRIVER_CONFIG_COMMAND_POSITION_2_I,
  MOTOR_DRIVER_CONFIG_COMMAND_POSITION_2_D,
  MOTOR_DRIVER_CONFIG_COMMAND_CURRENT_3_P,
  MOTOR_DRIVER_CONFIG_COMMAND_CURRENT_3_I,
  MOTOR_DRIVER_CONFIG_COMMAND_VELOCITY_3_P,
  MOTOR_DRIVER_CONFIG_COMMAND_VELOCITY_3_I,
  MOTOR_DRIVER_CONFIG_COMMAND_POSITION_3_P,
  MOTOR_DRIVER_CONFIG_COMMAND_POSITION_3_I,
  MOTOR_DRIVER_CONFIG_COMMAND_POSITION_3_D,
  MOTOR_DRIVER_CONFIG_COMMAND_STEERING_VELOCITY_COEFFICIENT,
  MOTOR_DRIVER_CONFIG_COMMAND_INITIALIZE_STEERING,
  MOTOR_DRIVER_CONFIG_COMMAND_TRACK_MOTOR_ACC,
};

enum MotorDriverConfigDataId {
  MOTOR_DRIVER_CONFIG_DATA_LOW_SPEED,
  MOTOR_DRIVER_CONFIG_DATA_HIGH_SPEED,
  MOTOR_DRIVER_CONFIG_DATA_LIFT_MOTOR_OVERLOAD_CURRENT_THRESHOLD,
  MOTOR_DRIVER_CONFIG_DATA_LIFT_MOTOR_OVERLOAD_TIME_THRESHOLD,
  MOTOR_DRIVER_CONFIG_DATA_LEFT_SPREAD_MOTOR_OVERLOAD_CURRENT_THRESHOLD,
  MOTOR_DRIVER_CONFIG_DATA_LEFT_SPREAD_MOTOR_OVERLOAD_TIME_THRESHOLD,
  MOTOR_DRIVER_CONFIG_DATA_RIGHT_SPREAD_MOTOR_OVERLOAD_CURRENT_THRESHOLD,
  MOTOR_DRIVER_CONFIG_DATA_RIGHT_SPREAD_MOTOR_OVERLOAD_TIME_THRESHOLD,
  MOTOR_DRIVER_CONFIG_DATA_SQUEEGEE_MOTOR_OVERLOAD_CURRENT_THRESHOLD,
  MOTOR_DRIVER_CONFIG_DATA_SQUEEGEE_MOTOR_OVERLOAD_TIME_THRESHOLD,
  MOTOR_DRIVER_CONFIG_DATA_SPRAY_MOTOR_OVERLOAD_CURRENT_THRESHOLD,
  MOTOR_DRIVER_CONFIG_DATA_SPRAY_MOTOR_OVERLOAD_TIME_THRESHOLD,
  MOTOR_DRIVER_CONFIG_DATA_FILTER_MOTOR_OVERLOAD_CURRENT_THRESHOLD,
  MOTOR_DRIVER_CONFIG_DATA_FILTER_MOTOR_OVERLOAD_TIME_THRESHOLD,
  MOTOR_DRIVER_CONFIG_DATA_BRUSH_MOTOR_OVERLOAD_CURRENT_THRESHOLD,
  MOTOR_DRIVER_CONFIG_DATA_BRUSH_MOTOR_OVERLOAD_TIME_THRESHOLD,		
	
  MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_0_P,
  MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_0_I,
  MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_0_P,
  MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_0_I,
  MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_0_P,
  MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_0_I,
	MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_0_D,

  MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_1_P,
  MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_1_I,
  MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_1_P,
  MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_1_I,
  MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_1_P,
  MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_1_I,
	MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_1_D,
	
  MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_2_P,
  MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_2_I,
  MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_2_P,
  MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_2_I,
  MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_2_P,
  MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_2_I,
	MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_2_D,

  MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_3_P,
  MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_3_I,
  MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_3_P,
  MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_3_I,
  MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_3_P,
  MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_3_I,
	MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_3_D,
	
  MOTOR_DRIVER_CONFIG_DATA_STEERING_VELOCITY_COEFFICIENT,
  MOTOR_DRIVER_CONFIG_DATA_INITIALIZE_STEERING,
	MOTOR_DRIVER_CONFIG_DATA_TRACT_PWM_ACC,
  MOTOR_DRIVER_CONFIG_DATA_LEFT_MOTOR_LOADRATE_THRESHOLD =54,
  MOTOR_DRIVER_CONFIG_DATA_LEFT_MOTOR_PROTECT_TIME = 55,
  MOTOR_DRIVER_CONFIG_DATA_RIGHT_MOTOR_LOADRATE_THRESHOLD = 56,
  MOTOR_DRIVER_CONFIG_DATA_RIGHT_MOTOR_PROTECT_TIME = 57,
};

enum DeviceDataID {
  // custom
  DEVICE_DATA_WATER_LEVEL = 10000,
  DEVICE_DATA_FAN_LEVEL = 10001,
  DEVICE_DATA_SPRAY_MOTOR_COMMAND = 10002,
  DEVICE_DATA_VALVE_COMMAND = 10003,
  DEVICE_DATA_BRUSH_MOTOR_COMMAND = 10004,
  DEVICE_DATA_BRUSH_SPIN_LEVEL = 10005,
  DEVICE_DATA_BRUSH_PRESSURE_LEVEL = 10006,
  DEVICE_DATA_DETERGENT_LEVEL = 10007,
  DEVICE_DATA_DETERGENT_MOTOR_COMMAND = 10008,
  DEVICE_DATA_LED_LIGHT_COMMAND = 10009,
  DEVICE_DATA_LEFT_FAN_COMMAND = 10010,
  DEVICE_DATA_RIGHT_FAN_COMMAND = 10011,

  // default, those on the document
  DEVICE_DATA_DI = 0, 
  DEVICE_DATA_DO = 1,
  DEVICE_DATA_RELAY = 2,
  DEVICE_DATA_EMERGENCY = 3,
  DEVICE_DATA_BRAKER_DOWN = 4,
  DEVICE_DATA_BATTERY_VOLTAGE_ADC = 5,
  DEVICE_DATA_CHARGER_VOLTAGE_ADC = 6,
  DEVICE_DATA_CHARGER_CURRENT_ADC = 7,
  DEVICE_DATA_REMOTE_CONTROLLER = 8,
  DEVICE_DATA_AUTO_MODE = 9,
  DEVICE_DATA_SEWAGE_LEVEL = 10,
  DEVICE_DATA_SEWAGE_FOAM = 11,
  DEVICE_DATA_CLEAN_WATER_LEVEL = 12,
  DEVICE_DATA_CLEAN_WATER_FOAM = 13,
  DEVICE_DATA_LEFT_SPREAD = 14,
  DEVICE_DATA_RIGHT_SPREAD = 15,
  DEVICE_DATA_BRUSH_DOWN = 16,
  DEVICE_DATA_BRUSH_MOTOR_WORKING = 17,
  DEVICE_DATA_FILTER_MOTOR_WORKING = 18,
  DEVICE_DATA_SQUEEGEE_DOWN = 19,
  DEVICE_DATA_SQUEEGEE_MOTOR_WORKING = 20,
  DEVICE_DATA_VALVE = 21,
  DEVICE_DATA_SPRAY_MOTOR_WORKING = 22,
  DEVICE_DATA_TRACK_MOTOR_WORKING = 23,
  DEVICE_DATA_TRACK_MOTOR = 24,
  DEVICE_DATA_STEERING_MOTOR_WORKING = 25,
  DEVICE_DATA_STEERING_MOTOR = 26,
  DEVICE_DATA_BATTERY_PERCENTAGE = 27,
  DEVICE_DATA_STEERING_WHEEL_VELOCITY,
  DEVICE_DATA_POWER_SWITCH,
  DEVICE_DATA_MCU_ERROR_CODE_0,
  DEVICE_DATA_MCU_ERROR_CODE_1,
  DEVICE_DATA_MCU_ERROR_CODE_2,
	DEVICE_DATA_MCU_ERROR_CODE_3,
	DEVICE_DATA_MANUAL_SPEED_LEVEL,
	DEVICE_DATA_MANUAL_SEAT_SWITCH,
	DEVICE_DATA_TURN_HOME_STATUS,
	DEVICE_DATA_MCU_ERROR_CODE_4,
	DEVICE_DATA_MCU_ERROR_CODE_5,
	DEVICE_DATA_MCU_ERROR_CODE_6,
	DEVICE_DATA_MCU_ERROR_CODE_7,
	DEVICE_DATA_TEMPERATURE,
	DEVICE_DATA_PRESSURE,
	DEVICE_DATA_MCU_STATUS_CODE0,
	DEVICE_DATA_MCU_STATUS_CODE1,
	DEVICE_DATA_MCU_STATUS_CODE2,
	DEVICE_DATA_MCU_STATUS_CODE3,
	DEVICE_DATA_MCU_ERROR_CODE8,
	DEVICE_DATA_MCU_ERROR_CODE9,
	DEVICE_DATA_MCU_ERROR_CODE10,
	DEVICE_DATA_MCU_ERROR_CODE11,
	DEVICE_DATA_MCU_ERROR_CODE12,
	DEVICE_DATA_MCU_ERROR_CODE13,
	
	DEVICE_DATA_DETERGENT_MOTOR = 53,
	DEVICE_DATA_CAR_FALL_DOWN,
	DEVICE_DATA_CAR_READY,
	DEVICE_DATA_LASER_HOR,
	DEVICE_DATA_BASE_HOR,
	DEVICE_DATA_LED_LIGHT,
	DEVICE_DATA_LEFT_VACUUM,
	DEVICE_DATA_RIGHT_VACUUM,
	DEVICE_DATA_MOTOR_DRIVER_EMER,
	DEVICE_DATA_PUSH_POD_MOTOR,		
	DEVICE_DATA_LED_CMD = 63,
	DEVICE_DATA_POTENTIONMETER_VALUE = 64,
	DEVICE_DATA_K = 65,
	DEVICE_DATA_B = 66,
	DEVICE_DATA_POTENTIONMETER_POSITION  = 67,
	DEVICE_DATA_SPRAY_MOTOR_CURRENT  = 68,
	DEVICE_DATA_FILTER_MOTOR_CURRENT 	= 69,
  DEVICE_DATA_CHARGE_ADC 				= 70,
  DEVICE_DATA_LIGHT1_WORKING 			= 71,
	DEVICE_DATA_VACUUM_CURRENT 			= 72,
  DEVICE_DATA_WATER_SENSOR_STATUS 		= 73,
  DEVICE_DATA_ENCODER_POSITION 			= 74,
  	DEVICE_DATA_WARNING_LIGHT			= 75,	//警报状态
	DEVICE_DATA_INLET_VALVE_WORKING		= 76,	//进水阀状态
	DEVICE_DATA_OUTLET_VALVE_WORKING	= 77,	//出水阀状态
	DEVICE_DATA_LEFT_MOTOR_LOAD			= 78,	//左电机负载率
	DEVICE_DATA_RIGHT_MOTOR_LOAD		= 79,	//右电机负载率
	DEVICE_DATA_MCU_ERROR_CODE_14		= 80,	//50和利时左驱动器故障
	DEVICE_DATA_MCU_ERROR_CODE_15		= 81,	//50和利时右驱动器故障
	DEVICE_DATA_MODE_BUTTON = 83,//模式切换的按钮状态1=被按下
	DEVICE_DATA_SIDE_BRUSH_DOWN           = 84,
    DEVICE_DATA_LEFT_BRUSH_MOTOR_WORKING  = 85,
	DEVICE_DATA_RIGHT_BRUSH_MOTOR_WORKING = 86,
    DEVICE_DATA_MCU_ERROR_CODE_24		= 91,	//
    DEVICE_DATA_MCU_ERROR_CODE_26       = 93,
    DEVICE_DATA_MCU_ERROR_CODE_27       = 94,
};

enum OtherDeviceDataID {
	OTHER_DEVICE_DATA_COMMAND_RECEIVE	= 1, //是否收到指令  0-未收到   1-收到， 位置模式下指令是否收到
	OTHER_DEVICE_DATA_IS_FINISHED		= 2, //是否执行完成(0-正常状态,1-正在执行,2-执行完成) 位置模式下
	OTHER_DEVICE_DATA_MOTOR_MODE		= 3, //0-速度环, 1-位置环, 2-电流环 ，3-扭矩控制
	OTHER_DEVICE_DATA_HEART_STATUS		= 4, //心跳
	OTHER_DEVICE_DATA_MCU_TYEP          = 5, // 控制盒类型

	OTHER_DEVICE_DATA_DN_MOTOR_DEVICE_TEMPERATURE = 6	,	  //小架驱动器实时温度
	OTHER_DEVICE_DATA_LEFT_MOTOR_TEMPERATURE	    = 7	,   //50左轮电机温度
	OTHER_DEVICE_DATA_RIGHT_MOTOR_TEMPERATURE	    = 8	,   //50右轮电机温度
	OTHER_DEVICE_DATA_MCU_ERROR_CODE_16	          = 9	,   //大能小架驱动器故障
	OTHER_DEVICE_DATA_MCU_ERROR_CODE_17	          = 10, //大能小架驱动器故障
	OTHER_DEVICE_DATA_MCU_ERROR_CODE_18	          = 11, //电机报警信息
	OTHER_DEVICE_DATA_MCU_ERROR_CODE_19	          = 12, //电机报警信息

	OTHER_DEVICE_DATA_BRUSH_LIFT_MOTOR_CURRENT     = 13,
	OTHER_DEVICE_DATA_BRUSH_MOTOR_CURRENT          = 14,
	OTHER_DEVICE_DATA_SQUEEGEE_LIFT_MOTOR_CURRENT  = 15,
	OTHER_DEVICE_DATA_LEFT_MOTOR_CURRENT           = 16,
	OTHER_DEVICE_DATA_RIGHT_MOTOR_CURRENT          = 17,
	OTHER_DEVICE_DATA_CHARGER_STATUS		       = 18,
	OTHER_DEVICE_DATA_MCU_ERROR_CODE_20            = 19,
	OTHER_DEVICE_DATA_STARTUP_TYPE                 = 21,
    OTHER_DEVICE_DATA_IMU_DELTA_YAW                = 47,
	OTHER_DEVICE_DATA_DUST_PUSH_SPIN               = 48,
	OTHER_DEVICE_DATA_DUST_PUSH_CLEAN_SPIN         = 49,
	OTHER_DEVICE_DATA_DUST_PUSH_CURRENT            = 50,
	OTHER_DEVICE_DATA_DUST_PUSH_CLEAN_CURRENT      = 51,
    OTHER_DEVICE_DATA_ANTI_DROP_STATUS             = 66,
    OTHER_DEVICE_DATA_EMERGENCY_BUTTON             = 74,
    OTHER_DEVICE_DATA_MIST_SPRAY_LEVEL             = 82,
	OTHER_DEVICE_DATA_SPRAY_DISTANCE               = 83,
    OTHER_DEVICE_DATA_TORQUE_VOLTAGE1              = 87,
    OTHER_DEVICE_DATA_TORQUE_VOLTAGE2              = 88,
    OTHER_DEVICE_DATA_DISINFECT_BOX_BEEP                = 89,
    OTHER_DEVICE_DATA_DISINFECT_DISINFECTANT_PUMP       = 90,
    OTHER_DEVICE_DATA_DISINFECT_ATOMIZATION_BOX_LEVEL   = 91,
    OTHER_DEVICE_DATA_DISINFECT_BOX_TEMPERATURE         = 92,
    OTHER_DEVICE_DATA_SIDE_DOOR_STATUS             = 127,
};

enum HealthDataID {
  // custom
  HEALTH_DATA_MCU_CONNECTION = 10000,
  HEALTH_DATA_LASER_CONNECTION = 10001,
  HEALTH_DATA_ROUTER_CONNECTION = 10002,
  HEALTH_DATA_INTERNET_CONNECTION = 10003,
  HEALTH_DATA_TABLET_CONNECTION = 10004,
  HEALTH_DATA_ULTRASONIC = 10005,
  HEALTH_DATA_ODOM_LEFT_DELTA = 10006,
  HEALTH_DATA_ODOM_RIGHT_DELTA = 10007,
  HEALTH_DATA_ODOM_DELTA_SPEED = 10008,
  HEALTH_DATA_ODOM_TRACK_DELTA = 10009,
  HEALTH_DATA_MOTOR_DRIVER_EMERGENCY = 10010,

  // default, those on the document
  HEALTH_DATA_POWER_BOARD = 0,
  HEALTH_DATA_ULTRASONIC_BOARD,
  HEALTH_DATA_IMU_BOARD,
  HEALTH_DATA_MOTOR_DRIVER,
  HEALTH_DATA_TOUCH_SCREEN,
  HEALTH_DATA_LEFT_ENCODER,
  HEALTH_DATA_RIGHT_ENCODER,
  HEALTH_DATA_UWB,
  HEALTH_DATA_GPS,
  HEALTH_DATA_RFID,
  HEALTH_DATA_LEFT_MOTOR,
  HEALTH_DATA_RIGHT_MOTOR,
  HEALTH_DATA_TRACK_MOTOR,
  HEALTH_DATA_STEERING_MOTOR,
  HEALTH_DATA_LEFT_SPREAD_MOTOR,
  HEALTH_DATA_RIGHT_SPREAD_MOTOR,
  HEALTH_DATA_SQUEEGEE_LIFT_MOTOR,
  HEALTH_DATA_SQUEEGEE_MOTOR,
  HEALTH_DATA_BRUSH_LIFT_MOTOR,
  HEALTH_DATA_BRUSH_MOTOR=19,
  HEALTH_DATA_VALVE,
  HEALTH_DATA_SPRAY_MOTOR,
  HEALTH_DATA_ANTI_DROP_BOARD,
  HEALTH_DATA_LEFT_BRUSH_MOTOR = 27,
  HEALTH_DATA_RIGHT_BRUSH_MOTOR = 28,
};

enum VersionDataID {
  VERSION_DATA_PRODUCT = 0,
  VERSION_DATA_MAIN_CONTROL_BOARD,
  VERSION_DATA_AVR,
  VERSION_DATA_IMU_BOARD,
  VERSION_DATA_ULTRASONIC_BOARD,
  VERSION_DATA_BACK_MAIN_CONTROL=5,
  VERSION_DATA_BATTERY_VERSION=6,
  VERSION_DATA_DX510_VERSION=7,
  VERSION_DATA_GCUB_VERSION=8,
};

enum BatteryDataID {
  BATTERY_DATA_BATTERY_CELL_VOLTAGE = 0x1 << 10,
  BATTERY_DATA_TEMPERATURE = 0x2 << 10,

  BATTERY_DATA_BATTERY_HALF_VOLTAGE 		 = 0 ,
  BATTERY_DATA_BATTERY_CELL_NUMBER			 = 1 ,
  BATTERY_DATA_CHARGE_CURRENT				 = 2 ,
  BATTERY_DATA_DISCAHRGE_CURRENT			 = 3 ,
  BATTERY_DATA_TEMPERATURE_NUM				 = 4 ,
  BATTERY_DATA_VOLTAGE_STATE				 = 5 ,
  BATTERY_DATA_CURRENT_STATE				 = 6 ,
  BATTERY_DATA_TEMPERATURE_STATE			 = 7 ,
  BATTERY_DATA_BATTERY_ALARM				 = 8 ,
  BATTERY_DATA_FETMOS_STATE 				 = 9 ,
  BATTERY_DATA_OVER_VOLTAGE_CELL_NUM		 = 10,
  BATTERY_DATA_UNDER_VOLTAGE_CELL_NUM		 = 11,
  BATTERY_DATA_WARN_OVER_VOLTAGE_CELL_NUM	 = 12,
  BATTERY_DATA_WARN_UNDER_VOLTAGE_CELL_NUM	 = 13,
  BATTERY_DATA_BALANCE_STATE				 = 14,
  BATTERY_DATA_DISCHARGE_NUM				 = 15,
  BATTERY_DATA_CHARGE_NUM					 = 16,
  BATTERY_DATA_CAP_NOW						 = 17,
  BATTERY_DATA_CAP_FULL 					 = 18,
  BATTERY_DATA_DRIVE_MOTOR_CURRENT			 = 19,
  BATTERY_DATA_BRUSH_MOTOR_CURRENT			 = 20,
  BATTERY_DATA_TURN_MOTOR_CURRENT			 = 21,
  BATTERY_DATA_BRUSHLIFT_MOTOR_CURRENT		 = 22,
  BATTERY_DATA_SEQUEEGEELIFT_MOTOR_CURRENT	 = 23,
  BATTERY_DATA_POTENTIOMETER_VOLTAGE		 = 24,
  BATTERY_DATA_ACCELERATOR_VOLTAGE			 = 25,
  BATTERY_DATA_TURN_LIMIT_STATUS			 = 26,
  BATTERY_DATA_BATTERY_TEMPERATURE			 = 27,
  BATTERY_DATA_BATTERY_VOLTAGE				 = 28,
  BATTERY_DATA_BATTERY_CONNECTION			 = 29,

};

enum ProtectorDataID{
	PROTECTOR_DATA_LEFT_BUMP = 0,
	PROTECTOR_DATA_LEFT_FRONT_BUMP = 1,
	PROTECTOR_DATA_RIGHT_FRONT_BUMP = 2,
	PROTECTOR_DATA_RIGHT_BUMP = 3,
	PROTECTOR_DATA_LEFT_PRESSER_FOOT = 4,
	PROTECTOR_DATA_RIGHT_PRESSER_FOOT = 5,
	PROTECTOR_DATA_RFID_ALARM = 6,
};

#endif  // _PROTOCOL_CONFIG_H
