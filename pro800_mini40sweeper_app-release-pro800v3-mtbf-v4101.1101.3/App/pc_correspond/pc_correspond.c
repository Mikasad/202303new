#include "pc_correspond.h"
#include "nanopb_tcpip.h"
//#include "debug.h"
#include "tcp_server.h"
#include "imu_task.h"
#include "motor_driver.h"
#include "version.h"
#include "sensor_hub.h"
#include "ctr_move_task.h"
#include "normal_io.h"
#include "rfid_etag.h"
#include "sensor.h"
#include "service.pb.h"
#include "iwdg_task.h"
#include "power_ctr.h"
#include "scada.h"
#include "XD510.h"
#include "main.h"
#include "anti_collision.h"
#include "anti_drop_collision.h"
#include "zd_driver.h"
//#include "log.h"
#include "xds_blvacuum.h"
#include "disinfect_bag.h"
#include "h25a15/app_h25a15.h"
#include "h25a15/drv_h25a15.h"
#include "app_bms.h"
#include "hub_motor/app_hub_motor.h"
#include "hub_motor/drv_zlac8015.h"
/*
*********************************************************************************************************
*
*	模块名称 : 与上位机通信模块
*	文件名称 : pc_correspond.c
*	版    本 : 
*	说    明 : 
*			1，与上位机通信
*			2，注意如果修改了代码，请在这里修改版本号
*			3，使用protobuf（这里是c版本-nanopb）对数据进行加解码，使用问答式，上位机下发报头指明当前的数据需求或者接下来即将发送的mes，
然后下位机把接下来收到的mes对应到相对应的处理逻辑中。
*			4，因为涉及的内容较多，而且原有的结构也不错，所以大部分内容没有重写。
*			5，该模块涉及的文件较多，都已经收拢在pc_correspond目录中
*	修改记录 :
*		版本号  		日期        作者     说明
*		V1.0.0    2018-11-05  杨臣   	正式发布
*		V1.1.0	  2018-12-13  杨臣	相对于上个版本应该是添加了一些东西但是忘了做记录。对超声波的配置添加了对总开关的使用。
*		V1.2.0	  2018-12-14  杨臣	添加了rfid功率配置功能，修改了获取rfid功率的变量。在Process_Socket_Data中添加了一个socket，用来与命令行功能通信
*		V1.2.1	  2018-12-18  杨臣	1，继电器配置之后变量清零，以防关机后依然保持关机命令。
*		V1.2.2	  2018-12-20  杨臣	1，添加了shutdown命令，只有关机指令为1时才赋值关机命令，而且赋值之后清零关机命令；2，命令行socket状态对gs_rxBufSizeState无影响
*       V1.3.0    2019-12-02  ypc
*	外设资源占用：
*	待优化：（需要优化但是未优化之前请保留该提示）
*		1,暂无
*********************************************************************************************************
*/

/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/


/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/


/************************************************************************************************************************************
														    变量定义
*************************************************************************************************************************************/
//static SystemStatus_Type m_device_status = {0};
TaskHandle_t taskHandlePcCorrespond = NULL;
//连接状态
static ConnectionStatus_t m_pc_connect_status = {5000,2000}; //设置超时时间，初始化时不检测
static ConnectionStatus_t m_pc_speed_connect_status = {5000,200}; //设置超时时间
static int32_t m_left_speed,m_right_speed;
static uint8_t m_pc_init_flag = 1; //0表示初始化完成

uint32_t  type;
uint32_t  major;
uint32_t  minor;
uint32_t  build;

static SemaphoreHandle_t m_mutex_speed = NULL;

BatteryMsg_t g_battery_data = {0};
RfidTag_t g_rfid_tag_table[RFID_TAG_MAX_NUM] = {0}; //存储单次读取到的RFID标签数据

static uint8_t m_pc_disconnect_flag = 0;
static uint8_t m_speed_timeout_flag = 0;


extern IMUData IMUDataMsg;
extern OdometryData OdometryDataMsg;
extern struct MOTIONCONTROLMOTORCOMMAND_CMD  MotionControlMotorCommandCMD;
extern VERSION_DATA_Typedef  V_Product;
extern VERSION_DATA_Typedef  V_Main;
extern VERSION_DATA_Typedef  V_Gyro;
extern VERSION_DATA_Typedef  V_Sonar;
extern VERSION_DATA_Typedef  V_Drop;
extern VERSION_DATA_Typedef  V_other;
extern VERSION_DATA_Typedef  V_Gcub;
extern VERSION_DATA_Typedef  V_Xd510;
extern VERSION_DATA_Typedef  V_H25A_S;
extern VERSION_DATA_Typedef  V_H25A_H;
extern VERSION_DATA_Typedef  V_ZLAC_S;
extern struct DEVICE_DATA      DeviceDATA ;
//extern uint32_t HandChargeVotage;
extern struct HEALTH_DATA      HealthDATA;
extern struct ULTRASONICDATA UltrasonicDATA;
extern struct DEVICECOMMAND_CMD  DeviceCommandCMD ;
extern struct CONFIGCOMMAND_CMD  ConfigCommandCMD;
//extern struct RFID_DATA    RfidDATA;
extern struct SHUTDOWN_CMD     ShutDown_CMD;
extern struct OTHER_DEVICE_DATA  Other_DeviceData;
extern struct PROTECTOR_DATA g_protector_data;


static void ReadUltrasonicData(void);
static void ReadVersionData(void);
static void ReadOdometryData(void);
static void ReadGyroData(void);
//static void ReadRfidData(void);
static void ReadAllDataFromDevice(void);



static void CfgUltrasonicParameter(void);
static void CfgChargeParameter(void);
//static void CfgSpeedLevelParameter(void);



static void WriteShutdownCtrData(void);
static void WriteRelayCtrData(void);
static void WriteNormalIoOutputCtrData(void);
void WriteBrkCtrData(void);



static void WriteDevCmdData(void);
static void WritePcSpeedData(void);
static void WriteConfigCmdParameter(void);

static void WriteAllDataToDevice(void);



//更新陀螺仪数据
static void ReadGyroData(void)
{
	mesGyro_t mesGyro = {0};

	GetGyroMes(&mesGyro);

	IMUDataMsg.pitch = mesGyro.pitchAngle;
	IMUDataMsg.roll = mesGyro.rollAngle;
	IMUDataMsg.yaw = mesGyro.yawAngle;

	IMUDataMsg.angular_velocity_x=mesGyro.angularVelocity_x;
	IMUDataMsg.angular_velocity_y=mesGyro.angularVelocity_y;
	IMUDataMsg.angular_velocity_z=mesGyro.angularVelocity_z;
	IMUDataMsg.has_angular_velocity_x=1;
	IMUDataMsg.has_angular_velocity_y=1;
	IMUDataMsg.has_angular_velocity_z=1;
	IMUDataMsg.linear_acceleration_x=mesGyro.linearAcceleration_x;
	IMUDataMsg.linear_acceleration_y=mesGyro.linearAcceleration_y;
	IMUDataMsg.linear_acceleration_z=mesGyro.linearAcceleration_z;
	IMUDataMsg.has_linear_acceleration_x=1;
	IMUDataMsg.has_linear_acceleration_y=1;
	IMUDataMsg.has_linear_acceleration_z=1;
	IMUDataMsg.magnetic_field_x=mesGyro.magneticField_x;
	IMUDataMsg.magnetic_field_y=mesGyro.magneticField_y;
	IMUDataMsg.magnetic_field_z=mesGyro.magneticField_z;
	IMUDataMsg.has_magnetic_field_x=1;
	IMUDataMsg.has_magnetic_field_y=1;
	IMUDataMsg.has_magnetic_field_z=1;

}


//更新里程数据
static void ReadOdometryData(void)
{
	int32_t  left_temp_odometry_data;
	int32_t  right_temp_odometry_data;
	if(ConfigCommandCMD.motor_driver_type == 1||ConfigCommandCMD.motor_driver_type == 2)
	{

		GetLeftDecoderInfo(&left_temp_odometry_data, &OdometryDataMsg.left_delta_count);
		GetRightDecoderInfo(&right_temp_odometry_data, &OdometryDataMsg.right_delta_count);
		//根据上位机配置上报odom原始值为int32或int16类型的数据。
		if(ConfigCommandCMD.odom_bit_len==16||ConfigCommandCMD.odom_bit_len==0)
		{
			OdometryDataMsg.left_count = (int16_t)left_temp_odometry_data;
			OdometryDataMsg.right_count = (int16_t) right_temp_odometry_data;
		}
		else if(ConfigCommandCMD.odom_bit_len==32)
		{
			OdometryDataMsg.left_count = left_temp_odometry_data;
			OdometryDataMsg.right_count = right_temp_odometry_data;
		}
	}
	else if(ConfigCommandCMD.motor_driver_type == 4)
	{	
		//根据上位机配置上报odom原始值为int32或int16类型的数据。
		if(ConfigCommandCMD.odom_bit_len==16||ConfigCommandCMD.odom_bit_len==0)
		{
			OdometryDataMsg.left_count=(uint16_t)(-ZLAC_Motor.left_odom_count);		//2022.6.15  add by cds
			OdometryDataMsg.left_delta_count= ZLAC_Motor.left_odom_dcount;
			OdometryDataMsg.right_count=(uint16_t)(ZLAC_Motor.right_odom_count); 
			OdometryDataMsg.right_delta_count=ZLAC_Motor.right_odom_dcount; 	
		}
		else if(ConfigCommandCMD.odom_bit_len==32)
		{
			OdometryDataMsg.left_count=(-ZLAC_Motor.left_odom_count);		//2022.6.15  add by cds
			OdometryDataMsg.left_delta_count= ZLAC_Motor.left_odom_dcount;
			OdometryDataMsg.right_count=(ZLAC_Motor.right_odom_count); 
			OdometryDataMsg.right_delta_count=ZLAC_Motor.right_odom_dcount; 	
		}		
	}


}

//更新版本信息
static void ReadVersionData(void)
{
	//主版本
	V_Main.type = MAIN_VERSION_TYPE;
	V_Main.major = MAIN_VERSION_MAJOR;
	V_Main.minor = MAIN_VERSION_MINOR;
	V_Main.build = MAIN_VERSION_BUILD;
	//日期版本
	V_Product.type = DATE_VERSION_TYPE;
	V_Product.major = DATE_VERSION_MAJOR;
	V_Product.minor = DATE_VERSION_MINOR;
	V_Product.build = DATE_VERSION_BUILD;
	//超声波版本
	GetUltrasonicVersion((version_t *)&V_Sonar);
    //gcub版本
	GetGcubVersion( (uint32_t*)&V_Gcub );
    //小架驱动器版本
	GetXd510Version((uint32_t*)&V_Xd510);
     //混合驱动器软件版本
	V_H25A_S.build=H25a_Info.soft_version&0xff;
	V_H25A_S.minor=(H25a_Info.soft_version>>8)&0xff;
	V_H25A_S.major=(H25a_Info.soft_version>>16)&0xff;
	//混合驱动器硬件版本
	V_H25A_H.type =  (H25a_Info.hard_version>>24)&0xff;;
	V_H25A_H.major = (H25a_Info.hard_version>>16)&0xff;
	V_H25A_H.minor = (H25a_Info.hard_version>>8)&0xff;
	V_H25A_H.build = H25a_Info.hard_version&0xff;
	//自研驱动器软件版本  
	V_ZLAC_S.type =  (ZLAC_Motor.soft_version>>24)&0xff;;
	V_ZLAC_S.major = (ZLAC_Motor.soft_version>>16)&0xff;
	V_ZLAC_S.minor = (ZLAC_Motor.soft_version>>8)&0xff;
	V_ZLAC_S.build =ZLAC_Motor.soft_version&0xff;	
    //imu版本
    if( !V_Gyro.major && !V_Gyro.minor && !V_Gyro.build )
        app_imu_get_version( (uint32_t*)&V_Gyro );
   
}

  
//更新设备信息
static void ReadDevData(void)
{

	DeviceDATA.Relay = GetRelayStatus();
	DeviceDATA.Emergency = GetEmergencyStopState();
	DeviceDATA.Braker_Down = GetMotorBrakeDownState();
	DeviceDATA.Charger_Voltage_ADC = GetChargeVoltage();
	DeviceDATA.Battery_Voltage_ADC = BMS_Info.vol;
	DeviceDATA.Charge_Current_ADC = BMS_Info.charge_current;
	DeviceDATA.BatterySoc =  BMS_Info.soc;	
	DeviceDATA.Auto_Mode = GetCurCtrMode();
	DeviceDATA.Power_Switch = GetKeyState();
//	HandChargeVotage = GetManualChargeVoltage();

	

#ifdef USE_MODE_BUTTON
	DeviceDATA.mode_button_status = GetTrolleyModeStatusByButton();
#endif
	
	DeviceDATA.mcu_error_code[13] = m_pc_disconnect_flag|(GetAntiDropDisconnectStatus()<<5)\
    |(!GetUltrasonicHealthState()<<4)|(m_speed_timeout_flag<<13)|(GetGcubDisconnectStatus()<<20) ;
	if(ConfigCommandCMD.hardware_type == 1) //非混合驱控整车
	{
		DeviceDATA.mcu_error_code[13] |=((!GetLeftBrushConnectStatus( )<<17)|(!GetRightBrushConnectStatus( ))<<18);
	}
	if(ConfigCommandCMD.motor_driver_type == 1||ConfigCommandCMD.motor_driver_type == 2)
	{
		DeviceDATA.mcu_error_code[13] |= ((!HealthDATA.Left_Motor<<1)|(!HealthDATA.Right_Motor<<2));	
		DeviceDATA.mcu_error_code[14] = GetLeftMotorErrorCode();
		DeviceDATA.mcu_error_code[15] = GetRightMotorErrorCode();
		DeviceDATA.left_motor_load_factor = GetLeftMotorLoadFactor();
		DeviceDATA.right_motor_load_factor = GetRightMotorLoadFactor();
	}		
	else if(ConfigCommandCMD.motor_driver_type == 4)
	{
		DeviceDATA.mcu_error_code[13] |= ((!HealthDATA.Left_Motor<<1)|(!HealthDATA.Right_Motor<<2));	
		DeviceDATA.zlac_error_code1 = GetMotorLeftErrorCode();							//自研轮毂错误码		add by cds 2022.6.15
		DeviceDATA.zlac_error_code2 = GetMotorRightErrorCode();
		DeviceDATA.zlac_version = ZLAC_Motor.soft_version;
	}		

    DeviceDATA.mcu_error_code[16] = GetXd510SysErrBits1();
    DeviceDATA.mcu_error_code[17] = GetXd510SysErrBits2();
    DeviceDATA.vacuum_motor_current = (GetXdsVacuumCur()>0)?GetXdsVacuumCur():GetVacuumMotorCurrent();

    DeviceDATA.mcu_error_code[18] = (GetLRTemperatureAlarm()<<3)|(GetLRCurrentAlarm()<<19)|\
        (GetDeviceMotorCurrentAlarm()<<16|(GetRollerOvercurrentAlarm()<<30))|(GetSideBrushOvercurrentAlarm()<<28)|\
        (GetXdsVacuumOC()<<23)|(Side_Brush_Lift_Info.over_current_status<<31)|(Squeegee_Lift_Info.over_current_status<<22);

	DeviceDATA.dust_bag_status = GetDustBagStatus() ;
	DeviceDATA.dust_bag_error_code = GetDustBagErrorStatus() ;
	DeviceDATA.mcu_error_code[20] = GetAntiCollisionDisconectStatus()|(DeviceDATA.dust_bag_error_code<<16)|(DeviceDATA.dust_bag_status<<17)| (DeviceDATA.hull_disconnect_flag<<18);
    DeviceDATA.mcu_error_code[24] = GetXd510SysErrSelf()|GetZdDriverAlarm()|(GetHepaDustBagAlarm()<<30);
    DeviceDATA.mcu_error_code[27] = GetXdsAlarm();
	if(ConfigCommandCMD.side_motor_type==1)
	{
	  DeviceDATA.mcu_error_code[27] |= (GetLeftSideAlarm()<<16|GetRightSideAlarm()<<24);
	}

	DeviceDATA.dust_bag_voltage1=GetDustBag1Voltage();  //
	
	DeviceDATA.Squeegee_Motor_Working = (GetXdsVacuumSt())?GetXdsVacuumSt():GetVacuumMotorWorkStatus();
	if(ConfigCommandCMD.hardware_type == 0) //E4.1整车
	{

		DeviceDATA.Brush_Motor_Working = Roller_motor_Info.tx_cmd;               //滚刷工作状态
		DeviceDATA.side_brush_down =Side_Brush_Lift_Info.run_status;              //边刷推杆状态
		DeviceDATA.Squeegee_Down = Squeegee_Lift_Info.run_status;                //吸水趴升降状态
		DeviceDATA.left_brush_motor_working=Side_motor_Info[0].tx_cmd;            //左边刷转速  
		DeviceDATA.right_brush_motor_working = Side_motor_Info[1].tx_cmd;         //右边刷转速
		DeviceDATA.h25a_error_code=H25a_Info.error_code;
		DeviceDATA.h25a_error_code_sub = H25a_Info.error_code_sub;
		DeviceDATA.Brush_Down=Roller_Brush_Lift_Info.run_status;
	}
	else                              //非E4.1整车
	{
		DeviceDATA.side_brush_down = GetSideBrushLiftStatus();
		DeviceDATA.Brush_Down = GetBrushLiftStatus();
		DeviceDATA.Squeegee_Down = GetSqueegeeLiftStatus();
		DeviceDATA.Brush_Motor_Working = GetRollerWorkStatus();
		DeviceDATA.vacuum_motor_current = GetVacuumMotorCurrent();
		DeviceDATA.spray_motor_current = GetSprayMotorCurrent();
		DeviceDATA.filter_motor_current = GetDustClearMotorCurrent();
	}	
	    
}
static void ReadOtherDevData(void)
{
	if(ConfigCommandCMD.motor_driver_type == 1||ConfigCommandCMD.motor_driver_type == 2)
	{
		Other_DeviceData.left_motor_current = GetLeftMotorCurrent();
		Other_DeviceData.right_motor_current = GetRightMotorCurrent();
		Other_DeviceData.left_motor_temperature = GetLeftMotorTemperature();
		Other_DeviceData.right_motor_temperature = GetRightMotorTemperature();
		Other_DeviceData.motor_mode = GetHollySysControlMode(); //ConfigCommandCMD.control_mode 硬件助力
	}		
	else if(ConfigCommandCMD.motor_driver_type == 4)
	{
		Other_DeviceData.left_motor_current =  ABS(ZLAC_Motor.left_motor_current)/39.7; //单位转换为0.1A
		Other_DeviceData.right_motor_current = ABS(ZLAC_Motor.right_motor_current)/39.7;
		Other_DeviceData.left_motor_temperature = ZLAC_Motor.left_temperature;
		Other_DeviceData.right_motor_temperature = ZLAC_Motor.right_temperature ;
		Other_DeviceData.motor_mode =ZLAC_Motor.work_mode==3?0:3; //ConfigCommandCMD.control_mode 硬件助力
	}

    Other_DeviceData.imu_delta_yaw = GetDeltaYaw();
    Other_DeviceData.anti_drop_switch = gt_AntiDrop.output_switch;

    Other_DeviceData.torque_voltage1 = GetTorqueVoltage1();
    Other_DeviceData.torque_voltage2 = GetTorqueVoltage2();
	Other_DeviceData.side_door_status = GetSideDoorOpenStatus();	

    Other_DeviceData.emergency_button = GetEmergencyButtonStatus();
    Other_DeviceData.disinfect_atomization_box_level = GetDisinfectLevel();
    Other_DeviceData.disinfect_mist_spray_level = GetDisinfectRelayStatus();

	if(ConfigCommandCMD.hardware_type == 0) //E4.1整车
	{
		Other_DeviceData.Brush_Down=Roller_Brush_Lift_Info.run_status;
		Other_DeviceData.side_lift_motor_current = Side_Brush_Lift_Info.current;
		Other_DeviceData.side_lift_position = Side_Brush_Lift_Info.run_status;
		Other_DeviceData.roller_motor_position=Self_Adaption.real_position;
		Other_DeviceData.brush_motor_current = RollerBrushMotor1.current.ActualUnitValue/10;//滚刷电流 混合驱控未修复
		Other_DeviceData.driver_heartbeat_time = H25A_Get_Heartbeat_Time();
	}
	else                              //非E4.1整车
	{
		Other_DeviceData.xd510_temperature = GetXd510Temperature();
		Other_DeviceData.brush_motor_current = GetRollerCurrent();
		Other_DeviceData.brush_lift_motor_current = GetBrushLiftMotorCurrent();
		Other_DeviceData.squeegee_lift_motor_current = GetSqueegeeLiftMotorCurrent();
		Other_DeviceData.dust_push_spin = GetDustMotorWorkStatus();
		Other_DeviceData.dust_push_clean_spin = GetDustClearMotorWorkStatus();
		Other_DeviceData.dust_push_current = GetDustMotorCurrent();
		Other_DeviceData.dust_push_clean_current = GetDustClearMotorCurrent();
		Other_DeviceData.left_side_motor_current=GetLeftBrushCurrent();
		Other_DeviceData.right_side_motor_current=GetRightBrushCurrent();
		Other_DeviceData.roller_motor_position = GetBrushLiftStatus();	

		
		
	}
}
extern uint32_t otaing_tick;
//更新健康状态（是否失联或者报警）信息.1=健康。0=不健康
static void ReadHealthData(void)
{
	if(ConfigCommandCMD.hardware_type == 0) //E4.1整车
	{
		HealthDATA.Brush_Motor          = 1;
		HealthDATA.left_brush_motor     = 1;
		HealthDATA.right_brush_motor    = 1;
		HealthDATA.Motor_Driver 		= H25A_Get_Link_Status(); 	
	}
	else
	{
		HealthDATA.Motor_Driver 		= GetXd510HealthStatus();
		HealthDATA.Brush_Motor          = GetRollerConnectStatus();
		HealthDATA.left_brush_motor     = GetLeftBrushConnectStatus();
		HealthDATA.right_brush_motor    = GetRightBrushConnectStatus(); 
	}

	if(ConfigCommandCMD.motor_driver_type == 1||ConfigCommandCMD.motor_driver_type == 2)
	{
		HealthDATA.Left_Motor 			= GetLeftMotorHealthState(); 
		HealthDATA.Right_Motor 			= GetRightMotorHealthState(); 
	}
	if(ConfigCommandCMD.motor_driver_type == 4)
	{
		HealthDATA.Left_Motor 			= Get_Homemade_LeftHealthState(); 	//自研驱动器
		HealthDATA.Right_Motor 			= Get_Homemade_RightHealthState(); 
	}	
	if(xTaskGetTickCount()-otaing_tick<15000) //混合驱控OTA过程中不报驱动器断链
	{
		HealthDATA.Motor_Driver 		= 1;
	 	HealthDATA.Left_Motor 			= 1; 
		HealthDATA.Right_Motor 			= 1; 
	}
	HealthDATA.Ultrasonic_Board 	= GetUltrasonicHealthState(); 
	HealthDATA.Imu_Board 			= GetGyroHealthState(); 

	HealthDATA.battery_connect      = BMS_Info.disconnect_flag?0:1; 

}
//超声波数据
static void ReadUltrasonicData(void)
{
	GetUltrasonicDistance(&UltrasonicDATA.ks103_data0);
}

static void ReadBatteryData(void)
{
	bms_get_battery_data(&g_battery_data);
}
static void ReadProtectorData(void)
{
	g_protector_data.rfid_alarm = GetRfidProtectFlag();
}

static void ReadRfidData(void)
{
	GetRfidData(g_rfid_tag_table,sizeof(g_rfid_tag_table));
}
//更新设备数据
static void ReadAllDataFromDevice(void)
{
	//更新电池数据
	ReadBatteryData();
	//更新版本信息
	ReadVersionData();
	//更新陀螺仪数据
	ReadGyroData();
	//更新里程数据
	ReadOdometryData();
	//更新设备数据
	ReadDevData();
	//更新健康状态
	ReadHealthData();
	//超声波数据
	ReadUltrasonicData();
	//RFID数据
	ReadRfidData();
    //更新设备数据
    ReadOtherDevData();

	//读取防护数据
	ReadProtectorData();


}
//写关机命令
static void WriteShutdownCtrData(void)
{
	if(ShutDown_CMD.type == 1)
    {
		CtrShutDown(ShutDown_CMD.type);
    }
    else if (ShutDown_CMD.type == 2)
    {
        ResartHost();
    }
	ShutDown_CMD.type = 0;
}

//写速度
static void WritePcSpeedData(void)
{
	//速度
	if(GetMotorCmdState() == 1)
	{
		Hub_Info.left_rx_speed =MotionControlMotorCommandCMD.MotionControlMotorCommand_Low;
		Hub_Info.right_rx_speed = MotionControlMotorCommandCMD.MotionControlMotorCommand_High;
		ResetConnectedTime(&m_pc_speed_connect_status);
		SetPcSpeed(MotionControlMotorCommandCMD.MotionControlMotorCommand_Low,MotionControlMotorCommandCMD.MotionControlMotorCommand_High);
		SetMotorCmdState(0);
	}
	if( !GetLinkStatus(&m_pc_speed_connect_status) ) //速度下发超时告警
	{
		SetPcSpeed(0,0);
		m_speed_timeout_flag = 1;
	}
}



//写继电器
static void WriteRelayCtrData(void)
{
	SetRelay(DeviceCommandCMD.Relay);
}
//写普通输出口
static void WriteNormalIoOutputCtrData(void)
{
//	 CtrNormalOutputIo(DeviceCommandCMD.Do);
}
//写刹车命令
void WriteBrkCtrData(void)
{
	SetRobotBrkInAutoMode((brkStatus_t)DeviceCommandCMD.Braker_Down);
}

//写xd510命令
static void WriteXd510Data(void)
{
	SetDustMotor( DeviceCommandCMD.dust_push_spin_level );
	SetVacuumMotor( DeviceCommandCMD.Squeegee_Motor, DeviceCommandCMD.Fan_Levle );
	SetDustClearMotor( DeviceCommandCMD.dust_push_clean_spin_level );
	SetBrushLift( DeviceCommandCMD.brush_pressure_level );
	SetBrushLiftPosition(  DeviceCommandCMD.brush_lift_taget_position);
	SetSqueegeeLift(DeviceCommandCMD.Squeegee_Down );
	SetRobotWorkMode( DeviceCommandCMD.work_status );
	SetSideBrushLift(DeviceCommandCMD.side_brush_down);
	Xd510_Self_Adaption.target_current = DeviceCommandCMD.self_adapt_target_current;
	Xd510_Self_Adaption.debounce_time = ConfigCommandCMD.self_adaption_filter_time;    //自适应电流滤波时间

}
//更新自研驱控板控制相关命令
static void WriteH25A15Data(void)
{
	Roller_motor_Info.rx_cmd=DeviceCommandCMD.brush_spin_level;               //滚刷转速
	Side_motor_Info[0].rx_cmd=DeviceCommandCMD.left_brush_pwm;                //左边刷转速
	Side_motor_Info[1].rx_cmd=DeviceCommandCMD.ritht_brush_pwm;               //右边刷转速
	Squeegee_Lift_Info.rx_cmd=DeviceCommandCMD.Squeegee_Down;                 //吸水趴升降控制
	Roller_Brush_Lift_Info.rx_cmd=  DeviceCommandCMD.brush_lift_taget_position;//滚刷升降控制
	Side_Brush_Lift_Info.rx_cmd=DeviceCommandCMD.side_brush_down;             //边刷升降控制	
	Self_Adaption.self_adaption_enable=ConfigCommandCMD.self_adaption_enable; //自适应使能
	Self_Adaption.target_current= DeviceCommandCMD.self_adapt_target_current ;//自适应目标电流                                                 
	Self_Adaption.deviation=ConfigCommandCMD.self_adaption_deviation;         //自适应允许电流偏差
//	Self_Adaption.filter_time= ConfigCommandCMD.self_adaption_filter_time;    //自适应电流滤波时间
	Side_Brush_Lift_Info.max_current= ConfigCommandCMD.side_motor_max_current;//滚刷推杆电机最大电流 单位ma
    Side_Brush_Lift_Info.over_current_time= ConfigCommandCMD.side_motor_max_current_time ;  
	Squeegee_Lift_Info.max_current= ConfigCommandCMD.squeegee_lift_motor_max_current;  //滚刷推杆电机最大电流 单位ma
	Squeegee_Lift_Info.over_current_time= ConfigCommandCMD.squeegee_lift_max_current_time ;  
	Side_Brush_Lift_Info.operate_time = ConfigCommandCMD.side_lift_operate_time;
	Squeegee_Lift_Info.operate_time =ConfigCommandCMD.squeegee_lift_operate_time;
	if(DeviceCommandCMD.clean_error)	
	{
		DeviceCommandCMD.clean_error=0;
		Side_Brush_Lift_Info.over_current_status=0;
		Squeegee_Lift_Info.over_current_status=0;
		H25A_Error_Clear();
	}

}

//写设备命令
static void WriteDevCmdData(void)
{
    static uint8_t s_auto_mode = 0;
//    static TickType_t s_tick;
    
	//继电器
	WriteRelayCtrData();
	//普通输出口
	WriteNormalIoOutputCtrData();
	//刹车
#ifdef USE_BREAK
	WriteBrkCtrData();
#endif
	//关机
	WriteShutdownCtrData();
    //机器人手自动模式
    if( !GetEmergencyStopState() )
    {
        SetRobotTrolleyMode( DeviceCommandCMD.Auto_Mode );
		if( s_auto_mode && !DeviceCommandCMD.Auto_Mode )
			ClearRfidProtectFlag();
        s_auto_mode = DeviceCommandCMD.Auto_Mode; //急停前的手自动命令状态
        if( s_auto_mode )
            ConfigCommandCMD.control_mode = MODE_SPEED;
    }
    else
    {
//        if( GetPcConnectStatus() )
//        {
//            if( s_auto_mode == DeviceCommandCMD.Auto_Mode )
//            {
//                s_tick = xTaskGetTickCount();
//            }
//            else if( xTaskGetTickCount() - s_tick >= 60000 )
//            {
        DeviceCommandCMD.Auto_Mode = s_auto_mode;
//            }
//        }
    }
	
	if(ConfigCommandCMD.hardware_type == 0) //E4.1整车
	{
		WriteH25A15Data();
	}
	else                              //非E4.1整车
	{
		WriteXd510Data();
		SetRollerMotorDir( DeviceCommandCMD.roller_dir ); //
		SetRollerMotor( DeviceCommandCMD.Brush_Motor, DeviceCommandCMD.brush_spin_level );
	}
	SetLeftBrushMotor( DeviceCommandCMD.side_brush_enable, DeviceCommandCMD.left_brush_pwm);
	SetRightBrushMotor( DeviceCommandCMD.side_brush_enable, DeviceCommandCMD.ritht_brush_pwm);
	
	SetAntiDropSwitch( DeviceCommandCMD.anti_drop_switch );
    if( DeviceCommandCMD.clear_emergency )
    {
        ClearEmergencyStop();
        DeviceCommandCMD.clear_emergency = 0;
    }
	if( DeviceCommandCMD.set_emergency )
    {
        SetEmergency( 1 );
        DeviceCommandCMD.set_emergency = 0;
    }
	SetVacuumCloseDelay( DeviceCommandCMD.keep_vaccum);
    SetXdsVacuumPwm( DeviceCommandCMD.Squeegee_Motor, DeviceCommandCMD.Fan_Levle );
    SetDisinfect2_0Work( DeviceCommandCMD.disinfect_mist_spray_level);

    set_hub_motor_damping(DeviceCommandCMD.hub_motor_damping);
    if ((!get_hub_motor_power_state()) && DeviceCommandCMD.hub_motor_power_on) /*  */
        DevicePowerOn();
    else if (get_hub_motor_power_state() && (!DeviceCommandCMD.hub_motor_power_on))
        DevicePowerOff();

}

//配置充电相关的数据
static void CfgChargeParameter(void)
{
	chargeConfig_t chargeConfig = {0};

	chargeConfig.batteryVoltageLow = ConfigCommandCMD.Battery_Empty_Voltage_adc;//电池电压无输入时的最大空载电压
	chargeConfig.batteryVoltageEmpty = 0;//复充电压			上位机没有配置
	chargeConfig.batteryVoltageFull = ConfigCommandCMD.Charge_Full_Voltage;//充满时的电压
	chargeConfig.chargeCurrentLow = ConfigCommandCMD.Charge_Empty_Current_adc;//无充电电流输入时的最大空载电流
	chargeConfig.chargeCurrentEmpty = ConfigCommandCMD.Charge_Full_Current;//充满电时的电流
	chargeConfig.chargeCurrentFast = 0;//正常充电时的电流，大电流充电       			上位机没有配置  
	chargeConfig.chargeCurrentShort = ConfigCommandCMD.Charge_Short_Current;//短期电流，指充电刚开始阶段的电流？
	chargeConfig.chargeVoltageLow = ConfigCommandCMD.Charge_Empty_Voltage_adc;//充电电压无输入的最大空载电压
	chargeConfig.chargeTouchVoltageLow = ConfigCommandCMD.Charge_Touched_Voltage_Low;//充电接触电压的低值
	chargeConfig.chargeTouchVoltageHigh = ConfigCommandCMD.Charge_Touched_Voltage_High;//接触电压的高值
	chargeConfig.chargeMaxTime = ConfigCommandCMD.Max_Charge_Time;//单次最长充电时间*2为实际S数

	CfgChargeData(chargeConfig);

}

//配置超声波参数
static void CfgUltrasonicParameter(void)
{
	sensorDirCfg_t ultrasonicDirCfg = {0};
	//方向设置，这里表示那几个端口表示当前的方向
	ultrasonicDirCfg.front		= ConfigCommandCMD.Ultrasonic_Front;
	ultrasonicDirCfg.back 		= ConfigCommandCMD.Ultrasonic_Rear;
	ultrasonicDirCfg.left 		= ConfigCommandCMD.Ultrasonic_Left;
	ultrasonicDirCfg.right		= ConfigCommandCMD.Ultrasonic_Right;
	ultrasonicDirCfg.leftFront	= ConfigCommandCMD.Ultrasonic_Left_Front;
	ultrasonicDirCfg.rightFront	= ConfigCommandCMD.Ultrasonic_Right_Front;
	ultrasonicDirCfg.leftBack 	= ConfigCommandCMD.Ultrasonic_Left_Rear;
	ultrasonicDirCfg.rightBack	= ConfigCommandCMD.Ultrasonic_Right_Rear;	

	GetUltrasonicCfgPara(ultrasonicDirCfg, ConfigCommandCMD.Ultrasonic_Enable);
}

//写配置命令
static void WriteConfigCmdParameter(void)
{
    static uint8_t s_last_wheel_type = 0;
    static uint8_t s_count = 0;
	if(GetHasGotCfgParaFlag())
    {
        s_count = 2;
        SetHasGotCfgParaFlag(0);
    }
    if( s_count>0 )
        s_count--;
    else
        return;
    if( !s_last_wheel_type && s_last_wheel_type!=ConfigCommandCMD.wheel_type ) //光编切磁编
    {
        s_last_wheel_type = ConfigCommandCMD.wheel_type;
        ConfigCommandCMD.left_motor_max_temperature = 100;
        ConfigCommandCMD.right_motor_max_temperature = 100;
    }
    else if( s_last_wheel_type && s_last_wheel_type!=ConfigCommandCMD.wheel_type ) //磁编切光编
    {
        s_last_wheel_type = ConfigCommandCMD.wheel_type;
        ConfigCommandCMD.left_motor_max_temperature = 75;
        ConfigCommandCMD.right_motor_max_temperature = 75;
    }
    

	//配置充电相关的参数
	CfgChargeParameter();

	//超声波配置
	CfgUltrasonicParameter();
	//配置RFID功率
//	CfgRfidPowerParameter();
    //配置轮毂电机在自动充电时最大电流
    SetMotorMaxCurrentInCharge( ConfigCommandCMD.run_motor_max_current_in_charge );
    bms_set_disconnec_time( ConfigCommandCMD.bms_max_disconnect_time );
    bms_set_recover_time( ConfigCommandCMD.bms_recover_time );
    ConfigAntiCollision( (uint16_t)(ConfigCommandCMD.Anti_Collision_Front\
                        |ConfigCommandCMD.Anti_Collision_Left\
                        |ConfigCommandCMD.Anti_Collision_Left_Front\
                        |ConfigCommandCMD.Anti_Collision_Left_Rear\
                        |ConfigCommandCMD.Anti_Collision_Rear\
                        |ConfigCommandCMD.Anti_Collision_Right\
                        |ConfigCommandCMD.Anti_Collision_Right_Front\
                        |ConfigCommandCMD.Anti_Collision_Right_Rear),
                        ConfigCommandCMD.anti_collision_period_time,
                        ConfigCommandCMD.Anti_Collision_Threshold,\
                        ConfigCommandCMD.anticollision_valid_voltage);
	ConfigAntiDrop( (uint16_t)(ConfigCommandCMD.Anti_Drop_Front\
                        |ConfigCommandCMD.Anti_Drop_Left\
                        |ConfigCommandCMD.Anti_Drop_Left_Front\
                        |ConfigCommandCMD.Anti_Drop_Left_Rear\
                        |ConfigCommandCMD.Anti_Drop_Rear\
                        |ConfigCommandCMD.Anti_Drop_Right\
                        |ConfigCommandCMD.Anti_Drop_Right_Front\
                        |ConfigCommandCMD.Anti_Drop_Right_Rear),
						ConfigCommandCMD.Anti_Drop_Threshold );
	SetHollySysDriverType( ConfigCommandCMD.motor_driver_type );
    SetHollySysPidIncharge(ConfigCommandCMD.velocity_1P, ConfigCommandCMD.velocity_1I );
    SetHollySysMotorTemperature( ConfigCommandCMD.left_motor_max_temperature, ConfigCommandCMD.right_motor_max_temperature );
	SetHollySysMotorCurrent(ConfigCommandCMD.left_motor_max_current, ConfigCommandCMD.right_motor_max_current );
    SetHullLiftTravelTable( ConfigCommandCMD.hull_lift_travel_low, ConfigCommandCMD.hull_lift_travel_median, ConfigCommandCMD.hull_lift_travel_high);
    SetRollerMaxCurrent( ConfigCommandCMD.brush_motor_max_current);
    SetSideLiftTravelTable(ConfigCommandCMD.side_lift_travel_low, ConfigCommandCMD.side_lift_travel_median, ConfigCommandCMD.side_lift_travel_high );
    SetVacuumMaxCurrent(ConfigCommandCMD.vacuum_motor_max_current );
    SetBrushLiftMaxCurrent(ConfigCommandCMD.brush_lift_motor_max_current );
    SetSqueegeeLiftMaxCurrent(ConfigCommandCMD.squeegee_lift_motor_max_current );
    SetHollySysControlMode(ConfigCommandCMD.control_mode);
    ConfigEmergencyParam( ConfigCommandCMD.emergency_type, ConfigCommandCMD.emergency_status_min_time);
    ConfigDisinfectEnable( ConfigCommandCMD.disinfect_enable );

}

//更新
static void WriteAllDataToDevice(void)
{
	//写速度
	WritePcSpeedData();
	//写设备命令
	WriteDevCmdData();
	//写配置命令
	WriteConfigCmdParameter();
}

static void DisconnectStopCmd(uint8_t f_type )
{
	DeviceCommandCMD.brush_spin_level = 0;
	DeviceCommandCMD.Fan_Levle = 0;
	DeviceCommandCMD.filter_motor_pwm = 0;
	DeviceCommandCMD.Water_Level = 0;
    DeviceCommandCMD.side_brush_enable = 0;
    DeviceCommandCMD.left_brush_pwm = 0;
    DeviceCommandCMD.ritht_brush_pwm = 0;
    DeviceCommandCMD.dust_push_spin_level = 0;
    DeviceCommandCMD.dust_push_clean_spin_level = 0;
    DeviceCommandCMD.disinfect_mist_spray_level =0;
    DeviceCommandCMD.disinfect_spray_distance = 0;
    DeviceCommandCMD.disinfect_box_beep = 0;
    DeviceCommandCMD.disinfect_disinfectant_pump = 0;
}

static void WriteIpcBreakLog(uint8_t f_type)
{
//    LogIpcBreak_t log_break = {0};
//    
//    log_break.head.byte.id = ID_IPC_BREAK;
//    log_break.type = f_type;
//    log_break.utc = GetUtc8();
//    xQueueSend( g_log_queue, &log_break, 2);
}

/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口定义
*************************************************************************************************************************************/

//对速度进行操作
void GetPcSpeed(int32_t *fp_leftSpeed,int32_t *fp_rightSpeed)
{
	xSemaphoreTake(m_mutex_speed, portMAX_DELAY);
	*fp_leftSpeed = m_left_speed;
	*fp_rightSpeed = m_right_speed;
    xSemaphoreGive(m_mutex_speed);
}
void SetPcSpeed(int32_t f_leftSpeed,int32_t f_rightSpeed)
{
	xSemaphoreTake(m_mutex_speed, portMAX_DELAY);
	m_left_speed= f_leftSpeed;
	m_right_speed = f_rightSpeed;
    xSemaphoreGive(m_mutex_speed);
}

//非零表示连接正常，0表示连接异常
int GetPcConnectStatus(void)
{
	static uint32_t delay_tick = 0;  //PRO800V3.0收到上位机数据后会断一会，导致反馈断链状态
	
    if( m_pc_init_flag )//上位机初始化时认为未断链
    {
		delay_tick = xTaskGetTickCount();
        return 1;
    }
    else
    {	if(xTaskGetTickCount() - delay_tick >9000)
		{
			return GetLinkStatus(&m_pc_connect_status);
		}
		else
		{
			return 1;
		}
    }
}

void ClearPcDisconnect(void)
{
    if( DeviceDATA.mcu_error_code[13] & 0x01 )
        m_pc_disconnect_flag = 0;
}

void ClearSpeedTimeout(void)
{
    if( DeviceDATA.mcu_error_code[13] & 0x2000 )
        m_speed_timeout_flag = 0;
}

/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 1，处理数据的收发；2，w5500和pc的连接状态检测；3，w5500失联后的初始化
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 高 
*********************************************************************************************************
*/
void PcCorrespondTask(void const *pvParameters)
{
	TickType_t lastWakeTime;
    TickType_t last_update_device_data_tick = 0;
    TickType_t pc_init_tick = 0;
	const TickType_t frequency = pdMS_TO_TICKS(3);
    const TickType_t max_pc_init_time = pdMS_TO_TICKS(180000);
	uint32_t bitPos = (uint32_t)pvParameters;
    uint8_t thread_num = (uint8_t)(uint32_t)pvParameters;
#ifdef USE_MODE_BUTTON
    uint8_t mode_key_status = 1;
#endif
//    LogResetIpc_t log_reset = {0};
	uint8_t connect_flag = 0;//0初始，1链接，2断链，3 180秒未连接
	/* 获取当前的系统时间 */
    lastWakeTime = xTaskGetTickCount();
    last_update_device_data_tick = lastWakeTime;
    pc_init_tick = lastWakeTime;
	InitSoftwarePcCorrespond();

    while(1)
    {
		/* 喂狗 */
//		FeedDog(bitPos);
		/* vTaskDelayUntil是绝对延迟，vTaskDelay是相对延迟。*/
        vTaskDelay(frequency);

        //5，检测是否接收到了pc的数据
		if(GetRxBufSizeState() == 1)
		{
			ResetConnectedTime(&m_pc_connect_status);
            m_pc_init_flag = 0;
		}
        
        if( m_pc_init_flag )
        {
            if( xTaskGetTickCount() - pc_init_tick >= max_pc_init_time )
            {
                if( m_pc_init_flag == 1 && GetKeyState() == 0x01) //上位机上电后90秒未与下位机通讯
                {
                    m_pc_init_flag = 2;
                    //重启上位机
                    ResartHost();
                    pc_init_tick = xTaskGetTickCount();
//                    log_reset.head.byte.id = ID_RESET_IPC;
//                    log_reset.utc = GetUtc8();
//                    log_reset.type = 1;
//                    xQueueSend( g_log_queue, &log_reset, 2);
                }
                else //重启上位机后90秒仍未与下位机通讯认为通讯断链
                {
                    m_pc_init_flag = 0;
                }
            }
            SetRobotTrolleyMode( DeviceCommandCMD.Auto_Mode );
        }
        if( !m_pc_init_flag ) //上位机初始化完成
        {
            //2，判断与pc的链接状态
            if( !GetLinkStatus(&m_pc_connect_status) )//与pc失联则把当前的通信状态改为等待head信息
            {
#ifdef USE_MODE_BUTTON
                if( mode_key_status != GetTrolleyModeStatusByButton() )
                {
                    mode_key_status = GetTrolleyModeStatusByButton();
                    if( mode_key_status )
                    {
                        DeviceCommandCMD.Auto_Mode = !DeviceCommandCMD.Auto_Mode;
                        SetRobotTrolleyMode( DeviceCommandCMD.Auto_Mode );
						if( !DeviceCommandCMD.Auto_Mode )
							ClearRfidProtectFlag();
                    }
                }
#endif
				DisconnectStopCmd(0);
                if( connect_flag == 0 )
                {
                    connect_flag = 3;
                    WriteIpcBreakLog(3); //上电后180秒未接入下位机
                }
				else if( connect_flag == 1 )
				{
					connect_flag = 2;
					SetRxBufSizeStatus();
                    SetTcpCurrentState(RX_HEAD_MSG);
                    if( GetKeyState() == 0x01 )
                    {
                        WriteIpcBreakLog(1); //上下位机断链
                        m_pc_disconnect_flag = 1;
                    }
				}
            }
            else//对于控制类的命令，要保证在上位机失联之后就不再写
            {
                if( connect_flag != 1)
                {
                    if( connect_flag == 2)
                    {
                        WriteIpcBreakLog(2); //上下位机断链恢复
                    }
                    else if( !connect_flag || connect_flag == 3)
                    {
                        WriteIpcBreakLog(4);//上位机第一次接入下位机
                    }
                    connect_flag = 1;
                }
                //把上位机下发的数据更新到各个模块
                WriteAllDataToDevice();
#ifdef USE_MODE_BUTTON
                mode_key_status = GetTrolleyModeStatusByButton();
#endif
            }
            //3，判断与pc速度控制的链接状态
            if( !GetLinkStatus(&m_pc_speed_connect_status) )
            {
                SetPcSpeed(0,0);
                m_speed_timeout_flag = 1;
            }
        }
        //急停和断链时需要清的命令
		if( GetEmergencyStopState() )
		{
			DisconnectStopCmd(1);
		}
		
		//6,获取各个模块采集到的最新数据
        if( xTaskGetTickCount() - last_update_device_data_tick >= 5 )
        {
            last_update_device_data_tick = xTaskGetTickCount();
            ReadAllDataFromDevice();
        }
        
//        g_thread_call_count[thread_num]++;
    }
}


int InitSoftwarePcCorrespond(void)
{
	m_mutex_speed = xSemaphoreCreateMutex();
    
    DeviceCommandCMD.Auto_Mode = 0;//将手自动模式初始化为手动模式
	DeviceCommandCMD.keep_vaccum = 10;
    ConfigCommandCMD.velocity_1P = 50;
    ConfigCommandCMD.velocity_1I = 1;
    ConfigCommandCMD.left_motor_max_temperature = 75;
    ConfigCommandCMD.right_motor_max_temperature = 75;
    ConfigCommandCMD.left_motor_max_current = 25000;
    ConfigCommandCMD.right_motor_max_current = 25000;
    ConfigCommandCMD.vacuum_motor_max_current = 30000;
    ConfigCommandCMD.brush_lift_motor_max_current = 3000;
    ConfigCommandCMD.squeegee_lift_motor_max_current = 3000;
    
	ConfigCommandCMD.dust_bag_error_distance=40;//单位0.1cm
	ConfigCommandCMD.dust_bag_error_vol=5;//单位0.1V
	ConfigCommandCMD.dust_full_threshold = 10000;//过滤时间默认10S

    DeviceCommandCMD.anti_drop_switch = 1;
    
    ConfigCommandCMD.emergency_status_min_time = 500;
    ConfigCommandCMD.emergency_type = 0; //自锁型
    ConfigCommandCMD.self_adaption_enable=0;
	ConfigCommandCMD.self_adaption_deviation=20;
	ConfigCommandCMD.self_adaption_filter_time=2000;
	ConfigCommandCMD.hardware_type = 0;  
	ConfigCommandCMD.side_motor_max_current = 2000;//滚刷推杆电机最大电流 单位ma
	ConfigCommandCMD.side_motor_max_current_time = 2000 ;  
	ConfigCommandCMD.squeegee_lift_motor_max_current = 2000;  //滚刷推杆电机最大电流 单位ma
	ConfigCommandCMD.squeegee_lift_max_current_time = 2000 ;  
	Side_Brush_Lift_Info.operate_time = 9000;
	Squeegee_Lift_Info.operate_time = 9000;

    ConfigCommandCMD.side_door_sensor_type = 1;

	if(m_mutex_speed == NULL)
		return 1;
	return 0;
}












