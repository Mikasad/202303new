#include <string.h>
#include "nanopb_tcpip.h"
#include "protocol_config.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "service.pb.h"

#include "flash_op.h"
  #include "app_bms.h"
  #include "drv_gs_can_bms.h"
#include "pc_correspond.h"
#include "motor_driver.h"
#include "XD510.h"
#include "anti_collision.h"
#include "anti_drop_collision.h"
#include "zd_driver.h"
#include "rfid_etag.h"
#include "xds_blvacuum.h"
#include "scada.h"
#include "app_ota.h"
uint8_t nanopb_buf[NONOPD_BUF_LEN];
uint16_t g_nanopb_buf_len = 0;
#define UPDATE_FIFO_SIZE 256



#define ZLAC_PDO_KEY_START   0
#define H4AA30_PDO_KEY_START 50

#ifndef UWB_BOOSTAR_DATA
#define UWB_BOOSTAR_DATA	UWB_DISTANCE_DATA
#endif

#define MCU_ERROR_CODE_0	0
#define MCU_ERROR_CODE_1	1
#define MCU_ERROR_CODE_6	6
#define MCU_ERROR_CODE_7	7
/***************************************************************************************************************************
*             debug Define Section
****************************************************************************************************************************/

#define SPEED_BYTES_LEN			8
#define IOSTATE_BYTES_LEN		8
#define ODOMETRY_BYTES_LEN		16
#define CHARGER_BYTES_LEN		8
#define IMU_BYTES_LEN			24
#define SONIC_BYTES_LEN			16
#define DROP_BYTES_LEN			16
#define RFID_BYTES_LEN			8
#define VERSION_BYTES_LEN		24
#define CONFIG_FUNC_LEN			16
#define ERRCODE_FUNC_LEN		16

#define SPEED_ADR0		0
#define IOSTATE_ADR0	(SPEED_ADR0+SPEED_BYTES_LEN)
#define ODOMETRY_ADR0	(IOSTATE_ADR0+IOSTATE_BYTES_LEN)
#define CHARGER_ADR0	(ODOMETRY_ADR0+ODOMETRY_BYTES_LEN)

#define IMU_ADR0		(CHARGER_ADR0+CHARGER_BYTES_LEN)
#define SONIC_ADR0		(IMU_ADR0+IMU_BYTES_LEN)
#define DROP_ADR0		(SONIC_ADR0+SONIC_BYTES_LEN)

#define RFID_ADR0		(DROP_ADR0+DROP_BYTES_LEN)
#define VERSION_ADR0	(RFID_ADR0+RFID_BYTES_LEN)
#define CONFIGX_ADR0	(VERSION_ADR0+VERSION_BYTES_LEN)
#define ERRCODE_ADR0	(CONFIGX_ADR0+CONFIG_FUNC_LEN)
#define MAX_LEN 		(ERRCODE_ADR0+ERRCODE_FUNC_LEN)
/***************************************************************************************************************************
*             Struct Define Section
****************************************************************************************************************************/
HeaderMessage HderMsg = HeaderMessage_init_zero;
VerifyMessage VerifyMsg = VerifyMessage_init_zero;
DeviceCommand DeviceCmdMsg = DeviceCommand_init_zero;
ConfigCommand ConfigCmdMsg = ConfigCommand_init_zero;
MotionControlMotorCommand  MCMCmdMsg = MotionControlMotorCommand_init_zero;
ManualModeDeviceCommand MMDCmdMsg = ManualModeDeviceCommand_init_zero;
HealthData HealthDataMsg = HealthData_init_zero;
RemoteCommand433  RemoteCmd433Msg = RemoteCommand433_init_zero;
ShutdownCommand ShutdownCmdMsg = ShutdownCommand_init_zero;
DeviceData DeviceDataMsg = DeviceData_init_zero;
UltrasonicData UltrasonicDataMsg = UltrasonicData_init_zero;
IMUData   IMUDataMsg = IMUData_init_zero;
UWBData   UWBDataMsg = UWBData_init_zero;
OdometryData OdometryDataMsg = OdometryData_init_zero;
MotionControlMotorCommand_MotorCommandPair  MCMC_MCmdPair = MotionControlMotorCommand_MotorCommandPair_init_zero;
RemoteData433     RemoteData433Msg = RemoteData433_init_zero;
VersionData VersionDataMsg = VersionData_init_zero;
ProtectorData ProtectorDataMsg = ProtectorData_init_zero;
RFIDData RFIDDataMsg = RFIDData_init_zero; 
UWBDistanceData BooStarDataMsg = UWBDistanceData_init_zero; 
AntiDropData AntiDropDataMsg = AntiDropData_init_zero;
AnalogAntiCollisionData AntiCollisionDataMsg = AnalogAntiCollisionData_init_zero;
DriverConfigCommand DriverConfigDataMsg = MotorDriverConfigData_init_zero;
HardwareDriverData HardWareDriverDataMsg = HardwareDriverData_init_zero;
ClearErrorCommand ClearErrorCmdMsg = ClearErrorCommand_init_zero;
SaberIMUData IMUDataTestMsg= SaberIMUData_init_zero;
MCUUpgradeCommand MCU_Update_CMD = MCUUpgradeCommand_init_zero;
MCUUpgradeStatusData MCU_Update_DATA = MCUUpgradeStatusData_init_zero;
BatteryData BatteryDataMsg = BatteryData_init_zero;
OtherDeviceData OtherDeviceDataMsg = OtherDeviceData_init_zero;
DriverData DriverDataMsg = DriverData_init_zero;

MotorDriverConfigData MotorDriverConfigDataMsg = MotorDriverConfigData_init_zero;

PhantasMcuUpgradeCommand PhantasMcuUpgrade_CMD = PhantasMcuUpgradeCommand_init_zero;
PhantasMcuUpgradeCommandStatusData PhantasMcuUpgrade_DATA = PhantasMcuUpgradeCommandStatusData_init_zero;
PhantasMcuUpgradePackage PhantasMcuUpgradePackage_CMD = PhantasMcuUpgradePackage_init_zero;
PhantasMcuUpgradePackageStatusData PhantasMcuUpgradePackage_DATA = PhantasMcuUpgradePackageStatusData_init_zero;
PhantasMcuTypeData PhantasMcuType_DATA = PhantasMcuTypeData_init_zero;
CANBusCommand CanBus_CMD = CANBusCommand_init_zero;
CANBusStatusData CanBus_DATA = CANBusStatusData_init_zero;
/*******************************************************************************/
//这些结构体用来暂存数据和指令
/*******************************************************************************/

static uint32_t GenerateHash(uint32_t seed) ;
//static uint8_t ProtectorCheck(void);
static bool MCU_update_callback(pb_istream_t *stream, const pb_field_t *field, void **arg);
static bool ConfigCMD_callback(pb_istream_t *stream, const pb_field_t *field, void **arg);
static bool ClearErrCMD_callback(pb_istream_t *stream, const pb_field_t *field, void **arg);
static bool DriverConfig_callback(pb_istream_t *stream, const pb_field_t *field, void **arg);
static bool MotorDriverConfigData_callback(pb_ostream_t* stream, const pb_field_t* field, void* const* arg);
static bool McmCMD_callback(pb_istream_t *stream, const pb_field_t *field, void **arg);
static bool MmdCMD_callback(pb_istream_t *stream, const pb_field_t *field, void **arg);
static bool DeviceCMD_callback(pb_istream_t *stream, const pb_field_t *field, void **arg);
//bool config_callback(pb_istream_t *stream, const pb_field_t *field, void **arg);
//static bool BooSTAR_DATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
//static bool RFIDDATA_callback_test(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static bool VersionDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static bool HealthDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static bool DeviceDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static bool OtherDeviceData_callback(pb_ostream_t* stream, const pb_field_t* field, void* const* arg);
static bool DriverData_callback(pb_ostream_t* stream, const pb_field_t* field, void* const* arg);
static bool UltraDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static bool ProtectorDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static bool RFIDDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static bool AntiDropDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static bool AntiDroptrigger_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static bool HardWareDriverDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static bool AntiCollisionDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static bool AntiCollisionDATAtrigger_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static bool BatteryDATA_callback(pb_ostream_t* stream, const pb_field_t* field, void* const* arg);
static void DoRxHeadMsg(const pb_field_t **Message_fields,void **pMessage);
static bool canbus_command_callback(pb_istream_t *stream, const pb_field_t *field, void **arg);
static bool canbus_status_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
static bool upgrade_package_callback(pb_istream_t *stream, const pb_field_t *field, void **arg);
uint8_t Dropconfigvalflg=0;
int MoveCheckCntx=0;
int MoveCheckCnt=0;
int MoveCheckFlg=0;
int MoveCheckSta=0;
int DataInitFlg=0;
int32_t mySetSpd=0;
uint32_t RfidConfigPowerVal=0;
int32_t mySetSpdLeft=0;
int32_t mySetSpdRight=0;
extern int32_t	mLeftSpeedx;
extern int32_t	mRightSpeedx;
void GetCurrentStateData(void);
void CheckMoveState(void);
void CheckMoveyawState(void);
void CheckMoveAngularState(void);
void CheckMoveAccelerationXState(void);
void CheckMoveAccelerationYState(void);
uint16_t CheckAntiDropFunction(uint16_t buf[],uint16_t num);
extern void PutValIntoBuf(unsigned char bufx[],int val);
//extern void SetSysUtc8(uint32_t f_utc8);
extern void ClearXdsVacuumOC(void);
/*******************************************************************************/
//内部函数声明
/*******************************************************************************/
extern uint8_t g_can_ota_buf[8];
extern uint8_t g_can_ota_buf_len;
extern int rfidmsFlg;
extern uint8_t g_using_disinfect_flag;
struct PROTECTOR_DATA g_protector_data = {0};
struct SHUTDOWN_CMD     ShutDown_CMD   = {0,0};
struct CONFIGCOMMAND_CMD  ConfigCommandCMD = {0};
struct DEVICECOMMAND_CMD  DeviceCommandCMD = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        .hub_motor_power_on = 1};
struct DEVICE_DATA      DeviceDATA     = {0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
struct OTHER_DEVICE_DATA  Other_DeviceData = {0};
struct HEALTH_DATA      HealthDATA     = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
struct MOTIONCONTROLMOTORCOMMAND_CMD  MotionControlMotorCommandCMD = {0,0};
struct MOTIONCONTROLMOTORCOMMAND_CMD  mcmCMDold = {0,0};
struct ULTRASONICDATA UltrasonicDATA = {0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF};
//struct RFID_DATA    RfidDATA = {0,0};
VERSION_DATA_Typedef  V_Product = {0,0,0,0};
VERSION_DATA_Typedef  V_Main = {0,0,0,0};
VERSION_DATA_Typedef  V_Gyro = {0,0,0,0};
VERSION_DATA_Typedef  V_Sonar = {0,0,0,0};
VERSION_DATA_Typedef  V_Drop = {0,0,0,0};
VERSION_DATA_Typedef  V_other = {0,0,0,0};
VERSION_DATA_Typedef  V_Gcub = {0,0,0,0};
VERSION_DATA_Typedef  V_Xd510 = {0,0,0,0};
VERSION_DATA_Typedef  V_H25A_S = {0,0,0,0};
VERSION_DATA_Typedef  V_H25A_H = {0,0,0,0};
VERSION_DATA_Typedef  V_ZLAC_S = {0,0,0,0};
ANTIDROP_DATA_Typedef AntiDropDATA = {0};

uint32_t MytestVal=0;
uint32_t PowerSwitchStateFlg=0;
uint32_t ShutDown_TEST=0;
//uint32_t HandChargeVotage=0;
uint32_t LaserAngleData=0;
uint32_t DeviceAngleData=0;
uint32_t BalanceIsReadyFlg=0;
uint32_t BalanceIsDropFlg=0;
//uint32_t BalancePowerLever=0;
uint32_t MotorSpeedClrFlg=0;
uint32_t McuErrCode=0,CurErrCode=0;
uint32_t LeftErrCode=0,RightErrCode=0;
uint32_t LeftErrClrFlg=0,RightErrClrFlg=0;

uint32_t MCUErrClrFlg6=0,MCUErrClrFlg7=0;
uint32_t MCUErrClrFlg0=0,MCUErrClrFlg1=0;

uint32_t MCUErrClrCnt6=0,MCUErrClrCnt7=0;
uint32_t MCUErrClrCnt0=0,MCUErrClrCnt1=0;

uint32_t RElayStatex=0;
uint32_t RElayStatey=0;
/******************************************************************************************************************************
*             Variable Define Section
*******************************************************************************************************************************/
//static uint16_t len = 0;
//static uint8_t sr_value = 0;  //定义为全局变量主要用来测试
uint8_t speed_counts = 0; //检测上位机与下位机的通讯
static uint8_t current_state = RX_HEAD_MSG;   //第一次进来将当前状态设为接收头
static uint8_t gs_motorCmdState = 0;//标明是否获取到电机命令
static uint8_t gs_hasGotCfgPara = 0;//标明是否获取到配置参数
uint32_t tcpic_connect_count = 0,w5500_reset_count = 0,w5500_reset_stepa_flg=0;
static uint16_t verify_num = 0xaa00;        //用作校验 
int16_t tvalx=0,tvaly=0,tvalz=0,tvalm;
static uint8_t gs_upgradeStatus = 0;
/******************************************************************************************************************************
*             Variable Define Section
*******************************************************************************************************************************/
extern uint16_t m6ePowerval;
extern uint16_t m6ePowerFlg;
extern uint8_t MyDebugBuf[20];
extern RfidTag_t g_rfid_tag_table[RFID_TAG_MAX_NUM];
uint32_t Reset_count  = 0; 
uint8_t Reset_status = 0 ;

uint8_t RFID_TagInx=0;
uint8_t RFID_TagNum=0;
uint8_t RFID_TagNumx=0;
uint8_t RFID_TagNumy=0;
uint8_t RFID_Tagbuf[20][2]={0};
uint8_t RFID_Tagbufx[20][2]={0};
uint8_t RFID_Tagbufy[20][4]={0};
uint16_t HardWareDriverisUsedBuf[16]={0};
uint16_t HardWareDriverConfigBuf[16]={0};
uint16_t HardWareDriverUpdateBuf[16]={0};
uint16_t HardWareDriverIsUpdated=0;
uint16_t Hinx=0;

uint8_t is_hardware_type_configed = 0;
uint8_t is_motor_driver_type_configed = 0;
void RfidDatHandle(void);
uint8_t nanopb_fill_ack_msg(uint32_t msg_id, uint8_t* buf, uint16_t* msg_len, const uint16_t buf_len)
{
	uint8_t status;
	pb_ostream_t ostream;
	const pb_field_t *Message_fields = NULL;
	void *pMessage = NULL;
	
	Message_fields = HeaderMessage_fields;  //用于接收到命令后返回header
	pMessage   =  &HderMsg;
	HderMsg.id = verify_num | msg_id;  //返回当前的cmd id 
//	current_state = RX_HEAD_MSG;
		
	memset(buf,0,buf_len);    //长度为128，由于传进来的buf是一个指针，所以sizeof(buf) = 4

	ostream = pb_ostream_from_buffer(buf, buf_len);
	status = pb_encode(&ostream, Message_fields, pMessage);
	*msg_len = ostream.bytes_written;
#if 0
printf("HderMsg.id:%d msg_len:%d buf_len:%d\r\n",HderMsg.id, *msg_len, buf_len);
for(int i = 0; i < *msg_len; i++)
	printf("%02X ", buf[i]);
printf("\r\n");
#endif
	HderMsg.id = 0;   //清0 Header结构
	if(!status)
	return 0;
	Reset_count = 0;
	return 1;
}

uint8_t nanopb_fill_canbus_status_ack_msg(uint32_t msg_id, uint8_t *buf, uint16_t *msg_len, const uint16_t buf_len)
{
    uint8_t status;
    pb_ostream_t ostream;
    const pb_field_t *Message_fields = NULL;
    void *pMessage = NULL;

    CanBus_DATA.id = verify_num | msg_id;

    current_state = RX_HEAD_MSG;
	CanBus_DATA.rtr = 0;
	CanBus_DATA.ide = 0;
	CanBus_DATA.data.funcs.encode = &canbus_status_callback;

    Message_fields = CANBusStatusData_fields;
    pMessage	=  &CanBus_DATA;
			
	memset(buf,0,buf_len);    //长度为128，由于传进来的buf是一个指针，所以sizeof(buf) = 4

	ostream = pb_ostream_from_buffer(buf, buf_len);
	status = pb_encode(&ostream, Message_fields, pMessage);
	*msg_len = ostream.bytes_written;
#if 0
printf("%s HderMsg.id:0x%X msg_len:%d buf_len:%d\r\n",__FUNCTION__,CanBus_DATA.id, ostream.bytes_written, buf_len);
for(int i = 0; i < ostream.bytes_written; i++)
	printf("%02X ", buf[i]);
printf("\r\n");
#endif
	HderMsg.id = 0;   //清0 Header结构
	if(!status)
	return 0;
	Reset_count = 0;
	return 1;
}
/***********************************************************************************************************************************
* Function Name       :  RunNanopb
* Create Date         :  2016/10/12
* Author/Corporation  :  YiPeng/Gaussian Robot
* Description         :  处理w5500接收到的数据；采用状态机的编程思想，真个过程分为接收头和接受命令数据(不是下位机要反馈给上位机的数据)  ;
*                        ，收到数据后，根据当前的状态进行解压缩，如果不是出于接收头状态，那么就，根据当前的状态对应的msg结构接收，
                          接收完之后，返回头，头中的id为当前msg的id；
                          如果处于接收头的状态，先用头msg结构接收，根据接收到的头msgid来判断，如果id是命令数据，则切换状态，并将id | 0xa0,
                          返回头，如果msgid是数据请求id，则将要压缩发送的数据指针指向对应msg结构，之后将下一个状态设置为接收头状态,
                          最终返回请求的数据，数据id为msgid | 0xa0.
* Parameter           :  RxMessage,info
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code      
************************************************************************************************************************************/
uint8_t RunNanopb(uint8_t buf[],uint16_t *msg_len,const uint16_t buf_len)
{
	uint8_t status;
	pb_ostream_t ostream;
	pb_istream_t istream;
	const pb_field_t *Message_fields = NULL;
	void *pMessage = NULL;
	if(msg_len == 0)
	{
		return 0 ;
	}
	switch(current_state)  
	{
	case RX_HEAD_MSG:
	Message_fields = HeaderMessage_fields;
	pMessage   =  &HderMsg;
	istream = pb_istream_from_buffer(buf, *msg_len);
	status = pb_decode(&istream, Message_fields, pMessage);
	DoRxHeadMsg(&Message_fields,&pMessage);    //这里要用到二级指针，否则在函数体内部赋值并不会改变实参本身的值

	break;
	case RX_VERIFY_MSG:
		Message_fields = VerifyMessage_fields;
		pMessage   =  &VerifyMsg;
		istream = pb_istream_from_buffer(buf, *msg_len);
		status = pb_decode(&istream, Message_fields, pMessage);
		VerifyMsg.id |= verify_num ;
		VerifyMsg.data = GenerateHash(VerifyMsg.data);
		memset(buf,0,buf_len);    //长度为128，由于传进来的buf是一个指针，所以sizeof(buf) = 4
		ostream = pb_ostream_from_buffer(buf, buf_len);
		status |= pb_encode(&ostream, Message_fields, pMessage);
		*msg_len = ostream.bytes_written;
		current_state = RX_HEAD_MSG;
		return status ? 1 : 0;
	case RX_SHUTDOWN_MSG:	  	
		Message_fields = ShutdownCommand_fields;
		pMessage   =  &ShutdownCmdMsg;
		istream = pb_istream_from_buffer(buf, *msg_len);
		status = pb_decode(&istream, Message_fields, pMessage);
		ShutDown_CMD.id = ShutdownCmdMsg.id;
		ShutDown_CMD.type = ShutdownCmdMsg.type;
		if(ShutDown_CMD.type == 1)
		{
			CtrShutDown(ShutDown_CMD.type);
		}
		Message_fields = HeaderMessage_fields;  //用于接收到命令后返回header
		pMessage   =  &HderMsg;
		if(ShutdownCmdMsg.id == SHUTDOWN_COMMAND)
		{
			HderMsg.id = verify_num | ShutdownCmdMsg.id;  //返回当前的cmd id 
		}
		else
		{
			HderMsg.id = verify_num | 0xff;
		}
		current_state = RX_HEAD_MSG;
        break;
        
    case RX_CONFIG_CMD:	  	

        Message_fields = ConfigCommand_fields;
        pMessage   =  &ConfigCmdMsg;
        istream = pb_istream_from_buffer(buf, *msg_len);
        ConfigCmdMsg.config.funcs.decode = &ConfigCMD_callback;
        status = pb_decode(&istream, Message_fields, pMessage);

        Message_fields = HeaderMessage_fields;  //用于接收到命令后返回header
        pMessage   =  &HderMsg;         

        if(ConfigCmdMsg.id == CONFIG_COMMAND)
        {
            HderMsg.id = verify_num | ConfigCmdMsg.id;  //返回当前的cmd id 

        }
        else
        {
            HderMsg.id = verify_num | 0xff;
        }

        current_state = RX_HEAD_MSG;
        SetHasGotCfgParaFlag(1);

        break;
        
    case RX_MCM_CMD:
        Message_fields = MotionControlMotorCommand_fields;
        pMessage   =  &MCMCmdMsg;
        istream = pb_istream_from_buffer(buf, *msg_len);
        MCMCmdMsg.command.funcs.decode = &McmCMD_callback;
        status = pb_decode(&istream, Message_fields, pMessage);

        Message_fields = HeaderMessage_fields;  //用于接收到命令后返回header
        pMessage   =  &HderMsg;
        if(MCMCmdMsg.id == MOTION_CONTROL_MOTOR_COMMAND)
        {
            HderMsg.id = verify_num | MCMCmdMsg.id;  //返回当前的cmd id 
        }
        else
        {
            HderMsg.id = verify_num | 0xff;
        }
        current_state = RX_HEAD_MSG;
        break;
        
    case RX_DEVICE_CMD:
        Message_fields = DeviceCommand_fields;
        pMessage   =  &DeviceCmdMsg;
        istream = pb_istream_from_buffer(buf, *msg_len);
        DeviceCmdMsg.command.funcs.decode =  &DeviceCMD_callback;     //函数回调
        status = pb_decode(&istream, Message_fields, pMessage);

        Message_fields = HeaderMessage_fields;  //用于接收到命令后返回header
        pMessage   =  &HderMsg;
        if(DeviceCmdMsg.id == DEVICE_COMMAND)
        {
            HderMsg.id = verify_num | DeviceCmdMsg.id;  //返回当前的cmd id 

        }
        else
        {
            HderMsg.id = verify_num | 0xff;
        }
        current_state = RX_HEAD_MSG;
        break;
        
    case RX_MMD_CMD:
        Message_fields = ManualModeDeviceCommand_fields;
        pMessage   =  &MMDCmdMsg;
        istream = pb_istream_from_buffer(buf, *msg_len);
        MMDCmdMsg.command.funcs.decode = &MmdCMD_callback;
        status = pb_decode(&istream, Message_fields, pMessage);

        Message_fields = HeaderMessage_fields;  //用于接收到命令后返回header
        pMessage   =  &HderMsg;
        if(MMDCmdMsg.id == MANUAL_MODE_DEVICE_COMMAND)
        {
            HderMsg.id = verify_num | MMDCmdMsg.id;  //返回当前的cmd id 
        }
        else
        {
            HderMsg.id = verify_num | 0xff;
        }
        current_state = RX_HEAD_MSG;
        break;
        
    case RX_REMOTE_CMD433:
        Message_fields = RemoteCommand433_fields;
        pMessage   =  &RemoteCmd433Msg;
        istream = pb_istream_from_buffer(buf, *msg_len);
        status = pb_decode(&istream, Message_fields, pMessage);

        Message_fields = HeaderMessage_fields;  //用于接收到命令后返回header
        pMessage   =  &HderMsg;
        if(RemoteCmd433Msg.id == REMOTE_COMMAND_433)
        {
            HderMsg.id = verify_num | RemoteCmd433Msg.id;  //返回当前的cmd id 
        }
        else
        {
            HderMsg.id = verify_num | 0xff;
        }
        current_state = RX_HEAD_MSG;
        break;
        
    case RX_CLEAR_ERROR_DATA:	

		Message_fields = ClearErrorCommand_fields;
		pMessage   =  &ClearErrorCmdMsg;
		istream = pb_istream_from_buffer(buf, *msg_len);
		ClearErrorCmdMsg.bits.funcs.decode = &ClearErrCMD_callback;
		status = pb_decode(&istream, Message_fields, pMessage);

		Message_fields = HeaderMessage_fields;  //用于接收到命令后返回header
		pMessage   =  &HderMsg;         

		if(ClearErrorCmdMsg.id == CLEAR_ERROR_COMMAND)
		{
			HderMsg.id = verify_num | ClearErrorCmdMsg.id;  //返回当前的cmd id 
		}
		else
		{
			HderMsg.id = verify_num | 0xff;
		}

		current_state = RX_HEAD_MSG;
		break;
        
    case RX_DRIVER_CONFIG_DATA:

		Message_fields = MotorDriverConfigCommand_fields;
		pMessage   =  &DriverConfigDataMsg;
		istream = pb_istream_from_buffer(buf, *msg_len);
		DriverConfigDataMsg.config.funcs.decode = &DriverConfig_callback;
		status = pb_decode(&istream, Message_fields, pMessage);

		Message_fields = HeaderMessage_fields;  //用于接收到命令后返回header
		pMessage   =  &HderMsg;         

		if(DriverConfigDataMsg.id == DRIVER_CONFIG_COMMOD)
		{
			HderMsg.id = verify_num | DriverConfigDataMsg.id;  //返回当前的cmd id 
		}
		else
		{
			HderMsg.id = verify_num | 0xff;
		}

		current_state = RX_HEAD_MSG;
		break;
	
	
	

	case RX_MCU_UPDATE_CMD://升级指令
		Message_fields = MCUUpgradeCommand_fields;
		pMessage   =  &MCU_Update_CMD;
		istream = pb_istream_from_buffer(buf, *msg_len);
		
		MCU_Update_CMD.command.funcs.decode = MCU_update_callback;
		status = pb_decode(&istream, Message_fields, pMessage);
		
		Message_fields = HeaderMessage_fields;  //用于接收到命令后返回header
		pMessage   =  &HderMsg;
		if(MCU_Update_CMD.id == MCU_UPGRADE_COMMOD)
		{
			HderMsg.id = verify_num | MCU_Update_CMD.id;  //返回当前的cmd id 
		}
		else
		{
			HderMsg.id = verify_num | 0xff;
		}
		current_state = RX_HEAD_MSG;		
		break;
		case RX_PHANTAS_MCU_UPGRADE_COMMAND://升级指令
		Message_fields = PhantasMcuUpgradeCommand_fields;
		pMessage   =  &PhantasMcuUpgrade_CMD;
		istream = pb_istream_from_buffer(buf, *msg_len);
		status = pb_decode(&istream, Message_fields, pMessage);

		post_to_ota_task(RX_PHANTAS_MCU_UPGRADE_COMMAND, buf, msg_len, buf_len);
		status = 0; /* ota消息由ota升级任务回消息给上位机，这里处理完上位机数据后直接返回 */
		current_state = RX_HEAD_MSG;
		break;
		case RX_PHANTAS_MCU_UPGRADE_PACKAGE://升级数据包
		Message_fields = PhantasMcuUpgradePackage_fields;
		pMessage   =  &PhantasMcuUpgradePackage_CMD;
		istream = pb_istream_from_buffer(buf, *msg_len);
		PhantasMcuUpgradePackage_CMD.data.funcs.decode = upgrade_package_callback;
		status = pb_decode(&istream, Message_fields, pMessage);
	
		post_to_ota_task(RX_PHANTAS_MCU_UPGRADE_PACKAGE, buf, msg_len, buf_len);
		status = 0; /* ota消息由ota升级任务回消息给上位机，这里处理完上位机数据后直接返回 */
		current_state = RX_HEAD_MSG;
		HderMsg.id = 0;   //清0 Header结构
		if(!status)
		return 0;
		return 1;
	break;
	
	case RX_CANBUS_COMMAND://驱控器升级指令
//		printf("222\r\n");
		Message_fields = CANBusCommand_fields;
		pMessage   =  &CanBus_CMD;
		istream = pb_istream_from_buffer(buf, *msg_len);
		CanBus_CMD.data.funcs.decode = canbus_command_callback;
		status = pb_decode(&istream, Message_fields, pMessage);
        app_ota_type = 0;
		post_to_ota_task(RX_CANBUS_COMMAND, buf, msg_len, buf_len);
		status = 0; /* ota消息由ota升级任务回消息给上位机，这里处理完上位机数据后直接返回 */
		current_state = RX_HEAD_MSG;
		HderMsg.id = 0;   //清0 Header结构
		if(!status)
		return 0;
		return 1;
		break;		
    case RX_CANBUS2_COMMAND: // bms
        Message_fields = CANBusCommand_fields;
        pMessage = &CanBus_CMD;
        istream = pb_istream_from_buffer(buf, *msg_len);
        CanBus_CMD.data.funcs.decode = canbus_command_callback;
        status = pb_decode(&istream, Message_fields, pMessage);

        app_ota_type = 1;
        post_to_ota_task(RX_CANBUS2_COMMAND, buf, msg_len, buf_len);
        status = 0; /* ota消息由ota升级任务回消息给上位机，这里处理完上位机数据后直接返回 */
        current_state = RX_HEAD_MSG;
        HderMsg.id = 0; // 清0 Header结构
        if (!status)
            return 0;
        return 1;
        break;
    case CANBUS2_STATUS://can2升级指令
        Message_fields = CANBusCommand_fields;
        pMessage = &CanBus_CMD;
        istream = pb_istream_from_buffer(buf, *msg_len);
        status = pb_decode(&istream, Message_fields, pMessage);
        app_ota_type = 1;
        set_recv_can_cmd_tick(HAL_GetTick());
        status = 0;
        current_state = RX_HEAD_MSG;
        HderMsg.id = 0; // 清0 Header结构
        if (!status)
            return 0;
        return 1;
		break;
	default :   
	current_state = RX_HEAD_MSG;
	break;
	}   	
/* ota消息处理返回失败，不回消息给上位机，由ota升级任务回消息给上位机 */
    if (!status)
		return false;
	if ((CANBUS_STATUS == HderMsg.id) || ( CANBUS2_STATUS == HderMsg.id))
	{
//		printf("CANBUS_STATUS has not recv ack\r\n");
		return false;
	}
	memset(buf,0,buf_len);    //长度为128，由于传进来的buf是一个指针，所以sizeof(buf) = 4

	ostream = pb_ostream_from_buffer(buf, buf_len);
	status = pb_encode(&ostream, Message_fields, pMessage);
	*msg_len = ostream.bytes_written;	

	HderMsg.id = 0;   //清0 Header结构
	if(!status)
	return 0;
	Reset_count = 0;
	return 1;
}

static void DoRxHeadMsg(const pb_field_t **Message_fields,void **pMessage)
{	
   switch(HderMsg.id)
    {
        case VERIFY_MESSAGE:
            HderMsg.id |= verify_num;
            current_state = RX_VERIFY_MSG;   //用于调试发送头命令
            break;
        
        case DEVICE_COMMAND:
            HderMsg.id |= verify_num;
            current_state = RX_DEVICE_CMD;		  
            break;
        
        case SHUTDOWN_COMMAND:
            HderMsg.id |= verify_num;
            current_state = RX_SHUTDOWN_MSG;
            break;
        
        case CONFIG_COMMAND:
            HderMsg.id |= verify_num;
            current_state = RX_CONFIG_CMD;	   
            break;
        
        case MOTION_CONTROL_MOTOR_COMMAND:
            HderMsg.id |= verify_num;
            current_state = RX_MCM_CMD;
            break;
        
        case MANUAL_MODE_DEVICE_COMMAND:
            HderMsg.id |= verify_num;
            current_state = RX_MMD_CMD;
            break;
        
        case REMOTE_COMMAND_433:
            HderMsg.id |= verify_num;
            current_state = RX_REMOTE_CMD433;
            break;
        
        case VERSION_DATA:
            VersionDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            VersionDataMsg.version.funcs.encode = & VersionDATA_callback;
            *Message_fields = VersionData_fields;
            *pMessage   =  &VersionDataMsg; 
            break; 
        
        case HEALTH_DATA:
            HealthDataMsg.id = verify_num| HderMsg.id;
            current_state = RX_HEAD_MSG;
            HealthDataMsg.data.funcs.encode = & HealthDATA_callback;
            *Message_fields = HealthData_fields;
            *pMessage = &HealthDataMsg;
            break;
        
        case DEVICE_DATA:
            DeviceDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            DeviceDataMsg.data.funcs.encode = & DeviceDATA_callback;
            *Message_fields = DeviceData_fields;
            *pMessage   =  &DeviceDataMsg;
            break;
        
        case ODOMETRY_DATA:
            OdometryDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            * Message_fields = OdometryData_fields;
            * pMessage   =  &OdometryDataMsg;		  
            break;
        
        case IMU_DATA:
            IMUDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            * Message_fields = IMUData_fields;
            * pMessage   =  &IMUDataMsg;
            break;
        
        case ULTRASONIC_DATA:
            UltrasonicDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            UltrasonicDataMsg.data.funcs.encode = & UltraDATA_callback;
            *Message_fields = UltrasonicData_fields;
            *pMessage   =  &UltrasonicDataMsg;
            break;

        case PROTECTOR_DATA:
            ProtectorDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            ProtectorDataMsg.data.funcs.encode = & ProtectorDATA_callback;
            *Message_fields = ProtectorData_fields;
            *pMessage   =  &ProtectorDataMsg; 
            break;
        
        case ANTIDROP_DATA:
            AntiDropDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            AntiDropDataMsg.data.funcs.encode = & AntiDropDATA_callback;
            AntiDropDataMsg.trigger_data.funcs.encode = & AntiDroptrigger_callback;	
            *Message_fields = AntiDropData_fields;
            *pMessage = & AntiDropDataMsg;
            break;
        
        case UWB_DATA:
            UWBDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            * Message_fields = UWBData_fields;
            * pMessage   =  &UWBDataMsg;
            break;

        case RFID_DATA:
            RFIDDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            #ifndef RFID_TWOTAG_TEST
            RFIDDataMsg.data.funcs.encode = & RFIDDATA_callback;  
            #else
            RFIDDataMsg.data.funcs.encode = & RFIDDATA_callback_test;  
            #endif
            *Message_fields = RFIDData_fields;
            *pMessage   =  &RFIDDataMsg;
            break;

        case REMOTE_DATA_433:
            RemoteData433Msg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            * Message_fields = RemoteData433_fields;
            * pMessage   =  &RemoteData433Msg;
            break;

        case CLEAR_ERROR_COMMAND: 
            HderMsg.id |= verify_num;
            current_state = RX_CLEAR_ERROR_DATA;		 
            break;

        case ANALOG_ANTI_COLLISION_DATA:
            AntiCollisionDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            AntiCollisionDataMsg.data.funcs.encode = & AntiCollisionDATA_callback;
            AntiCollisionDataMsg.trigger_data.funcs.encode = & AntiCollisionDATAtrigger_callback;	
            *Message_fields = AnalogAntiCollisionData_fields;
            *pMessage = & AntiCollisionDataMsg;
            break;	
            
        case DRIVER_CONFIG_COMMOD:	
            HderMsg.id |= verify_num;
            current_state = RX_DRIVER_CONFIG_DATA;	
            break;

        case HARDWARE_DRIVER_DATA:	
            HardWareDriverDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            HardWareDriverDataMsg.data.funcs.encode = & HardWareDriverDATA_callback;
            *Message_fields = HardwareDriverData_fields;
            *pMessage = & HardWareDriverDataMsg;	
            break;	
        
        case SABER_IMU_DATA:
            IMUDataTestMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            * Message_fields = SaberIMUData_fields;
            * pMessage   =  &IMUDataTestMsg;
            break;

        case BATTERY_DATA:
            BatteryDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            BatteryDataMsg.data.funcs.encode = &BatteryDATA_callback;
            *Message_fields = BatteryData_fields;
            *pMessage = &BatteryDataMsg;
            break;
        
        case MOTOR_DRIVER_CONFIG_DATA:
            MotorDriverConfigDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            MotorDriverConfigDataMsg.config.funcs.encode = &MotorDriverConfigData_callback;
            *Message_fields = MotorDriverConfigData_fields;
            *pMessage = &MotorDriverConfigDataMsg;
            break;
        
        case OTHER_DEVICE_DATA:
            OtherDeviceDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            OtherDeviceDataMsg.data.funcs.encode = &OtherDeviceData_callback;
            *Message_fields = OtherDeviceData_fields;
            *pMessage = &OtherDeviceDataMsg;
            break;
        case DRIVER_DATA:
            DriverDataMsg.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            DriverDataMsg.data.funcs.encode = &DriverData_callback;
            *Message_fields = DriverData_fields;
            *pMessage = &DriverDataMsg;
            break;

        case MCU_UPGRADE_COMMOD:
            HderMsg.id |= verify_num;
            current_state = RX_MCU_UPDATE_CMD;
            break;

        case MCU_UPGRADE_STATUS:
            MCU_Update_DATA.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            MCU_Update_DATA.status = GetUpgradeDataRecState() ;
            SetUpgradeDataRecState(0);

            * Message_fields = MCUUpgradeStatusData_fields;
            * pMessage	=  &MCU_Update_DATA;
            break;
		case CANBUS_COMMAND:
//			printf("111\r\n");
            HderMsg.id |= verify_num;
            current_state = RX_CANBUS_COMMAND;
            break;

        case CANBUS_STATUS:
//			printf("444\r\n");
			post_canbus_to_ota_task(CANBUS_STATUS);
            break;
			
        case PHANTAS_MCU_UPGRADE_COMMAND:
            HderMsg.id |= verify_num;
            current_state = RX_PHANTAS_MCU_UPGRADE_COMMAND;
            break;

//        case PHANTAS_MCU_UPGRADE_COMMAND_STATUS:
//            PhantasMcuUpgrade_DATA.id = verify_num | HderMsg.id;
//            current_state = RX_HEAD_MSG;
//            PhantasMcuUpgrade_DATA.status = get_phantas_mcu_upgrade_command_status();
//            set_phantas_mcu_upgrade_command_status(OTA_FAILED);

//            * Message_fields = PhantasMcuUpgradeCommandStatusData_fields;
//            * pMessage	=  &PhantasMcuUpgrade_DATA;
//            break;

        case PHANTAS_MCU_UPGRADE_PACKAGE:
            HderMsg.id |= verify_num;
            current_state = RX_PHANTAS_MCU_UPGRADE_PACKAGE;
            break;

//        case PHANTAS_MCU_UPGRADE_PACKAGE_STATUS:
//            PhantasMcuUpgradePackage_DATA.id = verify_num | HderMsg.id;
//            current_state = RX_HEAD_MSG;
//            PhantasMcuUpgradePackage_DATA.status = get_phantas_mcu_upgrade_package_status() ;
//            set_phantas_mcu_upgrade_package_status(OTA_FAILED);

//            * Message_fields = PhantasMcuUpgradePackageStatusData_fields;
//            * pMessage	=  &PhantasMcuUpgradePackage_DATA;
//            break;
			
        case PHANTAS_MCU_TYPE_DATA:
            PhantasMcuType_DATA.id = verify_num | HderMsg.id;
            current_state = RX_HEAD_MSG;
            PhantasMcuType_DATA.mcu_types.funcs.encode = &mcu_types_callback;

            * Message_fields = PhantasMcuTypeData_fields;
            * pMessage	=  &PhantasMcuType_DATA;
            break;
        case CANBUS2_COMMAND:
            HderMsg.id |= verify_num;
            current_state = RX_CANBUS2_COMMAND;
            break;
        case CANBUS2_STATUS:
            post_canbus_to_ota_task(CANBUS2_STATUS);
            break;
        default:
        break;
     } 
}

void RfidDatHandle(void)
{
	
}
void CheckMoveState(void)
{
	CheckMoveyawState();
	CheckMoveAngularState();
	CheckMoveAccelerationXState();
	CheckMoveAccelerationYState();
}

void CheckMoveyawState(void)
{
	
	
}

void CheckMoveAngularState(void)
{	
	
}

void CheckMoveAccelerationXState(void)
{	

	
}
void CheckMoveAccelerationYState(void)
{	

	
}
uint16_t CheckAntiDropFunction(uint16_t buf[],uint16_t num)
{
	uint16_t val,i;

	val=0;
	for(i=0;i<num;i++)
	{
		if(buf[i]!=0)
		{
			val=1;
			// �����书������
			break;
		}
	}
	return val;

}
void GetCurrentStateData(void)
{
	
}

/****************************************************************************************************************************
* Function Name       :  ConfigCMD_callback
* Create Date         :  2016/12/14
* Author/Corporation  :  ChengPengfei/Gaussian Robot
* Description         :  处理输入数据的repeated,对每一个key进行解码，获取相应的value
*
* Parameter           :  None
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.12.14   ChengPengfei               first code
*****************************************************************************************************************************/
static bool ConfigCMD_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
  
  UInt32Pair  uint32pair = {0,0};
  
    if (!pb_decode(stream, UInt32Pair_fields, &uint32pair))
        return false;	
 	
  switch(uint32pair.key)
  {
    case CONFIG_COMMAND_DEVICE_ID :
      ConfigCommandCMD.Device_Id = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_SPEED_LEVEL :
      ConfigCommandCMD.Speed_Level = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_BUMPER_FRONT :
      ConfigCommandCMD.Bumper_Front = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_BUMPER_LEFT :
      ConfigCommandCMD.Bumper_Left = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_BUMPER_RIGHT :
      ConfigCommandCMD.Bumper_Right = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_BUMPER_REAR :
      ConfigCommandCMD.Bumper_Rear = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_BUMPER_LEFT_FRONT :
      ConfigCommandCMD.Bumper_Left_Front = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_BUMPER_RIGHT_FRONT :
      ConfigCommandCMD.Bumper_Right_Front = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_BUMPER_LEFT_REAR :
      ConfigCommandCMD.Bumper_Left_Rear = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_BUMPER_RIGHT_REAR :
      ConfigCommandCMD.Bumper_Right_Rear = uint32pair.value;
      break;
	  
	case CONFIG_COMMAND_BUMPER_ENABLE :
      ConfigCommandCMD.Bumper_Enable = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_ULTRASONIC_FRONT :
      ConfigCommandCMD.Ultrasonic_Front = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_ULTRASONIC_LEFT :
      ConfigCommandCMD.Ultrasonic_Left = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_ULTRASONIC_RIGHT :
      ConfigCommandCMD.Ultrasonic_Right = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_ULTRASONIC_REAR :
      ConfigCommandCMD.Ultrasonic_Rear = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_ULTRASONIC_LEFT_FRONT :
      ConfigCommandCMD.Ultrasonic_Left_Front = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_ULTRASONIC_RIGHT_FRONT :
      ConfigCommandCMD.Ultrasonic_Right_Front = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_ULTRASONIC_LEFT_REAR :
      ConfigCommandCMD.Ultrasonic_Left_Rear = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_ULTRASONIC_RIGHT_REAR :
      ConfigCommandCMD.Ultrasonic_Right_Rear = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_ULTRASONIC_ENABLE :
      ConfigCommandCMD.Ultrasonic_Enable = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_CHARGE_FULL_VOLTAGE_ADC :
      ConfigCommandCMD.Charge_Full_Voltage = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_CHARGE_FULL_CURRENT_ADC :
      ConfigCommandCMD.Charge_Full_Current = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_CHARGE_SHORT_CIRCUIT_CURRENT_ADC :
      ConfigCommandCMD.Charge_Short_Current = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_CHARGE_TOUCHED_VOLTAGE_ADC_LOWER_BOUNDS :
      ConfigCommandCMD.Charge_Touched_Voltage_Low = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_CHARGE_TOUCHED_VOLTAGE_ADC_UPPER_BOUNDS :
      ConfigCommandCMD.Charge_Touched_Voltage_High = uint32pair.value;
      break;
    
    
    case CONFIG_COMMAND_ADDITIONAL_BRAKER :
      ConfigCommandCMD.Additional_Braker = uint32pair.value;
      break;
    
    case CONFIG_COMMAND_FILTER_MOTOR :
      ConfigCommandCMD.Filter_Motor = uint32pair.value;
			break;
    
    case CONFIG_COMMAND_ANTI_DROP_FRONT :
	
      ConfigCommandCMD.Anti_Drop_Front = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANTI_DROP_LEFT :
      ConfigCommandCMD.Anti_Drop_Left = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANTI_DROP_RIGHT :
      ConfigCommandCMD.Anti_Drop_Right = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANTI_DROP_REAR :
      ConfigCommandCMD.Anti_Drop_Rear = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANTI_DROP_LEFT_FRONT :
      ConfigCommandCMD.Anti_Drop_Left_Front = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANTI_DROP_RIGHT_FRONT :
      ConfigCommandCMD.Anti_Drop_Right_Front = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANTI_DROP_LEFT_REAR :
      ConfigCommandCMD.Anti_Drop_Left_Rear = uint32pair.value;
			break;
    case CONFIG_COMMAND_ANTI_DROP_RIGHT_REAR :
      ConfigCommandCMD.Anti_Drop_Right_Rear = uint32pair.value;  
		        break;	
	case CONFIG_COMMAND_BATTERY_EMPTY_VOLTAGE_ADC :
		ConfigCommandCMD.Battery_Empty_Voltage_adc= uint32pair.value;  
		break;
	case CONFIG_COMMAND_CHARGE_EMPTY_VOLTAGE_ADC :
		ConfigCommandCMD.Charge_Empty_Voltage_adc= uint32pair.value;  
		break;
	case CONFIG_COMMAND_CHARGE_EMPTY_CURRENT_ADC :
		ConfigCommandCMD.Charge_Empty_Current_adc= uint32pair.value;  
		break;
	case CONFIG_COMMAND_MAX_CHARGE_TIME :
		ConfigCommandCMD.Max_Charge_Time= uint32pair.value;  
		break;

	case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_FRONT :
      ConfigCommandCMD.Anti_Collision_Front = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_LEFT :
      ConfigCommandCMD.Anti_Collision_Left = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_RIGHT :
      ConfigCommandCMD.Anti_Collision_Right = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_REAR :
      ConfigCommandCMD.Anti_Collision_Rear = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_LEFT_FRONT :
      ConfigCommandCMD.Anti_Collision_Left_Front = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_RIGHT_FRONT :
      ConfigCommandCMD.Anti_Collision_Right_Front = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_LEFT_REAR :
      ConfigCommandCMD.Anti_Collision_Left_Rear = uint32pair.value;
			break;
    case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_RIGHT_REAR :
      ConfigCommandCMD.Anti_Collision_Right_Rear = uint32pair.value;  
		        break;
    
    case CONFIG_COMMAND_ANTI_DROP_THRESHOLD + 0 :
        ConfigCommandCMD.Anti_Drop_Threshold[0] = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_ANTI_DROP_THRESHOLD + 1 :
        ConfigCommandCMD.Anti_Drop_Threshold[1] = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_ANTI_DROP_THRESHOLD + 2 :
        ConfigCommandCMD.Anti_Drop_Threshold[2] = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_ANTI_DROP_THRESHOLD + 3 :
        ConfigCommandCMD.Anti_Drop_Threshold[3] = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_ANTI_DROP_THRESHOLD + 4 :
        ConfigCommandCMD.Anti_Drop_Threshold[4] = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_ANTI_DROP_THRESHOLD + 5 :
        ConfigCommandCMD.Anti_Drop_Threshold[5] = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_ANTI_DROP_THRESHOLD + 6 :
        ConfigCommandCMD.Anti_Drop_Threshold[6] = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_ANTI_DROP_THRESHOLD + 7 :
        ConfigCommandCMD.Anti_Drop_Threshold[7] = uint32pair.value;
        break;
    
	case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_THRESHOLD :
      ConfigCommandCMD.Anti_Collision_Threshold[0] = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_THRESHOLD + 1 :
      ConfigCommandCMD.Anti_Collision_Threshold[1] = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_THRESHOLD + 2 :
      ConfigCommandCMD.Anti_Collision_Threshold[2] = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_THRESHOLD + 3 :
      ConfigCommandCMD.Anti_Collision_Threshold[3] = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_THRESHOLD + 4 :
      ConfigCommandCMD.Anti_Collision_Threshold[4] = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_THRESHOLD + 5 :
      ConfigCommandCMD.Anti_Collision_Threshold[5] = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_THRESHOLD + 6 :
      ConfigCommandCMD.Anti_Collision_Threshold[6] = uint32pair.value;
			break;
		
    case CONFIG_COMMAND_ANALOG_ANTI_COLLISION_THRESHOLD + 7 :
      ConfigCommandCMD.Anti_Collision_Threshold[7] = uint32pair.value;
			break;
    
    case CONFIG_COMMAND_PROTECTOR_ANALOG_DELTA_T + 0 :
        ConfigCommandCMD.anti_collision_period_time[0] = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_PROTECTOR_ANALOG_DELTA_T + 1 :
        ConfigCommandCMD.anti_collision_period_time[1] = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_PROTECTOR_ANALOG_DELTA_T + 2 :
        ConfigCommandCMD.anti_collision_period_time[2] = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_PROTECTOR_ANALOG_DELTA_T + 3 :
        ConfigCommandCMD.anti_collision_period_time[3] = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_PROTECTOR_ANALOG_DELTA_T + 4 :
        ConfigCommandCMD.anti_collision_period_time[4] = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_PROTECTOR_ANALOG_DELTA_T + 5 :
        ConfigCommandCMD.anti_collision_period_time[5] = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_PROTECTOR_ANALOG_DELTA_T + 6 :
        ConfigCommandCMD.anti_collision_period_time[6] = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_PROTECTOR_ANALOG_DELTA_T + 7 :
        ConfigCommandCMD.anti_collision_period_time[7] = uint32pair.value;
        break;

	case CONFIG_COMMAND_RFID_POWER:
		ConfigCommandCMD.rfidPower = uint32pair.value;
			break;
	case 68:
		ConfigCommandCMD.squeegee_lift_operate_time = uint32pair.value;
		break;
		
	case CONFIG_COMMAND_LEFT_MOTOR_MAX_TEMPERATURE:
		ConfigCommandCMD.left_motor_max_temperature = uint32pair.value;
		break;
	
	case CONFIG_COMMAND_RIGHT_MOTOR_MAX_TEMPERATURE:
		ConfigCommandCMD.right_motor_max_temperature = uint32pair.value;
		break;
	
	case CONFIG_COMMAND_XD510_MAX_TEMPERATURE:
		ConfigCommandCMD.xd510_max_temperatrue = uint32pair.value;
		break;
	
	case CONFIG_COMMAND_BRUSH_MOTOR_CURRENT_MAX:
		ConfigCommandCMD.brush_motor_max_current = uint32pair.value;
		break;
	
	case CONFIG_COMMAND_SPRAY_MOTOR_CURRENT_MAX:
		ConfigCommandCMD.spray_motor_max_current = uint32pair.value;
		break;
	
	case CONFIG_COMMAND_FILTER_MOTOR_CURRENT_MAX:
		ConfigCommandCMD.fliter_motor_max_current = uint32pair.value;
		break;
	
	case CONFIG_COMMAND_LEFT_MOTOR_CURRENT_MAX:
		ConfigCommandCMD.left_motor_max_current = uint32pair.value;
		break;
	
	case CONFIG_COMMAND_RIGHT_MOTOR_CURRENT_MAX:
		ConfigCommandCMD.right_motor_max_current = uint32pair.value;
		break;
	
	case CONFIG_COMMAND_BRUSH_LIFT_MOTOR_CURRENT_MAX:
		ConfigCommandCMD.brush_lift_motor_max_current = uint32pair.value;
		break;
	
	case CONFIG_COMMAND_SQUEEGEE_LIFT_MOTOR_CURRENT_MAX:   //85
		ConfigCommandCMD.squeegee_lift_motor_max_current = uint32pair.value;
		break;
	
	case CONFIG_COMMAND_VACUUM_MOTOR_CURRENT_MAX:
		ConfigCommandCMD.vacuum_motor_max_current = uint32pair.value;
		break;
	
	case CONFIG_COMMAND_MOTOR_DRIVER_MAX_CURRENT_CHARGING:
		ConfigCommandCMD.run_motor_max_current_in_charge = uint32pair.value;
		break;
	case CONFIG_COMMAND_BMS_MAX_DISCONNECT_TIME:
        ConfigCommandCMD.bms_max_disconnect_time = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_BMS_RECOVER_TIME:
        ConfigCommandCMD.bms_recover_time = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_MOTOR_DRIVER_TYPE:
        ConfigCommandCMD.motor_driver_type = uint32pair.value;
        is_motor_driver_type_configed = 1;
        break;
    
    case CONFIG_COMMAND_HULL_LIFT_TRAVEL_LOW:
        ConfigCommandCMD.hull_lift_travel_low = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_HULL_LIFT_TRAVEL_MDDIAN:
        ConfigCommandCMD.hull_lift_travel_median = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_HULL_LIFT_TRAVEL_HIGH:
        ConfigCommandCMD.hull_lift_travel_high = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_SIDE_LIFT_TRAVEL_LOW:
        ConfigCommandCMD.side_lift_travel_low = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_SIDE_LIFT_TRAVEL_MDDIAN:
        ConfigCommandCMD.side_lift_travel_median = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_SIDE_LIFT_TRAVEL_HIGH:
        ConfigCommandCMD.side_lift_travel_high = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_WHEEL_TYPE:
        ConfigCommandCMD.wheel_type = uint32pair.value&0x01;
        break;
    
    case CONFIG_COMMAND_VELOCITY_1P:
        ConfigCommandCMD.velocity_1P = uint32pair.value?uint32pair.value:ConfigCommandCMD.velocity_1P;
        break;
    
    case CONFIG_COMMAND_VELOCITY_1I:
        ConfigCommandCMD.velocity_1I = uint32pair.value?uint32pair.value:ConfigCommandCMD.velocity_1I;
        break;
		
	case CONFIG_COMMAND_ANTICOLLISION_VALID_VOLTAGE:
        ConfigCommandCMD.anticollision_valid_voltage = uint32pair.value;
        break;
    
    case CONFIG_COMMAND_EMERGENCY_TYPE:
        ConfigCommandCMD.emergency_type = uint32pair.value&0x01;
        break;
    
    case CONFIG_COMMAND_EMERGENCY_STATUS_MIN_TIME:
        ConfigCommandCMD.emergency_status_min_time = uint32pair.value;
        break;
		
	case CONFIG_COMMAND_CONTROL_MODE:
        if( !GetCurCtrMode() ) //自动模式下不接受上位机模式配置命令
            ConfigCommandCMD.control_mode = uint32pair.value;
        break;
        
    case CONFIG_COMMAND_DISINFECT_ENABLE:
        ConfigCommandCMD.disinfect_enable = uint32pair.value;
        break;
	case 121:             //配置尘袋告警阈值默认2cm 单位0.1cm
        ConfigCommandCMD.dust_bag_error_distance = uint32pair.value;
        break;
	
	case 122:            //配置尘袋检测的告警电压，电压最低值默认0.5V 单位0.1V
        ConfigCommandCMD.dust_bag_error_vol = uint32pair.value;
        break;	
	
	case 123:             //1使用滚刷升降自适应 0不使用
        ConfigCommandCMD.self_adaption_enable  = uint32pair.value;  
        break;
	
   	case 124:            //自适应允许偏差百分比，最小百分之5
		if(uint32pair.value>100)
		{
			uint32pair.value=100;
		}
		if(uint32pair.value<5)
		{
			uint32pair.value=5;
		}
        ConfigCommandCMD.self_adaption_deviation = uint32pair.value;
        break; 	
		
	case 125:             //自适应超时滤波时间
		if(uint32pair.value>100000)
		{
			uint32pair.value=100000;
		}
		if(uint32pair.value<10)
		{
			uint32pair.value=10;
		}
        ConfigCommandCMD.self_adaption_filter_time  = uint32pair.value;  
        break;			
		
	case 126:            //取消485模块
        ConfigCommandCMD.cancel_485 = uint32pair.value;
		ConfigCommandCMD.rx_cancel_485_flag=1;
        break;
	case 128:
		ConfigCommandCMD.side_motor_type = uint32pair.value;   //0 中大  1本末
		break;	
	case 129:            //滚刷推杆电机最大电流 单位ma
        ConfigCommandCMD.side_motor_max_current = uint32pair.value;
        break;	
	
	case 130:            //滚刷推杆电机允许过流时间 单位ms
        ConfigCommandCMD.side_motor_max_current_time = uint32pair.value;
        break;
		
	case 131:
		ConfigCommandCMD.hardware_type = uint32pair.value;   //主驱动器类型 0:自研 1:xd510
        is_hardware_type_configed = 1;
		break;
    case 132:
		ConfigCommandCMD.board_type = uint32pair.value;       //1 PRO800 1.0单板  2 PRO800 2.0  
		break;	
	case 134:
		ConfigCommandCMD.side_lift_operate_time = uint32pair.value;   //边刷动作时间
		break;		
	case 135:
		ConfigCommandCMD.squeegee_lift_max_current_time = uint32pair.value;  
		break;
	case 136:
		if(uint32pair.value > 1000 && uint32pair.value <100000) //范围1s~100s
		{
			ConfigCommandCMD.dust_full_threshold = uint32pair.value;        
		}
		break;	
	case 137:
		ConfigCommandCMD.odom_bit_len = uint32pair.value;       //16 两字节 32 4字节
		break;
    case 138:
        ConfigCommandCMD.side_door_sensor_type = uint32pair.value;
        break;	  
		default :break;
    
  }  
    
    return true;
}
static bool ClearErrCMD_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
{

	UInt32Pair  uint32pair = {0,0};

	if (!pb_decode(stream, UInt32Pair_fields, &uint32pair))
	return false;		

	switch(uint32pair.key)
	{	
		case MCU_ERROR_CODE_0 :
			MCUErrClrFlg0 = uint32pair.value;
            ClearHollySysMotorAlarm();
            ClearXd510Overcurrent();
            ClearZdOverCurrent();
            ClearXdsVacuumOC();
			SetXdsCleanError();
			Hub_Motor_error_Clear();
			DeviceCommandCMD.clean_error=1;       //清除告警
			break;

		case MCU_ERROR_CODE_1 :
			MCUErrClrFlg1 = uint32pair.value;
			break;
			
		case MCU_ERROR_CODE_6 :
			MCUErrClrFlg6 = uint32pair.value;
			break;

		case MCU_ERROR_CODE_7 :
			MCUErrClrFlg7 = uint32pair.value;
			break;

		default :break;
		// return false; 
	}  

	return true;
}

static bool DriverConfig_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
{

	

	return true;
}

static bool MCU_update_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
	pb_byte_t byte[UPDATE_FIFO_SIZE] = {0};
	int ret = 0;
	int time = 0;
	gs_upgradeStatus = 1;
	if(!pb_read(stream, byte, MCU_Update_CMD.num))
	{
		gs_upgradeStatus = 2;
		return false;
	}
	if(MCU_Update_CMD.time != -1)
		printf("downloading...:%d\t%d\t%dBytes\r\n",MCU_Update_CMD.time,MCU_Update_CMD.num,MCU_Update_CMD.time*256+MCU_Update_CMD.num);
	ret = HandleUpgradeData(MCU_Update_CMD.time, MCU_Update_CMD.num, byte,  MCU_Update_CMD.check_code);
	
	if(ret != 0)
	{
		gs_upgradeStatus = 3;
		printf("***********downloade failed!***HandleUpgradeData() = %d***********\n",ret);
	}
	else
	{
		gs_upgradeStatus = 4;
		if(MCU_Update_CMD.time == -1 )
		{
			gs_upgradeStatus = 5;
			printf("***********downloade successed!***********\n");
		}
	}
		
	(void)gs_upgradeStatus;
    (void)time;
	
	if(ret == 0)
		SetUpgradeDataRecState(1);
	else
		SetUpgradeDataRecState(0);
	return true;
}
static bool MmdCMD_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
   UInt32Pair  uint32Pair = {0,0};
      
   if (!pb_decode(stream, UInt32Pair_fields, &uint32Pair))
        return false;
    switch(uint32Pair.key)
   {
//    case  DEVICE_COMMAND_DO :
//      DeviceCommandCMD.Do = uint32Pair.value;
//    break;
//    
//    case  DEVICE_COMMAND_RELAY :
//      DeviceCommandCMD.Relay = uint32Pair.value;
//		break;
//    
//    case  DEVICE_COMMAND_BRAKER_DOWN :
//      DeviceCommandCMD.Braker_Down = uint32Pair.value;
//    break;
//    
//    case  DEVICE_COMMAND_AUTO_MODE :
//      DeviceCommandCMD.Auto_Mode = uint32Pair.value;
//    break;
//    
//    case  DEVICE_COMMAND_TRACK_MOTOR :
//      DeviceCommandCMD.Track_Motor = uint32Pair.value;
//    break;
//    
//    case  DEVICE_COMMAND_STEERING_MOTOR :
//      DeviceCommandCMD.Steering_Motor = uint32Pair.value;
//    break;
//    
//    case  DEVICE_COMMAND_BRUSH_DOWN :
//      DeviceCommandCMD.Brush_Down = uint32Pair.value;
//    break;
//    
//    case  DEVICE_COMMAND_BRUSH_MOTOR :
//      DeviceCommandCMD.Brush_Motor = uint32Pair.value;
//    break;
//    
//    case  DEVICE_COMMAND_LEFT_BRUSH_MOTOR :
//      DeviceCommandCMD.Left_Brush_Motor = uint32Pair.value;
//    break;
//    
//    case  DEVICE_COMMAND_RIGHT_BRUSH_MOTOR :
//      DeviceCommandCMD.Right_Brush_Motor = uint32Pair.value;
//    break;
//    
//    case  DEVICE_COMMAND_SQUEEGEE_DOWN :
//      DeviceCommandCMD.Squeegee_Down = uint32Pair.value;
//    break;
//    
//    case  DEVICE_COMMAND_SQUEEGEE_MOTOR :
//      DeviceCommandCMD.Squeegee_Motor = uint32Pair.value;
//    break;
//    
//    case  DEVICE_COMMAND_SPRAY_MOTOR :
//      DeviceCommandCMD.Spray_Motor = uint32Pair.value;
//    break;
//    
//    case  DEVICE_COMMAND_VALVE :
//      DeviceCommandCMD.Valve = uint32Pair.value;
//    break;
//    
//    case  DEVICE_COMMAND_WATER_LEVEL :
//      DeviceCommandCMD.Water_Level = uint32Pair.value;
//    break;
//    
//    case  DEVICE_COMMAND_FAN_LEVEL :
//      DeviceCommandCMD.Fan_Levle = uint32Pair.value;
//    break;
//    
//    case DEVICE_COMMAND_BRUSH_SPIN_LEVEL:
//        DeviceCommandCMD.brush_spin_level = uint32Pair.value;
//        break;
//    
//    case DEVICE_COMMAND_BRUSH_PRESSURE_LEVEL:
//        DeviceCommandCMD.brush_pressure_level = uint32Pair.value;
//        break;
//    
//    case DEVICE_COMMAND_SPEED_LEVEL:
//        break;
//    
//    case DEVICE_COMMAND_DETERGENT_LEVEL:
//        break;
//    
//    case DEVICE_COMMAND_DETERGENT_MOTOR:
//        break;
//    
//    case DEVICE_COMMAND_LED_CMD:

//        break;
//    
//    case DEVICE_COMMAND_FILTRE_PWM:
//        DeviceCommandCMD.filter_motor_pwm = uint32Pair.value;
//        break;
//    
//	case DEVICE_COMMAND_OUTLET_VALVE:
//		DeviceCommandCMD.outlet_cmd = uint32Pair.value;
//		break;
//    
//	case DEVICE_COMMAND_WORK_STATUS:
//        DeviceCommandCMD.work_status = uint32Pair.value;
//        break;
//    
//	case DEVICE_COMMAND_HEART_STATUS: //充电桩心跳
//        break;
//    
//	case DEVICE_COMMAND_ORDER_CHARGER: //预约充电桩
//        break;	
//    
//	case DEVICE_COMMAND_MOTOR_MODE: //位置模式
//        DeviceCommandCMD.motor_mode = uint32Pair.value;
//        break;
//    
//	case DEVICE_COMMAND_CHECK_CHARGER: //充电桩查询命令
//        break;	
//    
//	case DEVICE_COMMAND_TIME: //上位机实时时间
//		DeviceCommandCMD.pc_current_time = uint32Pair.value + 28800;
//        break;
    
    default : break;

  }
    return true;
}
static bool upgrade_package_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
	pb_byte_t ota_data_buf[1024] = {0};
	
	if(!pb_read(stream, ota_data_buf, 1024))
	{
		return false;
	}
	copy_ota_package(ota_data_buf, 1024);
	
	return true;
}

static bool canbus_command_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
	uint8_t len = stream->bytes_left;
	pb_byte_t ota_data_buf[8] = {0};
	
	if(!pb_read(stream, ota_data_buf, len))
	{
		return false;
	}
	#if 0//OTA_DBG
	printf("CANBUS COMMAND: ");
	for (int i = 0; i < len; i++)
		printf("%02X ", ota_data_buf[i]);
	printf("\r\n");
	#endif
	copy_ota_package(ota_data_buf, len);
	
	return true;
}

bool canbus_status_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
	uint8_t* pData = g_can_ota_buf;
	uint8_t len = g_can_ota_buf_len;
	
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_string(stream, pData, len)))
		return false;
	memset(g_can_ota_buf, 0 , sizeof(g_can_ota_buf));
	memset(&CanBus_DATA, 0, sizeof(CanBus_DATA));
	return true;
}
static bool McmCMD_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
    MotionControlMotorCommand_MotorCommandPair  MCMC_MCmdPair = {0,0};
    
    if (!pb_decode(stream, MotionControlMotorCommand_MotorCommandPair_fields, &MCMC_MCmdPair))
        return false;
    
  
  switch(MCMC_MCmdPair.id)
  {
    case MotionControlMotorCommand_Low :
      MotionControlMotorCommandCMD.MotionControlMotorCommand_Low = MCMC_MCmdPair.command;
      SetMotorCmdState(1);
    break;
    
    case MotionControlMotorCommand_High :
      MotionControlMotorCommandCMD.MotionControlMotorCommand_High = MCMC_MCmdPair.command;
      SetMotorCmdState(1);
    break;
    case AKM_LINE_SPEED :
      MotionControlMotorCommandCMD.MotionControlMotorCommand_AKMLineSpeed = MCMC_MCmdPair.command;
      SetMotorCmdState(1);
    break;
    case AKM_ANGLE :
      MotionControlMotorCommandCMD.MotionControlMotorCommand_AKMAngle = MCMC_MCmdPair.command;
      SetMotorCmdState(1);
    break;
    case AKM_ANGLE_SPEED :
      MotionControlMotorCommandCMD.MotionControlMotorCommand_AKMAngleSpeed = MCMC_MCmdPair.command;
      SetMotorCmdState(1);
    break;
    default :break;
     // return false;
  }
    
    speed_counts = 0;
    return true;
}


static bool DeviceCMD_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
    UInt32Pair  uint32Pair = {0,0};
      
   if (!pb_decode(stream, UInt32Pair_fields, &uint32Pair))
        return false;
    switch(uint32Pair.key)
   {
    case  DEVICE_COMMAND_DO :
      DeviceCommandCMD.Do = uint32Pair.value;
    break;
    
    case  DEVICE_COMMAND_RELAY :
      DeviceCommandCMD.Relay = uint32Pair.value;
		break;
    
    case  DEVICE_COMMAND_BRAKER_DOWN :
      DeviceCommandCMD.Braker_Down = uint32Pair.value;
    break;
    
    case  DEVICE_COMMAND_AUTO_MODE :
      DeviceCommandCMD.Auto_Mode = uint32Pair.value;
    break;
    
//    case  DEVICE_COMMAND_TRACK_MOTOR :
//      DeviceCommandCMD.Track_Motor = uint32Pair.value;
//    break;
    
//    case  DEVICE_COMMAND_STEERING_MOTOR :
//      DeviceCommandCMD.Steering_Motor = uint32Pair.value;
//    break;
    
    case  DEVICE_COMMAND_BRUSH_DOWN :
      DeviceCommandCMD.Brush_Down = uint32Pair.value;
    break;
    
    case  DEVICE_COMMAND_BRUSH_MOTOR :
      DeviceCommandCMD.Brush_Motor = uint32Pair.value;
    break;
    
    case  DEVICE_COMMAND_LEFT_BRUSH_MOTOR :
      DeviceCommandCMD.Left_Brush_Motor = uint32Pair.value;
    break;
    
    case  DEVICE_COMMAND_RIGHT_BRUSH_MOTOR :
      DeviceCommandCMD.Right_Brush_Motor = uint32Pair.value;
    break;
    
    case  DEVICE_COMMAND_SQUEEGEE_DOWN :
      DeviceCommandCMD.Squeegee_Down = uint32Pair.value;
    break;
    
    case  DEVICE_COMMAND_SQUEEGEE_MOTOR :
      DeviceCommandCMD.Squeegee_Motor = uint32Pair.value;
    break;
    
    case  DEVICE_COMMAND_SPRAY_MOTOR :
      DeviceCommandCMD.Spray_Motor = uint32Pair.value;
    break;
    
    case  DEVICE_COMMAND_VALVE :
      DeviceCommandCMD.Valve = uint32Pair.value;
    break;
    
    case  DEVICE_COMMAND_WATER_LEVEL :
      DeviceCommandCMD.Water_Level = uint32Pair.value;
    break;
    
    case  DEVICE_COMMAND_FAN_LEVEL :
      DeviceCommandCMD.Fan_Levle = uint32Pair.value > 85 ?85:uint32Pair.value;
    break;
    
    case DEVICE_COMMAND_BRUSH_SPIN_LEVEL:
        DeviceCommandCMD.brush_spin_level = uint32Pair.value;
        break;
    
    case DEVICE_COMMAND_BRUSH_PRESSURE_LEVEL:
        DeviceCommandCMD.brush_pressure_level = uint32Pair.value;
        break;
    
    case DEVICE_COMMAND_SPEED_LEVEL:
        break;
    
    case DEVICE_COMMAND_DETERGENT_LEVEL:
        break;
    
    case DEVICE_COMMAND_DETERGENT_MOTOR:
        break;
    
    case DEVICE_COMMAND_LED_CMD:

        break;
    
    case DEVICE_COMMAND_FILTRE_PWM:
        DeviceCommandCMD.filter_motor_pwm = uint32Pair.value;
        break;
    
	case DEVICE_COMMAND_OUTLET_VALVE:
		DeviceCommandCMD.outlet_cmd = uint32Pair.value;
		break;
    
	case DEVICE_COMMAND_WORK_STATUS:
        DeviceCommandCMD.work_status = uint32Pair.value;
        break;
    
	case DEVICE_COMMAND_HEART_STATUS: //充电桩心跳
        break;
    
	case DEVICE_COMMAND_ORDER_CHARGER: //预约充电桩
        break;	
    
	case DEVICE_COMMAND_MOTOR_MODE: //位置模式
        DeviceCommandCMD.motor_mode = uint32Pair.value;
        break;
    
	case DEVICE_COMMAND_CHECK_CHARGER: //充电桩查询命令
        break;	
    
    case DEVICE_COMMAND_CLEAR_PROTECTOR_TRIGGET:
        ClearAntiCollisionTrigger();
        break;
    
	case DEVICE_COMMAND_TIME: //上位机实时时间
		DeviceCommandCMD.pc_current_time = uint32Pair.value + 28800;
//        SetSysUtc8( DeviceCommandCMD.pc_current_time );
        break;
    
    case DEVICE_COMMAND_RESTART_TIME:
        
        break;
	
    case DEVICE_COMMAND_EMERGENCY:
		DeviceCommandCMD.set_emergency = uint32Pair.value&0x01;
		break;
    
    case DEVICE_COMMAND_KEEP_VACCUM:
        DeviceCommandCMD.keep_vaccum = uint32Pair.value;
        break;
    
    case DEVICE_COMMAND_SIDE_BRUSH_DOWN:
        DeviceCommandCMD.side_brush_down = uint32Pair.value;
        break;
    
    case DEVICE_COMMAND_LEFT_BRUSH_SPIN_LEVEL: // 左刷盘转速
        DeviceCommandCMD.left_brush_pwm = uint32Pair.value;
        break;
    
	case DEVICE_COMMAND_RIGHT_BRUSH_SPIN_LEVEL: //右刷盘转速
        DeviceCommandCMD.ritht_brush_pwm = uint32Pair.value;
        break;
    
	case DEVICE_COMMAND_MANUAL_DRAIN_MODE: // 手动排水开关
        break;
    
	case DEVICE_COMMAND_AUTO_FILLER_MODE: // 自动加水开关
        break;
    
	case DEVICE_COMMAND_SIDE_BRUSH_MOTOR: // 左右边刷电机开关
        DeviceCommandCMD.side_brush_enable = uint32Pair.value;
        break;
    
    case DEVICE_COMMAND_CLEAR_EMERGENCY:
        DeviceCommandCMD.clear_emergency = uint32Pair.value&0x01;
        break;
    case DEVICE_COMMAND_DUST_PUSH_SPIN:
        DeviceCommandCMD.dust_push_spin_level = uint32Pair.value;
        break;
    
    case DEVICE_COMMAND_DUST_PUSH_CLEAN_SPIN:
        DeviceCommandCMD.dust_push_clean_spin_level = uint32Pair.value;
        break;
    
    case DEVICE_COMMAND_ANTI_DROP_SWITCH:
        DeviceCommandCMD.anti_drop_switch = uint32Pair.value&0x01;
        break;
    
    case DEVICE_COMMAND_MIST_SPRAY_LEVEL:
		DeviceCommandCMD.disinfect_mist_spray_level = uint32Pair.value&0xFF;
		break;
	
	case DEVICE_COMMAND_SPRAY_DISTANCE:
		DeviceCommandCMD.disinfect_spray_distance = uint32Pair.value&0xFF;
		break;
    
    case DEVICE_COMMAND_DISINFECT_BOX_BEEP:
        DeviceCommandCMD.disinfect_box_beep = uint32Pair.value;
        break;
    
    case DEVICE_COMMAND_DISINFECT_DISINFECTANT_PUMP:
        DeviceCommandCMD.disinfect_disinfectant_pump = uint32Pair.value;
        break;
    case 73:
        DeviceCommandCMD.roller_dir = uint32Pair.value&0xFF;
		break;
    case 96:
        DeviceCommandCMD.brush_lift_taget_position = uint32Pair.value;
        break;
    case 104:
		DeviceCommandCMD.reset_imu = uint32Pair.value;
		break;
	case 106:                                                      //边刷推杆位置控制
		DeviceCommandCMD.side_brush_level = uint32Pair.value;
		break;    
	case 107:                                                      //滚刷自适应控制目标电流
		DeviceCommandCMD.self_adapt_target_current = uint32Pair.value;
		break;  
	case 111:
        DeviceCommandCMD.mist_relay_1 = uint32Pair.value;  //消杀3.0继电器1 0关闭1打开
		break;
    case 112:
        DeviceCommandCMD.mist_relay_2 = uint32Pair.value;  //消杀3.0继电器2 0关闭1打开
		break;
    case 113:
        DeviceCommandCMD.incense_relay = uint32Pair.value; //香薰3.0继电器  0关闭1打开
  		break;  
    case 132:
        DeviceCommandCMD.zlac_current_limit = uint32Pair.value; //中菱电机速度环最大电流限制单位ma
  		break; 	
    
    case DEVICE_COMMAND_HUB_MOTOR_DAMPING:
        DeviceCommandCMD.hub_motor_damping = uint32Pair.value; /* 轮毂电机阻尼模式 */
        break;

    case DEVICE_COMMAND_HUB_MOTOR_POWER_ON:
        DeviceCommandCMD.hub_motor_power_on = uint32Pair.value; /* 轮毂电机上下电 */
        break;
	
    default : break;

  }
    return true;
}

/****************************************************************************************************************************
* Function Name       :  AntiDropDATA_callback
* Create Date         :  2017/4/1
* Author/Corporation  :  ChengPengfei/Gaussian Robot
* Description         :  callback function for antidrop
*
* Parameter           :  None
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        17.4.1   ChengPengfei               first code
*****************************************************************************************************************************/
static bool AntiDropDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
    UInt32Pair uint32Pair = { 0, 0 };

    uint32Pair.key = 0;
    uint32Pair.value =  gt_AntiDrop.un_channel1_Distance;//
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = 1;
    uint32Pair.value =  gt_AntiDrop.un_channel2_Distance;//
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = 2;
    uint32Pair.value =  gt_AntiDrop.un_channel3_Distance;//
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = 3;
    uint32Pair.value =  gt_AntiDrop.un_channel4_Distance;//
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	return true;
}

static bool AntiDroptrigger_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
	UInt32Pair uint32Pair = { 0, 0 };

    uint32Pair.key = 0;
    uint32Pair.value = g_antiDropStop & 0x01;//
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = 1;
    uint32Pair.value = g_antiDropStop & 0x02;//
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = 2;
    uint32Pair.value = g_antiDropStop & 0x04;//
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = 3;
    uint32Pair.value = g_antiDropStop & 0x08;//
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
	return true;
}

static bool HardWareDriverDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
	int i;	
	UInt32Pair uint32Pair = UInt32Pair_init_zero;

	for(i=0;i<7;i++)
	{		
		uint32Pair.key = i;
		#ifdef DRIVER_CONTORL_RXOFF
		uint32Pair.value = HardWareDriverUpdateBuf[i];		
		#else
		uint32Pair.value = HardWareDriverisUsedBuf[i];
		#endif
		if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;
	}

	return true;
}

static bool AntiCollisionDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
	int i;	
	UInt32Pair uint32Pair = UInt32Pair_init_zero;

	for(i=0;i<8;i++)
	{		
		uint32Pair.key = i;
		uint32Pair.value = (uint16_t)GetAntiCollisionReat(i);
		
		if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;
	}
    
    uint32Pair.key = 8;
    uint32Pair.value = g_all_rate_idx[7]; //当前有几组数据
    if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;
    
    for(i=9;i<17;i++)
    {
        uint32Pair.key = i;
		uint32Pair.value = g_all_rate[i-9];
        g_all_rate[i-9] = 0;
        g_all_rate_idx[i-9] = 0;
		
		if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;
    }

	return true;
}

static bool AntiCollisionDATAtrigger_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
	int i;
	UInt32Pair uint32Pair = UInt32Pair_init_zero;
    uint8_t stop = 0;
    
    stop = GetAntiCollisionStop();
	
	for(i=0;i<8;i++)
	{
		uint32Pair.key = i;
		uint32Pair.value = (stop >> i) & 0x01;
		
		if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;
	}

	return true;
}
/****************************************************************************************************************************
* Function Name       :  VersionDATA_callback
* Create Date         :  2016/12/14
* Author/Corporation  :  ChengPengfei/Gaussian Robot
* Description         :  处理输出数据的repeated
*
* Parameter           :  None
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.12.14   ChengPengfei               first code
*****************************************************************************************************************************/
static bool VersionDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{

  VersionData_VersionStruct versionStruct = {0,0,0,0,0};

  versionStruct.board = VERSION_DATA_PRODUCT;
  versionStruct.type = V_Product.type;
  versionStruct.major = V_Product.major;
  versionStruct.minor = V_Product.minor;
  versionStruct.build = V_Product.build;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, VersionData_VersionStruct_fields, &versionStruct)))
    return false;
  
  versionStruct.board = VERSION_DATA_MAIN_CONTROL_BOARD;
  versionStruct.type = V_Main.type;
  versionStruct.major = V_Main.major;
  versionStruct.minor = V_Main.minor;
  versionStruct.build = V_Main.build;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, VersionData_VersionStruct_fields, &versionStruct)))
    return false;
  
  
  versionStruct.board = VERSION_DATA_AVR;
  GetBootloadVer((version_t *)&versionStruct.type);
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, VersionData_VersionStruct_fields, &versionStruct)))
    return false;

  versionStruct.board = VERSION_DATA_IMU_BOARD;
  versionStruct.type = V_Gyro.type;
  versionStruct.major = V_Gyro.major;
  versionStruct.minor = V_Gyro.minor;
  versionStruct.build = V_Gyro.build;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, VersionData_VersionStruct_fields, &versionStruct)))
    return false;
  

  versionStruct.board = VERSION_DATA_ULTRASONIC_BOARD;
  versionStruct.type = V_Sonar.type;
  versionStruct.major = V_Sonar.major;
  versionStruct.minor = V_Sonar.minor;
  versionStruct.build = V_Sonar.build;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, VersionData_VersionStruct_fields, &versionStruct)))
    return false;
  
  versionStruct.board = VERSION_DATA_BACK_MAIN_CONTROL;
  GetAnotherVer((version_t *)&versionStruct.type);
  if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, VersionData_VersionStruct_fields, &versionStruct)))
    return false;
  
    versionStruct.board = VERSION_DATA_BATTERY_VERSION;
    versionStruct.type = 0;
    versionStruct.major = 0;
	if (BMS_Info.battery_type == 0)
	{
        versionStruct.minor = (get_rs485_bms_version() >> 8) & 0xFF;
        versionStruct.build = get_rs485_bms_version() & 0xFF;
	}
	else
	{
        versionStruct.minor = BmsParam.app_version/10;
        versionStruct.build = BmsParam.app_version%10;
	}
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, VersionData_VersionStruct_fields, &versionStruct)))
        return false;
  
  versionStruct.board = VERSION_DATA_DX510_VERSION;
  versionStruct.type = V_Xd510.type;
  versionStruct.major = V_Xd510.major;
  versionStruct.minor = V_Xd510.minor;
  versionStruct.build = V_Xd510.build;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, VersionData_VersionStruct_fields, &versionStruct)))
    return false;
  
  versionStruct.board = VERSION_DATA_GCUB_VERSION;
  versionStruct.type = V_Gcub.type;
  versionStruct.major = V_Gcub.major;
  versionStruct.minor = V_Gcub.minor;
  versionStruct.build = V_Gcub.build;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, VersionData_VersionStruct_fields, &versionStruct)))
    return false;
	
	versionStruct.board = 9;
	versionStruct.type = V_H25A_S.type;
	versionStruct.major = V_H25A_S.major;
	versionStruct.minor = V_H25A_S.minor;
	versionStruct.build = V_H25A_S.build;
	if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, VersionData_VersionStruct_fields, &versionStruct)))
	return false; 

	versionStruct.board = 10;
	versionStruct.type =  V_ZLAC_S.type;
	versionStruct.major = V_ZLAC_S.major;
	versionStruct.minor = V_ZLAC_S.minor;
	versionStruct.build = V_ZLAC_S.build;
	if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, VersionData_VersionStruct_fields, &versionStruct)))
	return false;  
	
	versionStruct.board = 12;
	versionStruct.type = V_H25A_H.type;
	versionStruct.major = V_H25A_H.major;
	versionStruct.minor = V_H25A_H.minor;
	versionStruct.build = V_H25A_H.build;
	if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, VersionData_VersionStruct_fields, &versionStruct)))
	return false;      

    return true;
}

/****************************************************************************************************************************
* Function Name       :  RFIDDATA_callback
* Create Date         :  2017/01/05
* Author/Corporation  :  ChengPengfei/Gaussian Robot
* Description         :  处理输入数据的repeated,对每一个key进行解码，获取相应的value
*
* Parameter           :  None
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        17.01.05   ChengPengfei               first code
*****************************************************************************************************************************/
static bool RFIDDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
	RFIDData_RFIDStruct RFID_Data = RFIDData_RFIDStruct_init_zero;
	for(int i=0;i<RFID_TAG_MAX_NUM;i++)
	{
		if( g_rfid_tag_table[i].tag_num )
		{
			RFID_Data.id = g_rfid_tag_table[i].id;
			RFID_Data.data = g_rfid_tag_table[i].tag_num;
			RFID_Data.strength = 0;
			if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, RFIDData_RFIDStruct_fields, &RFID_Data)))
			{	
				return false;
			}
		}
		else if( i )
			break;
	}
	return true;
}


/****************************************************************************************************************************
* Function Name       :  ProtecterDATA_callback
* Create Date         :  2016/12/14
* Author/Corporation  :  ChengPengfei/Gaussian Robot
* Description         :  处理输入数据的repeated,对每一个key进行解码，获取相应的value
*
* Parameter           :  None
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.12.26   ChengPengfei               first code
*****************************************************************************************************************************/
static bool ProtectorDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
    UInt32Pair uint32Pair = {0,0};

    uint32Pair.key = PROTECTOR_DATA_RFID_ALARM;
    uint32Pair.value = g_protector_data.rfid_alarm;
    if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    return true;
} 

/****************************************************************************************************************************
* Function Name       :  ProtectorCheck
* Create Date         :  2016/12/14
* Author/Corporation  :  ChengPengfei/Gaussian Robot
* Description         :  根据上位机下发的防撞条配置，将相应的状态位处理出来，返回有效状态位的个数
*
* Parameter           :  None
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.12.26   ChengPengfei               first code
*****************************************************************************************************************************/
//static uint8_t ProtectorCheck(void)
//{
//	return 0;
//}
/****************************************************************************************************************************
* Function Name       :  HealthDATA_callback
* Create Date         :  2016/12/14
* Author/Corporation  :  ChengPengfei/Gaussian Robot
* Description         :  处理输出数据的repeated
*
* Parameter           :  None
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.12.14   ChengPengfei               first code
*****************************************************************************************************************************/
static bool HealthDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
    UInt32Pair uint32Pair = {0,0};
    


	if(ConfigCommandCMD.cancel_485 ==0)    //
	{
	    uint32Pair.key = HEALTH_DATA_ULTRASONIC_BOARD;
		uint32Pair.value = HealthDATA.Ultrasonic_Board;
		if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;	
	}

    uint32Pair.key = HEALTH_DATA_IMU_BOARD;
    uint32Pair.value = HealthDATA.Imu_Board;
    if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = HEALTH_DATA_MOTOR_DRIVER;
    uint32Pair.value = HealthDATA.Motor_Driver;
    if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;


    uint32Pair.key = HEALTH_DATA_RFID;
    uint32Pair.value = HealthDATA.Rfid;
    if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = HEALTH_DATA_LEFT_MOTOR;
    uint32Pair.value = HealthDATA.Left_Motor;
    if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = HEALTH_DATA_RIGHT_MOTOR;
    uint32Pair.value = HealthDATA.Right_Motor;
    if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

//    uint32Pair.key = HEALTH_DATA_BRUSH_MOTOR;
//    uint32Pair.value = HealthDATA.Brush_Motor;
//    if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = HEALTH_DATA_LEFT_BRUSH_MOTOR;
//    uint32Pair.value = HealthDATA.left_brush_motor;
//    if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = HEALTH_DATA_RIGHT_BRUSH_MOTOR;
//    uint32Pair.value = HealthDATA.right_brush_motor;
//    if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
    

    return true;
}
static bool BatteryDATA_callback(pb_ostream_t* stream, const pb_field_t* field, void* const* arg)
{
    uint16_t i = 0;
    UInt32Pair uint32Pair = { 0, 0 };
    uint32Pair.key = BATTERY_DATA_BATTERY_HALF_VOLTAGE;
    uint32Pair.value = g_battery_data.voltage;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = BATTERY_DATA_BATTERY_CELL_NUMBER;
    uint32Pair.value = g_battery_data.cellNum;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    for (i = 0; i < g_battery_data.cellNum; i++) {
        uint32Pair.key = (1 << 10) + i;
        uint32Pair.value = g_battery_data.cellVoltage[i];
        if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
            return false;
    }
    
    uint32Pair.key = BATTERY_DATA_CHARGE_CURRENT;
    uint32Pair.value = (g_battery_data.current>0)?g_battery_data.current:0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = BATTERY_DATA_DISCAHRGE_CURRENT;
    uint32Pair.value = (g_battery_data.current<0)?-g_battery_data.current:0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = BATTERY_DATA_TEMPERATURE_NUM;
    uint32Pair.value = g_battery_data.temp_num;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    for (i = 0; i < g_battery_data.temp_num; i++) {
        uint32Pair.key = (2 << 10) + i;
        uint32Pair.value = g_battery_data.cellTemp[i];
        if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
            return false;
    }
    
//    uint32Pair.key = BATTERY_DATA_VOLTAGE_STATE;
//    uint32Pair.value = battery_data.VState;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = BATTERY_DATA_CURRENT_STATE;
//    uint32Pair.value = battery_data.CState;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = BATTERY_DATA_TEMPERATURE_STATE;
//    uint32Pair.value = battery_data.TState;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
    
    uint32Pair.key = BATTERY_DATA_BATTERY_ALARM;
    uint32Pair.value = g_battery_data.alarmSt;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
//    uint32Pair.key = BATTERY_DATA_FETMOS_STATE;
//    uint32Pair.value = battery_data.FETState;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = BATTERY_DATA_OVER_VOLTAGE_CELL_NUM;
//    uint32Pair.value = battery_data.cellnum_of_over_voltage;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = BATTERY_DATA_UNDER_VOLTAGE_CELL_NUM;
//    uint32Pair.value = battery_data.cellnum_of_under_voltage;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = BATTERY_DATA_WARN_OVER_VOLTAGE_CELL_NUM;
//    uint32Pair.value = battery_data.cellnum_of_warn_over_voltage;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = BATTERY_DATA_WARN_UNDER_VOLTAGE_CELL_NUM;
//    uint32Pair.value = battery_data.cellnum_of_warn_under_voltage;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
    
    uint32Pair.key = BATTERY_DATA_BALANCE_STATE;
    uint32Pair.value = g_battery_data.balanceSt;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = BATTERY_DATA_DISCHARGE_NUM;
    uint32Pair.value = g_battery_data.cycleTimes;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = BATTERY_DATA_CHARGE_NUM;
    uint32Pair.value = g_battery_data.cycleTimes;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = BATTERY_DATA_CAP_NOW;
    uint32Pair.value = g_battery_data.sp;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = BATTERY_DATA_CAP_FULL;
    uint32Pair.value = g_battery_data.capacityIn;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
//    uint32Pair.key = BATTERY_DATA_DRIVE_MOTOR_CURRENT;
//    uint32Pair.value = TractMotorCurrent;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = BATTERY_DATA_BRUSH_MOTOR_CURRENT;
//    uint32Pair.value = CarStatus.motor.brush.run.cur;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = BATTERY_DATA_TURN_MOTOR_CURRENT;
//    uint32Pair.value = TurnMotorCurrent;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = BATTERY_DATA_BRUSHLIFT_MOTOR_CURRENT;
//    uint32Pair.value = CarStatus.motor.brush_lift.run.cur;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = BATTERY_DATA_SEQUEEGEELIFT_MOTOR_CURRENT;
//    uint32Pair.value = CarStatus.motor.squeegee_lift.run.cur;;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = BATTERY_DATA_POTENTIOMETER_VOLTAGE;
//    uint32Pair.value = TurnPotentionmeterVoltage;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = BATTERY_DATA_ACCELERATOR_VOLTAGE;
//    uint32Pair.value = AcceleratorVoltage;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    
//    uint32Pair.key = BATTERY_DATA_TURN_LIMIT_STATUS;
//    uint32Pair.value = TurnLimitStatus;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
    
    uint32Pair.key = BATTERY_DATA_BATTERY_VOLTAGE;
    uint32Pair.value = g_battery_data.voltage;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;    
    
    uint32Pair.key = BATTERY_DATA_BATTERY_CONNECTION;
    uint32Pair.value = HealthDATA.battery_connect;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    return true;
}
/****************************************************************************************************************************
* Function Name       :  DEVICEDATA_callback
* Create Date         :  2016/12/14
* Author/Corporation  :  ChengPengfei/Gaussian Robot
* Description         :  处理输出数据的repeated
*
* Parameter           :  None
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.12.14   ChengPengfei               first code
*****************************************************************************************************************************/
extern uint32_t gs_left_encoder_read_cnt;
extern uint32_t gs_right_encoder_read_cnt;
extern uint32_t gs_left_encoder_tick;
extern uint32_t gs_left_encoder_cnt;
extern uint32_t gs_right_encoder_tick;
extern uint32_t gs_right_encoder_cnt;
static bool DeviceDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
    UInt32Pair uint32Pair = {0,0};
    
  uint32Pair.key = DEVICE_DATA_DI;
  uint32Pair.value = DeviceDATA.Di;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = DEVICE_DATA_DO;
  uint32Pair.value = DeviceDATA.Do;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = DEVICE_DATA_RELAY;
  uint32Pair.value = DeviceDATA.Relay;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = DEVICE_DATA_EMERGENCY;
  uint32Pair.value = DeviceDATA.Emergency;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = DEVICE_DATA_BRAKER_DOWN;
  uint32Pair.value = DeviceDATA.Braker_Down;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = DEVICE_DATA_BATTERY_VOLTAGE_ADC;
  uint32Pair.value = DeviceDATA.Battery_Voltage_ADC;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = DEVICE_DATA_CHARGER_VOLTAGE_ADC;
  uint32Pair.value = DeviceDATA.Charger_Voltage_ADC;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = DEVICE_DATA_CHARGER_CURRENT_ADC;
  uint32Pair.value = DeviceDATA.Charge_Current_ADC;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = DEVICE_DATA_REMOTE_CONTROLLER;
  uint32Pair.value = 0;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = DEVICE_DATA_AUTO_MODE;
  uint32Pair.value = DeviceDATA.Auto_Mode;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  
  uint32Pair.key = DEVICE_DATA_BRUSH_MOTOR_WORKING;
  uint32Pair.value = DeviceDATA.Brush_Motor_Working;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
//  uint32Pair.key = DEVICE_DATA_FILTER_MOTOR_WORKING;
//  uint32Pair.value = DeviceDATA.Filter_Motor_Working;
//  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//    return false;
  
  uint32Pair.key = DEVICE_DATA_SQUEEGEE_DOWN;
  uint32Pair.value = DeviceDATA.Squeegee_Down;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = DEVICE_DATA_SQUEEGEE_MOTOR_WORKING;
  uint32Pair.value = DeviceDATA.Squeegee_Motor_Working;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  

    uint32Pair.key = DEVICE_DATA_BATTERY_PERCENTAGE;
    uint32Pair.value = DeviceDATA.BatterySoc;//BalancePowerLever;
    if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;	
    
    uint32Pair.key = DEVICE_DATA_POWER_SWITCH;
    uint32Pair.value = DeviceDATA.Power_Switch;
    if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;

        
    uint32Pair.key = DEVICE_DATA_MCU_ERROR_CODE13;
    uint32Pair.value = DeviceDATA.mcu_error_code[13];
    ClearPcDisconnect();
    ClearSpeedTimeout();
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
//    uint32Pair.key = DEVICE_DATA_SPRAY_MOTOR_CURRENT;
//    uint32Pair.value = DeviceDATA.spray_motor_current;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
//    uint32Pair.key = DEVICE_DATA_FILTER_MOTOR_CURRENT;
//	uint32Pair.value = DeviceDATA.filter_motor_current;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
    
	uint32Pair.key = DEVICE_DATA_CHARGE_ADC;
	if(  DeviceDATA.Charge_Current_ADC > 0) //手动充电时，充电电压
	{
		uint32Pair.value = DeviceDATA.Battery_Voltage_ADC; //电池电压
	}
	else
	{
		uint32Pair.value = 0;
	}
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
    uint32Pair.key = DEVICE_DATA_VACUUM_CURRENT;
    uint32Pair.value = DeviceDATA.vacuum_motor_current;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
//    uint32Pair.key = DEVICE_DATA_WATER_SENSOR_STATUS;
//    uint32Pair.value = DeviceDATA.water_level_sensor_status;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;

    uint32Pair.key = DEVICE_DATA_LEFT_MOTOR_LOAD;
    uint32Pair.value = DeviceDATA.left_motor_load_factor;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
    uint32Pair.key = DEVICE_DATA_RIGHT_MOTOR_LOAD;
    uint32Pair.value = DeviceDATA.right_motor_load_factor;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = DEVICE_DATA_MCU_ERROR_CODE_14;
    uint32Pair.value = DeviceDATA.mcu_error_code[14];
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
	uint32Pair.key = DEVICE_DATA_MCU_ERROR_CODE_15;
    uint32Pair.value = DeviceDATA.mcu_error_code[15];
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
	uint32Pair.key = DEVICE_DATA_MODE_BUTTON;
    uint32Pair.value = DeviceDATA.mode_button_status;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = DEVICE_DATA_SIDE_BRUSH_DOWN;  //84
    uint32Pair.value = DeviceDATA.side_brush_down;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = DEVICE_DATA_LEFT_BRUSH_MOTOR_WORKING;
    uint32Pair.value = DeviceDATA.left_brush_motor_working;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = DEVICE_DATA_RIGHT_BRUSH_MOTOR_WORKING;
    uint32Pair.value = DeviceDATA.right_brush_motor_working;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = DEVICE_DATA_MCU_ERROR_CODE_24;
    uint32Pair.value = DeviceDATA.mcu_error_code[24];
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = DEVICE_DATA_MCU_ERROR_CODE_26;
    uint32Pair.value = DeviceDATA.mcu_error_code[26];
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = DEVICE_DATA_MCU_ERROR_CODE_27;
    uint32Pair.value = DeviceDATA.mcu_error_code[27];
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
  	uint32Pair.key = 16;
    uint32Pair.value = DeviceDATA.Brush_Down;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
  	uint32Pair.key = 104;
    uint32Pair.value = DeviceDATA.imu_reset_status;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;  
	
  	uint32Pair.key = 108;
    uint32Pair.value = DeviceDATA.dust_bag_voltage1;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
	uint32Pair.key = 109;
    uint32Pair.value = DeviceDATA.dust_bag_voltage2;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false; 
	
	uint32Pair.key = 114;
    uint32Pair.value = DeviceDATA.h25a_error_code;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false; 
	
    uint32Pair.key = 121;
    uint32Pair.value = DeviceDATA.imu_data_update_tick;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = 122;
    uint32Pair.value = DeviceDATA.imu_uart_recieve_cnt;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = 123;
    uint32Pair.value = DeviceDATA.imu_task_run_cnt;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
	uint32Pair.key = 126;
    uint32Pair.value = DeviceDATA.zlac_error_code1;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

	uint32Pair.key = 127;
    uint32Pair.value = DeviceDATA.zlac_error_code2;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
	uint32Pair.key = 128;
    uint32Pair.value = DeviceDATA.h25a_error_code_sub;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false; 	

	uint32Pair.key = 204;
    uint32Pair.value = DeviceDATA.zlac_version;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;		
	uint32Pair.key = 211;
    uint32Pair.value = DeviceDATA.hull_disconnect_flag;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;			

	uint32Pair.key = 300;
    uint32Pair.value = gs_left_encoder_tick;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;	

	uint32Pair.key = 301;
    uint32Pair.value = gs_left_encoder_cnt;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
	uint32Pair.key = 302;
    uint32Pair.value = gs_right_encoder_tick;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;	

	uint32Pair.key = 303;
    uint32Pair.value = gs_right_encoder_cnt;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

	uint32Pair.key = 304;
    uint32Pair.value = gs_left_encoder_read_cnt;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

	uint32Pair.key = 305;
    uint32Pair.value = gs_right_encoder_read_cnt;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

	return true;
}
#include "h25a15/app_h25a15.h"
#include "h25a15/drv_h25a15.h"
/****************************************************************************************************************************
*	Function Name       :  DEVICEDATA_callback
*	Create Date         :  2016/12/14
*	Author/Corporation  :  ChengPengfei/Gaussian Robot
*	Description         :  处理输出数据的repeated
*
* Parameter           :  None
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.12.14   ChengPengfei               first code
*****************************************************************************************************************************/
static bool OtherDeviceData_callback(pb_ostream_t* stream, const pb_field_t* field, void* const* arg)
{
    UInt32Pair uint32Pair = { 0, 0 };
    
//    uint32Pair.key = OTHER_DEVICE_DATA_COMMAND_RECEIVE;
//    uint32Pair.value = CarStatus.motor.position_mode_ack;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;

//	uint32Pair.key = OTHER_DEVICE_DATA_IS_FINISHED;
//    uint32Pair.value = CarStatus.motor.position_mode_status;
//    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//        return false;
    uint32Pair.key = OTHER_DEVICE_DATA_MOTOR_MODE;
    uint32Pair.value = Other_DeviceData.motor_mode;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = OTHER_DEVICE_DATA_MCU_TYEP;
    uint32Pair.value = 4;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

	uint32Pair.key = OTHER_DEVICE_DATA_DN_MOTOR_DEVICE_TEMPERATURE;
    uint32Pair.value = Other_DeviceData.xd510_temperature;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

	uint32Pair.key = OTHER_DEVICE_DATA_LEFT_MOTOR_TEMPERATURE;
    uint32Pair.value = Other_DeviceData.left_motor_temperature;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;	

 	uint32Pair.key = OTHER_DEVICE_DATA_RIGHT_MOTOR_TEMPERATURE;
    uint32Pair.value = Other_DeviceData.right_motor_temperature;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = OTHER_DEVICE_DATA_MCU_ERROR_CODE_16;
    uint32Pair.value = DeviceDATA.mcu_error_code[16];
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
    uint32Pair.key = OTHER_DEVICE_DATA_MCU_ERROR_CODE_17;
    uint32Pair.value = DeviceDATA.mcu_error_code[17];
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = OTHER_DEVICE_DATA_MCU_ERROR_CODE_18;
    uint32Pair.value = DeviceDATA.mcu_error_code[18];
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = OTHER_DEVICE_DATA_MCU_ERROR_CODE_19;
    uint32Pair.value = DeviceDATA.mcu_error_code[19];
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;	
	
	uint32Pair.key = OTHER_DEVICE_DATA_BRUSH_LIFT_MOTOR_CURRENT;
    uint32Pair.value = Other_DeviceData.brush_lift_motor_current;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;	
	
	uint32Pair.key = OTHER_DEVICE_DATA_BRUSH_MOTOR_CURRENT;
    uint32Pair.value = Other_DeviceData.brush_motor_current;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;	
	
	uint32Pair.key = OTHER_DEVICE_DATA_SQUEEGEE_LIFT_MOTOR_CURRENT;
    uint32Pair.value = Other_DeviceData.squeegee_lift_motor_current;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;	
	
	uint32Pair.key = OTHER_DEVICE_DATA_LEFT_MOTOR_CURRENT;
    uint32Pair.value = Other_DeviceData.left_motor_current;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;	
	
	uint32Pair.key = OTHER_DEVICE_DATA_RIGHT_MOTOR_CURRENT;
    uint32Pair.value = Other_DeviceData.right_motor_current;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = OTHER_DEVICE_DATA_MCU_ERROR_CODE_20;
    uint32Pair.value = DeviceDATA.mcu_error_code[20];
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
    uint32Pair.key = OTHER_DEVICE_DATA_STARTUP_TYPE;
    uint32Pair.value = Other_DeviceData.starup_type;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = OTHER_DEVICE_DATA_IMU_DELTA_YAW;
    uint32Pair.value = Other_DeviceData.imu_delta_yaw;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = OTHER_DEVICE_DATA_DUST_PUSH_SPIN;
    uint32Pair.value = Other_DeviceData.dust_push_spin;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = OTHER_DEVICE_DATA_DUST_PUSH_CLEAN_SPIN;
    uint32Pair.value = Other_DeviceData.dust_push_clean_spin;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = OTHER_DEVICE_DATA_DUST_PUSH_CURRENT;
    uint32Pair.value = Other_DeviceData.dust_push_current;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = OTHER_DEVICE_DATA_DUST_PUSH_CLEAN_CURRENT;
    uint32Pair.value = Other_DeviceData.dust_push_clean_current;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = OTHER_DEVICE_DATA_ANTI_DROP_STATUS;
    uint32Pair.value = Other_DeviceData.anti_drop_switch;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = OTHER_DEVICE_DATA_EMERGENCY_BUTTON;
    uint32Pair.value = Other_DeviceData.emergency_button;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = OTHER_DEVICE_DATA_MIST_SPRAY_LEVEL;
    uint32Pair.value = Other_DeviceData.disinfect_mist_spray_level;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = OTHER_DEVICE_DATA_TORQUE_VOLTAGE1;
    uint32Pair.value = Other_DeviceData.torque_voltage1;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = OTHER_DEVICE_DATA_TORQUE_VOLTAGE2;
    uint32Pair.value = Other_DeviceData.torque_voltage2;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    uint32Pair.key = OTHER_DEVICE_DATA_DISINFECT_ATOMIZATION_BOX_LEVEL;
    uint32Pair.value = Other_DeviceData.disinfect_atomization_box_level;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
	uint32Pair.key = 55;
    uint32Pair.value = GetRollerSpeed();//Other_DeviceData.roller_speed;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
	uint32Pair.key = 85;
    uint32Pair.value = GetBrushLiftHullCountReal();
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
	uint32Pair.key = 95;           //负压传感器电压值 AD2
    uint32Pair.value = GetTorqueVoltage2();
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

	uint32Pair.key = 100;
    uint32Pair.value = Other_DeviceData.roller_motor_position;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	    
	uint32Pair.key = 109;
    uint32Pair.value = Other_DeviceData.side_lift_position;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;	

	uint32Pair.key = 110;
	uint32Pair.value = Other_DeviceData.left_side_motor_current;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;
	
	uint32Pair.key = 111;
	uint32Pair.value = Other_DeviceData.right_side_motor_current;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
	return false;
		
	uint32Pair.key = 112;
    uint32Pair.value = Other_DeviceData.side_lift_motor_current;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	uint32Pair.key = 118;
	uint32Pair.value = Other_DeviceData.disfection_level;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
	return false;
    
    uint32Pair.key = 119;
	uint32Pair.value = Other_DeviceData.atomizer_level;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
	return false;
    
    uint32Pair.key = 120;
	uint32Pair.value = Other_DeviceData.incense_level;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
	return false;
   
    uint32Pair.key = 121;
	uint32Pair.value = (Other_DeviceData.disinfect_detect&0x01) | ((Other_DeviceData.incense_detect&0x01)<<1);
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
	return false;
	
	uint32Pair.key = 125;
    uint32Pair.value = GetChargeDetectionInputInfo();
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = OTHER_DEVICE_DATA_SIDE_DOOR_STATUS;
    uint32Pair.value = Other_DeviceData.side_door_status;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
 	uint32Pair.key = 132;
    uint32Pair.value =Other_DeviceData.zlac_current_limit;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
 	uint32Pair.key = 133;
    uint32Pair.value =Other_DeviceData.xds_driver_info;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = 136;
    uint32Pair.value = DeviceCommandCMD.Fan_Levle;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
	
 	uint32Pair.key = 200;
    uint32Pair.value = Other_DeviceData.driver_heartbeat_time;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    return true;
}

#include "hub_motor/drv_zlac8015.h"
#if 0
static bool DriverData_callback(pb_ostream_t* stream, const pb_field_t* field, void* const* arg)
{
	uint32_t key_index = 0;
	static uint32_t run_cnt = 0;
	UInt32Pair uint32Pair = { 0, 0 };
	
	/*由于驱动器数据太多，打包后超过512字节，需要分开进行多次打包*/
	uint8_t temp = run_cnt++%3;
	if(temp == 0) 	
	{
		for(uint8_t i =0;i<zlac_objdict_size;i++)
		{	
			key_index = (ZLAC_objdict[i].id<<24)|(ZLAC_objdict[i].index<<8)|ZLAC_objdict[i].sub_index;
			uint32Pair.key = key_index ;
			uint32Pair.value =GetValueByIndex(key_index) ;
			if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
				return false;
		}
	}
	else if(temp == 1)
	{
		for(uint8_t i =0;i<h25_objdict_size -30;i++) 
		{	
			key_index = (H25A_objdict[i].id<<24)|(H25A_objdict[i].index<<8)|H25A_objdict[i].sub_index;
			uint32Pair.key = key_index ;
			uint32Pair.value =GetValueByIndex(key_index) ;
			if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
				return false;
		}	
	}
	else if(temp == 2)
	{
		for(uint8_t i = h25_objdict_size -30;i<h25_objdict_size;i++) 
		{	
			key_index = (H25A_objdict[i].id<<24)|(H25A_objdict[i].index<<8)|H25A_objdict[i].sub_index;
			uint32Pair.key = key_index ;
			uint32Pair.value =GetValueByIndex(key_index) ;
			if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
				return false;
		}	
	}

	return true;
}
#else
static bool DriverData_callback(pb_ostream_t* stream, const pb_field_t* field, void* const* arg)
{
	uint8_t i = 0;
    uint8_t j = 0;
	UInt32Pair uint32Pair = { 0, 0 };
	
	for(i = ZLAC_PDO_KEY_START; i < (ZLAC_PDO_KEY_START + CAN_DATA_LEN/sizeof(uint32_t)*ZLAC_PDO_NUM); i++)
	{
		uint32Pair.key = i ;
		uint32Pair.value = get_zlac_pdo_data(i);
		if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
			return false;
	}

	//轮毂电机需要通过SDO上传的数据
	uint32Pair.key = i++ ;
	uint32Pair.value = ZLAC_Motor.left_error_code;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = ZLAC_Motor.right_error_code;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = ZLAC_Motor.left_target_torque;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = ZLAC_Motor.right_target_torque;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = ZLAC_Motor.left_temperature;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = ZLAC_Motor.right_temperature;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = ZLAC_Motor.control_word;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = ZLAC_Motor.right_control_word;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = ZLAC_Motor.status_word;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = ZLAC_Motor.right_status_word;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = ZLAC_Motor.left_actual_mode;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = ZLAC_Motor.right_actual_mode;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = ZLAC_Motor.soft_version;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	for(i = H4AA30_PDO_KEY_START; i < (H4AA30_PDO_KEY_START + CAN_DATA_LEN/sizeof(uint32_t)*H4AA30_PDO_NUM); i++)
	{
		uint32Pair.key = i ;
		uint32Pair.value = get_h4aa30_pdo_data(i - H4AA30_PDO_KEY_START);
		if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
			return false;
	}

	//混合驱控需要通过SDO上传的数据
	uint32Pair.key = i++ ;
	uint32Pair.value = PushRodMotor1.current.ActualUnitValue;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = g_h4aa30_sdo_data.bus_voltage;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = H25a_Info.soft_version;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = H25a_Info.hard_version;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = RollerBrushMotor2.current.ActualUnitValue;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;
	
	uint32Pair.key = i++ ;
	uint32Pair.value = RollerBrushMotor1.current.ActualUnitValue;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = H25a_Info.error_code;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = H25a_Info.error_code_sub;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = g_h4aa30_sdo_data.roller_lift_position_loop;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = g_h4aa30_sdo_data.dust_lift_positon;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = g_h4aa30_sdo_data.dust_lift_calib;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = PushRodMotor2.calibration;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = g_h4aa30_sdo_data.brush_type;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = H25a_Info.brush_self_lift_deviation;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = H25a_Info.brush_self_lift_time;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = g_h4aa30_sdo_data.motor2_current;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;

	uint32Pair.key = i++ ;
	uint32Pair.value = g_h4aa30_sdo_data.roller_brush_pwm;
	if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
		return false;
    
	return true;
}
#endif

static bool MotorDriverConfigData_callback(pb_ostream_t* stream, const pb_field_t* field, void* const* arg)
{
    UInt32Pair uint32Pair = { 0, 0 };

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_LOW_SPEED;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_HIGH_SPEED;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_LIFT_MOTOR_OVERLOAD_CURRENT_THRESHOLD;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_LIFT_MOTOR_OVERLOAD_TIME_THRESHOLD;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_LEFT_SPREAD_MOTOR_OVERLOAD_CURRENT_THRESHOLD;
    //uint32Pair.value = SqueegeeliftMotorCurrentLimit[1];
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_LEFT_SPREAD_MOTOR_OVERLOAD_TIME_THRESHOLD;
    //uint32Pair.value = SqueegeeliftMotorOverloadTime[1];
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_RIGHT_SPREAD_MOTOR_OVERLOAD_CURRENT_THRESHOLD;
    //uint32Pair.value = SprayMotorCurrentLimit[1];
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_RIGHT_SPREAD_MOTOR_OVERLOAD_TIME_THRESHOLD;
    //uint32Pair.value = SprayMotorOverloadTime[1];
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_SQUEEGEE_MOTOR_OVERLOAD_CURRENT_THRESHOLD;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_SQUEEGEE_MOTOR_OVERLOAD_TIME_THRESHOLD;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_SPRAY_MOTOR_OVERLOAD_CURRENT_THRESHOLD;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_SPRAY_MOTOR_OVERLOAD_TIME_THRESHOLD;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_FILTER_MOTOR_OVERLOAD_CURRENT_THRESHOLD;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_FILTER_MOTOR_OVERLOAD_TIME_THRESHOLD;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_BRUSH_MOTOR_OVERLOAD_CURRENT_THRESHOLD;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_BRUSH_MOTOR_OVERLOAD_TIME_THRESHOLD;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_0_P;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_0_I;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_0_P;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_0_I;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_0_P;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_0_I;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_0_D;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_1_P;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_1_I;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_1_P;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_1_I;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_1_P;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_1_I;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_1_D;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_2_P;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_2_I;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_2_P;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_2_I;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_2_P;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_2_I;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_2_D;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    ///////////////////////////////////////////////
    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_3_P;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_CURRENT_PID_3_I;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_3_P;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_VELOCITY_PID_3_I;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_3_P;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_3_I;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_POSITION_PID_3_D;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_STEERING_VELOCITY_COEFFICIENT;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_INITIALIZE_STEERING;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;

    uint32Pair.key = MOTOR_DRIVER_CONFIG_DATA_TRACT_PWM_ACC;
    uint32Pair.value = 0;
    if ((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
        return false;
    
    return true;
}
/****************************************************************************************************************************
* Function Name       :  UltraDATA_callback
* Create Date         :  2016/12/14
* Author/Corporation  :  ChengPengfei/Gaussian Robot
* Description         :  处理输出数据的repeated
*
* Parameter           :  None
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.12.14   ChengPengfei               first code
*****************************************************************************************************************************/
static bool UltraDATA_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
    UInt32Pair uint32Pair = {0,0};
    
  uint32Pair.key = 0; 
  uint32Pair.value = UltrasonicDATA.ks103_data0;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = 1;
  uint32Pair.value = UltrasonicDATA.ks103_data1;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = 2;
  uint32Pair.value = UltrasonicDATA.ks103_data2;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = 3;
  uint32Pair.value = UltrasonicDATA.ks103_data3;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = 4;
  uint32Pair.value = UltrasonicDATA.ks103_data4;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = 5;
  uint32Pair.value = UltrasonicDATA.ks103_data5;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = 6;
  uint32Pair.value = UltrasonicDATA.ks103_data6;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = 7;
  uint32Pair.value = UltrasonicDATA.ks103_data7;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = 8;
  uint32Pair.value = UltrasonicDATA.ks103_data8;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = 9;
  uint32Pair.value = UltrasonicDATA.ks103_data9;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = 10;
  uint32Pair.value = UltrasonicDATA.ks103_data10;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
  uint32Pair.key = 11;
  uint32Pair.value = UltrasonicDATA.ks103_data11;
  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
    return false;
  
//  uint32Pair.key = 12;
//  uint32Pair.value = UltrasonicDATA.ks103_data12;
//  if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
//    return false;
      
    return true;
}



static uint32_t GenerateHash(uint32_t seed) 
{
	uint8_t ch[] = "NTM4N2I2YmFiOWIwNzgzYmViYWFjYjc2";
	uint32_t hash = seed;
	for (int i = 0; i < 32; i++) {
		hash ^= ((hash << 5) + ch[i] + (hash >> 2));
	}
	for (int i = 0; i < 32; i++) {
		hash ^= (((hash - ch[i]) >> 2) + (hash << 3));
	}
	return hash;
}

void SetTcpCurrentState(enum State f_state)
{
	current_state = f_state;
}

uint8_t GetMotorCmdState(void)//返回值表示，是否获取到了新的电机控制命令，0=否，1=是
{
	return gs_motorCmdState;
}
void SetMotorCmdState(uint8_t f_newState)
{
	gs_motorCmdState = f_newState;
}
uint8_t GetHasGotCfgParaFlag(void)
{
	return gs_hasGotCfgPara;
}
void SetHasGotCfgParaFlag(uint8_t f_newState)
{
	gs_hasGotCfgPara = f_newState;
}

