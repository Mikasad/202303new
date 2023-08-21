//标准头文件
#include <string.h>
//stm32头文件
#include "stm32f4xx_hal.h"
//APP头文件
#include "sensor.h"
#include "sensor_hub.h"
#include "debug.h"
#include "can_task.h"
#include "iwdg_task.h"


/*
*********************************************************************************************************
*
*	模块名称 : 与sensorHub通信
*	文件名称 : sensor_hub.c
*	版    本 : 	V1.1.1
*	说    明 : 
*			1，读写sensorhub模块信息
*			2，具体的sensorhub的使用可以参考下面的介绍和sensorhub的手册
*			3，注意，不允许在单独的模块中对CAN进行硬件初始化，CAN的硬件初始化放在专门的CAN任务中
*	修改记录 :
*		版本号  		日期        作者     说明
*		V1.0.0    2018-11-05  杨臣   	正式发布
*		V1.0.1    2018-11-26  杨臣	清除了三个DMA串口的接收和解析信息的输出
*		V1.1.0    2018-12-03  杨臣	1，重新梳理了超声波的使能逻辑；2，遥控器的触发状态做了重新梳理；
*									3，各个传感器的使能状态会根据失联而清零
*		V1.1.1    2018-12-04  杨臣  	给sensorhub发送的数据，不能太快，这里每次写入之后都休眠2ms，如果1ms发了两条消息，会有一条被hub忽略
*		V1.1.2	  2018-12-18  杨臣  	1，防跌落和防碰撞默认关闭；2，安全距离的配置做了优化（否则配置失败）：失能距离发送0xFF,使能距离要限制在4~40cm。
*	外设资源占用：
*		无，因为使用的CAN，所以硬件的初始化在CAN任务模块中进行，注意如果多个模块使用一路can，要注意波特率
*	待优化：（需要优化但是未优化之前请保留该提示）
*		1,暂无
*********************************************************************************************************
*/







/*
*********************************************************************************************************
*	sensorHub的功能：
*		1，查询版本
		2，传感器使能
		3，超声使能
		4，遥控使能
		5，释放刹车
		6，配置距离值
	sensorHub的使用方法：
		1，握手（握手，同时1s一次，检查连接）：发送读版本命令，获取到版本信息，则连接正常。
		2，检查遥控器的使能情况,.......
		3，检查超声波是否使能或者失能，然后发送超声波的使能或者失能信息
		4，检查传感器是否失能或者失能，然后发送失能或者失能信息并判断是否掌控刹车，然后发送AA

*	下面时测试出的sensorhub特性
*		波特率 500k
*		1，上电后自动往外发送0x305（超时距离），此时发送0x301 03 03 00  关闭超声波，则0x305不发
*		2，任何时候发送请求版本都会有回应
*		3，任何时候只要发送AA，就会往外发送0x303和0x304
*		4，任何时候发送0x301 03 03 00  关闭超声波，都会导致遥控器数据被关闭
*		5，传感器的配置要注意，使能和距离都配置之后才会发数据，而且不能有冲突
*		6，传感器的操作：1，使能；2，配置距离；配置使能时没有冲突才会往外发送传感器信息
*		7，遥控器使能之后，（必须要保证超声波也使能）才会发送遥控器数据。
*		8，hub的刹车连接在急停上，所以当防跌落或者防碰撞触发时，会同时触发急停。当急停触发而且防跌落或者防碰撞触发时需要释放刹车
*		综上注意，如果遥控器使能则必须打开超声。
*********************************************************************************************************
*/


/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/
//事件标志
#define EVENT_GETVERSION	 			(1 << 0)
#define EVENT_COMPLETE_CFG_ULTRASONIC	(1 << 1)
#define EVENT_GET_ULTRASONIC	EVENT_COMPLETE_CFG_ULTRASONIC
#define EVENT_COMPLETE_CFG_SENSOR	 	(1 << 2)
#define EVENT_GET_SENSOR	EVENT_COMPLETE_CFG_SENSOR
#define EVENT_TELECONTROL_ENABLE 		(1 << 3)
#define EVENT_TELECONTROL_TRIGGER 		(1 << 4)
#define EVENT_CONN_SENSORHUB EVENT_GETVERSION



/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/
TaskHandle_t taskHandleSensorHub = NULL;
//sensorHub版本号
static uint8_t gs_sensorHubVersion[4];
//遥控器的触发状态
static telecontrolTriggerStatus_t gs_telecontrolTriggerStatus;
static uint8_t gs_speedLevel = 20;//根据协议，共100级
static uint8_t gs_telecontrolRawData = 0;//遥控器原始数据，用来反馈给上位机
//遥控器的连接状态
static linkStatus_t gs_telecontrolLinkStatus;
static connectionStatus_t gs_telecontrolConnStatus = {LINK_STATUS_INIT,0,0,0,500}; //设置超时时间为2500ms
//遥控器触发的连接状态
static linkStatus_t gs_telecontrolTriggerLinkStatus;
static connectionStatus_t gs_telecontrolTriggerConnStatus = {LINK_STATUS_INIT,0,0,0,2500}; //设置超时时间为2500ms
//超声波和防跌落传感器的距离信息
static uint8_t gs_ultrasonicDistance[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static uint8_t gs_dropSensorDistance[8];
static uint8_t gs_collisionSensorDistance[8];
//防撞和防跌落安全距离的配置信息
static uint8_t gs_cfgDropSensorDistance[8] = {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10};
static uint8_t gs_cfgCollisionSensorDistance[8] = {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10};
//防跌落和碰撞传感器的触发状态
static uint8_t gs_dropSensorTriggerStatus;
static uint8_t gs_collisionSensorTriggerStatus;
//sensorhub的连接状态
static linkStatus_t gs_sensorHubLinkFlag;//与sensorHub的连接状态
static connectionStatus_t gs_sensorHubConnStatus = {LINK_STATUS_INIT,0,0,0,1000}; //设置超时时间为1000ms

static linkStatus_t gs_dropSensorLinkFlag;//与防跌落传感器的连接状态
static connectionStatus_t gs_dropSensorConnStatus = {LINK_STATUS_INIT,0,0,0,5000}; 

static linkStatus_t gs_collisionSensorLinkFlag;//与防碰撞传感器的连接状态
static connectionStatus_t gs_collisionSensorConnStatus = {LINK_STATUS_INIT,0,0,0,5000};

static linkStatus_t gs_ultrasonicLinkFlag;//与超声波的连接状态
static connectionStatus_t gs_ultrasonicConnStatus = {LINK_STATUS_INIT,0,0,0,200}; 
//对上位机的配置信息处理之后能够发给sensorhub的配置信息，超声，防跌落，碰撞.8位代表8个端口的使能状态
static uint8_t gs_cfgUltrasonicEnable;
static uint8_t gs_cfgDropEnable;
static uint8_t gs_cfgCollisionEnable;
static FunctionalState gs_cfgTelecontrolEnable;
//上位机下发的的配置信息,这里保存这些信息主要是为了把触发状态和这里的配置信息融合后获得触发的是哪个方向的状态
static sensorDirCfg_t gs_ultrasonicCfgPara;
static sensorDirCfg_t gs_antiDropCfgPara;
static sensorDirCfg_t gs_antiCollisionCfgPara;
//当前使能状态,从hub反馈的实时状态
static uint8_t gs_ultrasonicEnableStatus;
static uint8_t gs_dropEnableStatus;
static uint8_t gs_collisionEnableStatus;
static uint8_t gs_telecontrolEnableStatus;
//事件标志，标明是否收到了各种信息
static uint8_t gs_sensorHubEventFlag = 0;



/************************************************************************************************************************************
																静态函数声明
*************************************************************************************************************************************/

//配置
static void ReqSensorHubVersion(void);
static void SetUltrasonicEnable(uint8_t f_cfgEnable);
static void SetSensorEnable(uint8_t f_cfgDropEnable,uint8_t f_cfgCollisionEnable);
static void SetTelecontrolEnable(FunctionalState f_newState);
static void SetSensorHubReleaseBrk(void);
static void SetSensorHubSecurityDistance(uint8_t *fp_distance);
static void CfgSensorHubSensor(void);
//发送数据到sensorhub
static void SendToSensorHub(CanTxMsg_t *fp_canTxMsg);
//接收到数据是调用的函数
static void HandleReceivedDataFromSensorHub_0x303(uint8_t *fp_data);
static int HandleReceivedDataFromSensorHub(CanRxMsg_t f_canRxMes);
static void WriteSensorHubVersion(const uint8_t *fp_data);
//检查连接状态
static void CheckSensorHubLinkState(void);
static void HasConnectedToSensorHub(void);
static void CheckSensorHubStatus(void);

static void CheckUltrasonicLinkState(void);
static void HasConnectedToUltrasonic(void);
static void CheckUltrasonicStatus(void);

static void CheckCollisionSensorLinkState(void);
static void HasConnectedToCollisionSensor(void);
static void CheckDropSensorLinkState(void);
static void HasConnectedToDropSensor(void);
static void CheckDropAndCollisionSensorStatus(void);

static void CheckTelecontrolLinkState(void);
static void HasConnectedToTelecontrol(void);
static void CheckTelecontrolStatus(void);

static void CheckTelecontrolTriggerLinkState(void);
static void HasConnectedToTelecontrolTrigger(void);
static void CheckTelecontrolTriggerStatus(void);



//获取上位机设置的速度级别
static uint8_t GetSpeedLevel(void);
static void GetSensorHubVersion(version_t *fp_version);




/************************************************************************************************************************************
																硬件初始化
*************************************************************************************************************************************/




/************************************************************************************************************************************
																中断处理
*************************************************************************************************************************************/
int SENSOR_CAN_IRQHandler(CanRxMsg_t f_canRxMes)
{
	return HandleReceivedDataFromSensorHub(f_canRxMes);
}



/************************************************************************************************************************************
																内部静态函数：数据接收、解析、处理、控制等
*************************************************************************************************************************************/
static uint8_t GetSpeedLevel(void)
{
	return gs_speedLevel;
}

static void CheckSensorHubLinkState(void)
{
	gs_sensorHubLinkFlag = CheckLinkState(&gs_sensorHubConnStatus);
}
static void HasConnectedToSensorHub(void)
{
	ConnectedDev(&gs_sensorHubConnStatus);
}
static void CheckSensorHubStatus(void)
{
	if((gs_sensorHubEventFlag & EVENT_GETVERSION) != 0)
	{
		HasConnectedToSensorHub();
		//清标记
		gs_sensorHubEventFlag &= (~EVENT_GETVERSION);
	}
	//检查当前的连接状态
	CheckSensorHubLinkState();
	//如果不处于连接状态则清除使能状态
	if(gs_sensorHubLinkFlag != LINK_STATUS_LINKED)
	{
		gs_ultrasonicEnableStatus = 0;
		gs_dropEnableStatus = 0;
		gs_collisionEnableStatus = 0;
		gs_telecontrolEnableStatus = 0;

	}
}


static void CheckUltrasonicLinkState(void)
{
	gs_ultrasonicLinkFlag = CheckLinkState(&gs_ultrasonicConnStatus);
}
static void HasConnectedToUltrasonic(void)
{
	ConnectedDev(&gs_ultrasonicConnStatus);
}
static void CheckUltrasonicStatus(void)
{
	if((gs_sensorHubEventFlag & EVENT_GET_ULTRASONIC) != 0)
	{
		HasConnectedToUltrasonic();
		//清标记
		gs_sensorHubEventFlag &= (~EVENT_GET_ULTRASONIC);
	}
	//检查当前的连接状态
	CheckUltrasonicLinkState();
	//如果不处于连接状态则清除使能状态
	if(gs_ultrasonicLinkFlag != LINK_STATUS_LINKED)
		gs_ultrasonicEnableStatus = 0;
}

static void CheckCollisionSensorLinkState(void)
{
	gs_collisionSensorLinkFlag = CheckLinkState(&gs_collisionSensorConnStatus);
}
static void HasConnectedToCollisionSensor(void)
{
	ConnectedDev(&gs_collisionSensorConnStatus);
}
static void CheckDropSensorLinkState(void)
{
	gs_dropSensorLinkFlag = CheckLinkState(&gs_dropSensorConnStatus);
}
static void HasConnectedToDropSensor(void)
{
	ConnectedDev(&gs_dropSensorConnStatus);
}
static void CheckDropAndCollisionSensorStatus(void)
{
	if((gs_sensorHubEventFlag & EVENT_GET_SENSOR) != 0)
	{
		HasConnectedToDropSensor();
		HasConnectedToCollisionSensor();
		//清标记
		gs_sensorHubEventFlag &= (~EVENT_GET_SENSOR);
	}
	//检查当前的连接状态
	CheckDropSensorLinkState();
	CheckCollisionSensorLinkState();
	//如果不处于连接状态则清除使能状态
	if(gs_dropSensorLinkFlag != LINK_STATUS_LINKED)
		gs_dropEnableStatus = 0;
		
	if(gs_collisionSensorLinkFlag != LINK_STATUS_LINKED)
		gs_collisionEnableStatus = 0;
}


static void CheckTelecontrolLinkState(void)
{
	gs_telecontrolLinkStatus = CheckLinkState(&gs_telecontrolConnStatus);
}
static void HasConnectedToTelecontrol(void)
{
	ConnectedDev(&gs_telecontrolConnStatus);
}
static void CheckTelecontrolStatus(void)
{
	if((gs_sensorHubEventFlag & EVENT_TELECONTROL_ENABLE) != 0)
	{
		HasConnectedToTelecontrol();
		//清标记
		gs_sensorHubEventFlag &= (~EVENT_TELECONTROL_ENABLE);
	}
	//检查当前的连接状态
	CheckTelecontrolLinkState();
	//如果不处于连接状态则清除使能状态
	if(gs_telecontrolLinkStatus != LINK_STATUS_LINKED)
		gs_telecontrolEnableStatus = 0;
}

static void CheckTelecontrolTriggerLinkState(void)
{
	gs_telecontrolTriggerLinkStatus = CheckLinkState(&gs_telecontrolTriggerConnStatus);
}
static void HasConnectedToTelecontrolTrigger(void)
{
	ConnectedDev(&gs_telecontrolTriggerConnStatus);
}
static void CheckTelecontrolTriggerStatus(void)
{
	if((gs_sensorHubEventFlag & EVENT_TELECONTROL_TRIGGER) != 0)
	{
		HasConnectedToTelecontrolTrigger();
		//清标记
		gs_sensorHubEventFlag &= (~EVENT_TELECONTROL_TRIGGER);
	}
	//检查当前的连接状态
	CheckTelecontrolTriggerLinkState();
}




static void WriteSensorHubVersion(const uint8_t *fp_data)
{
	memcpy(gs_sensorHubVersion,fp_data,4);
}
static void GetSensorHubVersion(version_t *fp_version)
{
	if((gs_sensorHubEventFlag & EVENT_GETVERSION) == 0)//这个标记在获得版本信息后，每隔1s清楚一次然后再去读取
		return;
	fp_version->type = gs_sensorHubVersion[0];
	fp_version->major = gs_sensorHubVersion[1];
	fp_version->minor = gs_sensorHubVersion[2];
	fp_version->build = gs_sensorHubVersion[3];
}

/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 处理来自sensorHub的数据,
*	形    参: 
*	返 回 值:1=表示ID是自己期望的，0=id不是自己期望的。
*********************************************************************************************************
*/
static int HandleReceivedDataFromSensorHub(CanRxMsg_t f_canRxMes)
{
	int i = 0;
	int ret = 1;
    (void)gs_collisionSensorDistance;
    (void)gs_dropSensorDistance;
	switch(f_canRxMes.head.StdId)
	{
		case 0x303://版本号、遥控的触发状态、传感器的触发状态
			HandleReceivedDataFromSensorHub_0x303(f_canRxMes.Data);
		break;
		case 0x304://传感器距离
			for(i=0;i<8;i++)
			{
				if(((gs_cfgDropEnable >> i) & 0x01) != 0)//说明i端口使能了需要配置距离
					gs_dropSensorDistance[i] = f_canRxMes.Data[i];
				else if(((gs_cfgCollisionEnable >> i) & 0x01) != 0)
					gs_collisionSensorDistance[i] = f_canRxMes.Data[i];	
			}
			gs_sensorHubEventFlag |= EVENT_COMPLETE_CFG_SENSOR;
			gs_sensorHubEventFlag |= EVENT_GET_SENSOR;
		break;
		case 0x305://超声波距离
			for(i=0;i<8;i++)
			{
				gs_ultrasonicDistance[i] = f_canRxMes.Data[i];
			}
			gs_ultrasonicEnableStatus = gs_cfgUltrasonicEnable;//因为不返回超声波使能信息，故这里接收到了就认为使能与配置相同
			gs_sensorHubEventFlag |= EVENT_COMPLETE_CFG_ULTRASONIC;
			gs_sensorHubEventFlag |= EVENT_GET_ULTRASONIC;
		break;
		default:
			ret = 0;
		break;
	}
	return ret;
}

/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 处理来自sensorHub的0x303数据,
*	形    参: 
*	返 回 值:
*********************************************************************************************************
*/
static void HandleReceivedDataFromSensorHub_0x303(uint8_t *fp_data)
{
	if(fp_data[0] != 0x03)
		;//DBGOut("error!\n");
	else
	{
		switch(fp_data[1])
		{
			case 0x01://版本信息
				WriteSensorHubVersion(fp_data);
				//设置获取到版本的标志
				gs_sensorHubEventFlag |= EVENT_GETVERSION;
			break;
			case 0x02://遥控器,bit3~0:右左后前
				gs_telecontrolEnableStatus = 1;//遥控已使能
				gs_telecontrolRawData = fp_data[2];//遥控器原始数据，用来反馈给上位机
				gs_telecontrolTriggerStatus.dir.front = (fp_data[2] >> 0) & 0x01;
				gs_telecontrolTriggerStatus.dir.back = (fp_data[2] >> 1) & 0x01;
				gs_telecontrolTriggerStatus.dir.left = (fp_data[2] >> 2) & 0x01;
				gs_telecontrolTriggerStatus.dir.right = (fp_data[2] >> 3) & 0x01;
				//设置遥控器触发标志
				gs_sensorHubEventFlag |= EVENT_TELECONTROL_ENABLE;
				if(gs_telecontrolTriggerStatus.allBits != 0)
					gs_sensorHubEventFlag |= EVENT_TELECONTROL_TRIGGER;
			break;
			case 0x03:
				gs_dropEnableStatus = fp_data[2];
				gs_collisionEnableStatus = fp_data[3];
				
				gs_dropSensorTriggerStatus = fp_data[4] & gs_dropEnableStatus;
				gs_collisionSensorTriggerStatus = fp_data[4] & gs_collisionEnableStatus;
				gs_sensorHubEventFlag |= EVENT_COMPLETE_CFG_SENSOR;
				gs_sensorHubEventFlag |= EVENT_GET_SENSOR;
			default:
			break;
		}
	}
}


/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: sensorHub的配置
*	形    参: 
*	返 回 值:
*********************************************************************************************************
*/
static void ReqSensorHubVersion(void)
{
	CanTxMsg_t canTxMsg = {0};
	
	canTxMsg.head.StdId = 0x301;
	canTxMsg.head.RTR = CAN_RTR_DATA;
	canTxMsg.head.IDE = CAN_ID_STD;
	canTxMsg.head.DLC = 2;
	
	canTxMsg.Data[0] = 0x03;
	canTxMsg.Data[1] = 0x01;
	
	SendToSensorHub(&canTxMsg);
}
static void SetUltrasonicEnable(uint8_t f_cfgEnable)
{
	CanTxMsg_t canTxMsg = {0};
	
	canTxMsg.head.StdId = 0x301;
	canTxMsg.head.RTR = CAN_RTR_DATA;
	canTxMsg.head.IDE = CAN_ID_STD;
	canTxMsg.head.DLC = 3;
	
	canTxMsg.Data[0] = 0x03;
	canTxMsg.Data[1] = 0x03;
	
	canTxMsg.Data[2] = f_cfgEnable;
	
	SendToSensorHub(&canTxMsg);
	
}

static void SetSensorEnable(uint8_t f_cfgDropEnable,uint8_t f_cfgCollisionEnable)
{
	CanTxMsg_t canTxMsg = {0};
	
	canTxMsg.head.StdId = 0x301;
	canTxMsg.head.RTR = CAN_RTR_DATA;
	canTxMsg.head.IDE = CAN_ID_STD;
	canTxMsg.head.DLC = 4;
	
	canTxMsg.Data[0] = 0x03;
	canTxMsg.Data[1] = 0x02;
	
	canTxMsg.Data[2] = f_cfgDropEnable;
	
	canTxMsg.Data[3] = f_cfgCollisionEnable;
	
	SendToSensorHub(&canTxMsg);
	
}

static void SetTelecontrolEnable(FunctionalState f_newState)
{
	
	CanTxMsg_t canTxMsg = {0};
	
	canTxMsg.head.StdId = 0x301;
	canTxMsg.head.RTR = CAN_RTR_DATA;
	canTxMsg.head.IDE = CAN_ID_STD;
	canTxMsg.head.DLC = 3;
	
	canTxMsg.Data[0] = 0x03;
	canTxMsg.Data[1] = 0x04;
	canTxMsg.Data[2] = f_newState;
	
	SendToSensorHub(&canTxMsg);
}

static void SetSensorHubReleaseBrk(void)
{
	
	CanTxMsg_t canTxMsg = {0};
	
	canTxMsg.head.StdId = 0x301;
	canTxMsg.head.RTR = CAN_RTR_DATA;
	canTxMsg.head.IDE = CAN_ID_STD;
	canTxMsg.head.DLC = 2;
	
	canTxMsg.Data[0] = 0x03;
	canTxMsg.Data[1] = 0xAA;
	
	SendToSensorHub(&canTxMsg);
}

static void SetSensorHubSecurityDistance(uint8_t *fp_distance)
{
	
	CanTxMsg_t canTxMsg = {0};
	
	canTxMsg.head.StdId = 0x302;
	canTxMsg.head.RTR = CAN_RTR_DATA;
	canTxMsg.head.IDE = CAN_ID_STD;
	canTxMsg.head.DLC = 8;
	
	memcpy(canTxMsg.Data,fp_distance,8);
	SendToSensorHub(&canTxMsg);
}
//配置使能和距离
static void CfgSensorHubSensor(void)
{
	//注意，如果失能，则距离必须配置为0xFF,否则失败。如果使能，则有效的距离配置值是4~40cm
	uint8_t cfgDistance[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	uint32_t dist = 0;
	int i=0;
	//1，配置使能
	SetSensorEnable(gs_cfgDropEnable,gs_cfgCollisionEnable);
	//2，配置距离
	for(i=0;i<8;i++)//总共8位，使能情况是由上位机配置，要注意不要配置冲突了
	{
		if(((gs_cfgDropEnable >> i) & 0x01) != 0)//说明i端口使能了需要配置距离
		{
			if(gs_cfgDropSensorDistance[i] > 40)
				dist = 40;
			else if(gs_cfgDropSensorDistance[i] < 4)
				dist = 4;
			else
				dist = gs_cfgDropSensorDistance[i];
			
			cfgDistance[i] = dist;
		}
		else if(((gs_cfgCollisionEnable >> i) & 0x01) != 0)//说明i端口使能了需要配置距离
		{
			if(gs_cfgCollisionSensorDistance[i] > 40)
				dist = 40;
			else if(gs_cfgCollisionSensorDistance[i] < 4)
				dist = 4;
			else
				dist = gs_cfgCollisionSensorDistance[i];
			
			cfgDistance[i] = dist;
		}
	}
	SetSensorHubSecurityDistance(cfgDistance);
	
}



/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 发送数据到sensorhub，写入can缓存区
*	形    参: 
*	返 回 值:
*********************************************************************************************************
*/
static void SendToSensorHub(CanTxMsg_t *fp_canTxMsg)
{
	WriteCan1TxBuf(*fp_canTxMsg);
	vTaskDelay(pdMS_TO_TICKS(2)); // 发送到sensorhub的频率不能太高，否则sensorhub会不响应
}



/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口定义
*************************************************************************************************************************************/
void SetSpeedLevel(uint8_t f_speedLevel)
{
	gs_speedLevel = f_speedLevel;
}
//获取遥控器发送的速度，这里只是通过上位机设置的速度级别以及遥控器的触发方向来计算的
void GetManualSpeed(int *fp_leftSpeed,int *fp_rightSpeed)
{
	uint8_t speedLevel = GetSpeedLevel();
	const uint16_t maxSpeed = 1000;
	//1，根据级别获取速度绝对值
	int32_t speed = speedLevel * (maxSpeed/100);//因为级别最大100
	
	if(gs_telecontrolTriggerStatus.dir.front)
	{
		*fp_leftSpeed = speed;
		*fp_rightSpeed = speed;
	}
	else if(gs_telecontrolTriggerStatus.dir.back)
	{
		*fp_leftSpeed = -speed;
		*fp_rightSpeed = -speed;
	}
	else if(gs_telecontrolTriggerStatus.dir.left)
	{
		*fp_leftSpeed = -speed;
		*fp_rightSpeed = speed;
	}
	else if(gs_telecontrolTriggerStatus.dir.right)
	{
		*fp_leftSpeed = speed;
		*fp_rightSpeed = -speed;
	}
	else
	{
		*fp_leftSpeed = 0;
		*fp_rightSpeed = 0;
	}
}

linkStatus_t GetSensorHubLinkState(void)
{
	return gs_sensorHubLinkFlag;
}
linkStatus_t GetTelecontrolLinkState(void)
{
	return gs_telecontrolLinkStatus;
}
linkStatus_t GetTelecontrolTriggerLinkState(void)
{
	return gs_telecontrolTriggerLinkStatus;
}

void ReleaseBrkOfSensorHub(void)
{
	SetSensorHubReleaseBrk();
}
__weak linkStatus_t GetUltrasonicLinkState(void)
{
	return gs_ultrasonicLinkFlag;
}
__weak linkStatus_t GetAntidropLinkState(void)
{
	return gs_dropSensorLinkFlag;
}
__weak linkStatus_t GetAntiCollisionLinkState(void)
{
	return gs_collisionSensorLinkFlag;
}
__weak void GetUltrasonicVersion(version_t *fp_version)
{
	GetSensorHubVersion(fp_version);
}
__weak void GetAntidropVersion(version_t *fp_version)
{
	GetSensorHubVersion(fp_version);
}
__weak void GetAntiCollisionVersion(version_t *fp_version)
{
	GetSensorHubVersion(fp_version);
}
__weak uint8_t GetTelecontrolRawData(void)
{
	return gs_telecontrolRawData;
}
__weak uint8_t GetUltrasonicHealthState(void)
{
	uint8_t ret = 0;//1=健康
	ret = (gs_ultrasonicLinkFlag == LINK_STATUS_LINKED)?1:0;
	return ret;
}
__weak void GetUltrasonicDistance(uint32_t * fp_dest)
{
	int i = 0;
	for(i=0;i<8;i++)
		fp_dest[i] = gs_ultrasonicDistance[i];
}

__weak void GetUltrasonicCfgPara(sensorDirCfg_t f_sensorDirCfg, uint32_t f_enableCfg)
{
	gs_ultrasonicCfgPara = f_sensorDirCfg;

	if(f_enableCfg == 0)
		gs_cfgUltrasonicEnable = 0;
	else
		gs_cfgUltrasonicEnable =   gs_ultrasonicCfgPara.front \
								| gs_ultrasonicCfgPara.back \
								| gs_ultrasonicCfgPara.left \
								| gs_ultrasonicCfgPara.right \
								| gs_ultrasonicCfgPara.leftFront \
								| gs_ultrasonicCfgPara.rightFront \
								| gs_ultrasonicCfgPara.leftBack \
								| gs_ultrasonicCfgPara.rightBack;
}
__weak void GetAntiDropCfgPara(sensorDirCfg_t f_sensorDirCfg,uint32_t *fp_threshold)
{
	int i = 0;
	
	gs_antiDropCfgPara = f_sensorDirCfg;

	gs_cfgDropEnable =   gs_antiDropCfgPara.front \
						| gs_antiDropCfgPara.back \
						| gs_antiDropCfgPara.left \
						| gs_antiDropCfgPara.right \
						| gs_antiDropCfgPara.leftFront \
						| gs_antiDropCfgPara.rightFront \
						| gs_antiDropCfgPara.leftBack \
						| gs_antiDropCfgPara.rightBack;
	for(i=0;i<8;i++)
	{
		if(fp_threshold[i] != 0)
			gs_cfgDropSensorDistance[i] = fp_threshold[i];
		else
			gs_cfgDropSensorDistance[i] = 0xFF;
	}
}
__weak void GetAntiCollisionCfgPara(sensorDirCfg_t f_sensorDirCfg,uint32_t *fp_threshold)
{
	int i = 0;
	
	gs_antiCollisionCfgPara = f_sensorDirCfg;
	
	gs_cfgCollisionEnable =   gs_antiCollisionCfgPara.front \
							| gs_antiCollisionCfgPara.back \
							| gs_antiCollisionCfgPara.left \
							| gs_antiCollisionCfgPara.right \
							| gs_antiCollisionCfgPara.leftFront \
							| gs_antiCollisionCfgPara.rightFront \
							| gs_antiCollisionCfgPara.leftBack \
							| gs_antiCollisionCfgPara.rightBack;
	for(i=0;i<8;i++)
	{
		if(fp_threshold[i] != 0)
			gs_cfgCollisionSensorDistance[i] = fp_threshold[i];
		else
			gs_cfgCollisionSensorDistance[i] = 0xFF;
	}
}

/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明:通过传感器的触发状态获取禁止运行的方向
*	形    参: 
*	返 回 值:
*********************************************************************************************************
*/
__weak uint32_t CheckForbidMoveDirByDrop(void)
{
	safetySensorTriggerStatus_t safetySensorTriggerStatus = {0};

//通过比较触发的bit和配置的方向的bit是否重合来判断该方向上是否被触发
	safetySensorTriggerStatus.dir.front = (gs_dropSensorTriggerStatus & gs_antiDropCfgPara.front) != 0;
	safetySensorTriggerStatus.dir.back = (gs_dropSensorTriggerStatus & gs_antiDropCfgPara.back) != 0;
	safetySensorTriggerStatus.dir.left = (gs_dropSensorTriggerStatus & gs_antiDropCfgPara.back) != 0;
	safetySensorTriggerStatus.dir.right = (gs_dropSensorTriggerStatus & gs_antiDropCfgPara.right) != 0;
	safetySensorTriggerStatus.dir.leftFront = (gs_dropSensorTriggerStatus & gs_antiDropCfgPara.leftFront) != 0;
	safetySensorTriggerStatus.dir.rightFront = (gs_dropSensorTriggerStatus & gs_antiDropCfgPara.rightFront) != 0;
	safetySensorTriggerStatus.dir.leftBack = (gs_dropSensorTriggerStatus & gs_antiDropCfgPara.leftBack) != 0;
	safetySensorTriggerStatus.dir.rightBack = (gs_dropSensorTriggerStatus & gs_antiDropCfgPara.rightBack) != 0;

	return CheckForbidMoveDirBySafetySensor(safetySensorTriggerStatus);
}
__weak uint32_t CheckForbidMoveDirByCollision(void)
{
	safetySensorTriggerStatus_t safetySensorTriggerStatus = {0};
	
	//通过比较触发的bit和配置的方向的bit是否重合来判断该方向上是否被触发
	safetySensorTriggerStatus.dir.front = (gs_collisionSensorTriggerStatus & gs_antiCollisionCfgPara.front) != 0;
	safetySensorTriggerStatus.dir.back = (gs_collisionSensorTriggerStatus & gs_antiCollisionCfgPara.back) != 0;
	safetySensorTriggerStatus.dir.left = (gs_collisionSensorTriggerStatus & gs_antiCollisionCfgPara.back) != 0;
	safetySensorTriggerStatus.dir.right = (gs_collisionSensorTriggerStatus & gs_antiCollisionCfgPara.right) != 0;
	safetySensorTriggerStatus.dir.leftFront = (gs_collisionSensorTriggerStatus & gs_antiCollisionCfgPara.leftFront) != 0;
	safetySensorTriggerStatus.dir.rightFront = (gs_collisionSensorTriggerStatus & gs_antiCollisionCfgPara.rightFront) != 0;
	safetySensorTriggerStatus.dir.leftBack = (gs_collisionSensorTriggerStatus & gs_antiCollisionCfgPara.leftBack) != 0;
	safetySensorTriggerStatus.dir.rightBack = (gs_collisionSensorTriggerStatus & gs_antiCollisionCfgPara.rightBack) != 0;
	
	return CheckForbidMoveDirBySafetySensor(safetySensorTriggerStatus);
}







/************************************************************************************************************************************
												任务+对外的硬件初始化+对外的软件初始化
*************************************************************************************************************************************/


/*
*********************************************************************************************************
*	函 数 名: SensorHubTask
*	功能说明: 使用sensorHub的任务	
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 高 
*********************************************************************************************************
*/
void SensorHubTask(void *pvParameters)
{
	TickType_t lastWakeTime;
	TickType_t reqVerTime = 0;//版本请求定时
	TickType_t checkLinkTime = 0;//检查连接情况定时
	TickType_t cfgTime = 0;//配置定时
	const TickType_t frequency = pdMS_TO_TICKS(1);
	uint32_t bitPos = (uint32_t)pvParameters;
    uint8_t thread_num = (uint8_t)(uint32_t)pvParameters;
    
	/* 获取当前的系统时间 */
    lastWakeTime = xTaskGetTickCount();

    while(1)
    {
		/* 喂狗 */
		FeedDog(bitPos);
		/* vTaskDelayUntil是绝对延迟，vTaskDelay是相对延迟。*/
        vTaskDelayUntil(&lastWakeTime, frequency);

		//1，请求版本信息,频率是10hz
		if(reqVerTime < xTaskGetTickCount())
		{
			//发送请求信息
			ReqSensorHubVersion();
			//更新时间
			reqVerTime += pdMS_TO_TICKS(100);
		}
		//检查各个部分的连接情况
		if(checkLinkTime < xTaskGetTickCount())
		{
			CheckSensorHubStatus();
			CheckUltrasonicStatus();
			CheckDropAndCollisionSensorStatus();
			CheckTelecontrolStatus();
			CheckTelecontrolTriggerStatus();
			//更新时间
			checkLinkTime += pdMS_TO_TICKS(50);
		}
		//检测配置是否成功，并进行配置，
		if(cfgTime < xTaskGetTickCount())
		{
			//遥控器使能/失能配置，该配置必须在使能超声波之后才会有效
			if(gs_telecontrolEnableStatus != gs_cfgTelecontrolEnable)
			{
				SetTelecontrolEnable(gs_cfgTelecontrolEnable);
			}
			
			//超声波使能/失能配置,如果遥控器使能则超声波必须使能
			if(gs_cfgTelecontrolEnable == ENABLE)
			{
				if(gs_ultrasonicEnableStatus == 0x00)
					SetUltrasonicEnable(0xFF);
			}
			else
			{
				if(gs_ultrasonicEnableStatus != gs_cfgUltrasonicEnable)
				{
					SetUltrasonicEnable(gs_cfgUltrasonicEnable);
				}
			}
			//传感器使能/失能、距离设置
			if(gs_dropEnableStatus!= gs_cfgDropEnable ||  gs_collisionEnableStatus != gs_cfgCollisionEnable)
			{
				CfgSensorHubSensor();
			}	
			//更新时间
			cfgTime += pdMS_TO_TICKS(1000);
		}
        
        g_thread_call_count[thread_num]++;
	}	
}



//0=成功
int InitHardwareSensorHub(void)
{
	return 0;
}

int InitSoftwareSensorHub(void)
{
	gs_cfgUltrasonicEnable = 0xFF;//超声波默认使能
	gs_cfgDropEnable = 0x00;//防跌落默认关闭
	gs_cfgCollisionEnable = 0x00;//防碰撞默认关闭
	gs_cfgTelecontrolEnable = ENABLE;//遥控器默认使能
	return 0;
}

