#ifndef __CANOPEN_OD_H__
#define __CANOPEN_OD_H__
#include "stm32f4xx_hal.h"
#include "canopen_def.h"
#include "objectdef.h"
#include "timerscfg.h"
//TODO:add "include "canopen_nmt.h" here will record 56 Errors - Identifier "CANopen_Status" is undefined"

/* MACRO DEFINE BEGIN */
#define _OBJ_DECLARE
#define _OBJ_DS301_DECLARE
#define _OBJ_MANUFACTURER
#define _OBJ_DS402_DECLARE

#define _ATLAS_DEBUG_EN 
//#include "canopen_sdo.h"
#ifdef _ATLAS_DEBUG_EN

#define ATLAS_ERR(...) printf("-ERR-  "__VA_ARGS__)
#define ATLAS_PRINT(...) printf("-ATLAS-  "__VA_ARGS__)
#define ATELAS_MAR(str,index,subindex)   ATLAS_PRINT(str);\
                              printf("   Index = %d",index);\
                              printf("   Subndex = %d",subindex);\
                              printf("\r\n");

#define MANUFACTURER_ADD_NUM    33
#endif
/* MACRO DEFINE END */

/*OD_Structure_Define*/
//OD Sub index struct
typedef struct pObjDictry_STRUCT
{
  unsigned short int Index;   //索引(16bit)
  unsigned char subIndex;     //子索引(8bit)
  unsigned char accessType;   //访问权限
  unsigned char dataType;     //数据类型
  unsigned int size;          //当前索引数据类型占用空间
  unsigned char pdoMap;       //此对象是否可以映射(PDO)
  
  void*  lpParam;             //参数指针
}index_pObjDictry;
//OD index struct
typedef struct pObjDictry_index_STRUCT
{
  index_pObjDictry *pSubindex;//子索引
  unsigned char bSubcount;    //子索引数
  unsigned short int index;   //索引号
}indextable;
//last obj
typedef struct pLastObj_STRUCT
{
  unsigned short int Index;
  unsigned char SubIndex;
  unsigned int size;
  unsigned char dataType;
  unsigned char Data[8]; 
}pLastObj_Status;  
//Time Consume test
typedef struct pTime_Consume_STRUCT
{
  unsigned int RPDO;
  unsigned int TPDO;
  unsigned int SDO;
  unsigned int CHOP;
}pTime_Consume_Status;
extern pTime_Consume_Status TimeConsume;
//CAN Load rate test
typedef struct pCanLoadRate_STRUCT
{
  unsigned int timer;
  unsigned int FrameSize;
  unsigned int Cycle;
  float Rate;
}pCanLoadRate_Status;
extern pCanLoadRate_Status CanLoadRate;

/* TYPEDEF DEFINE BEGIN */
/* Typedef Struct Name Define */
typedef struct CANopen_STRUCT CO_Data;
typedef struct struct_s_transfer s_transfer;
typedef struct struct_s_PDO_status s_PDO_status;
/* Typedef Function Define */
//对象字典子索引回调函数
typedef unsigned int (*ODCallback_t)(CO_Data *d, const indextable *, unsigned char bSubindex);
//按索引查字典
//函数指针：类型定义一个名为scanIndexOD_t的函数指针，指向形参为(unsigned short int wIndex, unsigned int *errorCode, ODCallback_t **Callback)的函数，返回值为indextable*指针
typedef const indextable *(*scanIndexOD_t)(unsigned short int wIndex, unsigned int *errorCode, ODCallback_t **Callback);
//对各数据类型的范围进行检查合法性检查
typedef unsigned int (*valueRangeTest_t)(unsigned char typeValue, void *Value);
//可保存参数修改回调函数
typedef  void (*storeODSubIndex_t)(CO_Data* d, unsigned short int wIndex, unsigned char bSubindex);
//接收到同步报文回调函数
typedef void (*post_sync_t)(CO_Data*);
//TPDO报文回调函数
typedef void (*post_TPDO_t)(CO_Data*);
//NMT states change callback
typedef void (*initialisation_t)(CO_Data*);
typedef void (*preOperational_t)(CO_Data*);
typedef void (*operational_t)(CO_Data*);
typedef void (*stopped_t)(CO_Data*);
//HeartBeat ERR Callback
typedef void (*heartbeatError_t)(CO_Data*, unsigned char);
//Monitored Node State Change Callback
typedef void (*post_SlaveStateChange_t)(CO_Data*,unsigned char,unsigned char);
/* Extern Function Define */
extern const indextable *ObjDict_scanIndexOD(unsigned short int wIndex, unsigned int *errorCode, ODCallback_t **callbacks);
extern unsigned int ObjDict_Get_Id(unsigned short int wIndex, unsigned int *ObjId);
extern unsigned int ObjDict_valueRangeTest(unsigned char typeValue, void *value);
extern void _storeODSubIndex(CO_Data* d, unsigned short int wIndex, unsigned char bSubindex);
extern void CANopen_Parameter_Init(CO_Data* d);


/* STRCUT DEFINE BEGIN */
/*CANopen ID Structure_Define*/
typedef struct CANopenID_STRUCT
{
  unsigned int inNum_max;
  unsigned int VendorID;
  unsigned int ProductCode;
  unsigned int RevisionNumber;
  unsigned int SerialNumber;
}pCANopen_ID_Status;
extern pCANopen_ID_Status  Ob_ID;  //CANopen对象身份
/*Time of day Structure_Define*/
typedef struct Time_of_Date_STRUCT
{
  unsigned int minisec;   //午夜起算毫秒计数
  unsigned short int days;//自1984年1月1日以来的天数
}pDate_Status;
/*TPDO_Statsu & RPDO_Statsu Structure_Define*/
//PDO状态机处理结构体
struct struct_s_PDO_status {
  unsigned char transmit_type_parameter;
  TIMER_HANDLE event_timer;
  TIMER_HANDLE inhibit_timer;
  CanTxMsgTypeDef last_message;
};
//PDO直接传输结构体
typedef struct PDO_DrctTrans_status
{
  unsigned char transmit_type_parameter;
  unsigned char TransStatus;
  unsigned char active;
  unsigned char skip[8];//DataSize
  unsigned int Pointer[2][8];//ID SUBINDEX
  CanRxMsgTypeDef lastmessage;
}PDO_DrctTrans_Struct;
extern PDO_DrctTrans_Struct pRpdoDirectPar[4];
//PDO映射参数结构体
typedef struct  PDO_MAP_STRUCT
{
  unsigned char inNum;                        //PDO映射对象数
  unsigned int  Index[PDO_MAP_NUMBER_MAX][1]; //[0]取地址(Index(16bit)  subIndex(8bit)  obj_lenth(8bit)) [1]每个地址对应的对象字典具体地址
}PDO_Map_Status;
//TPDO通信参数子结构体
typedef struct TPDO_STRUCT
{
  unsigned char inNum;          //入口数目-此索引下有效参数个数
  unsigned char tSync_Num;      //同步报文计数器，在同步传输下来和传输类型比较
  unsigned int  tPDO_ID;        //PDO标识符(32bit)
  unsigned char last_TransType; //上一次的传输类型（0-255）
  short int event_time;         //触发事件(异步发送时，如果该时间不为0，则会以此时间为周期进行周期发送)
  short int inhibit_time;       //禁止时间(PDO报文传输时的最小时间间隔，防止PDO报文过度占用总线)
  unsigned char reserved;       //保留
  unsigned int telectrlSign;    //远程帧标志
  CanTxMsgTypeDef Message;      //上一条TPDO报文，用来确定映射参数的值有无变化
}pTPDO_Status;

//RPDO通信参数子结构体
typedef struct RPDO_STRUCT
{
  unsigned char inNum;          //入口数目-此索引下有效参数个数
  unsigned char rSync_Num;      //同步报文计数器，在同步传输下来和传输类型比较
  unsigned int  rPDO_ID;        //PDO标识符(32bit)
  unsigned char last_TransType; //上一次的传输类型（0-255）
  short int event_time;         //触发事件(异步发送时，如果该时间不为0，则会以此时间为周期进行周期发送)
  short int inhibit_time;       //禁止时间(PDO报文传输时的最小时间间隔，防止PDO报文过度占用总线)
  unsigned char reserved;       //保留
  unsigned int telectrlSign;    //远程帧标志
  CanRxMsgTypeDef Message;      //上一条RPDO报文
}pRPDO_Status; 
//TPDO通信参数结构体
typedef struct PDO_COMMUNICATE_STRUCT
{
  pTPDO_Status TPDO1;
  pTPDO_Status TPDO2;
  pTPDO_Status TPDO3;
  pTPDO_Status TPDO4;
  
  pRPDO_Status RPDO1;
  pRPDO_Status RPDO2;
  pRPDO_Status RPDO3;
  pRPDO_Status RPDO4;
}PDO_COMMUNICATE_Status;
//TDPO映射参数结构体
typedef struct PDO_MAP_ALL_STRUCT
{
  PDO_Map_Status TPDO1;
  PDO_Map_Status TPDO2;
  PDO_Map_Status TPDO3;
  PDO_Map_Status TPDO4;
  
  PDO_Map_Status RPDO1;
  PDO_Map_Status RPDO2;
  PDO_Map_Status RPDO3;
  PDO_Map_Status RPDO4;
}PDO_MAP_ALL_Status;
//PDO传输结构体
typedef struct PDO_TRANS_STRUCT
{
  PDO_COMMUNICATE_Status pPdoComunictPar;   //PDO通信参数
  PDO_MAP_ALL_Status pPdoMapPar;            //PDO映射参数
}PDO_TRANS_Status;
extern PDO_TRANS_Status  pPdoPar;           //PDO传输结构体
/*SDO Stucture_Define*/
typedef struct SDO_STRUCT
{
  unsigned char inNum;          //入口数目-SDO支持的参数数量，服务器SDO取值：索引1200h:02h  索引1201h-127Fh:02h-03h  
  unsigned int  RSDO_COB_ID;    //从站接收COB-ID 服务器SDO取值：索引1200h(CAN-ID:600h+Node-ID  frame:0b  dyn:0b  valid:0b)  索引1201h-127Fh(CAN-ID:制造商指定  frame:制造商指定  dyn:0b  valid:1b或由应用协议定义)
  unsigned int  TSDO_COB_ID;    //从站发送COB-ID 服务器SDO取值：索引1200h(CAN-ID:580h+Node-ID  frame:0b  dyn:0b  valid:0b)  索引1201h-127Fh(CAN-ID:制造商指定  frame:制造商指定  dyn:0b  valid:1b或由应用协议定义)
  unsigned char nodeId;         //SDO客户端应答服务器的Node-ID 服务器SDO取值：01h-7Fh
}pSDO_Status;                   //客户端取值：制造商指定
//SDO trans struct
typedef enum 
{
	RXSTEP_INIT,		//接收初始化
	RXSTEP_STARTED, //接收开始
	RXSTEP_END			//接收结束
}rxStep_t;
typedef void (*SDOCallback_t)(CO_Data* d, unsigned char nodeId);
//SDO trans channel struct
struct struct_s_transfer 
{
	unsigned char 					CliServNbr;		  //客户端/服务器号
	unsigned char						whoami;					//属性客户端/服务器
	unsigned char						state;					//状态
  unsigned char						toggle;					//触发位
  unsigned int						abortCode;			//错误码
  unsigned short int						index;		//对象索引
  unsigned char						subIndex;				//对象子索引
  unsigned int						count;					//字节数
  unsigned int						offset;					//偏移量
  unsigned char						data[SDO_MAX_LENGTH_TRANSFER];	//数据缓冲区
#ifdef SDO_DYNAMIC_BUFFER_ALLOCATION
  unsigned char						*dynamicData;		//动态缓冲区指针
  unsigned int						dynamicDataSize;//动态缓冲区大小
#endif                                    
  unsigned char						peerCRCsupport;	//是否支持crc校验
  unsigned char						blksize;				//一次最多传输的块数量
  unsigned char						ackseq;					//块序列号响应
  unsigned int						objsize;				//一共要传输的字节数
  unsigned char						lastblockoffset;//上一次内存块发送完后偏移量
  unsigned char						seqno;					//块序列号
  unsigned char						endfield;				//非数据补零的个数
  rxStep_t				        rxstep;					//块传输接收状态
  unsigned char						tmpData[8];			//块传输时临时数据缓冲区
  unsigned char						dataType;				//数据类型
  TIMER_HANDLE		timer;					        //超时定时器
  SDOCallback_t		Callback;				        //超时回调函数
};

/* 错误状态 */
typedef enum enum_errorState 
{
  Error_free = 0x00,		/* 无错误发生 */
  Error_occurred = 0x01	/* 有错误发生 */
}e_errorState;
/* 错误结构体 */
typedef struct 
{
	unsigned short  errCode;		/* 错误码 */
	unsigned char errRegMask;	  /* 错误掩码 */
	unsigned char active;			  /* 错误存在标志位 */
}s_errors;
/* EMCY_TRANS_REQ */
typedef enum enum_emcyTransReq
{
  TRANS_DIS = 0x00,		/* 无错误发生 */
  TRANS_EN = 0x01	/* 有错误发生 */
}e_emcyTransReq;
/* EMCY Trans Struct */
typedef struct Emcy_Trans_Data_struct
{
  e_emcyTransReq TransReq;
  unsigned short errCode;
  unsigned char  errRegMask;
  unsigned short addInfo;
}Emcy_Trans_Data_status;
extern Emcy_Trans_Data_status EmcyTransData;
/*HERAT_BEAT Structure_Define*/
typedef struct HEART_BEAT_STRUCT
{
  unsigned int telectrlSign;           //心跳报文远程帧信号
  unsigned int Timeout_server;         //生产者心跳超时对象索引
  TIMER_HANDLE Timeout_server_timer;   //定时事件
  unsigned char toggle;                //节点保护同步
}HEART_BEAT_Status;
extern HEART_BEAT_Status pHeartBeatPar;//心跳报文结构体
//State Machine 
/*Global define Stucture_Define*/
/*CANopen Para Stucture_Define*/
typedef struct s_state_communication_struct
{
	char csBoot_Up;
	char csSDO;
	char csEmergency;
	char csSYNC;
	char csLifeGuard;
	char csPDO;
	char csLSS;
}s_state_communication;
//CAOD_Nopen_STRUCT

#include "canopen_sdo.h"
/**************************/
/* CANopen Co_Data Define */
/**************************/
typedef struct CANopen_STRUCT
{
  unsigned char Node_ID;                                    
  unsigned int canBandrate;                                 
  unsigned  int NMT_state;                                  
  unsigned char Node_state;                                 
  
  /* NMT State Begin */
  s_state_communication CurrentCommunicationState;          
  initialisation_t initialisation;                          
	preOperational_t preOperational;                          
	operational_t operational;                                
	stopped_t stopped;                                        
  void (*NMT_Slave_Node_Reset_Callback)(CO_Data*);          
	void (*NMT_Slave_Communications_Reset_Callback)(CO_Data*);
  /* NMT State End */
  
  /* NMT-心跳机制 */
  unsigned char *ConsumerHeartbeatCount;                    
  unsigned int *ConsumerHeartbeatEntries;                   
  TIMER_HANDLE *ConsumerHeartBeatTimers;                    
  unsigned char NMTable[5]; 	                              
  heartbeatError_t heartbeatError;                          
  post_SlaveStateChange_t post_SlaveStateChange;            
  /* Object Dictionary Begin */
  const indextable *objdict;                                
  pLastObj_Status LastObj;                                  
  scanIndexOD_t OD_ScanIndex;                               
  valueRangeTest_t valueRangeTest;                          
  storeODSubIndex_t storeODSubIndex;                        
  const Quick_Index_Status *firstIndex;                     
  const Quick_Index_Status *lastIndex;                      
  TIMER_HANDLE *RxPDO_EventTimers;                          
  void (*RxPDO_EventTimers_Handler)(CO_Data*, unsigned int);
  /* Object Dictionary End */
  
  /* SDO Trans Data Begin */
  s_transfer transfers[SDO_MAX_SIMULTANEOUS_TRANSFERS];     
  /* SDO Trans Data End */
  
  /* PDO Trans Data Begin */
  s_PDO_status *PDO_status;                                 
  /* PDO Trans Data End */
  
  /* SYNC Trans Data Begin */
  post_sync_t post_sync;                                    
	post_TPDO_t post_TPDO;                                    
  /* SYNC Trans Data End */
  
  /* EMCY Trans Data Begin */
  e_errorState error_state;                                 
  unsigned char error_history_size;                         
	unsigned char* error_number;                              
	unsigned int* error_first_element;                        
	unsigned char* error_register;//0x1001 ERR REG            
	unsigned int* error_cobid;                                
	s_errors error_data[EMCY_MAX_ERRORS];                     
  /* EMCY Trans Data End */
  
  CanRxMsgTypeDef *CanRx_Buffer;                            
  CanTxMsgTypeDef *CanTx_Buffer;                            
  
}CANopen_Status;

/* STRCUT DEFINE END */
/* TYPEDEF DEFINE END */

/* Extern VARIABLE DECLARE BEGIN */
extern const          indextable pObjDict[];
extern CO_Data        CANopen_Drive;  
extern unsigned char  BULID_FILE[];
extern unsigned char  BULID_LINE;

extern const Quick_Index_Status OBJ_FirstIndex;
extern const Quick_Index_Status OBJ_LastIndex;
extern s_PDO_status   PDO_Trans_chache[PDO_MAX_LENGTH_TRANSFER];

extern unsigned int  testGl[50];
extern unsigned char testStr[50];
//

extern pDate_Status  Motor_calibra_date;   
extern unsigned int RS232_OD_SIGN;
extern unsigned int RPDO_SYNC_SIGN;

/*DS301 BEGIN*/
extern  unsigned int    obj1000_deviceType ;//设备类型
extern  unsigned char   obj1001_errReg ;//错误寄存器
extern  unsigned int    obj1002_Manufacturer_status ;//制造商状态寄存器
extern  unsigned char   obj1003_preErrRegion_subindex;
extern  unsigned int    obj1003_preErrRegion[1];
extern  unsigned int    obj1005_CobId_sync;
extern  unsigned int    obj1006_Cmnct_period_sync;
extern  unsigned int    obj1007_syncWindow_len;
extern  unsigned char   obj1008_Manufacturer_device_Name[8] ;
extern  unsigned char   obj1009_Manufacturer_Hardware_Vesion[4] ;
extern  unsigned char   obj100A_Manufacturer_Firmware_Vesion[4] ;
extern  unsigned short int obj100C_Guard_period;
extern  unsigned char   obj100D_existPeriod_fact;
extern  unsigned int    obj1010_subindex;
extern  unsigned int    obj1010_saveflash[1];
extern  unsigned int    obj1011_subindex;
extern  unsigned int    obj1011_recover[1];
extern  unsigned int    obj1012_CobId_timestamp;
extern  unsigned int    obj1013_timestamp;
extern  unsigned int    obj1014_CobId_Emcy;
extern  unsigned int    obj1015_InhabitTime_Emcy;
extern  unsigned char    obj1016_ClientHeart_timeout_subindex;
extern  unsigned int    obj1016_ClientHeart_timeout[5];
extern TIMER_HANDLE ObjDict_heartBeatTimers[5];
extern  unsigned int    obj1017_ServerHeart_timeout_subindex;
extern  unsigned int    obj101s7_SeverHeart_timeout[1];
    //0x1018 已在&CANopen_Drive.Ob_ID中定义
extern  unsigned char   obj1019_syncTimer_overflow;
extern  unsigned int    obj1028_Client_emcy_subindex;
extern  unsigned int    obj1028_Client_emcy[1];
/*SDO Server*/
extern  unsigned char obj1200_SdoServer_SubIndex;
extern  unsigned int obj1200_SdoServer_RSDO_COB_ID;
extern  unsigned int obj1200_SdoServer_TSDO_COB_ID;
extern  unsigned char obj1200_SdoServer_NodeId;   
/*SDO Client*/
extern  unsigned char obj1280_SdoClient_SubIndex;
extern  unsigned int obj1280_SdoClient_RSDO_COB_ID;
extern  unsigned int obj1280_SdoClient_TSDO_COB_ID;
extern  unsigned char obj1280_SdoClient_NodeId;
    /*RPDO通信参数*/
    //0x1400 已在pPdoPar.pPdoComunictPar.RPDO1中定义
    //0x1401 已在pPdoPar.pPdoComunictPar.RPDO2中定义
    //0x1402 已在pPdoPar.pPdoComunictPar.RPDO3中定义
    //0x1403 已在pPdoPar.pPdoComunictPar.RPDO4中定义
    /*RPDO映射参数*/
    //0x1600 已在pPdoPar.pPdoMapPar.RPDO1中定义
    //0x1601 已在pPdoPar.pPdoMapPar.RPDO2中定义
    //0x1602 已在pPdoPar.pPdoMapPar.RPDO3中定义
    //0x1603 已在pPdoPar.pPdoMapPar.RPDO4中定义
    /*TPDO通信参数*/
    //0x1800 已在pPdoPar.pPdoComunictPar.TPDO1中定义
    //0x1801 已在pPdoPar.pPdoComunictPar.TPDO2中定义
    //0x1802 已在pPdoPar.pPdoComunictPar.TPDO3中定义
    //0x1803 已在pPdoPar.pPdoComunictPar.TPDO4中定义
    /*TPDO映射参数*/
    //0x1A00 已在pPdoPar.pPdoMapPar.TPDO1中定义
    //0x1A01 已在pPdoPar.pPdoMapPar.TPDO2中定义
    //0x1A02 已在pPdoPar.pPdoMapPar.TPDO3中定义
    //0x1A03 已在pPdoPar.pPdoMapPar.TPDO4中定义
/*DS301 END*/
/*设备制造商区域0x2000 - 0x5FFF BEGIN*/
extern  unsigned char   obj2000;
extern  unsigned char   obj2000_sub1;
extern  unsigned char   obj2000_sub2;
extern  unsigned char   obj2000_sub3;
extern  unsigned char   obj2000_sub4;
extern  unsigned char   obj2000_sub5;
extern  unsigned char   obj2000_sub6;
extern  unsigned char   obj2000_sub7;
extern  unsigned int    obj2000_sub8;
extern  unsigned char   obj2098_NodeID_Default;
extern  unsigned int    obj2099_CanBandrate_Default;
/*设备制造商区域0x2000 - 0x5FFF  END*/
/*DS402 BEGIN*/
extern  short int       obj6007_AbortCnect_optcode ;
extern  unsigned short int obj603F_errcode ;
extern  unsigned short int obj6402_motorType ;
extern  unsigned char   obj6403_Motor_Nameplate[];
extern  unsigned char   obj6404_Motor_manufacturer_name[];
extern  unsigned int    obj6502_Supported_driver_mode ;//put it in INITIALIZATION 
extern  unsigned int    obj60FD_Digital_inputs;
extern  unsigned int    obj60FE_Digital_outputs_subindex;
extern  unsigned int    obj60FE_Digital_outputs_dout;
extern  unsigned int    obj60FE_Digital_outputs_mask;
extern  unsigned short int obj6040_Control_word;//控制字
extern  unsigned short int obj6041_Status_word; //状态字
extern  short int       obj605B_ShutDown_optcode ;
extern  short int       obj605C_Disable_optcode ;//1：斜坡减速，然后关闭驱动器使能
extern  short int       obj605A_QuickStop_optcode ;//2：开启急停功能
extern  short int       obj605D_Halt_optcode ;//1：减速至停止状态
extern  short int       obj605E_FaultReact_optcode ;//2：开启急停功能
extern  char            obj6060_Control_mode ;//操作（控制）模式
extern  char            obj6061_Control_mode_status ;
    /*FACTOR GROUP*/
extern  char            obj6089_PosNotation_index ;//Pos
extern  unsigned char   obj608A_PosDimension_index ;
extern  char            obj608B_VelNotation_index ;//Vel
extern  unsigned char   obj608C_VelDimension_index ;
extern  char            obj608D_AccelNotation_index ;//Accel
extern  unsigned char   obj608E_AccelDimension_index ;
extern  unsigned int    obj6093_PosFactor_subindex ;//0x6093 2 subindex
extern  unsigned int    obj6093_PosFactor_numerator ;//deault
extern  unsigned int    obj6093_PosFactor_FeedConst ;//deault
extern  unsigned int    obj6094_VelFactor_subindex ;//0x6094 2 subindex
extern  unsigned int    obj6094_VelFactor_numerator ;//deault
extern  unsigned int    obj6094_VelFactor_FeedConst ;//deault
extern  unsigned int    obj6097_AccelFactor_subindex ;//0x6097 2 subindex
extern  unsigned int    obj6097_AccelFactor_numerator ;//deault
extern  unsigned int    obj6097_AccelFactor_FeedConst ;//deault
extern  unsigned char   obj607E_Pos_rotation ;
    /*Profile Position Mode(PP)*/
extern  int             obj607A_Targt_Position ;
extern  int             obj607D_Croft_PosLimit_subindex ;
extern  int             obj607D_Croft_PosLimit_min  ;//POS_MIN
extern  int             obj607D_Croft_PosLimit_max ;//POS_MAX
extern  unsigned int    obj6081_Profile_Vel ;
extern  unsigned int    obj6083_Profile_Accel ;
extern  unsigned int    obj6084_Profile_Decel ;
extern  unsigned int    obj6085_Profile_QuickDecel ;
extern  short int       obj6086_Motion_profile_mode ;//Default:线性斜坡(梯形坡面)
    /*Homing mode*/
extern  int             obj607C_Homing_offset ;
extern  int             obj6098_Homing_mode ;
extern  unsigned int    obj6099_Homing_speed_subindex ;
extern  unsigned int    obj6099_Homing_speed_switch ;
extern  unsigned int    obj6099_Homing_speed_zero ;
extern  unsigned int    obj609A_Homing_Accel ;
    /*Position Control function*/
extern  int             obj6062_Position_demand_value ;
extern  int             *obj6063_Pos_act;
extern  int             obj6064_Position_act ;
extern  unsigned int    obj6065_Follow_Err_window ;//跟踪误差窗（允许位置跟踪误差）
extern  unsigned short int obj6066_Follow_Err_window_timeout ;//ms
extern  unsigned int    obj6067_Position_window ;//位置窗
extern  unsigned short int obj6068_Position_window_time ;//位置窗时间ms
extern  unsigned short int obj60F4_Follow_Err_act ;
extern  int             *obj60FC_PosDemand_value;
    /*Interpolated position mode(IP)*/
    //##
    /*Profile Velocity Mode(PV)*/
extern  int             obj6069_Vel_sensor_act ;//速度传感器值(counts/s)
extern  int             obj606B_Vel_demand_value ;//位置轨迹发生器输出值JOG
extern  int             obj606C_Vel_actual ;//速度实际值
extern  short int       obj606D_Vel_window ;
extern  unsigned short int obj606E_Vel_window_time ;
extern  unsigned short int obj606F_Vel_threshold ;//速度阈值
extern  unsigned short int obj6070_Vel_threshold_time ;
extern  int             obj60FF_Vel_Targt ;//目标速度
extern  int             obj60FF_Vel_Max_slippage ;//最大速度偏差
    /*Profile Torque Mode(PT)*/
extern  short int       obj6071_Torque_Targt ;//目标力矩
extern  unsigned short int obj6072_Max_Torque ;//最大力矩
extern  unsigned short int obj6073_Max_Current ;//最大电流
extern  short int       obj6074_Torque_demand_value ;//力矩指令值
extern  unsigned int    obj6075_rated_Current ;//电机额定电流
extern  unsigned int    obj6076_rated_torque ;//电机额定力矩
extern  short int       obj6077_Torque_act ;
extern  short int       obj6078_Current_act ;
extern  unsigned int    obj6079_Voltage_bus ;//母线电压
extern  unsigned int    obj6087_Torque_slope ;//力矩轮廓类型
extern  short int       obj6088_Torque_profile_type ;//力矩轮廓类型
    /*Veocity Mode(VL)*/
extern  short int       obj6042_Vlvel_targt ;   //vl目标速度
extern  short int       obj6043_Vlvel_demand_value ;//vl速度指令
extern  short int       obj6053_Vl_percentage_demand ;//vl百分比指令
extern  short int       obj6054_Vl_percentage_act ;//vl实际百分比
extern  short int       obj6055_Vl_manipulated_percentage ;//vl操作百分比
extern  unsigned int    obj604E_Vlvel_reference ;//vl速度参考
extern  int             obj604C_Vl_DimFactor_subindex ;//vl尺寸因子
extern  int             obj604C_Vl_DimFactor_numerator ;
extern  int             obj604C_Vl_DimFactor_denominator ;
extern  short int       obj604B_Vl_SetPointFactor_subindex ;//vl设定点因子
extern  short int       obj604B_Vl_SetPointFactor_numerato ;
extern  short int       obj604B_Vl_SetPointFactor_denominator ;
extern  unsigned char   obj604D_Pole_Number ;//vl极对数
extern  unsigned int    obj6046_Vlvel_MinMax_amount_subindex ;//vl速度最小最大数量
extern  unsigned int    obj6046_Vlvel_Min_amount ;
extern  unsigned int    obj6046_Vlvel_Max_amount ;
extern  unsigned int    obj6047_Vlvel_MinMax_subindex ;//vl速度最小最大值
extern  unsigned int    obj6047_Vlvel_Min_Pos ;
extern  unsigned int    obj6047_Vlvel_Max_Pos ;
extern  unsigned int    obj6047_Vlvel_Min_neg ;
extern  unsigned int    obj6047_Vlvel_Max_neg ;
extern  unsigned int    obj6058_VlfreqMotor_MinMax_amount_subindex ;//vl电机频率最小最大数量
extern  unsigned int    obj6058_VlfreqMotor_Min_amount ;
extern  unsigned int    obj6058_VlfreqMotor_Max_amount ;
extern  unsigned int    obj6059_VlfreqMotor_MinMax_subindex ;//vl电机频率最小最大值
extern  unsigned int    obj6059_VlfreqMotor_Min_Pos ;
extern  unsigned int    obj6059_VlfreqMotor_Max_Pos ;
extern  unsigned int    obj6059_VlfreqMotor_Min_neg ;
extern  unsigned int    obj6059_VlfreqMotor_Max_neg ;
extern  unsigned int    obj6056_VlvelMotor_MinMax_amount_subindex ;//vl速度电机最小最大数量
extern  unsigned int    obj6056_VlvelMotor_Min_amount ;
extern  unsigned int    obj6056_VlvelMotor_Max_amount ;
extern  unsigned int    obj6057_VlvelMotor_MinMax_subindex ;//vl速度电机最小最大值
extern  unsigned int    obj6057_VlvelMotor_Min_Pos ;
extern  unsigned int    obj6057_VlvelMotor_Max_Pos ;
extern  unsigned int    obj6057_VlvelMotor_Min_neg ;
extern  unsigned int    obj6057_VlvelMotor_Max_neg ;
extern  unsigned int    obj6048_Vlvel_Accel_subindex ;//vl速度加速度
extern  unsigned int    obj6048_Vlvel_Accel_deltaSpeed ;
extern  unsigned int    obj6048_Vlvel_Accel_deltaTime ;
extern  unsigned int    obj6049_Vlvel_Decel_subindex ;//vl速度减速度
extern  unsigned int    obj6049_Vlvel_Decel_deltaSpeed ;
extern  unsigned int    obj6049_Vlvel_Decel_deltaTime ;
extern  unsigned int    obj604A_Vlvel_QuickStop_subindex ;//vl速度急停
extern  unsigned int    obj604A_Vlvel_QuickStop_deltaSpeed ;
extern  unsigned int    obj604A_Vlvel_QuickStop_deltaTime ;
    
extern  unsigned int    obj604F_Vl_SlopeFunction_time ;//vl斜坡功能时间
extern  unsigned int    obj6050_Vl_Decel_time ;//vl减速时间
extern  unsigned int    obj6051_Vl_QuickStop_time ;//vl急停时间
extern  short int       obj6044_Vl_Control_effort ;//vl控制增益
extern  short int       obj6045_Vlvel_Manipulated ;//vl操作速度
extern  short int       obj6052_Vl_Nominal_percentage ;//vl名义百分比
/*DS402 END*/
#define		_Conflt				0x2015
#define		_AlarmClrReq	0x2016
#define		_MotorEn			0x2017
#define		_StatRegs			0x2018
#define		_RecLen				0x2019
#define		_RecGap				0x201A
//#define		_Reserved			0x201B
#define		_RecTrigval		0x201C			// 触发值
#define		_RecTrigPos		0x201D			// 触发点时刻
#define		_TrigMode			0x201E			// 触发类型
#define		_RecData			0x201F			// 

#define		_RatedCurr		0x2020
#define		_PeakCurr			0x2021
#define		_PeakCurrTim	0x2022
#define		_MaxPhaseCur	0x2023
#define		_MaxVel				0x2024
#define		_MaxPosErr		0x2025
#define		_MaxVelErr		0x2026
#define		_ProtectOn		0x2027
#define		_InjectPoint	0x2028			// ID: 40

#define		_InjectType		0x2029
#define		_InjectCurAmp	0x202A
#define		_InjectVelAmp	0x202B
#define		_InjectPosAmp	0x202C
#define		_InjectFreq		0x202D
#define		_CalcFiltEn		0x202E
#define		_CurrFiltDef	0x202F
#define		_CurrFiltOn		0x2030
#define		_VelFiltDef		0x2031
#define		_VelFiltOn		0x2032			// ID: 50

#define		_Ia						0x2033
#define		_Ib						0x2034
#define		_Ic						0x2035
#define		_Va						0x2036
#define		_Vb						0x2037
#define		_Vc						0x2038
#define		_Vd						0x2039
#define		_Vq						0x203A
#define		_MorCurrFilt	0x203B
#define		_RatedTor			0x203C			// ID: 60

#define		_MaxTor				0x203D
#define		_TorConst			0x203E
#define		_RatedVel			0x203F
#define		_EncType			0x2040
#define		_EncRes				0x2041
#define		_EncDir				0x2042
#define		_EncZeroAng		0x2043
#define		_EncFilt			0x2044
#define		_MotorJm			0x2045
#define		_Poles				0x2046			// ID: 70

#define  	_MotorRs			0x2047
#define  	_MotorLs			0x2048
#define  	_SoftLimEn		0x2049
#define		_SoftRevLim		0x204A
#define		_SoftFwdLim		0x204B
#define		_ErrLog				0x204C
#define		_MaxAcc				0x204D
#define  	_MaxDec				0x204E
#define  	_DirectMode		0x204F
#define  	_Jerk					0x2050			// ID: 80

#define  	_MotionMode		0x2051
#define  	_AbsTrgt			0x2052
#define  	_RelTrgt			0x2053
#define  	_Speed				0x2054
#define  	_Accel				0x2055
#define  	_Decel				0x2056
#define  	_EmrgDec			0x2057
#define  	_MotionReson	0x2058
#define  	_RptWait			0x2059
#define		_CtrlMode			0x205A			// ID: 90

#define  	_ReverseDir		0x205B
#define  	_ProtectMask	0x205C
#define  	_Vbus					0x205D
//#define  	_Reserved		0x205E
#define  	_MaxPwm				0x205F
#define  	_EmgerStpReq	0x2060
#define  	_DisableStpOp	0x2061
#define  	_FaultStpOp		0x2062
#define  	_DeviceAdd			0x2063
#define  	_FWVerion			0x2064		// ID: 100

#define  	_HWVerion			0x2065
#define  	_VersionDate	0x2066
#define  	_InVelLimRev	0x2067
#define  	_InVelLimFwd	0x2068
#define  	_StuckCurr		0x2069
#define  	_StuckTime		0x206A
#define  	_StuckVel			0x206B
#define  	_ComtMode			0x206C
#define  	_ComtInc			0x206D
#define  	_ComtStep			0x206E		// ID: 110

#define  	_StartComt		0x206F	
#define  	_InTargetTol	0x2070
#define  	_InTargetTim	0x2071
#define 	_TriggerID    0x2072
#define 	_PdPos        0x2073
#define  	_TrgtVel		  0x2074
#define  	_TrgtTor		  0x2075
#define  	_DInPort			0x2076
#define  	_Motor_mode		0x2077
#define  	_TriggerSubID	0x2078	// ID: 120

#define  	_DInLog				0x2079
#define  	_DInFilt			0x207A
#define  	_DInputMask		0x207B
#define  	_DInMode			0x207C
#define  	_DOutPort			0x207D
#define  	_OutLog				0x207E
#define  	_DOutMask			0x207F
#define  	_DOutMode			0x2080
#define  	_HomingExec		0x2081
#define  	_HomingOn			0x2082		// ID: 130

#define  	_HomingStat		0x2083
#define  	_HomingDef		0x2084
#define  	_Speed_SW			0x2085		
#define  	_Pll_Kp				0x2086
#define  	_Pll_Ki				0x2087
#define  	_PosFFW				0x2088	
#define  	_PosFFWfilt		0x2089
#define  	_SetPosition	0x208A
#define  	_DOutMode_Pin	0x208B		
#define  	_BrkOffDly		0x208C		//ID: 140


#define  	_BrkOnDly			0x208D				
#define  	_HomeWallTime	0x208E		
#define  	_HomeWallCurr	0x208F
#define  	_RegenOn			0x2090
#define  	_RegenOff			0x2091	
#define  	_Save					0x2092
#define  	_HallStatData	0x2093
#define  	_HallDisP			0x2094		
#define  	_HallDisN			0x2095		
#define  	_SwitchZeroReq			 0x2096	// ID: 150
#define   _SwitchZeroPos_Range 0x2097




/* VARIABLE DECLARE END */


#endif
