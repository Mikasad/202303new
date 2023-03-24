#include "canopen_od.h"
#include "bsp_can.h"
#include "canopen_lifegrd.h"
#include "canopen_pdo.h"
#include "canopen_sync.h"
#include "public_global.h"
#include "bsp_ota.h"
#include "bsp_app_function.h"
//#include "bsp_BDCMotor.h"
#include "main.h"
#include "ds402.h"
/* VARIABLE DECLARE BEGIN */
extern long FlashArray[];
/* PUBLIC VAR DECLAR */
unsigned char BULID_FILE[] = {0};
unsigned char BULID_LINE = 0;
/* TEST VAR DECLAR */
unsigned int  testGl[50];
unsigned char testStr[50];
pTime_Consume_Status    TimeConsume;
pCanLoadRate_Status     CanLoadRate;
Emcy_Trans_Data_status  EmcyTransData;
//
unsigned int pReserved = 0;
unsigned char pTest_1;
unsigned short int pTest_2;
unsigned int pTest_4;
char pSest_1;
signed short pSest_2;
int pSest_4;
unsigned char pStr_8[] = "ATLAS";

pDate_Status  Motor_calibra_date;     //电机最后一次校准日期
unsigned int RS232_OD_SIGN = 0;
unsigned int RPDO_SYNC_SIGN = 0;

/* FUNCTION DECLAR BEGIN  */
unsigned int  ODCallback_t_Index1010_Subindex1(CO_Data *d, const indextable *index, unsigned char bSubindex);
//unsigned int  ODCallback_t_Index208A_Subindex0(CO_Data *d, const indextable *index, unsigned char bSubindex);
unsigned int  ODCallback_t_Index2011_Subindex0(CO_Data *d, const indextable *index, unsigned char bSubindex);
unsigned int  ODCallback_t_Index2012_Subindex0(CO_Data *d, const indextable *index, unsigned char bSubindex);
unsigned int  ODCallback_t_Index200B_Subindex0(CO_Data *d, const indextable *index, unsigned char bSubindex);
unsigned int  ODCallback_t_Index3001_Subindex0(CO_Data *d, const indextable *index, unsigned char bSubindex);
/* FUNCTION DECLAR END  */

/* SDO/PDO OD参数ID快速索引定义 */
const Quick_Index_Status OBJ_FirstIndex = {
    24,	//SDO服务器参数ID
    25,	//SDO客户端参数ID
    26,	//RPDO通信参数ID
    30,	//RPDO映射参数ID
    34,	//TPDO通信参数ID
    38  //TPDO映射参数ID
};

const Quick_Index_Status OBJ_LastIndex = {
    24,	//SDO服务器参数ID
    25,	//SDO客户端参数ID
    29,	//RPDO通信参数ID
    33,	//RPDO映射参数ID
    37,	//TPDO通信参数ID
    41	//TPDO映射参数ID
};

s_PDO_status PDO_Trans_chache[PDO_MAX_LENGTH_TRANSFER];

/* VARIABLE DECLARE END */

/*--------------------------------------------------------------------------------------------------------

                               Atlas Driver CANopen Object Dictionary

--------------------------------------------------------------------------------------------------------*/
#ifdef _OBJ_DECLARE
//对象字典参数定义
#ifdef _OBJ_DS301_DECLARE
/*DS301 BEGIN*/
//Device-Servo drive Bit0-15:0x0192h  Bit16-23:type:0x02  Bit24-31:xx
unsigned int obj1000_deviceType = (0x0002<<16) | 0x0192;   //设备类型
index_pObjDictry ObjDict_Index1000[]=
{
    {0x1000,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1000_deviceType}
};
/**/
unsigned char obj1001_errReg = 0x00; //错误寄存器
index_pObjDictry ObjDict_Index1001[]=
{
    {0x1001,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj1001_errReg},
};
/**/
unsigned int obj1002_Manufacturer_status = 0x00;
index_pObjDictry ObjDict_Index1002[]=
{
    {0x1002,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,&obj1002_Manufacturer_status},
};
/**/
unsigned char obj1003_preErrRegion_subindex = 0x00;
unsigned int obj1003_preErrRegion[] = {0};//错误域
index_pObjDictry ObjDict_Index1003[]=
{
    {0x1003,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj1003_preErrRegion_subindex},//错误数
    {0x1003,1,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1003_preErrRegion[0]}//标准错误域(可选错误域子索引：02h-FEh)
};
/**/
unsigned int obj1005_CobId_sync = 0x80;//SYNC cob id
index_pObjDictry ObjDict_Index1005[]=
{
    {0x1005,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1005_CobId_sync}
};
/**/
unsigned int obj1006_Cmnct_period_sync = 0x00; //同步循环周期
index_pObjDictry ObjDict_Index1006[]=
{
    {0x1006,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1006_Cmnct_period_sync}
};
/**/
unsigned int obj1007_syncWindow_len = 0x00;
index_pObjDictry ObjDict_Index1007[]=
{
    {0x1007,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1007_syncWindow_len}
};
/**/
unsigned char obj1008_Manufacturer_device_Name[8] = "gaussian";
index_pObjDictry ObjDict_Index1008[]=
{
    {0x1008,0,CONST,UNSIGNED8_STRING,sizeof(obj1008_Manufacturer_device_Name),PDO_BAN,(void*)&obj1008_Manufacturer_device_Name}
};
/**/
unsigned char obj1009_Manufacturer_Hardware_Vesion[4] = "1006";
index_pObjDictry ObjDict_Index1009[]=
{
    {0x1009,0,CONST,UNSIGNED8_STRING,sizeof(obj1009_Manufacturer_Hardware_Vesion),PDO_BAN,(void*)&obj1009_Manufacturer_Hardware_Vesion}
};
/**/
unsigned char obj100A_Manufacturer_Firmware_Vesion[4] = "1223";
index_pObjDictry ObjDict_Index100A[]=
{
    {0x100A,0,CONST,UNSIGNED8_STRING,sizeof(obj100A_Manufacturer_Firmware_Vesion),PDO_BAN,(void*)&obj100A_Manufacturer_Firmware_Vesion}
};
/**/
unsigned short int obj100C_Guard_period = 0x00;//监护周期[Master]
index_pObjDictry ObjDict_Index100C[]=
{
    {0x100C,0,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&obj100C_Guard_period},
};
/**/
unsigned char obj100D_existPeriod_fact = 0x00;//生存周期因子[Master]
index_pObjDictry ObjDict_Index100D[]=
{
    {0x100D,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj100D_existPeriod_fact}
};
/**/
unsigned int obj1010_subindex = 0x01;
u8 Atlas_WriteRead_Flag = 0;
ODCallback_t Index1010_callbacks[] =
{
    NULL,
    ODCallback_t_Index1010_Subindex1,
};
index_pObjDictry ObjDict_Index1010[]=
{
    {0x1010,0,CONST,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1010_subindex},
    {0x1010,1,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&Atlas_WriteRead_Flag}
};
/**/
unsigned int obj1011_subindex = 0x01;//恢复缺省参数子索引
unsigned int obj1011_recover[1];//恢复缺省参数
index_pObjDictry ObjDict_Index1011[]=
{
    {0x1011,0,CONST,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1011_subindex},//最高子索引数(01h-7Fh)
    {0x1011,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1011_recover[0]},//指所有可存储在CANopen设备上的参数
};
/**/
unsigned int obj1012_CobId_timestamp = 0x00;//时间戳对象cob id
index_pObjDictry ObjDict_Index1012[]=
{
    {0x1012,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1012_CobId_timestamp}
};
/**/
unsigned int obj1013_timestamp = 0x00;
index_pObjDictry ObjDict_Index1013[]=
{
    {0x1013,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1013_timestamp}
};
/**/
unsigned int obj1014_CobId_Emcy = 0x80;
index_pObjDictry ObjDict_Index1014[]=
{
    {0x1014,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1014_CobId_Emcy}
};
/**/
unsigned int obj1015_InhabitTime_Emcy = 0x00;
index_pObjDictry ObjDict_Index1015[]=
{
    {0x1015,0,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&obj1015_InhabitTime_Emcy}
};
/**/
unsigned char obj1016_ClientHeart_timeout_subindex = 0x01;//消费者心跳超时子索引（Subindex：1-5）
unsigned int obj1016_ClientHeart_timeout[5] = {0x007F03E8,0,0,0,0};;//消费者心跳超时 Bit31-24：Reserved   Bit23-16：Node-ID   Bit15-0：心跳超时时间（ms）
TIMER_HANDLE ObjDict_heartBeatTimers[5];
ODCallback_t Index1016_callbacks[] =
{
    ODCallback_t_Index1016_Subindex,
    ODCallback_t_Index1016_Subindex,
    ODCallback_t_Index1016_Subindex,
    ODCallback_t_Index1016_Subindex,
    ODCallback_t_Index1016_Subindex,
    ODCallback_t_Index1016_Subindex
};
index_pObjDictry ObjDict_Index1016[]=
{
    {0x1016,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj1016_ClientHeart_timeout_subindex}, //最大子索引数
    {0x1016,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1016_ClientHeart_timeout[0]}, //消费者1心跳超时参数
    {0x1016,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1016_ClientHeart_timeout[1]},
    {0x1016,3,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1016_ClientHeart_timeout[2]},
    {0x1016,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1016_ClientHeart_timeout[3]},
    {0x1016,5,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1016_ClientHeart_timeout[4]}
};
/**/
unsigned int obj1017_ServerHeart_timeout_subindex = 0x00;//生产者心跳超时子索引
unsigned int obj1017_SeverHeart_timeout[1];//生产者心跳超时
ODCallback_t Index1017_callbacks[] =
{
    NULL,
};
index_pObjDictry ObjDict_Index1017[]=
{
    {0x1017,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pHeartBeatPar.Timeout_server}
};
/**/
index_pObjDictry ObjDict_Index1018[]=
{
    {0x1018,0,CONST,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&Ob_ID.inNum_max},//最大子索引数(01h-04h)
    {0x1018,1,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&Ob_ID.VendorID},//特定的值为每个CANopen设备供应商所独有，值00000000表明无效的供应商标识
    {0x1018,2,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&Ob_ID.ProductCode},//特定的值，来识别特定类型的CANopen设备，值00000000应被保留
    {0x1018,3,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&Ob_ID.RevisionNumber},//包含CANopen设备的主修订版本号和次修订版本号
    {0x1018,4,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&Ob_ID.SerialNumber}//序列号，为某一组产品和CANopen设备的唯一标识
};
/**/
//0x1018 已在&CANopen_Drive.Ob_ID中定义
unsigned char obj1019_syncTimer_overflow = 0x00;
index_pObjDictry ObjDict_Index1019[]=
{
    {0x1019,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj1019_syncTimer_overflow}
};
/**/
unsigned int obj1028_Client_emcy_subindex = 0x00;//应急消费对象子索引
unsigned int obj1028_Client_emcy[1];//应急消费对象
index_pObjDictry ObjDict_Index1028[]=
{
    {0x1028,0,CONST,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1028_Client_emcy_subindex},//子索引数
    {0x1028,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1028_Client_emcy[0]},//应急消费者1(Index SubIndex length)
};
/**/
/*SDO*/
//SDO服务器对象(参见CANopen301 P119)
unsigned char obj1200_SdoServer_SubIndex = 2;
unsigned int obj1200_SdoServer_RSDO_COB_ID = CANID_RSDO;
unsigned int obj1200_SdoServer_TSDO_COB_ID = CANID_TSDO;
unsigned char obj1200_SdoServer_NodeId = 0;
index_pObjDictry ObjDict_Index1200[]=
{
    //SDO服务器对象(参见CANopen301 P119)
    {0x1200,0,CONST,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj1200_SdoServer_SubIndex},//子索引数
    {0x1200,1,CONST,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1200_SdoServer_RSDO_COB_ID},//COB-ID(Client->Server) PDO可选
    {0x1200,2,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1200_SdoServer_TSDO_COB_ID},//COB-ID(Server->Client) PDO可选
    {0x1200,3,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj1200_SdoServer_NodeId},//SDO客户端Node-ID
};
/**/
//SDO_1客户端参数(参见CANopen301 P121)
unsigned char obj1280_SdoClient_SubIndex = 0;
unsigned int obj1280_SdoClient_RSDO_COB_ID = 0;
unsigned int obj1280_SdoClient_TSDO_COB_ID = 0;
unsigned char obj1280_SdoClient_NodeId = 0;
index_pObjDictry ObjDict_Index1280[]=
{
    {0x1280,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj1280_SdoClient_SubIndex},//子索引数
    {0x1280,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1280_SdoClient_RSDO_COB_ID},//COB-ID(Client->Server) PDO可选
    {0x1280,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj1280_SdoClient_TSDO_COB_ID},//COB-ID(Server->Client) PDO可选
    {0x1280,3,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj1280_SdoClient_NodeId},//SDO客户端Node-ID
};
/**/
/*PDO*/
ODCallback_t RPDO_Cmct_callbacks[] =
{
    NULL,
    ODCallback_t_RPDO_Communicate,
    ODCallback_t_RPDO_Communicate,
};
index_pObjDictry ObjDict_Index1400[]=
{
    //RPDO1通信参数
    {0x1400,0,CONST,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.RPDO1.inNum},//对象条目记录，子索引数
    {0x1400,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.RPDO1.rPDO_ID},//RPDO1 COB-ID
    {0x1400,2,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.RPDO1.last_TransType},//传输类型
};
/**/
index_pObjDictry ObjDict_Index1401[]=
{
    //RPDO2通信参数
    {0x1401,0,CONST,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.RPDO2.inNum},
    {0x1401,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.RPDO2.rPDO_ID},
    {0x1401,2,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.RPDO2.last_TransType},
};
/**/
index_pObjDictry ObjDict_Index1402[]=
{
    //RPDO3通信参数
    {0x1402,0,CONST,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.RPDO3.inNum},
    {0x1402,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.RPDO3.rPDO_ID},
    {0x1402,2,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.RPDO3.last_TransType},
};
/**/
index_pObjDictry ObjDict_Index1403[]=
{
    //RPDO4通信参数
    {0x1403,0,CONST,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.RPDO4.inNum},
    {0x1403,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.RPDO4.rPDO_ID},
    {0x1403,2,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.RPDO4.last_TransType},
};
/**/
ODCallback_t IndexRPDO_Map_callbacks[] =
{
    ODCallback_t_RPDO_MapSubindex0,
    ODCallback_t_RPDO_Map,
    ODCallback_t_RPDO_Map,
    ODCallback_t_RPDO_Map,
    ODCallback_t_RPDO_Map,
    ODCallback_t_RPDO_Map,
    ODCallback_t_RPDO_Map,
    ODCallback_t_RPDO_Map,
    ODCallback_t_RPDO_Map,
};
index_pObjDictry ObjDict_Index1600[]=
{
    //RPDO1映射参数(更改信息子索引时，若OD中无对应对象(索引子索引不存在，数据长度不一致，PDO长度异常等)，将返回SDO中止应答或EMCY写服务(如果支持的话))
    //终止码发生在不同的操作和不同的状态中，参见CANopen 301 P127
    {0x1600,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO1.inNum},//指定有效对象条目
    {0x1600,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO1.Index[0][MAP_INDEX_OD_ADDR]},//对象映射信息子索引(01h - 40h)
    {0x1600,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO1.Index[1][MAP_INDEX_OD_ADDR]},
    {0x1600,3,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO1.Index[2][MAP_INDEX_OD_ADDR]},
    {0x1600,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO1.Index[3][MAP_INDEX_OD_ADDR]},
    {0x1600,5,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO1.Index[4][MAP_INDEX_OD_ADDR]},
    {0x1600,6,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO1.Index[5][MAP_INDEX_OD_ADDR]},
    {0x1600,7,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO1.Index[6][MAP_INDEX_OD_ADDR]},
    {0x1600,8,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO1.Index[7][MAP_INDEX_OD_ADDR]},
};
/**/
index_pObjDictry ObjDict_Index1601[]=
{
    //RPDO2映射参数
    {0x1601,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO2.inNum},
    {0x1601,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO2.Index[0][MAP_INDEX_OD_ADDR]},
    {0x1601,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO2.Index[1][MAP_INDEX_OD_ADDR]},
    {0x1601,3,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO2.Index[2][MAP_INDEX_OD_ADDR]},
    {0x1601,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO2.Index[3][MAP_INDEX_OD_ADDR]},
    {0x1601,5,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO2.Index[4][MAP_INDEX_OD_ADDR]},
    {0x1601,6,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO2.Index[5][MAP_INDEX_OD_ADDR]},
    {0x1601,7,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO2.Index[6][MAP_INDEX_OD_ADDR]},
    {0x1601,8,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO2.Index[7][MAP_INDEX_OD_ADDR]},
};
/**/
index_pObjDictry ObjDict_Index1602[]=
{
    //RPDO3映射参数
    {0x1602,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO3.inNum},
    {0x1602,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO3.Index[0][MAP_INDEX_OD_ADDR]},
    {0x1602,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO3.Index[1][MAP_INDEX_OD_ADDR]},
    {0x1602,3,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO3.Index[2][MAP_INDEX_OD_ADDR]},
    {0x1602,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO3.Index[3][MAP_INDEX_OD_ADDR]},
    {0x1602,5,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO3.Index[4][MAP_INDEX_OD_ADDR]},
    {0x1602,6,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO3.Index[5][MAP_INDEX_OD_ADDR]},
    {0x1602,7,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO3.Index[6][MAP_INDEX_OD_ADDR]},
    {0x1602,8,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO3.Index[7][MAP_INDEX_OD_ADDR]},
};
/**/
index_pObjDictry ObjDict_Index1603[]=
{
    //RPDO4映射参数
    {0x1603,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO4.inNum},
    {0x1603,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO4.Index[0][MAP_INDEX_OD_ADDR]},
    {0x1603,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO4.Index[1][MAP_INDEX_OD_ADDR]},
    {0x1603,3,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO4.Index[2][MAP_INDEX_OD_ADDR]},
    {0x1603,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO4.Index[3][MAP_INDEX_OD_ADDR]},
    {0x1603,5,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO4.Index[4][MAP_INDEX_OD_ADDR]},
    {0x1603,6,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO4.Index[5][MAP_INDEX_OD_ADDR]},
    {0x1603,7,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO4.Index[6][MAP_INDEX_OD_ADDR]},
    {0x1603,8,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.RPDO4.Index[7][MAP_INDEX_OD_ADDR]},
};
/**/
//1604h - 17FFh 制造商指定
ODCallback_t TPDO_Cmct_callbacks[] =
{
    NULL,
    ODCallback_t_TPDO_Communicate,
    ODCallback_t_TPDO_Communicate,
    ODCallback_t_TPDO_Communicate,
    ODCallback_t_TPDO_Communicate,
    ODCallback_t_TPDO_Communicate,
    ODCallback_t_TPDO_Communicate,
};
index_pObjDictry ObjDict_Index1800[]=
{
    //TPDO1通信参数
    {0x1800,0,CONST,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO1.inNum},//对象条目记录
    {0x1800,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO1.tPDO_ID},//TPDO1 COB-ID
    {0x1800,2,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO1.last_TransType},//传输类型
    {0x1800,3,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO1.inhibit_time},//禁止时间
    {0x1800,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO1.reserved},//Reserved，读取或写入将导致SDO中止应答(参见CANopen301 P131)
    {0x1800,5,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO1.event_time},//触发事件时间
    {0x1800,6,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO1.tSync_Num},//SYNC起始值，为0表示该PDO不处理SYNC帧的计数，为1-240表示处理SYNC帧的计数，SYNC帧计数器等于该值时，视为收到一帧，当PDO存在时该值不应更改
};
/**/
index_pObjDictry ObjDict_Index1801[]=
{
    //TPDO2通信参数
    {0x1801,0,CONST,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO2.inNum},
    {0x1801,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO2.tPDO_ID},
    {0x1801,2,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO2.last_TransType},
    {0x1801,3,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO2.inhibit_time},
    {0x1801,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO2.reserved},
    {0x1801,5,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO2.event_time},
    {0x1801,6,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO2.tSync_Num},
};
/**/
index_pObjDictry ObjDict_Index1802[]=
{
    //TPDO3通信参数
    {0x1802,0,CONST,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO3.inNum},
    {0x1802,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO3.tPDO_ID},
    {0x1802,2,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO3.last_TransType},
    {0x1802,3,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO3.inhibit_time},
    {0x1802,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO3.reserved},
    {0x1802,5,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO3.event_time},
    {0x1802,6,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO3.tSync_Num},
};
/**/
index_pObjDictry ObjDict_Index1803[]=
{
    //TPDO4通信参数
    {0x1803,0,CONST,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO4.inNum},
    {0x1803,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO4.tPDO_ID},
    {0x1803,2,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO4.last_TransType},
    {0x1803,3,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO4.inhibit_time},
    {0x1803,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO4.reserved},
    {0x1803,5,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO4.event_time},
    {0x1803,6,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoComunictPar.TPDO4.tSync_Num},
};
/**/
ODCallback_t IndexTPDO_Map_callbacks[] =
{
    ODCallback_t_TPDO_MapSubindex0,
    ODCallback_t_TPDO_Map,
    ODCallback_t_TPDO_Map,
    ODCallback_t_TPDO_Map,
    ODCallback_t_TPDO_Map,
    ODCallback_t_TPDO_Map,
    ODCallback_t_TPDO_Map,
    ODCallback_t_TPDO_Map,
    ODCallback_t_TPDO_Map,
};
//1804h - 19FFh 制造商指定
index_pObjDictry ObjDict_Index1A00[]=
{
    //TPDO1映射参数(参见CANopen301 P134)
    {0x1A00,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO1.inNum},//指定有效对象条目
    {0x1A00,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO1.Index[0][MAP_INDEX_OD_ADDR]},//对象映射信息子索引(01h - 40h)
    {0x1A00,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO1.Index[1][MAP_INDEX_OD_ADDR]},
    {0x1A00,3,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO1.Index[2][MAP_INDEX_OD_ADDR]},
    {0x1A00,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO1.Index[3][MAP_INDEX_OD_ADDR]},
    {0x1A00,5,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO1.Index[4][MAP_INDEX_OD_ADDR]},
    {0x1A00,6,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO1.Index[5][MAP_INDEX_OD_ADDR]},
    {0x1A00,7,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO1.Index[6][MAP_INDEX_OD_ADDR]},
    {0x1A00,8,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO1.Index[7][MAP_INDEX_OD_ADDR]},
};
/**/
index_pObjDictry ObjDict_Index1A01[]=
{
    //TPDO1映射参数(参见CANopen301 P134)
    {0x1A01,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO2.inNum},
    {0x1A01,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO2.Index[0][MAP_INDEX_OD_ADDR]},
    {0x1A01,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO2.Index[1][MAP_INDEX_OD_ADDR]},
    {0x1A01,3,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO2.Index[2][MAP_INDEX_OD_ADDR]},
    {0x1A01,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO2.Index[3][MAP_INDEX_OD_ADDR]},
    {0x1A01,5,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO2.Index[4][MAP_INDEX_OD_ADDR]},
    {0x1A01,6,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO2.Index[5][MAP_INDEX_OD_ADDR]},
    {0x1A01,7,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO2.Index[6][MAP_INDEX_OD_ADDR]},
    {0x1A01,8,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO2.Index[7][MAP_INDEX_OD_ADDR]},
};
/**/
index_pObjDictry ObjDict_Index1A02[]=
{
    //TPDO3映射参数
    {0x1A02,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO3.inNum},
    {0x1A02,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO3.Index[0][MAP_INDEX_OD_ADDR]},
    {0x1A02,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO3.Index[1][MAP_INDEX_OD_ADDR]},
    {0x1A02,3,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO3.Index[2][MAP_INDEX_OD_ADDR]},
    {0x1A02,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO3.Index[3][MAP_INDEX_OD_ADDR]},
    {0x1A02,5,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO3.Index[4][MAP_INDEX_OD_ADDR]},
    {0x1A02,6,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO3.Index[5][MAP_INDEX_OD_ADDR]},
    {0x1A02,7,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO3.Index[6][MAP_INDEX_OD_ADDR]},
    {0x1A02,8,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO3.Index[7][MAP_INDEX_OD_ADDR]},
};
/**/
index_pObjDictry ObjDict_Index1A03[]=
{
    //TPDO4映射参数
    {0x1A03,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO4.inNum},
    {0x1A03,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO4.Index[0][MAP_INDEX_OD_ADDR]},
    {0x1A03,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO4.Index[1][MAP_INDEX_OD_ADDR]},
    {0x1A03,3,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO4.Index[2][MAP_INDEX_OD_ADDR]},
    {0x1A03,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO4.Index[3][MAP_INDEX_OD_ADDR]},
    {0x1A03,5,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO4.Index[4][MAP_INDEX_OD_ADDR]},
    {0x1A03,6,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO4.Index[5][MAP_INDEX_OD_ADDR]},
    {0x1A03,7,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO4.Index[6][MAP_INDEX_OD_ADDR]},
    {0x1A03,8,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pPdoPar.pPdoMapPar.TPDO4.Index[7][MAP_INDEX_OD_ADDR]},
};
/**/
//1A04h - 1BFFh 制造商指定

//1FA0h - 1FCFh 对象扫描仪列表，用于SAM-MPDO(Reserved)

//1FD0h - 1FFFh 对象分配列表，用于SAM-MPDO(Reserved)
/*DS301 END*/
#endif
#ifdef _OBJ_MANUFACTURER
/*设备制造商区域0x2000 - 0x5FFF BEGIN*/
/* 测试对象 */
extern MotorParameters_t MotorParameters[NBR_OF_MOTORS];
unsigned char obj2000 = 8;
unsigned char obj2000_sub1 = 0x00;
unsigned char obj2000_sub2 = 0x00;
unsigned char obj2000_sub3 = 0x00;
unsigned char obj2000_sub4 = 0x00;
unsigned char obj2000_sub5 = 0x00;
unsigned char obj2000_sub6 = 0x00;
unsigned char obj2000_sub7 = 0x00;
unsigned int  obj2000_sub8 = 0x00;
index_pObjDictry ObjDict_Index2000[]=
{
    {0x2000,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj2000},
    {0x2000,1,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&obj2000_sub1},
    {0x2000,2,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&obj2000_sub2},
    {0x2000,3,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&obj2000_sub3},
    {0x2000,4,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&obj2000_sub4},
    {0x2000,5,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&obj2000_sub5},
    {0x2000,6,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&obj2000_sub6},
    {0x2000,7,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&obj2000_sub7},
    {0x2000,8,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&obj2000_sub8},
};
uint8_t ParInNum14 = 14;
uint8_t ParInNum12 = 12;
uint8_t ParInNum10 = 10;
uint8_t ParInNum2 = 2;
uint8_t ParInNum3 = 3;
uint8_t ParInNum4 = 4;
uint8_t ParInNum6 = 6;
uint8_t ParInNum7 = 7;

index_pObjDictry ObjDict_Index2001[]=
{
    {0x2001,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x2001,1,RW,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&MotorParameters[M1].PolePairNum}, //M1极对数
    {0x2001,2,RW,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&MotorParameters[M2].PolePairNum}, //M2极对数
};
index_pObjDictry ObjDict_Index2002[]=
{
    {0x2002,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x2002,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M1].ConstantTorque}, //M1力矩常数
    {0x2002,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M2].ConstantTorque}, //M2力矩常数
};
index_pObjDictry ObjDict_Index2003[]=
{
    {0x2003,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x2003,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M1].ConstantInertia}, //M1电机惯量
    {0x2003,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M2].ConstantInertia}, //M2电机惯量

};
index_pObjDictry ObjDict_Index2004[]=
{
    {0x2004,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x2004,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M1].RatedCurrent}, //M1额定电流 1.191
    {0x2004,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M2].RatedCurrent}, //M2额定电流
};
index_pObjDictry ObjDict_Index2005[]=
{
    {0x2005,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x2005,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M1].RatedTorque}, //M1额定转矩
    {0x2005,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M2].RatedTorque}, //M2额定转矩
};
index_pObjDictry ObjDict_Index2006[]=
{
    {0x2006,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x2006,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M1].RatedSpeed}, //M1额定转速 0.6
    {0x2006,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M2].RatedSpeed}, //M2额定转速
};
index_pObjDictry ObjDict_Index2007[]=
{
    {0x2007,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x2007,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M1].PulseNumber}, //M1编码器分辨率
    {0x2007,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M2].PulseNumber}, //M2编码器分辨率

};
index_pObjDictry ObjDict_Index2008[]=
{
    {0x2008,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x2008,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M1].PeakCurrent}, //M1峰值电流 1.191
    {0x2008,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M2].PeakCurrent}, //M2峰值电流
};
index_pObjDictry ObjDict_Index2009[]=
{
    {0x2009,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x2009,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M1].PeakTicks}, //M1峰值电流持续时间
    {0x2009,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M2].PeakTicks}, //M2峰值电流持续时间
};
index_pObjDictry ObjDict_Index200A[]=
{
    {0x200A,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x200A,1,RO,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M1].MAXPhaseCurrent}, //M1最大相电流 1.191
    {0x200A,2,RO,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M2].MAXPhaseCurrent}, //M2最大相电流
};
uint8_t obj2020_OTA_Default = 0x00;
ODCallback_t Index200B_callbacks[] =
{
    ODCallback_t_Index200B_Subindex0,
};
index_pObjDictry ObjDict_Index200B[]=
{
    {0x200B,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj2020_OTA_Default},
//    {0x200B,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M1].MAXSpeed}, //M1最大速度 0.6
//    {0x200B,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M2].MAXSpeed}, //M2最大速度
};
index_pObjDictry ObjDict_Index200C[]=
{
    {0x200C,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x200C,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M1].LockedSpeed}, //M1堵转速度 0.6
    {0x200C,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M2].LockedSpeed}, //M2堵转速度
};
index_pObjDictry ObjDict_Index200D[]=
{
    {0x200D,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x200D,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M1].ShortCircuit}, //M1堵转电流 1.191
    {0x200D,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M2].ShortCircuit}, //M2堵转电流
};
index_pObjDictry ObjDict_Index200E[]=
{
    {0x200E,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x200E,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M1].LockeTicks}, //M1堵转时间
    {0x200E,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M2].LockeTicks}, //M2堵转时间
};
index_pObjDictry ObjDict_Index200F[]=
{
    {0x200F,0,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&Sync_Async_Control}, //0：M1和M2异步控制  1：M1和M2同步控制
};
index_pObjDictry ObjDict_Index2010[]=
{
    {0x2010,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum7},
    {0x2010,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&PIDIqHandle_M2.hKpGain}, //M2电流环P
    {0x2010,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&PIDIqHandle_M2.hKiGain}, //M2电流环I
    {0x2010,3,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&PIDSpeedHandle_M2.hKpGain}, //M2速度环P
    {0x2010,4,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&PIDSpeedHandle_M2.hKiGain}, //M2速度环I
    {0x2010,5,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&PID_PosParamsM2.hKpGain}, //M2位置环P
    {0x2010,6,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&PID_PosParamsM2.hKiGain}, //M2位置环I
    {0x2010,7,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&PID_PosParamsM2.hKdGain}, //M2位置环D
};
unsigned char obj2011_NodeID_Default = 0x05;
ODCallback_t Index2011_callbacks[] =
{
    ODCallback_t_Index2011_Subindex0,
};
index_pObjDictry ObjDict_Index2011[]=
{
    {0x2011,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj2011_NodeID_Default},
};

unsigned int obj2012_CanBandrate_Default = 0x01F4;
ODCallback_t Index2012_callbacks[] =
{
    ODCallback_t_Index2012_Subindex0,
};
index_pObjDictry ObjDict_Index2012[]=
{
    {0x2012,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj2012_CanBandrate_Default},
};
index_pObjDictry ObjDict_Index2013[]=
{
    {0x2013,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum7},
    {0x2013,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&PIDIqHandle_M1.hKpGain}, //M1电流环P
    {0x2013,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&PIDIqHandle_M1.hKiGain}, //M1电流环I
    {0x2013,3,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&PIDSpeedHandle_M1.hKpGain}, //M1速度环P
    {0x2013,4,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&PIDSpeedHandle_M1.hKiGain}, //M1速度环I
    {0x2013,5,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&PID_PosParamsM1.hKpGain}, //M1位置环P
    {0x2013,6,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&PID_PosParamsM1.hKiGain}, //M1位置环I
    {0x2013,7,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&PID_PosParamsM1.hKdGain}, //M1位置环D
};
index_pObjDictry ObjDict_Index2014[]=
{
    {0x2014,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x2014,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&RelativeLocation[M1]},//M1相对位置
    {0x2014,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&RelativeLocation[M2]},//M2相对位置
};
extern uint8_t LockAxleON_OFF;
index_pObjDictry ObjDict_Index2015[]=
{
    {0x2015,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&LockAxleON_OFF}, //锁轴方式：0为速度环锁轴 1为位置环锁轴
};

index_pObjDictry ObjDict_Index2016[]=
{
    {0x2016,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum6},
    {0x2016,1,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&ProgramVersion.uVersionPartOfRobotType},//车型对应的版本号
    {0x2016,2,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&ProgramVersion.uVersionPartOfMainVersion},//大版本号
    {0x2016,3,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&ProgramVersion.uVersionFullVersion},//功能版本号
    {0x2016,4,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&ProgramVersion.uVersionPartOfSmallVersion},////小版本号，代码bug以及参数微调
    {0x2016,5,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&ProgramVersion.uVersionFullVersion},//完整版本号
    {0x2016,6,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&ProgramVersion.uHardwareVersion},//硬件版本号
};
index_pObjDictry ObjDict_Index2017[]=
{
    {0x2017,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&Damping_Mode_ON_OFF},
};
index_pObjDictry ObjDict_Index2018[]=
{
    {0x2018,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2}, //电机温度
    {0x2018,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&pCtrlPar[M1].MotorTemperature},//M1温度
    {0x2018,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&pCtrlPar[M2].MotorTemperature},//M2温度
};

index_pObjDictry ObjDict_Index2019[]=
{
    {0x2019,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2}, //电机温度
    {0x2019,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M1].MaxTemperature},//M1最大温度
    {0x2019,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&MotorParameters[M2].MaxTemperature},//M2最大温度
};

index_pObjDictry ObjDict_Index2020[]=
{
    {0x2020,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj2020_OTA_Default},
};
index_pObjDictry ObjDict_Index3000[]=
{
    {0x3000,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum3},
    {0x3000,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&ProgramVersion.uVersionFullVersion},//软件版本
    {0x3000,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&ProgramVersion.uVersionFullVersion},//软件版本
    {0x3000,3,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&ProgramVersion.uHardwareVersion},   //硬件版本
};
int16_t hUpperOutputLimittemp = 32000;
ODCallback_t Index3001_callbacks[] =
{
    ODCallback_t_Index3001_Subindex0,
};
index_pObjDictry ObjDict_Index3001[]=
{
    {0x3001,0,RW,INTEGER16,SIZE_INTEGER16,PDO_BAN,(void*)&hUpperOutputLimittemp},
};

long test111[10] = {0x00};//后面删除
/**/
/*设备制造商区域0x2000 - 0x5FFF  END*/
#endif
#ifdef _OBJ_MANUFACTURER
/*DS402 BEGIN*/
short int obj6007_AbortCnect_optcode = 0x03;
index_pObjDictry ObjDict_Index6007[]=
{
    {0x6007,0,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)NULL}, //PDO可选
};
/**/
unsigned short int obj603F_errcode = 0x0000; //驱动器最近一次故障码
index_pObjDictry ObjDict_Index603F[]=
{
    {0x603F,0,RO,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&ParInNum2}, //PDO可选
    {0x603F,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&STM[M1].hFaultOccurred},
    {0x603F,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&STM[M2].hFaultOccurred},
};
/**/
/*Device Control*/
unsigned short int obj6402_motorType = NON_STANDERD_MOTOR;
index_pObjDictry ObjDict_Index6402[]=
{
    {0x6402,0,RW,UNSIGNED16,SIZE_UINT16,PDO_ALLOW,(void*)NULL}, //PDO可选
};
/**/
unsigned char obj6403_Motor_Nameplate[] = "";
index_pObjDictry ObjDict_Index6403[]=
{
    {0x6403,0,RW,UNSIGNED8_STRING,sizeof(obj6403_Motor_Nameplate),PDO_BAN,(void*)&obj6403_Motor_Nameplate},
};
/**/
unsigned char obj6404_Motor_manufacturer_name[] = "";
index_pObjDictry ObjDict_Index6404[]=
{
    {0x6404,0,RW,UNSIGNED8_STRING,sizeof(obj6404_Motor_manufacturer_name),PDO_BAN,(void*)&obj6404_Motor_manufacturer_name},
};
/**/
unsigned int obj6502_Supported_driver_mode = DRIVER_MANUFACTUREER_SPECIFIC|DRIVER_PP|DRIVER_PV|DRIVER_TQ|DRIVER_HM;//put it in INITIALIZATION
index_pObjDictry ObjDict_Index6502[]=
{
    //支持的操作模式(参见CANopen402 P39)
    {0x6502,0,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&obj6502_Supported_driver_mode}
};
/**/
unsigned int obj60FD_Digital_inputs = 0x00;
index_pObjDictry ObjDict_Index60FD[]=
{
    {0x60FD,0,RO,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&test111[0]},
};
/**/
unsigned int obj60FE_Digital_outputs_subindex = 4;//数字信号输出子索引
unsigned int obj60FE_Digital_outputs_dout = 0x00;//数字信号输出
unsigned int obj60FE_Digital_outputs_mask = 0x00;//数字信号输出掩码
extern uint8_t debug_motor1_exit;//用于测试失速问题
extern uint8_t debug_motor2_exit;//用于测试失速问题
extern int16_t debug_motor1_Angle_Compensation;//用于测试失速问题
extern int16_t debug_motor2_Angle_Compensation;//用于测试失速问题
extern uint8_t initial_positioning_timing;//初始定位问题
extern uint8_t initial2_positioning_timing;//初始定位问题
extern int16_t HAL_Init_Electrical_Angle2;
extern int16_t HAL_Init_Electrical_Angle;
index_pObjDictry ObjDict_Index60FE[]=
{
    //数字信号输出(参见CANopen402 P43)
    {0x60FE,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj60FE_Digital_outputs_subindex},//子索引数(1-2)
    {0x60FE,1,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&debug_motor1_exit},//物理输出   PDO可选 //用于测试失速问题
    {0x60FE,2,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&debug_motor2_exit},//输出位掩码 PDO可选 //用于测试失速问题
    {0x60FE,3,RW,UNSIGNED16,SIZE_UINT16,PDO_ALLOW,(void*)&debug_motor1_Angle_Compensation},//输出位掩码 PDO可选 //用于测试失速问题
    {0x60FE,4,RW,UNSIGNED16,SIZE_UINT16,PDO_ALLOW,(void*)&debug_motor2_Angle_Compensation},//输出位掩码 PDO可选 //用于测试失速问题
    {0x60FE,5,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&initial_positioning_timing},//初始定位问题
    {0x60FE,6,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&initial2_positioning_timing},//初始定位问题
		{0x60FE,7,RW,INTEGER16,INTEGER16,PDO_ALLOW,(void*)&HAL_Init_Electrical_Angle},//初始定位的位置
    {0x60FE,8,RW,INTEGER16,INTEGER16,PDO_ALLOW,(void*)&HAL_Init_Electrical_Angle2},//初始定位的位置
};
/**/
unsigned short int obj6040_Control_word = 0x00;
index_pObjDictry ObjDict_Index6040[]=
{
//    {0x6040,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x6040,0,RW,UNSIGNED16,SIZE_UINT16,PDO_ALLOW,(void*)&Driver_Data[M1].device_control_data.Conrtolword}, //M1控制字
    {0x6040,1,RW,UNSIGNED16,SIZE_UINT16,PDO_ALLOW,(void*)&Driver_Data[M2].device_control_data.Conrtolword}, //M2控制字
};
/**/
unsigned short int obj6041_Status_word = 0x00;
uint32_t independentStatusword = 0;
index_pObjDictry ObjDict_Index6041[]=
{
    {0x6041,0,RO,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&independentStatusword},
    {0x6041,1,RW,UNSIGNED16,SIZE_UINT16,PDO_ALLOW,(void*)&Driver_Data[M1].device_control_data.Statusword}, //M1状态字
    {0x6041,2,RW,UNSIGNED16,SIZE_UINT16,PDO_ALLOW,(void*)&Driver_Data[M2].device_control_data.Statusword}, //M2状态字
};
/**/
/**/
short int obj605B_ShutDown_optcode = 0x0000;//关机选项代码OPERATION_ENABLE=>READY TO SWITCH ON   opcode
index_pObjDictry ObjDict_Index605B[]=
{
    {0x605B,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x605B,1,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&Driver_Data[M1].device_control_data.Shutdown_option_code}, //M1关机选项代码
    {0x605B,2,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&Driver_Data[M2].device_control_data.Shutdown_option_code}, //M2关机选项代码
};
/**/
short int obj605C_Disable_optcode = (1<<0)&0xFFFF;//1：斜坡减速，然后关闭驱动器使能
index_pObjDictry ObjDict_Index605C[]=
{
    {0x605C,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x605C,1,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&Driver_Data[M1].device_control_data.Disable_operation_option_code}, //M1关失能选项代码
    {0x605C,2,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&Driver_Data[M2].device_control_data.Disable_operation_option_code}, //M2关失能选项代码
};

short int obj605A_QuickStop_optcode = (1<<1)&0xFFFF;//2：开启急停功能
index_pObjDictry ObjDict_Index605A[]=
{
    {0x605A,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x605A,1,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&Driver_Data[M1].device_control_data.Quick_stop_option_code}, //M1急停选项代码 1-8
    {0x605A,2,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&Driver_Data[M2].device_control_data.Quick_stop_option_code}, //M2急停选项代码
};
/**/
short int obj605D_Halt_optcode = (1<<0)&0xFFFF;//1：减速至停止状态
index_pObjDictry ObjDict_Index605D[]=
{
    {0x605D,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x605D,1,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&Driver_Data[M1].device_control_data.Halt_option_code}, //M1暂停选项代码
    {0x605D,2,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&Driver_Data[M2].device_control_data.Halt_option_code}, //M2暂停选项代码
};
/**/
short int obj605E_FaultReact_optcode = 0;//参数故障反应选项代码决定如果驱动器中发生故障，应该采取什么行动。
index_pObjDictry ObjDict_Index605E[]=
{
    {0x605E,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x605E,1,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&Driver_Data[M1].device_control_data.Fault_reaction_option_code}, //M1错误应对选项代码
    {0x605E,2,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&Driver_Data[M2].device_control_data.Fault_reaction_option_code}, //M2错误应对选项代码
};
/**/
char obj6060_Control_mode = 0x03;//操作（控制）模式  //3 速度模式  1 位置模式  4 力矩模式 默认速度模式 
index_pObjDictry ObjDict_Index6060[]=
{
//    {0x6060,0,RW,INTEGER8,SIZE_INTEGER8,PDO_ALLOW,(void*)&ParInNum2},
    {0x6060,0,RW,INTEGER8,SIZE_INTEGER8,PDO_ALLOW,(void*)&Driver_Data[M1].device_control_data.Modes_of_operation}, //M1操作模式
    {0x6060,1,RW,INTEGER8,SIZE_INTEGER8,PDO_ALLOW,(void*)&Driver_Data[M2].device_control_data.Modes_of_operation}, //M2操作模式
};
/**/
char obj6061_Control_mode_status = 0x00;//操作（控制）模式显示
index_pObjDictry ObjDict_Index6061[]=
{
    {0x6061,0,RO,INTEGER8,SIZE_INTEGER8,PDO_ALLOW,(void*)&obj6061_Control_mode_status},
    {0x6061,1,RW,INTEGER8,SIZE_INTEGER8,PDO_ALLOW,(void*)&Driver_Data[M1].device_control_data.Modes_of_operation_display}, //M1操作模式显示
    {0x6061,2,RW,INTEGER8,SIZE_INTEGER8,PDO_ALLOW,(void*)&Driver_Data[M2].device_control_data.Modes_of_operation_display}, //M2操作模式显示
};
/**/
/*FACTOR GROUP位置速度加速度等转换因子(参见CANopen402 P61)*/
char obj6089_PosNotation_index = 0x00;//Pos符号索引
index_pObjDictry ObjDict_Index6089[]=
{
    {0x6089,0,RW,INTEGER8,SIZE_INTEGER8,PDO_BAN,(void*)&obj6089_PosNotation_index},
};
/**/
unsigned char obj608A_PosDimension_index = 0x00;//Pos尺寸索引
index_pObjDictry ObjDict_Index608A[]=
{
    {0x608A,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj608A_PosDimension_index},
};
/**/
char obj608B_VelNotation_index = 0x00;//Vel符号索引
index_pObjDictry ObjDict_Index608B[]=
{
    //速度符号索引 Default:0
    {0x608B,0,RW,INTEGER8,SIZE_INTEGER8,PDO_BAN,(void*)&obj608B_VelNotation_index},
};
/**/
unsigned char obj608C_VelDimension_index = 0x00;//Vel尺寸索引
index_pObjDictry ObjDict_Index608C[]=
{
    //速度尺寸索引
    {0x608C,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj608C_VelDimension_index},
};
/**/
char obj608D_AccelNotation_index = 0x00;//Accel符号索引
index_pObjDictry ObjDict_Index608D[]=
{
    //加速度符号索引
    {0x608D,0,RW,INTEGER8,SIZE_INTEGER8,PDO_BAN,(void*)&obj608D_AccelNotation_index},
};
/**/
unsigned char obj608E_AccelDimension_index = 0x00;//Accel尺寸索引
index_pObjDictry ObjDict_Index608E[]=
{
    //加速度尺寸索引
    {0x608E,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&obj608E_AccelDimension_index},
};
/**/
unsigned int obj6093_PosFactor_subindex = 0x00000002;//0x6093 2 subindex位置因子
unsigned int obj6093_PosFactor_numerator = 0x00000001;//deault
unsigned int obj6093_PosFactor_FeedConst = 0x00000001;//deault
index_pObjDictry ObjDict_Index6093[]=
{
    {0x6093,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6093_PosFactor_subindex},//子索引数(2)
    {0x6093,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6093_PosFactor_numerator},//分子 PDO可选 Default:1
    {0x6093,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6093_PosFactor_FeedConst},//反馈常数 PDO可选 Default:1
};
/**/
unsigned int obj6094_VelFactor_subindex = 0x00000002;//0x6094 2 subindex速度因子
unsigned int obj6094_VelFactor_numerator = 0x00000001;//deault
unsigned int obj6094_VelFactor_FeedConst = 0x00000001;//deault
index_pObjDictry ObjDict_Index6094[]=
{
    {0x6094,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6094_VelFactor_subindex},//子索引数(2)
    {0x6094,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6094_VelFactor_numerator},//分子 PDO可选 Default:1
    {0x6094,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6094_VelFactor_FeedConst},//除数因子 PDO可选 Default:1
};
/**/
unsigned int obj6097_AccelFactor_subindex = 0x00000002;//0x6097 2 subindex加速度因子
unsigned int obj6097_AccelFactor_numerator = 0x00000001;//deault
unsigned int obj6097_AccelFactor_FeedConst = 0x00000001;//deault
index_pObjDictry ObjDict_Index6097[]=
{
    {0x6097,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6097_AccelFactor_subindex},//子索引数(2)
    {0x6097,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6097_AccelFactor_numerator},//分子 PDO可选 Default:1
    {0x6097,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6097_AccelFactor_FeedConst},//除数因子 PDO可选 Default:1
};
/**/
unsigned char obj607E_Pos_rotation = 0x00;//位置极性
index_pObjDictry ObjDict_Index607E[]=
{
    {0x607E,0,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&test111[0]}, //PDO可选
};
/**/
/*Profile Position Mode(PP)*/
int obj607A_Targt_Position = 0x00000000;
index_pObjDictry ObjDict_Index607A[]=
{
    {0x607A,0,RO,INTEGER8,SIZE_INTEGER8,PDO_BAN,(void*)&ParInNum2},
    {0x607A,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&pCtrlPar[M1].SetPulseMotor}, //M1目标位置
    {0x607A,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&pCtrlPar[M2].SetPulseMotor}, //M2目标位置
};
/**/
int obj607D_Croft_PosLimit = 0x2;//软件位置限位
int obj607D_Croft_PosLimit_min = -2147483647;//POS_MIN
int obj607D_Croft_PosLimit_max = 2147483647;//POS_MAX
index_pObjDictry ObjDict_Index607D[]=
{
    //软件位置限位(参见CANopen402 P80)
    {0x607D,0,RO,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&ParInNum2},//子索引数 默认2
    {0x607D,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&test111[0]},//Soft Limit_MIN PDO可选
    {0x607D,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&test111[0]},//Soft Limit_MAX PDO可选
};
/**/
unsigned int obj6081_Profile_Vel = 0x00000000;//轮廓速度
index_pObjDictry ObjDict_Index6081[]=
{
    {0x6081,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x6081,1,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&pTrapezoidalM1.Vav},  //M1位置环最大速度
    {0x6081,2,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&pTrapezoidalM2.Vav},  //M2位置环最大速度
};
/**/
unsigned int obj6083_Profile_Accel = 0;//轮廓加速度  该值在402协议当中定义位加速度，在中菱和和利时的驱动器当中该值为时间
index_pObjDictry ObjDict_Index6083[]=
{
    {0x6083,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum4},
    {0x6083,1,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&pCtrlPar[M1].hDurationms},     //M1速度环加速度时间 默认值为200
    {0x6083,2,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&pCtrlPar[M2].hDurationms},     //M2速度环加速度时间 默认值为200
    {0x6083,3,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&pTrapezoidalM1.A}, //M1位置环加速度
    {0x6083,4,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&pTrapezoidalM2.A}, //M2位置环加速度
//    {0x6083,5,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&0},    //M1位置环加速度时间 默认值为0 在位置环不起作用
//    {0x6083,6,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&0},    //M2位置环加速度时间
};
/**/
unsigned int obj6084_Profile_Decel = 0;//轮廓减速度
index_pObjDictry ObjDict_Index6084[]=
{
    //轮廓(Profile)减速度(参见CANopen402 P83)
    {0x6084,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum4},
    {0x6084,1,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&pCtrlPar[M1].hDurationms},     //M1速度环减速度时间 默认值为200
    {0x6084,2,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&pCtrlPar[M2].hDurationms},     //M2速度环减速度时间 默认值为200，为了兼容中菱驱动器
    {0x6084,3,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&pTrapezoidalM1.D},  //M1位置环减速度
    {0x6084,4,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&pTrapezoidalM2.D},  //M2位置环减速度
};
/**/
unsigned int obj6085_Profile_QuickDecel = 0x00000000;//轮廓急停减速度 两个电机共用一个
index_pObjDictry ObjDict_Index6085[]=
{
    //急停减速度(参见CANopen402 P83)
    {0x6085,0,RW,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x6085,1,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&pCtrlPar[M1].quick_stop_decel},//默认值50ms
    {0x6085,2,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&pCtrlPar[M1].quick_stop_decel},//默认值50ms
};
/**/
short int obj6086_Motion_profile_mode = LINEAR_RAMP;//Default:线性斜坡(梯形坡面)
index_pObjDictry ObjDict_Index6086[]=
{
    {0x6086,0,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&obj6086_Motion_profile_mode},//PDO可选
};
/**/
/*Homing mode*/
int obj607C_Homing_offset = 0x00000000;//回零增益
index_pObjDictry ObjDict_Index607C[]=
{
    {0x607C,0,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&obj607C_Homing_offset}, //PDO可选
};
/**/
int obj6098_Homing_mode = 0x00000000;//回零模式
index_pObjDictry ObjDict_Index6098[]=
{
    {0x6098,0,RW,INTEGER8,SIZE_INTEGER8,PDO_ALLOW,(void*)&obj6098_Homing_mode},//PDO可选
};
/**/
index_pObjDictry ObjDict_Index60C5[]=
{
    //最大加速度
    {0x60C5,0,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&test111[0]},//PDO可选
};
/**/
index_pObjDictry ObjDict_Index60C6[]=
{
    //最大减速度
    {0x60C6,0,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&test111[0]},//PDO可选
};
/**/
unsigned int obj60F6_torque_para_subindex = 2;//回零速度
index_pObjDictry ObjDict_Index60F6[]=
{
    {0x60F6,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj60F6_torque_para_subindex}, //子索引数(1-254)
    {0x60F6,1,RW,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&test111[0]},//增益(KP) PDO可选
    {0x60F6,2,RW,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&test111[0]},//积分(KI) PDO可选
};
/**/
index_pObjDictry ObjDict_Index60F7[]=
{
    //功率参数(制造商指定)
    {0x60F7,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pReserved}, //子索引数(1-254)
    {0x60F7,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&pReserved},//制造商指定
};
/**/
unsigned int obj60F9_speed_para_subindex = 3;//回零速度
index_pObjDictry ObjDict_Index60F9[]=
{
    //速度控制参数设置(制造商指定)
    {0x60F9,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj60F9_speed_para_subindex},//子索引数(1-254)
    {0x60F9,1,RW,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&test111[0]},//增益(KP) PDO可选
    {0x60F9,2,RW,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&test111[0]},//积分(KI) PDO可选
    {0x60F9,3,RW,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&pReserved},//制造商指定(3-254)
};
/**/
unsigned int obj60FB_pos_para_subindex = 2;//回零速度
index_pObjDictry ObjDict_Index60FB[]=
{
    //位置控制参数设置(制造商指定)
    {0x60FB,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj60FB_pos_para_subindex},//子索引数(1-254)
    {0x60FB,1,RW,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&test111[0]},//增益(KP) PDO可选
    {0x60FB,2,RW,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&test111[0]},//积分(KI) PDO可选
};
/**/
unsigned int obj6099_Homing_speed_subindex = 2;//回零速度
unsigned int obj6099_Homing_speed_switch = 0x00;//第一回零速度
unsigned int obj6099_Homing_speed_zero = 0x00;//第二回零速度
index_pObjDictry ObjDict_Index6099[]=
{
    //回零速度(参见CANopen402 P90)
    {0x6099,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6099_Homing_speed_subindex},//子索引数
    {0x6099,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&test111[0]},//第一回零速度 PDO可选
    {0x6099,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&test111[0]},//第二回零速度 PDO可选
};
/**/
unsigned int obj609A_Homing_Accel = 0x00;//回零加速度
index_pObjDictry ObjDict_Index609A[]=
{
    //回零加速度(参见CANopen402 P91)
    {0x609A,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj609A_Homing_Accel},//PDO可选
};
/**/
/*Position Control function*/
int obj6062_Position_demand_value = 0x00;//位置指令值
index_pObjDictry ObjDict_Index6062[]=
{
    //位置指令值(Pos Demand value)(参见CANopen402 P98)
    {0x6062,0,RO,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&obj6062_Position_demand_value},//PDO可选
};
/**/
int *obj6063_Pos_act;//实际位置地址
index_pObjDictry ObjDict_Index6063[]=
{
    //位置实际值地址(value*)(参见CANopen402 P99)
    {0x6063,0,RO,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&obj6063_Pos_act},//PDO可选
};
/**/
int obj6064_Position_act = 0x00;//实际位置
index_pObjDictry ObjDict_Index6064[]=
{
    {0x6064,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum2},
    {0x6064,1,RO,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&ENCODER_M1._Super.Actual_Position},//M1实际位置
    {0x6064,2,RO,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&ENCODER_M2._Super.Actual_Position},//M2实际位置
};
/**/
unsigned int obj6065_Follow_Err_window = 1000;//跟踪误差窗（允许位置跟踪误差）
index_pObjDictry ObjDict_Index6065[]=
{
    {0x6065,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6065_Follow_Err_window},//PDO可选
};
/**/
unsigned short int obj6066_Follow_Err_window_timeout = 3000;//ms
index_pObjDictry ObjDict_Index6066[]=
{
    {0x6066,0,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&obj6066_Follow_Err_window_timeout},//PDO可选
};
/**/
unsigned int obj6067_Position_window = 1000;//位置窗
index_pObjDictry ObjDict_Index6067[]=
{
    {0x6067,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6067_Position_window},//PDO可选
};
/**/
unsigned short int obj6068_Position_window_time = 1;//位置窗时间ms
index_pObjDictry ObjDict_Index6068[]=
{
    {0x6068,0,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&obj6068_Position_window_time},//PDO可选
};
/**/
unsigned short int obj60F4_Follow_Err_act = 0x00;//跟踪误差实际值
index_pObjDictry ObjDict_Index60F4[]=
{
    //跟踪误差实际值
    {0x60F4,0,RO,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&obj60F4_Follow_Err_act},//PDO可选
};
/**/
int *obj60FC_PosDemand_value;//位置指令值地址
index_pObjDictry ObjDict_Index60FC[]=
{
    //位置指令值地址(value*)(参见CANopen402 P103)
    {0x60FC,0,RW,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&obj60FC_PosDemand_value},//PDO可选
};
/**/
/*Interpolated position mode(IP)*/
//##
/*Profile Velocity Mode(PV) 轮廓速度模式 参见CANopen402 P120*/
int obj6069_Vel_sensor_act = 0x00;//速度传感器值(counts/s)
index_pObjDictry ObjDict_Index6069[]=
{
    {0x6069,0,RO,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&obj6069_Vel_sensor_act}, //PDO可选
};
/**/
int obj606B_Vel_demand_value = 0x00;//位置轨迹发生器输出值JOG
index_pObjDictry ObjDict_Index606B[]=
{
    //速度指令值（位置轨迹发生器输出值）
    {0x606B,0,RO,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&obj606B_Vel_demand_value}, //PDO可选
};
/**/
int obj606C_Vel_actual = 0x00;//速度实际值
index_pObjDictry ObjDict_Index606C[]=
{
    {0x606C,0,RO,UNSIGNED8,SIZE_UINT8,PDO_BAN,(void*)&ParInNum3},
    {0x606C,1,RO,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&pCtrlPar[M1]._Vel_PLL_Motor}, //M1实际速度
    {0x606C,2,RO,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&pCtrlPar[M2]._Vel_PLL_Motor}, //M2实际速度
    {0x606C,3,RO,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&pCtrlPar[M1].M1M2_VelRel},   //M1M2实际速度组合
};
/**/
short int obj606D_Vel_window = 0x00;//速度窗
index_pObjDictry ObjDict_Index606D[]=
{
    //速度窗
    {0x606D,0,RW,INTEGER16,SIZE_INTEGER16,PDO_BAN,(void*)&obj606D_Vel_window}, //PDO可选
};
/**/
unsigned short int obj606E_Vel_window_time = 0x00;//速度窗时间
index_pObjDictry ObjDict_Index606E[]=
{
    //速度窗时间
    {0x606E,0,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&obj606E_Vel_window_time},//PDO可选
};
/**/
unsigned short int obj606F_Vel_threshold = 0x00;//速度阈值
index_pObjDictry ObjDict_Index606F[]=
{
    //速度阈值(参见CANopen402 P127)
    {0x606F,0,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&obj606F_Vel_threshold},//PDO可选
};
/**/
unsigned short int obj6070_Vel_threshold_time = 0x00;
index_pObjDictry ObjDict_Index6070[]=
{
    //速度阈值时间(参见CANopen402 P128)
    {0x6070,0,RW,UNSIGNED16,SIZE_UINT16,PDO_BAN,(void*)&obj6070_Vel_threshold_time},//PDO可选
};
/**/
int obj60FF_Vel_Targt = 0x00;//目标速度

//extern int32_t SetSpeedMotor1;
//extern int32_t SetSpeedMotor2;
index_pObjDictry ObjDict_Index60FF[]=
{
    {0x60FF,0,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&ParInNum3},
    {0x60FF,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&pCtrlPar[M1].SetVelMotor}, //M1目标速度
    {0x60FF,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&pCtrlPar[M2].SetVelMotor}, //M2目标速度
    {0x60FF,3,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&pCtrlPar[M1].M1M2_VelSet},    //同步模式下的两个电机的速度高16位代表M1电机低16位代表M2电机转速
};
/**/
int obj60F8_Vel_Max_slippage = 0x00;//最大速度偏差
index_pObjDictry ObjDict_Index60F8[]=
{
    //最大滑移(最大速度偏差)
    {0x60F8,0,RO,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&obj60F8_Vel_Max_slippage}, //PDO可选
};
/**/
/*Profile Torque Mode(PT) 轮廓力矩模式(参见CANopen402 P131)*/
short int  obj6071_Torque_Targt = 0x00;//目标力矩
index_pObjDictry ObjDict_Index6071[]=
{
    {0x6071,0,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&ParInNum2},
    {0x6071,1,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&pCtrlPar[M1].SetTorqueMotor}, //M1目标力矩
    {0x6071,2,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&pCtrlPar[M2].SetTorqueMotor}, //M2目标力矩
};
/**/
unsigned short int obj6072_Max_Torque = 0x00;//最大力矩
index_pObjDictry ObjDict_Index6072[]=
{
    //最大力矩
    {0x6072,0,RW,UNSIGNED16,SIZE_UINT16,PDO_ALLOW,(void*)&ParInNum2}, //PDO可选
    {0x6072,1,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&pCtrlPar[M1].torqueLimit}, //M1最大力矩
    {0x6072,2,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&pCtrlPar[M2].torqueLimit}, //M2最大力矩
};
/**/
unsigned short int obj6073_Max_Current = 0x00;//最大电流
index_pObjDictry ObjDict_Index6073[]=
{
    //最大电流
    {0x6073,0,RW,UNSIGNED16,SIZE_UINT16,PDO_ALLOW,(void*)&obj6073_Max_Current}, //PDO可选
};
/**/
short int  obj6074_Torque_demand_value = 0x00;//力矩指令值
index_pObjDictry ObjDict_Index6074[]=
{
    {0x6074,0,RO,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&obj6074_Torque_demand_value}, //PDO可选
};
/**/
unsigned int obj6075_rated_Current = 0x00;//电机额定电流
index_pObjDictry ObjDict_Index6075[]=
{
    {0x6075,0,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&obj6075_rated_Current}, //PDO可选
};
/**/
unsigned int obj6076_rated_torque = 0x00;//电机额定力矩
index_pObjDictry ObjDict_Index6076[]=
{
    {0x6076,0,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&obj6076_rated_torque}, //PDO可选
};
/**/
short int  obj6077_Torque_act = 0x00;
index_pObjDictry ObjDict_Index6077[]=
{
    {0x6077,0,RO,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&obj6077_Torque_act}, //PDO可选
};
/**/
short int  obj6078_Current_act = 0x00;
index_pObjDictry ObjDict_Index6078[]=
{
    {0x6078,0,RW,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&ParInNum2},
    {0x6078,1,RO,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&FOCVars[M1].Iqd.q}, //M1实际电流值
    {0x6078,2,RO,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&FOCVars[M2].Iqd.q}, //M2实际电流值
};
/**/
unsigned int obj6079_Voltage_bus = 0x00;//母线电压
index_pObjDictry ObjDict_Index6079[]=
{
    //母线电压
    {0x6079,0,RO,INTEGER32,SIZE_INTEGER32,PDO_ALLOW,(void*)&VBS_AvBusVoltage_V}, //PDO可选
};
/**/
unsigned int obj6087_Torque_slope = 0x00;//力矩轮廓类型
index_pObjDictry ObjDict_Index6087[]=
{
    {0x6087,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6087_Torque_slope}, //PDO可选
};
/**/
short int  obj6088_Torque_profile_type = 0x00;//力矩轮廓类型
index_pObjDictry ObjDict_Index6088[]=
{
    {0x6088,0,RW,INTEGER16,SIZE_INTEGER16,PDO_BAN,(void*)&obj6088_Torque_profile_type}, //PDO可选
};
/**/
/*Veocity Mode(VL) 速度模式(参见CANopen402 P141)*/
short int obj6042_Vlvel_targt = 0x00;   //vl目标速度
index_pObjDictry ObjDict_Index6042[]=
{
    {0x6042,0,RW,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&obj6042_Vlvel_targt}, //PDO可选
};
/**/
short int obj6043_Vlvel_demand_value = 0x00;//vl速度指令
index_pObjDictry ObjDict_Index6043[]=
{
    //vl 速度指令(该值是ramp斜坡功能提供的瞬时值)(参见CANopen402 P148)
    {0x6043,0,RO,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&obj6043_Vlvel_demand_value}, //PDO可选
};
/**/
short int obj6053_Vl_percentage_demand = 0x00;//vl百分比指令
index_pObjDictry ObjDict_Index6053[]=
{
    //vl百分比指令(是ramp斜坡函数提供的速度，以百分比表示，值16383对应100%的vl速度参考)(参见CANopen402 P148)
    {0x6053,0,RO,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&obj6053_Vl_percentage_demand}, //PDO可选
};
/**/
short int obj6054_Vl_percentage_act = 0x00;//vl实际百分比
index_pObjDictry ObjDict_Index6054[]=
{
    //vl实际百分比
    {0x6054,0,RO,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&obj6054_Vl_percentage_act}, //PDO可选
};
/**/
short int obj6055_Vl_manipulated_percentage = 0x00;//vl操作百分比
index_pObjDictry ObjDict_Index6055[]=
{
    //vl操作百分比(是以vl操作速度的基础计算出来的)
    {0x6055,0,RO,INTEGER16,SIZE_INTEGER16,PDO_ALLOW,(void*)&obj6055_Vl_manipulated_percentage}, //PDO可选
};
/**/
unsigned int obj604E_Vlvel_reference = 0x00;//vl速度参考
index_pObjDictry ObjDict_Index604E[]=
{
    //vl 速度参考(参见CANopen402 P150)
    {0x604E,0,RW,UNSIGNED32,SIZE_UINT32,PDO_ALLOW,(void*)&obj604E_Vlvel_reference}, //PDO可选
};
/**/
int obj604C_Vl_DimFactor_subindex = 2;//vl尺寸因子
int obj604C_Vl_DimFactor_numerator = 0x01;
int obj604C_Vl_DimFactor_denominator = 0x01;
index_pObjDictry ObjDict_Index604C[]=
{
    {0x604C,0,RO,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&obj604C_Vl_DimFactor_subindex}, //子索引数(2)(参见CANopen402 P150)
    {0x604C,1,RW,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&obj604C_Vl_DimFactor_numerator}, //尺寸因子分子 PDO可选
    {0x604C,2,RW,INTEGER32,SIZE_INTEGER32,PDO_BAN,(void*)&obj604C_Vl_DimFactor_denominator}, //尺寸因子分母 PDO可选
};
/**/
short int obj604B_Vl_SetPointFactor_subindex = 2;//vl设定点因子
short int obj604B_Vl_SetPointFactor_numerato = 0x01;
short int obj604B_Vl_SetPointFactor_denominator = 0x01;
index_pObjDictry ObjDict_Index604B[]=
{
    //vl设定点因子，由分子分母得到，不为0，用于修改设定点的分辨率或定向范围，只用在计算指定的设定值和速度函数的输出中
    {0x604B,0,RO,INTEGER16,SIZE_INTEGER16,PDO_BAN,(void*)&obj604B_Vl_SetPointFactor_subindex}, //子索引数(2)(参见CANopen402 P152)
    {0x604B,1,RW,INTEGER16,SIZE_INTEGER16,PDO_BAN,(void*)&obj604B_Vl_SetPointFactor_numerato}, //尺寸因子分子 PDO可选
    {0x604B,2,RW,INTEGER16,SIZE_INTEGER16,PDO_BAN,(void*)&obj604B_Vl_SetPointFactor_denominator}, //尺寸因子分母 PDO可选
};
/**/
unsigned char obj604D_Pole_Number = 0x00;//vl极对数
index_pObjDictry ObjDict_Index604D[]=
{
    {0x604D,0,RW,UNSIGNED8,SIZE_UINT8,PDO_ALLOW,(void*)&obj604D_Pole_Number}, //PDO可选
};
/**/
unsigned int obj6046_Vlvel_MinMax_amount_subindex = 2;//vl速度最小最大数量
unsigned int obj6046_Vlvel_Min_amount = 0x00;
unsigned int obj6046_Vlvel_Max_amount = 0x00;
index_pObjDictry ObjDict_Index6046[]=
{
    //vl速度最小最大数量(参见CANopen402 P153)
    {0x6046,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6046_Vlvel_MinMax_amount_subindex}, //子索引数(2)
    {0x6046,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6046_Vlvel_Min_amount}, //最小数量 PDO可选
    {0x6046,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6046_Vlvel_Max_amount}, //最大数量 PDO可选
};
/**/
unsigned int obj6047_Vlvel_MinMax_subindex = 4;//vl速度最小最大值
unsigned int obj6047_Vlvel_Min_Pos = 0x00;
unsigned int obj6047_Vlvel_Max_Pos = 0x00;
unsigned int obj6047_Vlvel_Min_neg = 0x00;
unsigned int obj6047_Vlvel_Max_neg = 0x00;
index_pObjDictry ObjDict_Index6047[]=
{
    //vl速度最小最大值
    {0x6047,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6047_Vlvel_MinMax_subindex}, //子索引数(4)
    {0x6047,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6047_Vlvel_Min_Pos}, //最小值POS PDO可选
    {0x6047,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6047_Vlvel_Max_Pos}, //最大值POS PDO可选
    {0x6047,3,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6047_Vlvel_Min_neg}, //最小值NEG PDO可选
    {0x6047,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6047_Vlvel_Max_neg}, //最大值NEG PDO可选
};
/**/
unsigned int obj6058_VlfreqMotor_MinMax_amount_subindex = 2;//vl电机频率最小最大数量
unsigned int obj6058_VlfreqMotor_Min_amount = 0x00;
unsigned int obj6058_VlfreqMotor_Max_amount = 0x00;
index_pObjDictry ObjDict_Index6058[]=
{
    //vl频率电机最小最大数量(将该值内部映射到相应的速度对象)
    {0x6058,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6058_VlfreqMotor_MinMax_amount_subindex}, //子索引数(2)
    {0x6058,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6058_VlfreqMotor_Min_amount}, //最小数量 PDO可选
    {0x6058,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6058_VlfreqMotor_Max_amount}, //最大数量 PDO可选
};
/**/
unsigned int obj6059_VlfreqMotor_MinMax_subindex = 4;//vl电机频率最小最大值
unsigned int obj6059_VlfreqMotor_Min_Pos = 0x00;
unsigned int obj6059_VlfreqMotor_Max_Pos = 0x00;
unsigned int obj6059_VlfreqMotor_Min_neg = 0x00;
unsigned int obj6059_VlfreqMotor_Max_neg = 0x00;
index_pObjDictry ObjDict_Index6059[]=
{
    //vl频率电机最小最大值
    {0x6059,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6059_VlfreqMotor_MinMax_subindex}, //子索引数(4)
    {0x6059,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6059_VlfreqMotor_Min_Pos}, //最小值POS PDO可选
    {0x6059,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6059_VlfreqMotor_Max_Pos}, //最大值POS PDO可选
    {0x6059,3,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6059_VlfreqMotor_Min_neg}, //最小值NEG PDO可选
    {0x6059,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6059_VlfreqMotor_Max_neg}, //最大值NEG PDO可选
};
/**/
unsigned int obj6056_VlvelMotor_MinMax_amount_subindex = 2;//vl速度电机最小最大数量
unsigned int obj6056_VlvelMotor_Min_amount = 0x00;
unsigned int obj6056_VlvelMotor_Max_amount = 0x00;
index_pObjDictry ObjDict_Index6056[]=
{
    //vl速度电机最小最大数量
    {0x6056,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6056_VlvelMotor_MinMax_amount_subindex}, //子索引数(2)
    {0x6056,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6056_VlvelMotor_Min_amount}, //最小数量 PDO可选
    {0x6056,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6056_VlvelMotor_Max_amount}, //最大数量 PDO可选
};
/**/
unsigned int obj6057_VlvelMotor_MinMax_subindex = 4;//vl速度电机最小最大值
unsigned int obj6057_VlvelMotor_Min_Pos = 0x00;
unsigned int obj6057_VlvelMotor_Max_Pos = 0x00;
unsigned int obj6057_VlvelMotor_Min_neg = 0x00;
unsigned int obj6057_VlvelMotor_Max_neg = 0x00;
index_pObjDictry ObjDict_Index6057[]=
{
    //vl速度电机最小最大值
    {0x6057,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6057_VlvelMotor_MinMax_subindex}, //子索引数(4)
    {0x6057,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6057_VlvelMotor_Min_Pos}, //最小值POS PDO可选
    {0x6057,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6057_VlvelMotor_Max_Pos}, //最大值POS PDO可选
    {0x6057,3,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6057_VlvelMotor_Min_neg}, //最小值NEG PDO可选
    {0x6057,4,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6057_VlvelMotor_Max_neg}, //最大值NEG PDO可选
};
/**/
unsigned int obj6048_Vlvel_Accel_subindex = 2;//vl速度加速度
unsigned int obj6048_Vlvel_Accel_deltaSpeed = 0x00;
unsigned int obj6048_Vlvel_Accel_deltaTime = 0x00;
index_pObjDictry ObjDict_Index6048[]=
{
    //vl速度加速度(参见CANopen402 P162)
    {0x6048,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6048_Vlvel_Accel_subindex}, //子索引数(2)
    {0x6048,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6048_Vlvel_Accel_deltaSpeed}, //Delta speed(速度增量) PDO可选
    {0x6048,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6048_Vlvel_Accel_deltaTime}, //Delta time(时间增量) PDO可选
};
/**/
unsigned int obj6049_Vlvel_Decel_subindex = 2;//vl速度减速度
unsigned int obj6049_Vlvel_Decel_deltaSpeed = 0x00;
unsigned int obj6049_Vlvel_Decel_deltaTime = 0x00;
index_pObjDictry ObjDict_Index6049[]=
{
    //vl速度减速度(参见CANopen402 P164)
    {0x6049,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6049_Vlvel_Decel_subindex}, //子索引数(2)
    {0x6049,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6049_Vlvel_Decel_deltaSpeed}, //Delta speed(速度增量) PDO可选
    {0x6049,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6049_Vlvel_Decel_deltaTime}, //Delta time(时间增量) PDO可选
};
/**/
unsigned int obj604A_Vlvel_QuickStop_subindex = 2;//vl速度急停
unsigned int obj604A_Vlvel_QuickStop_deltaSpeed = 0x00;
unsigned int obj604A_Vlvel_QuickStop_deltaTime = 0x00;
index_pObjDictry ObjDict_Index604A[]=
{
    //vl速度急停
    {0x604A,0,RO,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj604A_Vlvel_QuickStop_subindex}, //子索引数(2)
    {0x604A,1,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj604A_Vlvel_QuickStop_deltaSpeed}, //Delta speed(速度增量) PDO可选
    {0x604A,2,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj604A_Vlvel_QuickStop_deltaTime}, //Delta time(时间增量) PDO可选
};
/**/
unsigned int obj604F_Vl_SlopeFunction_time = 0x00;//vl斜坡功能时间
index_pObjDictry ObjDict_Index604F[]=
{
    //vl ramp斜坡功能时间，指驱动器从0启动到vl速度参考的时间(参见CANopen402 P166)
    {0x604F,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj604F_Vl_SlopeFunction_time},//PDO可选
};
/**/
unsigned int obj6050_Vl_Decel_time = 0x00;//vl减速时间
index_pObjDictry ObjDict_Index6050[]=
{
    //vl减速时间，指驱动器从vl速度基准减速到0的时间
    {0x6050,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6050_Vl_Decel_time},//PDO可选
};
/**/
unsigned int obj6051_Vl_QuickStop_time = 0x00;//vl急停时间
index_pObjDictry ObjDict_Index6051[]=
{
    //vl急停时间，指驱动器从vl速度基准减速到0的时间
    {0x6051,0,RW,UNSIGNED32,SIZE_UINT32,PDO_BAN,(void*)&obj6051_Vl_QuickStop_time},//PDO可选
};
/**/
short int obj6044_Vl_Control_effort = 0x00;//vl控制增益
index_pObjDictry ObjDict_Index6044[]=
{
    //vl控制输出，是指电机轴或负载的速度比例到vl目标速度的单位
    {0x6044,0,RO,INTEGER16,SIZE_INTEGER16,PDO_BAN,(void*)&obj6044_Vl_Control_effort}, //PDO可选
};
/**/
short int obj6045_Vlvel_Manipulated = 0x00;//vl操作速度
index_pObjDictry ObjDict_Index6045[]=
{
    //vl操作速度，是带补偿值的电机轴或负载的速度缩放到vl目标速度的单位，补偿值由控制函数生成(参见CANopen402 P169)
    {0x6045,0,RO,INTEGER16,SIZE_INTEGER16,PDO_BAN,(void*)&obj6045_Vlvel_Manipulated}, //PDO可选
};
/**/
short int obj6052_Vl_Nominal_percentage = 0x00;//vl名义百分比
index_pObjDictry ObjDict_Index6052[]=
{
    //vl名义百分比
    {0x6052,0,RW,INTEGER16,SIZE_INTEGER16,PDO_BAN,(void*)&obj6052_Vl_Nominal_percentage},//PDO可选
};
/**/
/*DS402 END*/
#endif
#endif

/* Obj Dictionary Declar */
const indextable pObjDict[] =
{
    {NULL, 0, 0x0},
    {(index_pObjDictry *)ObjDict_Index1000, sizeof(ObjDict_Index1000) / sizeof(ObjDict_Index1000[0]), 0x1000},
    {(index_pObjDictry *)ObjDict_Index1001, sizeof(ObjDict_Index1001) / sizeof(ObjDict_Index1001[0]), 0x1001},
    {(index_pObjDictry *)ObjDict_Index1002, sizeof(ObjDict_Index1002) / sizeof(ObjDict_Index1002[0]), 0x1002},
    {(index_pObjDictry *)ObjDict_Index1003, sizeof(ObjDict_Index1003) / sizeof(ObjDict_Index1003[0]), 0x1003},
    {(index_pObjDictry *)ObjDict_Index1005, sizeof(ObjDict_Index1005) / sizeof(ObjDict_Index1005[0]), 0x1005},
    {(index_pObjDictry *)ObjDict_Index1006, sizeof(ObjDict_Index1006) / sizeof(ObjDict_Index1006[0]), 0x1006},
    {(index_pObjDictry *)ObjDict_Index1007, sizeof(ObjDict_Index1007) / sizeof(ObjDict_Index1007[0]), 0x1007},
    {(index_pObjDictry *)ObjDict_Index1008, sizeof(ObjDict_Index1008) / sizeof(ObjDict_Index1008[0]), 0x1008},
    {(index_pObjDictry *)ObjDict_Index1009, sizeof(ObjDict_Index1009) / sizeof(ObjDict_Index1009[0]), 0x1009},
    {(index_pObjDictry *)ObjDict_Index100A, sizeof(ObjDict_Index100A) / sizeof(ObjDict_Index100A[0]), 0x100A},
    {(index_pObjDictry *)ObjDict_Index100C, sizeof(ObjDict_Index100C) / sizeof(ObjDict_Index100C[0]), 0x100C},
    {(index_pObjDictry *)ObjDict_Index100D, sizeof(ObjDict_Index100D) / sizeof(ObjDict_Index100D[0]), 0x100D},
    {(index_pObjDictry *)ObjDict_Index1010, sizeof(ObjDict_Index1010) / sizeof(ObjDict_Index1010[0]), 0x1010},
    {(index_pObjDictry *)ObjDict_Index1011, sizeof(ObjDict_Index1011) / sizeof(ObjDict_Index1001[0]), 0x1011},
    {(index_pObjDictry *)ObjDict_Index1012, sizeof(ObjDict_Index1012) / sizeof(ObjDict_Index1012[0]), 0x1012},
    {(index_pObjDictry *)ObjDict_Index1013, sizeof(ObjDict_Index1013) / sizeof(ObjDict_Index1013[0]), 0x1013},
    {(index_pObjDictry *)ObjDict_Index1014, sizeof(ObjDict_Index1014) / sizeof(ObjDict_Index1014[0]), 0x1014},
    {(index_pObjDictry *)ObjDict_Index1015, sizeof(ObjDict_Index1015) / sizeof(ObjDict_Index1015[0]), 0x1015},
    {(index_pObjDictry *)ObjDict_Index1016, sizeof(ObjDict_Index1016) / sizeof(ObjDict_Index1016[0]), 0x1016},
    {(index_pObjDictry *)ObjDict_Index1017, sizeof(ObjDict_Index1017) / sizeof(ObjDict_Index1017[0]), 0x1017},
    {(index_pObjDictry *)ObjDict_Index1018, sizeof(ObjDict_Index1018) / sizeof(ObjDict_Index1018[0]), 0x1018},
    {(index_pObjDictry *)ObjDict_Index1019, sizeof(ObjDict_Index1019) / sizeof(ObjDict_Index1019[0]), 0x1019},
    {(index_pObjDictry *)ObjDict_Index1028, sizeof(ObjDict_Index1028) / sizeof(ObjDict_Index1028[0]), 0x1028},
    {(index_pObjDictry *)ObjDict_Index1200, sizeof(ObjDict_Index1200) / sizeof(ObjDict_Index1200[0]), 0x1200},
    {(index_pObjDictry *)ObjDict_Index1280, sizeof(ObjDict_Index1280) / sizeof(ObjDict_Index1280[0]), 0x1000},
    {(index_pObjDictry *)ObjDict_Index1400, sizeof(ObjDict_Index1400) / sizeof(ObjDict_Index1400[0]), 0x1400},
    {(index_pObjDictry *)ObjDict_Index1401, sizeof(ObjDict_Index1401) / sizeof(ObjDict_Index1401[0]), 0x1401},
    {(index_pObjDictry *)ObjDict_Index1402, sizeof(ObjDict_Index1402) / sizeof(ObjDict_Index1402[0]), 0x1402},
    {(index_pObjDictry *)ObjDict_Index1403, sizeof(ObjDict_Index1403) / sizeof(ObjDict_Index1403[0]), 0x1403},
    {(index_pObjDictry *)ObjDict_Index1600, sizeof(ObjDict_Index1600) / sizeof(ObjDict_Index1600[0]), 0x1600},
    {(index_pObjDictry *)ObjDict_Index1601, sizeof(ObjDict_Index1601) / sizeof(ObjDict_Index1601[0]), 0x1601},
    {(index_pObjDictry *)ObjDict_Index1602, sizeof(ObjDict_Index1602) / sizeof(ObjDict_Index1602[0]), 0x1602},
    {(index_pObjDictry *)ObjDict_Index1603, sizeof(ObjDict_Index1603) / sizeof(ObjDict_Index1603[0]), 0x1603},
    {(index_pObjDictry *)ObjDict_Index1800, sizeof(ObjDict_Index1800) / sizeof(ObjDict_Index1800[0]), 0x1800},
    {(index_pObjDictry *)ObjDict_Index1801, sizeof(ObjDict_Index1801) / sizeof(ObjDict_Index1801[0]), 0x1801},
    {(index_pObjDictry *)ObjDict_Index1802, sizeof(ObjDict_Index1802) / sizeof(ObjDict_Index1802[0]), 0x1802},
    {(index_pObjDictry *)ObjDict_Index1803, sizeof(ObjDict_Index1803) / sizeof(ObjDict_Index1803[0]), 0x1803},
    {(index_pObjDictry *)ObjDict_Index1A00, sizeof(ObjDict_Index1A00) / sizeof(ObjDict_Index1A00[0]), 0x1A00},
    {(index_pObjDictry *)ObjDict_Index1A01, sizeof(ObjDict_Index1A01) / sizeof(ObjDict_Index1A01[0]), 0x1A01},
    {(index_pObjDictry *)ObjDict_Index1A02, sizeof(ObjDict_Index1A02) / sizeof(ObjDict_Index1A02[0]), 0x1A02},
    {(index_pObjDictry *)ObjDict_Index1A03, sizeof(ObjDict_Index1A03) / sizeof(ObjDict_Index1A03[0]), 0x1A03},

    {(index_pObjDictry *)ObjDict_Index2000, sizeof(ObjDict_Index2000) / sizeof(ObjDict_Index2000[0]), 0x2000},
    {(index_pObjDictry *)ObjDict_Index2001, sizeof(ObjDict_Index2001) / sizeof(ObjDict_Index2001[0]), 0x2001},
    {(index_pObjDictry *)ObjDict_Index2002, sizeof(ObjDict_Index2002) / sizeof(ObjDict_Index2002[0]), 0x2002},
    {(index_pObjDictry *)ObjDict_Index2003, sizeof(ObjDict_Index2003) / sizeof(ObjDict_Index2003[0]), 0x2003},
    {(index_pObjDictry *)ObjDict_Index2004, sizeof(ObjDict_Index2004) / sizeof(ObjDict_Index2004[0]), 0x2004},
    {(index_pObjDictry *)ObjDict_Index2005, sizeof(ObjDict_Index2005) / sizeof(ObjDict_Index2005[0]), 0x2005},
    {(index_pObjDictry *)ObjDict_Index2006, sizeof(ObjDict_Index2006) / sizeof(ObjDict_Index2006[0]), 0x2006},
    {(index_pObjDictry *)ObjDict_Index2007, sizeof(ObjDict_Index2007) / sizeof(ObjDict_Index2007[0]), 0x2007},
    {(index_pObjDictry *)ObjDict_Index2008, sizeof(ObjDict_Index2008) / sizeof(ObjDict_Index2008[0]), 0x2008},
    {(index_pObjDictry *)ObjDict_Index2009, sizeof(ObjDict_Index2009) / sizeof(ObjDict_Index2009[0]), 0x2009},
    {(index_pObjDictry *)ObjDict_Index200A, sizeof(ObjDict_Index200A) / sizeof(ObjDict_Index200A[0]), 0x200A},
    {(index_pObjDictry *)ObjDict_Index200B, sizeof(ObjDict_Index200B) / sizeof(ObjDict_Index200B[0]), 0x200B},
    {(index_pObjDictry *)ObjDict_Index200C, sizeof(ObjDict_Index200C) / sizeof(ObjDict_Index200C[0]), 0x200C},
    {(index_pObjDictry *)ObjDict_Index200D, sizeof(ObjDict_Index200D) / sizeof(ObjDict_Index200D[0]), 0x200D},
    {(index_pObjDictry *)ObjDict_Index200E, sizeof(ObjDict_Index200E) / sizeof(ObjDict_Index200E[0]), 0x200E},
    {(index_pObjDictry *)ObjDict_Index200F, sizeof(ObjDict_Index200F) / sizeof(ObjDict_Index200F[0]), 0x200F},
    {(index_pObjDictry *)ObjDict_Index2010, sizeof(ObjDict_Index2010) / sizeof(ObjDict_Index2010[0]), 0x2010},
    {(index_pObjDictry *)ObjDict_Index2011, sizeof(ObjDict_Index2011) / sizeof(ObjDict_Index2011[0]), 0x2011},
    {(index_pObjDictry *)ObjDict_Index2012, sizeof(ObjDict_Index2012) / sizeof(ObjDict_Index2012[0]), 0x2012},
    {(index_pObjDictry *)ObjDict_Index2013, sizeof(ObjDict_Index2013) / sizeof(ObjDict_Index2013[0]), 0x2013},
    {(index_pObjDictry *)ObjDict_Index2014, sizeof(ObjDict_Index2014) / sizeof(ObjDict_Index2014[0]), 0x2014},
    {(index_pObjDictry *)ObjDict_Index2015, sizeof(ObjDict_Index2015) / sizeof(ObjDict_Index2015[0]), 0x2015},
    {(index_pObjDictry *)ObjDict_Index2016, sizeof(ObjDict_Index2016) / sizeof(ObjDict_Index2016[0]), 0x2016},
    {(index_pObjDictry *)ObjDict_Index2017, sizeof(ObjDict_Index2017) / sizeof(ObjDict_Index2017[0]), 0x2017},
    {(index_pObjDictry *)ObjDict_Index2018, sizeof(ObjDict_Index2018) / sizeof(ObjDict_Index2018[0]), 0x2018},
    {(index_pObjDictry *)ObjDict_Index2019, sizeof(ObjDict_Index2019) / sizeof(ObjDict_Index2019[0]), 0x2019},
    {(index_pObjDictry *)ObjDict_Index2020, sizeof(ObjDict_Index2020) / sizeof(ObjDict_Index2020[0]), 0x2020},
    {(index_pObjDictry *)ObjDict_Index3000, sizeof(ObjDict_Index3000) / sizeof(ObjDict_Index3000[0]), 0x3000},
    {(index_pObjDictry *)ObjDict_Index3001, sizeof(ObjDict_Index3001) / sizeof(ObjDict_Index3001[0]), 0x3001},

    {(index_pObjDictry *)ObjDict_Index6007, sizeof(ObjDict_Index6007) / sizeof(ObjDict_Index6007[0]), 0x6007},
    {(index_pObjDictry *)ObjDict_Index603F, sizeof(ObjDict_Index603F) / sizeof(ObjDict_Index603F[0]), 0x603F},
    {(index_pObjDictry *)ObjDict_Index6040, sizeof(ObjDict_Index6040) / sizeof(ObjDict_Index6040[0]), 0x6040},
    {(index_pObjDictry *)ObjDict_Index6041, sizeof(ObjDict_Index6041) / sizeof(ObjDict_Index6041[0]), 0x6041},
    {(index_pObjDictry *)ObjDict_Index6042, sizeof(ObjDict_Index6042) / sizeof(ObjDict_Index6042[0]), 0x6042},
    {(index_pObjDictry *)ObjDict_Index6043, sizeof(ObjDict_Index6043) / sizeof(ObjDict_Index6043[0]), 0x6043},
    {(index_pObjDictry *)ObjDict_Index6044, sizeof(ObjDict_Index6044) / sizeof(ObjDict_Index6044[0]), 0x6044},
    {(index_pObjDictry *)ObjDict_Index6045, sizeof(ObjDict_Index6045) / sizeof(ObjDict_Index6045[0]), 0x6045},
    {(index_pObjDictry *)ObjDict_Index6046, sizeof(ObjDict_Index6046) / sizeof(ObjDict_Index6046[0]), 0x6046},
    {(index_pObjDictry *)ObjDict_Index6047, sizeof(ObjDict_Index6047) / sizeof(ObjDict_Index6047[0]), 0x6047},
    {(index_pObjDictry *)ObjDict_Index6048, sizeof(ObjDict_Index6048) / sizeof(ObjDict_Index6048[0]), 0x6048},
    {(index_pObjDictry *)ObjDict_Index6049, sizeof(ObjDict_Index6049) / sizeof(ObjDict_Index6049[0]), 0x6049},
    {(index_pObjDictry *)ObjDict_Index604A, sizeof(ObjDict_Index604A) / sizeof(ObjDict_Index604A[0]), 0x604A},
    {(index_pObjDictry *)ObjDict_Index604B, sizeof(ObjDict_Index604B) / sizeof(ObjDict_Index604B[0]), 0x604B},
    {(index_pObjDictry *)ObjDict_Index604C, sizeof(ObjDict_Index604C) / sizeof(ObjDict_Index604C[0]), 0x604C},
    {(index_pObjDictry *)ObjDict_Index604D, sizeof(ObjDict_Index604D) / sizeof(ObjDict_Index604D[0]), 0x604D},
    {(index_pObjDictry *)ObjDict_Index604E, sizeof(ObjDict_Index604E) / sizeof(ObjDict_Index604E[0]), 0x604E},
    {(index_pObjDictry *)ObjDict_Index604F, sizeof(ObjDict_Index604F) / sizeof(ObjDict_Index604F[0]), 0x604F},
    {(index_pObjDictry *)ObjDict_Index6050, sizeof(ObjDict_Index6050) / sizeof(ObjDict_Index6050[0]), 0x6050},
    {(index_pObjDictry *)ObjDict_Index6051, sizeof(ObjDict_Index6051) / sizeof(ObjDict_Index6051[0]), 0x6051},
    {(index_pObjDictry *)ObjDict_Index6052, sizeof(ObjDict_Index6052) / sizeof(ObjDict_Index6052[0]), 0x6052},
    {(index_pObjDictry *)ObjDict_Index6053, sizeof(ObjDict_Index6053) / sizeof(ObjDict_Index6053[0]), 0x6053},
    {(index_pObjDictry *)ObjDict_Index6054, sizeof(ObjDict_Index6054) / sizeof(ObjDict_Index6054[0]), 0x6054},
    {(index_pObjDictry *)ObjDict_Index6055, sizeof(ObjDict_Index6055) / sizeof(ObjDict_Index6055[0]), 0x6055},
    {(index_pObjDictry *)ObjDict_Index6056, sizeof(ObjDict_Index6056) / sizeof(ObjDict_Index6056[0]), 0x6056},
    {(index_pObjDictry *)ObjDict_Index6057, sizeof(ObjDict_Index6057) / sizeof(ObjDict_Index6057[0]), 0x6057},
    {(index_pObjDictry *)ObjDict_Index6058, sizeof(ObjDict_Index6058) / sizeof(ObjDict_Index6058[0]), 0x6058},
    {(index_pObjDictry *)ObjDict_Index6059, sizeof(ObjDict_Index6059) / sizeof(ObjDict_Index6059[0]), 0x6059},
    {(index_pObjDictry *)ObjDict_Index605A, sizeof(ObjDict_Index605A) / sizeof(ObjDict_Index605A[0]), 0x605A},
    {(index_pObjDictry *)ObjDict_Index605B, sizeof(ObjDict_Index605B) / sizeof(ObjDict_Index605B[0]), 0x605B},
    {(index_pObjDictry *)ObjDict_Index605C, sizeof(ObjDict_Index605C) / sizeof(ObjDict_Index605C[0]), 0x605C},
    {(index_pObjDictry *)ObjDict_Index605D, sizeof(ObjDict_Index605D) / sizeof(ObjDict_Index605D[0]), 0x605D},
    {(index_pObjDictry *)ObjDict_Index605E, sizeof(ObjDict_Index605E) / sizeof(ObjDict_Index605E[0]), 0x605E},
    {(index_pObjDictry *)ObjDict_Index6060, sizeof(ObjDict_Index6060) / sizeof(ObjDict_Index6060[0]), 0x6060},
    {(index_pObjDictry *)ObjDict_Index6061, sizeof(ObjDict_Index6061) / sizeof(ObjDict_Index6061[0]), 0x6061},
    {(index_pObjDictry *)ObjDict_Index6062, sizeof(ObjDict_Index6062) / sizeof(ObjDict_Index6062[0]), 0x6062},
    {(index_pObjDictry *)ObjDict_Index6063, sizeof(ObjDict_Index6063) / sizeof(ObjDict_Index6063[0]), 0x6063},
    {(index_pObjDictry *)ObjDict_Index6064, sizeof(ObjDict_Index6064) / sizeof(ObjDict_Index6064[0]), 0x6064},
    {(index_pObjDictry *)ObjDict_Index6065, sizeof(ObjDict_Index6065) / sizeof(ObjDict_Index6065[0]), 0x6065},
    {(index_pObjDictry *)ObjDict_Index6066, sizeof(ObjDict_Index6066) / sizeof(ObjDict_Index6066[0]), 0x6066},
    {(index_pObjDictry *)ObjDict_Index6067, sizeof(ObjDict_Index6067) / sizeof(ObjDict_Index6067[0]), 0x6067},
    {(index_pObjDictry *)ObjDict_Index6068, sizeof(ObjDict_Index6068) / sizeof(ObjDict_Index6068[0]), 0x6068},
    {(index_pObjDictry *)ObjDict_Index6069, sizeof(ObjDict_Index6069) / sizeof(ObjDict_Index6069[0]), 0x6069},
    {(index_pObjDictry *)ObjDict_Index606B, sizeof(ObjDict_Index606B) / sizeof(ObjDict_Index606B[0]), 0x606B},
    {(index_pObjDictry *)ObjDict_Index606C, sizeof(ObjDict_Index606C) / sizeof(ObjDict_Index606C[0]), 0x606C},
    {(index_pObjDictry *)ObjDict_Index606D, sizeof(ObjDict_Index606D) / sizeof(ObjDict_Index606D[0]), 0x606D},
    {(index_pObjDictry *)ObjDict_Index606E, sizeof(ObjDict_Index606E) / sizeof(ObjDict_Index606E[0]), 0x606E},
    {(index_pObjDictry *)ObjDict_Index606F, sizeof(ObjDict_Index606F) / sizeof(ObjDict_Index606F[0]), 0x606F},
    {(index_pObjDictry *)ObjDict_Index6070, sizeof(ObjDict_Index6070) / sizeof(ObjDict_Index6070[0]), 0x6070},
    {(index_pObjDictry *)ObjDict_Index6071, sizeof(ObjDict_Index6071) / sizeof(ObjDict_Index6071[0]), 0x6071},
    {(index_pObjDictry *)ObjDict_Index6072, sizeof(ObjDict_Index6072) / sizeof(ObjDict_Index6072[0]), 0x6072},
    {(index_pObjDictry *)ObjDict_Index6073, sizeof(ObjDict_Index6073) / sizeof(ObjDict_Index6073[0]), 0x6073},
    {(index_pObjDictry *)ObjDict_Index6074, sizeof(ObjDict_Index6074) / sizeof(ObjDict_Index6074[0]), 0x6074},
    {(index_pObjDictry *)ObjDict_Index6075, sizeof(ObjDict_Index6075) / sizeof(ObjDict_Index6075[0]), 0x6075},
    {(index_pObjDictry *)ObjDict_Index6076, sizeof(ObjDict_Index6076) / sizeof(ObjDict_Index6076[0]), 0x6076},
    {(index_pObjDictry *)ObjDict_Index6077, sizeof(ObjDict_Index6077) / sizeof(ObjDict_Index6077[0]), 0x6077},
    {(index_pObjDictry *)ObjDict_Index6078, sizeof(ObjDict_Index6078) / sizeof(ObjDict_Index6078[0]), 0x6078},
    {(index_pObjDictry *)ObjDict_Index6079, sizeof(ObjDict_Index6079) / sizeof(ObjDict_Index6079[0]), 0x6079},
    {(index_pObjDictry *)ObjDict_Index607A, sizeof(ObjDict_Index607A) / sizeof(ObjDict_Index607A[0]), 0x607A},
    {(index_pObjDictry *)ObjDict_Index607C, sizeof(ObjDict_Index607C) / sizeof(ObjDict_Index607C[0]), 0x607C},
    {(index_pObjDictry *)ObjDict_Index607D, sizeof(ObjDict_Index607D) / sizeof(ObjDict_Index607D[0]), 0x607D},
    {(index_pObjDictry *)ObjDict_Index607E, sizeof(ObjDict_Index607E) / sizeof(ObjDict_Index607E[0]), 0x607E},
    {(index_pObjDictry *)ObjDict_Index6081, sizeof(ObjDict_Index6081) / sizeof(ObjDict_Index6081[0]), 0x6081},
    {(index_pObjDictry *)ObjDict_Index6083, sizeof(ObjDict_Index6083) / sizeof(ObjDict_Index6083[0]), 0x6083},
    {(index_pObjDictry *)ObjDict_Index6084, sizeof(ObjDict_Index6084) / sizeof(ObjDict_Index6084[0]), 0x6084},
    {(index_pObjDictry *)ObjDict_Index6085, sizeof(ObjDict_Index6085) / sizeof(ObjDict_Index6085[0]), 0x6085},
    {(index_pObjDictry *)ObjDict_Index6086, sizeof(ObjDict_Index6086) / sizeof(ObjDict_Index6086[0]), 0x6086},
    {(index_pObjDictry *)ObjDict_Index6087, sizeof(ObjDict_Index6087) / sizeof(ObjDict_Index6087[0]), 0x6087},
    {(index_pObjDictry *)ObjDict_Index6088, sizeof(ObjDict_Index6088) / sizeof(ObjDict_Index6088[0]), 0x6088},
    {(index_pObjDictry *)ObjDict_Index6089, sizeof(ObjDict_Index6089) / sizeof(ObjDict_Index6089[0]), 0x6089},
    {(index_pObjDictry *)ObjDict_Index608A, sizeof(ObjDict_Index608A) / sizeof(ObjDict_Index608A[0]), 0x608A},
    {(index_pObjDictry *)ObjDict_Index608B, sizeof(ObjDict_Index608B) / sizeof(ObjDict_Index608B[0]), 0x608B},
    {(index_pObjDictry *)ObjDict_Index608C, sizeof(ObjDict_Index608C) / sizeof(ObjDict_Index608C[0]), 0x608C},
    {(index_pObjDictry *)ObjDict_Index608D, sizeof(ObjDict_Index608D) / sizeof(ObjDict_Index608D[0]), 0x608D},
    {(index_pObjDictry *)ObjDict_Index608E, sizeof(ObjDict_Index608E) / sizeof(ObjDict_Index608E[0]), 0x608E},
    {(index_pObjDictry *)ObjDict_Index6093, sizeof(ObjDict_Index6093) / sizeof(ObjDict_Index6093[0]), 0x6093},
    {(index_pObjDictry *)ObjDict_Index6094, sizeof(ObjDict_Index6094) / sizeof(ObjDict_Index6094[0]), 0x6094},
    {(index_pObjDictry *)ObjDict_Index6097, sizeof(ObjDict_Index6097) / sizeof(ObjDict_Index6097[0]), 0x6097},
    {(index_pObjDictry *)ObjDict_Index6098, sizeof(ObjDict_Index6098) / sizeof(ObjDict_Index6098[0]), 0x6098},
    {(index_pObjDictry *)ObjDict_Index6099, sizeof(ObjDict_Index6099) / sizeof(ObjDict_Index6099[0]), 0x6099},
    {(index_pObjDictry *)ObjDict_Index609A, sizeof(ObjDict_Index609A) / sizeof(ObjDict_Index609A[0]), 0x609A},
    {(index_pObjDictry *)ObjDict_Index60C5, sizeof(ObjDict_Index60C5) / sizeof(ObjDict_Index60C5[0]), 0x60C5},
    {(index_pObjDictry *)ObjDict_Index60C6, sizeof(ObjDict_Index60C6) / sizeof(ObjDict_Index60C6[0]), 0x60C6},
    {(index_pObjDictry *)ObjDict_Index60F4, sizeof(ObjDict_Index60F4) / sizeof(ObjDict_Index60F4[0]), 0x60F4},
    {(index_pObjDictry *)ObjDict_Index60F6, sizeof(ObjDict_Index60F6) / sizeof(ObjDict_Index60F6[0]), 0x60F6},
    {(index_pObjDictry *)ObjDict_Index60F7, sizeof(ObjDict_Index60F7) / sizeof(ObjDict_Index60F7[0]), 0x60F7},
    {(index_pObjDictry *)ObjDict_Index60F8, sizeof(ObjDict_Index60F8) / sizeof(ObjDict_Index60F8[0]), 0x60F8},
    {(index_pObjDictry *)ObjDict_Index60F9, sizeof(ObjDict_Index60F9) / sizeof(ObjDict_Index60F9[0]), 0x60F9},
    {(index_pObjDictry *)ObjDict_Index60FB, sizeof(ObjDict_Index60FB) / sizeof(ObjDict_Index60FB[0]), 0x60FB},
    {(index_pObjDictry *)ObjDict_Index60FC, sizeof(ObjDict_Index60FC) / sizeof(ObjDict_Index60FC[0]), 0x60FC},
    {(index_pObjDictry *)ObjDict_Index60FD, sizeof(ObjDict_Index60FD) / sizeof(ObjDict_Index60FD[0]), 0x60FD},
    {(index_pObjDictry *)ObjDict_Index60FE, sizeof(ObjDict_Index60FE) / sizeof(ObjDict_Index60FE[0]), 0x60FE},
    {(index_pObjDictry *)ObjDict_Index60FF, sizeof(ObjDict_Index60FF) / sizeof(ObjDict_Index60FF[0]), 0x60FF},
    {(index_pObjDictry *)ObjDict_Index6402, sizeof(ObjDict_Index6402) / sizeof(ObjDict_Index6402[0]), 0x6402},
    {(index_pObjDictry *)ObjDict_Index6403, sizeof(ObjDict_Index6403) / sizeof(ObjDict_Index6403[0]), 0x6403},
    {(index_pObjDictry *)ObjDict_Index6404, sizeof(ObjDict_Index6404) / sizeof(ObjDict_Index6404[0]), 0x6404},
    {(index_pObjDictry *)ObjDict_Index6502, sizeof(ObjDict_Index6502) / sizeof(ObjDict_Index6502[0]), 0x6502}
};
/* Obj Dictionary Size */
const unsigned short int pObjDictSize = sizeof(pObjDict)/sizeof(pObjDict[0]);

/*------------------------------------------------
Function:保存参数回调函数
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
unsigned int  ODCallback_t_Index1010_Subindex1(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
    /* Driver status check */
    if(STM[M1].bState == RUN || STM[M2].bState == RUN) //如果电机处于使能状态则不进行flash保存
    {
        return ACCESS_FAILED_DUE_TO_HARDWARE;
    }
    /* Save parameters */
    if(Atlas_Write_Flash(Atlas_Flash_Addr,FlashArray) != FLASH_SUCCESS)
    {
        printf("Flash_Write error\r\n");
    }

    return 0;
}

//unsigned int  ODCallback_t_Index208A_Subindex0(CO_Data *d, const indextable *index, unsigned char bSubindex)
//{
//    long SetPosValue = 0;

//    /* HANLDE IN MOTOR OFF */
////  if (pAxisPar.motorEn[A_AXIS] != MOTOR_OFF)
//    {
//        memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
//        return SDOABT_LOCAL_CTRL_ERROR;
//    }
//    SetPosValue = *(unsigned int*)index->pSubindex[bSubindex].lpParam;
////  SetPositionValue(SetPosValue, A_AXIS);

//    return 0;
//}
/*------------------------------------------------
Function:配置NodeID回调函数
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
unsigned int  ODCallback_t_Index2011_Subindex0(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
    unsigned char newNodeID;

    /* Get new Node id */
    newNodeID = *(unsigned char*)index->pSubindex[bSubindex].lpParam;
    /* Node_ID range check */
    if(newNodeID > SLAVE_NODE_NUM_MIN\
            || newNodeID < SLAVE_NODE_NUM_MAX\
      )
    {
        /* Recover Node id */
//        *(unsigned char*)index->pSubindex[bSubindex].lpParam = d->Node_ID;
        SetNodeId(&CANopen_Drive,newNodeID);
//        return SDOABT_GENERAL_ERROR;
    }
    return 0;
}
/*------------------------------------------------
Function:配置CAN波特率回调函数
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
unsigned int  ODCallback_t_Index2012_Subindex0(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
    unsigned int newBandrate;

    /* Get new Bandrate */
    newBandrate = *(unsigned int*)index->pSubindex[bSubindex].lpParam;
    /* Permision Bandrate Check */
    if(newBandrate != CAN_BANDRATE_50K &&\
            newBandrate != CAN_BANDRATE_100K &&\
            newBandrate != CAN_BANDRATE_125K &&\
            newBandrate != CAN_BANDRATE_200K &&\
            newBandrate != CAN_BANDRATE_250K &&\
            newBandrate != CAN_BANDRATE_500K &&\
            newBandrate != CAN_BANDRATE_1000K\
      )
    {
        /* Recover CAN Bandrate */
        *(unsigned int*)index->pSubindex[bSubindex].lpParam = d->canBandrate;
        return SDOABT_GENERAL_ERROR;
    }

    return 0;
}
/*------------------------------------------------
Function:OTA升级回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int ODCallback_t_Index200B_Subindex0(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
    uint8_t ota_remote_upgrade ;
    ota_remote_upgrade  = *(unsigned int*)index->pSubindex[bSubindex].lpParam;
    if(ota_remote_upgrade == 1)
    {
        SoftReset();
    }
    else if(ota_remote_upgrade == 2)
    {
        Jump_to_APP(0x08000000);
    }
		return 0 ;
}
/*------------------------------------------------
Function:速度环最大输出限制用于机器人对桩电流限制
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int ODCallback_t_Index3001_Subindex0(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
    int16_t OutputLimit = 0;
	  int16_t temp = 0;
	  float   unitchage = 0;
	  temp = *(unsigned int*)index->pSubindex[bSubindex].lpParam;
	  unitchage = ((float)temp/1000)*ADC_AMPER_Ratio*1000;
    OutputLimit  = (int16_t)unitchage;
    if(OutputLimit >= ADC_AMPER_Ratio*1000)
    {
        if(OutputLimit >= MAX_CURRENT )//5950*5 限制最大值
        {
            PIDSpeedHandle_M1.hUpperOutputLimit =  MAX_CURRENT;
            PIDSpeedHandle_M1.hLowerOutputLimit = -MAX_CURRENT;
            PIDSpeedHandle_M2.hUpperOutputLimit =  MAX_CURRENT;
            PIDSpeedHandle_M2.hLowerOutputLimit = -MAX_CURRENT;
        }
        else if(OutputLimit >= ADC_AMPER_Ratio*1000)//电流采样系数397
        {
					  PIDSpeedHandle_M1.hUpperOutputLimit =  OutputLimit;
            PIDSpeedHandle_M1.hLowerOutputLimit = -OutputLimit;//将两个轴的四个参数全部改为一致
            PIDSpeedHandle_M2.hUpperOutputLimit =  OutputLimit;
            PIDSpeedHandle_M2.hLowerOutputLimit = -OutputLimit;
        }
    }
    else //参数设置过小或者为负值
    {
			//下发的参数，不做修改
    }
		return 0;
}


/* 子索引接收到数据之后的回调函数 */
unsigned int  ODCallback_t_Index2000_Subindex1(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
    ATLAS_PRINT("-------------------------------------------------OD_Callback_Test_Begin-------------------------------------------------\r\n");
    ATLAS_PRINT("ControlWordAxis1 = %d\r\n", obj2000_sub1);
    ATLAS_PRINT("Index = %d\r\n", index->index);
    ATLAS_PRINT("SubIndex = %d\r\n", bSubindex);
    ATLAS_PRINT("-------------------------------------------------OD_Callback_Test_End-------------------------------------------------\r\n");

    return 0;
}
unsigned int  ODCallback_t_Index2000_Subindex2(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
    ATLAS_PRINT("-------------------------------------------------OD_Callback_Test_Begin-------------------------------------------------\r\n");
    ATLAS_PRINT("ControlWordAxis2 = %d\r\n", obj2000_sub2);
    ATLAS_PRINT("Index = %d\r\n", index->index);
    ATLAS_PRINT("SubIndex = %d\r\n", bSubindex);
    ATLAS_PRINT("-------------------------------------------------OD_Callback_Test_End-------------------------------------------------\r\n");
    return 0;
}
ODCallback_t ObjDict_Index2000_callbacks[] =
{
    NULL,
    NULL,
    NULL,
};
ODCallback_t ObjDict_Index2001_callbacks[] =
{
    NULL,
};

/*------------------------------------------------
Function:通过索引查该条目的指针
Input   :Index subindex *errcode **callbaks
Output  :No
Explain :No
------------------------------------------------*/
const indextable *ObjDict_scanIndexOD(unsigned short int wIndex, unsigned int *errorCode, ODCallback_t **callbacks)
{
    int i;
    *callbacks = NULL;

    switch(wIndex)
    {
    case 0x1000:
        i = 1;
        break;
    case 0x1001:
        i = 2;
        break;
    case 0x1002:
        i = 3;
        break;
    case 0x1003:
        i = 4;
        break;/* TODO:Lack a callback,Vide DS301 P89 */
    case 0x1005:
        i = 5;
        break;
    case 0x1006:
        i = 6;
        break;
    case 0x1007:
        i = 7;
        break;
    case 0x1008:
        i = 8;
        break;
    case 0x1009:
        i = 9;
        break;
    case 0x100A:
        i = 10;
        break;
    case 0x100C:
        i = 11;
        break;
    case 0x100D:
        i = 12;
        break;
    case 0x1010:
        i = 13;
        *callbacks =Index1010_callbacks;
        break;
    case 0x1011:
        i = 14;
        break;
    case 0x1012:
        i = 15;
        break;
    case 0x1013:
        i = 16;
        break;
    case 0x1014:
        i = 17;
        break;
    case 0x1015:
        i = 18;
        break;
    case 0x1016:
        i = 19;
        *callbacks = Index1016_callbacks;
        break;
    case 0x1017:
        i = 20;
        *callbacks = Index1017_callbacks;
        break;
    case 0x1018:
        i = 21;
        break;
    case 0x1019:
        i = 22;
        break;
    case 0x1028:
        i = 23;
        break;
    case 0x1200:
        i = 24;
        break;
    case 0x1280:
        i = 25;
        break;
    case 0x1400:
        i = 26;
        *callbacks = RPDO_Cmct_callbacks;
        break;
    case 0x1401:
        i = 27;
        *callbacks = RPDO_Cmct_callbacks;
        break;
    case 0x1402:
        i = 28;
        *callbacks = RPDO_Cmct_callbacks;
        break;
    case 0x1403:
        i = 29;
        *callbacks = RPDO_Cmct_callbacks;
        break;
    case 0x1600:
        i = 30;
        *callbacks = IndexRPDO_Map_callbacks;
        break;
    case 0x1601:
        i = 31;
        *callbacks = IndexRPDO_Map_callbacks;
        break;
    case 0x1602:
        i = 32;
        *callbacks = IndexRPDO_Map_callbacks;
        break;
    case 0x1603:
        i = 33;
        *callbacks = IndexRPDO_Map_callbacks;
        break;
    case 0x1800:
        i = 34;
        *callbacks = TPDO_Cmct_callbacks;
        break;
    case 0x1801:
        i = 35;
        *callbacks = TPDO_Cmct_callbacks;
        break;
    case 0x1802:
        i = 36;
        *callbacks = TPDO_Cmct_callbacks;
        break;
    case 0x1803:
        i = 37;
        *callbacks = TPDO_Cmct_callbacks;
        break;
    case 0x1A00:
        i = 38;
        *callbacks = IndexTPDO_Map_callbacks;
        break;
    case 0x1A01:
        i = 39;
        *callbacks = IndexTPDO_Map_callbacks;
        break;
    case 0x1A02:
        i = 40;
        *callbacks = IndexTPDO_Map_callbacks;
        break;
    case 0x1A03:
        i = 41;
        *callbacks = IndexTPDO_Map_callbacks;
        break;

    case 0x2000:
        i = 42;
        *callbacks = ObjDict_Index2000_callbacks;
        break;
    case 0x2001:
        i = 43;
        break;
    case 0x2002:
        i = 44;
        break;
    case 0x2003:
        i = 45;
        break;
    case 0x2004:
        i = 46;
        break;
    case 0x2005:
        i = 47;
        break;
    case 0x2006:
        i = 48;
        break;
    case 0x2007:
        i = 49;
        break;
    case 0x2008:
        i = 50;
        break;
    case 0x2009:
        i = 51;
        break;
    case 0x200A:
        i = 52;
        break;
    case 0x200B:
        i = 53;
        *callbacks = Index200B_callbacks;
        break;
    case 0x200C:
        i = 54;
        break;
    case 0x200D:
        i = 55;
        break;
    case 0x200E:
        i = 56;
        break;
    case 0x200F:
        i = 57;
        break;
    case 0x2010:
        i = 58;
        break;
    case 0x2011:
        i = 59;
        *callbacks = Index2011_callbacks;
        break;
    case 0x2012:
        i = 60;
        *callbacks = Index2012_callbacks;
        break;
    case 0x2013:
        i = 61;
        break;
    case 0x2014:
        i = 62;
        break;
    case 0x2015:
        i = 63;
        break;
    case 0x2016:
        i = 64;
        break;
    case 0x2017:
        i = 65;
        break;
    case 0x2018:
        i = 66;
        break;
    case 0x2019:
        i = 67;
        break;
    case 0x2020:
        i = 68;
        break;
    case 0x3000:
        i = 69;
        break;
    case 0x3001:
        i = 70;
        *callbacks = Index3001_callbacks;
        break;



    case 0x6007:
        i = 41+MANUFACTURER_ADD_NUM;
        break;
    case 0x603F:
        i = 42+MANUFACTURER_ADD_NUM;
        break;
    case 0x6040:
        i = 43+MANUFACTURER_ADD_NUM;
        break;
    case 0x6041:
        i = 44+MANUFACTURER_ADD_NUM;
        break;
    case 0x6042:
        i = 45+MANUFACTURER_ADD_NUM;
        break;
    case 0x6043:
        i = 46+MANUFACTURER_ADD_NUM;
        break;
    case 0x6044:
        i = 47+MANUFACTURER_ADD_NUM;
        break;
    case 0x6045:
        i = 48+MANUFACTURER_ADD_NUM;
        break;
    case 0x6046:
        i = 49+MANUFACTURER_ADD_NUM;
        break;
    case 0x6047:
        i = 50+MANUFACTURER_ADD_NUM;
        break;
    case 0x6048:
        i = 51+MANUFACTURER_ADD_NUM;
        break;
    case 0x6049:
        i = 52+MANUFACTURER_ADD_NUM;
        break;
    case 0x604A:
        i = 53+MANUFACTURER_ADD_NUM;
        break;
    case 0x604B:
        i = 54+MANUFACTURER_ADD_NUM;
        break;
    case 0x604C:
        i = 55+MANUFACTURER_ADD_NUM;
        break;
    case 0x604D:
        i = 56+MANUFACTURER_ADD_NUM;
        break;
    case 0x604E:
        i = 57+MANUFACTURER_ADD_NUM;
        break;
    case 0x604F:
        i = 58+MANUFACTURER_ADD_NUM;
        break;
    case 0x6050:
        i = 59+MANUFACTURER_ADD_NUM;
        break;
    case 0x6051:
        i = 60+MANUFACTURER_ADD_NUM;
        break;
    case 0x6052:
        i = 61+MANUFACTURER_ADD_NUM;
        break;
    case 0x6053:
        i = 62+MANUFACTURER_ADD_NUM;
        break;
    case 0x6054:
        i = 63+MANUFACTURER_ADD_NUM;
        break;
    case 0x6055:
        i = 64+MANUFACTURER_ADD_NUM;
        break;
    case 0x6056:
        i = 65+MANUFACTURER_ADD_NUM;
        break;
    case 0x6057:
        i = 66+MANUFACTURER_ADD_NUM;
        break;
    case 0x6058:
        i = 67+MANUFACTURER_ADD_NUM;
        break;
    case 0x6059:
        i = 68+MANUFACTURER_ADD_NUM;
        break;
    case 0x605A:
        i = 69+MANUFACTURER_ADD_NUM;
        break;
    case 0x605B:
        i = 70+MANUFACTURER_ADD_NUM;
        break;
    case 0x605C:
        i = 71+MANUFACTURER_ADD_NUM;
        break;
    case 0x605D:
        i = 72+MANUFACTURER_ADD_NUM;
        break;
    case 0x605E:
        i = 73+MANUFACTURER_ADD_NUM;
        break;
    case 0x6060:
        i = 74+MANUFACTURER_ADD_NUM;
        break;
    case 0x6061:
        i = 75+MANUFACTURER_ADD_NUM;
        break;
    case 0x6062:
        i = 76+MANUFACTURER_ADD_NUM;
        break;
    case 0x6063:
        i = 77+MANUFACTURER_ADD_NUM;
        break;
    case 0x6064:
        i = 78+MANUFACTURER_ADD_NUM;
        break;
    case 0x6065:
        i = 79+MANUFACTURER_ADD_NUM;
        break;
    case 0x6066:
        i = 80+MANUFACTURER_ADD_NUM;
        break;
    case 0x6067:
        i = 81+MANUFACTURER_ADD_NUM;
        break;
    case 0x6068:
        i = 82+MANUFACTURER_ADD_NUM;
        break;
    case 0x6069:
        i = 83+MANUFACTURER_ADD_NUM;
        break;
    case 0x606B:
        i = 84+MANUFACTURER_ADD_NUM;
        break;
    case 0x606C:
        i = 85+MANUFACTURER_ADD_NUM;
        break;
    case 0x606D:
        i = 86+MANUFACTURER_ADD_NUM;
        break;
    case 0x606E:
        i = 87+MANUFACTURER_ADD_NUM;
        break;
    case 0x606F:
        i = 88+MANUFACTURER_ADD_NUM;
        break;
    case 0x6070:
        i = 89+MANUFACTURER_ADD_NUM;
        break;
    case 0x6071:
        i = 90+MANUFACTURER_ADD_NUM;
        break;
    case 0x6072:
        i = 91+MANUFACTURER_ADD_NUM;
        break;
    case 0x6073:
        i = 92+MANUFACTURER_ADD_NUM;
        break;
    case 0x6074:
        i = 93+MANUFACTURER_ADD_NUM;
        break;
    case 0x6075:
        i = 94+MANUFACTURER_ADD_NUM;
        break;
    case 0x6076:
        i = 95+MANUFACTURER_ADD_NUM;
        break;
    case 0x6077:
        i = 96+MANUFACTURER_ADD_NUM;
        break;
    case 0x6078:
        i = 97+MANUFACTURER_ADD_NUM;
        break;
    case 0x6079:
        i = 98+MANUFACTURER_ADD_NUM;
        break;
    case 0x607A:
        i = 99+MANUFACTURER_ADD_NUM;
        break;
    case 0x607C:
        i = 100+MANUFACTURER_ADD_NUM;
        break;
    case 0x607D:
        i = 101+MANUFACTURER_ADD_NUM;
        break;
    case 0x607E:
        i = 102+MANUFACTURER_ADD_NUM;
        break;
    case 0x6081:
        i = 103+MANUFACTURER_ADD_NUM;
        break;
    case 0x6083:
        i = 104+MANUFACTURER_ADD_NUM;
        break;
    case 0x6084:
        i = 105+MANUFACTURER_ADD_NUM;
        break;
    case 0x6085:
        i = 106+MANUFACTURER_ADD_NUM;
        break;
    case 0x6086:
        i = 107+MANUFACTURER_ADD_NUM;
        break;
    case 0x6087:
        i = 108+MANUFACTURER_ADD_NUM;
        break;
    case 0x6088:
        i = 109+MANUFACTURER_ADD_NUM;
        break;
    case 0x6089:
        i = 110+MANUFACTURER_ADD_NUM;
        break;
    case 0x608A:
        i = 111+MANUFACTURER_ADD_NUM;
        break;
    case 0x608B:
        i = 112+MANUFACTURER_ADD_NUM;
        break;
    case 0x608C:
        i = 113+MANUFACTURER_ADD_NUM;
        break;
    case 0x608D:
        i = 114+MANUFACTURER_ADD_NUM;
        break;
    case 0x608E:
        i = 115+MANUFACTURER_ADD_NUM;
        break;
    case 0x6093:
        i = 116+MANUFACTURER_ADD_NUM;
        break;
    case 0x6094:
        i = 117+MANUFACTURER_ADD_NUM;
        break;
    case 0x6097:
        i = 118+MANUFACTURER_ADD_NUM;
        break;
    case 0x6098:
        i = 110+MANUFACTURER_ADD_NUM;
        break;
    case 0x6099:
        i = 120+MANUFACTURER_ADD_NUM;
        break;
    case 0x609A:
        i = 121+MANUFACTURER_ADD_NUM;
        break;
    case 0x60C5:
        i = 122+MANUFACTURER_ADD_NUM;
        break;
    case 0x60C6:
        i = 123+MANUFACTURER_ADD_NUM;
        break;
    case 0x60F4:
        i = 124+MANUFACTURER_ADD_NUM;
        break;
    case 0x60F6:
        i = 125+MANUFACTURER_ADD_NUM;
        break;
    case 0x60F7:
        i = 126+MANUFACTURER_ADD_NUM;
        break;
    case 0x60F8:
        i = 127+MANUFACTURER_ADD_NUM;
        break;
    case 0x60F9:
        i = 128+MANUFACTURER_ADD_NUM;
        break;
    case 0x60FB:
        i = 129+MANUFACTURER_ADD_NUM;
        break;
    case 0x60FC:
        i = 130+MANUFACTURER_ADD_NUM;
        break;
    case 0x60FD:
        i = 131+MANUFACTURER_ADD_NUM;
        break;
    case 0x60FE:
        i = 132+MANUFACTURER_ADD_NUM;
        break;
    case 0x60FF:
        i = 133+MANUFACTURER_ADD_NUM;
        break;
    case 0x6402:
        i = 134+MANUFACTURER_ADD_NUM;
        break;
    case 0x6403:
        i = 135+MANUFACTURER_ADD_NUM;
        break;
    case 0x6404:
        i = 136+MANUFACTURER_ADD_NUM;
        break;
    case 0x6502:
        i = 137+MANUFACTURER_ADD_NUM;
        break;
    default:
        *errorCode = OD_NO_SUCH_OBJECT;
        return NULL;
    }
    *errorCode = OD_SUCCESSFUL;
    return &pObjDict[i];
}





/*------------------------------------------------
Function:获取参数ID地址
Input   :Index subindex *errcode **callbaks
Output  :No
Explain :No
------------------------------------------------*/
unsigned int ObjDict_Get_Id(unsigned short int wIndex, unsigned int *ObjId)
{
    int i;
    unsigned int errorCode;

    switch(wIndex)
    {
    case 0x1000:
        i = 1;
        break;
    case 0x1001:
        i = 2;
        break;
    case 0x1002:
        i = 3;
        break;
    case 0x1003:
        i = 4;
        break;
    case 0x1005:
        i = 5;
        break;
    case 0x1006:
        i = 6;
        break;
    case 0x1007:
        i = 7;
        break;
    case 0x1008:
        i = 8;
        break;
    case 0x1009:
        i = 9;
        break;
    case 0x100A:
        i = 10;
        break;
    case 0x100C:
        i = 11;
        break;
    case 0x100D:
        i = 12;
        break;
    case 0x1010:
        i = 13;
        break;
    case 0x1011:
        i = 14;
        break;
    case 0x1012:
        i = 15;
        break;
    case 0x1013:
        i = 16;
        break;
    case 0x1014:
        i = 17;
        break;
    case 0x1015:
        i = 18;
        break;
    case 0x1016:
        i = 19;
        break;
    case 0x1017:
        i = 20;
        break;
    case 0x1018:
        i = 21;
        break;
    case 0x1019:
        i = 22;
        break;
    case 0x1028:
        i = 23;
        break;
    case 0x1200:
        i = 24;
        break;
    case 0x1280:
        i = 25;
        break;
    case 0x1400:
        i = 26;
        break;
    case 0x1401:
        i = 27;
        break;
    case 0x1402:
        i = 28;
        break;
    case 0x1403:
        i = 29;
        break;
    case 0x1600:
        i = 30;
        break;
    case 0x1601:
        i = 31;
        break;
    case 0x1602:
        i = 32;
        break;
    case 0x1603:
        i = 33;
        break;
    case 0x1800:
        i = 34;
        break;
    case 0x1801:
        i = 35;
        break;
    case 0x1802:
        i = 36;
        break;
    case 0x1803:
        i = 37;
        break;
    case 0x1A00:
        i = 38;
        break;
    case 0x1A01:
        i = 39;
        break;
    case 0x1A02:
        i = 40;
        break;
    case 0x1A03:
        i = 41;
        break;

    case 0x2000:
        i = 42;
        break;
    case 0x2001:
        i = 43;
        break;
    case 0x2002:
        i = 44;
        break;
    case 0x2003:
        i = 45;
        break;
    case 0x2004:
        i = 46;
        break;
    case 0x2005:
        i = 47;
        break;
    case 0x2006:
        i = 48;
        break;
    case 0x2007:
        i = 49;
        break;
    case 0x2008:
        i = 50;
        break;
    case 0x2009:
        i = 51;
        break;
    case 0x200A:
        i = 52;
        break;
    case 0x200B:
        i = 53;
        break;
    case 0x200C:
        i = 54;
        break;
    case 0x200D:
        i = 55;
        break;
    case 0x200E:
        i = 56;
        break;
    case 0x200F:
        i = 57;
        break;
    case 0x2010:
        i = 58;
        break;
    case 0x2011:
        i = 59;
        break;
    case 0x2012:
        i = 60;
        break;
    case 0x2013:
        i = 61;
        break;
    case 0x2014:
        i = 62;
        break;
    case 0x2015:
        i = 63;
        break;
    case 0x2016:
        i = 64;
        break;
    case 0x2017:
        i = 65;
        break;
    case 0x2018:
        i = 66;
        break;
    case 0x2019:
        i = 67;
        break;
    case 0x2020:
        i = 68;
        break;
    case 0x3000:
        i = 69;
        break;
    case 0x3001:
        i = 70;
        break;

    case 0x6007:
        i = 41+MANUFACTURER_ADD_NUM;
        break;
    case 0x603F:
        i = 42+MANUFACTURER_ADD_NUM;
        break;
    case 0x6040:
        i = 43+MANUFACTURER_ADD_NUM;
        break;
    case 0x6041:
        i = 44+MANUFACTURER_ADD_NUM;
        break;
    case 0x6042:
        i = 45+MANUFACTURER_ADD_NUM;
        break;
    case 0x6043:
        i = 46+MANUFACTURER_ADD_NUM;
        break;
    case 0x6044:
        i = 47+MANUFACTURER_ADD_NUM;
        break;
    case 0x6045:
        i = 48+MANUFACTURER_ADD_NUM;
        break;
    case 0x6046:
        i = 49+MANUFACTURER_ADD_NUM;
        break;
    case 0x6047:
        i = 50+MANUFACTURER_ADD_NUM;
        break;
    case 0x6048:
        i = 51+MANUFACTURER_ADD_NUM;
        break;
    case 0x6049:
        i = 52+MANUFACTURER_ADD_NUM;
        break;
    case 0x604A:
        i = 53+MANUFACTURER_ADD_NUM;
        break;
    case 0x604B:
        i = 54+MANUFACTURER_ADD_NUM;
        break;
    case 0x604C:
        i = 55+MANUFACTURER_ADD_NUM;
        break;
    case 0x604D:
        i = 56+MANUFACTURER_ADD_NUM;
        break;
    case 0x604E:
        i = 57+MANUFACTURER_ADD_NUM;
        break;
    case 0x604F:
        i = 58+MANUFACTURER_ADD_NUM;
        break;
    case 0x6050:
        i = 59+MANUFACTURER_ADD_NUM;
        break;
    case 0x6051:
        i = 60+MANUFACTURER_ADD_NUM;
        break;
    case 0x6052:
        i = 61+MANUFACTURER_ADD_NUM;
        break;
    case 0x6053:
        i = 62+MANUFACTURER_ADD_NUM;
        break;
    case 0x6054:
        i = 63+MANUFACTURER_ADD_NUM;
        break;
    case 0x6055:
        i = 64+MANUFACTURER_ADD_NUM;
        break;
    case 0x6056:
        i = 65+MANUFACTURER_ADD_NUM;
        break;
    case 0x6057:
        i = 66+MANUFACTURER_ADD_NUM;
        break;
    case 0x6058:
        i = 67+MANUFACTURER_ADD_NUM;
        break;
    case 0x6059:
        i = 68+MANUFACTURER_ADD_NUM;
        break;
    case 0x605A:
        i = 69+MANUFACTURER_ADD_NUM;
        break;
    case 0x605B:
        i = 70+MANUFACTURER_ADD_NUM;
        break;
    case 0x605C:
        i = 71+MANUFACTURER_ADD_NUM;
        break;
    case 0x605D:
        i = 72+MANUFACTURER_ADD_NUM;
        break;
    case 0x605E:
        i = 73+MANUFACTURER_ADD_NUM;
        break;
    case 0x6060:
        i = 74+MANUFACTURER_ADD_NUM;
        break;
    case 0x6061:
        i = 75+MANUFACTURER_ADD_NUM;
        break;
    case 0x6062:
        i = 76+MANUFACTURER_ADD_NUM;
        break;
    case 0x6063:
        i = 77+MANUFACTURER_ADD_NUM;
        break;
    case 0x6064:
        i = 78+MANUFACTURER_ADD_NUM;
        break;
    case 0x6065:
        i = 79+MANUFACTURER_ADD_NUM;
        break;
    case 0x6066:
        i = 80+MANUFACTURER_ADD_NUM;
        break;
    case 0x6067:
        i = 81+MANUFACTURER_ADD_NUM;
        break;
    case 0x6068:
        i = 82+MANUFACTURER_ADD_NUM;
        break;
    case 0x6069:
        i = 83+MANUFACTURER_ADD_NUM;
        break;
    case 0x606B:
        i = 84+MANUFACTURER_ADD_NUM;
        break;
    case 0x606C:
        i = 85+MANUFACTURER_ADD_NUM;
        break;
    case 0x606D:
        i = 86+MANUFACTURER_ADD_NUM;
        break;
    case 0x606E:
        i = 87+MANUFACTURER_ADD_NUM;
        break;
    case 0x606F:
        i = 88+MANUFACTURER_ADD_NUM;
        break;
    case 0x6070:
        i = 89+MANUFACTURER_ADD_NUM;
        break;
    case 0x6071:
        i = 90+MANUFACTURER_ADD_NUM;
        break;
    case 0x6072:
        i = 91+MANUFACTURER_ADD_NUM;
        break;
    case 0x6073:
        i = 92+MANUFACTURER_ADD_NUM;
        break;
    case 0x6074:
        i = 93+MANUFACTURER_ADD_NUM;
        break;
    case 0x6075:
        i = 94+MANUFACTURER_ADD_NUM;
        break;
    case 0x6076:
        i = 95+MANUFACTURER_ADD_NUM;
        break;
    case 0x6077:
        i = 96+MANUFACTURER_ADD_NUM;
        break;
    case 0x6078:
        i = 97+MANUFACTURER_ADD_NUM;
        break;
    case 0x6079:
        i = 98+MANUFACTURER_ADD_NUM;
        break;
    case 0x607A:
        i = 99+MANUFACTURER_ADD_NUM;
        break;
    case 0x607C:
        i = 100+MANUFACTURER_ADD_NUM;
        break;
    case 0x607D:
        i = 101+MANUFACTURER_ADD_NUM;
        break;
    case 0x607E:
        i = 102+MANUFACTURER_ADD_NUM;
        break;
    case 0x6081:
        i = 103+MANUFACTURER_ADD_NUM;
        break;
    case 0x6083:
        i = 104+MANUFACTURER_ADD_NUM;
        break;
    case 0x6084:
        i = 105+MANUFACTURER_ADD_NUM;
        break;
    case 0x6085:
        i = 106+MANUFACTURER_ADD_NUM;
        break;
    case 0x6086:
        i = 107+MANUFACTURER_ADD_NUM;
        break;
    case 0x6087:
        i = 108+MANUFACTURER_ADD_NUM;
        break;
    case 0x6088:
        i = 109+MANUFACTURER_ADD_NUM;
        break;
    case 0x6089:
        i = 110+MANUFACTURER_ADD_NUM;
        break;
    case 0x608A:
        i = 111+MANUFACTURER_ADD_NUM;
        break;
    case 0x608B:
        i = 112+MANUFACTURER_ADD_NUM;
        break;
    case 0x608C:
        i = 113+MANUFACTURER_ADD_NUM;
        break;
    case 0x608D:
        i = 114+MANUFACTURER_ADD_NUM;
        break;
    case 0x608E:
        i = 115+MANUFACTURER_ADD_NUM;
        break;
    case 0x6093:
        i = 116+MANUFACTURER_ADD_NUM;
        break;
    case 0x6094:
        i = 117+MANUFACTURER_ADD_NUM;
        break;
    case 0x6097:
        i = 118+MANUFACTURER_ADD_NUM;
        break;
    case 0x6098:
        i = 110+MANUFACTURER_ADD_NUM;
        break;
    case 0x6099:
        i = 120+MANUFACTURER_ADD_NUM;
        break;
    case 0x609A:
        i = 121+MANUFACTURER_ADD_NUM;
        break;
    case 0x60C5:
        i = 122+MANUFACTURER_ADD_NUM;
        break;
    case 0x60C6:
        i = 123+MANUFACTURER_ADD_NUM;
        break;
    case 0x60F4:
        i = 124+MANUFACTURER_ADD_NUM;
        break;
    case 0x60F6:
        i = 125+MANUFACTURER_ADD_NUM;
        break;
    case 0x60F7:
        i = 126+MANUFACTURER_ADD_NUM;
        break;
    case 0x60F8:
        i = 127+MANUFACTURER_ADD_NUM;
        break;
    case 0x60F9:
        i = 128+MANUFACTURER_ADD_NUM;
        break;
    case 0x60FB:
        i = 129+MANUFACTURER_ADD_NUM;
        break;
    case 0x60FC:
        i = 130+MANUFACTURER_ADD_NUM;
        break;
    case 0x60FD:
        i = 131+MANUFACTURER_ADD_NUM;
        break;
    case 0x60FE:
        i = 132+MANUFACTURER_ADD_NUM;
        break;
    case 0x60FF:
        i = 133+MANUFACTURER_ADD_NUM;
        break;
    case 0x6402:
        i = 134+MANUFACTURER_ADD_NUM;
        break;
    case 0x6403:
        i = 135+MANUFACTURER_ADD_NUM;
        break;
    case 0x6404:
        i = 136+MANUFACTURER_ADD_NUM;
        break;
    case 0x6502:
        i = 137+MANUFACTURER_ADD_NUM;
        break;
    default:
        errorCode = OD_NO_SUCH_OBJECT;
        return errorCode;
    }
    testGl[25] = i;
    *ObjId = i;
    errorCode = OD_SUCCESSFUL;

    return errorCode;
}
/*------------------------------------------------
Function:对各数据类型的范围进行检查合法性检查
Input   :typeValue *value
Output  :No
Explain :No
------------------------------------------------*/
#define valueRange_EMC 0x9F /* Type for index 0x1003 subindex 0x00 (only set of value 0 is possible) */
unsigned int ObjDict_valueRangeTest(unsigned char typeValue, void *value)
{
    switch(typeValue)
    {
    case valueRange_EMC:
        if(*(char *)value != (char)0)
            return OD_VALUE_RANGE_EXCEEDED;
        break;

    default:
        break;
    }
    return 0;
}
/*------------------------------------------------
Function:可保存参数对象改变时调用的回调函数
Input   :d index subindex
Output  :No
Explain :No
------------------------------------------------*/
void _storeODSubIndex(CO_Data* d, unsigned short int wIndex, unsigned char bSubindex) {}
/*------------------------------------------------
Function:Driver Parameters Init
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
void CANopen_Parameter_Init(CO_Data* d)
{
    /*Device_Manufacturer*/

    /*Device_Manufacturer_End*/

    /* Bellow Data Is For Test - BEGIN */
    /* TPDO1 MAP DATA*/
    obj2000_sub1 = 0x55;
    obj2000_sub2 = 0x66;
    obj2000_sub3 = 0x88;
    obj2000_sub4 = 0x99;
    obj2000_sub8 = 0x11223344;
    /* SetODentry Test */
    testStr[0] = 0x11;
    testStr[1] = 0x22;
    testStr[2] = 0x33;
    testStr[3] = 0x44;
    setODentry(&CANopen_Drive,0x2000,0x02,testStr,&testGl[30],1);
    /* Bellow Data Is For Test - END */

    /* SetNodeId */
    SetNodeId(&CANopen_Drive,obj2011_NodeID_Default);

    /* SetState */
    d->NMT_state = UNKNOWN_STATE;
    d->canBandrate = obj2012_CanBandrate_Default;
    SetState(d, INITIALIZATION);
    SetState(d, OPERATION_STATE);//规避进入两次操作态的问题 2022.9.19 wzj
    SetState(d, STOPSTOP);
    SetState(d, INITIALIZATION);
}
/* CANOPEN DATA INIT */
CO_Data CANopen_Drive = {\
                         NULL,   /*Node_Id*/\
                         NULL,   /*CAN BandRate*/\
                         UNKNOWN_STATE,   /*NMT_state*/\
                         UNKNOWN_STATE,   /*Node_state*/\
                         /* NMT StateBegin */\
{   \
    0,          /* csBoot_Up */\
    0,          /* csSDO */\
    0,          /* csEmergency */\
    0,          /* csSYNC */\
    0,          /* csHeartbeat */\
    0,           /* csPDO */\
    0           /* csLSS */\
},\
_initialisation,\
_preOperational,\
_operational,\
_operational,\
NULL,\
NULL,\
/* NMT HeartBeat */
&obj1016_ClientHeart_timeout_subindex,\
obj1016_ClientHeart_timeout,\
ObjDict_heartBeatTimers,\
{   \
    0x00,0x00,0x00,0x00,0x00 \
},\
_heartbeatError,\
_post_SlaveStateChange,\
/* NMT State End */\
\
pObjDict,                   /* Object Dictionary Begin */\
{   \
    0x00,\
    0x00,\
    0x00,\
    0x00,\
    {
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 \
    }
},\
ObjDict_scanIndexOD,        /* OD SCAN CALLBACK */\
ObjDict_valueRangeTest,     /* VALUE RANGE TEST CALLBACK */\
_storeODSubIndex,           /* STORE PARA CALLBACK */\
&OBJ_FirstIndex,            /* QUICK INDEX DATA */\
&OBJ_LastIndex,\
NULL,                       /* RxPDO_EventTimers */\
_RxPDO_EventTimers_Handler, /* RxPDO_EventTimers_Handler */\
\
/* SDO, structure s_transfer */\
{   \
    REPEAT_SDO_MAX_SIMULTANEOUS_TRANSFERS_TIMES(s_transfer_Initializer)\
},\
\
/* PDO Trans Data Begin */\
PDO_Trans_chache,           /* PDO_status */\
/* PDO Trans Data End */\
\
/* SYNC Trans Data Begin */\
_post_sync,                 /* post_sync */\
_post_TPDO,                 /* post_TPDO */\
/* SYNC Trans Data End */\
\
/* EMCY Trans Data Begin */\
Error_free,                /* error_state */\
sizeof(ObjDict_Index1003) / sizeof(ObjDict_Index1003[0]),/* error_history_size */\
&obj1003_preErrRegion_subindex,/* error_number */\
&obj1003_preErrRegion[0],   /* error_first_element */\
&obj1001_errReg,\
&obj1014_CobId_Emcy,        /* error_cobid */\
/* error_data: structure s_errors */\
{   \
    REPEAT_EMCY_MAX_ERRORS_TIMES(ERROR_DATA_INITIALIZER)\
},\
/* EMCY Trans Data End */\
\
NULL,\
NULL,\
                        };
/* PDO DATA INIT */
PDO_TRANS_Status  pPdoPar = {\
    {   /* PDO COMMUNICATE */\
        {   /* TPDO1 COMMUNICATE */\
            0x06,           /* 入口数目 */\
            NULL,           /* 同步报文计数器 */\
            CANID_TPDO1,    /* PDO标识符(32bit) */\
            ASYNC_TRIGGER_B,/* 上一次的传输类型 */\
            0x0001,           /* 触发事件 */\
            0x0032,          /* 禁止时间 */\
            NULL,           /* 保留 */\
            NULL,           /* 远程帧标志 */\
            MSG_DATA_INITIALIZER/* 上一条RPDO报文 */\
        },\
        {   /* TPDO2 COMMUNICATE */\
            0x06,\
            NULL,\
            CANID_TPDO2,\
            ASYNC_TRIGGER_B,\
            0x0001,\
            0x0032,\
            NULL,\
            NULL,\
            MSG_DATA_INITIALIZER\
        },\
        {   /* TPDO3 COMMUNICATE */\
            0x06,\
            NULL,\
            CANID_TPDO3,\
            ASYNC_TRIGGER_B,\
            0x0001,\
            0x0032,\
            NULL,\
            NULL,\
            MSG_DATA_INITIALIZER\
        },\
        {   /* TPDO4 COMMUNICATE */\
            0x06,\
            NULL,\
            CANID_TPDO4,\
            ASYNC_TRIGGER_B,\
            0x0001,\
            0x0032,\
            NULL,\
            NULL,\
            MSG_DATA_INITIALIZER\
        },\
        {   /* RPDO1 COMMUNICATE */\
            0x02,\
            NULL,\
            CANID_RPDO1,\
            SYNC_CYCLE_BEGIN,\
            NULL,\
            NULL,\
            NULL,\
            NULL,\
            MSG_DATA_INITIALIZER\
        },\
        {   /* RPDO2 COMMUNICATE */\
            0x02,\
            NULL,\
            CANID_RPDO2,\
            SYNC_CYCLE_BEGIN,\
            NULL,\
            NULL,\
            NULL,\
            NULL,\
            MSG_DATA_INITIALIZER\
        },\
        {   /* RPDO3 COMMUNICATE */\
            0x02,\
            NULL,\
            CANID_RPDO3,\
            SYNC_CYCLE_BEGIN,\
            NULL,\
            NULL,\
            NULL,\
            NULL,\
            MSG_DATA_INITIALIZER\
        },\
        {   /* RPDO4 COMMUNICATE */\
            0x02,\
            NULL,\
            CANID_RPDO4,\
            SYNC_CYCLE_BEGIN,\
            NULL,\
            NULL,\
            NULL,\
            NULL,\
            MSG_DATA_INITIALIZER\
        },\
    },\
    {   /* PDO MAP */\
        {   /* TPDO1 MAP */\
            0x2,       /* 映射参数子索引数Table[8][1] */\
            {   /* show status */\
                {0x60640120},{0x60640220},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL}\
            }\
        },\
        {   /* TPDO2 MAP */\
            0x2,\
            {   /* show status and current mode f operation */\
                {0x606C0120},{0x606C0220},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL}\
            }\
        },\
        {   /* TPDO3 MAP */\
            0x2,\
            {   /* show status and current position(pp) */\
                {0x60780120},{0x60780220},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL}\
            }\
        },\
        {   /* TPDO4 MAP */\
            0x4,\
            {   /* show status and current Vel(pv) */\
                {0x60410110},{0x60410210},{0x60600008},{0x60600108},{NULL},{NULL},{NULL},{NULL}\
            }\
        },\
        {   /* RPDO1 MAP */\
            0x1,\
            {   /* Default:Controlword */\
                {0x60FF0320},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL}\
            }\
        },\
        {   /* RPDO2 MAP */\
            0x0,\
            {   /* Default:Controlword/OperationMode */\
                {NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL}\
            }\
        },\
        {   /* RPDO3 MAP */\
            0x0,\
            {   /* Default:Controlword/Target Position(pp) */\
                {NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL}\
            }\
        },\
        {   /* RPDO4 MAP */\
            0x0,\
            {   /* Default:Controlword/Target Vel(pv) */\
                {NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL}\
            }\
        },\
    }\
};
/* PDO DIRECT TRANS DATA INIT */
PDO_DrctTrans_Struct pRpdoDirectPar[4];
/* HEART_BEAT INIT */
HEART_BEAT_Status pHeartBeatPar = {\
                                   NULL,   /* 心跳报文远程帧信号 */\
                                   NULL,   /* 生产者心跳超时对象索引 */\
                                   TIMER_NONE,   /* 定时事件 */\
                                   NULL,   /* 节点保护同步 */\
                                  };
/* OBJECT IDENTITY INIT */
pCANopen_ID_Status  Ob_ID = {\
                             0x4,    /* 最大子索引数 */\
                             NULL,   /* Vendor-ID */\
                             NULL,   /* 产品码 */\
                             NULL,   /* 修订版本号 */\
                             NULL,   /* 序列号 */\
                            };



