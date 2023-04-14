#ifndef __CANOPEN_PDO_H__
#define __CANOPEN_PDO_H__

#include "canopen_od.h"
#include "canopen_def.h"
#include "bsp_can.h"

/* MACRO DEFINE BEGIN */

#define PDO_MAP_PERMISON
/* TPDO处于禁止状态 */
#define PDO_INHIBITED 0x01
/* 远程同步已经准备好 */
#define PDO_RTR_SYNC_READY 0x01
/* 同步RPDO数据已经准备好 */
#define PDD_REC_DATA_READY 0x01
/* 同步信号已被预处理 */
#define RPDO_SYNC_READY    0x01
  
/* MACRO DEFINE END */

unsigned char buildPDO(CO_Data *d, unsigned char numPdo, CanTxMsgTypeDef * pdo);
unsigned char sendPDOrequest(CO_Data *d, unsigned short int RPDOIndex);
unsigned char proceedPDO(CO_Data *d, CanRxMsgTypeDef *m);
void CopyBits(unsigned char NbBits, unsigned char *SrcByteIndex, unsigned char SrcBitIndex, unsigned char SrcBigEndian, 
							unsigned char *DestByteIndex, unsigned char DestBitIndex, unsigned char DestBigEndian);
static void sendPdo(CO_Data * d, unsigned int pdoNum, CanTxMsgTypeDef *pdo);
unsigned char sendPDOevent(CO_Data *d);
unsigned char sendOnePDOevent(CO_Data *d, unsigned char pdoNum);
void PDOEventTimerAlarm(CO_Data *d, unsigned int pdoNum);
void PDOInhibitTimerAlarm(CO_Data *d, unsigned int pdoNum);
void _RxPDO_EventTimers_Handler(CO_Data *d, unsigned int pdoNum);
unsigned char _sendPDOevent(CO_Data *d,PDO_DrctTrans_Struct *DrctPdo, unsigned char isSyncEvent);
unsigned char _RPOD_SyncEvent(CO_Data *d,PDO_DrctTrans_Struct *DrctPdo, unsigned char isSyncEvent);
unsigned int TPDO_Communication_Parameter_Callback(CO_Data *d, const indextable *OD_entry, unsigned char bSubindex);
void PDOInit(CO_Data *d);
void PDOStop(CO_Data *d);
unsigned int RPDO_Direct_Trans_Init(CO_Data *d,PDO_DrctTrans_Struct *Data);
unsigned int  ODCallback_t_RPDO_Communicate(CO_Data *d, const indextable *index, unsigned char bSubindex);
unsigned int  ODCallback_t_TPDO_Communicate(CO_Data *d, const indextable *index, unsigned char bSubindex);
unsigned int  ODCallback_t_TPDO_MapSubindex0(CO_Data *d, const indextable *index, unsigned char bSubindex);
unsigned int  ODCallback_t_TPDO_Map(CO_Data *d, const indextable *index, unsigned char bSubindex);
unsigned int  ODCallback_t_RPDO_MapSubindex0(CO_Data *d, const indextable *index, unsigned char bSubindex);
unsigned int  ODCallback_t_RPDO_Map(CO_Data *d, const indextable *index, unsigned char bSubindex);
unsigned int RPDO_Direct_SetODentry(CO_Data *d,PDO_DrctTrans_Struct *DrctPdo,unsigned int pdoNum,unsigned int rpdoSubindex,unsigned char *pSourceData,unsigned char checkAccess);

#endif
