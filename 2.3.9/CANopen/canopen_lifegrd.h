#ifndef __CANOPEN_LIFEGRD_H__
#define __CANOPEN_LIFEGRD_H__

#include "canopen_od.h"
#include "canopen_def.h"
#include "canopen_nmt.h"
#include "bsp_can.h"


void ConsumerHeartbeatAlarm(CO_Data* d, unsigned int id);
void HeartBeat_Message_Handle(CO_Data *d, CanRxMsgTypeDef *m);
void ProducerHeartbeatAlarm(CO_Data *d, unsigned int id);
void heartbeatInit(CO_Data *d);
void heartbeatStop(CO_Data *d);
unsigned int OnHeartbeatProducerUpdate(CO_Data *d, const indextable *unused_indextable, unsigned char unused_bSubindex);
void lifeGuardInit(CO_Data *d);
void lifeGuardStop(CO_Data *d);
void _heartbeatError(CO_Data *d, unsigned char heartbeatID);
void _post_SlaveStateChange(CO_Data* d, unsigned char nodeId, unsigned char newNodeState);
unsigned int  ODCallback_t_Index1016_Subindex(CO_Data *d, const indextable *index, unsigned char bSubindex);

#endif
