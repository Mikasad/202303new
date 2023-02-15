#ifndef __CANOPEN_SYNC_H__
#define __CANOPEN_SYNC_H__
#include "canopen_objacces.h"
#include "canopen_od.h"
#include "canopen_nmt.h"
#include "canopen_pdo.h"

void StopSync(CO_Data *d);
void ResrtSync(void);

unsigned char proceedSYNC(CO_Data *d);
void _post_sync(CO_Data* d);
void _post_TPDO(CO_Data* d);

#endif
