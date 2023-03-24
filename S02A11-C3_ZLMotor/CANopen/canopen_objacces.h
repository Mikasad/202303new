#ifndef __CANOPEN_OBJACCES_H__
#define __CANOPEN_OBJACCES_H__

#include "canopen_def.h"
#include "canopen_nmt.h"
#include "canopen_od.h"
#include "objectdef.h"

/* CONTENT BEGIN */

/* 对象字典查询错误 - 报告 */
unsigned char OBJ_Access_Err_Record(unsigned short int Index,unsigned char SubIndex,unsigned int DataSize,unsigned int DataSizeGiven,unsigned int errcode);
/* 通过字典索引和子索引将数据拷贝出来 */
unsigned int _getODentry(CO_Data *d, unsigned short int wIndex, unsigned char bSubindex, void *pDestData, unsigned int *pExpectedSize,\
                         unsigned char *pDataType, unsigned char checkAccess, unsigned char endianize);
#define getODentry(OD, wIndex, bSubindex, pDestData, pExpectedSize, pDataType,  checkAccess)		\
       _getODentry(OD, wIndex, bSubindex, pDestData, pExpectedSize, pDataType,  checkAccess, 0)
/* 通过字典索引和子索引将数据拷贝进去 */
unsigned int _setODentry(CO_Data *d, unsigned short int wIndex, unsigned char bSubindex, void *pSourceData,\
                         unsigned int *pExpectedSize, unsigned char checkAccess, unsigned char endianize);
#define setODentry(d, wIndex, bSubindex, pSourceData, pExpectedSize, checkAccess) \
				_setODentry(d, wIndex, bSubindex, pSourceData, pExpectedSize, checkAccess, 0)
/* 查字典，从中取出索引号为wIndex的条目的回调函数 */
const indextable *OD_Index_san(CO_Data *d, unsigned short int wIndex, unsigned int *errorCode, ODCallback_t **Callback);
unsigned int RegisterSetOnDentry_Callback(CO_Data *d, unsigned short int wIndex, unsigned char bSubindex, ODCallback_t Callback);
/* CONTENT END */
unsigned int CallOn_ObjDictionary(unsigned int Index,unsigned char subIndex,unsigned int* CoodStat,unsigned int* odNum);


#endif
