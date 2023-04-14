/**************************************************************************************************
  * @file           : bsp_queue.h
  * @brief          : quene data functions and data struct
**************************************************************************************************/

#ifndef __BSP_QUENE_H
#define __BSP_QUENE_H

//---------------------------------------------------------------------------------------
#include "main.h"

//---------------------------------------------------------------------------------------
typedef struct
{
	u16 uFReadPos;
	u16 uFWritePos;
	u16 uRQueSize;
	u16 uRPerMsgLen;
	void *pQue;
}QUE_DATA_LIST;

//---------------------------------------------------------------------------------------
void QueInit(QUE_DATA_LIST *QueDataDst, void *tmpQue, u16 uRQueTotalSize, u16 uRQueLen);
u16 isQueFull(QUE_DATA_LIST *QueDataDst);
u16 isQueEmpty(QUE_DATA_LIST *QueDataDst);
u16 PutQueData(QUE_DATA_LIST *QueDataDst, void *pNode);
u16 GetQueData(QUE_DATA_LIST *QueDataDst, void *pNode);

#endif /* __BSP_QUENE_H */





