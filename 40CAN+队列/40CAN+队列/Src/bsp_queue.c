/**************************************************************************************************
  * @file           : bsp_queue.c
  * @brief          : quene function body
**************************************************************************************************/
#include "bsp_queue.h"


/****************************************************************************************
*函 数 名: void QueInit(QUE_DATA_LIST *QueDataDst, void *tmpQue, u16 uRQueTotalSize, u16 uRQueLen)
*功能说明: init the quene                
*形    参: QueDataDst:your quene pointer, tmpQue:the data stored the data,
			uRTmpQueLen: length of per msg in quene 
*返 回 值: none
*注    意: none
****************************************************************************************/
void QueInit(QUE_DATA_LIST *QueDataDst, void *tmpQue, u16 uRQueTotalSize, u16 uRQueLen)
{ 
	QueDataDst->uFReadPos = QueDataDst->uFWritePos = 0;
	QueDataDst->uRQueSize = uRQueTotalSize;//the size of your que
	
	QueDataDst->uRPerMsgLen = uRQueLen;//per que length
	QueDataDst->pQue = tmpQue;//pointer of your golabal que array
}

/****************************************************************************************
*函 数 名: u16 isQueFull(QUE_DATA_LIST *QueDataDst)
*功能说明: is the quene full?           
*形    参: QueDataDst:your quene pointer
*返 回 值: none
*注    意: none
****************************************************************************************/
u16 isQueFull(QUE_DATA_LIST *QueDataDst)
{
    return (((QueDataDst->uFWritePos+1) % QueDataDst->uRQueSize) == QueDataDst->uFReadPos);
}

/****************************************************************************************
*函 数 名: u16 isQueEmpty(QUE_DATA_LIST *QueDataDst)
*功能说明: is the quene empty?           
*形    参: QueDataDst:your quene pointer
*返 回 值: none
*注    意: none
****************************************************************************************/
u16 isQueEmpty(QUE_DATA_LIST *QueDataDst)
{
    return (QueDataDst->uFWritePos == QueDataDst->uFReadPos);
}

/****************************************************************************************
*函 数 名: u16 PutQueData(QUE_DATA_LIST *QueDataDst, void *pNode)
*功能说明: Put data to quene           
*形    参: QueDataDst:your quene pointer, pNode:the pointer of the new data
*返 回 值: none
*注    意: none
****************************************************************************************/
u16 PutQueData(QUE_DATA_LIST *QueDataDst, void *pNode)
{
    if (isQueFull(QueDataDst))
    {
        //printf("queque is overflow!\r\n");
        return 1;
    }
    else
    {
        memcpy((u8*)QueDataDst->pQue+(QueDataDst->uFWritePos*QueDataDst->uRPerMsgLen),pNode,QueDataDst->uRPerMsgLen);
        QueDataDst->uFWritePos = (QueDataDst->uFWritePos+1) % QueDataDst->uRQueSize;
        return 0;
    }
}

/****************************************************************************************
*函 数 名: u16 GetQueData(QUE_DATA_LIST *QueDataDst, void *pNode)
*功能说明: get data from quene           
*形    参: QueDataDst:your quene pointer, pNode:the pointer of the data read out 
*返 回 值: none
*注    意: none
****************************************************************************************/
u16 GetQueData(QUE_DATA_LIST *QueDataDst, void *pNode)
{
    if (isQueEmpty(QueDataDst))
    {
        //printf("queque is empty!\r\n");
        return 1;
    }
    else
    {
        memcpy(pNode,(u8*)QueDataDst->pQue+(QueDataDst->uFReadPos*QueDataDst->uRPerMsgLen),QueDataDst->uRPerMsgLen);
        QueDataDst->uFReadPos = (QueDataDst->uFReadPos+1) % QueDataDst->uRQueSize;

		return 0;
    }
}

/****END OF FILE****/
