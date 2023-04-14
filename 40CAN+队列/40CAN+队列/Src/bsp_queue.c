/**************************************************************************************************
  * @file           : bsp_queue.c
  * @brief          : quene function body
**************************************************************************************************/
#include "bsp_queue.h"


/****************************************************************************************
*�� �� ��: void QueInit(QUE_DATA_LIST *QueDataDst, void *tmpQue, u16 uRQueTotalSize, u16 uRQueLen)
*����˵��: init the quene                
*��    ��: QueDataDst:your quene pointer, tmpQue:the data stored the data,
			uRTmpQueLen: length of per msg in quene 
*�� �� ֵ: none
*ע    ��: none
****************************************************************************************/
void QueInit(QUE_DATA_LIST *QueDataDst, void *tmpQue, u16 uRQueTotalSize, u16 uRQueLen)
{ 
	QueDataDst->uFReadPos = QueDataDst->uFWritePos = 0;
	QueDataDst->uRQueSize = uRQueTotalSize;//the size of your que
	
	QueDataDst->uRPerMsgLen = uRQueLen;//per que length
	QueDataDst->pQue = tmpQue;//pointer of your golabal que array
}

/****************************************************************************************
*�� �� ��: u16 isQueFull(QUE_DATA_LIST *QueDataDst)
*����˵��: is the quene full?           
*��    ��: QueDataDst:your quene pointer
*�� �� ֵ: none
*ע    ��: none
****************************************************************************************/
u16 isQueFull(QUE_DATA_LIST *QueDataDst)
{
    return (((QueDataDst->uFWritePos+1) % QueDataDst->uRQueSize) == QueDataDst->uFReadPos);
}

/****************************************************************************************
*�� �� ��: u16 isQueEmpty(QUE_DATA_LIST *QueDataDst)
*����˵��: is the quene empty?           
*��    ��: QueDataDst:your quene pointer
*�� �� ֵ: none
*ע    ��: none
****************************************************************************************/
u16 isQueEmpty(QUE_DATA_LIST *QueDataDst)
{
    return (QueDataDst->uFWritePos == QueDataDst->uFReadPos);
}

/****************************************************************************************
*�� �� ��: u16 PutQueData(QUE_DATA_LIST *QueDataDst, void *pNode)
*����˵��: Put data to quene           
*��    ��: QueDataDst:your quene pointer, pNode:the pointer of the new data
*�� �� ֵ: none
*ע    ��: none
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
*�� �� ��: u16 GetQueData(QUE_DATA_LIST *QueDataDst, void *pNode)
*����˵��: get data from quene           
*��    ��: QueDataDst:your quene pointer, pNode:the pointer of the data read out 
*�� �� ֵ: none
*ע    ��: none
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
