#include "canopen_sdo.h"

#define SDO_SERVICE

#ifdef SDO_SERVICE
/*------------------------------------------------
Function:SDO传输超时回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void SDOTimeoutAlarm(CO_Data *d, unsigned int id)
{
    unsigned short int offset;
    unsigned char nodeId;

    offset = d->firstIndex->SDO_CLT;
    if((offset == 0) || ((offset + d->transfers[id].CliServNbr) > d->lastIndex->SDO_CLT))
    {
        return;
    }


    nodeId = (unsigned char)*((unsigned int*)d->objdict[offset + d->transfers[id].CliServNbr].pSubindex[3].lpParam);

//  ATLAS_ERR("SDO timeout. SDO response not received");
//  ATLAS_ERR("server node id : %d", nodeId);
//  ATLAS_ERR("         index : %d", d->transfers[id].index);
//  ATLAS_ERR("      subIndex : %d", d->transfers[id].subIndex);

    d->transfers[id].timer = TIMER_NONE;
    d->transfers[id].state = SDO_ABORTED_INTERNAL;

    SDO_Err_Record(d, d->transfers[id].whoami, d->transfers[id].CliServNbr, d->transfers[id].index, d->transfers[id].subIndex, SDOABT_TIMED_OUT);
    d->transfers[id].abortCode = SDOABT_TIMED_OUT;

    if(d->transfers[id].Callback)
        (*d->transfers[id].Callback)(d, nodeId);

    if(d->transfers[id].abortCode == SDOABT_TIMED_OUT)
    {
        resetSDOline(d, (unsigned char)id);
    }

}

#define StopSDO_TIMER(id) \
d->transfers[id].timer = DelAlarm(d->transfers[id].timer);

#define StartSDO_TIMER(id) \
	d->transfers[id].timer = SetAlarm(d, id, &SDOTimeoutAlarm,US_TO_TIMEVAL(SDO_TIMEOUT_MS),0);

#define RestartSDO_TIMER(id) \
if(d->transfers[id].timer != TIMER_NONE) { StopSDO_TIMER(id) StartSDO_TIMER(id) }

/*------------------------------------------------
Function:复位所有sdo传输通道
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void resetSDO(CO_Data *d)
{
    unsigned char j;

    for(j = 0; j < SDO_MAX_SIMULTANEOUS_TRANSFERS; j++)
    {
        resetSDOline(d, j);
    }
}
/*------------------------------------------------
Function:将数据从传输通道缓冲区拷贝到字典中
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int SDOlineToObjdict(CO_Data *d, unsigned char line)
{
    unsigned int size;
    unsigned int errorCode;

    //ATLAS_PRINT("Enter in SDOlineToObjdict : %d\r\n",line);

    /* 确定该传输通道传输了多少字节 */
    if(d->transfers[line].count == 0)
    {
        d->transfers[line].count = d->transfers[line].offset;
    }
    size = d->transfers[line].count;

    errorCode = setODentry(d, d->transfers[line].index, d->transfers[line].subIndex,
                           (void *)d->transfers[line].data, &size, 1);
    if(errorCode != OD_SUCCESSFUL)
        return errorCode;

    //ATLAS_PRINT("exit of SDOlineToObjdict : %d\r\n",line);

    return 0;

}


/*------------------------------------------------
Function:将数据从字典中拷贝到传输通道缓冲区
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int objdictToSDOline(CO_Data *d, unsigned char line)
{
    unsigned int size = SDO_MAX_LENGTH_TRANSFER;
    unsigned char dataType;
    unsigned int errorCode;

    //ATELAS_MAR("obj to trans chache",d->transfers[line].index,d->transfers[line].subIndex);

    errorCode = getODentry(d, d->transfers[line].index, d->transfers[line].subIndex,
                           (void *)d->transfers[line].data, &size, &dataType, 1);

    if(errorCode != OD_SUCCESSFUL)
        return errorCode;

    d->transfers[line].count = size;
    d->transfers[line].offset = 0;

    return 0;
}

/*------------------------------------------------
Function:将数据从传输通道缓冲区拷贝出来
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char lineToSDO(CO_Data *d, unsigned char line, unsigned int nbBytes, unsigned char *data)
{
    unsigned char i;
    unsigned int offset;

    if((d->transfers[line].offset + nbBytes) > d->transfers[line].count)
    {
//		ATLAS_ERR("SDO Size of data too large. Exceed count : %d\r\n", nbBytes);
        return 0xFF;
    }
    offset = d->transfers[line].offset;

    for(i = 0; i < nbBytes; i++)
        *(data + i) = d->transfers[line].data[offset + i];

    /* 偏移量增大 */
    d->transfers[line].offset = d->transfers[line].offset + nbBytes;

    return 0;
}
/*------------------------------------------------
Function:将数据拷贝到传输通道缓冲区
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char SDOtoLine(CO_Data *d, unsigned char line, unsigned int nbBytes, unsigned char *data)
{
    unsigned char i;
    unsigned int offset;

    /* 缓冲区偏移量 */
    offset = d->transfers[line].offset;

    for (i = 0; i < nbBytes; i++)
        d->transfers[line].data[offset + i] = *(data + i);

    d->transfers[line].offset = d->transfers[line].offset + nbBytes;

    return 0;
}
/*------------------------------------------------
Function:复位sdo通道
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void resetSDOline(CO_Data *d, unsigned char line)
{
    unsigned int i;

    //ATLAS_PRINT("reset SDO line nb : %d\r\n",line);

    initSDOline(d, line, 0, 0, 0, SDO_RESET);

    for(i = 0; i < SDO_MAX_LENGTH_TRANSFER; i++)
        d->transfers[line].data[i] = 0;

    d->transfers[line].whoami = 0;

    d->transfers[line].abortCode = 0;
}

/*------------------------------------------------
Function:sdo传输通道失败
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char failedSDO(CO_Data *d, unsigned char CliServNbr, unsigned char whoami, unsigned short int index, unsigned char subIndex, unsigned int abortCode)
{
    unsigned char err;
    unsigned char line;

    err = getSDOlineOnUse(d, CliServNbr, whoami, &line);
    if(!err)
    {
        //ATLAS_PRINT("FailedSDO : line found : %d\r\n", line);
    }

    if((!err) && (whoami == SDO_SERVER))
    {
        resetSDOline(d, line);
        //ATLAS_PRINT("FailedSDO : line released : %d\r\n", line);
    }
    if((!err) && (whoami == SDO_CLIENT))
    {
        StopSDO_TIMER(line);
        d->transfers[line].state = SDO_ABORTED_INTERNAL;
        d->transfers[line].abortCode = abortCode;
    }

    //ATLAS_PRINT("Sending SDO abort ");

    /* 发送sdo中止报文 */
    err = SDO_Err_Record(d, whoami, CliServNbr, index, subIndex, abortCode);
    if(err)
    {
//		ATLAS_ERR("Unable to send the SDO abort\r\n");
        return 0xFF;
    }

    return 0;
}
/*------------------------------------------------
Function:初始化sdo传输通道
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char initSDOline(CO_Data *d, unsigned char line, unsigned char CliServNbr, unsigned short int index, unsigned char subIndex, unsigned char state)
{
    //ATLAS_PRINT("init SDO line nb : %d\r\n",line);

    if(state == SDO_DOWNLOAD_IN_PROGRESS || state == SDO_UPLOAD_IN_PROGRESS ||
            state == SDO_BLOCK_DOWNLOAD_IN_PROGRESS || state == SDO_BLOCK_UPLOAD_IN_PROGRESS)
    {
        StartSDO_TIMER(line)
    }
    else
    {
        StopSDO_TIMER(line)
    }


    d->transfers[line].CliServNbr = CliServNbr;

    d->transfers[line].index = index;

    d->transfers[line].subIndex = subIndex;

    d->transfers[line].state = state;
    d->transfers[line].toggle = 0;

    d->transfers[line].count = 0;
    d->transfers[line].offset = 0;
    d->transfers[line].peerCRCsupport = 0;
    d->transfers[line].blksize = 0;
    d->transfers[line].ackseq = 0;
    d->transfers[line].objsize = 0;
    d->transfers[line].lastblockoffset = 0;
    d->transfers[line].seqno = 0;
    d->transfers[line].endfield = 0;
    /* 初始化块传输接收状态 */
    d->transfers[line].rxstep = RXSTEP_INIT;
    d->transfers[line].dataType = 0;

    d->transfers[line].Callback = NULL;

    return 0;
}
/*------------------------------------------------
Function:获取空闲的sdo传输通道
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char getSDOfreeLine(CO_Data *d, unsigned char whoami, unsigned char *line)
{
    unsigned char i;


    for(i = 0; i < SDO_MAX_SIMULTANEOUS_TRANSFERS; i++)
    {
        if(d->transfers[i].state == SDO_RESET)
        {
            *line = i;
            d->transfers[i].whoami = whoami;
            return 0;
        }
    }

    //ATLAS_PRINT("Too many SDO in progress. Aborted. %d\r\n", i);

    return 0xFF;
}
/*------------------------------------------------
Function:通过客户端/服务器号获取sdo传输通道号
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char getSDOlineOnUse(CO_Data *d, unsigned char CliServNbr, unsigned char whoami, unsigned char *line)
{
    unsigned char i;

    /* 遍历所有sdo传输通道 */
    for(i = 0; i < SDO_MAX_SIMULTANEOUS_TRANSFERS; i++)
    {
        if((d->transfers[i].state != SDO_RESET) &&
                (d->transfers[i].state != SDO_ABORTED_INTERNAL) &&
                (d->transfers[i].CliServNbr == CliServNbr) &&
                (d->transfers[i].whoami == whoami))
        {
            if(line)
            {
                *line = i;
            }

            return 0;
        }
    }

    return 0xFF;
}
/*------------------------------------------------
Function:通过客户端/服务器号获取sdo传输通道号
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char getSDOlineToClose(CO_Data *d, unsigned char CliServNbr, unsigned char whoami, unsigned char *line)
{
    unsigned char i;

    for(i = 0; i < SDO_MAX_SIMULTANEOUS_TRANSFERS; i++)
    {
        if((d->transfers[i].state != SDO_RESET) &&
                (d->transfers[i].CliServNbr == CliServNbr) &&
                (d->transfers[i].whoami == whoami))
        {
            if(line)
                *line = i;

            return 0;
        }
    }

    return 0xFF;
}
/*------------------------------------------------
Function:关闭sdo传输通道
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char closeSDOtransfer(CO_Data *d, unsigned char nodeId, unsigned char whoami)
{
//	unsigned char err;
//	unsigned char line;
//	unsigned char CliNbr;
//
//	/* 通过节点号查找客户端号 */
//	CliNbr = GetSDOClientFromNodeId(d, nodeId);
//	if(CliNbr >= 0xFE)
//		return SDO_ABORTED_INTERNAL;
//
//	/* 通过客户端/服务器号获取sdo传输通道号 */
//	err = getSDOlineToClose(d, CliNbr, whoami, &line);
//	if(err)
//	{
//		MSG_WAR(0x2A30, "No SDO communication to close", 0);
//		return 0xFF;
//	}

//	/* 复位sdo通道 */
//	resetSDOline(d, line);

    return 0;
}
/*------------------------------------------------
Function:获取SDO传输通道剩余字节数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char getSDOlineRestBytes(CO_Data *d, unsigned char line, unsigned int *nbBytes)
{
    if(d->transfers[line].count == 0)
        *nbBytes = 0;
    else
        *nbBytes = d->transfers[line].count - d->transfers[line].offset;
    return 0;
}
/*------------------------------------------------
Function:设置sdo传输通道剩余字节数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char setSDOlineRestBytes(CO_Data *d, unsigned char line, unsigned int nbBytes)
{
    d->transfers[line].count = nbBytes;

    return 0;
}

#endif
/*------------------------------------------------
Function:发送SDO报文
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char SendSDO(CO_Data *d,unsigned char whoami,unsigned char CliServNbr,unsigned char *data)
{
    unsigned short int offset;
    CanTxMsgTypeDef m;
    unsigned char i;

    //ATELAS_MAR("SendSDO",0,0);

    if(!((d->Node_state == HEART_STAT_PRE_OPERAT)||(d->Node_state == HEART_STAT_OPERAT)))
    {
        //ATELAS_MAR("unable to send the SDO (not in op or pre-op mode",0,0);
        return 0xFF;
    }

    /* 服务器发送SDO */
    if(whoami == SDO_SERVER)
    {
        offset = d->firstIndex->SDO_SVR;
        if((offset == 0)||((offset+CliServNbr) > d->lastIndex->SDO_SVR))//sdo not exist
        {
            //ATELAS_MAR("SendSDO : SDO server not found",0,0);
            return 0xFF;
        }
        m.StdId = (unsigned short int)*((unsigned int *)d->objdict[offset + CliServNbr].pSubindex[2].lpParam);
        //ATELAS_MAR("I am server Tx cobId :",m.StdId,0);
    }
    for(i=0; i<8; i++)
    {
        m.Data[i] = data[i];
    }
    m.RTR = RTR_INVALID;
    m.DLC = 8;

    return CAN2_Send_Msg(m.StdId,CAN_ID_STD,m.RTR,data,m.DLC);
}

/*------------------------------------------------
Function:SDO终止码报告
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char SDO_Err_Record(CO_Data *d,unsigned char whoami,unsigned char CliServNbr,unsigned int Index,unsigned char subIndex,unsigned int EndCode)
{
    unsigned char data[8];
    unsigned char ret;

    data[0] = (ABORT_TRANSER<<5)&0x80;
    data[1] = (Index>>0)&0xFF;
    data[2] = (Index>>8)&0xFF;
    data[3] = subIndex;
    data[4] = (unsigned char)((EndCode>>0)&0xFF);
    data[5] = (unsigned char)((EndCode>>8)&0xFF);
    data[6] = (unsigned char)((EndCode>>16)&0xFF);
    data[7] = (unsigned char)((EndCode>>24)&0xFF);

    /* Send SDO */
    ret = SendSDO(d,whoami,CliServNbr,data);

    return ret;
}


/*------------------------------------------------
Function:SDO Message handler
Input   :*CO_Data  *m
Output  :No
Explain :No
------------------------------------------------*/
unsigned char SDO_Email_Handler(CO_Data *d,CanRxMsgTypeDef *m)
{
    unsigned char err;
    unsigned char cs;
    unsigned char line;
    unsigned int nbBytes;
//    unsigned char nodeId = 0;
    unsigned char CliServNbr;
    unsigned char whoami = SDO_UNKNOWN;
    unsigned int errorCode;
    unsigned char data[8];
    unsigned short int index;
    unsigned char subIndex;
    unsigned int i;
    unsigned char	j;
    unsigned int *pCobId = NULL;
    unsigned short int offset;
    unsigned short int lastIndex;
//    unsigned int TimeCheck;

    //ATLAS_PRINT("SDO Handle Begin\r\n");

    whoami = SDO_UNKNOWN;

    offset = d->firstIndex->SDO_SVR;

    lastIndex = d->lastIndex->SDO_SVR;

    j = 0;

//    TimeCheck = TIM6->CNT;

    if(offset)
    {
        while(offset <= lastIndex)
        {

            if(d->objdict[offset].bSubcount <= 1)
            {
//				ATLAS_ERR("Subindex 1  not found at index : %d\r\n", 0x1200 + j);
                return 0xFF;
            }

            pCobId = (unsigned int*)d->objdict[offset].pSubindex[1].lpParam;

            if(((*pCobId)&CAN_ID_BIT) == ((UNS16_LE(m->StdId))&CAN_ID_BIT))
            {

                whoami = SDO_SERVER;
                //ATLAS_PRINT("SDO_Email_Handler. I am server. index : %d\r\n", 0x1200 + j);

                CliServNbr = j;
                break;
            }
            j++;
            offset++;
        }
    }
    if(whoami == SDO_UNKNOWN)
    {
        offset = d->firstIndex->SDO_CLT;
        lastIndex = d->lastIndex->SDO_CLT;

        j = 0;

        /* 如果配置了客户端 */
        if(offset)
        {
            /* 遍历所有sdo客户端 */
            while(offset <= lastIndex)
            {
                if(d->objdict[offset].bSubcount <= 3)
                {
//				ATLAS_ERR("Subindex 3  not found at index : %d\r\n", 0x1280 + j);
                    return 0xFF;
                }
                pCobId = (unsigned int*)d->objdict[offset].pSubindex[2].lpParam;
                if(((*pCobId)&CAN_ID_BIT) == ((UNS16_LE(m->StdId))&CAN_ID_BIT))
                {
                    whoami = SDO_CLIENT;

                    //ATLAS_PRINT("proceedSDO. I am client index : %d\r\n", 0x1280 + j);

                    /* 服务器/客户端号 */
                    CliServNbr = j;

                    /* 服务器端的节点号 */
//                    nodeId = *((unsigned char*) d->objdict[offset].pSubindex[3].lpParam);
                    break;
                }
                j++;
                offset++;
            }
        }

        if(whoami == SDO_UNKNOWN)
        {
            return 0xFF;
        }
    }

    if(whoami == SDO_UNKNOWN)
    {
        return 0xFF;
    }

    /* SDO报文一定是8字节 */
    if((*m).DLC != 8)
    {
        ATLAS_ERR("Error size SDO\r\n");
        failedSDO(d, CliServNbr, whoami, 0, 0, SDOABT_GENERAL_ERROR);
        return 0xFF;
    }

    if(whoami == SDO_CLIENT)
    {
        //ATLAS_PRINT("I am CLIENT number :%d\r\n", CliServNbr);
    }
    else
    {
        //ATLAS_PRINT("I am SERVER number :%d\r\n", CliServNbr);
    }

    /* 通服务器/客户端号获取通道号 */
    err = getSDOlineOnUse(d, CliServNbr, whoami, &line);

    cs = 0xFF;
    if(!err)
    {
        if(((whoami == SDO_SERVER) && (d->transfers[line].state == SDO_BLOCK_DOWNLOAD_IN_PROGRESS)) ||
                ((whoami == SDO_CLIENT) && (d->transfers[line].state == SDO_BLOCK_UPLOAD_IN_PROGRESS)))
        {
            if(m->Data[0] == 0x80)
                cs = 4;
            else
                cs = 6;
        }
    }
    if(cs == 0xFF)
    {
        cs = getSDOcs(m->Data[0]);
    }

    /* SDO cs handle */
    switch(cs)
    {
    case SEGMENT_DOWNLOAD_REQUEST:
    {
        if(whoami == SDO_SERVER)
        {
            if(!err)
            {
                err = d->transfers[line].state != SDO_DOWNLOAD_IN_PROGRESS;
            }
            if(err)
            {
//					ATLAS_ERR("SDO error : Received download segment for unstarted trans. index 0x1200 + %d\r\n", CliServNbr);
                failedSDO(d, CliServNbr, whoami, 0, 0, SDOABT_LOCAL_CTRL_ERROR);
                return 0xFF;
            }
            /* 重启传输通道超时定时器 */
            RestartSDO_TIMER(line)

            //ATLAS_PRINT("Received SDO download segment defined at index 0x1200 + %d\r\n", CliServNbr);

            index = d->transfers[line].index;
            subIndex = d->transfers[line].subIndex;

            if(d->transfers[line].toggle != getSDOt(m->Data[0]))
            {
//					ATLAS_ERR("SDO error : Toggle error : %d\r\n", getSDOt(m->Data[0]));
                failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_TOGGLE_NOT_ALTERNED);
                return 0xFF;
            }

            /* 取出字节数 */
            nbBytes = 7 - getSDOn3(m->Data[0]);
            err = SDOtoLine(d, line, nbBytes, (*m).Data + 1);
            if(err)
            {
                failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_GENERAL_ERROR);
                return 0xFF;
            }

            /* 构建响应包 */
            data[0] = (1 << 5) | (d->transfers[line].toggle << 4);
            for(i = 1 ; i < 8 ; i++)
            {
                data[i] = 0;
            }
            //ATLAS_PRINT("SDO. Send response to download request defined at index 0x1200 + %d\r\n", CliServNbr);

            /* 发送响应包 */
            SendSDO(d, whoami, CliServNbr, data);

            d->transfers[line].toggle = !d->transfers[line].toggle & 1;
            if(getSDOc(m->Data[0]))
            {
                errorCode = SDOlineToObjdict(d, line);
                if(errorCode)
                {
//						ATLAS_ERR("SDO error : Unable to copy the data in the object dictionary ");
                    failedSDO(d, CliServNbr, whoami, index, subIndex, errorCode);
                    return 0xFF;
                }
                resetSDOline(d, line);
                //ATLAS_PRINT("SDO. End of download defined at index 0x1200 + %d\r\n", CliServNbr);
            }
        }
    }
    break;
    case INIT_DOWNLOAD_REQUEST://183us
    {
        if(whoami == SDO_SERVER)
        {
            /* 索引 */
            index = getSDOindex(m->Data[1], m->Data[2]);
            /* 子索引 */
            subIndex = getSDOsubIndex(m->Data[3]);
            //ATLAS_PRINT("Received SDO Initiate Download (to store data) defined at index 0x1200 + %d\r\n", CliServNbr);
            //ATLAS_PRINT("Writing at index : %d\r\n", index);
            //ATLAS_PRINT("Writing at subIndex : %d\r\n", subIndex);
            if(!err)
            {
//					ATLAS_ERR("SDO error : Transmission yet started.");
                failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_LOCAL_CTRL_ERROR);
                return 0xFF;
            }

            err = getSDOfreeLine(d, whoami, &line);
            if(err)
            {
//					ATLAS_ERR("SDO error : No line free, too many SDO in progress. Aborted.");
                failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_LOCAL_CTRL_ERROR);
                return 0xFF;
            }
            initSDOline(d, line, CliServNbr, index, subIndex, SDO_DOWNLOAD_IN_PROGRESS);

            if(getSDOe(m->Data[0]))
            {
                nbBytes = 4 - getSDOn2(m->Data[0]);
                d->transfers[line].count = nbBytes;
                err = SDOtoLine(d, line, nbBytes, (*m).Data + 4);
                if(err)
                {
                    failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_GENERAL_ERROR);
                    return 0xFF;
                }

                //ATLAS_PRINT("SDO Initiate Download is an expedited transfer. Finished. ");

                errorCode = SDOlineToObjdict(d, line);
                if(errorCode)
                {
//						ATLAS_ERR("SDO error : Unable to copy the data in the object dictionary");
                    failedSDO(d, CliServNbr, whoami, index, subIndex, errorCode);
                    return 0xFF;
                }
                /* 复位sdo通道 */
                resetSDOline(d, line);
            }
            else
            {
                /* 数据字节为字节计数器 */
                if(getSDOs(m->Data[0]))
                {
                    /* 取出字节数 */
                    nbBytes = (m->Data[4]) + ((unsigned int)(m->Data[5])<<8) + ((unsigned int)(m->Data[6])<<16) + ((unsigned int)(m->Data[7])<<24);
                    err = setSDOlineRestBytes(d, line, nbBytes);
                    if(err)
                    {
                        failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_GENERAL_ERROR);
                        return 0xFF;
                    }
                }
            }
            /* 发送启动域下载响应 */
            data[0] = 3 << 5;
            data[1] = index & 0xFF;
            data[2] = (index >> 8) & 0xFF;
            data[3] = subIndex;
            for(i = 4 ; i < 8 ; i++)
                data[i] = 0;
            SendSDO(d, whoami, CliServNbr, data);
        }
    }
    break;
    case INIT_UPLOAD_REQUEST://251us
    {
        if(whoami == SDO_SERVER)
        {
            index = getSDOindex(m->Data[1], m->Data[2]);
            subIndex = getSDOsubIndex(m->Data[3]);
            //ATLAS_PRINT("Received SDO Initiate upload (to send data) defined at index 0x1200 + %d\r\n", CliServNbr);
            //ATLAS_PRINT("Reading at index : %d\r\n", index);
            //ATLAS_PRINT("Reading at subIndex : %d\r\n", subIndex);
            if(!err)
            {
//					ATLAS_ERR("SDO error : Transmission yet started at line : %d\r\n", line);
                //ATLAS_PRINT("Server Nbr = %d\r\ns", CliServNbr);
                failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_LOCAL_CTRL_ERROR);
                return 0xFF;
            }
            err = getSDOfreeLine(d, whoami, &line);
            //传输通
            if(err)
            {
//					ATLAS_ERR("SDO error : No line free, too many SDO in progress. Aborted.");
                failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_LOCAL_CTRL_ERROR);
                return 0xFF;
            }
            /* 初始化传输通道 */
            initSDOline(d, line, CliServNbr, index, subIndex, SDO_UPLOAD_IN_PROGRESS);
            errorCode = objdictToSDOline(d, line);
            if(errorCode)
            {
//					ATLAS_ERR("SDO error : Unable to copy the data from object dictionary. Err code : %d\r\n", errorCode);
                failedSDO(d, CliServNbr, whoami, index, subIndex, errorCode);
                return 0xFF;
            }
            getSDOlineRestBytes(d, line, &nbBytes);
            if(nbBytes > 4)
            {
                /* 命令 */
                data[0] = (2 << 5) | 1;
                /* 索引 */
                data[1] = index & 0xFF;
                data[2] = (index >> 8) & 0xFF;
                /* 子索引 */
                data[3] = subIndex;
                /* 字节计数器 */
                data[4] = (unsigned char)nbBytes;
                data[5] = (unsigned char)(nbBytes >> 8);
                data[6] = (unsigned char)(nbBytes >> 16);
                data[7] = (unsigned char)(nbBytes >> 24);
                //ATLAS_PRINT("SDO. Sending normal upload initiate response defined at index 0x1200 + %d\r\n", nodeId);
                /* 发送sdo报文 */
                SendSDO(d, whoami, CliServNbr, data);
            }
            /* 字节数不高于4，快速传输 */
            else
            {
                /* 命令 */
                data[0] = (unsigned char)((2 << 5) | ((4 - nbBytes) << 2) | 3);
                /* 索引 */
                data[1] = index & 0xFF;
                data[2] = (index >> 8) & 0xFF;
                /* 子索引 */
                data[3] = subIndex;
                /* 将数据从传输通道缓冲区拷贝到sdo报文 */
                err = lineToSDO(d, line, nbBytes, data + 4);
                if(err)
                {
                    failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_GENERAL_ERROR);
                    return 0xFF;
                }
                /* 不足8字节填0 */
                for(i = 4 + nbBytes; i < 8; i++)
                {
                    data[i] = 0;
                }

                //ATLAS_PRINT("SDO. Sending expedited upload initiate response defined at index 0x1200 + %d\r\n", CliServNbr);

                /* 发送sdo报文 */
                SendSDO(d, whoami, CliServNbr, data);

                /* 复位sdo传输通道 */
                resetSDOline(d, line);
            }
        }
    }
    break;
    case SEGMENT_UPLOAD_REQUEST:
    {
        if(whoami == SDO_SERVER)
        {
            if(!err)
                err = d->transfers[line].state != SDO_UPLOAD_IN_PROGRESS;
            if(err)
            {
//					ATLAS_ERR("SDO error : Received upload segment for unstarted trans. index 0x1200 + %d\r\n", CliServNbr);
                failedSDO(d, CliServNbr, whoami, 0, 0, SDOABT_LOCAL_CTRL_ERROR);

                return 0xFF;
            }

            RestartSDO_TIMER(line)

            //ATLAS_PRINT("Received SDO upload segment defined at index 0x1200 + %d\r\n", CliServNbr);

            index = d->transfers[line].index;
            subIndex = d->transfers[line].subIndex;

            if(d->transfers[line].toggle != getSDOt(m->Data[0]))
            {
//					ATLAS_ERR("SDO error : Toggle error : %d\r\n", getSDOt(m->Data[0]));
                failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_TOGGLE_NOT_ALTERNED);

                return 0xFF;
            }

            /* 获取传输通道剩余字节数 */
            getSDOlineRestBytes(d, line, &nbBytes);
            if(nbBytes > 7)
            {
                /* 构建报文 */
                data[0] = (d->transfers[line].toggle << 4);
                /* 将数据从sdo传输通道缓冲区拷贝到sdo报文中 */
                err = lineToSDO(d, line, 7, data + 1);
                if(err)
                {
                    failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_GENERAL_ERROR);
                    return 0xFF;
                }

                /* 将触发位取反 */
                d->transfers[line].toggle = !d->transfers[line].toggle & 1;

                //ATLAS_PRINT("SDO. Sending upload segment defined at index 0x1200 + %d\r\n", CliServNbr);

                /* 发送报文 */
                SendSDO(d, whoami, CliServNbr, data);
            }
            else
            {
                /* 构建报文 */
                data[0] = (unsigned char)((d->transfers[line].toggle << 4) | ((7 - nbBytes) << 1) | 1);
                /* 将数据从sdo传输通道缓冲区拷贝到sdo报文中 */
                err = lineToSDO(d, line, nbBytes, data + 1);
                if (err) {
                    failedSDO(d, CliServNbr, whoami, index, subIndex, SDOABT_GENERAL_ERROR);
                    return 0xFF;
                }
                /* 不满8字节补0 */
                for(i = nbBytes + 1; i < 8; i++)
                    data[i] = 0;

                //ATLAS_PRINT("SDO. Sending last upload segment defined at index 0x1200 + %d\r\n", CliServNbr);

                /* 发送报文 */
                SendSDO(d, whoami, CliServNbr, data);

                /* 复位传输通道 */
                resetSDOline(d, line);
            }
        }
    }
    break;
    default:
//      ATLAS_ERR("SDO. Received unknown command specifier : %d\r\n", cs);
        break;
    }

// TimeConsume.SDO = (TIM6->CNT - TimeCheck)>(TIMER6_PERIOD/2)?(TIM6->CNT - TimeCheck + TIMER6_PERIOD):(TIM6->CNT - TimeCheck);

    return 0;
}

