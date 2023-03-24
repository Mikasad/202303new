#include "canopen_sync.h"


/*------------------------------------------------
Function:定义是否生成同步消息
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
void StopSync(CO_Data *d)
{
    RegisterSetOnDentry_Callback(&CANopen_Drive,0x1005,0,NULL);//COB-ID同步消息(SYNC对象的COB-ID)
    RegisterSetOnDentry_Callback(&CANopen_Drive,0x1006,0,NULL);//通信循环周期，如果为0则禁用SYNC
}
/*------------------------------------------------
Function:初始化SYNC
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void ResrtSync(void)
{

}
/*------------------------------------------------
Function:处理同步报文
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
unsigned char proceedSYNC(CO_Data *d)
{
    unsigned char res;

    (*d->post_sync)(d);

    if(!d->CurrentCommunicationState.csPDO)
        return 0;

    RPDO_SYNC_SIGN |= RPDO_SYNC_READY;

    res = _sendPDOevent(d,pRpdoDirectPar,1);

    (*d->post_TPDO)(d);

    return res;
}
/*------------------------------------------------
Function:接收到同步报文回调函数
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
void _post_sync(CO_Data* d) {}
/*------------------------------------------------
Function:tpdo报文回调函数
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
void _post_TPDO(CO_Data* d) {}
