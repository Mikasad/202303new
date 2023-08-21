/********************************************************************************
*文件名			: hc_can.c
*简  介			: can数据收发测试。
*日	 期			: 2022/6/15
*作  者			: 刘德明 
********************************************************************************
*使用范例： 
*        
* 		 
********************************************************************************/

/******************************头文件*******************************************/
#include "hc_can.h"
#include "drv_h4aa30_usb_iap.h"

extern uint32_t otaing_tick;
extern ApplicationTypeDef Appli_state;

QueueHandle_t bms_can2_rx_quene_handle = NULL;
QueueHandle_t anti_can2_rx_quene_handle = NULL;

#define  USART_DEBUG_ON   0

#if  USART_DEBUG_ON
#define CAN_DEBUG_PRINTF(...) do {  printf(__VA_ARGS__); } while(0)
#else
#define  CAN_DEBUG_PRINTF(...) do { } while(0)
#endif




/* Number of RX frame */
#define CAN_RX_FRAME_NUM                (8U)



static void can_data_handle(void);
static void CAN1_IrqCallback(void);
static void CAN2_IrqCallback(void);

 
const static char *m_s8IDTypeStr[] = {
    "standard",
    "extended",
};

const static char *m_s8FrameTypeStr[] = {
    "classical",
    "CAN-FD",
};

const static char *m_s8ErrorTypeStr[] = {
    "NO error.",
    "Bit Error.",
    "Form Error.",
    "Stuff Error.",
    "ACK Error.",
    "CRC Error.",
    "Other Error.",
    "Error type is NOT defined.",
};
 

static stc_can_rx_frame_t m_astRxFrame[CAN_RX_FRAME_NUM];

/****************************************************************************
*函数名	: can_function_test
*介	 绍	：can功能测试, 发送使用了系统延时需要放在任务里测试或修改延迟为非系统延时
*形  参 : 无 
*返回值 : 无
******************************************************************************/
int32_t can_function_test(void)
{
 
    /* CAN1参数配置 */
	can1_config_init();
    /* CAN2参数配置 */
	can2_config_init();
  
    uint32_t data = 0x12345600;

    for (;;) 
	{
        can_standard_send(CAN1_UNIT, 0x11, &data, 0x08);
		DDL_DelayMS(500);
//      can_extend_send(CAN2_UNIT, 0x22, &data, 0x08);
    }
}

/****************************************************************************
*函数名	: can1_config_init
*介	 绍	：初始化CAN1
*形  参 : 无 
*返回值 : 无
******************************************************************************/
void can1_config_init(void)
{
	CLK_SetCANClockSrc(CAN1_CLK_UNIT, CAN1_CLK_SRC);

	GPIO_SetFunc(CAN1_TX_PORT, CAN1_TX_PIN, CAN1_TX_PIN_FUNC);
	GPIO_SetFunc(CAN1_RX_PORT, CAN1_RX_PIN, CAN1_RX_PIN_FUNC);
	
	stc_can_init_t stcCanInit;
    stc_can_filter_config_t astcFilter[CAN_FILTER_NUM] = {
        {CAN_FILTER1_ID, CAN_FILTER1_ID_MASK, CAN_FILTER1_ID_TYPE},
        {CAN_FILTER2_ID, CAN_FILTER2_ID_MASK, CAN_FILTER2_ID_TYPE},
        {CAN_FILTER3_ID, CAN_FILTER3_ID_MASK, CAN_FILTER3_ID_TYPE},
    };

	(void)CAN_StructInit(&stcCanInit);
    stcCanInit.stcBitCfg.u32Prescaler = 4U;
    stcCanInit.stcBitCfg.u32TimeSeg1  = 15U;
    stcCanInit.stcBitCfg.u32TimeSeg2  = 5U;
    stcCanInit.stcBitCfg.u32SJW       = 5U;
    stcCanInit.pstcFilter             = astcFilter;
    stcCanInit.u16FilterSelect        = CAN_FILTER_SEL;
    stcCanInit.u8WorkMode             = CAN_WORK_MD_NORMAL;

    /* Enable peripheral clock of CAN. */
    FCG_Fcg1PeriphClockCmd(CAN1_PERIPH_CLK, ENABLE);
    (void)CAN_Init(CAN1_UNIT, &stcCanInit);
    /* Enable the interrupts, the status flags can be read. */
    CAN_IntCmd(CAN1_UNIT, CAN_INT_ALL, DISABLE);
    /* Enalbe the interrupts that needed. */
    CAN_IntCmd(CAN1_UNIT, CAN_INT_SEL, ENABLE);
	
	stc_irq_signin_config_t stcIrq;
	stcIrq.enIntSrc    = CAN1_INT_SRC;
	stcIrq.enIRQn      = CAN1_INT_IRQn;
	stcIrq.pfnCallback = &CAN1_IrqCallback;
 
    (void)INTC_IrqSignIn(&stcIrq);
    NVIC_ClearPendingIRQ(stcIrq.enIRQn);
	NVIC_SetPriority(stcIrq.enIRQn, CAN1_INT_PRIO);
    NVIC_EnableIRQ(stcIrq.enIRQn);

}

/****************************************************************************
*函数名	: can2_config_init
*介	 绍	：初始化CAN2
*形  参 : 无 
*返回值 : 无
******************************************************************************/
void can2_config_init(void)
{
    bms_can2_rx_quene_handle = xQueueCreate(8, sizeof(stc_can_rx_frame_t));
    if (bms_can2_rx_quene_handle == NULL) 
    {
        printf("can2 rx Create queue error.\r\n");
    }
    anti_can2_rx_quene_handle = xQueueCreate(8, sizeof(stc_can_rx_frame_t));
    if (anti_can2_rx_quene_handle == NULL) 
    {
        printf("can2 rx Create queue error.\r\n");
    }

	CLK_SetCANClockSrc(CAN2_CLK_UNIT, CAN2_CLK_SRC);

	GPIO_SetFunc(CAN2_TX_PORT, CAN2_TX_PIN, CAN2_TX_PIN_FUNC);
	GPIO_SetFunc(CAN2_RX_PORT, CAN2_RX_PIN, CAN2_RX_PIN_FUNC);
	
	stc_can_init_t stcCanInit;
    stc_can_filter_config_t astcFilter[CAN_FILTER_NUM] = {
        {CAN_FILTER1_ID, CAN_FILTER1_ID_MASK, CAN_FILTER1_ID_TYPE},
        {CAN_FILTER2_ID, CAN_FILTER2_ID_MASK, CAN_FILTER2_ID_TYPE},
        {CAN_FILTER3_ID, CAN_FILTER3_ID_MASK, CAN_FILTER3_ID_TYPE},
    };

	(void)CAN_StructInit(&stcCanInit);
    stcCanInit.stcBitCfg.u32Prescaler = 2U;
    stcCanInit.stcBitCfg.u32TimeSeg1  = 15U;
    stcCanInit.stcBitCfg.u32TimeSeg2  = 5U;
    stcCanInit.stcBitCfg.u32SJW       = 5U;
    stcCanInit.pstcFilter             = astcFilter;
    stcCanInit.u16FilterSelect        = CAN_FILTER_SEL;
    stcCanInit.u8WorkMode             = CAN_WORK_MD_NORMAL;

    /* Enable peripheral clock of CAN. */
    FCG_Fcg1PeriphClockCmd(CAN2_PERIPH_CLK, ENABLE);
    (void)CAN_Init(CAN2_UNIT, &stcCanInit);
    /* Enable the interrupts, the status flags can be read. */
    CAN_IntCmd(CAN2_UNIT, CAN_INT_ALL, DISABLE);
    /* Enalbe the interrupts that needed. */
    CAN_IntCmd(CAN2_UNIT, CAN_INT_SEL, ENABLE);
	
	stc_irq_signin_config_t stcIrq;
	stcIrq.enIntSrc    = CAN2_INT_SRC;
	stcIrq.enIRQn      = CAN2_INT_IRQn;
	stcIrq.pfnCallback = &CAN2_IrqCallback;
 
    (void)INTC_IrqSignIn(&stcIrq);
    NVIC_ClearPendingIRQ(stcIrq.enIRQn);
	NVIC_SetPriority(stcIrq.enIRQn, CAN2_INT_PRIO);
    NVIC_EnableIRQ(stcIrq.enIRQn);
}
#include "can_task.h"

/****************************************************************************
*函数名	: can_standard_send
*介	 绍	：发送CAN标准帧数据，数据长度0~8
*形  参 : CANx:使用的can id:发送帧ID  f_data:发送数据的指针 data_len：数据长度
*返回值 : 无
******************************************************************************/
int can_standard_send(CM_CAN_TypeDef *CANx, uint32_t id, void* f_data, uint8_t data_len)
{
    uint8_t i;
    stc_can_tx_frame_t stcTx1 = {0};
    uint8_t* p_data = f_data;
	if(data_len > 8)
	{
		data_len = 0;
	}
    stcTx1.u32Ctrl = 0x0UL;
    stcTx1.u32ID   = id;
    stcTx1.IDE     = 0;            /*0标准帧 1扩展帧*/
    stcTx1.DLC     = data_len;     /*帧数据长度*/
	
    for (i = 0U; i < data_len && data_len<9; i++)
    {
        stcTx1.au8Data[i] = p_data[i];
    }
    /* Transmit frames via PTB and STB */
//    (void)CAN_FillTxFrame(CANx, CAN_TX_BUF_PTB, &stcTx1);
//    CAN_StartTx(CANx, CAN_TX_REQ_PTB);
//    CAN_StartTx(CANx, CAN_TX_REQ_STB_ALL);
//	vTaskDelay(1);
	CanTxMsg_t tx_msg = {0};
	tx_msg.head = stcTx1;
	if(CANx ==CAN1_UNIT )
	{
		return WriteMutexCanTxBuf( &can1TxBuf,tx_msg );
	}
	else 
	{
		return WriteMutexCanTxBuf( &can2TxBuf,tx_msg );
	}
}
/****************************************************************************
*函数名	: can_extend_send
*介	 绍	：发送CAN扩展帧数据，数据长度0~8
*形  参 : CANx:使用的can id:发送帧ID  f_data:发送数据的指针 data_len：数据长度
*返回值 : 无
******************************************************************************/
int can_extend_send(CM_CAN_TypeDef *CANx, uint32_t id, void* f_data, uint8_t data_len)
{
    uint8_t i;
    stc_can_tx_frame_t stcTx = {0};
    uint8_t* p_data = f_data;
	if(data_len > 8)
	{
		data_len = 0;
	}
    stcTx.u32Ctrl = 0x0UL;
    stcTx.u32ID   = id;
    stcTx.IDE     = 1;            /*0标准帧 1扩展帧*/
    stcTx.DLC     = data_len;     /*帧数据长度*/
	
    for (i = 0U; i < data_len && data_len<9; i++)
    {
        stcTx.au8Data[i] = p_data[i];
    }
    /* Transmit frames via PTB and STB */
//    (void)CAN_FillTxFrame(CANx, CAN_TX_BUF_PTB, &stcTx);
//    CAN_StartTx(CANx, CAN_TX_REQ_PTB);
//    CAN_StartTx(CANx, CAN_TX_REQ_STB_ALL);
//	vTaskDelay(1);
//	return 0;
	CanTxMsg_t tx_msg = {0};
	tx_msg.head = stcTx;
	if(CANx ==CAN1_UNIT )
	{
		return WriteMutexCanTxBuf( &can1TxBuf,tx_msg );
	}
	else 
	{
		return WriteMutexCanTxBuf( &can2TxBuf,tx_msg );
	}
}
/****************************************************************************
*函数名	: CanWriteSDO
*介	 绍	：CANopen的SDO数据数据发送
*形  参 : 无
*返回值 : 无
******************************************************************************/
int CanWriteSDO(uint8_t f_id, uint16_t f_index, uint8_t f_sub_index, uint8_t f_data_type, void* f_data)
{
    uint8_t i;
    uint32_t size = 0;
    stc_can_tx_frame_t stcTx1={0};
    uint8_t* p_data = f_data;
	
	if(xTaskGetTickCount()-otaing_tick<7000)
	{
		return 0;
	}

    stcTx1.u32Ctrl = 0x0UL;
    stcTx1.u32ID   = 0x600 + f_id;;
    stcTx1.IDE     = 0;
    stcTx1.DLC     = 8;

    switch( f_data_type )
    {
        case int8:
        case uint8:
            size = 1;
            stcTx1.au8Data[0] = 0x2F;
            break;
        case int16:
        case uint16:
            size = 2;
            stcTx1.au8Data[0] = 0x2B;
            break;
        case int24:
        case uint24:
            size = 3;
            stcTx1.au8Data[0] = 0x27;
            break;
        case int32:
        case uint32:
            size = 4;
            stcTx1.au8Data[0] = 0x23;
            break;
        default:
            return -1;
    }

    stcTx1.au8Data[1] = f_index & 0x00FF;
    stcTx1.au8Data[2] = (f_index >> 8) & 0x00FF;
    stcTx1.au8Data[3] = f_sub_index;

    for (i = 0U; i < size; i++)
    {
        stcTx1.au8Data[4+i] = p_data[i];
    }
    /* Transmit frames via PTB and STB */
//    (void)CAN_FillTxFrame(CAN1_UNIT, CAN_TX_BUF_PTB, &stcTx1);
//    CAN_StartTx(CAN1_UNIT, CAN_TX_REQ_PTB);
//    CAN_StartTx(CAN1_UNIT, CAN_TX_REQ_STB_ALL);
//	vTaskDelay(1);
//	return 0;
	CanTxMsg_t tx_msg = {0};
	tx_msg.head = stcTx1;
	return WriteMutexCanTxBuf( &can1TxBuf,tx_msg );
}

/****************************************************************************
*函数名	: CanReadSDO
*介	 绍	：CANopen的SDO数据数据请求
*形  参 : 无
*返回值 : 无
******************************************************************************/
int CanReadSDO( uint8_t f_id, uint16_t f_index, uint8_t f_sub_index, uint8_t f_data_type, void* f_data )
{
	uint8_t i;
    uint32_t size = 0;
    stc_can_tx_frame_t stcTx1 = {0};
    uint8_t* p_data = f_data;
	if(xTaskGetTickCount()-otaing_tick<7000)
	{
		return 0;
	}
    stcTx1.u32Ctrl = 0x0UL;
    stcTx1.u32ID   = 0x600 + f_id;;
    stcTx1.IDE     = 0;
    stcTx1.DLC     = 8;


	stcTx1.au8Data[0] = 0x40;
    stcTx1.au8Data[1] = f_index & 0x00FF;
    stcTx1.au8Data[2] = (f_index >> 8) & 0x00FF;
    stcTx1.au8Data[3] = f_sub_index;

    /* Transmit frames via PTB and STB */
//    (void)CAN_FillTxFrame(CAN1_UNIT, CAN_TX_BUF_PTB, &stcTx1);
//    CAN_StartTx(CAN1_UNIT, CAN_TX_REQ_PTB);
//    CAN_StartTx(CAN1_UNIT, CAN_TX_REQ_STB_ALL);
//	vTaskDelay(1);
//	return 0;
	CanTxMsg_t tx_msg = {0};
	tx_msg.head = stcTx1;
	return WriteMutexCanTxBuf( &can1TxBuf,tx_msg );
}
/****************************************************************************
*函数名	: CanReadSDO
*介	 绍	：CANopen的SDO数据数据请求
*形  参 : 无
*返回值 : 无
******************************************************************************/
int Can2ReadSDO( uint8_t f_id, uint16_t f_index, uint8_t f_sub_index, uint8_t f_data_type, void* f_data )
{
	uint8_t i;
    uint32_t size = 0;
    stc_can_tx_frame_t stcTx = {0};
    uint8_t* p_data = f_data;

    stcTx.u32Ctrl = 0x0UL;
    stcTx.u32ID   = 0x600 + f_id;;
    stcTx.IDE     = 0;
    stcTx.DLC     = 8;


	stcTx.au8Data[0] = 0x40;
    stcTx.au8Data[1] = f_index & 0x00FF;
    stcTx.au8Data[2] = (f_index >> 8) & 0x00FF;
    stcTx.au8Data[3] = f_sub_index;

    /* Transmit frames via PTB and STB */
//    (void)CAN_FillTxFrame(CAN1_UNIT, CAN_TX_BUF_PTB, &stcTx1);
//    CAN_StartTx(CAN1_UNIT, CAN_TX_REQ_PTB);
//    CAN_StartTx(CAN1_UNIT, CAN_TX_REQ_STB_ALL);
//	vTaskDelay(1);
//	return 0;
	CanTxMsg_t tx_msg = {0};
	tx_msg.head = stcTx;
	return WriteMutexCanTxBuf( &can2TxBuf,tx_msg );
}
int Can2WriteSDO( uint8_t f_id, uint16_t f_index, uint8_t f_sub_index, uint8_t f_data_type, void* f_data )
{
	uint8_t i;
    uint32_t size = 0;
    stc_can_tx_frame_t stcTx = {0};
    uint8_t* p_data = f_data;
    
    if( f_data == NULL )
    {
        return -1;
    }
	
 	stcTx.u32Ctrl = 0x0UL;
    stcTx.u32ID   = 0x600 + f_id;;
    stcTx.IDE     = 0;
    stcTx.DLC     = 8;
	
    switch( f_data_type )
    {
        case int8:
        case uint8:
            size = 1;
            stcTx.au8Data[0] = 0x2F;
            break;
        case int16:
        case uint16:
            size = 2;
            stcTx.au8Data[0] = 0x2B;
            break;
        case int24:
        case uint24:
            size = 3;
            stcTx.au8Data[0] = 0x27;
            break;
        case int32:
        case uint32:
            size = 4;
            stcTx.au8Data[0] = 0x23;
            break;
        default:
            return -1;
    }

    stcTx.au8Data[1] = f_index & 0x00FF;
    stcTx.au8Data[2] = (f_index >> 8) & 0x00FF;
    stcTx.au8Data[3] = f_sub_index;   
 	
    for( int i=0; i<size; i++ )
    {
        stcTx.au8Data[4+i] = p_data[i];
    }
	
	CanTxMsg_t tx_msg = {0};
	tx_msg.head = stcTx;
    
    return WriteMutexCanTxBuf( &can2TxBuf,tx_msg );
}


#include "drv_h4aa30_usb_iap.h"
/****************************************************************************
*函数名	: can_data_handle
*介	 绍	：CAN数据处理
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void can_data_handle(void)
{
    uint8_t i;
    uint8_t j;
    uint8_t u8RxFrameNum = 0U;
    int32_t i32Ret;

    /* Get all received frames. */
    do {
        i32Ret = CAN_GetRxFrame(CAN1_UNIT, &m_astRxFrame[u8RxFrameNum]);
        if (i32Ret == LL_OK) {
            u8RxFrameNum++;
        }
    } while (i32Ret == LL_OK);

    /* Handle received frames. */
    for (i = 0U; i < u8RxFrameNum; i++) 
	{
		ObjdictDispatch(&m_astRxFrame[i]);
		// 驱控OTA
		if( m_astRxFrame[i].u32ID == 0x20  || m_astRxFrame[i].u32ID == 0x21  
			||m_astRxFrame[i].u32ID==0x581 || m_astRxFrame[i].u32ID==0x701  
			||m_astRxFrame[i].u32ID==0x585 || m_astRxFrame[i].u32ID==0x705)
		{	
			if(m_astRxFrame[i].au8Data[0]!=0x00&&m_astRxFrame[i].DLC==1)
			{
				return; //过滤驱动器的心跳包
			}
 			if(xTaskGetTickCount()-otaing_tick<7000 && xTaskGetTickCount()>20000)
			{	H4aa_OTA_RxCpltCallback(m_astRxFrame[i].u32ID,m_astRxFrame[i].au8Data);
				H25A15_Ota_Can_Read(m_astRxFrame[i].u32ID,m_astRxFrame[i].au8Data,m_astRxFrame[i].DLC);
			}
		}
 
		
		
//        CAN_DEBUG_PRINTF("CAN received %s frame with %s ID %.8x:\r\n", \
//                   m_s8FrameTypeStr[m_astRxFrame[i].FDF], \
//                   m_s8IDTypeStr[m_astRxFrame[i].IDE],    \
//                   (unsigned int)m_astRxFrame[i].u32ID);
//        for (j = 0; j < (uint8_t)m_astRxFrame[i].DLC; j++) 
//		{
//            CAN_DEBUG_PRINTF(" %.2x.", m_astRxFrame[i].au8Data[j]);
//            m_astRxFrame[i].au8Data[j] = 0U;
//        }
//        CAN_DEBUG_PRINTF("\r\n");
    }
}

 
/****************************************************************************
*函数名	: CAN1_IrqCallback
*介	 绍	：CAN1的中断回调函数
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void CAN1_IrqCallback(void)
{
    stc_can_error_info_t stcErr;

    (void)CAN_GetErrorInfo(CAN1_UNIT, &stcErr);
	if(stcErr.u8ErrorType)
	{
		CAN_DEBUG_PRINTF("---> CAN error type: %u, %s\r\n", stcErr.u8ErrorType, m_s8ErrorTypeStr[stcErr.u8ErrorType]);
		SET_REG8_BIT(CAN1_UNIT->CFG_STAT, CAN_CFG_STAT_RESET);
		CLR_REG8_BIT(CAN1_UNIT->CFG_STAT, CAN_CFG_STAT_RESET);
	}
	
	
    if (CAN_GetStatus(CAN1_UNIT, CAN_FLAG_BUS_OFF) == SET) {
         CAN_DEBUG_PRINTF("BUS OFF.\r\n");
    }

    if (CAN_GetStatus(CAN1_UNIT, CAN_FLAG_RX_BUF_OVF) == SET) {
         CAN_DEBUG_PRINTF("RX overflow.\r\n");
        CAN_ClearStatus(CAN1_UNIT, CAN_FLAG_RX_BUF_OVF);
    }

    if (CAN_GetStatus(CAN1_UNIT, CAN_FLAG_TX_BUF_FULL) == SET) {
         CAN_DEBUG_PRINTF("TX buffer full.\r\n");
    }

    if (CAN_GetStatus(CAN1_UNIT, CAN_FLAG_TX_ABORTED) == SET) {
         CAN_DEBUG_PRINTF("TX aborted.\r\n");
        CAN_ClearStatus(CAN1_UNIT, CAN_FLAG_TX_ABORTED);
    }

    if (CAN_GetStatus(CAN1_UNIT, CAN_FLAG_STB_TX) == SET) {
         CAN_DEBUG_PRINTF("STB transmitted.\r\n");
        CAN_ClearStatus(CAN1_UNIT, CAN_FLAG_STB_TX);
    }

    if (CAN_GetStatus(CAN1_UNIT, CAN_FLAG_PTB_TX) == SET) {
         CAN_DEBUG_PRINTF("PTB transmitted.\r\n");
        CAN_ClearStatus(CAN1_UNIT, CAN_FLAG_PTB_TX);
    }

    if (CAN_GetStatus(CAN1_UNIT, CAN_FLAG_RX) == SET) {
        /* Received frame can be read here. */
//         CAN_DEBUG_PRINTF("Received a frame.\r\n");
        CAN_ClearStatus(CAN1_UNIT, CAN_FLAG_RX);
    }

    if (CAN_GetStatus(CAN1_UNIT, CAN_FLAG_RX_BUF_WARN) == SET) {
        /* Received frames can be read here. */
         CAN_DEBUG_PRINTF("RX buffer warning.\r\n");
        CAN_ClearStatus(CAN1_UNIT, CAN_FLAG_RX_BUF_WARN);
    }

    if (CAN_GetStatus(CAN1_UNIT, CAN_FLAG_RX_BUF_FULL) == SET) {
        /* Received frames can be read here. */
         CAN_DEBUG_PRINTF("RX buffer full.\r\n");
        CAN_ClearStatus(CAN1_UNIT, CAN_FLAG_RX_BUF_FULL);
    }

    if (CAN_GetStatus(CAN1_UNIT, CAN_FLAG_RX_OVERRUN) == SET) {
         CAN_DEBUG_PRINTF("RX buffer overrun.\r\n");
        CAN_ClearStatus(CAN1_UNIT, CAN_FLAG_RX_OVERRUN);
    }

    if (CAN_GetStatus(CAN1_UNIT, CAN_FLAG_TEC_REC_WARN) == SET) {
         CAN_DEBUG_PRINTF("TEC or REC reached warning limit.\r\n");
        CAN_ClearStatus(CAN1_UNIT, CAN_FLAG_TEC_REC_WARN);
    }
	if (CAN_GetStatus(CAN1_UNIT, CAN_FLAG_TEC_REC_WARN) == SET) {
         CAN_DEBUG_PRINTF("TEC or REC reached warning limit.\r\n");
        CAN_ClearStatus(CAN1_UNIT, CAN_FLAG_TEC_REC_WARN);
    }
	
	can_data_handle();
}

/****************************************************************************
*函数名	: CAN2_IrqCallback
*介	 绍	：CAN2的中断回调函数
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void CAN2_IrqCallback(void)
{
    stc_can_error_info_t stcErr;

    (void)CAN_GetErrorInfo(CAN2_UNIT, &stcErr);
 	if(stcErr.u8ErrorType)
	{
		CAN_DEBUG_PRINTF("---> CAN2 error type: %u, %s\r\n", stcErr.u8ErrorType, m_s8ErrorTypeStr[stcErr.u8ErrorType]);
		SET_REG8_BIT(CAN2_UNIT->CFG_STAT, CAN_CFG_STAT_RESET);
		CLR_REG8_BIT(CAN2_UNIT->CFG_STAT, CAN_CFG_STAT_RESET);
	}
    if (CAN_GetStatus(CAN2_UNIT, CAN_FLAG_BUS_OFF) == SET) {
        CAN_DEBUG_PRINTF("BUS OFF.\r\n");
    }

    if (CAN_GetStatus(CAN2_UNIT, CAN_FLAG_RX_BUF_OVF) == SET) {
        CAN_DEBUG_PRINTF("RX overflow.\r\n");
        CAN_ClearStatus(CAN2_UNIT, CAN_FLAG_RX_BUF_OVF);
    }

    if (CAN_GetStatus(CAN2_UNIT, CAN_FLAG_TX_BUF_FULL) == SET) {
        CAN_DEBUG_PRINTF("TX buffer full.\r\n");
    }

    if (CAN_GetStatus(CAN2_UNIT, CAN_FLAG_TX_ABORTED) == SET) {
        CAN_DEBUG_PRINTF("TX aborted.\r\n");
        CAN_ClearStatus(CAN2_UNIT, CAN_FLAG_TX_ABORTED);
    }

    if (CAN_GetStatus(CAN2_UNIT, CAN_FLAG_STB_TX) == SET) {
        CAN_DEBUG_PRINTF("STB transmitted.\r\n");
        CAN_ClearStatus(CAN2_UNIT, CAN_FLAG_STB_TX);
    }

    if (CAN_GetStatus(CAN2_UNIT, CAN_FLAG_PTB_TX) == SET) {
        CAN_DEBUG_PRINTF("PTB transmitted.\r\n");
        CAN_ClearStatus(CAN2_UNIT, CAN_FLAG_PTB_TX);
    }

    if (CAN_GetStatus(CAN2_UNIT, CAN_FLAG_RX) == SET) {
        /* Received frame can be read here. */
        CAN_DEBUG_PRINTF("Received a frame.\r\n");
        CAN_ClearStatus(CAN2_UNIT, CAN_FLAG_RX);
    }

    if (CAN_GetStatus(CAN2_UNIT, CAN_FLAG_RX_BUF_WARN) == SET) {
        /* Received frames can be read here. */
        CAN_DEBUG_PRINTF("RX buffer warning.\r\n");
        CAN_ClearStatus(CAN2_UNIT, CAN_FLAG_RX_BUF_WARN);
    }

    if (CAN_GetStatus(CAN2_UNIT, CAN_FLAG_RX_BUF_FULL) == SET) {
        /* Received frames can be read here. */
        CAN_DEBUG_PRINTF("RX buffer full.\r\n");
        CAN_ClearStatus(CAN2_UNIT, CAN_FLAG_RX_BUF_FULL);
    }

    if (CAN_GetStatus(CAN2_UNIT, CAN_FLAG_RX_OVERRUN) == SET) {
        CAN_DEBUG_PRINTF("RX buffer overrun.\r\n");
        CAN_ClearStatus(CAN2_UNIT, CAN_FLAG_RX_OVERRUN);
    }

    if (CAN_GetStatus(CAN2_UNIT, CAN_FLAG_TEC_REC_WARN) == SET) {
        CAN_DEBUG_PRINTF("TEC or REC reached warning limit.\r\n");
        CAN_ClearStatus(CAN2_UNIT, CAN_FLAG_TEC_REC_WARN);
    }
    int32_t i32Ret;
    stc_can_rx_frame_t rx_frame;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    do
    {
        i32Ret = CAN_GetRxFrame(CAN2_UNIT, &rx_frame);
        if (i32Ret == LL_OK)
        {
            if (rx_frame.IDE != 0)
            {
                if (anti_can2_rx_quene_handle != NULL)
                {
                    xQueueSendFromISR(anti_can2_rx_quene_handle, &rx_frame, &xHigherPriorityTaskWoken);
                }
            }
            else
            {
                if (bms_can2_rx_quene_handle != NULL)
                {
                    xQueueSendFromISR(bms_can2_rx_quene_handle, &rx_frame, &xHigherPriorityTaskWoken);
                }                
            }
        }
    } while (i32Ret == LL_OK);
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


