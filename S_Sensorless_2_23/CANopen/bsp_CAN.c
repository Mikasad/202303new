/**
  ******************************************************************************
  * 文件名程: bsp_debug_usart.c
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2015-10-04
  * 功    能: 板载调试串口底层驱动程序：默认使用USART1
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F1Pro使用。
  *
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_CAN.h"
#include "canopen_od.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/

CAN_HandleTypeDef hcan2;
CanTxMsgTypeDef TxMessage;
CanRxMsgTypeDef RxMessage;
pFunction Jump_To_Application;
extern void Error_Handler(void);
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: CAN外设初始
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void MX_CAN2_Init(void)
{

    /* USER CODE BEGIN CAN2_Init 0 */

    /* USER CODE END CAN2_Init 0 */

    /* USER CODE BEGIN CAN2_Init 1 */

    /* USER CODE END CAN2_Init 1 */
    /* USER CODE BEGIN CAN2_Init 0 */
    CAN_FilterConfTypeDef  sFilterConfig;
    /* USER CODE END CAN2_Init 0 */

    /* USER CODE BEGIN CAN2_Init 1 */

    /* USER CODE END CAN2_Init 1 */
    hcan2.Instance = CAN2;
    hcan2.pTxMsg = &TxMessage;
    hcan2.pRxMsg = &RxMessage;

    hcan2.Init.Prescaler = 6;          // BTR-BRP  42/(1+6+7)/6=500kbps       PRE:7       STW:1 BS1:10  bs2:1  
    hcan2.Init.Mode = CAN_MODE_NORMAL;
    hcan2.Init.SJW = CAN_SJW_1TQ;      // BTR-SJW ???????? 1?????
    hcan2.Init.BS1 = CAN_BS1_6TQ;      // BTR-TS1 ???1 ???6?????
    hcan2.Init.BS2 = CAN_BS2_7TQ;      // BTR-TS1 ???2 ???7?????
    hcan2.Init.TTCM = DISABLE;         // MCR-TTCM  ????????????
    hcan2.Init.ABOM = ENABLE;          // MCR-ABOM  ??????
    hcan2.Init.AWUM = ENABLE;          // MCR-AWUM  ????????
    hcan2.Init.NART = DISABLE;         // MCR-NART  ????????	  DISABLE-????
    hcan2.Init.RFLM = DISABLE;         // MCR-RFLM  ??FIFO ????  DISABLE-?????????????
    hcan2.Init.TXFP = DISABLE;         // MCR-TXFP  ??FIFO??? DISABLE-???????????
    if (HAL_CAN_Init(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN2_Init 2 */
    /*CAN??????*/
    sFilterConfig.FilterNumber = 14;                    //????14
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  //???????????
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //????????32??
    /* ????????????????????????,??ID?????????,???,???FIFO0? */

    sFilterConfig.FilterIdHigh         = 0X0000;//(((uint32_t)0x1314<<3)&0xFFFF0000)>>16;				//????ID??
    sFilterConfig.FilterIdLow          = 0X0000;//(((uint32_t)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //????ID??
    sFilterConfig.FilterMaskIdHigh     = 0X0000;//0xFFFF;			//????16???????
    sFilterConfig.FilterMaskIdLow      = 0X0000;//0xFFFF;			//????16???????
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;           //???????FIFO 0
    sFilterConfig.FilterActivation = ENABLE;          //?????
    sFilterConfig.BankNumber = 14;
    HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
    /* USER CODE BEGIN CAN2_Init 2 */

    /* USER CODE END CAN2_Init 2 */

}

/**
  * 函数功能: CAN引脚初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hcan->Instance==CAN2)
    {
        /* USER CODE BEGIN CAN2_MspInit 0 */

        /* USER CODE END CAN2_MspInit 0 */
        /* Peripheral clock enable */

        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_CAN2_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**CAN2 GPIO Configuration
        PB13     ------> CAN2_TX
        PB5     ------> CAN2_RX
        */
        GPIO_InitStruct.Pin = CAN_TX_Pin|CAN_RX_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USER CODE BEGIN CAN2_MspInit 1 */
        CANopen_Drive.CanRx_Buffer =  hcan2.pRxMsg;
        CANopen_Drive.CanTx_Buffer = hcan2.pTxMsg;
       
        /* USER CODE END CAN2_MspInit 1 */
    }
}


/**
  * 函数功能: CAN外设引脚反初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{

    if(hcan->Instance==CAN2)
    {
        /* USER CODE BEGIN CAN2_MspDeInit 0 */

        /* USER CODE END CAN2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_CAN2_CLK_DISABLE();
        __HAL_RCC_CAN1_CLK_DISABLE();

        /**CAN2 GPIO Configuration
        PB13     ------> CAN2_TX
        PB5     ------> CAN2_RX
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_5);

        /* CAN2 interrupt DeInit */
        HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
        /* USER CODE BEGIN CAN2_MspDeInit 1 */

        /* USER CODE END CAN2_MspDeInit 1 */
    }

}
void SoftReset(void)
{
	__set_FAULTMASK(1);    
	NVIC_SystemReset();
}
uint32_t JumpAddress;
uint8_t Jump_to_APP(uint32_t APP_addr)
{
	if(((*(__IO uint32_t*)APP_addr) & 0x2FFE0000 ) == 0x20000000)
	{
		JumpAddress = *(__IO uint32_t*) (APP_addr + 4);
		Jump_To_Application = (pFunction) JumpAddress;
		__set_PRIMASK(1);
		HAL_NVIC_DisableIRQ(SysTick_IRQn);
//		__disable_irq();
		__set_MSP(*(__IO uint32_t*) APP_addr);
		Jump_To_Application();
	}
	else
		return 0;
	return 0;
}
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
{
		uint8_t return_data[8];
    {
        if(CAN2->RF0R & (1<<4))
        {
            CAN2->RF0R &= (0<<4);
        }
        if(CANopen_Drive.CanRx_Buffer->IDE == BIT_INVALID)
        {
						/*bootloader相关*/
						if(RxMessage.StdId == 0x10)					
						{
							if(RxMessage.Data[0]==0xe0)			//软重启
							{
								SoftReset();
							}
							else if(RxMessage.Data[0]==0xe1&&RxMessage.Data[1]==0x01)		//程序跳转重启
							{
								return_data[0] = 0xe2;
								return_data[1] = 0x00;
								CAN1_Send_Msg(0x20,0,0,return_data,2);
								Jump_to_APP(0x08000000);
							}
						}
						/*bootloader相关*/
            CanLoadRate.FrameSize = CanLoadRate.FrameSize + (47 + CANopen_Drive.CanRx_Buffer->DLC * 8);
            MessageType_Check(&CANopen_Drive,CANopen_Drive.CanRx_Buffer->StdId,CANopen_Drive.CanRx_Buffer->RTR);
        }
        else
        {}
        HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
    }
}
/**
  * @brief CAN1 ????
  * @param None
  * @retval None
  */
uint8_t CAN1_Send_Msg(uint32_t stdId,uint32_t Ide,uint32_t Rtr,uint8_t* msg,uint8_t len)
{
    uint16_t i=0;
    hcan2.pTxMsg->StdId=stdId;        //?????0001_0010 ??????????????????
    hcan2.pTxMsg->ExtId=0xFFF;        //?????(29?)
    hcan2.pTxMsg->IDE=Ide;    //?????(????)
    hcan2.pTxMsg->RTR=Rtr;  //???(????)
    hcan2.pTxMsg->DLC=len;           //????
    for(i=0; i<len; i++)
    {
        hcan2.pTxMsg->Data[i]=msg[i];
    }
    for(i=0; HAL_CAN_Transmit(&hcan2,10)!=HAL_OK && i<3; i++) {}
    if(i>=3)
    {
    }
    CanLoadRate.FrameSize = CanLoadRate.FrameSize + (47 + hcan2.pTxMsg->DLC * 8);
    return i != 3? 0:1;
}


//extern CAN_HandleTypeDef hcan2;
void CAN2_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan2);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
