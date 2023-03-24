/**
  ******************************************************************************
  * �ļ�����: bsp_debug_usart.c
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2015-10-04
  * ��    ��: ���ص��Դ��ڵײ���������Ĭ��ʹ��USART1
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  *
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "bsp_CAN.h"
#include "canopen_od.h"
#include "bsp_usart1.h"
#include "mc_type.h"
#include "bsp_ota.h"
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/

CAN_HandleTypeDef hcan2;
CanTxMsgTypeDef TxMessage;
CanRxMsgTypeDef RxMessage;
extern void Error_Handler(void);
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: CAN�����ʼ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
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

    hcan2.Init.Prescaler = 6;          // BTR-BRP ??????  ???????????? 42/(1+6+7)/3=1Mbps
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
  * ��������: CAN���ų�ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
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
        HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
        /* USER CODE END CAN2_MspInit 1 */
    }
}


/**
  * ��������: CAN�������ŷ���ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
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
uint8_t damping_mode_flag = 0;//�ϵ�Ĭ��ֵΪ0���յ���λ��ָ��֮��ֵΪ1������ģʽ���ܿ���������ڹػ�״̬���ƶ�����
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
{
    {
        if(CAN2->RF0R & (1<<4))
        {
            CAN2->RF0R &= (0<<4);
        }
        if(CANopen_Drive.CanRx_Buffer->IDE == BIT_INVALID)
        {
            CanLoadRate.FrameSize = CanLoadRate.FrameSize + (47 + CANopen_Drive.CanRx_Buffer->DLC * 8);
            MessageType_Check(&CANopen_Drive,CANopen_Drive.CanRx_Buffer->StdId,CANopen_Drive.CanRx_Buffer->RTR);
            damping_mode_flag = 1;
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
uint8_t CAN2_Send_Msg(uint32_t stdId,uint32_t Ide,uint32_t Rtr,uint8_t* msg,uint8_t len)
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
    for(i=0; HAL_CAN_Transmit(&hcan2,0x64)!=HAL_OK && i<3; i++) {}
    CanLoadRate.FrameSize = CanLoadRate.FrameSize + (47 + hcan2.pTxMsg->DLC * 8);
    return i != 3? 0:1;
}


void CAN2_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan2);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
