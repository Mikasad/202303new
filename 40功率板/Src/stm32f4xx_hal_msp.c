/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */
#include "canopen_od.h"
/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan2;
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/**
* Initializes the Global MSP.
*/
void HAL_MspInit(void)
{
    /* USER CODE BEGIN MspInit 0 */

    /* USER CODE END MspInit 0 */

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);

    /* System interrupt init*/

    /* USER CODE BEGIN MspInit 1 */

    /* USER CODE END MspInit 1 */
}

/**
* @brief ADC MSP Initialization
* This function configures the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hadc->Instance==ADC1)
    {
        /* USER CODE BEGIN ADC1_MspInit 0 */

        /* USER CODE END ADC1_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_ADC1_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**ADC1 GPIO Configuration
        PC0     ------> ADC1_IN10
        PC1     ------> ADC1_IN11
        PC2     ------> ADC1_IN12
        PC3     ------> ADC1_IN13
        PA0-WKUP     ------> ADC1_IN0
        PA1     ------> ADC1_IN1
        PA2     ------> ADC1_IN2
        PA3     ------> ADC1_IN3
        PA4     ------> ADC1_IN4
        PA6     ------> ADC1_IN6
        PC4     ------> ADC1_IN14
        PC5     ------> ADC1_IN15
        */
        GPIO_InitStruct.Pin = Isense1_Pin|Isense2_Pin|Isense3_Pin
                              |BUS_VOL_DECT_Pin|MotorTemp1_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = Isense5_Pin|Isense6_Pin
                              |MotorTemp2_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* ADC1 DMA Init */
        /* ADC1 Init */
        hdma_adc1.Instance = DMA2_Stream0;
        hdma_adc1.Init.Channel = DMA_CHANNEL_0;
        hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment =DMA_PDATAALIGN_WORD;// DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;//DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode = DMA_CIRCULAR;
        hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
        hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);

        /* USER CODE BEGIN ADC1_MspInit 1 */

        /* USER CODE END ADC1_MspInit 1 */
    }
    if(hadc->Instance==ADC2)
    {
        /* 外设时钟使能 */
        __HAL_RCC_ADC2_CLK_ENABLE();
			  /* AD转换通道引脚时钟使能 */
        __HAL_RCC_GPIOA_CLK_ENABLE();
			
    /* AD转换通道引脚初始化 */
    GPIO_InitStruct.Pin = Current_2_Pin; //电机6
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Current_1_Pin; //电机5
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
		}
    if(hadc->Instance==ADC3)
    {
        /* 外设时钟使能 */
//        __HAL_RCC_ADC3_CLK_ENABLE();
//			  /* AD转换通道引脚时钟使能 */
//        __HAL_RCC_GPIOA_CLK_ENABLE();
//			
//    /* AD转换通道引脚初始化 */
//    GPIO_InitStruct.Pin = Current_1_Pin; //电机5
//    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		}
}

/**
* @brief ADC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance==ADC1)
    {
        /* USER CODE BEGIN ADC1_MspDeInit 0 */

        /* USER CODE END ADC1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_ADC1_CLK_DISABLE();

        /**ADC1 GPIO Configuration
        PC0     ------> ADC1_IN10
        PC1     ------> ADC1_IN11
        PC2     ------> ADC1_IN12
        PC3     ------> ADC1_IN13
        PA0-WKUP     ------> ADC1_IN0
        PA1     ------> ADC1_IN1
        PA2     ------> ADC1_IN2
        PA3     ------> ADC1_IN3
        PA4     ------> ADC1_IN4
        PA6     ------> ADC1_IN6
        PC4     ------> ADC1_IN14
        PC5     ------> ADC1_IN15
        */
        HAL_GPIO_DeInit(GPIOC, Isense1_Pin|Isense2_Pin|Isense3_Pin
                        |BUS_VOL_DECT_Pin|MotorTemp1_Pin);

        HAL_GPIO_DeInit(GPIOA, Isense5_Pin|Isense6_Pin
                        |MotorTemp2_Pin);

        /* ADC1 DMA DeInit */
        HAL_DMA_DeInit(hadc->DMA_Handle);
        /* USER CODE BEGIN ADC1_MspDeInit 1 */

        /* USER CODE END ADC1_MspDeInit 1 */
    }
		if(hadc->Instance==ADC2)
		{
			/* 禁用ADC外设时钟 */
        __HAL_RCC_ADC2_CLK_DISABLE();
		
			/* AD转换通道引脚反初始化 */
			HAL_GPIO_DeInit(GPIOA, Current_1_Pin);
			HAL_GPIO_DeInit(GPIOA, Current_2_Pin);

		//  HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
		}
		if(hadc->Instance==ADC3)
		{
			/* 禁用ADC外设时钟 */
//        __HAL_RCC_ADC3_CLK_DISABLE();
//		
//			/* AD转换通道引脚反初始化 */
//			HAL_GPIO_DeInit(GPIOA, Current_2_Pin);

		//  HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
		}
}

/**
* @brief CAN MSP Initialization
* This function configures the hardware resources used in this example
* @param hcan: CAN handle pointer
* @retval None
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
* @brief CAN MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hcan: CAN handle pointer
* @retval None
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
        HAL_GPIO_DeInit(GPIOB, CAN_TX_Pin|CAN_RX_Pin);

        /* CAN2 interrupt DeInit */
        HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
        /* USER CODE BEGIN CAN2_MspDeInit 1 */

        /* USER CODE END CAN2_MspDeInit 1 */
    }

}

/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(htim_base->Instance==TIM1)
    {
        /* USER CODE BEGIN TIM1_MspInit 0 */

        /* USER CODE END TIM1_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM1_CLK_ENABLE();

        __HAL_RCC_GPIOE_CLK_ENABLE();
        /**TIM1 GPIO Configuration
        PE15     ------> TIM1_BKIN
        */
        GPIO_InitStruct.Pin = GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* TIM1 interrupt Init */
        HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
        HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
        /* USER CODE BEGIN TIM1_MspInit 1 */

        /* USER CODE END TIM1_MspInit 1 */
    }
    else if(htim_base->Instance==TIM2)
    {
        /* USER CODE BEGIN TIM2_MspInit 0 */

        /* USER CODE END TIM2_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();
        /* USER CODE BEGIN TIM2_MspInit 1 */

        /* USER CODE END TIM2_MspInit 1 */
    }
    else if(htim_base->Instance==TIM3)
    {
        /* USER CODE BEGIN TIM3_MspInit 0 */

        /* USER CODE END TIM3_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM3_CLK_ENABLE();
        /* USER CODE BEGIN TIM3_MspInit 1 */

        /* USER CODE END TIM3_MspInit 1 */
    }
    else if(htim_base->Instance==TIM4)
    {
        /* USER CODE BEGIN TIM4_MspInit 0 */

        /* USER CODE END TIM4_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM4_CLK_ENABLE();
        /* USER CODE BEGIN TIM4_MspInit 1 */

        /* USER CODE END TIM4_MspInit 1 */
    }
    else if(htim_base->Instance==TIM6)
    {
        /* USER CODE BEGIN TIM6_MspInit 0 */

        /* USER CODE END TIM6_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM6_CLK_ENABLE();
        /* USER CODE BEGIN TIM6_MspInit 1 */

        /* USER CODE END TIM6_MspInit 1 */
    }
    else if(htim_base->Instance==TIM7)
    {
        /* USER CODE BEGIN TIM7_MspInit 0 */

        /* USER CODE END TIM7_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM7_CLK_ENABLE();
        /* TIM7 interrupt Init */
        HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM7_IRQn);
        /* USER CODE BEGIN TIM7_MspInit 1 */

        /* USER CODE END TIM7_MspInit 1 */
    }
    else if(htim_base->Instance==TIM8)
    {
        /* USER CODE BEGIN TIM8_MspInit 0 */

        /* USER CODE END TIM8_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM8_CLK_ENABLE();
        /* USER CODE BEGIN TIM8_MspInit 1 */

        /* USER CODE END TIM8_MspInit 1 */
    }
    else if(htim_base->Instance==TIM10)
    {
        /* USER CODE BEGIN TIM10_MspInit 0 */

        /* USER CODE END TIM10_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM10_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM10 GPIO Configuration
        PB8     ------> TIM10_CH1
        */
        GPIO_InitStruct.Pin = PWM1_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
        HAL_GPIO_Init(PWM1_GPIO_Port, &GPIO_InitStruct);

        /* TIM10 interrupt Init */
        HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
        /* USER CODE BEGIN TIM10_MspInit 1 */

        /* USER CODE END TIM10_MspInit 1 */
    }
    else if(htim_base->Instance==TIM11)
    {
        /* USER CODE BEGIN TIM11_MspInit 0 */

        /* USER CODE END TIM11_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM11_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM11 GPIO Configuration
        PB9     ------> TIM11_CH1
        */
        GPIO_InitStruct.Pin = PWM2_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
        HAL_GPIO_Init(PWM2_GPIO_Port, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM11_MspInit 1 */

        /* USER CODE END TIM11_MspInit 1 */
    }

}

/**
* @brief TIM_IC MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_ic: TIM_IC handle pointer
* @retval None
*/
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_ic)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(htim_ic->Instance==TIM9)
    {
        /* USER CODE BEGIN TIM9_MspInit 0 */

        /* USER CODE END TIM9_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM9_CLK_ENABLE();

        __HAL_RCC_GPIOE_CLK_ENABLE();
        /**TIM9 GPIO Configuration
        PE5     ------> TIM9_CH1
        PE6     ------> TIM9_CH2
        */
        GPIO_InitStruct.Pin = SB2_FG_Pin|SideBrush_FG_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* TIM9 interrupt Init */
        HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
        /* USER CODE BEGIN TIM9_MspInit 1 */

        /* USER CODE END TIM9_MspInit 1 */
    }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(htim->Instance==TIM1)
    {
        /* USER CODE BEGIN TIM1_MspPostInit 0 */

        /* USER CODE END TIM1_MspPostInit 0 */
        __HAL_RCC_GPIOE_CLK_ENABLE();
        /**TIM1 GPIO Configuration
        PE8     ------> TIM1_CH1N
        PE9     ------> TIM1_CH1
        PE10     ------> TIM1_CH2N
        PE11     ------> TIM1_CH2
        PE12     ------> TIM1_CH3N
        PE13     ------> TIM1_CH3
        PE14     ------> TIM1_CH4
        */
        GPIO_InitStruct.Pin = INLU1_Pin|INHU1_Pin|INLV1_Pin|INHV1_Pin
                              |INLW1_Pin|INHW1_Pin|FAN_PWM_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = FAN_PWM_Pin;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
        /* USER CODE BEGIN TIM1_MspPostInit 1 */

        /* USER CODE END TIM1_MspPostInit 1 */
    }
    else if(htim->Instance==TIM2)
    {
        /* USER CODE BEGIN TIM2_MspPostInit 0 */

        /* USER CODE END TIM2_MspPostInit 0 */

        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM2 GPIO Configuration
        PB10     ------> TIM2_CH3
        PB11     ------> TIM2_CH4
        PA15     ------> TIM2_CH1
        PB3     ------> TIM2_CH2
        */
        GPIO_InitStruct.Pin = PWM_AH2_Pin|PWM_BH2_Pin|PWM_BH1_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = PWM_AH1_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(PWM_AH1_GPIO_Port, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM2_MspPostInit 1 */

        /* USER CODE END TIM2_MspPostInit 1 */
    }
    else if(htim->Instance==TIM3)
    {
        /* USER CODE BEGIN TIM3_MspPostInit 0 */

        /* USER CODE END TIM3_MspPostInit 0 */

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM3 GPIO Configuration
        PA7     ------> TIM3_CH2
        PB0     ------> TIM3_CH3
        PB1     ------> TIM3_CH4
        PB4     ------> TIM3_CH1
        */
        GPIO_InitStruct.Pin = PWM4_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(PWM4_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = PWM5_Pin|PWM6_Pin|PWM3_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM3_MspPostInit 1 */

        /* USER CODE END TIM3_MspPostInit 1 */
    }
    else if(htim->Instance==TIM4)
    {
        /* USER CODE BEGIN TIM4_MspPostInit 0 */

        /* USER CODE END TIM4_MspPostInit 0 */

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**TIM4 GPIO Configuration
        PD12     ------> TIM4_CH1
        PD13     ------> TIM4_CH2
        PD14     ------> TIM4_CH3
        PD15     ------> TIM4_CH4
        */
        GPIO_InitStruct.Pin = PWM_AH3_Pin|PWM_BH3_Pin|SB2_PWM_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM4_MspPostInit 1 */

        /* USER CODE END TIM4_MspPostInit 1 */
    }
    else if(htim->Instance==TIM8)
    {
        /* USER CODE BEGIN TIM8_MspPostInit 0 */

        /* USER CODE END TIM8_MspPostInit 0 */

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**TIM8 GPIO Configuration
        PA5     ------> TIM8_CH1N
        PB14     ------> TIM8_CH2N
        PB15     ------> TIM8_CH3N
        PC6     ------> TIM8_CH1
        PC7     ------> TIM8_CH2
        PC8     ------> TIM8_CH3
        PC9     ------> TIM8_CH4
        */
        GPIO_InitStruct.Pin = INLU2_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(INLU2_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = INLV2_Pin|INLW2_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = INHU2_Pin|INHV2_Pin|INHW2_Pin|SideBrush_PWM_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM8_MspPostInit 1 */

        /* USER CODE END TIM8_MspPostInit 1 */
    }
    else if(htim->Instance==TIM10)
    {

			  __HAL_RCC_TIM10_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM10 GPIO Configuration
        PB8     ------> TIM10_CH1
        */
        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    }
    else if(htim->Instance==TIM11)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM11 GPIO Configuration
        PB9     ------> TIM11_CH1
        */
        GPIO_InitStruct.Pin = PWM2_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
        HAL_GPIO_Init(PWM2_GPIO_Port, &GPIO_InitStruct);
    }

}
/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
    if(htim_base->Instance==TIM1)
    {
        /* USER CODE BEGIN TIM1_MspDeInit 0 */

        /* USER CODE END TIM1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM1_CLK_DISABLE();

        /**TIM1 GPIO Configuration
        PE8     ------> TIM1_CH1N
        PE9     ------> TIM1_CH1
        PE10     ------> TIM1_CH2N
        PE11     ------> TIM1_CH2
        PE12     ------> TIM1_CH3N
        PE13     ------> TIM1_CH3
        PE14     ------> TIM1_CH4
        PE15     ------> TIM1_BKIN
        */
        HAL_GPIO_DeInit(GPIOE, INLU1_Pin|INHU1_Pin|INLV1_Pin|INHV1_Pin
                        |INLW1_Pin|INHW1_Pin|FAN_PWM_Pin|GPIO_PIN_15);

        /* TIM1 interrupt DeInit */
        /* USER CODE BEGIN TIM1:TIM1_BRK_TIM9_IRQn disable */
        /**
        * Uncomment the line below to disable the "TIM1_BRK_TIM9_IRQn" interrupt
        * Be aware, disabling shared interrupt may affect other IPs
        */
        /* HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn); */
        /* USER CODE END TIM1:TIM1_BRK_TIM9_IRQn disable */

        /* USER CODE BEGIN TIM1:TIM1_UP_TIM10_IRQn disable */
        /**
        * Uncomment the line below to disable the "TIM1_UP_TIM10_IRQn" interrupt
        * Be aware, disabling shared interrupt may affect other IPs
        */
        /* HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn); */
        /* USER CODE END TIM1:TIM1_UP_TIM10_IRQn disable */

        /* USER CODE BEGIN TIM1_MspDeInit 1 */

        /* USER CODE END TIM1_MspDeInit 1 */
    }
    else if(htim_base->Instance==TIM2)
    {
        /* USER CODE BEGIN TIM2_MspDeInit 0 */

        /* USER CODE END TIM2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM2_CLK_DISABLE();
        /* USER CODE BEGIN TIM2_MspDeInit 1 */

        /* USER CODE END TIM2_MspDeInit 1 */
    }
    else if(htim_base->Instance==TIM3)
    {
        /* USER CODE BEGIN TIM3_MspDeInit 0 */

        /* USER CODE END TIM3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM3_CLK_DISABLE();
        /* USER CODE BEGIN TIM3_MspDeInit 1 */

        /* USER CODE END TIM3_MspDeInit 1 */
    }
    else if(htim_base->Instance==TIM4)
    {
        /* USER CODE BEGIN TIM4_MspDeInit 0 */

        /* USER CODE END TIM4_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM4_CLK_DISABLE();
        /* USER CODE BEGIN TIM4_MspDeInit 1 */

        /* USER CODE END TIM4_MspDeInit 1 */
    }
    else if(htim_base->Instance==TIM6)
    {
        /* USER CODE BEGIN TIM6_MspDeInit 0 */

        /* USER CODE END TIM6_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM6_CLK_DISABLE();
        /* USER CODE BEGIN TIM6_MspDeInit 1 */

        /* USER CODE END TIM6_MspDeInit 1 */
    }
    else if(htim_base->Instance==TIM7)
    {
        /* USER CODE BEGIN TIM7_MspDeInit 0 */

        /* USER CODE END TIM7_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM7_CLK_DISABLE();

        /* TIM7 interrupt DeInit */
        HAL_NVIC_DisableIRQ(TIM7_IRQn);
        /* USER CODE BEGIN TIM7_MspDeInit 1 */

        /* USER CODE END TIM7_MspDeInit 1 */
    }
    else if(htim_base->Instance==TIM8)
    {
        /* USER CODE BEGIN TIM8_MspDeInit 0 */

        /* USER CODE END TIM8_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM8_CLK_DISABLE();
        /* USER CODE BEGIN TIM8_MspDeInit 1 */

        /* USER CODE END TIM8_MspDeInit 1 */
    }
    else if(htim_base->Instance==TIM10)
    {
        /* USER CODE BEGIN TIM10_MspDeInit 0 */

        /* USER CODE END TIM10_MspDeInit 0 */
        /* Peripheral clock disable */
//    __HAL_RCC_TIM10_CLK_DISABLE();

        /**TIM10 GPIO Configuration
        PB8     ------> TIM10_CH1
        */
//    HAL_GPIO_DeInit(PWM1_GPIO_Port, PWM1_Pin);

        /* TIM10 interrupt DeInit */
        /* USER CODE BEGIN TIM10:TIM1_UP_TIM10_IRQn disable */
        /**
        * Uncomment the line below to disable the "TIM1_UP_TIM10_IRQn" interrupt
        * Be aware, disabling shared interrupt may affect other IPs
        */
        /* HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn); */
        /* USER CODE END TIM10:TIM1_UP_TIM10_IRQn disable */

        /* USER CODE BEGIN TIM10_MspDeInit 1 */

        /* USER CODE END TIM10_MspDeInit 1 */
    }
    else if(htim_base->Instance==TIM11)
    {
        /* USER CODE BEGIN TIM11_MspDeInit 0 */

        /* USER CODE END TIM11_MspDeInit 0 */
        /* Peripheral clock disable */
//    __HAL_RCC_TIM11_CLK_DISABLE();

        /**TIM11 GPIO Configuration
        PB9     ------> TIM11_CH1
        */
//    HAL_GPIO_DeInit(PWM2_GPIO_Port, PWM2_Pin);

        /* USER CODE BEGIN TIM11_MspDeInit 1 */

        /* USER CODE END TIM11_MspDeInit 1 */
    }

}

/**
* @brief TIM_IC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_ic: TIM_IC handle pointer
* @retval None
*/
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* htim_ic)
{
    if(htim_ic->Instance==TIM9)
    {
        /* USER CODE BEGIN TIM9_MspDeInit 0 */

        /* USER CODE END TIM9_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM9_CLK_DISABLE();

        /**TIM9 GPIO Configuration
        PE5     ------> TIM9_CH1
        PE6     ------> TIM9_CH2
        */
        HAL_GPIO_DeInit(GPIOE, SB2_FG_Pin|SideBrush_FG_Pin);

        /* TIM9 interrupt DeInit */
        /* USER CODE BEGIN TIM9:TIM1_BRK_TIM9_IRQn disable */
        /**
        * Uncomment the line below to disable the "TIM1_BRK_TIM9_IRQn" interrupt
        * Be aware, disabling shared interrupt may affect other IPs
        */
        /* HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn); */
        /* USER CODE END TIM9:TIM1_BRK_TIM9_IRQn disable */

        /* USER CODE BEGIN TIM9_MspDeInit 1 */

        /* USER CODE END TIM9_MspDeInit 1 */
    }

}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(huart->Instance==USART3)
    {
        /* USER CODE BEGIN USART3_MspInit 0 */

        /* USER CODE END USART3_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**USART3 GPIO Configuration
        PC10     ------> USART3_TX
        PC11     ------> USART3_RX
        */
        GPIO_InitStruct.Pin = TX_485_Pin|RX_485_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USART3 interrupt Init */
        HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
        /* USER CODE BEGIN USART3_MspInit 1 */

        /* USER CODE END USART3_MspInit 1 */
    }
    if(huart->Instance==UART4)
    {
        /* USER CODE BEGIN USART3_MspInit 0 */

        /* USER CODE END USART3_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_UART4_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**USART3 GPIO Configuration
        PC10     ------> USART3_TX
        PC11     ------> USART3_RX
        */
        GPIO_InitStruct.Pin = USART_DUB_TX_Pin|USART_DUB_RX_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* USART3 interrupt Init */
        HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
        /* USER CODE BEGIN USART3_MspInit 1 */

        /* USER CODE END USART3_MspInit 1 */
    }
}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if(huart->Instance==USART3)
    {
        /* USER CODE BEGIN USART3_MspDeInit 0 */

        /* USER CODE END USART3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();

        /**USART3 GPIO Configuration
        PC10     ------> USART3_TX
        PC11     ------> USART3_RX
        */
        HAL_GPIO_DeInit(GPIOD, TX_485_Pin|RX_485_Pin);

        /* USART3 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART3_IRQn);
        /* USER CODE BEGIN USART3_MspDeInit 1 */

        /* USER CODE END USART3_MspDeInit 1 */
    }
    if(huart->Instance==UART4)
    {
        /* USER CODE BEGIN USART3_MspDeInit 0 */

        /* USER CODE END USART3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_UART4_CLK_DISABLE();

        /**USART3 GPIO Configuration
        PC10     ------> USART3_TX
        PC11     ------> USART3_RX
        */
        HAL_GPIO_DeInit(GPIOC, USART_DUB_TX_Pin|USART_DUB_RX_Pin);

        /* USART3 interrupt DeInit */
        HAL_NVIC_DisableIRQ(UART4_IRQn);
        /* USER CODE BEGIN USART3_MspDeInit 1 */

        /* USER CODE END USART3_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
