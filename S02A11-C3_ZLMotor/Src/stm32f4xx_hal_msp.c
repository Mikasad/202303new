/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32f4xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization
  *                      and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

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

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3);

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
        __HAL_RCC_ADC1_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = M1_BUS_VOLTAGE_Pin; /*电压采集 PA3*/
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(M1_BUS_VOLTAGE_GPIO_Port, &GPIO_InitStruct);
			
			  GPIO_InitStruct.Pin = M1_TEMPERATURE_Pin; /*M1 温度采集 PC4*/
        HAL_GPIO_Init(M1_TEMPERATURE_GPIO_Port, &GPIO_InitStruct);
				
				GPIO_InitStruct.Pin = M2_TEMPERATURE_Pin; /*M2 温度采集 PC3*/
        HAL_GPIO_Init(M2_TEMPERATURE_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = M2_CURR_AMPL_U_Pin|M2_CURR_AMPL_V_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = M1_CURR_AMPL_U_Pin|M1_CURR_AMPL_V_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(M1_CURR_AMPL_U_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = M1_CURR_AMPL_W_Pin;    /*M1  U*/
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(M1_CURR_AMPL_W_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = M1_CURR_AMPL_V_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(M1_CURR_AMPL_V_GPIO_Port, &GPIO_InitStruct);

    }
    else if(hadc->Instance==ADC2)
    {
        __HAL_RCC_ADC2_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        GPIO_InitStruct.Pin = M2_CURR_AMPL_V_Pin|M2_CURR_AMPL_W_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = M1_CURR_AMPL_W_Pin|M1_CURR_AMPL_V_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
        __HAL_RCC_ADC1_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOC, M2_CURR_AMPL_U_Pin|M2_CURR_AMPL_V_Pin);

        HAL_GPIO_DeInit(GPIOA, M1_CURR_AMPL_U_Pin|M1_CURR_AMPL_V_Pin);


    }
    else if(hadc->Instance==ADC2)
    {
        __HAL_RCC_ADC2_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOC, M2_CURR_AMPL_V_Pin|M2_CURR_AMPL_W_Pin);

        HAL_GPIO_DeInit(GPIOA, M1_CURR_AMPL_W_Pin|M1_CURR_AMPL_V_Pin);


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
        /*Configure GPIO pin : PE15 tim1刹车*/
        GPIO_InitStruct.Pin = M1_TIM1_BRAKE_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(M1_TIM1_BRAKE_GPIO_Port, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM1_MspInit 1 */

        /* USER CODE END TIM1_MspInit 1 */
    }
    else if(htim_base->Instance==TIM4)
    {
        /* USER CODE BEGIN TIM4_MspInit 0 */

        /* USER CODE END TIM4_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM4_CLK_ENABLE();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**TIM4 GPIO Configuration
        PD14     ------> TIM4_CH3
        PD13     ------> TIM4_CH2
        PD12     ------> TIM4_CH1
        */
//    GPIO_InitStruct.Pin = M1_HALL_H3_Pin|M1_HALL_H2_Pin|M1_HALL_H1_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM4_MspInit 1 */

        /* USER CODE END TIM4_MspInit 1 */
    }
    else if(htim_base->Instance==TIM8)
    {
        /* USER CODE BEGIN TIM8_MspInit 0 */

        /* USER CODE END TIM8_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM8_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM8 GPIO Configuration
        PA6     ------> TIM8_BKIN
        */
        /*Configure GPIO pin : PA6 tim8刹车*/
        GPIO_InitStruct.Pin = M2_TIM8_BRAKE_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(M2_TIM8_BRAKE_GPIO_Port, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM8_MspInit 1 */

        /* USER CODE END TIM8_MspInit 1 */
    }

}

/**
* @brief TIM_Encoder MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_encoder: TIM_Encoder handle pointer
* @retval None
*/
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(htim_encoder->Instance==TIM2)
    {
        /* USER CODE BEGIN TIM2_MspInit 0 */

        /* USER CODE END TIM2_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM2 GPIO Configuration
        PB3     ------> TIM2_CH2
        PA15     ------> TIM2_CH1
        */
        GPIO_InitStruct.Pin = M2_ENCODER_B_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(M2_ENCODER_B_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = M2_ENCODER_A_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(M2_ENCODER_A_GPIO_Port, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM2_MspInit 1 */

        /* USER CODE END TIM2_MspInit 1 */
    }
    else if(htim_encoder->Instance==TIM4)
    {
        /* USER CODE BEGIN TIM3_MspInit 0 */

        /* USER CODE END TIM3_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM4_CLK_ENABLE();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**TIM4 GPIO Configuration
        PD13     ------> TIM4_CH2
        PD12     ------> TIM4_CH1
        */
        GPIO_InitStruct.Pin = M1_ENCODER_B_Pin|M1_ENCODER_A_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM3_MspInit 1 */

        /* USER CODE END TIM3_MspInit 1 */
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
        PE13     ------> TIM1_CH3
        PE8     ------> TIM1_CH1N
        PE9     ------> TIM1_CH1
        PE11     ------> TIM1_CH2
        PE10     ------> TIM1_CH2N
        PE12     ------> TIM1_CH3N
        */
        GPIO_InitStruct.Pin = M1_PWM_WH_Pin|M1_PWM_UL_Pin|M1_PWM_UH_Pin|M1_PWM_VH_Pin
                              |M1_PWM_VL_Pin|M1_PWM_WL_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


        /* USER CODE BEGIN TIM1_MspPostInit 1 */

        /* USER CODE END TIM1_MspPostInit 1 */
    }
    else if(htim->Instance==TIM8)
    {
        /* USER CODE BEGIN TIM8_MspPostInit 0 */

        /* USER CODE END TIM8_MspPostInit 0 */

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM8 GPIO Configuration
        PC8     ------> TIM8_CH3
        PC7     ------> TIM8_CH2
        PC6     ------> TIM8_CH1
        PA5     ------> TIM8_CH1N
        PB14     ------> TIM8_CH2N
        PB15     ------> TIM8_CH3N
        */
        GPIO_InitStruct.Pin = M2_PWM_WH_Pin|M2_PWM_VH_Pin|M2_PWM_UH_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = M2_PWM_UL_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(M2_PWM_UL_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = M2_PWM_VL_Pin|M2_PWM_WL_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM8_MspPostInit 1 */

        /* USER CODE END TIM8_MspPostInit 1 */
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
        PE13     ------> TIM1_CH3
        PE8     ------> TIM1_CH1N
        PE9     ------> TIM1_CH1
        PE11     ------> TIM1_CH2
        PE10     ------> TIM1_CH2N
        PE12     ------> TIM1_CH3N
        PE15     ------> TIM1_BKIN
        */
        HAL_GPIO_DeInit(GPIOE, M1_PWM_WH_Pin|M1_PWM_UL_Pin|M1_PWM_UH_Pin|M1_PWM_VH_Pin
                        |M1_PWM_VL_Pin|M1_PWM_WL_Pin);

        /* TIM1 interrupt DeInit */
        HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
        HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
        /* USER CODE BEGIN TIM1_MspDeInit 1 */

        /* USER CODE END TIM1_MspDeInit 1 */
    }
    else if(htim_base->Instance==TIM4)
    {
        /* USER CODE BEGIN TIM4_MspDeInit 0 */

        /* USER CODE END TIM4_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM4_CLK_DISABLE();

        /**TIM4 GPIO Configuration
        PD14     ------> TIM4_CH3
        PD13     ------> TIM4_CH2
        PD12     ------> TIM4_CH1
        */
//    HAL_GPIO_DeInit(GPIOD, M1_HALL_H3_Pin|M1_HALL_H2_Pin|M1_HALL_H1_Pin);

        /* TIM4 interrupt DeInit */
        HAL_NVIC_DisableIRQ(TIM4_IRQn);
        /* USER CODE BEGIN TIM4_MspDeInit 1 */

        /* USER CODE END TIM4_MspDeInit 1 */
    }
    else if(htim_base->Instance==TIM8)
    {
        /* USER CODE BEGIN TIM8_MspDeInit 0 */

        /* USER CODE END TIM8_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM8_CLK_DISABLE();

        /**TIM8 GPIO Configuration
        PI4     ------> TIM8_BKIN
        PC8     ------> TIM8_CH3
        PC7     ------> TIM8_CH2
        PC6     ------> TIM8_CH1
        PA5     ------> TIM8_CH1N
        PB14     ------> TIM8_CH2N
        PB15     ------> TIM8_CH3N
        */
        HAL_GPIO_DeInit(M2_OCP_GPIO_Port, M2_OCP_Pin);

        HAL_GPIO_DeInit(GPIOC, M2_PWM_WH_Pin|M2_PWM_VH_Pin|M2_PWM_UH_Pin);

        HAL_GPIO_DeInit(M2_PWM_UL_GPIO_Port, M2_PWM_UL_Pin);

        HAL_GPIO_DeInit(GPIOB, M2_PWM_VL_Pin|M2_PWM_WL_Pin);

        /* TIM8 interrupt DeInit */
        HAL_NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
        HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
        /* USER CODE BEGIN TIM8_MspDeInit 1 */

        /* USER CODE END TIM8_MspDeInit 1 */
    }

}

/**
* @brief TIM_Encoder MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_encoder: TIM_Encoder handle pointer
* @retval None
*/
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_encoder)
{
    if(htim_encoder->Instance==TIM2)
    {
        /* USER CODE BEGIN TIM2_MspDeInit 0 */

        /* USER CODE END TIM2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM2_CLK_DISABLE();

        /**TIM2 GPIO Configuration
        PB3     ------> TIM2_CH2
        PA15     ------> TIM2_CH1
        */
        HAL_GPIO_DeInit(M2_ENCODER_B_GPIO_Port, M2_ENCODER_B_Pin);

        HAL_GPIO_DeInit(M2_ENCODER_A_GPIO_Port, M2_ENCODER_A_Pin);

        /* TIM2 interrupt DeInit */
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
        /* USER CODE BEGIN TIM2_MspDeInit 1 */

        /* USER CODE END TIM2_MspDeInit 1 */
    }
    else if(htim_encoder->Instance==TIM4)
    {
        /* USER CODE BEGIN TIM3_MspDeInit 0 */

        /* USER CODE END TIM3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM4_CLK_DISABLE();

        /**TIM3 GPIO Configuration
        PB5     ------> TIM3_CH2
        PB4     ------> TIM3_CH1
        */
        HAL_GPIO_DeInit(GPIOD, M1_ENCODER_B_Pin|M1_ENCODER_A_Pin);

        /* TIM3 interrupt DeInit */
        HAL_NVIC_DisableIRQ(TIM4_IRQn);
        /* USER CODE BEGIN TIM3_MspDeInit 1 */

        /* USER CODE END TIM3_MspDeInit 1 */
    }

}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
//void HAL_UART_MspInit(UART_HandleTypeDef* huart)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(huart->Instance==USART1)
//  {
//  /* USER CODE BEGIN USART1_MspInit 0 */

//  /* USER CODE END USART1_MspInit 0 */
//    /* Peripheral clock enable */
//    __HAL_RCC_USART1_CLK_ENABLE();
//
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    /**USART1 GPIO Configuration
//    PA10     ------> USART1_RX
//    PA9     ------> USART1_TX
//    */
//    GPIO_InitStruct.Pin = UART_RX_Pin|UART_TX_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//  /* USER CODE BEGIN USART1_MspInit 1 */

//  /* USER CODE END USART1_MspInit 1 */
//  }

//}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
//void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
//{
//  if(huart->Instance==USART1)
//  {
//  /* USER CODE BEGIN USART1_MspDeInit 0 */

//  /* USER CODE END USART1_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_USART1_CLK_DISABLE();
//
//    /**USART1 GPIO Configuration
//    PA10     ------> USART1_RX
//    PA9     ------> USART1_TX
//    */
//    HAL_GPIO_DeInit(GPIOA, UART_RX_Pin|UART_TX_Pin);

//    /* USART1 interrupt DeInit */
//    HAL_NVIC_DisableIRQ(USART1_IRQn);
//  /* USER CODE BEGIN USART1_MspDeInit 1 */

//  /* USER CODE END USART1_MspDeInit 1 */
//  }

//}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
