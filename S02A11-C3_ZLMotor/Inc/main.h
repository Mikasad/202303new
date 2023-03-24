/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define USE_LCD        /* enable LCD  */  

#define USE_DHCP       /* enable DHCP, if disabled static address is used */

/* USER CODE BEGIN Includes */
#define IP_ADDR0   (uint8_t) 192
#define IP_ADDR1   (uint8_t) 168
#define IP_ADDR2   (uint8_t) 0
#define IP_ADDR3   (uint8_t) 23
   
/*NETMASK*/
#define NETMASK_ADDR0   (uint8_t) 255
#define NETMASK_ADDR1   (uint8_t) 255
#define NETMASK_ADDR2   (uint8_t) 255
#define NETMASK_ADDR3   (uint8_t) 0

/*Gateway Address*/
#define GW_ADDR0   (uint8_t) 192
#define GW_ADDR1   (uint8_t) 168
#define GW_ADDR2   (uint8_t) 0
#define GW_ADDR3   (uint8_t) 1

#define TCP_SERVER_RX_BUFSIZE	200		//?“辰?tcp server℅?∩車?車那?那y?Y3∟?豕
#define TCP_SERVER_PORT			8080	//?“辰?tcp server米????迆
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Start_Stop_Pin GPIO_PIN_0
#define Start_Stop_GPIO_Port GPIOD
#define Start_Stop_EXTI_IRQn EXTI0_IRQn
#define M1_ENCODER_B_Pin GPIO_PIN_13
#define M1_ENCODER_B_GPIO_Port GPIOD
#define M1_ENCODER_A_Pin GPIO_PIN_12
#define M1_ENCODER_A_GPIO_Port GPIOD
#define M2_ENCODER_B_Pin GPIO_PIN_3
#define M2_ENCODER_B_GPIO_Port GPIOB
#define M2_ENCODER_A_Pin GPIO_PIN_15
#define M2_ENCODER_A_GPIO_Port GPIOA
#define M1_DISSIPATIVE_BRK_Pin GPIO_PIN_8
#define M1_DISSIPATIVE_BRK_GPIO_Port GPIOD

#define RS485_RX_Pin GPIO_PIN_10
#define RS485_RX_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_9
#define RS485_TX_GPIO_Port GPIOA
#define RS485_DE_Pin GPIO_PIN_9
#define RS485_DE_GPIO_Port GPIOC
#define RS485_nRE_Pin GPIO_PIN_8
#define RS485_nRE_GPIO_Port GPIOA
#define M2_OCP_Pin GPIO_PIN_4
#define M2_OCP_GPIO_Port GPIOI

#define M2_HALL_U_Pin GPIO_PIN_5
#define M2_HALL_U_GPIO_Port GPIOD
#define M2_HALL_V_Pin GPIO_PIN_6
#define M2_HALL_V_GPIO_Port GPIOD
#define M2_HALL_W_Pin GPIO_PIN_7
#define M2_HALL_W_GPIO_Port GPIOD

#define UART_TX_Pin GPIO_PIN_10
#define UART_TX_GPIO_Port GPIOB
#define UART_RX_Pin GPIO_PIN_11
#define UART_RX_GPIO_Port GPIOB

#define M2_TEMPERATURE_Pin GPIO_PIN_4
#define M2_TEMPERATURE_GPIO_Port GPIOC
#define M1_TEMPERATURE_Pin GPIO_PIN_3
#define M1_TEMPERATURE_GPIO_Port GPIOC

#define M2_PWM_WH_Pin GPIO_PIN_8
#define M2_PWM_WH_GPIO_Port GPIOC
#define M2_PWM_VH_Pin GPIO_PIN_7
#define M2_PWM_VH_GPIO_Port GPIOC
#define M2_PWM_UH_Pin GPIO_PIN_6
#define M2_PWM_UH_GPIO_Port GPIOC
#define M1_BUS_VOLTAGE_Pin GPIO_PIN_3
#define M1_BUS_VOLTAGE_GPIO_Port GPIOA


#define M1_TIM1_BRAKE_Pin GPIO_PIN_15
#define M1_TIM1_BRAKE_GPIO_Port GPIOE
#define M2_TIM8_BRAKE_Pin GPIO_PIN_6
#define M2_TIM8_BRAKE_GPIO_Port GPIOA
//#define M1_HALL_H3_Pin GPIO_PIN_12
//#define M1_HALL_H3_GPIO_Port GPIOB
//#define M1_HALL_H2_Pin GPIO_PIN_11
//#define M1_HALL_H2_GPIO_Port GPIOD
//#define M1_HALL_H1_Pin GPIO_PIN_10
//#define M1_HALL_H1_GPIO_Port GPIOD

//#define M1_HALL_V_Pin GPIO_PIN_11
//#define M1_HALL_V_GPIO_Port GPIOD
//#define M1_HALL_U_Pin GPIO_PIN_10
//#define M1_HALL_U_GPIO_Port GPIOD
//#define M1_HALL_W_Pin GPIO_PIN_12
//#define M1_HALL_W_GPIO_Port GPIOB


#define M1_CURR_AMPL_U_Pin GPIO_PIN_5
#define M1_CURR_AMPL_U_GPIO_Port GPIOC
#define M1_CURR_AMPL_W_Pin GPIO_PIN_1
#define M1_CURR_AMPL_W_GPIO_Port GPIOB
#define M1_CURR_AMPL_V_Pin GPIO_PIN_0
#define M1_CURR_AMPL_V_GPIO_Port GPIOB

#define M1_PWM_WH_Pin GPIO_PIN_13
#define M1_PWM_WH_GPIO_Port GPIOE




#define M2_PWM_UL_Pin GPIO_PIN_5
#define M2_PWM_UL_GPIO_Port GPIOA
#define M1_PWM_UL_Pin GPIO_PIN_8
#define M1_PWM_UL_GPIO_Port GPIOE
#define M1_PWM_UH_Pin GPIO_PIN_9
#define M1_PWM_UH_GPIO_Port GPIOE
#define M1_PWM_VH_Pin GPIO_PIN_11
#define M1_PWM_VH_GPIO_Port GPIOE
#define Encoder_2_A_Pin GPIO_PIN_4
#define Encoder_2_A_GPIO_Port GPIOB
#define Encoder_2_B_Pin GPIO_PIN_7
#define Encoder_2_B_GPIO_Port GPIOA
#define Fault_1_Pin GPIO_PIN_14
#define Fault_1_GPIO_Port GPIOE


#define M1_HALL_W_Pin GPIO_PIN_12
#define M1_HALL_W_GPIO_Port GPIOB
#define M1_HALL_V_Pin GPIO_PIN_11
#define M1_HALL_V_GPIO_Port GPIOD
#define M1_HALL_U_Pin GPIO_PIN_10
#define M1_HALL_U_GPIO_Port GPIOD


#define M2_CURR_AMPL_U_Pin GPIO_PIN_0
#define M2_CURR_AMPL_U_GPIO_Port GPIOC
#define M2_CURR_AMPL_V_Pin GPIO_PIN_1
#define M2_CURR_AMPL_V_GPIO_Port GPIOC
#define M2_CURR_AMPL_W_Pin GPIO_PIN_2
#define M2_CURR_AMPL_W_GPIO_Port GPIOC

#define Fault_2_Pin GPIO_PIN_7
#define Fault_2_GPIO_Port GPIOE
#define M1_PWM_VL_Pin GPIO_PIN_10
#define M1_PWM_VL_GPIO_Port GPIOE
#define M1_PWM_WL_Pin GPIO_PIN_12
#define M1_PWM_WL_GPIO_Port GPIOE
#define Normal_Pin GPIO_PIN_5
#define Normal_GPIO_Port GPIOE

#define M2_PWM_VL_Pin GPIO_PIN_14
#define M2_PWM_VL_GPIO_Port GPIOB
#define M2_PWM_WL_Pin GPIO_PIN_15
#define M2_PWM_WL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
