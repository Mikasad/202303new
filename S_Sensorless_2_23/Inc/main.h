/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "bsp_BDCMotor.h"
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Eleva1_Hall_1_Pin GPIO_PIN_3
#define Eleva1_Hall_1_GPIO_Port GPIOE
#define Eleva1_Hall_1_EXTI_IRQn EXTI3_IRQn
#define Eleva2_Hall_1_Pin GPIO_PIN_4
#define Eleva2_Hall_1_GPIO_Port GPIOE
#define Eleva2_Hall_1_EXTI_IRQn EXTI4_IRQn
#define Eleva2_PWM1_Pin GPIO_PIN_5
#define Eleva2_PWM1_GPIO_Port GPIOE
#define Eleva2_PWM2_Pin GPIO_PIN_6
#define Eleva2_PWM2_GPIO_Port GPIOE
#define POWER_EN_Pin GPIO_PIN_13
#define POWER_EN_GPIO_Port GPIOC
#define SelfLock_Pin GPIO_PIN_14
#define SelfLock_GPIO_Port GPIOC
#define Power_Ctrl_Pin GPIO_PIN_15
#define Power_Ctrl_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define PUMP1ISense_Pin GPIO_PIN_0
#define PUMP1ISense_GPIO_Port GPIOC
#define PUMP1_NTC_Pin GPIO_PIN_1
#define PUMP1_NTC_GPIO_Port GPIOC
#define PUMP2ISense_Pin GPIO_PIN_2
#define PUMP2ISense_GPIO_Port GPIOC
#define PUMP2_NTC_Pin GPIO_PIN_3
#define PUMP2_NTC_GPIO_Port GPIOC
#define BUS_VOL_DECT_Pin GPIO_PIN_0
#define BUS_VOL_DECT_GPIO_Port GPIOA
#define Eleva1ISense_Pin GPIO_PIN_1
#define Eleva1ISense_GPIO_Port GPIOA
#define Eleva2ISense_Pin GPIO_PIN_2
#define Eleva2ISense_GPIO_Port GPIOA
#define SideBrushISense_Pin GPIO_PIN_3
#define SideBrushISense_GPIO_Port GPIOA
#define FanISense_Pin GPIO_PIN_4
#define FanISense_GPIO_Port GPIOA
#define BL1_MOS_T_Pin GPIO_PIN_5
#define BL1_MOS_T_GPIO_Port GPIOA
#define BL1ISense_Pin GPIO_PIN_6
#define BL1ISense_GPIO_Port GPIOA
#define INLU2_Pin GPIO_PIN_7
#define INLU2_GPIO_Port GPIOA
#define BL2ISense_Pin GPIO_PIN_4
#define BL2ISense_GPIO_Port GPIOC
#define BL2_MOS_T_Pin GPIO_PIN_5
#define BL2_MOS_T_GPIO_Port GPIOC
#define MotorTemp1_Pin GPIO_PIN_0
#define MotorTemp1_GPIO_Port GPIOB
#define MotorTemp2_Pin GPIO_PIN_1
#define MotorTemp2_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define INLU1_Pin GPIO_PIN_8
#define INLU1_GPIO_Port GPIOE
#define INHU1_Pin GPIO_PIN_9
#define INHU1_GPIO_Port GPIOE
#define INLV1_Pin GPIO_PIN_10
#define INLV1_GPIO_Port GPIOE
#define INHV1_Pin GPIO_PIN_11
#define INHV1_GPIO_Port GPIOE
#define INLW1_Pin GPIO_PIN_12
#define INLW1_GPIO_Port GPIOE
#define INHW1_Pin GPIO_PIN_13
#define INHW1_GPIO_Port GPIOE
#define BL2HOCP_Pin GPIO_PIN_14
#define BL2HOCP_GPIO_Port GPIOE
#define BL2HOCP_EXTI_IRQn EXTI15_10_IRQn
#define BL1HOCP_Pin GPIO_PIN_15
#define BL1HOCP_GPIO_Port GPIOE
#define Eleva1_PWM1_Pin GPIO_PIN_10
#define Eleva1_PWM1_GPIO_Port GPIOB
#define Eleva1_PWM2_Pin GPIO_PIN_11
#define Eleva1_PWM2_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_13
#define CAN_TX_GPIO_Port GPIOB
#define INLV2_Pin GPIO_PIN_14
#define INLV2_GPIO_Port GPIOB
#define INLW2_Pin GPIO_PIN_15
#define INLW2_GPIO_Port GPIOB
#define SideBrush_F_R_Pin GPIO_PIN_8
#define SideBrush_F_R_GPIO_Port GPIOD
#define SideBrush_BRAKE_Pin GPIO_PIN_9
#define SideBrush_BRAKE_GPIO_Port GPIOD
#define Fan_EN_Pin GPIO_PIN_10
#define Fan_EN_GPIO_Port GPIOD
#define SideBrush_PWM_Pin GPIO_PIN_14
#define SideBrush_PWM_GPIO_Port GPIOD
#define Fan_PWM_Pin GPIO_PIN_15
#define Fan_PWM_GPIO_Port GPIOD
#define INHU2_Pin GPIO_PIN_6
#define INHU2_GPIO_Port GPIOC
#define INHV2_Pin GPIO_PIN_7
#define INHV2_GPIO_Port GPIOC
#define INHW2_Pin GPIO_PIN_8
#define INHW2_GPIO_Port GPIOC
#define Hall_U2_Pin GPIO_PIN_10
#define Hall_U2_GPIO_Port GPIOA
#define Hall_U2_EXTI_IRQn EXTI15_10_IRQn
#define Hall_V2_Pin GPIO_PIN_11
#define Hall_V2_GPIO_Port GPIOA
#define Hall_V2_EXTI_IRQn EXTI15_10_IRQn
#define Hall_W2_Pin GPIO_PIN_12
#define Hall_W2_GPIO_Port GPIOA
#define Hall_W2_EXTI_IRQn EXTI15_10_IRQn
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define Fault_1_Pin GPIO_PIN_15
#define Fault_1_GPIO_Port GPIOA
#define USART_DUB_TX_Pin GPIO_PIN_10
#define USART_DUB_TX_GPIO_Port GPIOC
#define USART_DUB_RX_Pin GPIO_PIN_11
#define USART_DUB_RX_GPIO_Port GPIOC
#define SideBrush_FG_Pin GPIO_PIN_1
#define SideBrush_FG_GPIO_Port GPIOD
#define Fault_1D4_Pin GPIO_PIN_4
#define Fault_1D4_GPIO_Port GPIOD
#define Hall_U1_Pin GPIO_PIN_5
#define Hall_U1_GPIO_Port GPIOD
#define Hall_U1_EXTI_IRQn EXTI9_5_IRQn
#define Hall_V1_Pin GPIO_PIN_6
#define Hall_V1_GPIO_Port GPIOD
#define Hall_V1_EXTI_IRQn EXTI9_5_IRQn
#define Hall_W1_Pin GPIO_PIN_7
#define Hall_W1_GPIO_Port GPIOD
#define Hall_W1_EXTI_IRQn EXTI9_5_IRQn
#define CAN_STBY_Pin GPIO_PIN_13
#define CAN_STBY_GPIO_Port GPIOC
#define Fan_FG_Pin GPIO_PIN_4
#define Fan_FG_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_5
#define CAN_RX_GPIO_Port GPIOB
#define Pump2_H_Pin GPIO_PIN_6
#define Pump2_H_GPIO_Port GPIOB
#define Pump1_H_Pin GPIO_PIN_7
#define Pump1_H_GPIO_Port GPIOB
#define Pump2_L_Pin GPIO_PIN_8
#define Pump2_L_GPIO_Port GPIOB
#define Pump1_L_Pin GPIO_PIN_9
#define Pump1_L_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define SELF_LOCK					HAL_GPIO_WritePin(SelfLock_GPIO_Port,SelfLock_Pin,GPIO_PIN_SET);
#define	SELF_UNLOCK				HAL_GPIO_WritePin(SelfLock_GPIO_Port,SelfLock_Pin,GPIO_PIN_RESET);
#define VOTAGE_ON 				HAL_GPIO_WritePin(Power_Ctrl_GPIO_Port,Power_Ctrl_Pin,GPIO_PIN_SET);
#define VOTAGE_OFF 				HAL_GPIO_WritePin(Power_Ctrl_GPIO_Port,Power_Ctrl_Pin,GPIO_PIN_RESET);
#define FAN_ON 						HAL_GPIO_WritePin(Fan_EN_GPIO_Port,Fan_EN_Pin,GPIO_PIN_SET);
#define FAN_OFF 					HAL_GPIO_WritePin(Fan_EN_GPIO_Port,Fan_EN_Pin,GPIO_PIN_RESET);
#define	BitSet(Var,Mask)	(Var |= Mask)		//??????
#define	BitClr(Var,Mask)	(Var &= (~Mask))	//????3y
#define	BitTst(Var,Mask)	((Var & Mask) != 0 ? 1 : 0)	//??2aê?
#define	BitInv(Var,Mask)	((Var & Mask) != 0 ? (Var &= (~Mask)) : (Var |= Mask))	//??·′×a
#define	MOTOR8_PWM_PERIOD			50	//风机的PWM周期 ms,原本是50ms
#define	FILTER_COEFFICIENT		0.005f	//一阶滤波的系数
typedef enum 
{
	IDLE, INIT, START, RUN, STOP1, BRAKE, WAIT, FAULT,HALL_STUDY_MOTOR5,HALL_STUDY_MOTOR6,Senless_start,Sensor_Mode
} SystStatus_t;
extern SystStatus_t MotorState;

typedef struct              //测量高电平脉宽
{   
	uint8_t   ucStartFlag;
	uint8_t   ucFinishFlag;
	uint16_t  usCtr;					//定时器计数
	uint16_t  usPeriod;
	uint16_t	uDuty_ratio;		//算出的占空比
	uint32_t	uPulseCnt;			//高脉冲计数
}STRUCT_CAPTURE;
/* USER CODE END Private defines */
extern s16 GetMotorSpeed(u8 Mortor_NUM);
extern uint32_t Err_codeHis[10],Err_codeTick[10];
extern int8_t OverFlow_Cnt[5];
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
