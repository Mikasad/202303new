#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "bsp_BDCMotor.h"
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);
#define HALL_3_Pin GPIO_PIN_2
#define HALL_3_GPIO_Port GPIOE
#define HALL_3_EXTI_IRQn EXTI2_IRQn
#define HALL_4_Pin GPIO_PIN_3
#define HALL_4_GPIO_Port GPIOE
#define HALL_4_EXTI_IRQn EXTI3_IRQn
#define FAN_FG_Pin GPIO_PIN_5
#define FAN_FG_GPIO_Port GPIOE
#define SideBrush_FG_Pin GPIO_PIN_6
#define SideBrush_FG_GPIO_Port GPIOE
#define HALL_U2_Pin GPIO_PIN_13
#define HALL_U2_GPIO_Port GPIOC
#define HALL_U2_EXTI_IRQn EXTI15_10_IRQn
#define HALL_V2_Pin GPIO_PIN_14
#define HALL_V2_GPIO_Port GPIOC
#define HALL_V2_EXTI_IRQn EXTI15_10_IRQn
#define HALL_W2_Pin GPIO_PIN_15
#define HALL_W2_GPIO_Port GPIOC
#define HALL_W2_EXTI_IRQn EXTI15_10_IRQn

#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define Isense1_Pin GPIO_PIN_0
#define Isense1_GPIO_Port GPIOC
#define Isense2_Pin GPIO_PIN_1
#define Isense2_GPIO_Port GPIOC
#define Isense3_Pin GPIO_PIN_2
#define Isense3_GPIO_Port GPIOC
#define Isense4_Pin GPIO_PIN_3
#define Isense4_GPIO_Port GPIOC
#define Isense5_Pin GPIO_PIN_0
#define Isense5_GPIO_Port GPIOA
#define Isense6_Pin GPIO_PIN_1
#define Isense6_GPIO_Port GPIOA
#define Current_1_Pin GPIO_PIN_2
#define Current_1_GPIO_Port GPIOA
#define MotorTemp1_Pin GPIO_PIN_3
#define MotorTemp1_GPIO_Port GPIOA
#define Current_2_Pin GPIO_PIN_4
#define Current_2_GPIO_Port GPIOA
#define INLU2_Pin GPIO_PIN_5
#define INLU2_GPIO_Port GPIOA
#define MotorTemp2_Pin GPIO_PIN_6
#define MotorTemp2_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_7
#define PWM4_GPIO_Port GPIOA
#define BUS_VOL_DECT_Pin GPIO_PIN_4
#define BUS_VOL_DECT_GPIO_Port GPIOC
#define FANTemp_Pin GPIO_PIN_5
#define FANTemp_GPIO_Port GPIOC
#define PWM5_Pin GPIO_PIN_0
#define PWM5_GPIO_Port GPIOB
#define PWM6_Pin GPIO_PIN_1
#define PWM6_GPIO_Port GPIOB
#define FAN_F_R GPIO_PIN_7
#define FAN_F_R_GPIO_Port GPIOE
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
#define FAN_PWM_Pin GPIO_PIN_14
#define FAN_PWM_GPIO_Port GPIOE
#define PWM_AH2_Pin GPIO_PIN_10
#define PWM_AH2_GPIO_Port GPIOB
#define PWM_BH2_Pin GPIO_PIN_11
#define PWM_BH2_GPIO_Port GPIOB
#define RUN_Pin GPIO_PIN_12
#define RUN_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_13
#define CAN_TX_GPIO_Port GPIOB
#define INLV2_Pin GPIO_PIN_14
#define INLV2_GPIO_Port GPIOB
#define INLW2_Pin GPIO_PIN_15
#define INLW2_GPIO_Port GPIOB
#define PWM_AL3_Pin GPIO_PIN_8
#define PWM_AL3_GPIO_Port GPIOD
#define PWM_BL3_Pin GPIO_PIN_9
#define PWM_BL3_GPIO_Port GPIOD
#define PWM_AL4_Pin GPIO_PIN_10
#define PWM_AL4_GPIO_Port GPIOD
#define PWM_BL4_Pin GPIO_PIN_11
#define PWM_BL4_GPIO_Port GPIOD
#define PWM_AH3_Pin GPIO_PIN_12
#define PWM_AH3_GPIO_Port GPIOD
#define PWM_BH3_Pin GPIO_PIN_13
#define PWM_BH3_GPIO_Port GPIOD
#define PWM_AH4_Pin GPIO_PIN_14
#define PWM_AH4_GPIO_Port GPIOD
#define PWM_BH4_Pin GPIO_PIN_15
#define PWM_BH4_GPIO_Port GPIOD
#define INHU2_Pin GPIO_PIN_6
#define INHU2_GPIO_Port GPIOC
#define INHV2_Pin GPIO_PIN_7
#define INHV2_GPIO_Port GPIOC
#define INHW2_Pin GPIO_PIN_8
#define INHW2_GPIO_Port GPIOC
#define SideBrush_PWM_Pin GPIO_PIN_9
#define SideBrush_PWM_GPIO_Port GPIOC
#define SideBrush_BRAKE_Pin GPIO_PIN_8
#define SideBrush_BRAKE_GPIO_Port GPIOA
#define BRAKE3 GPIO_PIN_11
#define BRAKE3_GPIO_Port GPIOA
#define BRAKE4 GPIO_PIN_10
#define BRAKE4_GPIO_Port GPIOA
#define BRAKE5 GPIO_PIN_9
#define BRAKE5_GPIO_Port GPIOA
#define FAULT1_Pin GPIO_PIN_6
#define FAULT1_GPIO_Port GPIOB
#define SideBrush_F_R_Pin GPIO_PIN_7
#define SideBrush_F_R_GPIO_Port GPIOB
#define FAULT2_Pin GPIO_PIN_12
#define FAULT2_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define PWM_AH1_Pin GPIO_PIN_15
#define PWM_AH1_GPIO_Port GPIOA
#define USART_DUB_TX_Pin GPIO_PIN_10
#define USART_DUB_TX_GPIO_Port GPIOC
#define USART_DUB_RX_Pin GPIO_PIN_11
#define USART_DUB_RX_GPIO_Port GPIOC
#define PWM_AL1_Pin GPIO_PIN_0
#define PWM_AL1_GPIO_Port GPIOD
#define PWM_BL1_Pin GPIO_PIN_1
#define PWM_BL1_GPIO_Port GPIOD
#define PWM_AL2_Pin GPIO_PIN_2
#define PWM_AL2_GPIO_Port GPIOD
#define PWM_BL2_Pin GPIO_PIN_3
#define PWM_BL2_GPIO_Port GPIOD
#define BRAKE2_Pin GPIO_PIN_4
#define BRAKE2_GPIO_Port GPIOD
#define BRAKE2_EXTI_IRQn EXTI4_IRQn
#define HALL_U1_Pin GPIO_PIN_5
#define HALL_U1_GPIO_Port GPIOD
#define HALL_U1_EXTI_IRQn EXTI9_5_IRQn
#define HALL_V1_Pin GPIO_PIN_6
#define HALL_V1_GPIO_Port GPIOD
#define HALL_V1_EXTI_IRQn EXTI9_5_IRQn
#define HALL_W1_Pin GPIO_PIN_7
#define HALL_W1_GPIO_Port GPIOD
#define HALL_W1_EXTI_IRQn EXTI9_5_IRQn
#define PWM_BH1_Pin GPIO_PIN_3
#define PWM_BH1_GPIO_Port GPIOB
#define PWM3_Pin GPIO_PIN_4
#define PWM3_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_5
#define CAN_RX_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_9
#define PWM2_GPIO_Port GPIOB
#define HALL_1_Pin GPIO_PIN_0
#define HALL_1_GPIO_Port GPIOE
#define HALL_1_EXTI_IRQn EXTI0_IRQn
#define HALL_2_Pin GPIO_PIN_1
#define HALL_2_GPIO_Port GPIOE
#define HALL_2_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

#define	BitSet(Var,Mask)	(Var |= Mask)		//??????
#define	BitClr(Var,Mask)	(Var &= (~Mask))	//????3y
#define	BitTst(Var,Mask)	((Var & Mask) != 0 ? 1 : 0)	//??2aê?
#define	BitInv(Var,Mask)	((Var & Mask) != 0 ? (Var &= (~Mask)) : (Var |= Mask))	//??·′×a

#define A1mSec		  0x01
#define A2mSec		  0x02
#define A10mSec			0x04
#define A20mSec			0x08
#define A250mSec	  0x10
#define A500mSec	  0x20
#define A1Sec		  	0x40

#define	MOTOR8_PWM_PERIOD			50	//风机的PWM周期 ms
#define	TIM9_PRESCALER				167	
#define	PHASE_ERR_MAX					5000
#define SUTCK_ERR_MAX         3000
#define	FILTER_COEFFICIENT		0.005f	//一阶滤波的系数  0.005
#define	FAN_F_R_SET			HAL_GPIO_WritePin(FAN_F_R_GPIO_Port, FAN_F_R, GPIO_PIN_SET);
#define	FAN_F_R_RESET		HAL_GPIO_WritePin(FAN_F_R_GPIO_Port, FAN_F_R, GPIO_PIN_RESET);
#define	FAN_F_R_TOG			HAL_GPIO_TogglePin(FAN_F_R_GPIO_Port,FAN_F_R);

typedef enum 
{
	IDLE, INIT, START, RUN, STOP1, BRAKE, WAIT, FAULT,HALL_STUDY_MOTOR5,HALL_STUDY_MOTOR6
} SystStatus_t;
extern SystStatus_t MotorState;
typedef enum 
{
NO_FAULT, OVER_VOLT, UNDER_VOLT, FRE_UNDER_VOLT
} BusV_t;
typedef struct              //测量高电平脉宽
{   
	uint8_t   ucStartFlag;
	uint8_t   ucFinishFlag;
	uint16_t  usCtr;					//定时器计数
	uint16_t  usPeriod;
	uint16_t	uDuty_ratio;		//算出的占空比
	uint16_t	uPulseCnt;			//高脉冲计数
}STRUCT_CAPTURE;
extern  void HallStudyHandle0(void);
extern  void HallStudyHandle1(void);
extern s16 GetMotorSpeed(u8 Mortor_NUM);
extern uint32_t Err_codeHis[10],Err_codeTick[10];
extern uint8_t OverFlow_Cnt[5];
typedef  void (*pFunction)(void);
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
