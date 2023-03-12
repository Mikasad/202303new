/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
#include "bsp_BDCMotor.h"
#include "Agreement.h"
#include "flash.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
void BLDC1_PhaseChange(u8 bHallState,s16 PWM_Duty);
void BLDC2_PhaseChange(u8 bHallState,s16 PWM_Duty);

const s8 Dir_tab[64]=				/* 相序转换表 */
{
//0
    ERROR1,//000|000无变化
    ERROR1,//000|001ERR
    ERROR1,//000|010ERR
    ERROR1,//000|011ERR
    ERROR1,//000|100ERR
    ERROR1,//000|101ERR
    ERROR1,//000|110ERR
    ERROR1,//000|111ERR
//1
    ERROR1,//001|000ERR
    MISSTEP,//001|001无变化
    ERROR1,//001|010ERR
    POSITIVE,//001|011正转dir_real=1
    ERROR1,//001|100ERR
    NEGATIVE,//001|101反转dir_real=2
    ERROR1,//001|110ERR
    ERROR1,//001|111ERR
//2
    ERROR1,//010|000ERR
    ERROR1,//010|001ERR
    MISSTEP,//010|010无变化
    NEGATIVE,//010|011反转dir_real=2
    ERROR1,//010|100ERR
    ERROR1,//010|101ERR
    POSITIVE,//010|110正转dir_real=1
    ERROR1,//010|111ERR
//3
    ERROR1,//011|000ERR
    NEGATIVE,//011|001反转dir_real=2
    POSITIVE,//011|010正转dir_real=1
    MISSTEP,//011|011无变化
    ERROR1,//011|100ERR
    ERROR1,//011|101ERR
    ERROR1,//011|110ERR
    ERROR1,//011|111ERR
//4
    ERROR1,//100|000ERR
    ERROR1,//100|001ERR
    ERROR1,//100|010ERR
    ERROR1,//100|011ERR
    MISSTEP,//100|100无变化
    POSITIVE,//100|101正转dir_real=1
    NEGATIVE,//100|110反转dir_real=2
    ERROR1,//100|111ERR
//5
    ERROR1,//101|000ERR
    POSITIVE,//101|001正转dir_real=1
    ERROR1,//101|010ERR
    ERROR1,//101|011ERR
    NEGATIVE,//101|100反转dir_real=2
    MISSTEP,//101|101无变化
    ERROR1,//101|110ERR
    ERROR1,//101|111ERR
//6
    ERROR1,//110|000ERR
    ERROR1,//110|001ERR
    NEGATIVE,//110|010反转dir_real=2
    ERROR1,//110|011ERR
    POSITIVE,//110|100正转dir_real=1
    ERROR1,//110|101ERR
    MISSTEP,//110|110无变化
    ERROR1,//110|111ERR
//7
    ERROR1,//111|000ERR
    ERROR1,//111|001ERR
    ERROR1,//111|010ERR
    ERROR1,//111|011ERR
    ERROR1,//111|100ERR
    ERROR1,//111|101ERR
    ERROR1,//111|110ERR
    ERROR1 //111|111无变化
};
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim12;
extern UART_HandleTypeDef huart3;
extern ADC_HandleTypeDef hadc2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    HAL_RCC_NMI_IRQHandler();
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1)
    {
    }
    /* USER CODE END NonMaskableInt_IRQn 1 */
}
/**
  * @brief  This function handles ADC1/ADC2 interrupt request.
  * @param  None
  * @retval None
  */
void ADC_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc2);
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVCall_IRQn 0 */

    /* USER CODE END SVCall_IRQn 0 */
    /* USER CODE BEGIN SVCall_IRQn 1 */

    /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI1_IRQn 0 */

    /* USER CODE END EXTI1_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
    /* USER CODE BEGIN EXTI1_IRQn 1 */

    /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI3_IRQn 0 */

    /* USER CODE END EXTI3_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
    /* USER CODE BEGIN EXTI3_IRQn 1 */

    /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI4_IRQn 0 */

    /* USER CODE END EXTI4_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
    /* USER CODE BEGIN EXTI4_IRQn 1 */

    /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI9_5_IRQn 0 */

    /* USER CODE END EXTI9_5_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    /* USER CODE BEGIN EXTI9_5_IRQn 1 */

    /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
    /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

    /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
    HAL_TIM_IRQHandler(&htim1);
//  HAL_TIM_IRQHandler(&htim9);
    /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

    /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}


void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
//  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
    /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */

    /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
    HAL_TIM_IRQHandler(&htim12);

    /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */

    /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

void TIM5_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim5);
}
/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
    /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

    /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
    HAL_TIM_IRQHandler(&htim1);
    HAL_TIM_IRQHandler(&htim10);
    /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

    /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
//void USART3_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART3_IRQn 0 */

//  /* USER CODE END USART3_IRQn 0 */
//  HAL_UART_IRQHandler(&huart3);
//  /* USER CODE BEGIN USART3_IRQn 1 */

//  /* USER CODE END USART3_IRQn 1 */
//}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI15_10_IRQn 0 */

    /* USER CODE END EXTI15_10_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    /* USER CODE BEGIN EXTI15_10_IRQn 1 */

    /* USER CODE END EXTI15_10_IRQn 1 */
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	s32 TempValue;
	 if(hadc->Instance==ADC2)
	 {
//		 BEMF();
		 Sensorless[0].PhaseUAD=ADC2->JDR3;      //     ml1_mos_t   adc12->5   pa5
		 TempValue = (s32)((Sensorless[0].PhaseUAD - Sensorless[0].PhaseUCurrent )>>3);
		 Sensorless[0].PhaseUCurrent =(u16)(TempValue + Sensorless[0].PhaseUCurrent);
			 
		 Sensorless[0].PhaseVAD=ADC2->JDR2; //ADC12->8   Moto1_temp   PB0           
		 TempValue = (s32)((Sensorless[0].PhaseVAD - Sensorless[0].PhaseVCurrent )>>3);
		 Sensorless[0].PhaseVCurrent =(u16)(TempValue + Sensorless[0].PhaseVCurrent);
			 
		 Sensorless[0].PhaseWAD=ADC2->JDR1;   //ml1_mos_t   adc12->15   pc5
		 TempValue = (s32)((Sensorless[0].PhaseWAD - Sensorless[0].PhaseWCurrent )>>3);
		 Sensorless[0].PhaseWCurrent =(u16)(TempValue + Sensorless[0].PhaseWCurrent);
		 
//		 RED_LED_TOGGLE; 
	 }
}

















extern u8 angleflag,Hall_circle_cnt,Hall_circle_cnt1;

/*超前角的周期计算和补偿*/
static void BLDC_AdvAngle(u8 motor_num)
{
    float temp_val = 0;
    if(motor_num==5)
    {
        MotorControl[5].Hall.chgperiod =	TIM5->CNT/6;																	//无刷一圈的周期
        MotorControl[5].Hall.HallState_CW = MotorControl[5].Hall.HallState;
        MotorControl[5].Hall.HallState_CCW = 0x07 ^ MotorControl[5].Hall.HallState;
        /*计算超前角*/
        MotorControl[5].Hall.Advance_Angle = 4;                                        //repair by 829
        if(MotorControl[5].Speed_Ref<=0)
            BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CCW,MotorControl[5].PWM_Duty);
        else
            BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CW,MotorControl[5].PWM_Duty);
        TIM5->CNT = 0;																																	//清零，重新计数
        __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC1);			//防止同时产生2个中断
        Hall_circle_cnt = 0;
        __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,MotorControl[5].Hall.chgperiod*(PHASE_ANGLE-MotorControl[5].Hall.Advance_Angle)/PHASE_ANGLE);		//设置第一个比较点
    }
    else if(motor_num==6)
    {
        MotorControl[6].Hall.chgperiod =	TIM12->CNT/6;
        MotorControl[6].Hall.HallState_CW = MotorControl[6].Hall.HallState;
        MotorControl[6].Hall.HallState_CCW = 0x07 ^ MotorControl[6].Hall.HallState;

        /*计算超前角*/
        if(MotorControl[6].Current.DeepFilterVAL>0) temp_val = MotorControl[6].Current.DeepFilterVAL;
        else temp_val = -MotorControl[6].Current.DeepFilterVAL;
        MotorControl[6].Hall.Advance_Angle = (temp_val*10/MOTOR6_CurrentValue(4))+5;
        if(MotorControl[6].Hall.Advance_Angle>16) MotorControl[6].Hall.Advance_Angle=16;
        if(MotorControl[6].Hall.Advance_Angle<5) MotorControl[6].Hall.Advance_Angle=5;

        if(MotorControl[6].Speed_Ref<0)
            BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CCW,MotorControl[6].PWM_Duty);
        else
            BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CW,MotorControl[6].PWM_Duty);

        TIM12->CNT = 0;
        __HAL_TIM_CLEAR_IT(&htim12, TIM_IT_CC1);			//防止同时产生2个中断
        Hall_circle_cnt1 = 0;
        __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,MotorControl[6].Hall.chgperiod*(PHASE_ANGLE-MotorControl[6].Hall.Advance_Angle)/PHASE_ANGLE);
    }
}

uint16_t  HALL_CaptureValue,HALL_PreCaptureValue;
uint16_t  HALL_CaptureValue2,HALL_PreCaptureValue2;
s32 HALL_CaptureValueDelta,HALL_CaptureValueDelta2;
u32 HALL_OVF_Counter = 0,PreHALL_OVF_Counter=0;
s8 Hall_Dir[2]= {1,1};
extern u8 errormotor6,errormotor5;
extern int CH2TEMP;
extern int8_t Sensorless_Time[7];
extern int bldc_change_time1;
extern int bldc_change_time;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	HALL1_CC_First++;
    if((GPIO_Pin == Hall_U1_Pin )||(GPIO_Pin == Hall_V1_Pin )||(GPIO_Pin == Hall_W1_Pin )) //
    {
       if(MotorControl[5].BrulessMode==1)
		{
			MotorControl[5].Hall.PrevHallState = MotorControl[5].Hall.HallState;
			MotorControl[5].Hall.HallState = HALL_GetPhase1();
			MotorControl[5].Hall.HallState_Temp = (MotorControl[5].Hall.PrevHallState<<3) | MotorControl[5].Hall.HallState ;
			MotorControl[5].Direction = Hall_Dir[0]*Dir_tab[MotorControl[5].Hall.HallState_Temp];
			BLDC5_Phase_Check();

			if(MotorState == HALL_STUDY_MOTOR5)/*霍尔换向*/
			{
				if((MotorControl[5].Hall.HallState>0)&&(MotorControl[5].Hall.HallState<7))
				{
					if(HALL_Study[0].HallSector == 1)      HALL_Study[0].HallTab[0] = MotorControl[5].Hall.HallState;
					else if(HALL_Study[0].HallSector == 2) HALL_Study[0].HallTab[1] = MotorControl[5].Hall.HallState;
					else if(HALL_Study[0].HallSector == 3) HALL_Study[0].HallTab[2] = MotorControl[5].Hall.HallState;
					else if(HALL_Study[0].HallSector == 4) HALL_Study[0].HallTab[3] = MotorControl[5].Hall.HallState;
					else if(HALL_Study[0].HallSector == 5) HALL_Study[0].HallTab[4] = MotorControl[5].Hall.HallState;
					else if(HALL_Study[0].HallSector == 6) HALL_Study[0].HallTab[5] = MotorControl[5].Hall.HallState;
				}
				else
				{
					/*HALL错误*/
					MC_SetFault(HALL5_SENSOR_ERR);
				}

				if(MotorControl[5].Direction < 0)      //上面部分只是拖着正转，但是如果方向是<0，需要反向
				{
					Hall_Dir[0] = -Hall_Dir[0];
				}
			}
			else if(MotorControl[5].Direction == 1 || MotorControl[5].Direction == -1)
			{

				if(MotorControl[5].Fault_Flag == 0&& MotorControl[5].Motor_Start_Stop==1) // 8-28修改待测试，禁止报错之后继续拧电机轴，进入外部中断，外部中断会换向
				{
								
					if(angleflag==0 || (MotorControl[5].Speed_Ref<1000&&MotorControl[5].Speed_Ref>-1000))
					{
						  HAL_TIM_OC_Stop_IT(&htim5,TIM_CHANNEL_1);
						  __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC1);	
						if(MotorControl[5].Speed_Ref <= 0)	//测试修改为参考速度	2022.1.25
						{
							MotorControl[5].Hall.HallState_CCW = 0x07 ^ MotorControl[5].Hall.HallState;
							BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CCW,MotorControl[5].PWM_Duty);
						}
						else
						{
							MotorControl[5].Hall.HallState_CW = MotorControl[5].Hall.HallState;
							BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CW,MotorControl[5].PWM_Duty);
						}
					}
					else
					{
						if(MotorControl[5].Hall.HallState == 1 )  
						{
							HAL_TIM_OC_Start_IT(&htim5,TIM_CHANNEL_1);
						}
						if(MotorControl[5].Speed_Ref <=0)
						{
	//                        if(MotorControl[5].Speed_Real>=-1300)
							{
								MotorControl[5].Hall.HallState_CCW = 0x07 ^ MotorControl[5].Hall.HallState;
							}
							BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CCW,MotorControl[5].PWM_Duty);
						}
						else
						{
							if(MotorControl[5].Speed_Real<900)
							{
								MotorControl[5].Hall.HallState_CW = MotorControl[5].Hall.HallState;
							}
							BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CW,MotorControl[5].PWM_Duty);
						}
					}

				}

				if(MotorControl[5].Hall.HallState == 1 )   //一个霍尔周期进行一次测速
				{
					bldc_change_time1=bldc_change_time;
				    bldc_change_time=0;
					MotorControl[5].Hall.HALL_CaptureValue = TIM7->CNT ;
					MotorControl[5].Hall.HALL_CaptureValueDelta = MotorControl[5].Hall.HALL_CaptureValue - MotorControl[5].Hall.HALL_PreCaptureValue + \
							65536*(HALL_OVF_Counter - MotorControl[5].Hall.PreHALL_OVF_Counter);
					BLDC_AdvAngle(5);
					MotorControl[5].Hall.HALL_PreCaptureValue = MotorControl[5].Hall.HALL_CaptureValue;
					MotorControl[5].Hall.PreHALL_OVF_Counter = HALL_OVF_Counter ;
					Sensorless_Time[1]=0;
					Sensorless_Time[2]=0;
					Sensorless_Time[3]=0;
					Sensorless_Time[4]=0;
					Sensorless_Time[5]=0;
					Sensorless_Time[6]=0;
				}
			}
			else if(MotorControl[5].Hall.HallState == 0 || MotorControl[5].Hall.HallState == 7)
			{
				MC_SetFault(HALL5_SENSOR_ERR);
			}
		}
    }
    if((GPIO_Pin == Hall_U2_Pin )||(GPIO_Pin == Hall_V2_Pin )||(GPIO_Pin == Hall_W2_Pin ))   //13 14 15
    {
        MotorControl[6].Hall.PrevHallState = MotorControl[6].Hall.HallState;
        MotorControl[6].Hall.HallState = HALL_GetPhase2();

        MotorControl[6].Hall.HallState_Temp = (MotorControl[6].Hall.PrevHallState<<3) | MotorControl[6].Hall.HallState ;
        MotorControl[6].Direction = Hall_Dir[1]*Dir_tab[MotorControl[6].Hall.HallState_Temp];
        BLDC6_Phase_Check();
        /*换向状态换向成功State会自动切换至IDLE*/
        if(MotorState == HALL_STUDY_MOTOR6)
        {
            if((MotorControl[6].Hall.HallState>0)&&(MotorControl[6].Hall.HallState<7))
            {
                if(HALL_Study[1].HallSector == 1)      HALL_Study[1].HallTab[0] = MotorControl[6].Hall.HallState;
                else if(HALL_Study[1].HallSector == 2) HALL_Study[1].HallTab[1] = MotorControl[6].Hall.HallState;
                else if(HALL_Study[1].HallSector == 3) HALL_Study[1].HallTab[2] = MotorControl[6].Hall.HallState;
                else if(HALL_Study[1].HallSector == 4) HALL_Study[1].HallTab[3] = MotorControl[6].Hall.HallState;
                else if(HALL_Study[1].HallSector == 5) HALL_Study[1].HallTab[4] = MotorControl[6].Hall.HallState;
                else if(HALL_Study[1].HallSector == 6) HALL_Study[1].HallTab[5] = MotorControl[6].Hall.HallState;
            }
            else
            {
                /*HALL错误*/
                MC_SetFault(HALL6_SENSOR_ERR);
            }

            if(MotorControl[6].Direction < 0)
            {
                Hall_Dir[1] = -Hall_Dir[1];
            }


        }
        else if(MotorControl[6].Direction == 1 || MotorControl[6].Direction == -1)
        {
            if(MotorControl[6].Hall.HallState == 1)
            {
                MotorControl[6].Hall.HALL_CaptureValue = TIM7->CNT ;

                MotorControl[6].Hall.HALL_CaptureValueDelta = MotorControl[6].Hall.HALL_CaptureValue - MotorControl[6].Hall.HALL_PreCaptureValue + 65536*(HALL_OVF_Counter - MotorControl[6].Hall.PreHALL_OVF_Counter);
                BLDC_AdvAngle(6);
                MotorControl[6].Hall.HALL_PreCaptureValue = MotorControl[6].Hall.HALL_CaptureValue;
                MotorControl[6].Hall.PreHALL_OVF_Counter = HALL_OVF_Counter ;
            }
            if(MotorControl[6].Fault_Flag == 0) // 8-28修改待测试，禁止报错之后继续拧电机轴，进入外部中断，外部中断会换向
            {
                if(angleflag==0)
                {
                    if(MotorControl[6].Speed_Ref < 0)	//测试修改为参考速度	2022.1.25
                    {
                        MotorControl[6].Hall.HallState_CCW = 0x07 ^ MotorControl[6].Hall.HallState;
                        BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CCW,MotorControl[6].PWM_Duty);
                    }
                    else
                    {
                        MotorControl[6].Hall.HallState_CW = MotorControl[6].Hall.HallState;
                        BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CW,MotorControl[6].PWM_Duty);
                    }
                }
                else
                {
                    if(MotorControl[6].Speed_Ref < 0)
                    {
                        if(MotorControl[6].Speed_Real>-900)
                        {
                            MotorControl[6].Hall.HallState_CCW = 0x07 ^ MotorControl[6].Hall.HallState;
                        }
                        BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CCW,MotorControl[6].PWM_Duty);
                    }
                    else
                    {
                        if(MotorControl[6].Speed_Real<900)
                        {
                            MotorControl[6].Hall.HallState_CW = MotorControl[6].Hall.HallState;
                        }
                        BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CW,MotorControl[6].PWM_Duty);
                    }
                }
            }

        }
        else if(MotorControl[6].Hall.HallState == 0 || MotorControl[6].Hall.HallState == 7)
        {
//          MC_SetFault(HALL6_SENSOR_ERR);
        }
    }
    if(GPIO_Pin == Eleva1_Hall_1_Pin )//这里需要检查   3
    {
//        if(MotorControl[0].PWM_DutySet > 0)
        if(MotorControl[0].PWM_Duty > 0)
        {
            if(MotorControl[0].Hall.HALL_CaptureValue>0)
            {
                MotorControl[0].Hall.HALL_CaptureValue--;
            }

        }
//        else if(MotorControl[0].PWM_DutySet < 0)
        else if(MotorControl[0].PWM_Duty < 0)
        {
            MotorControl[0].Hall.HALL_CaptureValue++;
        }
        else
        {
            if(MotorControl[0].Hall.HALL_CaptureValue_Direction_Judgment == 2)
            {
                if(MotorControl[0].Hall.HALL_CaptureValue>0)
                {
                    MotorControl[0].Hall.HALL_CaptureValue--;
                }
            }
            else if(MotorControl[0].Hall.HALL_CaptureValue_Direction_Judgment == 1)
            {
                MotorControl[0].Hall.HALL_CaptureValue++;
            }
        }
    }
    if(GPIO_Pin == Eleva2_Hall_1_Pin )  //4
    {
        if(MotorControl[1].PercentOfPWM > 0)
        {
            if(MotorControl[1].Hall.HALL_CaptureValue>0)
            {
                MotorControl[1].Hall.HALL_CaptureValue--;
            }

        }
        else if(MotorControl[1].PercentOfPWM < 0)
        {
            MotorControl[1].Hall.HALL_CaptureValue++;
        }
    }
    if(GPIO_Pin == SideBrush_FG_Pin)		//1
    {
        CH2TEMP +=1;
    }
    if(GPIO_Pin == BL2HOCP_Pin ) //电机6break信号  14
    {
        if( ((GPIOE->IDR>>14)&(0x1)) ==0 )
        {
            TIM8->CCR1 = 0;
            TIM8->CCR2 = 0;
            TIM8->CCR3 = 0;
            TIM8->EGR = 1;
            MotorControl[6].Motor_Start_Stop = DISABLE;
            errormotor6++;
            MotorControl[6].Fault_Flag = 1;
            OverFlow_Cnt[1]+=1;
            MC_SetFault(MOTOR6_BREAK);
        }
        else
        {

        }
    }
}
/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
    /* USER CODE BEGIN TIM7_IRQn 0 */

    /* USER CODE END TIM7_IRQn 0 */
    HAL_TIM_IRQHandler(&htim7);
    /* USER CODE BEGIN TIM7_IRQn 1 */

    /* USER CODE END TIM7_IRQn 1 */
}




/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
