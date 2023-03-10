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
extern int8_t OverFlow_Cnt[5];
//extern const s8 Dir_tab[64];
const s8 Dir_tab[64]=
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
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern UART_HandleTypeDef huart3;
extern CAN_HandleTypeDef hcan2;
extern ADC_HandleTypeDef hadc1;
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
    HAL_ADC_IRQHandler(&hadc1);
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
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI0_IRQn 0 */

    /* USER CODE END EXTI0_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    /* USER CODE BEGIN EXTI0_IRQn 1 */

    /* USER CODE END EXTI0_IRQn 1 */
}

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
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI2_IRQn 0 */

    /* USER CODE END EXTI2_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
    /* USER CODE BEGIN EXTI2_IRQn 1 */

    /* USER CODE END EXTI2_IRQn 1 */
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
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
    /* USER CODE BEGIN EXTI9_5_IRQn 1 */

    /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI15_10_IRQn 0 */

    /* USER CODE END EXTI15_10_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
    /* USER CODE BEGIN EXTI15_10_IRQn 1 */

    /* USER CODE END EXTI15_10_IRQn 1 */
}

uint8_t PrevHallState1,PrevHallState2,HallState1,HallState2;
uint8_t HallState1_CW,HallState1_CCW,HallState1_CW,HallState2_CCW;
uint16_t  HALL_CaptureValue,HALL_PreCaptureValue,Speed_Hz;
s16 BLDC1_Speed_RPM = 0;
s8 HallState1_Temp,HallState2_Temp;
uint16_t  HALL_CaptureValue2,HALL_PreCaptureValue2;
s16 BLDC2_Speed_RPM = 0;

s32 HALL_CaptureValueDelta,HALL_CaptureValueDelta2;
u32 HALL_OVF_Counter = 0,PreHALL_OVF_Counter=0;

u16 flaggggg=0;
s8 Hall_Dir[2]= {1,1};  
extern u8 errormotor5;
extern u8 errormotor6;
int test0 = 0;
int Brake2_Flag=0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	HALL1_CC_First++;
    if((GPIO_Pin == GPIO_PIN_5 )||(GPIO_Pin == GPIO_PIN_6 )||(GPIO_Pin == GPIO_PIN_7 )) //
    {

        MotorControl[5].Hall.PrevHallState = MotorControl[5].Hall.HallState;
        MotorControl[5].Hall.HallState = (GPIOD->IDR>>5)&(0x7);

        MotorControl[5].Hall.HallState_Temp = (MotorControl[5].Hall.PrevHallState<<3) | MotorControl[5].Hall.HallState ;
        MotorControl[5].Direction = Hall_Dir[0]*Dir_tab[MotorControl[5].Hall.HallState_Temp];
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
//                MC_SetFault(HALL5_SENSOR_ERR);
            }

            if(MotorControl[5].Direction < 0)      //上面部分只是拖着正转，但是如果方向是<0，需要反向
            {
                Hall_Dir[0] = -Hall_Dir[0];
            }
        }
        else if(MotorControl[5].Direction == 1 || MotorControl[5].Direction == -1)
        {
						BLDC5_Phase_Check();
            if(MotorControl[5].Hall.HallState == 1)   //一个霍尔周期进行一次测速
            {
                MotorControl[5].Hall.HALL_CaptureValue = TIM7->CNT ;
                MotorControl[5].Hall.HALL_CaptureValueDelta = MotorControl[5].Hall.HALL_CaptureValue - MotorControl[5].Hall.HALL_PreCaptureValue + \
							65536*(HALL_OVF_Counter - MotorControl[5].Hall.PreHALL_OVF_Counter);
                MotorControl[5].Hall.HALL_PreCaptureValue = MotorControl[5].Hall.HALL_CaptureValue;
                MotorControl[5].Hall.PreHALL_OVF_Counter = HALL_OVF_Counter ;
								if(MotorControl[5].Speed_Real > 100 && MotorControl[5].Motor_Start_Stop == 1 && MotorControl[5].Speed_Set != 0)
                {

//                    Motor5_Default_Phase_Cheek_IT(); //缺相检测

                }
                //				MotorControl[5].Direction = Dir_tab[MotorControl[5].Hall.HallState_Temp];
                //				Hall2_Slow_CNT = 0;
            }
            if(MotorControl[5].Fault_Flag == 0) // 8-28修改待测试，禁止报错之后继续拧电机轴，进入外部中断，外部中断会换向
            {
                if(MotorControl[5].PWM_Duty < 0)
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

        }
        else if(MotorControl[5].Hall.HallState == 0 || MotorControl[5].Hall.HallState == 7)
        {
//          MC_SetFault(HALL5_SENSOR_ERR);
        }
    }
    if((GPIO_Pin == HALL_U2_Pin )||(GPIO_Pin == HALL_V2_Pin )||(GPIO_Pin == HALL_W2_Pin ))   //13 14 15
    {
        MotorControl[6].Hall.PrevHallState = MotorControl[6].Hall.HallState;
        MotorControl[6].Hall.HallState = HALL_GetPhase2();
        MotorControl[6].Hall.HallState_Temp = (MotorControl[6].Hall.PrevHallState<<3) | MotorControl[6].Hall.HallState ;
        MotorControl[6].Direction = Hall_Dir[1]*Dir_tab[MotorControl[6].Hall.HallState_Temp];
        Bldc6_Hall_Check();
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
						BLDC6_Phase_Check();
            if(MotorControl[6].Hall.HallState == 1)
            {
                MotorControl[6].Hall.HALL_CaptureValue = TIM7->CNT ;

                MotorControl[6].Hall.HALL_CaptureValueDelta = MotorControl[6].Hall.HALL_CaptureValue - MotorControl[6].Hall.HALL_PreCaptureValue + 65536*(HALL_OVF_Counter - MotorControl[6].Hall.PreHALL_OVF_Counter);

                MotorControl[6].Hall.HALL_PreCaptureValue = MotorControl[6].Hall.HALL_CaptureValue;
                MotorControl[6].Hall.PreHALL_OVF_Counter = HALL_OVF_Counter ;
								if(MotorControl[6].Speed_Real  > 100 && MotorControl[6].Motor_Start_Stop == 1 && MotorControl[6].Speed_Set != 0  )
                {
//                    Motor6_Default_Phase_Cheek_IT();
                }
            }
            if(MotorControl[6].Fault_Flag == 0) // 8-28修改待测试，禁止报错之后继续拧电机轴，进入外部中断，外部中断会换向
            {
                if(MotorControl[6].PWM_Duty < 0)
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

        }
    }
    if(GPIO_Pin == HALL_1_Pin )//这里需要检查   0
    {
        if(MotorControl[0].PWM_DutySet > 0)
        {
            if(MotorControl[0].Hall.HALL_CaptureValue>0)
            {
                MotorControl[0].Hall.HALL_CaptureValue--;
            }

        }
        else if(MotorControl[0].PWM_DutySet < 0)
        {
            MotorControl[0].Hall.HALL_CaptureValue++;
        }
    }
    if(GPIO_Pin == HALL_2_Pin )  //1
    {
        if(MotorControl[1].PWM_DutySet > 0)
        {
            if(MotorControl[1].Hall.HALL_CaptureValue>0)
            {
                MotorControl[1].Hall.HALL_CaptureValue--;
            }

        }
        else if(MotorControl[1].PWM_DutySet < 0)
        {
            MotorControl[1].Hall.HALL_CaptureValue++;
        }
		else//MotorControl[1].PWM_DutySet == 0
        {

            if(MotorControl[1].Hall.HALL_CaptureValue_Direction_Judgment == 1)
            {
//                push_test++;
                if(MotorControl[1].Hall.HALL_CaptureValue > 0)
                {
                    MotorControl[1].Hall.HALL_CaptureValue--;
                }
            }
            else if(MotorControl[1].Hall.HALL_CaptureValue_Direction_Judgment == 2)
            {
//                push_test1++;
                MotorControl[1].Hall.HALL_CaptureValue++;
            }
        }
    }
    if(GPIO_Pin == HALL_3_Pin )  //2
    {
        if(MotorControl[2].PWM_DutySet > 0)
        {
            if(MotorControl[2].Hall.HALL_CaptureValue>0)
            {
                MotorControl[2].Hall.HALL_CaptureValue--;
            }
        }
        else if(MotorControl[2].PWM_DutySet < 0)
        {
            MotorControl[2].Hall.HALL_CaptureValue++;
        }
    }
    if(GPIO_Pin == HALL_4_Pin )  //9
    {
        if(MotorControl[9].PWM_DutySet > 0)
        {
            if(MotorControl[9].Hall.HALL_CaptureValue>0)
            {
                MotorControl[9].Hall.HALL_CaptureValue--;
            }
        }
        else if(MotorControl[9].PWM_DutySet < 0)
        {
            MotorControl[9].Hall.HALL_CaptureValue++;
        }
    }
    if(GPIO_Pin == BRAKE2_Pin ) //电机6break信号  4
    {
        if( ((GPIOD->IDR>>4)&(0x1)) ==0 )
        {
			errormotor6++;
			Brake2_Flag=1;
			MotorControl[6].Motor_Start_Stop = 0;    
			if(errormotor6>=5)
			{
				TIM8->CCR1 = 0;
				TIM8->CCR2 = 0;
				TIM8->CCR2 = 0;
				TIM8->EGR = 1;
				MC_SetFault(MOTOR6_BREAK);
				MotorControl[6].Motor_Start_Stop = 0;    
				MotorControl[6].Fault_Flag = 1;
				Brake2_Flag=2;
//				OverFlow_Cnt[1] +=1;
//				errormotor6=0;
			}
        }
		else
        {
			if(Brake2_Flag==1)
			{
				MotorControl[6].Motor_Start_Stop = 1; 
				Brake2_Flag=0;
			}				
        }
		
    }
    if(GPIO_Pin == BRAKE3 )  //电机3 4
    {
        if( ((GPIOA->IDR>>11)&(0x1)) ==0 )
        {

            if( ((GPIOA->IDR>>11)&(0x1)) ==0 )
            {
								
							
                TIM10->CCR1 = 2*(PWM_PERIOD+PWM_PeriodOFFSET) ;
                TIM11->CCR1 = 2*(PWM_PERIOD+PWM_PeriodOFFSET) ;
								TIM10->EGR = 1;
								TIM11->EGR = 1;
                MotorControl[3].PWM_Duty = 0;
                MotorControl[4].PWM_Duty = 0;
                MotorControl[3].Motor_Start_Stop = 0;
                MotorControl[4].Motor_Start_Stop = 0;
                MotorControl[3].Fault_Flag = 1;
                MotorControl[4].Fault_Flag = 1;
                OverFlow_Cnt[2] +=1;
            }
        }

    }
    if(GPIO_Pin == BRAKE4 )  //推杆
    {
        if( ((GPIOA->IDR>>10)&(0x1)) ==0)
        {

            if( ((GPIOA->IDR>>10)&(0x1)) ==0 )
            {
                MOTOR0STOP();
                MOTOR1STOP();
                MOTOR2STOP();
                MOTOR9STOP();
								TIM2->EGR = 1;
								TIM4->EGR = 1;
                MotorControl[0].Motor_Start_Stop = 0;
                MotorControl[1].Motor_Start_Stop = 0;
                MotorControl[2].Motor_Start_Stop = 0;
                MotorControl[9].Motor_Start_Stop = 0;
                MotorControl[0].Fault_Flag = 1;
                MotorControl[1].Fault_Flag = 1;
                MotorControl[2].Fault_Flag = 1;
                MotorControl[9].Fault_Flag = 1;
				 OverFlow_Cnt[3] +=1;
//                test0++;
            }
        }
    }
    if(GPIO_Pin == BRAKE5 )  //10 11 12 13
    {
        if( ((GPIOA->IDR>>9)&(0x1)) ==0 )
        {
//            for(u8 i=0; i<200; i++)
//            {};
            if( ((GPIOA->IDR>>9)&(0x1)) ==0 )
            {
                TIM3->CCR1= PWM_PERIOD+PWM_PeriodOFFSET;   //10
                TIM3->CCR2= PWM_PERIOD+PWM_PeriodOFFSET;   //11
                TIM8->CCR4 = 0;           //12
                TIM1->CCR4 = 0;  					//13
								TIM1->EGR = 1;
								TIM3->EGR = 1;
								TIM8->EGR = 1;
                MotorControl[10].PWM_Duty = 0;
                MotorControl[11].PWM_Duty = 0;
                MotorControl[12].PWM_Duty = 0;
                MotorControl[13].PWM_Duty = 0;
                MotorControl[10].Motor_Start_Stop = 0;
                MotorControl[11].Motor_Start_Stop = 0;
                MotorControl[12].Motor_Start_Stop = 0;
                MotorControl[13].Motor_Start_Stop = 0;
                MotorControl[10].Fault_Flag = 1;
                MotorControl[11].Fault_Flag = 1;
                MotorControl[12].Fault_Flag = 1;
                MotorControl[13].Fault_Flag = 1;
				 OverFlow_Cnt[4] +=1;

            }
        }

    }
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
    /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

    /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
    HAL_TIM_IRQHandler(&htim1);
    HAL_TIM_IRQHandler(&htim9);
    /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

    /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
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
//    /* USER CODE BEGIN USART3_IRQn 0 */

//    /* USER CODE END USART3_IRQn 0 */
//    HAL_UART_IRQHandler(&huart3);
//    /* USER CODE BEGIN USART3_IRQn 1 */

//    /* USER CODE END USART3_IRQn 1 */
//}



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

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

    /* USER CODE END DMA2_Stream0_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_adc1);
    /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

    /* USER CODE END DMA2_Stream0_IRQn 1 */
}

void CAN2_RX0_IRQHandler(void)
{
    /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

    /* USER CODE END CAN2_RX0_IRQn 0 */
    HAL_CAN_IRQHandler(&hcan2);
    /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

    /* USER CODE END CAN2_RX0_IRQn 1 */
}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
