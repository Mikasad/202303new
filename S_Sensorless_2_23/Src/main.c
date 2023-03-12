/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "canopen_od.h"
#include "canopen_timer.h"
#include "ds402.h"
#include "canopen_pdo.h"
#include "flash.h"
#include "Agreement.h"
#include "bsp_CAN.h"
#include "Sensorless bldc.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
MCU_Version ProgramVersion;
STRUCT_CAPTURE strCapture = { 0, 0, 0 };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;


IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
void Sys_Time(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
//static void MX_CAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_IWDG_Init(void);
static void MX_NVIC_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
u8 enableallmotor = 0, errormotor5 = 0, errormotor6 = 0;
uint8_t aRxBuffer[50];
uint32_t Err_codeHis[10],Err_codeTick[10];
int8_t OverFlow_Cnt[5];
uint32_t ADC_ConvertedValue[15];
uint32_t DMA_Transfer_Complete_Count=0;
u8 First_Enter_STUDY_MOTOR5 = 1;
u8 First_Enter_STUDY_MOTOR6 = 1;
int CH1TEMP = 0, CH2TEMP = 0;
SysCLC_TypeDef SysCLC;
SystStatus_t MotorState;
extern u8 change_temp;
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	SCB->VTOR = 0X8020000;30
	
//  __enable_irq();
  /* USER CODE END 1 */
	
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	
	#ifdef OTA_VERSION
    SCB->VTOR = 0X8020000;
	#endif

  /* USER CODE END 1 */
	
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	MX_DMA_Init();		/*传输ADC*/
  MX_ADC1_Init();	
 MX_ADC2_Init();  
  MX_CAN2_Init();
  MX_TIM1_Init();		/*无刷1PWM,brake，100us定时*/
  MX_TIM2_Init();		/*推杆0PWM*/
	MX_TIM3_Init();		/*风机输入捕获*/
  MX_TIM4_Init();		/*边刷，风机PWM*/
	MX_TIM5_Init();		/*无刷1效率优化*/
	TIM6_Init();			/*canopen定时器*/
	MX_TIM7_Init();			/* 计算无刷，边刷风机速度 */
  MX_TIM8_Init();			/*无刷2PWM*/
  MX_TIM9_Init();			/*推杆1PWM*/
  MX_TIM10_Init();		/*滚刷水泵*/
  MX_TIM11_Init();		/*边刷水泵*/
	MX_TIM12_Init();		/*无刷2效率优化*/
  MX_USART3_UART_Init();
	__HAL_USART_ENABLE_IT(&huart3,USART_IT_RXNE );    //开启串口中断处理函数
//  MX_IWDG_Init();
//	__HAL_IWDG_START(&hiwdg);	/* 启动独立看门狗 */
	MX_NVIC_Init();
	HAL_ADC_Start_DMA(&hadc1,ADC_ConvertedValue,15);
	HAL_ADCEx_InjectedStart(&hadc2);  
  Init_Drive_Para();
  Program_Version_Init(&ProgramVersion,'S',1,3,1);
  #ifdef OTA_VERSION
	Program_Version_Init(&ProgramVersion,'S',2,2,2);
  #endif
	HardVersion_Init(&ProgramVersion);
//	Flash_Read(ParaNum);
	Hardware_flowCheck();		//检查过流保护次数是否超限	2021.1125
//	CANopen_Parameter_Init(&CANopen_Drive);
	#ifdef OTA_VERSION
	__enable_irq();  
	#endif
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim3); 	
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim8);
	HAL_TIM_Base_Start_IT(&htim10);
    HAL_TIM_Base_Start_IT(&htim11);
	HAL_TIM_Base_Start_IT(&htim12);
	

	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_UART_Receive_IT(&huart3,aRxBuffer,1);
	
	HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
	CANopen_Parameter_Init(&CANopen_Drive);
	
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(CanLoadRate.timer >= 10000)
		{
				CanLoadRate.timer = 0;
				CanLoadRate.Rate = (CanLoadRate.FrameSize * (1000))/(obj2099_CanBandrate_Default*1000);
				CanLoadRate.FrameSize = 0;
		}
		if(enableallmotor == 1)			//使能所有电机
		{
				enableallmotor = 0;
				for(u8 i = 0 ; i <= 8; i++)
				{
						MotorControl[i].Fault_Flag = 0;
						MotorControl[i].Motor_Start_Stop = ENABLE;
						if(errormotor5 > 10)
						{
								MotorControl[5].Motor_Start_Stop = DISABLE;
						}
						if(errormotor6 > 10)
						{
								MotorControl[6].Motor_Start_Stop = DISABLE;
						}
				}

		}
		 if(wGlobal_Flags != prewGlobal_Flags)
		{
			static u32 filterwGlobal_Flags;
			if(wGlobal_Flags!=0&&wGlobal_Flags!=filterwGlobal_Flags)
			{
					for(u8 i=9;i>0;i--)													//add by diamond 2021.1125
					{
						Err_codeHis[i] = Err_codeHis[i-1];		//所有事件按顺序存入数组，清除最久的报警
					}
					for(u8 i=9;i>0;i--)
					{
						Err_codeTick[i] = Err_codeTick[i-1];		//事件的嘀嗒
					}		
					Err_codeHis[0] = wGlobal_Flags;
					Err_codeTick[0] = uwTick;
					Flash_Writesign =1;		//FLASH写入标志位  
					filterwGlobal_Flags = wGlobal_Flags;												
			}
			prewGlobal_Flags = wGlobal_Flags;
			sendPDOevent(&CANopen_Drive);									//错误码变化自动上报
		}
		if(BitTst(SysCLC.SysTimFlag,A1mSec))		/*1MS	*/
		{
				BitClr(SysCLC.SysTimFlag,A1mSec);
				BitInv(SysCLC.SysTimFlag,A2mSec);
				Sys_Time();

				if(BitTst(SysCLC.SysTimFlag,A2mSec))	/*2MS	*/
				{
//                Labview_uart();
						Startagreement();						/* 上位机通信 */
						QuickRead();								/* 自动上报数据 */
						if(strCapture.ucFinishFlag == 1)		/* 风机故障检测 */
						{
							Suction_motor_errhandle(strCapture.uDuty_ratio);
							strCapture.ucFinishFlag = 0;
						}
						Motor_Fault_Clear();
				}
		}


		if(BitTst(SysCLC.SysTimFlag,A10mSec))		/*10MS	*/
		{
				BitClr(SysCLC.SysTimFlag,A10mSec);
				BitInv(SysCLC.SysTimFlag,A20mSec);
//				DisplayErrLed();					/* 状态灯 */
				ReturnError1();						/* 自动上报错误 */
				if(BitTst(SysCLC.SysTimFlag,A20mSec))	/*20MS	*/
				{

				}
		}


		if(BitTst(SysCLC.SysTimFlag,A250mSec))	/*250MS	*/
		{
				Motor_Choose();
				BitClr(SysCLC.SysTimFlag,A250mSec);
				BitInv(SysCLC.SysTimFlag,A500mSec);

				if(MotorControl[5].Hall.HallState == 0||MotorControl[5].Hall.HallState == 7 )
				{
						MC_SetFault(HALL5_SENSOR_ERR);
						MotorControl[5].Motor_Start_Stop = DISABLE;
				}
				if(MotorControl[6].Hall.HallState == 0||MotorControl[6].Hall.HallState == 7 )
				{
//						MC_SetFault(HALL6_SENSOR_ERR);
//						MotorControl[6].Motor_Start_Stop = DISABLE;
				}
//						Flash_WriteCheck();
				SelfLock_Ctl();

		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
	
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
	
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
//  HAL_RCC_EnableCSS();
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/2000);                // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  
}

/*------------------------------------------------
Function:使用内部时钟16M
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
//void SystemClock_Config(void)
//{
//    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//    /** Configure the main internal regulator output voltage
//    */
//    __HAL_RCC_PWR_CLK_ENABLE();
//    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//    /** Initializes the RCC Oscillators according to the specified parameters
//    * in the RCC_OscInitTypeDef structure.
//    */
//    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//    RCC_OscInitStruct.PLL.PLLM = 16;
//    RCC_OscInitStruct.PLL.PLLN = 336;
//    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//    RCC_OscInitStruct.PLL.PLLQ = 4;
//    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    /** Initializes the CPU, AHB and APB buses clocks
//    */
//    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

//    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    // HAL_RCC_GetHCLKFreq()/1000    1ms?D??ò?′?
//    // HAL_RCC_GetHCLKFreq()/100000	 10us?D??ò?′?
//    // HAL_RCC_GetHCLKFreq()/1000000 1us?D??ò?′?
//    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/2000);                 // ????2￠???ˉ?μí3μ?′e?¨ê±?÷
//    /* ?μí3μ?′e?¨ê±?÷ê±?ó?′ */
//    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

//    /* ?μí3μ?′e?¨ê±?÷?D??ó??è?????? */
//    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

//}
/*------------------------------------------------
Function:中断优先级设置
Input   :No
Output  :No
Explain :特别注意：不要比嘀嗒定时器优先级高，否则在主机从机同时心跳代码断线功能失效，会卡在 if(Timeout != HAL_MAX_DELAY)
------------------------------------------------*/
static void MX_NVIC_Init(void)
{
    HAL_NVIC_SetPriority(SysTick_IRQn, 2, 0);
    /* EXTI interrupt init*/

		/* EXTI1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);  //边刷速度反馈
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	
    /* EXTI3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);  //推杆hall
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    /* EXTI4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI4_IRQn, 3, 0);  //电机6刹车
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    /* EXTI9_5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    /* EXTI15_10_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	
		/* TIM1_BRK_TIM9_IRQn interrupt configuration TIM1_UP_TIM10_IRQn */  //电机5刹车
    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);    //电机5刹车
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

    /* TIM3_IRQn interrupt configuration TIM3_IRQn */  //输入捕获
    HAL_NVIC_SetPriority(TIM3_IRQn,3, 0);
		HAL_NVIC_EnableIRQ(TIM3_IRQn);

    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	
	
	
	HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 1, 0);   //7.25新增
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

    HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0); //无刷测速 和电机7、8测速
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    /* USART3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    /* CAN2_RX0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,3, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		
		HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);

		HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
}
/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 15;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
//  sConfig.Channel = ADC_CHANNEL_5;
//  sConfig.Rank = 6;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
//  sConfig.Channel = ADC_CHANNEL_8;
//  sConfig.Rank = 8;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}



static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_MultiModeTypeDef multimode = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;    //原来是4
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = 1;
  hadc2.Init.EOCSelection = DISABLE;
  hadc2.Init.ContinuousConvMode = 1;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }  
  multimode.Mode =ADC_MODE_INDEPENDENT ;
//  if (HAL_ADCEx_MultiModeConfigChannel(&hadc2, &multimode) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_15;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE ;
  sConfigInjected.AutoInjectedConv = 0;//DISABLE ;
  sConfigInjected.ExternalTrigInjecConv =ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.ExternalTrigInjecConvEdge =ADC_EXTERNALTRIGINJECCONVEDGE_RISINGFALLING;//ADC_EXTERNALTRIGINJECCONVEDGE_RISINGFALLING;//ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;        
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_ADC_MspInit(&hadc2);
  __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_IT_JEOC);
   __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
}
/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 3125;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */
	
  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;    //1
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
//  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.Pulse = 50;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 200;		//200
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_4);

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
	
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8400;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */
	TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 20;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
	
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
	
  /* USER CODE END TIM3_Init 2 */
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8400;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}
static void MX_TIM5_Init(void)
{

    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
		TIM_OC_InitTypeDef sConfigOC;

    /* USER CODE BEGIN TIM5_Init 1 */

    /* USER CODE END TIM5_Init 1 */
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 83;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim5.Init.Period = 0xffffffff;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN TIM5_Init 2 */
		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);

		
		sConfigOC.OCMode = TIM_OCMODE_TIMING;   // 比较定时模式
		sConfigOC.Pulse = 0;                  // 比较模式下这是无意义的值
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	
    HAL_TIM_OC_ConfigChannel(&htim5,&sConfigOC,TIM_CHANNEL_1);
    /* USER CODE END TIM5_Init 2 */
		HAL_TIM_OC_Start_IT(&htim5,TIM_CHANNEL_1);
}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

    /* USER CODE BEGIN TIM7_Init 0 */

    /* USER CODE END TIM7_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM7_Init 1 */

    /* USER CODE END TIM7_Init 1 */
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 83;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 65535;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM7_Init 2 */

    /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim8.Init.Period = 8400;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 1;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM2;
//   sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
//  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
//  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = 200;		//200
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);
	
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 8400;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim9, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 199;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 16800;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 16800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 199;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 16800;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

static void MX_TIM12_Init(void)
{

    /* USER CODE BEGIN TIM12_Init 0 */

    /* USER CODE END TIM12_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
		TIM_OC_InitTypeDef sConfigOC;
    __HAL_RCC_TIM12_CLK_ENABLE();
    /* USER CODE BEGIN TIM12_Init 1 */

    /* USER CODE END TIM12_Init 1 */
    htim12.Instance = TIM12;
    htim12.Init.Prescaler = 0;   //83
    htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim12.Init.Period = 840;  //0xFFF
    htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	    if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
    {
        Error_Handler();
    }
//    if (HAL_TIM_OC_Init(&htim12) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    /* USER CODE BEGIN TIM12_Init 2 */
//		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//    HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig);

//    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//    HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig);

		
//		sConfigOC.OCMode = TIM_OCMODE_TIMING;   // 比较定时模式
//		sConfigOC.Pulse = 0;                  // 比较模式下这是无意义的值
//		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	
//    HAL_TIM_OC_ConfigChannel(&htim12,&sConfigOC,TIM_CHANNEL_1);
//    /* USER CODE END TIM12_Init 2 */
//		HAL_TIM_OC_Start_IT(&htim12,TIM_CHANNEL_1);
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SelfLock_Pin|Power_Ctrl_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SideBrush_F_R_Pin|SideBrush_BRAKE_Pin|Fan_EN_Pin|Fault_1D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Fault_1_GPIO_Port, Fault_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_STBY_GPIO_Port, CAN_STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Eleva1_Hall_1_Pin Eleva2_Hall_1_Pin BL2HOCP_Pin */
  GPIO_InitStruct.Pin = Eleva1_Hall_1_Pin|Eleva2_Hall_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = BL2HOCP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_EN_Pin */
  GPIO_InitStruct.Pin = POWER_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(POWER_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SelfLock_Pin Power_Ctrl_Pin */
  GPIO_InitStruct.Pin = SelfLock_Pin|Power_Ctrl_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SideBrush_F_R_Pin SideBrush_BRAKE_Pin Fan_EN_Pin Fault_1D4_Pin */
  GPIO_InitStruct.Pin = SideBrush_F_R_Pin|SideBrush_BRAKE_Pin|Fan_EN_Pin|Fault_1D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOD,SideBrush_F_R_Pin|SideBrush_BRAKE_Pin,GPIO_PIN_SET);

  /*Configure GPIO pins : Hall_U2_Pin Hall_V2_Pin Hall_W2_Pin */
  GPIO_InitStruct.Pin = Hall_U2_Pin|Hall_V2_Pin|Hall_W2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Fault_1_Pin */
  GPIO_InitStruct.Pin = Fault_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Fault_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Hall_U1_Pin Hall_V1_Pin Hall_W1_Pin */
  GPIO_InitStruct.Pin = SideBrush_FG_Pin|Hall_U1_Pin|Hall_V1_Pin|Hall_W1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	/*Configure GPIO pins : SideBrush_FG_Pin */
  GPIO_InitStruct.Pin = SideBrush_FG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_STBY_Pin */
  GPIO_InitStruct.Pin = CAN_STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_STBY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
    /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SelfLock_Pin|Power_Ctrl_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD,Fan_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Fault_1_GPIO_Port, Fault_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_STBY_GPIO_Port, CAN_STBY_Pin, GPIO_PIN_RESET);
  
//  FAN_OFF;
  
//   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_RESET);
}
u32 Cnt_100us=0;                   //100us计数
extern s32 OverZerotime;
s32 bldc_change_time=0;
s32 bldc_change_time1=0;
int32_t boot_charge5_6cnt[2] = {0};
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		s32 TempValue;
    if(htim==(&htim8))               //由TIM1中断改成TIM8中断
    {
       if(Sensorless[0].Turn_Into_Flag==1)
		{
            Sensorless[0].PWM_DutySet = PID_Regulator(MotorControl[5].Current.ADCValue_Ref,MotorControl[5].Current.DeepFilterVAL,&PID_Current_InitStructure[0]);  //2月17号加入电流环				
			if(Sensorless[0].PWM_DutySet>0)
			{
				if(Sensorless[0].PWM_DutySet<1000)
				{
					Sensorless[0].PWM_DutySet=1000;
				}
			}
			else 
			{
				if(Sensorless[0].PWM_DutySet>-1000)
			   {
				  Sensorless[0].PWM_DutySet=-1000;
			   }
			}
			Sensorless_ChangePwm(0);          //改变PWM的时候用       
		}			
		DMA_Transfer_Complete_Count++;
		CanLoadRate.timer++;
        if(RPDO_SYNC_SIGN & RPDO_SYNC_READY)
        {
            RPDO_SYNC_SIGN &= ~RPDO_SYNC_READY;
            _RPOD_SyncEvent(&CANopen_Drive,pRpdoDirectPar,1);
        }
				Push_Motor_Cheek();	//推杆，有刷3，4检测
        MotorStuckCheck();
				Motor5_Default_Phase_Cheek();//缺相检测
        Motor6_Default_Phase_Cheek();//缺相检测
        Cnt_100us++;
        if(Cnt_100us>4000000000)Cnt_100us=0;
				   
				/*CANopen Sync Data Handler Begin*/
				//Normal sync - 4us  RPDO sync - 13us
				//Think Can FIFO Rec time and here	
				
				/**PA1->CHANNAL1**/
				MotorControl[0].Current.GetADCValue = ADC_ConvertedValue[1]-MotorControl[0].Current.offset;
				if(MotorControl[0].Current.GetADCValue< 0 ) MotorControl[0].Current.GetADCValue = 0;
				TempValue = (s32)((MotorControl[0].Current.GetADCValue - MotorControl[0].Current.FilterValue )>>4);
				MotorControl[0].Current.FilterValue = (u16)(TempValue + MotorControl[0].Current.FilterValue);

				/**PA2->CHANNAL2**/
				MotorControl[1].Current.GetADCValue = ADC_ConvertedValue[2]-MotorControl[1].Current.offset;
				if(MotorControl[1].Current.GetADCValue < 0) MotorControl[1].Current.GetADCValue= 0;
				TempValue = (s32)((MotorControl[1].Current.GetADCValue - MotorControl[1].Current.FilterValue )>>4);
				MotorControl[1].Current.FilterValue = (u16)(TempValue + MotorControl[1].Current.FilterValue);

				/**3->PC0->CHANNAL10**/
				MotorControl[3].Current.GetADCValue = ADC_ConvertedValue[9]-MotorControl[3].Current.offset;
				if(MotorControl[3].Current.GetADCValue < 0) MotorControl[3].Current.GetADCValue = 0;
				TempValue = (s32)((MotorControl[3].Current.GetADCValue - MotorControl[3].Current.FilterValue )>>4);
				MotorControl[3].Current.FilterValue = (u16)(TempValue + MotorControl[3].Current.FilterValue);

				/**3temp->PC1->CHANNAL11**/
				MotorControl[3].Current.GetADC_Temp = ADC_ConvertedValue[10];
				if(MotorControl[3].Current.GetADC_Temp < 0) MotorControl[3].Current.GetADC_Temp = 0;
				TempValue = (s32)((MotorControl[3].Current.GetADC_Temp - MotorControl[3].Current.FilterTemp )>>4);
				MotorControl[3].Current.FilterTemp = (u16)(TempValue + MotorControl[3].Current.FilterTemp);
				

				/**4->PC2->CHANNAL12**/
				MotorControl[4].Current.GetADCValue = ADC_ConvertedValue[11]-MotorControl[4].Current.offset;   //修改
				if(MotorControl[4].Current.GetADCValue <0)  MotorControl[4].Current.GetADCValue= 0;
				TempValue = (s32)((MotorControl[4].Current.GetADCValue - MotorControl[4].Current.FilterValue )>>4);
				MotorControl[4].Current.FilterValue = (u16)(TempValue + MotorControl[4].Current.FilterValue);


				/**5->PA6->CHANNAL6**/
				MotorControl[5].Current.GetADCValue = ADC2->JDR4 - MotorControl[5].Current.offset;
				if(MotorControl[5].Current.GetADCValue <0 ) MotorControl[5].Current.GetADCValue = 0;
				TempValue = (s32)((MotorControl[5].Current.GetADCValue - MotorControl[5].Current.FilterValue )>>4);
				MotorControl[5].Current.FilterValue = (u16)(TempValue + MotorControl[5].Current.FilterValue);


				/**6->PC4->CHANNAL14**/
				MotorControl[6].Current.GetADCValue = ADC_ConvertedValue[13] - MotorControl[6].Current.offset;
				if(MotorControl[6].Current.GetADCValue < 0) MotorControl[6].Current.GetADCValue = 0;
				TempValue = (s32)((MotorControl[6].Current.GetADCValue - MotorControl[6].Current.FilterValue )>>4);
				MotorControl[6].Current.FilterValue = (u16)(TempValue + MotorControl[6].Current.FilterValue);

				/**7->PA3->CHANNAL3**/
				MotorControl[7].Current.GetADCValue = ADC_ConvertedValue[3]-MotorControl[7].Current.offset;
				if(MotorControl[7].Current.GetADCValue <0)  MotorControl[7].Current.GetADCValue = 0 ;
				TempValue = (s32)((MotorControl[7].Current.GetADCValue - MotorControl[7].Current.FilterValue )>>4);
				MotorControl[7].Current.FilterValue = (u16)(TempValue + MotorControl[7].Current.FilterValue);
				
				/**8->PA4->CHANNAL4**/
				MotorControl[8].Current.GetADCValue = ADC_ConvertedValue[4]-MotorControl[8].Current.offset;
				if(MotorControl[8].Current.GetADCValue <0)  MotorControl[8].Current.GetADCValue = 0 ;
				TempValue = (s32)((MotorControl[8].Current.GetADCValue - MotorControl[8].Current.FilterValue )>>4);
				MotorControl[8].Current.FilterValue = (u16)(TempValue + MotorControl[8].Current.FilterValue);
				/*深度滤波*/
				MotorControl[5].Current.DeepFilterVAL = (MotorControl[5].Current.FilterValue*FILTER_COEFFICIENT+(1-FILTER_COEFFICIENT)*MotorControl[5].Current.PreFilterVal);		
				MotorControl[6].Current.DeepFilterVAL = (MotorControl[6].Current.FilterValue*FILTER_COEFFICIENT+(1-FILTER_COEFFICIENT)*MotorControl[6].Current.PreFilterVal);
				MotorControl[5].Current.PreFilterVal = MotorControl[5].Current.DeepFilterVAL;
				MotorControl[6].Current.PreFilterVal = MotorControl[6].Current.DeepFilterVAL;
				
				
				 
				/*end*/
				VoltVar.AdBuf = ADC_ConvertedValue[0];
				TempValue = (s32)((VoltVar.AdBuf - VoltVar.BUS )>>4);
				VoltVar.BUS = (u16)(TempValue +VoltVar.BUS);
				if(DMA_Transfer_Complete_Count > 30000) //偏置电压校准，开始数据不稳定丢弃
				{
						if( MotorState == IDLE)
						{
								Voltage_offset_cali();
						}
				}
				
				if(MotorControl[5].BrulessMode==1)
				{
/***********************5号无刷下发速度*****************************/
					if(MotorControl[5].Speed_Real<0) MotorControl[5].Current.DeepFilterVAL = -MotorControl[5].Current.DeepFilterVAL;
					if(MotorControl[5].PWM_Duty>1000&&MotorControl[5].Hall.Hall_Changecnt !=0&&MotorControl[5].PWM_Duty<6600)
					{
						MotorControl[5].PWM_Duty = PWM_DutyControl(MotorControl[5].Hall.Hall_Changecnt,MotorControl[5].PWM_Duty);
						MotorControl[5].Hall.Hall_Changecnt--;
					}
					else 
						MotorControl[5].PWM_Duty = PID_Regulator(MotorControl[5].Current.ADCValue_Ref,MotorControl[5].Current.DeepFilterVAL,&PID_Current_InitStructure[0]);
					if((MotorControl[5].Speed_Ref < 0 && MotorControl[5].PWM_Duty > 0) || \
						(MotorControl[5].Speed_Ref >0 && MotorControl[5].PWM_Duty < 0) || MotorControl[5].Speed_Ref == 0 )
					{
							MotorControl[5].PWM_Duty = 0;
					}
					if(MotorControl[5].Motor_Start_Stop==ENABLE)
					{
							if(MotorControl[5].Fault_Flag == 0&&OverFlow_Cnt[0]<BREAK_CNT_MAX)
							{
								  boot_charge5_6cnt[0]++;
									if(boot_charge5_6cnt[0]>5)//充电完成
									{
											boot_charge5_6cnt[0] = 5;
											SetMotorSpeed(5, MotorControl[5].PWM_Duty);
									}
									else
									{
										if(MotorControl[5].Speed_Real==0)
										{
											TIM1->CCER = OPEN_ALL_DOWN_TUBE;//充电500us
										}
									}
										
							}
							else if(OverFlow_Cnt[0]>=BREAK_CNT_MAX)
							{
									MotorControl[5].Motor_Start_Stop = DISABLE;
									MC_SetFault(MOTOR5_BREAK);
							}
							else MotorControl[5].Motor_Start_Stop = DISABLE;
					}
					else if(MotorControl[5].Motor_Start_Stop==DISABLE)
					{
						SetMotorStop(5);			
						boot_charge5_6cnt[0] = 0;//清除充电计数
					}
/***********************5号无刷下发速度end*****************************/

/***********************6号无刷下发速度*****************************/
					if((MotorControl[6].Speed_Ref < 0 && MotorControl[6].PWM_Duty > 0) || \
						(MotorControl[6].Speed_Ref >0 && MotorControl[6].PWM_Duty < 0) || MotorControl[6].Speed_Ref == 0 )
					{
							MotorControl[6].PWM_Duty = 0;
					}
					else 
					{
						if(MotorControl[6].Speed_Real<0) MotorControl[6].Current.DeepFilterVAL = -MotorControl[6].Current.DeepFilterVAL;
						if(MotorControl[6].PWM_Duty>1000&&MotorControl[6].PWM_Duty<6600&&MotorControl[6].Hall.Hall_Changecnt !=0)
						{
							MotorControl[6].PWM_Duty = PWM_DutyControl(MotorControl[6].Hall.Hall_Changecnt,MotorControl[6].PWM_Duty);
							MotorControl[6].Hall.Hall_Changecnt--;
						}
						else 
						MotorControl[6].PWM_Duty = PID_Regulator(MotorControl[6].Current.ADCValue_Ref,MotorControl[6].Current.DeepFilterVAL,&PID_Current_InitStructure[1]);
					}
					if(MotorControl[6].Motor_Start_Stop==ENABLE)
					{
							if(MotorControl[6].Fault_Flag == 0&&OverFlow_Cnt[1]<BREAK_CNT_MAX)
									SetMotorSpeed(6, MotorControl[6].PWM_Duty);
							else if(OverFlow_Cnt[1]>=BREAK_CNT_MAX)
							{
									MotorControl[6].Motor_Start_Stop = DISABLE;
									MC_SetFault(MOTOR6_BREAK);
							}
							else MotorControl[6].Motor_Start_Stop = DISABLE;
					}
					else if(MotorControl[6].Motor_Start_Stop==DISABLE)
						SetMotorStop(6);							
				}
/***********************6号无刷下发速度end*****************************/
    }

    if(htim==(&htim7))
    {
        HALL_OVF_Counter++;
        if(HALL_OVF_Counter%4 == 0)
        {
            if(MotorControl[8].Motor_Start_Stop == ENABLE)
            {
				if(CH1TEMP>5||CH1TEMP==0)		//exclude error state
				{
                   MotorControl[8].Speed_Real  = CH1TEMP*228.88;	//计算速度
				}
				else
				{
					MotorControl[8].Speed_Real  =0;
				}
                CH1TEMP = 0;
            }

            if(MotorControl[7].Motor_Start_Stop == ENABLE)
            {
                MotorControl[7].Speed_Real  = CH2TEMP*228.88;		//计算速度
                CH2TEMP = 0;
            }
        }
    }
		else if(htim ==&htim3)
		{
			strCapture .usPeriod ++;
			if(strCapture .usPeriod>8&&((GPIOB->IDR>>4)&(0x1)) ==1)
			{
				if(MotorControl[8].Motor_Start_Stop==ENABLE)
				{
					strCapture .usPeriod = 0;						
					strCapture .ucStartFlag = 0;			
					strCapture .ucFinishFlag = 1;
					strCapture.uDuty_ratio = 100;			
				}					
			}
		}
		else if(htim ==&htim12)        //0.01ms
		{
			bldc_change_time++;
			if(Sensorless[0].Speed_Set>0)
			{
				if(MotorControl[5].BrulessMode==0)
				{
//		          MotorState=Senless_start;
				  Sensorless_Start();
				}
			}
			if((Sensorless[0].Speed_Set==0)&&(MotorControl[5].BrulessMode==0))
			{
//		          MotorState=3;
				  Sensorless[0].State=0;
				  Sensorless[0].CountSectorCnt3=300;
				  Sensorless[0].PWM_DutySet =1000;
				  Sensorless[0].PWM_Duty=1000;
				  Sensorless[0].Turn_Into_Closed_Cnt=0;
				  Sensorless[0].Turn_Into_Flag=0;
				  Sensorless[0].Speed_Real=0;
				  Sensorless_Mototr_Stop();
			}
//			if((MotorState==RUN)||(Sensorless[0].Speed_Set==0))
//			{
//				Sensorless[0].State=0;
//				Sensorless[0].CountSectorCnt=0;
//				Sensorless_Mototr_Stop();
//			}
            	OverZerotime++;			
		}
}

/*------------------------------------------------
Function:刹车
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
    if(htim == (&htim1))
    {
        CanLoadRate.timer++;
        if( ((GPIOE->IDR>>15)&(0x1)) ==0 )		/* 硬件过流 */
        {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            TIM1->CCR3 = 0;
			TIM1->EGR = 1;
            MotorControl[5].Motor_Start_Stop = 0;
            errormotor5++;
            MotorControl[5].Fault_Flag = 1;
            MC_SetFault(MOTOR5_BREAK);
			OverFlow_Cnt[0] +=1;
			MotorState=0x03;
        }
        else
        {
        }
    }
}
/**
  * @brief HAL_TIM_IC_CaptureCallback·??ú2a?ù
  * @param None
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
		TIM_IC_InitTypeDef sConfigIC;
    if(htim==(&htim3))
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
			if ( strCapture .ucStartFlag == 0 )
			{     
				__HAL_TIM_SET_COUNTER(htim,0); // clear timer cnt;
				strCapture .usPeriod = 0;			
				strCapture .usCtr = 0;
				
				//config  rise->fall
				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, TIM_CHANNEL_1);
				// clear IT
				__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
				// start IT IC
				HAL_TIM_IC_Start_IT(htim,TIM_CHANNEL_1);    
				strCapture .ucStartFlag = 1;	
				CH1TEMP = CH1TEMP+1;							
			}		
			else
			{
				// get timer cnt
				strCapture .usCtr = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
				// config  fall->rise
				sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
				sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
				sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
				sConfigIC.ICFilter = 0;
				HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, TIM_CHANNEL_1);
				
				// clear IT
				__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1); 
				// start IT IC
				HAL_TIM_IC_Start_IT(htim,TIM_CHANNEL_1);    
				strCapture .ucStartFlag = 0;			
				strCapture .ucFinishFlag = 1;   
				strCapture .uPulseCnt = strCapture .usPeriod * 65535 + strCapture .usCtr;
//							strCapture.uDuty_ratio = strCapture .uPulseCnt*(TIM9_PRESCALER+1)*1000*(100/MOTOR8_PWM_PERIOD)/((HAL_RCC_GetPCLK2Freq()<<1));		//50ms周期的占空比
				strCapture.uDuty_ratio = strCapture .uPulseCnt/(40*MOTOR8_PWM_PERIOD);		//用上面公式数据会溢出
			}
        }

    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim5)
	{
//		BLDC_Hall_Handle(5);
	}
//	else if(htim==&htim12)
//	{
//		BLDC_Hall_Handle(6);
//	}
}

/* USER CODE BEGIN 4 */

void HAL_SYSTICK_Callback(void)
{ 
    if(HAL_GetTick()%2 == 0)  //1MS
    {			
		BLDC_Stuck_Chk();	//堵转保护500ms报警
        BitSet(SysCLC.SysTimFlag,A1mSec);
    }
    if(HAL_GetTick()%4 == 0)  //2MS
    {
		if(Sensorless[0].Turn_Into_Flag==1)
		{
			MotorControl[5].Current.ADCValue_Ref = PID_Regulator(Sensorless[0].Speed_Set,Sensorless[0].Speed_Real,&Sensorless_PID_PWM); 
		}
        switch (MotorState)
        {
        case IDLE:

//            MotorState = INIT;		偏置电压修正后进入INIT

            break;

        case INIT:
						VOTAGE_ON;
						MotorState = START; //标定成功再进入下一个状态，再使能CAN通信

            break;

        case START:

            MotorState = RUN;

            break;

        case RUN:   // motor running
//			 MotorControl[5].Motor_Start_Stop=1;
//              MotorControl[5].PercentOfPWM=100;
            for(u8 i=0; i < MOTOR_NUM; i++)
            {
			 if(MotorControl[i].Push_motor_calibrationFLAG == 1)
                {
                    MotorControl[i].Motor_Start_Stop = ENABLE;
                }
                if(MotorControl[i].Motor_Start_Stop == ENABLE)			/* 硬件过流次数不能超限，否则不能使能 */
                {
					switch (i)
					{
						case 5:
							if(MotorControl[5].BrulessMode==1)
							{
								Ramp_Speed(5);
								MotorControl[5].Speed_Real = GetMotorSpeed(5);
								BLDC1_OverSpdChk();
								MotorControl[5].Current.ADCValue_Ref = PID_Regulator(MotorControl[5].Speed_Ref,MotorControl[5].Speed_Real,&PID_Speed_InitStruct[0]);
							}
							break;
						case 6:
							Ramp_Speed(6);
							MotorControl[6].Speed_Real = GetMotorSpeed(6);
							BLDC2_OverSpdChk();
							MotorControl[6].Current.ADCValue_Ref = PID_Regulator(MotorControl[6].Speed_Ref,MotorControl[6].Speed_Real,&PID_Speed_InitStruct[1]);
							break;
						case 3:
						case 4:
							if(MotorControl[i].Fault_Flag == 0)
									SetMotorSpeed(i, MotorControl[i].PWM_Duty);
							else MotorControl[i].Motor_Start_Stop = DISABLE;
							break;
						case 0:
						case 1:
							if(MotorControl[5].Current.SetValue!=0&&MotorControl[0].Push_motor_calibrationFLAG <2)
							{
								MC_SetFault(PUSHMOTOR_INT_ERROR);
							}
							if(MotorControl[i].Push_motor_calibrationFLAG == 1)    //每次标定完毕会将该标志位置0
							{
									Push_Motor_Calibrate(i);
							}
							else if(MotorControl[i].Fault_Flag == 0)
									SetMotorSpeed(i, MotorControl[i].PWM_Duty);
							else MotorControl[i].Motor_Start_Stop = DISABLE;
							if(HAL_GetTick()%20 == 0) //10ms位置环
							{
									Push_Motor_Location_Control(i);		/* 推杆位置环 */
							}
							break;
						default:
							if(MotorControl[i].Fault_Flag == 0)
							{
									SetMotorSpeed(i, MotorControl[i].PWM_Duty);
							}
							else
							{
									MotorControl[i].Motor_Start_Stop = DISABLE;
							}
							break;
					}
                    
                }
                else if(MotorControl[i].Motor_Start_Stop == DISABLE)
                {
                    SetMotorStop(i);
                }
            }

            if(HAL_GetTick()%20 == 0) //10ms位置环
            {
								if(Push_Type==0)
				{
				    Push_Motor_CurSelfAdapt(5,0,190,30);		//推杆高度自适应
				}
				else if(Push_Type==1)
				{
					Push_Motor_CurSelfAdapt(5,0,420,30);		//推杆高度自适应
				}
            }
			if(HAL_GetTick()%200 == 0)
			{	
					Motor7_Err_Chk();		//边刷断连检测
			}

            break;

        case STOP1:    // motor stopped

            MotorState = WAIT;

            break;

        case WAIT:    // wait MotorState

            break;

        case FAULT:

            if(wGlobal_Flags == 0)
            {
                MotorState = IDLE;
            }

            break;

        case HALL_STUDY_MOTOR5:

            if(First_Enter_STUDY_MOTOR5 == 1)
            {
                TIM1->CCR1 = 0;
                TIM1->CCR2 = 0;
                TIM1->CCR3 = 0;
                for(u16 i = 0; i < 10000; i++) {}
                First_Enter_STUDY_MOTOR5 = 0;
            }
            HallStudyHandle0();

            if(HALL_Study[0].CommuntionState == 3 )
            {
                HALL_Study[0].CommuntionState = 0;
                HALL_Study[0].StudySectorCnt = 0;
                TIM1->CCR1 = 0;
                TIM1->CCR2 = 0;
                TIM1->CCR3 = 0;
                if ( (wGlobal_Flags & HALL5_SENSOR_ERR) == 0 )
                {
                    MotorState = IDLE;
                }
                else
                {
                    MotorState = FAULT;
                }
                First_Enter_STUDY_MOTOR5 = 1;
            }
            break;
        case HALL_STUDY_MOTOR6:
            if(First_Enter_STUDY_MOTOR6 == 1)
            {
                TIM8->CCR1 = 0;
                TIM8->CCR2 = 0;
                TIM8->CCR3 = 0;
                for(u16 i = 0; i < 10000; i++) {}
                First_Enter_STUDY_MOTOR6 = 0;
            }
            HallStudyHandle1();
            if(HALL_Study[1].CommuntionState == 3)
            {
                HALL_Study[1].CommuntionState = 0;
                HALL_Study[1].StudySectorCnt = 0;
                TIM8->CCR1 = 0;
                TIM8->CCR2 = 0;
                TIM8->CCR3 = 0;
                if ((wGlobal_Flags & HALL6_SENSOR_ERR) == 0 )
                {
                    MotorState = IDLE;
                }
                else
                {
                    MotorState = FAULT;
                }
                First_Enter_STUDY_MOTOR6 = 1;
            }
            break;
		case Senless_start:
		   break;

        default:
            break;
        }
    }
    if(HAL_GetTick()%20 == 0) //10ms
    {
        Over_VoltageCheck(); //过压保护
				Current_Real_Trans();		//电流转化
    }
    if(HAL_GetTick()%1000 == 0) //500ms喂狗
    {
        HAL_IWDG_Refresh(&hiwdg);
				Push_OverRunChk(1,10,1,12);		//推杆动作时间限制
    }
}

void Sys_Time(void)
{
    SysCLC.MilliSec++;
    if((SysCLC.MilliSec%10)==0)//20MS
    {
        BitSet(SysCLC.SysTimFlag,A10mSec);
    }

    if(SysCLC.MilliSec >= 250)		//250MS
    {
        SysCLC.MilliSec = 0;
        BitSet(SysCLC.SysTimFlag,A250mSec);
        SysCLC.SecondCntr++;		//1????êy?÷
    }

    if(SysCLC.SecondCntr >= 4)		//1000MS = 1S
    {
        SysCLC.SecondCntr = 0;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
