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
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_BDCMotor.h"
#include "canopen_od.h"
#include "canopen_timer.h"
#include "ds402.h"
#include "canopen_pdo.h"
#include "bsp_iwdg.h"
#include "Agreement.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CanTxMsgTypeDef TxMessage;
CanRxMsgTypeDef RxMessage;
MCU_Version ProgramVersion;
uint8_t Rx_flag = 0;
STRUCT_CAPTURE strCapture = { 0, 0, 0 };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
void Sys_Time(void);
void TIM1_TIM8_StartTimers(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
//static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
//static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);
void Labview_uart(void);
/* USER CODE BEGIN PFP */
uint32_t ADC_ConvertedValue[12];
uint32_t DMA_Transfer_Complete_Count=0;
uint8_t aRxBuffer[50];
SysCLC_TypeDef SysCLC;
SystStatus_t MotorState;
int CH1TEMP = 0, CH2TEMP = 0;
u8 First_Enter_STUDY_MOTOR5 = 1;
u8 First_Enter_STUDY_MOTOR6 = 1;
u16 Uart_TxBuffer_CNT = 0,Uart_TxBuffer_CNT1 = 0;
#define LV_TxBufnum 2010
int16_t LV_Uart_TxBuffer[4][LV_TxBufnum] = {0};
u8 motorset=0;
uint32_t Err_codeHis[10],Err_codeTick[10];
uint8_t OverFlow_Cnt[5];
/* 将编译信息写到固件指定地址*/												
const char Compiler_Date[] __attribute__((at(0x8020000 + 0x200))) = "H4AA30" ;
/* USER CODE END PFP */
/*------------------------------------------------
Function:CAN接受中断函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
pFunction Jump_To_Application;
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
    {
        if(CAN2->RF0R & (1<<4))//清除CAN接受标志
        {
            CAN2->RF0R &= (0<<4);
        }
        if(CANopen_Drive.CanRx_Buffer->IDE == BIT_INVALID)
        {
            CanLoadRate.FrameSize = CanLoadRate.FrameSize + (47 + CANopen_Drive.CanRx_Buffer->DLC * 8);
            MessageType_Check(&CANopen_Drive,CANopen_Drive.CanRx_Buffer->StdId,CANopen_Drive.CanRx_Buffer->RTR);//接收报文解码
        }
        else
        {}
        HAL_CAN_Receive_IT(hcan, CAN_FIFO0);//打开CAN中断
    }
}
/*------------------------------------------------
Function:CAN发送函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
uint8_t CAN1_Send_Msg(uint32_t stdId,uint32_t Ide,uint32_t Rtr,uint8_t* msg,uint8_t len)
{
    uint16_t i=0;
    hcan2.pTxMsg->StdId=stdId;
    hcan2.pTxMsg->ExtId=0xFFF;
    hcan2.pTxMsg->IDE=Ide;
    hcan2.pTxMsg->RTR=Rtr;
    hcan2.pTxMsg->DLC=len;
    for(i=0; i<len; i++)
    {
        hcan2.pTxMsg->Data[i]=msg[i];
    }
//    if(HAL_CAN_Transmit(&hcan2,20)!=HAL_OK)
////		{
////			pHeartBeatPar.Timeout_server = 0;
////		}
//        CanLoadRate.FrameSize = CanLoadRate.FrameSize + (47 + hcan2.pTxMsg->DLC * 8);
//    return i != 3? 0:1;
	    for(i=0; HAL_CAN_Transmit(&hcan2,10)!=HAL_OK && i<3; i++) {}
    if(i>=3)
    {
    }
    CanLoadRate.FrameSize = CanLoadRate.FrameSize + (47 + hcan2.pTxMsg->DLC * 8);
    return i != 3? 0:1;
}
u8 enableallmotor = 0;
u8 errormotor5 = 0;
u8 errormotor6 = 0;
u32 whiletest = 0;
int main(void)
{
    /* USER CODE BEGIN 1 */
   	SCB->VTOR = 0X8020000;
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
    MX_DMA_Init();
    MX_ADC1_Init();
	MX_ADC2_Init();
    MX_CAN2_Init();
    MX_TIM1_Init();			/* 无刷5 单向有刷13*/
	MX_TIM8_Init();			/* 无刷6 单向有刷12*/
    MX_TIM2_Init();			/* 0_1号推杆 */
    MX_TIM3_Init();			/* 边刷7 风机8 单向有刷10_11*/
    MX_TIM4_Init();			/* 2_9号推杆 */
    TIM6_Init();				/* can通信 */
    // MX_TIM6_Init();
    MX_TIM7_Init();			/* 定时器计算速度 */   
    MX_TIM9_Init();			/* 输入捕获 */
    MX_TIM10_Init();		/* 单向有刷电磁阀3 */
    MX_TIM11_Init();		/* 单向有刷电磁阀4 */
    MX_USART3_UART_Init();
//    __HAL_USART_ENABLE_IT(&huart3,USART_IT_RXNE );    //开启串口中断处理函数
    /* USER CODE BEGIN 2 */
    MX_IWDG_Init(IWDG_PRESCALER_64,3125);  //5S
    __HAL_IWDG_START(&hiwdg);	/* 启动独立看门狗 */

    MX_NVIC_Init();

    HAL_ADC_Start_DMA(&hadc1,ADC_ConvertedValue,12);
    Init_Drive_Para();
    Program_Version_Init(&ProgramVersion,'5',2,2,5);    //软件版本1.0.2   //1.0.3 见Readme
    HardVersion_Init(&ProgramVersion,'H','A',30,2);
    Flash_Read(ParaNum);
    Hardware_flowCheck();		//检查过流保护次数是否超限	2021.1125
	__enable_irq();  
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim10);
    HAL_TIM_Base_Start_IT(&htim11);
    HAL_ADCEx_InjectedStart(&hadc2); 
	HAL_ADCEx_InjectedStart(&hadc3); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);



    /* USER CODE END 2 */

    CANopen_Parameter_Init(&CANopen_Drive);
    HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
    HAL_UART_Receive_IT(&huart3,aRxBuffer,1);

    __HAL_TIM_CLEAR_IT(&htim9, TIM_IT_CC1);
    __HAL_TIM_CLEAR_IT(&htim9, TIM_IT_CC2);
    HAL_TIM_IC_Start_IT(&htim9,TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim9,TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	TIM1_TIM8_StartTimers();

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        whiletest++;
        if(CanLoadRate.timer >= 10000) //总线负载率计算
        {
            CanLoadRate.timer = 0;
            CanLoadRate.Rate = (CanLoadRate.FrameSize * (1000))/(obj2099_CanBandrate_Default*1000);
            CanLoadRate.FrameSize = 0;
        }
        if(enableallmotor == 1)			//使能所有电机
        {
            enableallmotor = 0;
            for(u8 i = 0 ; i <= 13; i++)
            {
                MotorControl[i].Fault_Flag = 0;
                MotorControl[i].Motor_Start_Stop = 1;
                if(errormotor5 > 10)
                {
                    MotorControl[5].Motor_Start_Stop = 0;
                }
                if(errormotor6 > 10)
                {
                    MotorControl[6].Motor_Start_Stop = 0;
                }
            }
        }

        if(wGlobal_Flags != prewGlobal_Flags)//错误发生改变
        {
            static u32 filterwGlobal_Flags;
            if(wGlobal_Flags!=0&&wGlobal_Flags!=filterwGlobal_Flags)
            {
                for(u8 i=9; i>0; i--)                        //add by diamond 2021.1125
                {
                    Err_codeHis[i] = Err_codeHis[i-1];    //所有事件按顺序存入数组，清除最久的报警
                }
                for(u8 i=9; i>0; i--)
                {
                    Err_codeTick[i] = Err_codeTick[i-1];    //事件的嘀嗒
                }
                Err_codeHis[0] = wGlobal_Flags;
                Err_codeTick[0] = uwTick;
//                Flash_Writesign =1;    //FLASH写入标志位
                filterwGlobal_Flags = wGlobal_Flags;
            }
            prewGlobal_Flags = wGlobal_Flags;
            sendPDOevent(&CANopen_Drive);                  //错误码变化自动上报
        }
        if(BitTst(SysCLC.SysTimFlag,A1mSec))		/*1MS	*/
        {
            BitClr(SysCLC.SysTimFlag,A1mSec);
            BitInv(SysCLC.SysTimFlag,A2mSec);
            Sys_Time();

            if(BitTst(SysCLC.SysTimFlag,A2mSec))	/*2MS	*/
            {

                Labview_uart();
//                Startagreement();						/* 上位机通信 */
//                QuickRead();								/* 自动上报数据 */
                if(strCapture.ucFinishFlag == 1)		/* 风机故障检测 */
                {
//                    Suction_motor_errhandle(strCapture.uDuty_ratio);
                    strCapture.ucFinishFlag = 0;
                }
                Motor_Fault_Clear();//错误清除
//								else if(motorset==1)
//								{
//									FAN_F_R_SET;
//								}
//								else if(motorset==2)
//									FAN_F_R_RESET;
            }
        }


        if(BitTst(SysCLC.SysTimFlag,A10mSec))		/*10MS	*/
        {
            BitClr(SysCLC.SysTimFlag,A10mSec);
            BitInv(SysCLC.SysTimFlag,A20mSec);
            DisplayErrLed();					/* 状态灯 */
//            ReturnError1();						/* 自动上报错误 */

            if(BitTst(SysCLC.SysTimFlag,A20mSec))	/*20MS	*/
            {

            }
        }
        if(BitTst(SysCLC.SysTimFlag,A250mSec))	/*250MS	*/
        {
			Motor_Choose();
            BitClr(SysCLC.SysTimFlag,A250mSec);
            BitInv(SysCLC.SysTimFlag,A500mSec);
           	if(MotorControl[5].Hall.HallState == 0||MotorControl[5].Hall.HallState == 7 )    //电机霍尔增加滤波
			{
				MotorControl[5].Hall_Error_Cnt++;
				if(MotorControl[5].Hall_Error_Cnt>=3)
				{
					MC_SetFault(HALL5_SENSOR_ERR);
					MotorControl[5].Motor_Start_Stop = DISABLE;
					MotorControl[5].Hall_Error_Cnt=0;
				}
			}
			else
			{
				if(MotorControl[5].Hall_Error_Cnt>0)
				{
					MotorControl[5].Hall_Error_Cnt--;
				}
			}
			if(MotorControl[6].Hall.HallState == 0||MotorControl[6].Hall.HallState == 7 )
			{
				MotorControl[6].Hall_Error_Cnt++;
				if(MotorControl[6].Hall_Error_Cnt>=3)
				{
					MC_SetFault(HALL6_SENSOR_ERR);
					MotorControl[6].Motor_Start_Stop = DISABLE;
					MotorControl[6].Hall_Error_Cnt=0;
				}
			}
			else
			{
				if(MotorControl[6].Hall_Error_Cnt>0)
				{
					MotorControl[6].Hall_Error_Cnt--;
				}
			}
				
        }
    }
    /* USER CODE END 3 */
}

/*------------------------------------------------
Function:使用外部时钟25M
Input   :NoOutput  :No
Explain :No
------------------------------------------------*/
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
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
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

    // HAL_RCC_GetHCLKFreq()/1000    1ms?D??ò?′?
    // HAL_RCC_GetHCLKFreq()/100000	 10us?D??ò?′?
    // HAL_RCC_GetHCLKFreq()/1000000 1us?D??ò?′?
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/2000);                 // ????2￠???ˉ?μí3μ?′e?¨ê±?÷
    /* ?μí3μ?′e?¨ê±?÷ê±?ó?′ */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* ?μí3μ?′e?¨ê±?÷?D??ó??è?????? */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

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
Explain :特别注意：CAN不要比嘀嗒定时器优先级高，否则在主机从机同时心跳代码断线功能失效，会卡在 if(Timeout != HAL_MAX_DELAY)
------------------------------------------------*/
static void MX_NVIC_Init(void)
{
    HAL_NVIC_SetPriority(SysTick_IRQn, 2, 0);   // 2  0     3   0
    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);  //推杆hall
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

//    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0); //模拟看门狗
//    HAL_NVIC_EnableIRQ(ADC_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);  //推杆hall
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    /* EXTI2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);  //推杆hall
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    /* EXTI3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);  //推杆hall
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    /* EXTI4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);  //电机6刹车
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    /* EXTI9_5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);     //1  0    1  1
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    /* EXTI15_10_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0); //1 0   1   1
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


    /* TIM1_BRK_TIM9_IRQn interrupt configuration TIM1_UP_TIM10_IRQn */  //电机5刹车
    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);    //电机5刹车
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 1);  //1 1           2   1
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0); //无刷测速 和电机7、8测速
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    /* USART3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART3_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    /* DMA2_Stream0_IRQn interrupt configuration */
//    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
//    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    /* CAN2_RX0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,3, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}
/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    ADC_AnalogWDGConfTypeDef AnalogWDGConfig;
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 12;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    sConfig.Channel = ADC_CHANNEL_0;  //电机3电流采样
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_1;   //电机4电流采样
    sConfig.Rank = 2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_2;  //电机5电流采样
    sConfig.Rank = 3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_2;    //ADC_CHANNEL_3 电机5温度
    sConfig.Rank = 4;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_4;   //电机6电流采样
    sConfig.Rank = 5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
//    sConfig.Channel = ADC_CHANNEL_5;
//    sConfig.Rank = 5;
//    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_4; //ADC_CHANNEL_6  电机6温度
    sConfig.Rank = 6;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */

//    sConfig.Channel = ADC_CHANNEL_8;
//    sConfig.Rank = 7;
//    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }

//    sConfig.Channel = ADC_CHANNEL_9;
//    sConfig.Rank = 8;
//    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }

    sConfig.Channel = ADC_CHANNEL_10;  //推杆1电流采样
    sConfig.Rank = 7;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_11;//推杆2电流采样
    sConfig.Rank = 8;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_12; //推杆3电流采样
    sConfig.Rank = 9;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_13;  //推杆4电流采样
    sConfig.Rank = 10;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_14;     //母线电压采集
    sConfig.Rank = 11;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */

    sConfig.Channel = ADC_CHANNEL_14; //ADC_CHANNEL_15 为风机温度
    sConfig.Rank = 12;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    ADC1->CR1 |= 0x06000 ;
    AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_ALL_REG;
    AnalogWDGConfig.HighThreshold =ADC1AnalogWDGHighThreshold; /* 由于无刷ADC采样值最大，所以看门狗对应按照无刷电机电流采样保护，126对应1A  设置为3500在27A左右 */
    AnalogWDGConfig.LowThreshold = 0;  /* 设置下限值为0.41V   4095*1/8=511  3.3/4095*511=0.41V*/
    AnalogWDGConfig.ITMode = ENABLE;
    if(HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig)!=HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_ADC2_Init(void)
{
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_MultiModeTypeDef multimode = {0};
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
//  multimode.Mode =ADC_MODE_INDEPENDENT ;
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;//无刷1五号电机检测
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;//ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE ;
  sConfigInjected.AutoInjectedConv = 0;//DISABLE ;
  sConfigInjected.ExternalTrigInjecConv =ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge =ADC_EXTERNALTRIGINJECCONVEDGE_RISINGFALLING;//ADC_EXTERNALTRIGINJECCONVEDGE_RISINGFALLING;//ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;//adcc通道 电机6
  sConfigInjected.InjectedRank = 2;
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
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
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
	 TIM_SlaveConfigTypeDef sSlaveConfig; //定时器级联
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period = 8400;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;  //TIM1与	TIM8同步
	sSlaveConfig.InputTrigger = TIM_TS_ITR1;
	if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;//TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCPOLARITY_LOW;
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
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 200;
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

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    /* USER CODE END TIM1_Init 2 */
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
    if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
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
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 0;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 0;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 0;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 8400;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.Pulse = 8400;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; //边刷
    if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; //风机
    if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);

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

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 8400;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
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
    if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 0;
    if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse =0;
    if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.Pulse = 0;
    if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
    HAL_TIM_MspPostInit(&htim4);

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
    TIM_SlaveConfigTypeDef sSlaveConfig; //定时器级联
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
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;  //TIM1和TIM8同步
    sSlaveConfig.InputTrigger = TIM_TS_ITR1;
    if (HAL_TIM_SlaveConfigSynchronization(&htim8, &sSlaveConfig) != HAL_OK)
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
    sConfigOC.OCNPolarity = TIM_OCPOLARITY_LOW;
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
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
    sBreakDeadTimeConfig.DeadTime = 200;//200
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
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_IC_InitTypeDef sConfigIC = {0};

    /* USER CODE BEGIN TIM9_Init 1 */

    /* USER CODE END TIM9_Init 1 */
    htim9.Instance = TIM9;
    htim9.Init.Prescaler = TIM9_PRESCALER;
    htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim9.Init.Period = 0xFFFF;
    htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;//选择内部时钟
    HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim9, &sMasterConfig);

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM9_Init 2 */

    /* USER CODE END TIM9_Init 2 */

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
    htim10.Init.Prescaler = 0;
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


    /* USER CODE BEGIN TIM10_Init 0 */

    /* USER CODE END TIM10_Init 0 */

    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM10_Init 1 */

    /* USER CODE END TIM10_Init 1 */
    htim11.Instance = TIM11;
    htim11.Init.Prescaler = 0;
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
    sConfigOC.Pulse = 16800;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM10_Init 2 */

    /* USER CODE END TIM10_Init 2 */
    HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_USART3_UART_Init(void)
//{

//    /* USER CODE BEGIN USART3_Init 0 */

//    /* USER CODE END USART3_Init 0 */

//    /* USER CODE BEGIN USART3_Init 1 */

//    /* USER CODE END USART3_Init 1 */
//    huart3.Instance = USART3;
//    huart3.Init.BaudRate = 115200;
//    huart3.Init.WordLength = UART_WORDLENGTH_8B;
//    huart3.Init.StopBits = UART_STOPBITS_1;
//    huart3.Init.Parity = UART_PARITY_NONE;
//    huart3.Init.Mode = UART_MODE_TX_RX;
//    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//    if (HAL_UART_Init(&huart3) != HAL_OK)
//    {
//        Error_Handler();
//    }
//    /* USER CODE BEGIN USART3_Init 2 */

//    /* USER CODE END USART3_Init 2 */

//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    HAL_UART_Receive_IT(&huart3,aRxBuffer,1);

//}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

//  OneFilter(MotorControl[0].Current.FilterValue,ADC_ConvertedValue[4],3);
//  OneFilter(MotorControl[1].Current.FilterValue,ADC_ConvertedValue[5],3);
//  OneFilter(MotorControl[2].Current.FilterValue,ADC_ConvertedValue[12],3);
//  OneFilter(MotorControl[3].Current.FilterValue,ADC_ConvertedValue[13],3);
//  OneFilter(MotorControl[4].Current.FilterValue,ADC_ConvertedValue[6],3);
//  if(DMA_Transfer_Complete_Count%2==0)
//  {
//    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);ADC_ConvertedValue
//  }
//  else
//  {
//    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
//  }
}
u32 Cnt_100us=0;                   //100us计数
float DeepFilterValPer_A[2] = {1.08,1.14};
s16 MotorControl5_Current_DeepFilterVAL,MotorControl6_Current_DeepFilterVAL;
int32_t int32test = 0;
int16_t ADeepFilter=0;
int16_t BDeepFilter=0;
int32_t boot_charge5_6cnt[2] = {0};
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    s32 TempValue;

    if(htim==(&htim1))
    {
        
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
    if(htim ==&htim9)
    {
        strCapture .usPeriod ++;
    }
}
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	s32 TempValue;
	 if(hadc->Instance==ADC2)
	 {
//		RED_LED2_TOGGLE;
        MotorControl[5].Current.GetADCValue = ADC2->JDR1 - MotorControl[5].Current.offset;
        if(MotorControl[5].Current.GetADCValue <0 ) MotorControl[5].Current.GetADCValue = 0;
        TempValue = (s32)((MotorControl[5].Current.GetADCValue - MotorControl[5].Current.FilterValue )>>4);
        MotorControl[5].Current.FilterValue = (u16)(TempValue + MotorControl[5].Current.FilterValue);
        MotorControl[5].Current.DeepFilterVAL = (MotorControl[5].Current.FilterValue*FILTER_COEFFICIENT+(1-FILTER_COEFFICIENT)*MotorControl[5].Current.PreFilterVal);		//100	代表1A
        MotorControl[5].Current.PreFilterVal = MotorControl[5].Current.DeepFilterVAL;   
	 

		 MotorControl[6].Current.GetADCValue = ADC2->JDR2  - MotorControl[6].Current.offset;
        if(MotorControl[6].Current.GetADCValue < 0) MotorControl[6].Current.GetADCValue = 0;
        TempValue = (s32)((MotorControl[6].Current.GetADCValue - MotorControl[6].Current.FilterValue )>>4);
        MotorControl[6].Current.FilterValue = (u16)(TempValue + MotorControl[6].Current.FilterValue);		 
		MotorControl[6].Current.DeepFilterVAL = (MotorControl[6].Current.FilterValue*FILTER_COEFFICIENT+(1-FILTER_COEFFICIENT)*MotorControl[6].Current.PreFilterVal);//1.37f
		MotorControl[6].Current.PreFilterVal = MotorControl[6].Current.DeepFilterVAL;
		 
		 CanLoadRate.timer++;
        DMA_Transfer_Complete_Count++;
        if(DMA_Transfer_Complete_Count>30000)
        {
            DMA_Transfer_Complete_Count = 30001;
        }
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
        MotorControl[0].Current.GetADCValue = ADC_ConvertedValue[6]-MotorControl[0].Current.offset;
        if(MotorControl[0].Current.GetADCValue< 0 ) MotorControl[0].Current.GetADCValue = 0;
        TempValue = (s32)((MotorControl[0].Current.GetADCValue - MotorControl[0].Current.FilterValue )>>4);
        MotorControl[0].Current.FilterValue = (u16)(TempValue + MotorControl[0].Current.FilterValue);

        //μ??ú1μ?μ?á÷ADC??2¨?μ
        MotorControl[1].Current.GetADCValue = ADC_ConvertedValue[7]-MotorControl[1].Current.offset;
        if(MotorControl[1].Current.GetADCValue < 0) MotorControl[1].Current.GetADCValue= 0;
        TempValue = (s32)((MotorControl[1].Current.GetADCValue - MotorControl[1].Current.FilterValue )>>4);
        MotorControl[1].Current.FilterValue = (u16)(TempValue + MotorControl[1].Current.FilterValue);


        //μ??ú2μ?μ?á÷ADC??2¨?μ
        MotorControl[2].Current.GetADCValue = ADC_ConvertedValue[8]-MotorControl[2].Current.offset;
        if(MotorControl[2].Current.GetADCValue < 0) MotorControl[2].Current.GetADCValue= 0;
        TempValue = (s32)((MotorControl[2].Current.GetADCValue - MotorControl[2].Current.FilterValue )>>4);
        MotorControl[2].Current.FilterValue = (u16)(TempValue + MotorControl[2].Current.FilterValue);


        //μ??ú3μ?μ?á÷ADC??2¨?μ
        MotorControl[3].Current.GetADCValue = ADC_ConvertedValue[0]-MotorControl[3].Current.offset;
        if(MotorControl[3].Current.GetADCValue < 0) MotorControl[3].Current.GetADCValue = 0;
        TempValue = (s32)((MotorControl[3].Current.GetADCValue - MotorControl[3].Current.FilterValue )>>4);
        MotorControl[3].Current.FilterValue = (u16)(TempValue + MotorControl[3].Current.FilterValue);
        //    MotorControl[3].Current.FilterValue -= offset[3];
        //    if(MotorControl[3].Current.FilterValue<0) MotorControl[3].Current.FilterValue= 0;

        //μ??ú4μ?μ?á÷ADC??2¨?μ
        MotorControl[4].Current.GetADCValue = ADC_ConvertedValue[1]-MotorControl[4].Current.offset;
        if(MotorControl[4].Current.GetADCValue <0)  MotorControl[4].Current.GetADCValue= 0;
        TempValue = (s32)((MotorControl[4].Current.GetADCValue - MotorControl[4].Current.FilterValue )>>4);
        MotorControl[4].Current.FilterValue = (u16)(TempValue + MotorControl[4].Current.FilterValue);


        MotorControl[9].Current.GetADCValue = ADC_ConvertedValue[9]-MotorControl[9].Current.offset;
        if(MotorControl[9].Current.GetADCValue <0)  MotorControl[9].Current.GetADCValue = 0 ;
        TempValue = (s32)((MotorControl[9].Current.GetADCValue - MotorControl[9].Current.FilterValue )>>4);
        MotorControl[9].Current.FilterValue = (u16)(TempValue + MotorControl[9].Current.FilterValue);

        ADeepFilter=MotorControl[5].Current.DeepFilterVAL;
        BDeepFilter=MotorControl[6].Current.DeepFilterVAL;
//				MotorControl5_Current_DeepFilterVAL = MotorControl[5].Current.DeepFilterVAL/DeepFilterValPer_A[0];
//				MotorControl6_Current_DeepFilterVAL = MotorControl[6].Current.DeepFilterVAL/DeepFilterValPer_A[1];
        /*end*/
        VoltVar.AdBuf = ADC_ConvertedValue[10];
        TempValue = (s32)((VoltVar.AdBuf - VoltVar.BUS )>>4);
        VoltVar.BUS = (u16)(TempValue +VoltVar.BUS);
        if(DMA_Transfer_Complete_Count > 30000) //偏置电压校准，开始数据不稳定丢弃
        {
            if( MotorState == IDLE)
            {
                Voltage_offset_cali();
            }
        }
        if(MotorState == RUN&&Motor_Type<2)
        {
            //			MotorControl[5].PWM_Duty = PID_Regulator(MotorControl[5].Current.ADCValue_Ref,MotorControl[5].Current.FilterValue,&PID_Speed_InitStruct[0]);
            /***********************5号无刷下发速度*****************************/
            if((MotorControl[5].Speed_Ref <= 0 && MotorControl[5].PWM_Duty > 0) || \
                    (MotorControl[5].Speed_Ref >=0 && MotorControl[5].PWM_Duty < 0) || MotorControl[5].Speed_Ref == 0 )
            {
                PID_Speed_InitStruct[0].wIntegral = 0;
                PID_Current_InitStructure[0].wIntegral = 0;
                MotorControl[5].PWM_Duty = 0;
            }
            else
            {
                if(MotorControl[5].Speed_Real<0)
                {
                    MotorControl[5].Current.DeepFilterVAL = -MotorControl[5].Current.DeepFilterVAL;
                }
                MotorControl[5].PWM_Duty = PID_Regulator(MotorControl[5].Current.ADCValue_Ref,MotorControl[5].Current.DeepFilterVAL,&PID_Current_InitStructure[0]);
            }
            if(MotorControl[5].Motor_Start_Stop==ENABLE)
            {
                if(MotorControl[5].Fault_Flag == 0&&OverFlow_Cnt[0]<BREAK_CNT_MAX)
                {
                    boot_charge5_6cnt[0]++;
                    if(boot_charge5_6cnt[0]>10)//充电完成    5
                    {
                        boot_charge5_6cnt[0] = 10;
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
                boot_charge5_6cnt[0] = 0;//清除充电计数
                SetMotorStop(5);
            }
            /***********************5号无刷下发速度end*****************************/

            /***********************6号无刷下发速度*****************************/
            if((MotorControl[6].Speed_Ref <= 0 && MotorControl[6].PWM_Duty > 0) || \
                    (MotorControl[6].Speed_Ref >=0 && MotorControl[6].PWM_Duty < 0) || MotorControl[6].Speed_Ref == 0 )
            {

                PID_Speed_InitStruct[1].wIntegral = 0;
                PID_Current_InitStructure[1].wIntegral = 0;
                MotorControl[6].PWM_Duty = 0;
            }
            else
            {
                if(MotorControl[6].Speed_Real<0)
                {
                    MotorControl[6].Current.DeepFilterVAL = -MotorControl[6].Current.DeepFilterVAL;
                }
                MotorControl[6].PWM_Duty = PID_Regulator(MotorControl[6].Current.ADCValue_Ref,MotorControl[6].Current.DeepFilterVAL,&PID_Current_InitStructure[1]);
            }

            if(MotorControl[6].Motor_Start_Stop==ENABLE)
            {
                if(MotorControl[6].Fault_Flag == 0&&OverFlow_Cnt[1]<BREAK_CNT_MAX)
                {
                    boot_charge5_6cnt[1]++;
                    if(boot_charge5_6cnt[1]>5)//充电完成
                    {
                        boot_charge5_6cnt[1] = 5;
                        SetMotorSpeed(6, MotorControl[6].PWM_Duty);
                    }
                    else
                    {
						  if(MotorControl[6].Speed_Real==0)
						{
							TIM8->CCER = OPEN_ALL_DOWN_TUBE;//充电500us
						}
                    }
                }
                else if(OverFlow_Cnt[1]>=BREAK_CNT_MAX)
                {
                    MotorControl[6].Motor_Start_Stop = DISABLE;
                    MC_SetFault(MOTOR6_BREAK);
                }
                else
                {
                    MotorControl[6].Motor_Start_Stop = DISABLE;
                }
            }
            else if(MotorControl[6].Motor_Start_Stop==DISABLE)
            {
                boot_charge5_6cnt[1] = 0;////清除充电计数
                SetMotorStop(6);
            }

        }
		else if(MotorState == RUN&&Motor_Type==2)
		{
			if(MotorControl[5].Motor_Start_Stop==ENABLE)
            {
			      SetMotorSpeed(5, MotorControl[5].PWM_Duty);
			}
			  else if(MotorControl[5].Motor_Start_Stop==DISABLE)
            {
                SetMotorStop(5);
            }
			if(MotorControl[6].Motor_Start_Stop==ENABLE)
            {
			      SetMotorSpeed(6, MotorControl[5].PWM_Duty);
			}
			  else if(MotorControl[6].Motor_Start_Stop==DISABLE)
            {
                SetMotorStop(6);
            }
		}
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
        if( ((GPIOE->IDR>>15)&(0x1)) ==0 )		/* 硬件过流brake保护*/
        {
              errormotor5++;
//            OverFlow_Cnt[0] +=1;
			if(errormotor5>=5)
			{
				TIM1->CCR1 = 0;
				TIM1->CCR2 = 0;
				TIM1->CCR3 = 0;
				TIM1->EGR = 1;
				MotorControl[5].Motor_Start_Stop = 0;
				MotorControl[5].PWM_Duty=0;
				MC_SetFault(MOTOR5_BREAK);
				OverFlow_Cnt[0]=0;
			}
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
    if(htim==(&htim9))
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  //7边刷
        {
            HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
            CH2TEMP = CH2TEMP+1;
        }
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  //8风机
        {
            HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
            CH1TEMP = CH1TEMP+1;
        }
    }
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */static void MX_GPIO_Init(void)
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


    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, PWM_AL3_Pin|PWM_BL3_Pin|PWM_AL4_Pin|PWM_BL4_Pin
                      |PWM_AL1_Pin|PWM_BL1_Pin|PWM_AL2_Pin|PWM_BL2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
//    HAL_GPIO_WritePin(GPIOA, SideBrush_BRAKE_Pin|SideBrush_F_R_Pin|SideBrush_EN_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOA,BRAKE3|FAULT2_Pin,GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOB,RUN_Pin,GPIO_PIN_SET);

    /*Configure GPIO pins : HALL_3_Pin HALL_4_Pin HALL_1_Pin HALL_2_Pin */
    GPIO_InitStruct.Pin = HALL_3_Pin|HALL_4_Pin|HALL_1_Pin|HALL_2_Pin;   //4个推杆 外部中断HALL
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : HALL_U2_Pin HALL_V2_Pin */
    GPIO_InitStruct.Pin = HALL_U2_Pin|HALL_V2_Pin|HALL_W2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PC15 */
//  GPIO_InitStruct.Pin = GPIO_PIN_15;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : FAN_EN_Pin */
    GPIO_InitStruct.Pin = FAN_F_R;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(FAN_F_R_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(FAN_F_R_GPIO_Port, FAN_F_R, GPIO_PIN_RESET);



    /*Configure GPIO pin : RUN_Pin */
    GPIO_InitStruct.Pin = RUN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RUN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PWM_AL3_Pin PWM_BL3_Pin PWM_AL4_Pin PWM_BL4_Pin
                             PWM_AL1_Pin PWM_BL1_Pin PWM_AL2_Pin PWM_BL2_Pin */
    GPIO_InitStruct.Pin = PWM_AL3_Pin|PWM_BL3_Pin|PWM_AL4_Pin|PWM_BL4_Pin
                          |PWM_AL1_Pin|PWM_BL1_Pin|PWM_AL2_Pin|PWM_BL2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(GPIOD,PWM_AL3_Pin|PWM_BL3_Pin|PWM_AL4_Pin|PWM_BL4_Pin
                      |PWM_AL1_Pin|PWM_BL1_Pin|PWM_AL2_Pin|PWM_BL2_Pin,GPIO_PIN_SET); //因为FD2103S的L通道为反向，所以这个初始化为高电平
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : SideBrush_BRAKE_Pin SideBrush_F_R_Pin SideBrush_EN_Pin BRAKE3
                             FAULT2_Pin */
    GPIO_InitStruct.Pin = FAULT2_Pin|SideBrush_BRAKE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA,FAULT2_Pin|SideBrush_BRAKE_Pin,GPIO_PIN_SET);

    GPIO_InitStruct.Pin = SideBrush_F_R_Pin|FAULT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB,FAULT1_Pin|SideBrush_F_R_Pin,GPIO_PIN_SET);



    /*Configure GPIO pin : BRAKE3 4 5 */

    GPIO_InitStruct.Pin = BRAKE5|BRAKE3|BRAKE4; //BRAKE4|SideBrush_BRAKE_Pin
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : BRAKE2_Pin */
    GPIO_InitStruct.Pin = BRAKE2_Pin;
     GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;  //为了brake滤波静电，所以上升沿和下降沿都触发外部中断
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BRAKE2_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : HALL_U1_Pin HALL_V1_Pin HALL_W1_Pin */
    GPIO_InitStruct.Pin = HALL_U1_Pin|HALL_V1_Pin|HALL_W1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}
void TIM1_TIM8_StartTimers( void )            //如果不配置主从模式，PWM较大的时候，由于高电平的时间比较长，所以两者采样点都可以为高电平。但是PWM较低的情况，高电平较短，则会导致一个采样点为PWM_ON,一个为采样点PWM_OFF,造成两者电流差距很大
{
  uint32_t isTIM2ClockOn;
  uint32_t trigOut;
  isTIM2ClockOn = __HAL_RCC_TIM2_IS_CLK_ENABLED(); //是否使能定时器2
  if ( isTIM2ClockOn == 0 )//未使能
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
    HAL_TIM_GenerateEvent ( &htim2, TIM_EVENTSOURCE_UPDATE); //生成更新事件
    __HAL_RCC_TIM2_CLK_DISABLE();
  }
  else //使能
  {
    trigOut = TIM2->CR2  & TIM_CR2_MMS;
	TIM2->CR2 &= ~TIM_CR2_MMS;     //清除主从模式
	TIM2->CR2 |= TIM_TRGO_UPDATE;  //更新触发
    HAL_TIM_GenerateEvent ( &htim2, TIM_EVENTSOURCE_UPDATE); //生成更新事件
	TIM2->CR2 &= ~TIM_CR2_MMS;    //主从模式打开
	TIM2->CR2 |= trigOut;		
  }
}

/* USER CODE BEGIN 4 */
extern u8 timer_cnt[4];
u16 Motor5_BreakTime;
u16 Motor6_BreakTime;
void HAL_SYSTICK_Callback(void)
{
//	Push_Motor_Location_Control(1);
    if(HAL_GetTick()%2 == 0)  //1MS
    {
		if(errormotor5>0)//电机5break触发后计数5s内没有再触发清零
		{
			Motor5_BreakTime++;
			if(Motor5_BreakTime>5000)
			{
			 errormotor5=0;
			 Motor5_BreakTime=0;
			}
		}
		else
		{
			Motor5_BreakTime=0;
		}
		if(errormotor6>0)//电机6break触发后计数5s内没有再触发清零
		{
			Motor6_BreakTime++;
			if(Motor6_BreakTime>5000)
			{
			 errormotor6=0;
			 Motor6_BreakTime=0;
			}
		}
		else
		{
			Motor6_BreakTime=0;
		}
		if(Motor_Type<2)
		{
          BLDC_Stuck_Chk();	//堵转保护500ms报警
		}
        BitSet(SysCLC.SysTimFlag,A1mSec);
    }
    if(HAL_GetTick()%4 == 0)  //2MS
    {
        switch (MotorState)
        {
        case IDLE:

//            MotorState = INIT;		偏置电压修正后进入INIT

            break;

        case INIT:

            MotorState = START; //标定成功再进入下一个状态，再使能CAN通信
            break;

        case START:

            MotorState = RUN;

            break;

        case RUN:   // motor running


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
                        Ramp_Speed(5);
                        MotorControl[5].Speed_Real = GetMotorSpeed(5);//得到实际速度
                        BLDC1_OverSpdChk();//失速保护
                        MotorControl[5].Current.ADCValue_Ref = PID_Regulator(MotorControl[5].Speed_Ref,MotorControl[5].Speed_Real,&PID_Speed_InitStruct[0]);  //速度环得到电流参考
                        break;
                    case 6:
                        Ramp_Speed(6);
                        MotorControl[6].Speed_Real = GetMotorSpeed(6);//得到实际速度
                        BLDC2_OverSpdChk();//失速保护
                        MotorControl[6].Current.ADCValue_Ref = PID_Regulator(MotorControl[6].Speed_Ref,MotorControl[6].Speed_Real,&PID_Speed_InitStruct[1]); //速度环得到电流参考
                        break;
                    case 3:
                    case 4:
                        if(MotorControl[i].Fault_Flag == 0&&OverFlow_Cnt[2]<BREAK_CNT_MAX) //没有出现硬件保护的时候
                        {
                            SetMotorSpeed(i, MotorControl[i].PWM_Duty);
                        }
                        else if(OverFlow_Cnt[2]>=BREAK_CNT_MAX)
                        {
                            MC_SetFault(BRAKE_3_4);
                            MotorControl[i].Motor_Start_Stop = DISABLE;
                        }
                        else
                        {
                            MotorControl[i].Motor_Start_Stop = DISABLE;
                        }
                        break;
                    case 0:
                    case 1:
                    case 2:
                    case 9:
                        if(MotorControl[i].Push_motor_calibrationFLAG == 1)    //每次标定完毕会将该标志位置2
                        {
                            Push_Motor_Calibrate(i);
                        }
                        else if(MotorControl[i].Fault_Flag == 0&&OverFlow_Cnt[3]<BREAK_CNT_MAX)
                        {
                            SetMotorSpeed(i, MotorControl[i].PWM_Duty);
                        }
                        else if(OverFlow_Cnt[3]>=BREAK_CNT_MAX)
                        {
                            MC_SetFault(BRAKE_0_1_2_9);
                            MotorControl[i].Motor_Start_Stop = DISABLE;
                        }
                        else
                        {
                            MotorControl[i].Motor_Start_Stop = DISABLE;
                        }
                        if(HAL_GetTick()%4 == 0) //10ms位置环
                        {
                            Push_Motor_Location_Control(i);		/* 推杆位置环 */
                        }
                        break;
                    case 10:
                    case 11:
                    case 12:
                    case 13:
                        if(MotorControl[i].Fault_Flag == 0&&OverFlow_Cnt[4]<BREAK_CNT_MAX)
                        {
                            SetMotorSpeed(i, MotorControl[i].PWM_Duty);
                        }
                        else if(OverFlow_Cnt[4]>=BREAK_CNT_MAX)
                        {
                            MC_SetFault(BRAKE10_11_12_13);
                            MotorControl[i].Motor_Start_Stop = DISABLE;
                        }
                        else
                        {
                            MotorControl[i].Motor_Start_Stop = DISABLE;
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
					if(i==0)
					{
						timer_cnt[0]=0;
					}
					else if(i==1)
					{
						timer_cnt[1]=0;
					}
					else if(i==2)
					{
						timer_cnt[2]=0;
						
					}
					else if(i==9)
					{
						timer_cnt[3]=0;
						
					}
                }
            }
            if(HAL_GetTick()%30 == 0)  //15ms
            {
                Push_Motor_CurSelfAdapt(5,1,580,5);    //推杆高度自适应
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

        default:
            break;
        }
    }
    if(HAL_GetTick()%20 == 0) //10ms 一次 500ms保护
    {
        Over_VoltageCheck(); //过压保护
    }
    if(HAL_GetTick()%1000 == 0) //500ms喂狗
    {
        HAL_IWDG_Refresh(&hiwdg);
        Push_OverRunChk(1,20,1,20,0,30,0,30);    //推杆动作时间限制
    }

}

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

/**
  * oˉêy1|?ü: ???¨?òc?aoˉêygetchar,scanfμ?DEBUG_USARTx
  * ê?è?2?êy: ?T
  * ·μ ?? ?μ: ?T
  * ?μ    ?÷￡o?T
  */
int fgetc(FILE * f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart3,&ch, 1, 0xffff);
    return ch;
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

int16_t LV_TxBuffer[10],TxBufferSize=10;
u8 uart_counter=7;
#define ac 0
#define ab 1
#define bc 2
#define ac_ab_bc_index 3
extern uint32_t M6_100us_ac_ab_bc_cur[ac_ab_bc_index];
extern uint32_t testM6_100us_ac_ab_bc_cur;
void Labview_uart(void)
{
    s16 temp_tab[5];
    switch(uart_counter)
    {
    case 0:
        temp_tab[0] = 0x5A0A;;//
        temp_tab[1] = 23055;//
        temp_tab[2] = 24555;//
        temp_tab[3] = 30000;//
        temp_tab[4] = 20000 ;//

        break;
    case 1:
        temp_tab[0] = 0x5A0A;;//
        temp_tab[1] = MotorControl[0].Current.FilterValue;//
        temp_tab[2] = MotorControl[1].Current.FilterValue;//
        temp_tab[3] = MotorControl[2].Current.FilterValue;//
        temp_tab[4] = MotorControl[3].Current.FilterValue;//

        break;
    case 2:
        temp_tab[0] = 0x5A0A;;//
        temp_tab[1] = MotorControl[5].Current.GetADCValue;//
        temp_tab[2] = MotorControl[5].Current.FilterValue;//
        temp_tab[3] = MotorControl[6].Current.GetADCValue;//
        temp_tab[4] = MotorControl[6].Current.FilterValue ;//

        break;
    case 3:
        temp_tab[0] = 0x5A0A;;//
        temp_tab[1] = MotorControl[4].Current.FilterValue;//
        temp_tab[2] = MotorControl[9].Current.FilterValue;//
        temp_tab[3] = MotorControl[2].Current.FilterValue;//
        temp_tab[4] = MotorControl[3].Current.FilterValue ;//

        break;

    case 4:
        temp_tab[0] = 0x5A0A;;//
        temp_tab[1] = MotorControl[5].Speed_Ref;//
        temp_tab[2] = MotorControl[5].Speed_Real;//
        temp_tab[3] = MotorControl[6].Speed_Ref;//
        temp_tab[4] = MotorControl[6].Speed_Real ;//

        break;
    case 5:
        temp_tab[0] = 0x5A0A;;//
        temp_tab[1] = MotorControl[5].Speed_Ref;//
        temp_tab[2] = MotorControl[5].Speed_Real;//
        temp_tab[3] = MotorControl[5].PWM_Duty;//
        temp_tab[4] = MotorControl[5].Speed_Set ;

        break;

    case 6:
        temp_tab[0] = 0x5A0A;;//
        temp_tab[1] = M6_100us_ac_ab_bc_cur[ab]/10;//
        temp_tab[2] = M6_100us_ac_ab_bc_cur[ac]/10;//
        temp_tab[3] = M6_100us_ac_ab_bc_cur[bc]/10;//
        temp_tab[4] = testM6_100us_ac_ab_bc_cur/10;

        break;
    case 7:
        temp_tab[0] = 0x5A0A;;//
        temp_tab[1] = (u16)LV_Uart_TxBuffer[0][Uart_TxBuffer_CNT];//
        temp_tab[2] = (u16)LV_Uart_TxBuffer[1][Uart_TxBuffer_CNT];//
        temp_tab[3] = (u16)LV_Uart_TxBuffer[2][Uart_TxBuffer_CNT];//
        temp_tab[4] = (u16)LV_Uart_TxBuffer[3][Uart_TxBuffer_CNT] ;//
//            Uart_TxBuffer_CNT++;
        if(Uart_TxBuffer_CNT>LV_TxBufnum-1)
        {
            Uart_TxBuffer_CNT = 0 ;
            Uart_TxBuffer_CNT1 =0 ;
        }
        break;

    default:
        break;
    }

    LV_TxBuffer[0] = temp_tab[0]/256;
    LV_TxBuffer[1] = temp_tab[0]%256;

    LV_TxBuffer[2] = temp_tab[1]/256;
    LV_TxBuffer[3] = temp_tab[1]%256;

    LV_TxBuffer[4] = temp_tab[2]/256;
    LV_TxBuffer[5] = temp_tab[2]%256;

    LV_TxBuffer[6] = temp_tab[3]/256;
    LV_TxBuffer[7] = temp_tab[3]%256;

    LV_TxBuffer[8] = temp_tab[4]/256;
    LV_TxBuffer[9] = temp_tab[4]%256;

//	  HAL_UART_Transmit(&UART6_Handler, (uint8_t *)LV_TxBuffer, TxBufferSize,0x20);

//	  HAL_UART_Transmit_IT(&UART6_Handler, (uint8_t *)LV_TxBuffer, TxBufferSize);

    printf("%c%c%c%c%c%c%c%c%c%c",LV_TxBuffer[0],LV_TxBuffer[1],LV_TxBuffer[2],LV_TxBuffer[3],LV_TxBuffer[4],LV_TxBuffer[5],LV_TxBuffer[6],LV_TxBuffer[7],LV_TxBuffer[8],LV_TxBuffer[9]);
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
