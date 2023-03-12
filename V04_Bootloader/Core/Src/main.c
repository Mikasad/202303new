/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan2;
CanTxMsgTypeDef TxMessage;
CanRxMsgTypeDef RxMessage;
UART_HandleTypeDef huart3;
pFunction Jump_To_Application;
TIM_HandleTypeDef htim7;
/* USER CODE BEGIN PV */
uint32_t JumpAddress;
typedef int32_t s32;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
SysStatus_t SysState;
OTA_TypeDef OTA;

uint16_t Receive_cnt000=0;

uint16_t number12=0;
//int total_time=0;
uint16_t ii;
uint8_t Can_Receive_Flag=0;
uint8_t Can_Err=0;
int OTA_Time=0;
int Message_Flag=0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t flash_buf[PACK_SIZE];
uint8_t pack_cnt = 0,byte_cnt =0;
uint32_t ApplicationAddress = 0x8020000,APP_data[2],BackUp_data[2];
const unsigned char auchCRCHi[] =
{
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;
/* CRC低位字节值表*/
const unsigned char auchCRCLo[] =
{
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;

unsigned short crc16(unsigned char *puchMsg, unsigned short usDataLen)
{
    unsigned char uchCRCHi = 0xFF; /* 高CRC字节初始化 */
    unsigned char uchCRCLo = 0xFF; /* 低CRC 字节初始化 */
    unsigned short uIndex; /* CRC循环中的索引 */

    while (usDataLen--)  /* 传输消息缓冲区 */
    {
        uIndex = uchCRCHi^ *puchMsg++; /* 计算CRC */
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo);
}


uint8_t CRC16_chk(unsigned char *puchMsg, unsigned short usDataLen)
{
    uint16_t CRC16_temp = 0;
    CRC16_temp = crc16(puchMsg, usDataLen);
    if(((uint8_t)(CRC16_temp >> 8) == *(puchMsg + usDataLen)) && ((uint8_t)(CRC16_temp) == *(puchMsg + usDataLen + 1)))
        return TRUE;
    else return FALSE;
}

uint8_t CRC16_creat(unsigned char *puchMsg, unsigned short usDataLen)
{
    uint16_t CRC16_temp = 0;
    CRC16_temp = crc16(puchMsg, usDataLen);
    *(puchMsg + usDataLen) = (uint8_t)(CRC16_temp >> 8);
    *(puchMsg + usDataLen + 1) = (uint8_t)CRC16_temp;
    return TRUE;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t buf_send[8],rollback_packnum =0;
	uint32_t flash_buftemp[4];
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
  MX_CAN2_Init();
  MX_TIM7_Init();			
//  MX_USART3_UART_Init();
	HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
	HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE BEGIN 2 */
	APP_data[0] = *(long*)Data_Version_Sector;					//app版本号
	APP_data[1] = *(long*)(Data_Version_Sector+4);			//app代码大小
	BackUp_data[0] = *(long*)(Data_Version_Sector+8);		//备份代码版本号
	BackUp_data[1] = *(long*)(Data_Version_Sector+12);	//备份代码大小
	
  /* USER CODE END 2 */
	buf_send[0] = 0xe2;
	buf_send[1] = 0x01;
	CANx_SendNormalData(&hcan2,0x20,buf_send,2);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//	  if(Can_Receive_Flag==2&&Can_Err==0x03)
//	  {
//		  SysState=Rollback;
//	  }
	  if(OTA_Time>10000)
	  {
		   SysState=Rollback;
	  }
		switch(SysState)
		{
			case Wait:
				if(HAL_GetTick()>5000)   //30000
				{
					SendErrorCode(0x04,0x01);
//					if(APP_data[0]==0xffffffff&&APP_data[1]==0xffffffff&&BackUp_data[0]!=0xffffffff&&BackUp_data[1]!=0xffffffff)
//					{
//						ApplicationAddress = APP_Sector_2;
//						SendErrorCode(0x06,0x01);
//					}
//					else
						ApplicationAddress = APP_Sector_1;         //app主程序区
					if(Jump_to_APP(ApplicationAddress)==FALSE)
					{
						SendErrorCode(0x05,0x01);                   //无法跳转至app区
					}
				}
				break;
			case Updating:
				if(OTA.EraseFlag)
				{
					if(Erase_Sector(ApplicationAddress,OTA.ByteNum)==TRUE)
					{
						pack_cnt++;
						buf_send[0] = pack_cnt;
						buf_send[1] = byte_cnt;
						CANx_SendNormalData(&hcan2,0x21,buf_send,2);     //驱动器数据包信息握手
					}
					else 
					{
						SendErrorCode(0x01,0x02);              //擦除失败
					}
					OTA.EraseFlag=0;
				}
				if(OTA.WriteFlag)                            //CRC校验
				{
					if(Flash_Write((pack_cnt-1)*PACK_SIZE+ApplicationAddress,flash_buf,OTA.SizeOfArray)==TRUE)
					{
						byte_cnt =0;
						Receive_cnt000=0;
						pack_cnt++;
						buf_send[0] = pack_cnt;
						buf_send[1] = byte_cnt;
						CANx_SendNormalData(&hcan2,0x21,buf_send,2);
						
					}
					else 
					{
						SendErrorCode(0x02,0x03);
					}
					OTA.WriteFlag=0;
				}
//			if(pack_cnt==OTA.PackNum+1&&pack_cnt!=1)                  //校验最后的接收到的数据包和实际下位机发送的pack数据包数量是否一致
//			{
//				number12=1;
					if(OTA.Back_finishFlag&&OTA.version_flag==1)           //备份完成并且下位机发送固件信息
					{
						number12=2;
						flash_buftemp[0] = OTA.Code_Ver;
						flash_buftemp[1] = OTA.ByteNum;
						flash_buftemp[2] = APP_data[0];
						flash_buftemp[3] = APP_data[1];
						if(Flash_Write_long(Data_Version_Sector,flash_buftemp,4)==TRUE)        //与wait状态一样，判断是否在app区域
						{
							number12=3;
							ApplicationAddress = APP_Sector_1;
							if(Jump_to_APP(ApplicationAddress)==FALSE)
							{
								number12=4;
								HAL_Delay(1000);
								SendErrorCode(0x05,0x01);
							}
						}
						else 
						{
							number12=6;
							SendErrorCode(0x02,0x04);
						}
					}
					
//				}
//				else
//				{
//					SendErrorCode(0x07,0x01);
//				}
				break;
			case Copying: 
//				OTA.BackUp_ByteNum = APP_data[1];                  //app代码大小
			    OTA.BackUp_ByteNum = 102400;		//100K
				OTA.BackUp_PackNum = (OTA.BackUp_ByteNum%PACK_SIZE==0) ? OTA.BackUp_ByteNum/PACK_SIZE:OTA.BackUp_ByteNum/PACK_SIZE+1;       //多少个pack包
//				if(APP_data[0]==0xffffffff&&APP_data[1]==0xffffffff)
//				{
//					buf_send[0] = 0xe4;
//					buf_send[1] = 0x01;
//					CANx_SendNormalData(&hcan2,0x20,buf_send,2);          //备份完成
//					OTA.Back_finishFlag =1;
//					SysState = Updating;
//				}
				 if(APP_Backup(APP_Sector_1,APP_Sector_2,OTA.BackUp_ByteNum,OTA.BackUp_PackNum)==TRUE)            //程序备份函数
				{
					flash_buftemp[0] = 0xffffffff;
					flash_buftemp[1] = 0xffffffff;
					flash_buftemp[2] = APP_data[0];
					flash_buftemp[3] = APP_data[1];
					if(Flash_Write_long(Data_Version_Sector,flash_buftemp,4)==TRUE)            
					{
						OTA.Back_finishFlag =1;
						buf_send[0] = 0xe4;
						buf_send[1] = 0x01;
						CANx_SendNormalData(&hcan2,0x20,buf_send,2);              //备份完成后进入更新状态
						SysState = Updating;
					}
					else 
						SendErrorCode(0x02,0x02);        //写入失败
				}
				  
				break;
			case Rollback:                                //版本回退
				{
//					if(BackUp_data[0]==0xffffffff&&BackUp_data[1]==0xffffffff)
//					{
//					
//					}
//					else 
//					{
					BackUp_data[1]=102400;  //100k
						rollback_packnum = (BackUp_data[1]%PACK_SIZE==0) ? BackUp_data[1]/PACK_SIZE:BackUp_data[1]/PACK_SIZE+1;    //pack包数量  
						if(APP_Backup(APP_Sector_2,APP_Sector_1,BackUp_data[1],rollback_packnum)==TRUE)   //备份区程序写入主程序
						{
							flash_buftemp[0] = BackUp_data[0];
							flash_buftemp[1] = BackUp_data[1];
							flash_buftemp[2] = BackUp_data[0];
							flash_buftemp[3] = BackUp_data[1];
							if(Flash_Write_long(Data_Version_Sector,flash_buftemp,4)==TRUE)    //程序信息区写入，备份区域的代码版本号和大小写入程序信息区
							{
								ApplicationAddress = APP_Sector_1;
								if(Jump_to_APP(ApplicationAddress)==FALSE)  //跳转至app区
								{
									HAL_Delay(1000);
									SendErrorCode(0x05,0x01);
								}
							}
							else 
								SendErrorCode(0x02,0x02);
						}
//					}
				}
			
				break;
			case Fault:
				break;
		}
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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
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

    // HAL_RCC_GetHCLKFreq()/1000    1ms
    // HAL_RCC_GetHCLKFreq()/100000	 10us
    // HAL_RCC_GetHCLKFreq()/1000000 1us
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/2000);                 
    
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

   
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}





static void MX_TIM7_Init(void)     //中断1ms
{

    /* USER CODE BEGIN TIM7_Init 0 */

    /* USER CODE END TIM7_Init 0 */
    
    TIM_MasterConfigTypeDef sMasterConfig = {0};
     __HAL_RCC_TIM7_CLK_ENABLE();
    /* USER CODE BEGIN TIM7_Init 1 */

    /* USER CODE END TIM7_Init 1 */
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 83;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 1000;
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
	HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0); //无刷测速 和电机7、8测速
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

}

void TIM7_IRQHandler(void)
{
    /* USER CODE BEGIN TIM7_IRQn 0 */

    /* USER CODE END TIM7_IRQn 0 */
    HAL_TIM_IRQHandler(&htim7);
    /* USER CODE BEGIN TIM7_IRQn 1 */

    /* USER CODE END TIM7_IRQn 1 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		s32 TempValue;
    if(htim==(&htim7))
    {
		if(Message_Flag==1)
		{
		   OTA_Time++;
		}
		
	}
}


/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{
  /* USER CODE BEGIN CAN2_Init 0 */
	CAN_FilterConfTypeDef  sFilterConfig;
  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
	hcan2.pTxMsg = &TxMessage;
  hcan2.pRxMsg = &RxMessage;
  hcan2.Init.Prescaler = 7;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SJW = CAN_SJW_1TQ;      
	hcan2.Init.BS1 = CAN_BS1_10TQ;     
	hcan2.Init.BS2 = CAN_BS2_1TQ;     
	hcan2.Init.TTCM = DISABLE;        
	hcan2.Init.ABOM = ENABLE;          
	hcan2.Init.AWUM = ENABLE;          
	hcan2.Init.NART = DISABLE ;  //DISABLE      
	hcan2.Init.RFLM = DISABLE;         
	hcan2.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
	sFilterConfig.FilterNumber = 14; 
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  
//	sFilterConfig.FilterIdHigh =0X0000; //要过滤的ID高位
//	sFilterConfig.FilterIdLow =0X0000;//要过滤的ID低位
//	sFilterConfig.FilterMaskIdHigh = 0X0000;
//	sFilterConfig.FilterMaskIdLow = 0X0000;
  
    sFilterConfig.FilterIdHigh         = (((uint32_t)0x17<<21)&0xFFFF0000)>>16;        /* 要过滤的ID高位 */
    sFilterConfig.FilterIdLow          = (((uint32_t)0x17<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF; /* 要过滤的ID低位 */
    sFilterConfig.FilterMaskIdHigh     = 0xFF1F;      /* 过滤器高16位每位必须匹配 */
    sFilterConfig.FilterMaskIdLow      = 0xFFFF;      /* 过滤器低16位每位必须匹配 */

  
	sFilterConfig.FilterFIFOAssignment=CAN_FilterFIFO0;
	sFilterConfig.BankNumber = 14;
	sFilterConfig.FilterActivation=ENABLE;
	if(HAL_CAN_ConfigFilter(&hcan2,&sFilterConfig)!=HAL_OK)
	{
		Error_Handler();
	}
  /* USER CODE END CAN2_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
//	
  GPIO_InitStruct.Pin = GPIO_PIN_15;              //Power_Ctrl
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
	
  GPIO_InitStruct.Pin = SideBrush_BRAKE_Pin|Fan_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOD,SideBrush_BRAKE_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD,Fan_EN_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */
uint8_t APP_Backup(uint32_t App_addr,uint32_t back_up_addr,uint32_t byte_num,uint8_t pack_num)		//备份程序
{
	uint16_t i,j;
	if(Erase_Sector(back_up_addr,byte_num)==TRUE)
	{
		for(j=0;j<pack_num;j++)
		{
			for(i=0;i<PACK_SIZE;i++)
			{
				flash_buf[i] = *(uint8_t*)App_addr;
				App_addr++;
			}
			if(Flash_Write(back_up_addr+j*PACK_SIZE,flash_buf,PACK_SIZE)==FALSE)
			{
				SendErrorCode(0x02,0x01);
				return FALSE;
			}
		}
	}
	else 
	{
		SendErrorCode(0x01,0x01);
	}
	return TRUE;
}


void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan) 
{
	uint8_t return_data[8],i,tempsize;
	if(hcan->Instance ==CAN2)
	{
		Can_Receive_Flag=2;
		Message_Flag=1;
//		total_time=OTA_Time;
			OTA_Time=0;
		if(RxMessage.StdId == 0x10)        //CAN ID:0x10
		{
			if(RxMessage.Data[0]==0xe0)
			{
				SoftReset();              //重启应用，跳转至Bootloader区域
			}
			else if(RxMessage.Data[0]==0xe1&&RxMessage.Data[1]==0x01)       //更新固件
			{
				return_data[0] = 0xe2;
				return_data[1] = 0x01;
				CANx_SendNormalData(&hcan2,0x20,return_data,2);          //进入bootloader
			}
			else if(RxMessage.Data[0]==0xe3)            //备份旧版固件
			{
				SysState = Copying;
			}
			else if(RxMessage.Data[0]==0xe5)		//回退指令
			{
				return_data[0] = 0xe6;
				return_data[1] = 0x01;
				CANx_SendNormalData(&hcan2,0x20,return_data,2);  
				SysState = Rollback;                                   //进入回退
			}
		}
		else if(RxMessage.StdId ==0x11)              //上位机发送的代码数据
		{
			if(OTA.SizeOfArray ==1024) OTA.SizeOfArray = 0;           //每隔1024个字节接收
			OTA.SizeOfArray+=RxMessage.DLC;                             //收到的数据长度
			tempsize = (byte_cnt*8<OTA.SizeOfArray)?8:(OTA.SizeOfArray-8*(byte_cnt-1));       // //每隔8个字节握手一次，10 20 30 40 50 60 70 80握手一次
			for(i=0;i<tempsize;i++)
			{
				flash_buf[8*byte_cnt+i] = RxMessage.Data[i];
			}
			
			for( ii=0,ii<10000;ii++;);
			
			byte_cnt++;
			return_data[0] = pack_cnt;
			return_data[1] = byte_cnt;
			CANx_SendNormalData(&hcan2,0x21,return_data,2);          //驱动器收到的数据包信息握手
			Receive_cnt000++;
			
			
		}
		else if(RxMessage.StdId == 0x12)             //CRC校验
		{
			if(RxMessage.DLC==2&&crc16(flash_buf,OTA.SizeOfArray)==(RxMessage.Data[0]<<8)+RxMessage.Data[1])
			{
				OTA.WriteFlag=1;
			}
			else 
			{
			     SendErrorCode(0x03,pack_cnt);
//				 pack_cnt=-1;		
			}
		}
		else if(RxMessage.StdId == 0x13)          //上位机发送的固件版本
		{
				OTA.Code_Ver = (RxMessage.Data[3]<<24)+(RxMessage.Data[2]<<16)+(RxMessage.Data[1]<<8)+RxMessage.Data[0]; //版本号
				OTA.ByteNum = (RxMessage.Data[7]<<24)+(RxMessage.Data[6]<<16)+(RxMessage.Data[5]<<8)+RxMessage.Data[4];  //固件字节数
				OTA.PackNum = (OTA.ByteNum%PACK_SIZE==0) ? OTA.ByteNum/PACK_SIZE:OTA.ByteNum/PACK_SIZE+1;                //计算多少包
				OTA.version_flag=1;
//		       if(pack_cnt!=OTA.PackNum+1&&pack_cnt!=1)                  //校验最后的接收到的数据包和实际下位机发送的pack数据包数量不一致
//				{
//					SendErrorCode(0x07,0x01);
//				}
		}
		else if(RxMessage.StdId == 0x14)                   //擦除指令
		{
			OTA.EraseFlag=1;	
		}
	}
	HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
	
}


void CANx_SendNormalData(CAN_HandleTypeDef *hcanX,uint16_t ID,uint8_t *pData, uint16_t Len)
{
	uint8_t i=0;
	hcan2.pTxMsg->StdId = ID;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->ExtId=0xFFF;
	hcan2.pTxMsg->DLC=Len;
	for(i=0; i<Len; i++)
	{
			hcan2.pTxMsg->Data[i]=pData[i];
	}
	Can_Err = HAL_CAN_Transmit(&hcan2,100);
}
void SoftReset(void)
{
	__set_FAULTMASK(1);    
	NVIC_SystemReset();
}
void SendErrorCode(uint8_t ErrorCode,uint8_t CodeNum)
{
	static uint32_t last_tick;
	if(HAL_GetTick()-last_tick>2000)
	{
	uint8_t CodeArray[2];
	CodeArray[0] = ErrorCode;
	CodeArray[1] = CodeNum;
	CANx_SendNormalData(&hcan2,0xff,CodeArray,2);
		last_tick = HAL_GetTick();
	}
}
uint8_t Jump_to_APP(uint32_t APP_addr)
{
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;     //新增7/23
	
	if(((*(__IO uint32_t*)APP_addr) & 0x2FFE0000 ) == 0x20000000)
	{
		JumpAddress = *(__IO uint32_t*) (APP_addr + 4);
		Jump_To_Application = (pFunction) JumpAddress;
		__set_PRIMASK(1);
		HAL_NVIC_DisableIRQ(SysTick_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);   //新增
		__disable_irq();
		__set_MSP(*(__IO uint32_t*) APP_addr);
		Jump_To_Application();
	}
	else
		return FALSE;
	return FALSE;
}
static uint32_t GetSector(uint32_t Address)
{
    uint32_t sector = 0;

    if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
    {
        sector = FLASH_SECTOR_7;
    }
    else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
    {
        sector = FLASH_SECTOR_8;
    }
    else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
    {
        sector = FLASH_SECTOR_9;
    }
    else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
    {
        sector = FLASH_SECTOR_10;
    }
    else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
    {
        sector = FLASH_SECTOR_11;
    }

    return sector;
}
uint8_t WriteData_toFlash(uint8_t *data,uint32_t len,uint32_t address)
{
    uint32_t i=0;
    for(i=0; i<len; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)address, *data) == HAL_OK)
        {
            address = address+1;
            data = data+1;
        }
        else
        {
            return FALSE;
        }
    }
    return TRUE;
}
uint8_t Write_Int_Data(uint32_t *data,uint32_t len,uint32_t address)
{
	uint32_t i=0;
	for(i=0; i<len; i++)
	{
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)address, *data) == HAL_OK)
			{
					address = address+4;
					data = data+1;
			}
			else
			{
					return FALSE;
			}
	}
	return TRUE;
}

uint32_t FirstSector = 0, NbOfSectors = 0,SectorError = 0,MemoryProgramStatus = 0;
uint8_t Flash_Write(uint32_t Address,uint8_t *data,uint32_t data_len)
{
	uint32_t i;
	uint8_t tempdata;
	HAL_FLASH_Unlock();
	if(WriteData_toFlash(data,data_len,Address)==FALSE)
	{
	
	}
	HAL_FLASH_Lock();
	MemoryProgramStatus = 0;
	for(i=0;i<data_len;i++)
	{
		tempdata = *(__IO uint8_t*)(Address+i);
		if(tempdata !=data[i])
		{
			MemoryProgramStatus++;
		}
	}
	if(MemoryProgramStatus)
		return FALSE;
	else 
		return TRUE;
}

uint8_t Erase_Sector(uint32_t StartAddress, uint32_t data_len)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	HAL_FLASH_Unlock();
	FirstSector = GetSector(StartAddress);
	NbOfSectors = 1;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;//扇区擦除
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FirstSector;
  EraseInitStruct.NbSectors = NbOfSectors;
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
		HAL_FLASH_Lock();
		return FALSE;
	}
	else 
	{
		HAL_FLASH_Lock();
		return TRUE;
	}
}
uint8_t Flash_Write_long(uint32_t Address,uint32_t *data,uint32_t data_len)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t i,temp32;
	
	HAL_FLASH_Unlock();
	FirstSector = GetSector(Address);
	NbOfSectors = 1;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;//扇区擦除
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FirstSector;
  EraseInitStruct.NbSectors = NbOfSectors;
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
		
	}
	if(Write_Int_Data(data,data_len,Address)==FALSE)
	{
	
	}
	HAL_FLASH_Lock();
	MemoryProgramStatus = 0;
	for(i=0;i<data_len;i++)
	{
		temp32 = *(__IO uint32_t*)(Address+i*4);
		if(temp32 !=data[i])
		{
			MemoryProgramStatus++;
		}
	}
	if(MemoryProgramStatus)
		return FALSE;
	else 
		return TRUE;
}
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

int fgetc(FILE * f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart3,&ch, 1, 0xffff);
    return ch;
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
