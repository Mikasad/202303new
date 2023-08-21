#include "main.h"
#include <stdio.h>
#include "stm32f4xx_hal_flash.h"
CAN_HandleTypeDef hcan2;
CanTxMsgTypeDef TxMessage;
CanRxMsgTypeDef RxMessage;
UART_HandleTypeDef huart3;
pFunction Jump_To_Application;
TIM_HandleTypeDef htim7;
const char Compiler_Date[] __attribute__((at(0x8000000 + 0x200))) = "1111" ;
uint32_t JumpAddress;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
SysStatus_t SysState;
OTA_TypeDef OTA;
#define GREEN_LED_TOGGLE        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12)
#define RED_LED2_TOGGLE         HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_12)
#define RED_LED1_TOGGLE         HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_6)
uint16_t Receive_cnt000=0;
uint8_t buf_send[8];                  /*CAN回复的数据包*/
uint16_t ii;                          /*等待时间*/
uint8_t Can_Err=0;                    /*调试查看CAN是否有发送失败的情况*/
uint32_t OTA_Flag[2]= {2,0};          /*OTA判断标志位，APP跳转前写为1，APP初始化直接擦除为FF,升级完成直接跳转不用判断，
                                        假设此时APP程序不正确，因为没有擦除ADDR_FLASH_SECTOR_4地址的01,所以一直在boot
                                        中等待升级命令，如果升级过程中断电\bin文件不正确，重启后那么这个地址的值不能
                                        被擦除一直保持为1，不能进行跳转*/
int OTA_Time=0;                       /*用于判断OTA升级超时*/
int Message_Flag=0;                   /*收到CAN命令，此值赋1*/
void Flash_Read(uint32_t addresss,uint32_t *pBuffer,uint32_t bufsize);   /*读某个地址某个长度的数据放在 pbuffer所指的数组中*/
uint8_t flash_buf[PACK_SIZE];         /*1024大小，总计1kb数据*/
uint8_t pack_cnt = 0,byte_cnt =0;     /*pack cnt 表示多少kb，byte cnt 表示多少个字节*/
uint32_t ApplicationAddress = 0x8020000,APP_data[2],BackUp_data[2]; /*APP开始地址，appdata未使用，backupdata表示需要回退的代码大小*/
uint32_t  read_buf[1] = {0};         /*用于存放从APP 写入的 8020200 地址写入的 S02A数据 读出来显示：0x41323053*/
uint8_t Version_Flag=0;   //版本判断标志
uint8_t New_Bootloader_Flag=0;
/* CRC高位字节值表*/
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


int main(void)
{
    uint8_t rollback_packnum =0;         /*字节数*/
    uint32_t flash_buftemp[4];           /*未使用*/
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_CAN2_Init();
    MX_TIM7_Init();
    Flash_Read(ADDR_FLASH_SECTOR_4,OTA_Flag,1);        /*读取 0x0800C00地址的数据*/
//  MX_USART3_UART_Init();
    HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
    HAL_TIM_Base_Start_IT(&htim7);

    APP_data[0] = *(long*)Data_Version_Sector;					//app版本号
    APP_data[1] = *(long*)(Data_Version_Sector+4);			//app代码大小
    BackUp_data[0] = *(long*)(Data_Version_Sector+8);		//备份代码版本号
    BackUp_data[1] = *(long*)(Data_Version_Sector+12);	//备份代码大小

    buf_send[0] = 0xe2;
    buf_send[1] = 0x01;
    CANx_SendNormalData(&hcan2,0x20,buf_send,2);  /*上电发送 CANID: 0x20 data: e2 01*/

    while (1)
    {
		if(New_Bootloader_Flag==1)
		{
			New_Bootloader_Flag=0;
				ApplicationAddress = ADDR_FLASH_SECTOR_1;
				if(Jump_to_APP(ApplicationAddress)==FALSE)
				{
					HAL_Delay(1000);                                   /*不能在CAN中断里面跳app，CAN会有问题*/
					SendErrorCode(0x05,0x01);                          /*无法跳转至app区*/
				}
		}
        switch(SysState)
        {
        case Wait:
            if(HAL_GetTick()>5000)                                         /*如果在wait状态持续5000=2.5S则跳转至app，上电不升级跳转*/
            {
                SendErrorCode(0x04,0x01);                                  /*持续在Wait状态*/
                Flash_Read(0x08020200,read_buf,1);                         /*此处与蔡工确认*//*该地址的数据为APP当中写入的 S02A*/
                Flash_Read(ADDR_FLASH_SECTOR_4,OTA_Flag,1);                /*这个otaflag可以闭环掉电的情况*/
                if((OTA_Flag[0] == 0xFF ||OTA_Flag[0] == 0xFFFFFFFF) && (read_buf[0] == 0x41413448))  /*  读取app内容是否为可运行的app， 0x41323053 为 asc码 A20S*/
                {
                    ApplicationAddress = APP_Sector_1;                         /*app主程序区*/
                    if(Jump_to_APP(ApplicationAddress)==FALSE)
                    {
                        HAL_Delay(1000);                                   /*此处与蔡工确认*/
                        SendErrorCode(0x05,0x01);                          /*无法跳转至app区*/
                    }
                }
                else if(OTA_Flag[0] == 0x00000002 || OTA_Flag[0] == 0x00000001)      /*掉电情况1，APP区可用，备份区域不可用，1代表此时bootloader里面没有进行升级，可以直接跳转app*/
                {
                    ApplicationAddress = APP_Sector_1;
                    if(Jump_to_APP(ApplicationAddress)==FALSE)
                    {
                        HAL_Delay(1000);                                   /*此处与蔡工确认*/
                        SendErrorCode(0x05,0x01);                          /*无法跳转至app区*/
                    }
                }
                else if(OTA_Flag[0] == 0x00000003)                         /*掉电情况2，备份区可用，APP区域不可用*/
                {
                    SysState = Rollback;                                   /*进行回退*/
                }
                else
                {
                    SendErrorCode(0x02,0x04);                              /*不是该设备的bin文件不进行跳转*/
                }
            }
            break;
        case Rollback:                                                  /*版本回退*/
            BackUp_data[1]=131072;                                      /*128K 大小102400 改为 131072*/
//            rollback_packnum = (BackUp_data[1]%PACK_SIZE==0) ? BackUp_data[1]/PACK_SIZE:BackUp_data[1]/PACK_SIZE+1;    //pack包数量
            if(BackUp_data[1]%PACK_SIZE == 0)
            {
                rollback_packnum = BackUp_data[1]/PACK_SIZE;
            }
            else
            {
                rollback_packnum = BackUp_data[1]/PACK_SIZE+1;
            }
            Flash_Read(0x08040200,read_buf,1);                           /*该地址的数据为APP当中写入的 S02A，因为是回退所以需要判断备份区的内容*/
            if(read_buf[0] == 0x41413448)   /*读取app内容是否为可运行的app， 0x41323053 为 asc码 A20S*/
            {
                if(APP_Backup(APP_Sector_2,APP_Sector_1,BackUp_data[1],rollback_packnum)==TRUE)   /*APP程序回退*/
                {
					Erase_Sector(ADDR_FLASH_SECTOR_4,1);   
                    ApplicationAddress = APP_Sector_1;
                    if(Jump_to_APP(ApplicationAddress)==FALSE)  //跳转至app区
                    {
                        HAL_Delay(1000);
                        SendErrorCode(0x05,0x01);
                    }
                }
            }
            else
            {
                SendErrorCode(0x02,0x04);                                /*不是该设备的bin文件不进行跳转*/
                SysState = Wait;                                         /*状态跳转至Wait,否则该错误码持续快速发送*/
//                ApplicationAddress = APP_Sector_1;                     /*app主程序区*/
//                Flash_Read(0x08020200,read_buf,1);                     /*此处与蔡工确认*//*该地址的数据为APP当中写入的 S02A*/
//                if(read_buf[0] == 0x41323053)                          /*读取app内容是否为可运行的app， 0x41323053 为 asc码 A20S*/
//                {
//                    if(Jump_to_APP(ApplicationAddress)==FALSE)
//                    {
//                        HAL_Delay(1000);                               /*此处与蔡工确认*/
//                        SendErrorCode(0x05,0x01);                      /*无法跳转至app区*/
//                    }
//                }
            }
            break;

        case Fault:
            break;
        }
    }
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
    if(HAL_TIM_Base_Init(&htim7) != HAL_OK)
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
#define frequency_of_led 50
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    int32_t TempValue;
    static uint32_t red_led_cnt = 0;
    static uint32_t green_led_cnt = 0;
    if(htim==(&htim7))                                  /*用于升级错误计时*/
    {
        red_led_cnt++;
        green_led_cnt++;
        if(Message_Flag==1 )   /*升级中绿灯闪烁*/
        {
            OTA_Time++;      /*升级超时时间，超过时间未收到数据，则进行回退*/
            if(RxMessage.StdId ==0x11)
            {
                if(green_led_cnt > frequency_of_led)
                {
                    green_led_cnt = 0;
                    GREEN_LED_TOGGLE;
                }
            }
        }
        if(red_led_cnt > frequency_of_led)              /*在boot中红灯闪烁*/
        {
            red_led_cnt = 0;
            RED_LED1_TOGGLE;
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
    sFilterConfig.FilterNumber = 15;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x602<<5;        /* 要过滤的ID高位 */
    sFilterConfig.FilterIdLow          = 0x0000;         /* 要过滤的ID低位 */
    sFilterConfig.FilterMaskIdHigh     = 0x602<<5;      /* 过滤器高16位每位必须匹配 */
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
    __HAL_RCC_GPIOE_CLK_ENABLE();
//	__HAL_RCC_GPIOD_CLK_ENABLE();
//
    GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	
	GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
  
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);

}

/* USER CODE BEGIN 4 */
/*------------------------------------------------
Function:代码备份，默认 0x8020000为运行代码区域，　0x8040000为备份代码区域
Input   :No
Output  :No
Explain :App_addr：运行代码区域  back_up_addr：代码备份区域 byte_num：擦除字节数 pack_num：数据包个数 只能按照扇区擦除
------------------------------------------------*/
uint8_t APP_Backup(uint32_t App_addr,uint32_t back_up_addr,uint32_t byte_num,uint8_t pack_num)		//备份程序
{
    uint16_t i,j;
    if(Erase_Sector(back_up_addr,byte_num)==TRUE) //擦除扇区
    {
        for(j=0; j<pack_num; j++)//包个数  每次写1K的数据 直到pack_num   128K 结束
        {
            for(i=0; i<PACK_SIZE; i++)//每包 1024个字节
            {
                flash_buf[i] = *(uint8_t*)App_addr; //需要备份的数据起始地址
                App_addr++;
            }
            if(Flash_Write(back_up_addr+j*PACK_SIZE,flash_buf,PACK_SIZE)==FALSE)//备份地址从back_up_addr地址开始往后每次写1K的数据
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
        Message_Flag=1;
        OTA_Time=0;                          /*收到CAN数据OTAtime清零，此处需要考虑CAN上面如果很多数据此处判断是否有效*/
        if(RxMessage.StdId == 0x10)          /*CAN ID:0x10 data:e0 xx 处理下位机发送指令*/
        {
            if(RxMessage.Data[0]==0xe0)
            {
                SoftReset();                                           /*软重启，该指令已经废弃*/
            }
            else if(RxMessage.Data[0]==0xe3)                           /*下位机需要发送备份命令*/
            {
				New_Bootloader_Flag=1;
            }
        }
		   else if(RxMessage.StdId == 0x602)                   /*为了防止boot升级过程中断电，boot不进行跳转等待升级指令，回复 CANID: 0x20 data: */
        {
            if(RxMessage.Data[0]==0x2F&&RxMessage.Data[1]==0x0B&&RxMessage.Data[2]==0x20&&RxMessage.Data[3]==0x00&&RxMessage.Data[4]==0x01)
            {
                buf_send[0] = 0xe2;
                buf_send[1] = 0x01;
                CANx_SendNormalData(&hcan2,0x20,buf_send,2);
            }
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
/*------------------------------------------------
* @function :1S发送一次错误帧
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/3/20
------------------------------------------------*/
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

    if(((*(__IO uint32_t*)APP_addr) & 0x2FFE0000 )== 0x20000000)
    {
        JumpAddress = *(__IO uint32_t*) (APP_addr + 4);
        Jump_To_Application = (pFunction) JumpAddress;
        __set_PRIMASK(1);
        __disable_irq();
        HAL_NVIC_DisableIRQ(SysTick_IRQn);
        HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);   //新增

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
uint8_t WriteUint32_tData_toFlash(uint32_t *data,uint32_t len,uint32_t address)
{
	HAL_FLASH_Unlock();
    uint32_t i=0;
    for(i=0; i<len; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)address, *data) == HAL_OK)
        {
            address = address+1;
            data = data+1;
        }
        else
        {
            return FALSE;
        }
    }
	HAL_FLASH_Lock();
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
    for(i=0; i<data_len; i++)
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
    for(i=0; i<data_len; i++)
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
void Flash_Read(uint32_t addresss,uint32_t *pBuffer,uint32_t bufsize)			//读取flash参数
{
    uint32_t i;
    for(i=0; i<bufsize; i++)
    {
        pBuffer[i] = *(long*)(addresss);
        addresss +=4;
    }
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
/*----------------------------
*Fuction：擦除某个地址所在的 flash 扇区
*Explain：No
----------------------------*/
int FlashEraseSector( uint32_t f_sector, uint8_t f_num_sectors )
{
    int ret = 1;
    uint32_t SectorError = 0;
    FLASH_EraseInitTypeDef eraseInit = {0};


    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.NbSectors = f_num_sectors;
    eraseInit.Sector = f_sector;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    /*进入临界区*/
//    __disable_irq();

    /*1解锁*/
    HAL_FLASH_Unlock();

    /*2擦除扇区*/
    HAL_FLASHEx_Erase(&eraseInit, &SectorError);

    /*3上锁*/
    HAL_FLASH_Lock();

    /*退出临界区*/
//    __enable_irq();

    if( 0xFFFFFFFFU != SectorError )
    {
        ret = -1;
    }
    return ret;
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
