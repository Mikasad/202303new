/********************************************************************************
*文件名			: drv_h4aa30_usb_iap.c
*简  介			: 混合驱控板U盘升级驱动  
*日	 期			: 2021/8/3
*作  者			: 刘德明/高仙     
********************************************************************************
*备注： 
 
********************************************************************************/
#include "drv_h4aa30_usb_iap.h"


 
ApplicationTypeDef Appli_state = APPLICATION_IDLE;
extern __USB_ALIGN_BEGIN usb_core_instance      usb_app_instance;
extern __USB_ALIGN_BEGIN USBH_HOST              usb_app_host;

uint32_t otaing_tick = 0;
H4AA_UPGRADE_t upgrade = {0};

/*驱动器升级开始指令*/
static void USB_upgrade_start_cmd(void  );                         
static uint16_t crc16_modbus(uint8_t *data, uint16_t length);     /*crc校验*/
static int can_ota_send(CM_CAN_TypeDef *CANx, uint32_t id, void* f_data, uint8_t data_len);
static void USB_Task(void *pvParameters );

/*将U盘中的bin文件发送给驱动器*/ 
uint8_t send_Udisk_bin_to_flash(void);

/****************************************************************
*介	 绍	：msc回调函数
****************************************************************/
int host_user_msc_app(void)
{
//	printf("enter host_user_msc_app\r\n");
	Appli_state = APPLICATION_READY;
    return ((int)0);
}

void host_user_devdisconn(void)
{
#if (LL_PRINT_ENABLE == DDL_ON)
    DDL_Printf(MSG_DEV_DISCONNECTED);
#endif
	Appli_state = APPLICATION_DISCONNECT;
}


/************************************************************************************
*介	 绍	：创建U盘升级任务
************************************************************************************/
void USB_IAP_Task_Init(void)
{	
	xTaskCreate( USB_Task, "usb_upgrade", 2048, 0, 2, NULL );
}
/************************************************************************************
*介	 绍	：U盘升级任务
*************************************************************************************/
static void USB_Task(void *pvParameters )
{ 
	uint8_t upgrade_flg = 1;
	uint32_t usb_ota_rst = 0;
	
 	while(!upgrade.driver_init_flg)  /*USB驱动初始化完成*/
	{
		vTaskDelay(50);
	}
	
	while(1)
	{
	
		if(Appli_state==APPLICATION_READY && upgrade_flg )          /*检测到U盘进行程序升级*/
		{
 			upgrade_flg = 0;
			vTaskDelay(500);
  			usb_ota_rst = send_Udisk_bin_to_flash();
			
			if( usb_ota_rst == 0)
			{
				for(uint8_t i=0;i<3;i++)  //升级成功响三声
				{
					vTaskDelay(500);
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
					vTaskDelay(500);
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
				}
			}	
			memset(&upgrade,0,sizeof(upgrade));
 		}
		
		if(Appli_state == APPLICATION_DISCONNECT)
		{
			upgrade_flg = 1;
		}

		vTaskDelay(2);
	}
}



/****************************************************************
*功  能:将U盘中的bin文件发送给驱动器
*****************************************************************/ 
uint8_t send_Udisk_bin_to_flash(void)
{ 
	FIL   MyFile;
	FATFS usb_fatfs;  
 	uint32_t size_all = 0;
   	uint8_t can_send_buf[8] = {0};
	/* Mount Udisk */
    if(f_mount(&usb_fatfs, "", 1) != FR_OK)
    {				
		usb_iap_log("f_mount error!");
		return 1;
    }
	/* Open the binary file to be downloaded */
	if(f_open(&MyFile, DOWNLOAD_FILENAME, FA_READ) != FR_OK)
	{
		usb_iap_log("f_open error!"); 
		return 2;
	}
	
	upgrade.file_size = f_size(&MyFile);
	upgrade.pack_num = upgrade.file_size/BIN_FILE_BUFFER_SIZE;
	(upgrade.file_size%BIN_FILE_BUFFER_SIZE)?upgrade.pack_num++:upgrade.pack_num;
	
	usb_iap_log("app bin file size =%ld bytes.",f_size(&MyFile));
	
	while(1)
	{
 		memset(upgrade.pack_buff,0,BIN_FILE_BUFFER_SIZE);
		/* Read binary file to Buff */
		uint32_t result = f_read(&MyFile,&upgrade.pack_buff[0],BIN_FILE_BUFFER_SIZE, &upgrade.read_pack_size );
		upgrade.read_pack_cnt ++;
		
		if(size_all >= f_size(&MyFile))
		{
			upgrade.ota_flag = 2;
			usb_iap_log("ota finish!");		
			
			f_close(&MyFile);
			f_mount(NULL, (TCHAR const*)"", 0);
			return 0;
		}
		
		if(result != FR_OK)    
		{
			usb_iap_log("f_read error!");
			f_close(&MyFile);
			f_mount(NULL, (TCHAR const*)"", 0);	
			return 6;		
		}
	
		if(upgrade.read_pack_cnt == 1)
		{
			if(upgrade.pack_buff[0x200]!='H'||upgrade.pack_buff[0x201]!='4'||upgrade.pack_buff[0x202]!='A'||upgrade.pack_buff[0x203]!='A')
			{
				usb_iap_log("bin file error!");
				f_close(&MyFile);
				f_mount(NULL, (TCHAR const*)"", 0);
				return 5;
			}
			else
			{
				otaing_tick = xTaskGetTickCount();
				vTaskDelay(100);	
				USB_upgrade_start_cmd();
				upgrade.ota_flag = 1;
			}	
		}
			
		/*最后一包需要保证为8的整数倍，不足的补0XFF*/
		if(upgrade.read_pack_size < 1024 || (upgrade.read_pack_cnt == upgrade.pack_num))        
		{
			uint8_t remainder = upgrade.read_pack_size%8;
			
			if(remainder)
			{
				upgrade.read_pack_size = upgrade.read_pack_size + 8 - remainder;
				
				for(uint16_t i=upgrade.read_pack_size+remainder-8-1; i<upgrade.read_pack_size;i++)
				{
					upgrade.pack_buff[i] = 0xFF;
				}   
			}
		}	
		
		upgrade.pack_crc = crc16_modbus(upgrade.pack_buff,upgrade.read_pack_size);
		size_all += upgrade.read_pack_size;		
		usb_iap_log("write app finishing %d%%.",(int)(size_all*100/(float)f_size(&MyFile)));
		upgrade.wait_response_tick = xTaskGetTickCount();
		
		while (1)
		{
			otaing_tick = xTaskGetTickCountFromISR();
			//等待驱动器响应，超时则退出升级
			if(xTaskGetTickCount() - upgrade.wait_response_tick > MAX_OTA_WAIT_MS)
			{
				usb_iap_log("time out\r\n");
				f_close(&MyFile);
				f_mount(NULL, (TCHAR const*)"", 0);
				return 5;	
			}
			 
			//收到驱动器响应则发送固件到驱动器
			if(upgrade.request_primary_num && upgrade.request_primary_num <= upgrade.pack_num)
			{
				//收到驱动器响应则重置超时时间
				upgrade.wait_response_tick = xTaskGetTickCount();
				
				if(upgrade.request_secondary_num < (upgrade.read_pack_size/8)) //
				{
					for(uint8_t i=0;i<8;i++)
					{
						can_send_buf[i] = upgrade.pack_buff[(upgrade.request_secondary_num*8)+i];
					}
					upgrade.request_primary_num =  0  ;
					upgrade.request_secondary_num = 0 ;	
					can_standard_send(CAN1_UNIT, 0x11, &can_send_buf, 0x08);		
				}
				else //一大包发完需要发校验
				{
					can_send_buf[0] = upgrade.pack_crc&0xFF;
					can_send_buf[1] = (upgrade.pack_crc>>8)&0xFF;
					upgrade.request_primary_num =  0  ;
					upgrade.request_secondary_num = 0 ;
					can_standard_send(CAN1_UNIT, 0x12, &can_send_buf, 0x02);	
					break;
				}
			}
			
			vTaskDelay(1);		
		}
	}
	  
	f_close(&MyFile);
	f_mount(NULL, (TCHAR const*)"", 0);

	return 0;
}
/************************************************************************************
*介	 绍	：驱动器升级开始指令
*************************************************************************************/
static void USB_upgrade_start_cmd(void)
{ 
	static uint8_t  data[8]={0x2f,0x0B,0x20,0x00,0x01,0x00,0x00,0x00};
	can_standard_send(CAN1_UNIT, 0x601, data, 0x08);		
}

/**********************************************************************************************
*介	 绍	：驱动器OTA数据接收处理	 
***********************************************************************************************/
void H4aa_OTA_RxCpltCallback(uint32_t id,uint8_t* p_data)
{
	uint16_t i;
	uint8_t buftex[8] = {0};
    
	
	if(Appli_state != APPLICATION_READY)
	{
//		printf("Appli_state != APPLICATION_READY\r\n");
		return;
	}
 	otaing_tick = xTaskGetTickCountFromISR();
	if(id==0x20)
	{
		switch (p_data[0])
		{

			case 0xe2:
				buftex[0] = 0xe3;
				buftex[1] = 0x01;
			if(upgrade.file_size)
			{
				can_ota_send(CAN1_UNIT, 0x10, &buftex, 0x02);	//收到E2 01 返回E3 01		
			}
				break;
			case 0xe4:
				buftex[0] =	0X01;
				can_ota_send(CAN1_UNIT, 0x14, &buftex, 0x01);  //收到E4 01 返回 01	
 			case 0xff:
				break;
		}
	}
	else if(id==0x21) //
	{
			
		upgrade.request_primary_num = p_data[0];                
		upgrade.request_secondary_num =  p_data[1];              
		if(upgrade.request_primary_num <= upgrade.pack_num)
		{
			upgrade.wait_response_tick = xTaskGetTickCount();
		}
		else if(upgrade.request_primary_num>upgrade.pack_num)
		{
				buftex[0] =	5;
				buftex[1] = 0;
				buftex[2] = 1;
				buftex[3] = 'S';
				buftex[4] = upgrade.file_size&0xff;
				buftex[5] = upgrade.file_size>>8&0xff;
				buftex[6] = upgrade.file_size>>16&0xff;
				buftex[7] = upgrade.file_size>>24&0xff;
				can_ota_send(CAN1_UNIT, 0x13, &buftex, 0x08);		
		}
	} 
}


void UsbBeep(void)
{
	static uint32_t run_tick = 0;
	
	if(upgrade.ota_flag == 0) //未升级
	{
		run_tick =  xTaskGetTickCount();
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	}   
    else if( upgrade.ota_flag == 1 )  //拷贝中蜂鸣器滴答声
    {
		if( xTaskGetTickCount() - run_tick > 500)
		{
			run_tick =  xTaskGetTickCount();
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}
    }
    else if( Appli_state != APPLICATION_READY )//U盘拔出后蜂鸣器不响
    {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    }
	
	upgrade.driver_init_flg = 1; //UsbBeep被调用,说明初始化已经完成了
}

/**********************************************************************************************
*介	 绍	：crc16 modbus数据校验	 
***********************************************************************************************/
static uint16_t crc16_modbus(uint8_t *data, uint16_t length)
{
    uint8_t i;
    uint16_t crc = 0xffff;        // Initial value
    while(length--)
    {
        crc ^= *data++;            // crc ^= *data; data++;
        for (i = 0; i < 8; ++i)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;        // 0xA001 = reverse 0x8005
            else
                crc = (crc >> 1);
        }
    }
    return crc;
}

static int can_ota_send(CM_CAN_TypeDef *CANx, uint32_t id, void* f_data, uint8_t data_len)
{
    uint8_t i;
    stc_can_tx_frame_t stcTx1 = {0};
    uint8_t* p_data = f_data;
	if(data_len > 8)
	{
		data_len = 0;
	}
    stcTx1.u32Ctrl = 0x0UL;
    stcTx1.u32ID   = id;
    stcTx1.IDE     = 0;            /*0标准帧 1扩展帧*/
    stcTx1.DLC     = data_len;     /*帧数据长度*/
	
    for (i = 0U; i < data_len && data_len<9; i++)
    {
        stcTx1.au8Data[i] = p_data[i];
    }
	 
    /* Transmit frames via PTB and STB */
    (void)CAN_FillTxFrame(CANx, CAN_TX_BUF_PTB, &stcTx1);
    CAN_StartTx(CANx, CAN_TX_REQ_PTB);
    CAN_StartTx(CANx, CAN_TX_REQ_STB_ALL);


}