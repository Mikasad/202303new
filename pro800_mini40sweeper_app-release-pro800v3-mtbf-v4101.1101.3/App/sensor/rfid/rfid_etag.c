#include "rfid_etag.h"
#include "hc_usart.h"
#include "iwdg_task.h"
//#include "ctr_move_task.h"
#include <string.h>
#include "main.h"

//#define UART_SEND(buff, len)  HAL_UART_Transmit_DMA(&huart4, buff, len )
#define UART_SEND(buff, len)  hc_usart4_transmit(buff, len )

#define PRESET_VALUE 0xFFFF
#define POLYNOMIAL 0x8408

#define ST_R_COMPLETE  0x01 //命令执行结束， 同时返回询查到的电子标签数据
#define ST_R_TIMEOUT   0x02 //询查时间结束， 命令执行强制退出， 同时返回已询查到的标签数据
#define ST_R_SEND_PART 0x03 //如果读到的标签数量无法在一条消息内传送完， 将分多次发送。 如果 Status 为 0x03， 则表示这条数据结束后， 还有数据
#define ST_R_NOT_ALL   0x04 //还有电子标签未读取， 电子标签数量太多， 读写器的存储区已满， 返回此状态值， 同时返回已询查到得电子标签数据

#define CMD_QUERY      0x01 //命令号，查询标签


uint8_t g_enable_rfid = 0;

static uint8_t m_cmd_read_tag[5] = {0x04,0x00,0x01,0xDB,0x4B}; //读取命令
static uint8_t m_protector_trigger = 0;  //是否检测到危险提示标签
static RfidTag_t m_rfid_tag_table[RFID_TAG_MAX_NUM] = {0}; //存储单次读取到的RFID标签数据
static uint8_t m_disconnect_count = 0;  //读取RFID连续失败次数
static uint8_t m_tag_num = 0; //单次读取到的标签数量


//crc校验
static uint16_t uiCrc16Cal(uint8_t const * pucY, uint8_t ucX)
{
	uint8_t ucI, ucJ;
	uint16_t uiCrcValue = PRESET_VALUE;
	for (ucI = 0; ucI < ucX; ucI++)
	{
		uiCrcValue = uiCrcValue ^ *(pucY + ucI);
		for (ucJ = 0; ucJ < 8; ucJ++)
		{
			if (uiCrcValue & 0x0001)
			{
				uiCrcValue = (uiCrcValue >> 1) ^ POLYNOMIAL;
			}
			else
			{
				uiCrcValue = (uiCrcValue >> 1);
			}
		}
	}
	return uiCrcValue;
}

//查询标签
static void ReadTag(void)
{
    UART_SEND(m_cmd_read_tag, 5);
    if( m_disconnect_count<40 )
        m_disconnect_count++;
}

//解析标签中EPC数据
static int AnalyzeEpc( const uint8_t fp_epc[], uint8_t f_idx )
{
    static uint8_t s_idx = 0;
    int ret = 0;
    
    if( !f_idx)
    {
        s_idx = 1;
    }
    
    if( fp_epc[0]==0x0A && fp_epc[1]==0x0B ) //急停标签
    {
        m_rfid_tag_table[0].id = 1;
        m_rfid_tag_table[0].tag_num = (fp_epc[0]<<24)|(fp_epc[1]<<16)|(fp_epc[2]<<8)|(fp_epc[3]<<0);

        if(GetCurCtrMode())
        {
            m_protector_trigger = 1;
        }
        m_tag_num++;
    }
    else if( s_idx<RFID_TAG_MAX_NUM )
    {
        m_rfid_tag_table[s_idx].id = s_idx+1;
        m_rfid_tag_table[s_idx].tag_num = (fp_epc[0]<<24)|(fp_epc[1]<<16)|(fp_epc[2]<<8)|(fp_epc[3]<<0);
        s_idx++;
        m_tag_num++;
    }
    else
        ret = -1;
    
    return ret;
}

//获取是否检测到危险环境提示标签
uint8_t GetRfidProtectFlag(void)
{
    return m_protector_trigger;
}

//清危险提示标志
void ClearRfidProtectFlag(void)
{
    m_protector_trigger = 0;
}

//0表示断链
uint8_t GetRfidHealthStatus(void)
{
	if( g_enable_rfid )
		return (uint8_t)(m_disconnect_count<40);
	else
		return 1;
}

void GetRfidData( RfidTag_t* fp_tags, uint16_t f_cpy_size )
{
    if( fp_tags != NULL )
    {
        taskENTER_CRITICAL();
        memcpy(fp_tags, m_rfid_tag_table, f_cpy_size);
        taskEXIT_CRITICAL();
    }
}


//3A 00 01 01 04 0C E2 00 00 1D 38 0E 01 44 12 30 70 AC 0C E2 00 00 1D 38 0E 01 14 12 30 56 8D 0C E2 00 00 1D 38 0E 01 58 12 30 82 2B 0C E2 00 00 1D 38 0E 01 16 12 30 56 8E C7 8C
//处理协议数据
void HandleRfidData(const uint8_t fp_data[], uint16_t f_len )
{
    uint16_t crc_r = 0;
    uint16_t crc_c = 0;
    uint8_t  tag_num = 0;
    uint8_t tag_size = 0;
    uint16_t tag_idx = 5;
    
    if( fp_data != NULL&&f_len!=0 )
    {
//		printf("RFID data:");
//		for (int i = 0; i < f_len; i++)
//			printf(" %02X", fp_data[i]);
//			printf("\r\n");
        crc_c = uiCrc16Cal( fp_data, f_len-2 );
        crc_r = ((uint16_t)fp_data[f_len-1]<<8)|fp_data[f_len-2];
        if( crc_r == crc_c )
        {
            memset(m_rfid_tag_table, 0, sizeof(m_rfid_tag_table));
            m_tag_num = 0;
            if( fp_data[2]==CMD_QUERY && fp_data[3]<=ST_R_NOT_ALL )
            {
                tag_num = fp_data[4];
                if( tag_num >RFID_TAG_MAX_NUM)
                    tag_num = RFID_TAG_MAX_NUM;

                for( int i=0;i<tag_num;i++)
                {
//                    if( fp_data[5+i*11] == 0x0A )
//                    {
//                        AnalyzeEpc( &fp_data[6+i*11], i );
//                    }
                    tag_size = fp_data[tag_idx];
                    AnalyzeEpc( &fp_data[tag_idx+1], i );
                    tag_idx += tag_size+1;
                }
            }
            
            m_disconnect_count = 0;
        }
    }
}    

//RFID任务
void RfidTask(void const *pvParameters)
{
    TickType_t last_wake_tick;
	const TickType_t c_frequency = pdMS_TO_TICKS(50);//xms进行一次任务处理
	uint32_t bitPos = (uint32_t)pvParameters;
    uint8_t thread_num = (uint8_t)(uint32_t)pvParameters;
	uint16_t len = 0;
    
    last_wake_tick = xTaskGetTickCount();
    
    while(1)
    {
    #if 0
        FeedDog(bitPos);
	#endif
        
        ReadTag();
        
        if( m_disconnect_count>=20)
            m_tag_num = 0;
        
//        g_thread_call_count[thread_num]++;
        
        vTaskDelay( c_frequency);
		len = BUF_UsedSize(&m_stcRingBuf_RS232_1);
		BUF_Read(&m_stcRingBuf_RS232_1, g_uart4RxBuf, len);
		if( len >= 6 )
		{
			HandleRfidData(g_uart4RxBuf, len);
		}
    }
}



