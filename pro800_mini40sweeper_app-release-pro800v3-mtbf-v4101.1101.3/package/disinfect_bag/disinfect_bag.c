#include "disinfect_bag.h"
#include "iwdg_task.h"
#include "normal_io.h"
#include "pc_correspond.h"
#include "nanopb_tcpip.h"
#include "main.h"
#include "power_ctr.h"

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

#define ID_MIST_SPRAY       0xC000100
#define ID_PERISTALTIC_PUMP 0xC000200
#define ID_FAN              0xC000300
#define ID_BEEP             0xC000400

#define ID_ALL_STATUS       0xC008000




#pragma pack(push) 
#pragma pack(1)
typedef struct
{
	uint8_t spray_level;
	uint8_t spray_distance;
    uint8_t beep;
    uint8_t pump;
}DisinfectCmd_t;
typedef struct
{
	uint8_t ska1;
	uint8_t ska2;
	uint8_t ska3;
	uint8_t sleft_fan;
	uint8_t sright_fan;
	union
	{
		uint8_t level;
		struct
		{
			uint8_t llow_level :1;
			uint8_t low_level :1;
			uint8_t high_level:1;
			uint8_t :5;
		}level_bits;
	};
	uint8_t sbeep;
	uint16_t stemperature;
}DisinfectStatus_t;
#pragma pack(pop)

uint8_t g_using_disinfect_flag = 0;//1使用消杀背包

static DisinfectCmd_t m_disinfect_cmd = {0};
static DisinfectStatus_t m_disinfect_status = {0};

static uint32_t m_last_rx_tick = 0;

static uint8_t m_disinfect2_0_cmd = 0;//消杀降本控制命令，0不启动，1启动

//控制喷雾量
static void CtrlMistSpray(void)
{
    uint8_t data[2] = {0};
    
    switch( m_disinfect_cmd.spray_level )
    {
        case 0:
            data[0] = 0;
            data[1] = 0;
            break;
        case 1:
            data[0] = 1;
            data[1] = 0;
            break;
        case 2:
            data[0] = 0;
            data[1] = 1;
            break;
        case 3:
            data[0] = 1;
            data[1] = 1;
            break;
    }
    
    if( GetEmergencyStopState() || GetKeyState() == 0x02 || !GetPcConnectStatus() )
    {
        data[0] = 0;
        data[1] = 0;
    }
    
    Can2SendExt(ID_MIST_SPRAY, data, 2);
}

//控制距离
static void CtrlFan(void)
{
    uint8_t data[2] = {0};
    
    data[0] = m_disinfect_cmd.spray_distance;
    data[1] = m_disinfect_cmd.spray_distance;
    
    if( GetEmergencyStopState() || GetKeyState() == 0x02 || !GetPcConnectStatus() )
    {
        data[0] = 0;
        data[1] = 0;
    }
    
    Can2SendExt(ID_FAN, data, 2);
}

//控制蜂鸣器
static void CtrlBeep(void)
{
    uint8_t data[2] = {0};
    
    data[0] = m_disinfect_cmd.beep;
    
    if( GetEmergencyStopState() || GetKeyState() == 0x02 || !GetPcConnectStatus() )
    {
        data[0] = 0;
    }
    
    Can2SendExt(ID_BEEP, data, 2);
}

//控制蠕动泵
static void CtrlPump(void)
{
    uint8_t data[2] = {0};
    
    data[0] = m_disinfect_cmd.pump;
    
    if( GetEmergencyStopState() || GetKeyState() == 0x02 || !GetPcConnectStatus() )
    {
        data[0] = 0;
    }
    
    Can2SendExt(ID_PERISTALTIC_PUMP, data, 2);
}

//读取状态
static void ReadAllStatus(void)
{
    uint8_t data[2] = {0};
    
    Can2SendExt(ID_ALL_STATUS, data, 0);
}

//运行消杀通讯任务
void DisinfectRun(void)
{
    static uint8_t s_idx = 0;
    
    s_idx++;
    switch( s_idx )
    {
        case 1:
            CtrlMistSpray();
            break;
        case 2:
            CtrlFan();
            break;
        case 3:
            CtrlBeep();
            break;
        case 4:
            CtrlPump();
            break;
        default:
            ReadAllStatus();
            s_idx = 0;
            break;
    }
}

static void Disinfect2_0Task(void)
{
    if(m_disinfect2_0_cmd)
    {
        SetDisinfectRelay(1);
    }
    else
    {
        SetDisinfectRelay(0);
    }
}

void HandleDisinfectData(CanRxMsg_t* fp_rx_msg)
{

//    switch( fp_rx_msg->head.ExtId )
//    {
//        case ID_MIST_SPRAY+1:
//            m_disinfect_status.ska1 = fp_rx_msg->Data[0];
//            m_disinfect_status.ska2 = fp_rx_msg->Data[1];
//            break;
//        
//        case ID_PERISTALTIC_PUMP+1:
//            m_disinfect_status.ska3 = fp_rx_msg->Data[0];
//            break;
//        
//        case ID_FAN+1:
//            m_disinfect_status.sleft_fan = fp_rx_msg->Data[0];
//            m_disinfect_status.sright_fan = fp_rx_msg->Data[1];
//            break;
//        
//        case ID_BEEP+1:
//            m_disinfect_status.sbeep = fp_rx_msg->Data[0];
//            break;

//        case ID_ALL_STATUS+1:
//            m_disinfect_status.ska1 = fp_rx_msg->Data[0];
//            m_disinfect_status.ska2 = fp_rx_msg->Data[1];
//            m_disinfect_status.ska3 = fp_rx_msg->Data[2];
//            m_disinfect_status.sleft_fan = fp_rx_msg->Data[3];
//            m_disinfect_status.sright_fan = fp_rx_msg->Data[4];
//            m_disinfect_status.level = fp_rx_msg->Data[5];
//            m_disinfect_status.sbeep = fp_rx_msg->Data[6];
//            break;
//        case ID_ALL_STATUS+2:
//            m_disinfect_status.stemperature = fp_rx_msg->Data[0]|(fp_rx_msg->Data[1]<<8);
//            break;
//        default:
//            break;
//    }
//    if( (fp_rx_msg->head.ExtId&0xC000000)==0xC000000 )
//        m_last_rx_tick = xTaskGetTickCount();
}

void SetDisinfectMistSpray(uint8_t f_cmd)
{
    if( g_using_disinfect_flag ==1 )
    {
        if( f_cmd <= 3 )
        {
            m_disinfect_cmd.spray_level = f_cmd;
        }
    }
    else
        m_disinfect_cmd.spray_level = 0;
    m_disinfect2_0_cmd = f_cmd;
}

void SetDisinfectSprayDistance( uint8_t f_cmd)
{
    if( g_using_disinfect_flag == 1 )
    {
        if( f_cmd <= 100 )
        {
            m_disinfect_cmd.spray_distance = f_cmd;
        }
    }
    else
        m_disinfect_cmd.spray_distance = 0;
}

void SetDisinfectBoxBeep( uint8_t f_cmd )
{
    if( g_using_disinfect_flag == 1 )
        m_disinfect_cmd.beep = f_cmd;
    else
        m_disinfect_cmd.beep = 0;
}

void SetDisinfectBoxPump( uint8_t f_cmd )
{
    if( g_using_disinfect_flag == 1 )
        m_disinfect_cmd.pump = f_cmd;
    else
        m_disinfect_cmd.pump = 0;
}

void ConfigDisinfectEnable( uint8_t f_cmd )
{
    g_using_disinfect_flag = f_cmd;
}

void SetDisinfect2_0Work(uint8_t f_cmd)
{
    m_disinfect2_0_cmd = f_cmd;
}

uint8_t GetDisinfectMistSprayLevel(void)
{
    uint8_t level = 0;
    
    if( m_disinfect_status.ska1 && m_disinfect_status.ska2 )
        level = 3;
    else if( m_disinfect_status.ska2 )
        level = 2;
    else if( m_disinfect_status.ska1 )
        level = 1;
    else
        level = 0;
    return level;
}

uint8_t GetDisinfectSprayDistance(void)
{
    return m_disinfect_status.sleft_fan;
}

uint8_t GetDisinfectDisinfectantLevel(void)
{
    uint8_t level = 0;
    
    level = m_disinfect_status.level;

    return level;
}

uint8_t GetDisinfectBeepStatus(void)
{
    return m_disinfect_status.sbeep;
}

uint8_t GetDisinfectPumpStatus(void)
{
    return m_disinfect_status.ska3;
}

uint16_t GetDisinfectTemperature(void)
{
    return m_disinfect_status.stemperature;
}

uint8_t GetDisinfectDisconnectStatus(void)
{
    if( g_using_disinfect_flag == 1 )
    {
        if( xTaskGetTickCount()-m_last_rx_tick >=1000 )
        {
            m_disinfect_status.sbeep = 0;
            m_disinfect_status.ska1 = 0;
            m_disinfect_status.ska2 = 0;
            m_disinfect_status.ska3 = 0;
            m_disinfect_status.sleft_fan = 0;
            m_disinfect_status.sright_fan = 0;
            return 1;
        }
        else
            return 0;
    }
    else
        return 0;
}

void GetDisinfectDebugData(uint8_t* fp_data)
{
    if( fp_data!=NULL )
    {
        fp_data[0] = m_disinfect_cmd.spray_level;
        fp_data[1] = m_disinfect_cmd.spray_distance;
        fp_data[2] = m_disinfect_cmd.beep;
        fp_data[3] = m_disinfect_cmd.pump;
    }
}

void DisinfectDebug(uint8_t f_cmd)
{
    switch(f_cmd)
    {
        case 1:
            DeviceCommandCMD.disinfect_mist_spray_level = 1;
            //SetDisinfectMistSpray(1);
        break;
        case 2:
            DeviceCommandCMD.disinfect_mist_spray_level = 3;
            //SetDisinfectMistSpray(3);
        break;
        case 3:
            DeviceCommandCMD.disinfect_mist_spray_level = 0;
            //SetDisinfectMistSpray(0);
        break;
        case 4:
            DeviceCommandCMD.disinfect_spray_distance = 0;
            //SetDisinfectSprayDistance(0);
            break;
        case 5:
            DeviceCommandCMD.disinfect_spray_distance = 30;
            //SetDisinfectSprayDistance(30);
            break;
        case 6:
            DeviceCommandCMD.disinfect_spray_distance = 40;
            //SetDisinfectSprayDistance(40);
        break;
        case 7:
            DeviceCommandCMD.disinfect_spray_distance = 60;
            //SetDisinfectSprayDistance(60);
        break;
        case 8:
            DeviceCommandCMD.disinfect_spray_distance = 80;
            //SetDisinfectSprayDistance(80);
        break;
        case 9:
            DeviceCommandCMD.disinfect_spray_distance = 100;
            //SetDisinfectSprayDistance(100);
        break;
        case 10:
            DeviceCommandCMD.disinfect_box_beep = 1;
            //SetDisinfectBoxBeep(1);
            break;
        case 11:
            DeviceCommandCMD.disinfect_box_beep = 0;
            //SetDisinfectBoxBeep(0);
            break;
        case 12:
            DeviceCommandCMD.disinfect_disinfectant_pump = 1;
            //SetDisinfectBoxPump(1);
            break;
        case 13:
            DeviceCommandCMD.disinfect_disinfectant_pump = 0;
            //SetDisinfectBoxPump(0);
            break;
        case 14:
            ConfigCommandCMD.disinfect_enable = 1;
            //ConfigDisinfectEnable(1);
            break;
        case 15:
            ConfigCommandCMD.disinfect_enable = 0;
            //ConfigDisinfectEnable(0);
            break;
        case 16:
            SetDisinfect2_0Work(0);
            break;
        case 17:
            SetDisinfect2_0Work(1);
            break;
        default:
            break;
        
    }
}

void DisinfectTask(void const *pvParameters)
{
    TickType_t lastWakeTime;
	const TickType_t frequency = pdMS_TO_TICKS(50);
	uint32_t bitPos = (uint32_t)pvParameters;
    uint8_t thread_num = (uint8_t)(uint32_t)pvParameters;
    
	/* 获取当前的系统时间 */
    lastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        /* 喂狗 */
//		FeedDog(bitPos);
        /* vTaskDelayUntil是绝对延迟，vTaskDelay是相对延迟。*/
        vTaskDelay( frequency);
        
//        DisinfectRun();
        if(g_using_disinfect_flag == 0)
		{
			Disinfect2_0Task();  //不可屏蔽已做处理，配置正确时，不影响消杀3.0控制
		}
        
//        g_thread_call_count[thread_num]++;
    }
}


