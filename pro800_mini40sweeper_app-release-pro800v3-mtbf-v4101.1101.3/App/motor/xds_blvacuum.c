#include "xds_blvacuum.h"
#include "hc_usart.h"
#include "hc_gpio.h"
#include "normal_io.h"
#include "pc_correspond.h"


#define UART_SEND(buff, len)  do{\
 	hc_usart3_transmit(buff, len);\
     }while(0)
#define CMD_W_WATT  0x32
#define CMD_R_VERSION  0x21
#define CMD_R_CURRENT  0x22
#define CMD_R_WATT  0x24



typedef struct
{
    uint32_t last_rx_tick;
    uint32_t version;
    uint16_t current;
    uint16_t watt;
    int8_t temperature;
    uint8_t error;
	uint8_t rx_clean_cnt;  //接收到清除反馈计数
    uint8_t run_status;
    uint8_t over_current;
    uint8_t health;
}XdsStatus_t;


typedef struct
{
    uint8_t enable;
    uint16_t watt;
    uint16_t max_current; //mA
}XdsCmd_t;


static XdsCmd_t m_vacuum_cmd = {0,0,20000};
static XdsStatus_t m_vacuum_status = {0,0,0,0,0,0};
static uint8_t m_send_buff[16] = {0};
static uint8_t clean_error_flag = 0;



static uint8_t CalSum(const uint8_t *fp_cbuf, uint8_t f_len)
{
    uint8_t sum = 0;
    int i = 0;

    for(i=0;i<f_len;i++)
        sum += fp_cbuf[i];
        
    return sum;
}

static void SendCmd( uint8_t f_cmd, uint16_t f_data )
{
    uint8_t idx = 0;
    
    m_send_buff[idx++] = 0x55;
    m_send_buff[idx++] = 0xA1;
    m_send_buff[idx++] = f_cmd;
    m_send_buff[idx++] = (uint8_t)f_data;
    m_send_buff[idx++] = (f_data>>8)&0xFF;
    m_send_buff[idx] = CalSum(m_send_buff, idx );
    idx++;
    UART_SEND( m_send_buff, idx );
}

extern uint8_t m_vacuum_close_delay_s; //急停和钥匙开关响应后关吸风延时时间

static void VacuumMotor(void)
{
	static TickType_t s_last_run_tick = 0;
    static int16_t data = 0;//用于延时关闭	
	
	if( GetEmergencyStopState() || GetKeyState() == 0x02 || !GetPcConnectStatus())
	{
		if( xTaskGetTickCount() - s_last_run_tick >= 1000*m_vacuum_close_delay_s )//延时设定时间关闭吸风
			data = 0;
	}
	else if( m_vacuum_cmd.enable )
	{
		data = m_vacuum_cmd.watt;
		s_last_run_tick = xTaskGetTickCount();
	}
	else
	{
		data = 0;
	}
	
	if( data >=400 )
		data = 400;
	
    if( data )
        m_vacuum_status.run_status = 1;
    else
        m_vacuum_status.run_status = 0;
            
	SendCmd( CMD_W_WATT, data );
}

static void ReadStatus(void)
{
    static uint8_t s_idx = 0;
    
    s_idx++;
    
    switch( s_idx )
    {
        case 1:
            SendCmd( CMD_R_CURRENT, 0 );
            break;
        case 2:
            SendCmd( CMD_R_WATT, 0 );
            if( m_vacuum_status.version)
                s_idx = 0;
            break;
        default:
            SendCmd( CMD_R_VERSION, 0 );
            s_idx = 0;
            break;
    }
}

static void XdsClearError(void)
{
	SendCmd( 0x36, 0 );   //清除故障，用于电机故障时重启
}

static void CheckCurrent(void)
{
    static TickType_t s_vacuum_tick = 0;
    
    if( m_vacuum_status.current >= m_vacuum_cmd.max_current )
    {
        if( xTaskGetTickCount()-s_vacuum_tick >= 5000 )
            m_vacuum_status.over_current = 1;
    }
    else
        s_vacuum_tick = xTaskGetTickCount();
}

static uint32_t AnalyzeAlarm(uint8_t f_alarm)
{
    uint32_t alarm = 0;
    switch( f_alarm )
    {
        case 0x01://过流
            alarm = 1;
            break;
        case 0x03://过压
            alarm = 1<<1;
            break;
        case 0x04://欠压
            alarm = 1<<2;
            break;
        case 0x05://叶轮卡住
            alarm = 1<<3;
            break;
        case 0x07://电机超速
            alarm = 1<<4;
            break;
        case 0x08://未知命令
            alarm = 1<<5;
            break;
        case 0x09://过温
            alarm = 1<<6;
            break;
        case 0x0b://和校验错误
            alarm = 1<<7;
            break;
        case 0x0f://通讯超时
            alarm = 1<<8;
            break;
    }
    return alarm;
}

void HaldleXdsData(uint8_t fp_data[], uint16_t f_len)
{
    uint8_t sum = 0;
    
    if( fp_data!=NULL && f_len>=8 )
    {
        sum = CalSum( fp_data, 7 );
        if( sum==fp_data[7] )
        {
            switch( fp_data[2] )
            {
                case CMD_W_WATT:
                    m_vacuum_status.error = fp_data[3];
                    m_vacuum_status.temperature = fp_data[4];
                    break;
                case CMD_R_CURRENT:
                    m_vacuum_status.error = fp_data[3];
                    m_vacuum_status.temperature = fp_data[4];
                    m_vacuum_status.current = fp_data[5]|(fp_data[6]<<8);
                    break;
                case CMD_R_VERSION:
                    m_vacuum_status.version = (fp_data[4]<<16)|(fp_data[5]<<8)|fp_data[6];
                    break;
                case CMD_R_WATT:
                    m_vacuum_status.error = fp_data[3];
                    m_vacuum_status.temperature = fp_data[4];
                    m_vacuum_status.watt = fp_data[5]|(fp_data[6]<<8);
                    break;
                case 0x34: //调节中
                    m_vacuum_status.error = fp_data[3];
                    m_vacuum_status.temperature = fp_data[4];
                    break;
				case 0x36: //收到故障清除反馈
					m_vacuum_status.rx_clean_cnt++;			
                    break;
                default:
                    break;
            }
            m_vacuum_status.last_rx_tick = xTaskGetTickCount();
        }
    }
}

void SetXdsVacuumPwm(uint8_t f_enable, uint32_t f_pwm )
{
    m_vacuum_cmd.enable = f_enable;
    m_vacuum_cmd.watt = f_pwm*4;
}

uint16_t GetXdsVacuumCur(void)
{
    return m_vacuum_status.current/10;
}

uint8_t GetXdsVacuumSt(void)
{
    return m_vacuum_status.health&m_vacuum_status.run_status;
}

uint8_t GetXdsVacuumHealth(void)
{
    return m_vacuum_status.health;
}

uint8_t GetXdsVacuumOC(void)
{
    return m_vacuum_status.over_current;
}

void ClearXdsVacuumOC(void)
{
    m_vacuum_status.over_current = 0;
}

uint32_t GetXdsVacuumVersion(void)
{
    return m_vacuum_status.version;
}

uint32_t GetXdsAlarm(void)
{
    uint32_t alarm = 0;
    alarm = AnalyzeAlarm( m_vacuum_status.error )|(!m_vacuum_status.health<<9);
    return alarm;
}

void GetXdsDebugData(uint16_t fp_data[])
{
    if( fp_data!=  NULL )
    {
        fp_data[0] = m_vacuum_status.temperature;
        fp_data[1] = m_vacuum_status.run_status;
        fp_data[2] = m_vacuum_status.watt;
        fp_data[3] = m_vacuum_status.error;
        fp_data[4] = m_vacuum_cmd.enable;
        fp_data[5] = m_vacuum_cmd.watt;
    }
}

void XdsDebug(uint8_t f_cmd)
{
    switch( f_cmd )
    {
        case 1:
            SetXdsVacuumPwm(1, 5);
            break;
        case 2:
            SetXdsVacuumPwm(0, 0);
            break;
    }
}

void SetXdsCleanError(void)
{
	clean_error_flag=1;
}

void SetXdsMcuErrorCode(uint16_t f_code)
{
    if( f_code==0 )
        m_vacuum_status.error = 1;
    else if( f_code==1 )
        m_vacuum_status.error = 3;
    else if( f_code==2 )
        m_vacuum_status.error = 4;
    else if( f_code==3 )
        m_vacuum_status.error = 5;
    else if( f_code==4 )
        m_vacuum_status.error = 7;
    else if( f_code==5 )
        m_vacuum_status.error = 8;
    else if( f_code==6 )
        m_vacuum_status.error = 9;
    else if( f_code==7 )
        m_vacuum_status.error = 0xB;
    else if( f_code==8 )
        m_vacuum_status.error = 0xF;
    else if( f_code==9 )
        m_vacuum_status.error = 0;
}
#include "nanopb_tcpip.h"
extern struct DEVICE_DATA      DeviceDATA ;
extern struct OTHER_DEVICE_DATA  Other_DeviceData;

void XdsTask(void)
{
    static uint8_t s_flag = 0;
    DeviceDATA.xds_motor_temperature = m_vacuum_status.temperature;
	Other_DeviceData.xds_driver_info = m_vacuum_status.error<<24 | m_vacuum_status.temperature<<16 |  m_vacuum_status.watt;
	if(clean_error_flag)
	{
		XdsClearError();
		clean_error_flag=0;
	}
	else if( !s_flag ) 
	{
        s_flag=1;
        VacuumMotor();
    }
    else
    {
        s_flag=0;
        ReadStatus();
    }
        
    CheckCurrent();
    
    if( xTaskGetTickCount()-m_vacuum_status.last_rx_tick >= 3000 )
        m_vacuum_status.health = 0;
    else
        m_vacuum_status.health = 1;
}





