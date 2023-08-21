#include "zd_driver.h"
#include "normal_io.h"
#include "pc_correspond.h"
//#include "iwdg_task.h"
#include "xds_blvacuum.h"
#include "nanopb_tcpip.h"
#include "hc_usart.h"
#include "hc_gpio.h"
#include "stdlib.h"
#define ADDR_ROLLER_MOTOR   0x02
#define ADDR_LEFT_MOTOR     0x03
#define ADDR_RIGHT_MOTOR    0x04

#define BM_ADDR_LEFT_MOTOR     0x04
#define BM_ADDR_RIGHT_MOTOR    0x05

#define TIMEOUT_458  50
#define DISCONNECT_TIME_MS  1000



static uint32_t UART_SEND(uint8_t* buff, uint16_t len)  
{ 
	hc_usart3_transmit(buff, len);

	return 0;
}
typedef struct
{
    uint32_t last_rx_tick;
    uint16_t status1;
    uint16_t status2;
    uint16_t hitch;
    int16_t current;
    uint16_t timeout485;
    uint8_t  run_status;
    uint8_t  flag;
    uint8_t  over_current;
	uint16_t speed;
}MotorStatus_t;

typedef struct
{
    uint16_t target_pwm; //0~3000
    uint16_t max_current; //0.01A
    uint8_t enable;
    uint8_t dir;
}MotorCmd_t;

typedef enum
{
    UNKNOW = 0,
    ZHONGDA,
}DriverType_t;


static DriverType_t m_driver_type = ZHONGDA;
static MotorCmd_t m_roller_cmd = {0,1500, 0};
static MotorCmd_t m_left_brush_cmd = {0,300,0};
static MotorCmd_t m_right_brush_cmd = {0,300,0};

static MotorStatus_t m_roller_status = {0};
static MotorStatus_t m_left_brush_status = {0};
static MotorStatus_t m_right_brush_status = {0};

static uint8_t m_send_error_count = 0;
static uint8_t m_send_buff[32] = {0};

static uint8_t m_send_complete_flag = 1; //非零表示本次发收完成

static uint8_t side_motor_type =0;       //0中大 1本末

static uint8_t param_read_type =0;       //1读驱动器状态 2读驱动器转速

//状态读取收发实例
//[13:58:07.073]发→◇01 03 21 00 00 03 0F F7 □
//[13:58:07.084]收←◆01 03 06 00 03 00 41 00 00 35 61 
//电机配置收发实例
//[13:59:17.818]发→◇01 10 20 00 00 02 04 00 01 00 FF 7B EE □
//[13:59:17.830]收←◆01 10 20 00 00 02 4A 08 

/****rtu crc16***/

/* Table of CRC values for high–order byte */
const uint8_t crctablehi[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40
};
/* Table of CRC values for low–order byte */
const uint8_t crctablelo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
	0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
	0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
	0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
	0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
	0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
	0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
	0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
	0x40
};

static void ParamInit(void)
{
    m_roller_status.timeout485 = 1;//初始化为1，上电后从驱动器读取上来的值非1
    m_left_brush_status.timeout485 = 1;
    m_right_brush_status.timeout485 = 1;
    m_roller_cmd.dir = 0x02;
    m_left_brush_cmd.dir = 0x01;
    m_right_brush_cmd.dir = 0x02;
    m_roller_status.status1 = 3;
    m_left_brush_status.status1 = 3;
    m_right_brush_status.status1 = 3;
}

static uint16_t crc16table(uint8_t *ptr, uint16_t len)
{
	uint8_t crchi = 0xff;
	uint8_t crclo = 0xff;
	uint16_t index;
	while (len--)
	{
		index = crclo ^ *ptr++;
		crclo = crchi ^ crctablehi[index];
		crchi = crctablelo[index];
	}
	return (crchi << 8 | crclo);
}

uint8_t crc8_MAXIM(uint8_t *data, uint8_t len)
{
    uint8_t crc, i;
    crc = 0x00;

    while(len--)
    {
        crc ^= *data++;
        for(i = 0;i < 8;i++)
        {
            if(crc & 0x01)
            {
                crc = (crc >> 1) ^ 0x8c;
            }
                else crc >>= 1;
        }
    }
    return crc;
}


//本末设置pwm
static void BmSetPwm(uint8_t f_addr, int16_t speed )
{
    uint8_t  idx = 0;
    uint16_t crc = 0;
    
    m_send_buff[idx++] = f_addr;
    m_send_buff[idx++] = 0x64;  
    m_send_buff[idx++] = (uint8_t)(speed>>8);  
    m_send_buff[idx++] = (uint8_t)speed; 
	m_send_buff[idx++] = 0x00; //	
	m_send_buff[idx++] = 0x00; //
	m_send_buff[idx++] = 0x00; //
	m_send_buff[idx++] = 0x00; //
	m_send_buff[idx++] = 0x00; //
    crc = crc8_MAXIM( m_send_buff, idx );
    m_send_buff[idx++] = (uint8_t)crc; //CRCL
    UART_SEND( m_send_buff, idx );
}


//中大驱动器设置电机速度和方向
static void ZdSetPwm( uint8_t f_addr, uint16_t f_reg1, uint16_t f_reg2 )
{
    uint8_t  idx = 0;
    uint16_t crc = 0;
    
    m_send_buff[idx++] = f_addr;
    m_send_buff[idx++] = 0x10; //写多个寄存器
    m_send_buff[idx++] = 0x20; //起始寄存器地址高字节
    m_send_buff[idx++] = 0x00; //起始寄存器地址低字节
    m_send_buff[idx++] = 0x00; //寄存器数量高字节
    m_send_buff[idx++] = 0x02; //寄存器数量低字节
    m_send_buff[idx++] = 0x04; //数据字节总数
    m_send_buff[idx++] = (uint8_t)(f_reg1>>8); //寄存器1数据高字节
    m_send_buff[idx++] = (uint8_t)f_reg1;      //寄存器1数据低字节
    m_send_buff[idx++] = (uint8_t)(f_reg2>>8); //寄存器2数据高字节
    m_send_buff[idx++] = (uint8_t)f_reg2;      //寄存器2数据低字节
    crc = crc16table( m_send_buff, idx );
    m_send_buff[idx++] = (uint8_t)crc; //CRCL
    m_send_buff[idx++] = (uint8_t)(crc>>8);     //CRCH
    if( UART_SEND( m_send_buff, idx ) != 0 )
        m_send_error_count++;
    else
        m_send_error_count = 0;
}

//中大驱动器读取电机状态
static void ZdReadStatus( uint8_t f_addr )
{
    uint8_t  idx = 0;
    uint16_t crc = 0;
    param_read_type = 2;
	
    m_send_buff[idx++] = f_addr;
    m_send_buff[idx++] = 0x03; //读寄存器
    m_send_buff[idx++] = 0x21; //起始寄存器地址高字节
    m_send_buff[idx++] = 0x00; //起始寄存器地址低字节
    m_send_buff[idx++] = 0x00; //寄存器数量高字节
    m_send_buff[idx++] = 0x03; //寄存器数量低字节
    crc = crc16table( m_send_buff, idx );
    m_send_buff[idx++] = (uint8_t)crc; //CRCL
    m_send_buff[idx++] = (uint8_t)(crc>>8); //CRCH
    if( UART_SEND( m_send_buff, idx ) != 0 )
        m_send_error_count++;
    else
        m_send_error_count = 0;
}

//中大驱动器读取电机电流
static void ZdReadCourent( uint8_t f_addr )
{
    uint8_t  idx = 0;
    uint16_t crc = 0;
    param_read_type = 1;
	
    m_send_buff[idx++] = f_addr;
    m_send_buff[idx++] = 0x03; //读寄存器
    m_send_buff[idx++] = 0x30; //起始寄存器地址高字节
    m_send_buff[idx++] = 0x04; //起始寄存器地址低字节
    m_send_buff[idx++] = 0x00; //寄存器数量高字节
    m_send_buff[idx++] = 0x01; //寄存器数量低字节
    crc = crc16table( m_send_buff, idx );
    m_send_buff[idx++] = (uint8_t)crc; //CRCL
    m_send_buff[idx++] = (uint8_t)(crc>>8); //CRCH
    if( UART_SEND( m_send_buff, idx ) != 0 )
        m_send_error_count++;
    else
        m_send_error_count = 0;
}

//中大驱动器读取电机电流
static void ZdReadSpeed( uint8_t f_addr )
{
    uint8_t  idx = 0;
    uint16_t crc = 0;
    
    m_send_buff[idx++] = f_addr;
    m_send_buff[idx++] = 0x03; //读寄存器
    m_send_buff[idx++] = 0x20; //起始寄存器地址高字节
    m_send_buff[idx++] = 0x01; //起始寄存器地址低字节
    m_send_buff[idx++] = 0x00; //寄存器数量高字节
    m_send_buff[idx++] = 0x01; //寄存器数量低字节
    crc = crc16table( m_send_buff, idx );
    m_send_buff[idx++] = (uint8_t)crc; //CRCL
    m_send_buff[idx++] = (uint8_t)(crc>>8); //CRCH
    if( UART_SEND( m_send_buff, idx ) != 0 )
        m_send_error_count++;
    else
        m_send_error_count = 0;
}

//读取485通讯超时配置时间
static void ZdReadTimeout( uint8_t f_addr )
{
    uint8_t  idx = 0;
    uint16_t crc = 0;
    
    m_send_buff[idx++] = f_addr;
    m_send_buff[idx++] = 0x03; //读寄存器
    m_send_buff[idx++] = 0x08; //起始寄存器地址高字节
    m_send_buff[idx++] = 0x04; //起始寄存器地址低字节
    m_send_buff[idx++] = 0x00; //寄存器数量高字节
    m_send_buff[idx++] = 0x01; //寄存器数量低字节
    crc = crc16table( m_send_buff, idx );
    m_send_buff[idx++] = (uint8_t)crc; //CRCL
    m_send_buff[idx++] = (uint8_t)(crc>>8); //CRCH
    if( UART_SEND( m_send_buff, idx ) != 0 )
        m_send_error_count++;
    else
        m_send_error_count = 0;
}

//中大滚刷控制
static void ZdRollerMotor(void)
{
    uint16_t data = 0;
    uint16_t cmd = 2;
	
	if( GetEmergencyStopState() || GetKeyState() == 0x02 || !GetPcConnectStatus() )
	{
		data = 0;
        cmd = 0x05;
	}
	else if( m_roller_cmd.enable )
	{
        cmd =m_roller_cmd.dir;
		data = m_roller_cmd.target_pwm;
	}
	else
	{
		data = 0;
        cmd = 0x05;
	}
    
    if(m_roller_status.hitch == 18 )//485断链告警
    {
        data = 0;
        cmd = 0x07;
    }
	
	if( data >=3000 )
		data = 2999;
	
    if( data )
        m_roller_status.run_status = 1;
    else
        m_roller_status.run_status = 0;
    
    m_roller_status.flag = 0;
        
	ZdSetPwm( ADDR_ROLLER_MOTOR, cmd, data );
}

//中大左边刷控制
static void ZdLeftMotor(void)
{
    uint16_t data = 0;
    uint16_t cmd = 2;
	
	if( GetEmergencyStopState() || GetKeyState() == 0x02 || !GetPcConnectStatus()  )
	{
		data = 0;
        cmd = 0x05;
	}
	else if( m_left_brush_cmd.enable )
	{
		cmd = m_left_brush_cmd.dir;
		data = m_left_brush_cmd.target_pwm;
	}
	else
	{
		data = 0;
        cmd = 0x05;
	}
    
    if(m_left_brush_status.hitch == 18 )//485断链告警
    {
        data = 0;
        cmd = 0x07;
    }
	
	if( data >=3000 )
		data = 2999;
	
    if( data )
        m_left_brush_status.run_status = 1;
    else
        m_left_brush_status.run_status = 0;
    
    m_left_brush_status.flag = 0;
  
	if(side_motor_type==0)
	{
		ZdSetPwm( ADDR_LEFT_MOTOR, cmd, data );
	}
	else if(side_motor_type==1)
	{
	   BmSetPwm(BM_ADDR_LEFT_MOTOR,data/30) ;
	}	

}

//中大右边刷控制
static void ZdRightMotor(void)
{
    uint16_t data = 0;
    uint16_t cmd = 2;
	
	if( GetEmergencyStopState() || GetKeyState() == 0x02 || !GetPcConnectStatus()  )
	{
		data = 0;
        cmd = 0x05;
	}
	else if( m_right_brush_cmd.enable )
	{
        cmd =  m_right_brush_cmd.dir;
		data = m_right_brush_cmd.target_pwm;
	}
	else
	{
		data = 0;
        cmd = 0x05;
	}
    
    if(m_right_brush_status.hitch == 18 )//485断链告警
    {
        data = 0;
        cmd = 0x07;
    }
	
	if( data >=3000 )
		data = 2999;
	
    if( data )
        m_right_brush_status.run_status = 1;
    else
        m_right_brush_status.run_status = 0;
    
    m_right_brush_status.flag = 0;
        
	if(side_motor_type==0)
	{
		ZdSetPwm( ADDR_RIGHT_MOTOR, cmd, data );
	}
	else if(side_motor_type==1)
	{
	    BmSetPwm(BM_ADDR_RIGHT_MOTOR,-(data/30));
	}
}

//试能485配置功能
void EnableWrite( uint8_t f_addr )
{
    uint8_t  idx = 0;
    uint16_t crc = 0;
    
    m_send_buff[idx++] = f_addr;
    m_send_buff[idx++] = 0x06; //写寄存器
    m_send_buff[idx++] = 0x20; //寄存器地址高字节
    m_send_buff[idx++] = 0x0E; //寄存器地址低字节
    m_send_buff[idx++] = 0x00;
    m_send_buff[idx++] = 0x01; //通讯功能码写操作使能
    crc = crc16table( m_send_buff, idx );
    m_send_buff[idx++] = (uint8_t)crc; //CRCL
    m_send_buff[idx++] = (uint8_t)(crc>>8); //CRCH
    if( UART_SEND( m_send_buff, idx ) != 0 )
        m_send_error_count++;
    else
        m_send_error_count = 0;
}
//配置485通讯超时时间为1000ms
void WriteTime( uint8_t f_addr )
{
    uint8_t  idx = 0;
    uint16_t crc = 0;
    
    uint16_t timeout485 = 0;
    
    if( GetKeyState() != 0x02 )
        timeout485 = TIMEOUT_458;
    
    m_send_buff[idx++] = f_addr;
    m_send_buff[idx++] = 0x06; //写寄存器
    m_send_buff[idx++] = 0x08; //寄存器地址高字节
    m_send_buff[idx++] = 0x04; //寄存器地址低字节
    m_send_buff[idx++] = 0x00;
    m_send_buff[idx++] = timeout485; //通讯功能码写操作使能
    crc = crc16table( m_send_buff, idx );
    m_send_buff[idx++] = (uint8_t)crc; //CRCL
    m_send_buff[idx++] = (uint8_t)(crc>>8); //CRCH
    if( UART_SEND( m_send_buff, idx ) != 0 )
        m_send_error_count++;
    else
        m_send_error_count = 0;
}
//轮询配置各驱动器通讯超时时间
static void EnableTimeout(void )
{
    static uint8_t s_idx = 0;
    
    uint16_t timeout485 = 0;
    
    if( GetKeyState() != 0x02 )
        timeout485 = TIMEOUT_458;
    
    s_idx++;
    switch(s_idx)
    {
        case 1:
            if(m_roller_status.timeout485!=timeout485 && m_roller_status.timeout485!=1)
                EnableWrite(ADDR_ROLLER_MOTOR);
            break;
        case 2:
            if(m_roller_status.timeout485!=timeout485 && m_roller_status.timeout485!=1)
                WriteTime(ADDR_ROLLER_MOTOR);
			if(side_motor_type!=0)  //使用bm电机
			{
				s_idx = 0;
			}
            break;
        case 3:
            if( m_left_brush_status.timeout485!=timeout485 && m_left_brush_status.timeout485!=1 )
                EnableWrite(ADDR_LEFT_MOTOR);
            break;
        case 4:
            if( m_left_brush_status.timeout485!=timeout485 && m_left_brush_status.timeout485!=1 )
                WriteTime(ADDR_LEFT_MOTOR);
            break;
        case 5:
            if( m_right_brush_status.timeout485!=timeout485 && m_right_brush_status.timeout485!=1)
                EnableWrite(ADDR_RIGHT_MOTOR);
            break;
        case 6:
            if( m_right_brush_status.timeout485!=timeout485 && m_right_brush_status.timeout485!=1)
                WriteTime(ADDR_RIGHT_MOTOR);
            s_idx = 0;
            break;
        default:
            s_idx = 0;
            break;
    }
}

//中大电机控制
static void MotorCtrl( void )
{
    static uint8_t s_idx = 0;
    
    s_idx++;
    switch(s_idx)
    {
        case 1:
            ZdRollerMotor();
            break;
        
        case 2:
            ZdLeftMotor();
            break;
        
        case 3:
            ZdRightMotor();
            break;
        
        default:
            EnableTimeout();
            s_idx = 0;
            break;
    }
    
    if( m_send_error_count >= 20 )
    {
        m_send_error_count = 0;
//        RESET_UART();
    }
}
//中大状态读取
static void ReadDriverStatus(void)
{
    static uint8_t s_idx = 0;
    
    uint16_t timeout485 = 0;
    
    if( GetKeyState() != 0x02 )
        timeout485 = TIMEOUT_458;
    
    s_idx++;
    switch(s_idx)
    {
        case 1:
            ZdReadStatus( ADDR_ROLLER_MOTOR);  
            break;
        case 2:
            ZdReadStatus(ADDR_LEFT_MOTOR);
            break;
        case 3:
            ZdReadStatus(ADDR_RIGHT_MOTOR);
            break;
		case 4:
            ZdReadSpeed(ADDR_ROLLER_MOTOR);
            break;
        case 5:
            if( GetKeyState() == 0x02 || m_roller_status.timeout485!=timeout485 )
                ZdReadTimeout(ADDR_ROLLER_MOTOR);
            else
                ZdReadCourent(ADDR_ROLLER_MOTOR);
			if(side_motor_type!=0)
			{
				s_idx=0;     //使用的本末边刷电机
			}
            break;
        case 6:
            if( GetKeyState() == 0x02 || m_left_brush_status.timeout485!=timeout485 )
                ZdReadTimeout(ADDR_LEFT_MOTOR);
            else
                ZdReadCourent(ADDR_LEFT_MOTOR);
            break;
        case 7:
            if( GetKeyState() == 0x02 || m_right_brush_status.timeout485!=timeout485 )
                ZdReadTimeout(ADDR_RIGHT_MOTOR);
            else
                ZdReadCourent(ADDR_RIGHT_MOTOR);
            break;
        default:
            s_idx = 0;
            break;
    }
}



static void OverCurrentCheck(void)
{
    static TickType_t s_roller_tick = 0;
    static TickType_t s_left_side_tick = 0;
    static TickType_t s_right_side_tick = 0;
    
    if( m_roller_status.current >= m_roller_cmd.max_current && xTaskGetTickCount()-s_roller_tick >= 5000)
    {
        m_roller_status.over_current = 1;
    }
    else if( m_roller_status.current < m_roller_cmd.max_current )
    {
        s_roller_tick = xTaskGetTickCount();
    }
    
    if( abs(m_left_brush_status.current) >= m_left_brush_cmd.max_current )
    {
        if( xTaskGetTickCount()-s_left_side_tick>=5000)
            m_left_brush_status.over_current = 1;
    }
    else
    {
        s_left_side_tick = xTaskGetTickCount();
    }
    
    if( abs(m_right_brush_status.current) >= m_right_brush_cmd.max_current )
    {
        if( xTaskGetTickCount()-s_right_side_tick>=5000)
            m_right_brush_status.over_current = 1;
    }
    else
    {
        s_right_side_tick = xTaskGetTickCount();
    }
}

static uint32_t AnalyzeZdAlarm( uint16_t f_alarm )
{
    uint32_t alarm = 0;
    
    switch( f_alarm )
    {
        case 10: //母线欠压
            alarm |= 1;
            break;
        case 11://电机过载
            alarm |= 1<<1;
            break;
        case 12://驱动器过载
            alarm |= 1<<2;
            break;
        case 13://霍尔故障
            alarm |= 1<<3;
            break;
        case 14://堵转
            alarm |= 1<<4;
            break;
		case 19://电流检测故障
            alarm |= 1<<15;
			break;
		case 23://电机过载   //电子过载合并到电机过载
            alarm |= 1<<11;
			break;
        case 27://参数存储故障
            alarm |= 1<<5;
            break;
        case 30://缺相故障
            alarm |= 1<<6;
            break;
        case 18://485通讯故障
            alarm |= 1<<7;
            break;
    }
    return alarm;
}


void ZdDriverReceive( uint8_t fp_buff[], uint16_t len )
{
    uint32_t idx = 0;
    uint16_t crc_c = 0;
    uint16_t crc_r = 0;
    uint16_t timeout485 = 0;
    
    if( GetKeyState() != 0x02 )
        timeout485 = TIMEOUT_458;
    
    if( NULL != fp_buff && len >4 )
    {
        if(fp_buff[0]==0x55 && fp_buff[1]==0xA1 )
        {
			m_send_complete_flag =1;
            HaldleXdsData( fp_buff, len );
            return;
        }
		if(len==10)  //本末电机数据反馈
		{
			crc_c = crc8_MAXIM( fp_buff, 9);
			crc_r = fp_buff[9];
			if(crc_c==crc_r)
			{
				m_send_complete_flag =1;
				switch(fp_buff[0])
				{
					case BM_ADDR_LEFT_MOTOR:
						m_left_brush_status.last_rx_tick = xTaskGetTickCount();
						m_left_brush_status.current = ((fp_buff[2]<<8)|fp_buff[3])/10; //转换为单位0.01A
						m_left_brush_status.hitch = fp_buff[8];
						break;
					case BM_ADDR_RIGHT_MOTOR:
						m_right_brush_status.last_rx_tick = xTaskGetTickCount();
						m_right_brush_status.current = ((fp_buff[2]<<8)|fp_buff[3])/10;
						m_right_brush_status.hitch = fp_buff[8];
						break;
				}
			 return;
			}
		}		
		
        crc_c = crc16table( fp_buff, len-2);
        crc_r = ((uint16_t)fp_buff[len-1]<<8)|(uint16_t)fp_buff[len-2];
        if( crc_c == crc_r )
        {
			m_send_complete_flag = 1;
            switch(fp_buff[0])
            {
                case ADDR_ROLLER_MOTOR:
                    m_roller_status.last_rx_tick = xTaskGetTickCount();
                    break;
                case ADDR_LEFT_MOTOR:
                    m_left_brush_status.last_rx_tick = xTaskGetTickCount();
                    break;
                case ADDR_RIGHT_MOTOR:
                    m_right_brush_status.last_rx_tick = xTaskGetTickCount();
                    break;
            }
            if( ZHONGDA==m_driver_type)
            {
                idx = (fp_buff[0]<<16)|(fp_buff[1]<<8)|(fp_buff[2]);
                
                switch( idx )
                {
                    case 0x021020:
                        m_roller_status.flag = 1;
                        break;
                    case 0x031020:
                        m_left_brush_status.flag = 1;
                        break;
                    case 0x041020:
                        m_right_brush_status.flag = 1;
                        break;
                    case 0x020306:
                        m_roller_status.status1 = (fp_buff[3]<<8)|fp_buff[4];
                        m_roller_status.status2 = (fp_buff[5]<<8)|fp_buff[6];
                        m_roller_status.hitch = (fp_buff[7]<<8)|fp_buff[8];
                        break;
                    case 0x030306:
                        m_left_brush_status.status1 = (fp_buff[3]<<8)|fp_buff[4];
                        m_left_brush_status.status2 = (fp_buff[5]<<8)|fp_buff[6];
                        m_left_brush_status.hitch = (fp_buff[7]<<8)|fp_buff[8];
                        break;
                    case 0x040306:
                        m_right_brush_status.status1 = (fp_buff[3]<<8)|fp_buff[4];
                        m_right_brush_status.status2 = (fp_buff[5]<<8)|fp_buff[6];
                        m_right_brush_status.hitch = (fp_buff[7]<<8)|fp_buff[8];
                        break;
                    case 0x020302:
						if(param_read_type == 1)
						{
							if( GetKeyState() == 0x02 || m_roller_status.timeout485!=timeout485 )
								m_roller_status.timeout485 = (fp_buff[3]<<8)|fp_buff[4];
							else
								m_roller_status.current = (fp_buff[3]<<8)|fp_buff[4];
						}
						else 
						{
							m_roller_status.speed =(fp_buff[3]<<8)|fp_buff[4];
						}
						break;
                    case 0x030302:
                        if( GetKeyState() == 0x02 || m_left_brush_status.timeout485!=timeout485)
                            m_left_brush_status.timeout485 = (fp_buff[3]<<8)|fp_buff[4];
                        else
                            m_left_brush_status.current = (fp_buff[3]<<8)|fp_buff[4];
                        break;
                    case 0x040302:
                        if( GetKeyState() == 0x02 || m_right_brush_status.timeout485!=timeout485)
                            m_right_brush_status.timeout485= (fp_buff[3]<<8)|fp_buff[4];
                        else
                            m_right_brush_status.current = (fp_buff[3]<<8)|fp_buff[4];
                        break;
                    default:
                        break;
                }
            }
            else if( UNKNOW == m_driver_type )
            {
                idx = (fp_buff[3]<<8)|fp_buff[4];
					if( idx==0x06 )
                    m_driver_type = ZHONGDA;
            }
        }
    }
}

void SetRollerMotor(uint32_t f_enable, uint32_t f_pwm )
{
    m_roller_cmd.enable = f_enable?1:0;
    m_roller_cmd.target_pwm = f_pwm*30;
}

void SetLeftBrushMotor(uint32_t f_enable, uint32_t f_pwm )
{
    m_left_brush_cmd.enable = f_enable?1:0;
    m_left_brush_cmd.target_pwm = f_pwm*30;
}

void SetRightBrushMotor(uint32_t f_enable, uint32_t f_pwm )
{
    m_right_brush_cmd.enable = f_enable?1:0;
    m_right_brush_cmd.target_pwm = f_pwm*30;
}

void SetRollerMaxCurrent(uint16_t f_max)
{
    if( f_max )
        m_roller_cmd.max_current = f_max/10;
}

void ClearZdOverCurrent(void)
{
    m_roller_status.over_current = 0;
    m_left_brush_status.over_current = 0;
    m_right_brush_status.over_current = 0;
}

uint16_t GetRollerCurrent(void)
{
    return m_roller_status.current/10;
}
uint16_t GetRollerSpeed(void)
{
    return m_roller_status.speed;
}
uint16_t GetLeftBrushCurrent(void)
{
    return m_left_brush_status.current;
}

uint16_t GetRightBrushCurrent(void)
{
    return m_right_brush_status.current;
}

uint8_t GetRollerOvercurrentAlarm(void)
{
    uint8_t alarm = 0;
    alarm |= m_roller_status.over_current;
    return alarm;
}

uint8_t GetSideBrushOvercurrentAlarm(void)
{
    uint8_t alarm = 0;
    alarm |= m_left_brush_status.over_current;
    alarm |= m_right_brush_status.over_current<<1;
    return alarm;
}

uint8_t GetRollerConnectStatus(void)
{
    return (xTaskGetTickCount()-m_roller_status.last_rx_tick<=2000);
}

uint8_t GetLeftBrushConnectStatus(void)
{
    return (xTaskGetTickCount()-m_left_brush_status.last_rx_tick<=2000);
}

uint8_t GetRightBrushConnectStatus(void)
{
    return (xTaskGetTickCount()-m_right_brush_status.last_rx_tick<2000);
}

uint8_t GetRollerWorkStatus(void)
{
    if(GetRollerConnectStatus())
    {

		return (m_roller_status.status1<3);
    }
    else
    {
        m_roller_status.current = 0;
        m_roller_status.status1 = 4;
		return 0;

    }
}

uint8_t GetLeftSideBrushWorkStatus(void)
{
    if(GetLeftBrushConnectStatus())
    {

		return (m_left_brush_status.status1<3);
    }
    else
    {
        m_left_brush_status.current = 0;
        m_left_brush_status.status1 = 4;
        return 0;

    }
}

uint8_t GetRightSideBrushWorkStatus(void)
{
    if(GetRightBrushConnectStatus())
    {
		return (m_right_brush_status.status1<3);
    }
    else
    {
        m_right_brush_status.current = 0;
		m_right_brush_status.status1 = 4;
		return 0;  
    }
}

uint8_t GetRollerDriverType(void)
{
    return (uint8_t)m_driver_type;
}

uint32_t GetZdDriverAlarm(void)
{
    uint32_t alarm = 0;
    if( m_driver_type == ZHONGDA )
	{
		alarm |= AnalyzeZdAlarm(m_roller_status.hitch)<<6;
		alarm |= AnalyzeZdAlarm(m_left_brush_status.hitch)<<14;
		alarm |= AnalyzeZdAlarm(m_right_brush_status.hitch)<<22;
	}
    
    return alarm;
}

uint32_t GetLeftSideAlarm(void)
{
	uint32_t alarm = 0;
	static uint32_t last_tick[6]={0};
	static uint16_t error_count=0;
	static uint16_t  error_flag=0;
	if(side_motor_type == 0)
	{
	    alarm = AnalyzeZdAlarm(m_left_brush_status.hitch);
		 return alarm;
	}
	
	else
	{
		if(m_left_brush_status.hitch)   //本末电机有告警
		{
			if(m_left_brush_status.hitch&0x01)//传感器故障
			{
				alarm=alarm|0x01;
			}
			if(m_left_brush_status.hitch&0x02)//过流故障 ,报过载
			{
				if((error_flag&0x02)==0)
				{
					error_flag|=0x02;
					last_tick[1]=xTaskGetTickCount();
				}
				else
				{
				   if(xTaskGetTickCount()-last_tick[1]>6000)
				   {
						alarm=alarm|0x01;			   
				   }	
				}
			}
			if(m_left_brush_status.hitch&0x04)//相过流故障
			{
				if((error_flag&0x04)==0)
				{
					error_flag|=0x04;
					last_tick[2]=xTaskGetTickCount();
				}
				else
				{
				   if(xTaskGetTickCount()-last_tick[2]>6000)
				   {
						alarm=alarm|0x02;			   
				   }	
				}
			}
			if(m_left_brush_status.hitch&0x08)//堵转故障
			{
				if((error_flag&0x08)==0)
				{
					error_flag|=0x08;
					last_tick[3]=xTaskGetTickCount();
				}
				else
				{
				   if(xTaskGetTickCount()-last_tick[3]>6000)
				   {
						alarm=alarm|0x04;			   
				   }	
				}			
			}
			if(m_left_brush_status.hitch&0x10)//过温故障
			{
				if((error_flag&0x10)==0)
				{
					error_flag|=0x08;
					last_tick[4]=xTaskGetTickCount();
				}
				else
				{
				   if(xTaskGetTickCount()-last_tick[4]>6000)  
				   {
					   if(error_count==0)
					   {		   
							last_tick[5]=xTaskGetTickCount();    //记录第一次告警的时间，30min出现两次才会真正上报过温		   
					   }
					   if(error_count<2)
					   {
							error_count++;
							error_flag &= (~0x10);
					   }
					   else
					   {
						   if(xTaskGetTickCount()-last_tick[5]<30*60000)  //30min出现两次才会真正上报过温
						   {
								alarm=alarm|0x08;	
						   }
						   else
						   {
								last_tick[5]=xTaskGetTickCount();    //记录第一次告警的时间，30min出现两次才会真正上报过温		   
								error_count=1;
								error_flag &= (~0x10);
						   }
					   }				
				   }	
				}

				
			}
			else
			{
				error_count=0;
			}
		}
		else
		{
			error_flag=0;
		}	
	}

    return alarm;
}

uint32_t GetRightSideAlarm(void)
{
	uint32_t alarm = 0;
	static uint32_t last_tick[6]={0};
	static uint16_t error_count=0;
	static uint16_t  error_flag=0;
	if(side_motor_type == 0)
	{
	    alarm = AnalyzeZdAlarm(m_right_brush_status.hitch);
		 return alarm;
	}
	else
	{
		if(m_right_brush_status.hitch)   //本末电机有告警
		{
			if(m_right_brush_status.hitch&0x01)//传感器故障
			{
				alarm=alarm|0x01;
			}
			if(m_right_brush_status.hitch&0x02)//过流故障 ,报过载
			{
				if((error_flag&0x02)==0)
				{
					error_flag|=0x02;
					last_tick[1]=xTaskGetTickCount();
				}
				else
				{
				   if(xTaskGetTickCount()-last_tick[1]>6000)
				   {
						alarm=alarm|0x01;			   
				   }	
				}
			}
			if(m_right_brush_status.hitch&0x04)//相过流故障
			{
				if((error_flag&0x04)==0)
				{
					error_flag|=0x04;
					last_tick[2]=xTaskGetTickCount();
				}
				else
				{
				   if(xTaskGetTickCount()-last_tick[2]>6000)
				   {
						alarm=alarm|0x02;			   
				   }	
				}
			}
			if(m_right_brush_status.hitch&0x08)//堵转故障
			{
				if((error_flag&0x08)==0)
				{
					error_flag|=0x08;
					last_tick[3]=xTaskGetTickCount();
				}
				else
				{
				   if(xTaskGetTickCount()-last_tick[3]>6000)
				   {
						alarm=alarm|0x04;			   
				   }	
				}			
			}
			if(m_right_brush_status.hitch&0x10)//过温故障
			{
				if((error_flag&0x10)==0)
				{
					error_flag|=0x08;
					last_tick[4]=xTaskGetTickCount();
				}
				else
				{
				   if(xTaskGetTickCount()-last_tick[4]>6000)  
				   {
					   if(error_count==0)
					   {		   
							last_tick[5]=xTaskGetTickCount();    //记录第一次告警的时间，30min出现两次才会真正上报过温		   
					   }
					   if(error_count<2)
					   {
							error_count++;
							error_flag &= (~0x10);
					   }
					   else
					   {
						   if(xTaskGetTickCount()-last_tick[5]<30*60000)  //30min出现两次才会真正上报过温
						   {
								alarm=alarm|0x08;	
						   }
						   else
						   {
								last_tick[5]=xTaskGetTickCount();    //记录第一次告警的时间，30min出现两次才会真正上报过温		   
								error_count=1;
								error_flag &= (~0x10);
						   }
					   }
						
				   }	
				}	
			}
			else
			{
				error_count=0;
			}
		}
		else
		{
			error_flag=0;
		}
	}
    return alarm;
}	
void GetZdDebugData( uint16_t fp_data[] )
{
    if( fp_data != NULL )
    {
        fp_data[0] = m_roller_cmd.target_pwm;
        fp_data[1] = m_roller_status.run_status;
        fp_data[2] = m_roller_status.status1;
        fp_data[3] = m_roller_status.current;
        fp_data[4] = m_left_brush_cmd.target_pwm;
        fp_data[5] = m_left_brush_status.run_status;
        fp_data[6] = m_left_brush_status.status1;
        fp_data[7] = m_left_brush_status.current;
        fp_data[8] = m_right_brush_cmd.target_pwm;
        fp_data[9] = m_right_brush_status.run_status;
        fp_data[10] = m_right_brush_status.status1;
        fp_data[11] = m_right_brush_status.current;
		fp_data[12] = side_motor_type;
    }
}

void ZdDebug(uint8_t f_cmd)
{
    if( f_cmd == 1)
    {
        m_roller_cmd.enable = 1;
        m_roller_cmd.target_pwm = 1500;
    }
    else if( f_cmd == 2)
    {
        m_roller_cmd.enable = 0;
        m_roller_cmd.target_pwm = 0;
    }
    else if( f_cmd == 3)
    {
        m_left_brush_cmd.enable = 1;
        m_left_brush_cmd.target_pwm = 1500;
    }
    else if( f_cmd == 4)
    {
        m_left_brush_cmd.enable = 0;
        m_left_brush_cmd.target_pwm = 0;
    }
    else if( f_cmd == 5)
    {
        m_right_brush_cmd.enable = 1;
        m_right_brush_cmd.target_pwm = 1500;
    }
    else if( f_cmd == 6)
    {
        m_right_brush_cmd.enable = 0;
        m_right_brush_cmd.target_pwm = 0;
    }
}

void SetRollerMotorDir(uint32_t dir)
{
    m_roller_cmd.dir = (dir & 0x01)?2:1;
	m_left_brush_cmd.dir = (dir & 0x04)?2:1; 
	m_right_brush_cmd.dir = (dir & 0x08)?2:1; 
}
uint16_t len = 0;
void ZdDriverTask(void const *pvParameters)
{
    TickType_t lastWakeTime;

	const TickType_t frequency = pdMS_TO_TICKS(20);
	uint32_t bitPos = (uint32_t)pvParameters;
    uint8_t thread_num = (uint8_t)(uint32_t)pvParameters;
    uint8_t flag = 1;
    uint8_t count = 0;
    uint8_t cmd_zd[8] = {0x02, 0x03, 0x20, 0x09, 0x00, 0x01, 0x5F, 0xFB};
    uint8_t cmd_flag = 1;
    
    ParamInit();
    
    (void)bitPos;
    lastWakeTime = xTaskGetTickCount();
	vTaskDelay(15000);
    while(1)
    {
//        FeedDog(bitPos);
        side_motor_type=ConfigCommandCMD.side_motor_type;
        count++;
        if( count>=3 )
        {
            m_send_complete_flag = 1;
        }
        if( m_send_complete_flag )
        {
            count = 0;
            m_send_complete_flag = 0;


            if( m_driver_type==ZHONGDA)
            {
                flag++;
                switch(flag)
                {
                    case 1:
                        MotorCtrl();
                        break;
                    case 2:
                        ReadDriverStatus();
                        break;
                    default:
                        XdsTask();
                        flag = 0;
                        break;
                }
            }
            else if( m_driver_type==UNKNOW )
            {
                cmd_flag++;
                switch(cmd_flag)
                {
                    case 1:
                        UART_SEND(cmd_zd, 8);
                        break;
                    case 2:

                        break;
                    default:
                        XdsTask();
                        cmd_flag = 0;
                        break;
                }
				if(cmd_zd[0]++>4)cmd_zd[0]=2;

            }
        }
        
        OverCurrentCheck();
        
//        g_thread_call_count[thread_num]++;
		/* vTaskDelayUntil是绝对延迟，vTaskDelay是相对延迟。*/
        vTaskDelay(frequency);
		len = BUF_UsedSize(&m_stcRingBuf_RS485_3);
		BUF_Read(&m_stcRingBuf_RS485_3, g_uart3RxBuf, len);
		if( len >1 )
		{
			ZdDriverReceive(g_uart3RxBuf, len);
		}
    }
}





