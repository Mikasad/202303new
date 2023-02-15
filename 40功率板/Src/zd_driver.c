#include "zd_driver.h"
#include "Agreement.h"
#include "main.h"
#define ADDR_FRONT_MOTOR       0x02
#define ADDR_REAR_MOTOR        0x03
#define ADDR_LEFT_MOTOR        0x04
#define ADDR_RIGHT_MOTOR       0x05

#define TIMEOUT_458  100
#define DISCONNECT_TIME_MS  1000

uint8_t RXC_states=3;


/*485发送*/
static uint32_t UART_SEND(uint8_t* buff, uint16_t len)  
{ 
	static  uint8_t latst_status=0;
	uint32_t result=0;
	  RXC_states=0;  //接收完成状态置0
		HAL_GPIO_WritePin(EN_485_Port, EN_485_Pin, GPIO_PIN_SET); //45发送状态
	  for(int i=10;i>0;i--)//延时10us左右，485芯片要求
	  {}
		result=HAL_UART_Transmit(&huart3, buff, len, 10 ); 
		HAL_GPIO_WritePin(EN_485_Port, EN_485_Pin, GPIO_PIN_RESET); //45接收状态

	return result;
} 



typedef struct
{
    uint32_t last_rx_tick;//上次接收到驱动器回复的时间戳
    uint16_t status1;
    uint16_t status2;
    uint16_t hitch;//告警码
    uint16_t current;
    uint16_t timeout485;//485超时时间
    uint8_t  run_status;
    uint8_t  run_pwm;
    uint8_t  flag;
    uint8_t  over_current;//过流告警
}MotorStatus_t;

typedef struct
{
    uint16_t target_pwm; //0~3000
    uint16_t max_current; //0.01A
    uint8_t enable;
    uint8_t dir;//电机转动方向
}MotorCmd_t;

typedef enum
{
    UNKNOW = 0,
    ZHONGDA,
    ZHICHUANG,
}DriverType_t;

static DriverType_t m_driver_type = ZHONGDA;
static MotorCmd_t m_roller_cmd_front = {0, 1100, 0, 3};
static MotorCmd_t m_roller_cmd_rear  = {0, 1100, 0, 1};
static MotorCmd_t m_left_brush_cmd = {0,300,0};
static MotorCmd_t m_right_brush_cmd = {0,300,0};

static MotorStatus_t m_roller_status_front = {0};
static MotorStatus_t m_roller_status_rear = {0};
static MotorStatus_t m_left_brush_status = {0};
static MotorStatus_t m_right_brush_status = {0};

static uint8_t m_send_buff[32] = {0};

static uint8_t m_send_complete_flag = 1; //非零表示本次发收完成
static uint8_t side_motor_type =0;       //0中大 1本末

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

static void ParamInit(void)
{
    m_roller_status_front.timeout485 = 1;//初始化为1，上电后从驱动器读取上来的值非1
    m_roller_status_rear.timeout485 = 1;
    m_roller_status_front.status1 = 3;
    m_roller_status_rear.status1 = 3;
	m_left_brush_cmd.dir = 0x01;
    m_right_brush_cmd.dir = 0x01;
    m_left_brush_status.status1 = 3;
    m_right_brush_status.status1 = 3;
	
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
	UART_SEND( m_send_buff, idx );
}

//中大驱动器读取电机状态
static void ZdReadStatus( uint8_t f_addr )
{
    uint8_t  idx = 0;
    uint16_t crc = 0;
    
    m_send_buff[idx++] = f_addr;
    m_send_buff[idx++] = 0x03; //读寄存器
    m_send_buff[idx++] = 0x21; //起始寄存器地址高字节
    m_send_buff[idx++] = 0x00; //起始寄存器地址低字节
    m_send_buff[idx++] = 0x00; //寄存器数量高字节
    m_send_buff[idx++] = 0x03; //寄存器数量低字节
    crc = crc16table( m_send_buff, idx );
    m_send_buff[idx++] = (uint8_t)crc; //CRCL
    m_send_buff[idx++] = (uint8_t)(crc>>8); //CRCH
	UART_SEND( m_send_buff, idx );
}

//中大驱动器读取电机电流
static void ZdReadCourent( uint8_t f_addr )
{
    uint8_t  idx = 0;
    uint16_t crc = 0;
    
    m_send_buff[idx++] = f_addr;
    m_send_buff[idx++] = 0x03; //读寄存器
    m_send_buff[idx++] = 0x30; //起始寄存器地址高字节
    m_send_buff[idx++] = 0x04; //起始寄存器地址低字节
    m_send_buff[idx++] = 0x00; //寄存器数量高字节
    m_send_buff[idx++] = 0x01; //寄存器数量低字节
    crc = crc16table( m_send_buff, idx );
    m_send_buff[idx++] = (uint8_t)crc; //CRCL
    m_send_buff[idx++] = (uint8_t)(crc>>8); //CRCH
	UART_SEND( m_send_buff, idx );
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
	UART_SEND( m_send_buff, idx );;
}

//中大前滚刷电机
static void ZdRollerMotor_Front(void)
{
    uint16_t data = 0;
    uint16_t cmd = 2;
	

        
	ZdSetPwm( ADDR_FRONT_MOTOR, cmd, data );
}

//中大后滚刷电机
static void ZdRollerMotor_Rear(void)
{
    uint16_t data = 0;
    uint16_t cmd = 2;
	
	
        
	ZdSetPwm( ADDR_REAR_MOTOR, cmd, data );
}

uint8_t crc8_MAXIM(uint8_t *data, uint8_t len) //数据校验
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

uint8_t CONNECT7_f=0,CONNECT8_f=0,CONNECT9_f=0;
uint16_t CONNECT7_t=0,CONNECT8_t=0,CONNECT9_t=0;
//本末设置pwm
void BmSetPwm(uint8_t f_addr, int16_t speed,int16_t Acce)
{
	  if(f_addr==SB_ID)//左边刷断链检测
		{
		 CONNECT7_f=1;
		}
	  else if(f_addr==SB2_ID)//右边刷断链检测
		{
		 CONNECT8_f=1;
		}
    uint8_t  idx = 0;
    uint16_t crc = 0;
    
    m_send_buff[idx++] = f_addr;
    m_send_buff[idx++] = 0x64;  
    m_send_buff[idx++] = (uint8_t)(speed>>8);  
    m_send_buff[idx++] = (uint8_t)speed; 
		m_send_buff[idx++] = 0x00; //	
		m_send_buff[idx++] = 0x00; //
		m_send_buff[idx++] = 0; //每 1rpm 的加速时间，单位为 0.1ms，当设置为 1 时，每1rpm 的加速时间为 0.1ms，当设置为 10 时，每 1rpm 的加速时间为 10*0.1ms=1ms，设置为 0 时，既默认为 1，每 1rpm 的加速时间为 0.1ms。
		m_send_buff[idx++] = 0x00; //
		m_send_buff[idx++] = 0x00; //
    crc = crc8_MAXIM( m_send_buff, idx );
    m_send_buff[idx++] = (uint8_t)crc; //CRCL
    UART_SEND( m_send_buff, idx );
}




//试能485配置功能
void BmState_Inquire(uint8_t id)
{
    uint8_t  idx = 0;
    uint16_t crc = 0;
    
    m_send_buff[idx++] = id;
    m_send_buff[idx++] = 0x74;  
    m_send_buff[idx++] = 0x00;  
    m_send_buff[idx++] = 0x00; 
	  m_send_buff[idx++] = 0x00; //	
    m_send_buff[idx++] = 0x00;
	  m_send_buff[idx++] = 0x00; //
	  m_send_buff[idx++] = 0x00; //
	  m_send_buff[idx++] = 0x00; //
    crc = crc8_MAXIM( m_send_buff, idx );
    m_send_buff[idx++] = (uint8_t)crc; //CRCL
    UART_SEND( m_send_buff, idx );
}
//配置485通讯超时时间为1000ms
void BmMode_Set(uint8_t id,uint8_t mode)
{
    uint8_t  idx = 0;
    uint16_t crc = 0;
    
    
    m_send_buff[idx++] = id;
    m_send_buff[idx++] = 0xA0;  
    m_send_buff[idx++] = 0x00;  
    m_send_buff[idx++] = 0x00; 
    m_send_buff[idx++] = 0x00;
  	m_send_buff[idx++] = 0x00; //
  	m_send_buff[idx++] = 0x00; //
  	m_send_buff[idx++] = 0x00; //
  	m_send_buff[idx++] = 0x00; //
    m_send_buff[idx++] = mode; //
    UART_SEND( m_send_buff, idx );
}

//本末ID设置
void BmID_Set(uint8_t id)
{
    uint8_t  idx = 0;
    uint16_t crc = 0;
    
    m_send_buff[idx++] = 0XAA;
    m_send_buff[idx++] = 0x55;  
    m_send_buff[idx++] = 0x53;  
    m_send_buff[idx++] = id; 
  	m_send_buff[idx++] = 0x00; //	
  	m_send_buff[idx++] = 0x00; //
  	m_send_buff[idx++] = 0x00; //
  	m_send_buff[idx++] = 0x00; //
  	m_send_buff[idx++] = 0x00; //
    m_send_buff[idx++] = 0X00; //
	  for(int i=0;i<5;i++) 
    {
    UART_SEND( m_send_buff, idx );
	}
}
//本末ID查询
void BmID_Inquire(void)
        {
    uint8_t  idx = 0;
    uint16_t crc = 0;
    m_send_buff[idx++] = 0XC8;
    m_send_buff[idx++] = 0x64;  
    m_send_buff[idx++] = 0x00;  
    m_send_buff[idx++] = 0x00; 
	m_send_buff[idx++] = 0x00; //	
	m_send_buff[idx++] = 0x00; //
	m_send_buff[idx++] = 0x00; //
	m_send_buff[idx++] = 0x00; //
	m_send_buff[idx++] = 0x00; //
    crc = crc8_MAXIM( m_send_buff, idx );
    m_send_buff[idx++] = (uint8_t)crc; //CRCL
    UART_SEND( m_send_buff, idx );
        }
//		if(len==10)
int16_t MOTOR7GetCurrent,MOTOR8GetCurrent;
int16_t MOTOR7FilterCurrent,MOTOR8FilterCurrent;
extern int32_t  MOTOR7_addCurrent,MOTOR8_addCurrent;//自检电流之和
extern uint16_t Side_brush_num7,Side_brush_num8;//自检电流之和次数
extern uint16_t Side_brush_flag7,Side_brush_flag8;//自检电流标志				
void Bm_state( uint8_t fp_buff[], uint16_t len )//用 id区分 边刷和风机
{
	    s32 TempValue;
	    uint8_t fault=0;
			uint8_t crc_c = 0;
			uint8_t crc_r = 0;

			if((fp_buff[0]!=SB_ID) && fp_buff[0]!=SB2_ID && fp_buff[0]!=0x55) //测试代码
			{
			  fault++;
			}	
			
			if((fp_buff[0]!=SB_ID) && fp_buff[0]!=SB2_ID) //用 id区分 边刷和风机
			 return;	
			crc_c = crc8_MAXIM( fp_buff, len-1 );
			crc_r = fp_buff[len-1];
			if( crc_c == crc_r )
			{
				if(fp_buff[0]==SB_ID && MotorControl[7].Motor_mode==Control_485)
				{
					RXC_states=1;  //右边刷接收完成状态置1
					CONNECT7_f=0;//右边刷正常连接
					RS_485_TIME=SB_TIME+1; //发送时间置
          MC_ClearFault2(MOTOR7_DISCONNECT);//右边刷连接正常
					MotorControl[7].Current.GetADCValue=(fp_buff[2]<<8)|fp_buff[3]; //电流
					MotorControl[7].Speed_Real=(int16_t)((fp_buff[4]<<8)|fp_buff[5])*25; //速度 
					MotorControl[7].Hall.HALL_CaptureValue=(fp_buff[6]<<8)|fp_buff[7];//位移
					MotorControl[7].Bm_fault_code=fp_buff[8]; //故障码
					
					MOTOR7GetCurrent=MotorControl[7].Current.GetADCValue;	
					TempValue = (s32)((MotorControl[7].Current.GetADCValue - MotorControl[7].Current.FilterValue )>>4);//左边刷
					MotorControl[7].Current.FilterValue = (u16)(TempValue + MotorControl[7].Current.FilterValue);
					MOTOR7FilterCurrent=MotorControl[7].Current.FilterValue;	
          if(Side_brush_flag7==1)
					{
					 MOTOR7_addCurrent+=MotorControl[7].Current.FilterValue;
					 Side_brush_num7++;
					}						
				}
				else 	if(fp_buff[0]==SB2_ID&& MotorControl[8].Motor_mode==Control_485)
				{
					RXC_states=2;  //左边刷接收完成状态置2
					CONNECT8_f=0;//左边刷正常连接
					RS_485_TIME=SB2_TIME+1; //发送时间置
          MC_ClearFault2(MOTOR8_DISCONNECT);//左边刷连接正常
					MotorControl[8].Current.GetADCValue=(fp_buff[2]<<8)|fp_buff[3];
					MotorControl[8].Speed_Real=(int16_t)((fp_buff[4]<<8)|fp_buff[5])*25;
					MotorControl[8].Hall.HALL_CaptureValue=(fp_buff[6]<<8)|fp_buff[7];
					MotorControl[8].Bm_fault_code=fp_buff[8];
					
					MOTOR8GetCurrent=MotorControl[8].Current.GetADCValue;
					TempValue = (s32)((MotorControl[8].Current.GetADCValue - MotorControl[8].Current.FilterValue )>>4);//右边刷
					MotorControl[8].Current.FilterValue = (u16)(TempValue + MotorControl[8].Current.FilterValue);
					MOTOR8FilterCurrent=MotorControl[8].Current.FilterValue;
          if(Side_brush_flag8==1)
					{
					 MOTOR8_addCurrent+=MotorControl[8].Current.FilterValue;
					 Side_brush_num8++;
					}								
				}
			}
			if(MotorControl[8].Bm_fault_code!=0 || MotorControl[7].Bm_fault_code!=0) //测试代码
			{
			 fault++;
			}
}

uint8_t checksum(uint8_t *data, uint8_t len)
{
    uint8_t crc, i;
    crc = 0x00;

    while(len--)
    {
        crc += *data++;
    }
    return crc;
}

void B86SetPwm(uint8_t f_addr, int16_t speed )
{
	  if(f_addr==FAN_ID)//风机断链检测
		{
	  CONNECT9_f=1;
		}
    uint8_t  idx = 0;
    uint8_t crc = 0;
    m_send_buff[idx++] = 0X55;
    m_send_buff[idx++] = f_addr;   //设备ID
    m_send_buff[idx++] = 0x31;     //速度模式
    m_send_buff[idx++] = (uint8_t)speed; 
	  m_send_buff[idx++] = (uint8_t)speed>>8; //	
    crc = checksum( m_send_buff, idx );
    m_send_buff[idx++] = crc; //CRCL
    UART_SEND( m_send_buff, idx );	
        }
void B86_state( uint8_t fp_buff[], uint16_t len )
{
	   uint8_t fault=0;
    uint8_t crc_c = 0;
    uint8_t crc_r = 0;
			if((fp_buff[0]!=SB_ID) && fp_buff[0]!=SB2_ID && fp_buff[0]!=0x55) //测试代码
			{
			  fault++;
			}	
			
	  if(fp_buff[1]!=0xA1)//用 id区分 边刷和风机
		return;
	  crc_c = checksum( fp_buff, len-1 );
	  crc_r = fp_buff[len-1];
	  if( crc_c == crc_r && MotorControl[9].Motor_mode==Control_485)
		{
			RXC_states=3;  //风机接收完成状态置3
			CONNECT9_f=0;//风机正常连接
			RS_485_TIME=FAN_TIME+1; //发送时间置
      MC_ClearFault2(MOTOR9_DISCONNECT);//风机连接正常		
		  MotorControl[9].Bm_fault_code=fp_buff[3]; //故障码
			MotorControl[9].temperature=fp_buff[4]; //温度：摄氏度
			MotorControl[9].Speed_Real=((fp_buff[6]<<8)|fp_buff[5])*210;//速度 1000rmp
    }
			if(MotorControl[9].Bm_fault_code!=0) //测试代码
			{
			 fault++;
			}
}
