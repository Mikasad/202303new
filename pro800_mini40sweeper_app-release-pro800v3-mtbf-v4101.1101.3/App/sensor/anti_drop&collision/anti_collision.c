#include "anti_collision.h"
#include "nanopb_tcpip.h"
//#include "can_task.h"
//#include "ctr_move_task.h"
#include "normal_io.h"
//#include <string.h>

int Can2SendExt( uint32_t f_ext_id, uint8_t f_value[], uint8_t f_len )
{
	can_extend_send(CM_CAN2,f_ext_id, f_value,f_len);
	return 0;
}
static antiCollisionInfo_t m_antiCollisionInfo[8] = {0};

static uint8_t m_antiCollisionTriggerFlag = 0;
static uint8_t m_antiCollisionDisconnectFlag = 0;
static uint8_t m_antiCollisionDisconnectReceive = 0;
static uint8_t m_antiCollisionStop = 0;

static uint32_t gs_noDataCnt = 0;

static uint16_t m_anti_collision_enable = 0;

static uint8_t m_software_version[3] = {0};
uint32_t g_all_rate[8] = {0}; //存储存储最多四次最近的变化率，变化率放入单个字节中
uint8_t  g_all_rate_idx[8] = {0};

static uint16_t m_cfg_antiCollisionValidVoltage = 420;//防碰撞传感器计算斜率时过滤的电压范围，0~420内的斜率不会触发防碰撞
static uint16_t m_cur_antiCollisionValidVoltage = 420;

void ReadAntiCollisionCfgInfo(void)
{
	uint8_t CanTx[8] = {0};

	Can2SendExt(0x0B011010, CanTx, 0);
}

void ReadAntiCollisionDataInfo(void)
{
	uint8_t CanTx[8] = {0};

	Can2SendExt(0x0B010000, CanTx, 0);
}

void ReadSoftwareVersion(void)
{
  uint8_t CanTx[8] = {0};
  static uint8_t count = 0;

  count++;
  if (count>=50)
  {
    Can2SendExt(0x0B011011, CanTx, 0);
    count = 0;
  }
}

void CfgAntiCollisionInfo(uint8_t id)
{
	uint8_t CanTx[8] = {0};

	CanTx[0] = (uint8_t)(m_antiCollisionInfo[id].cfgRate >> 24);
	CanTx[1] = (uint8_t)(m_antiCollisionInfo[id].cfgRate >> 16);
	CanTx[2] = (uint8_t)(m_antiCollisionInfo[id].cfgRate >> 8);
	CanTx[3] = (uint8_t)m_antiCollisionInfo[id].cfgRate;
	CanTx[4] = (uint8_t)(m_antiCollisionInfo[id].cfgPeriodTime_ms >> 24);
	CanTx[5] = (uint8_t)(m_antiCollisionInfo[id].cfgPeriodTime_ms >> 16);
	CanTx[6] = (uint8_t)(m_antiCollisionInfo[id].cfgPeriodTime_ms >> 8);
	CanTx[7] = (uint8_t)m_antiCollisionInfo[id].cfgPeriodTime_ms;
	Can2SendExt(0x0B011040 + id, CanTx, 8);
}

void CfgAntiCollisionValidVoltage(void)
{
    uint8_t CanTx[8] = {0};
    
    CanTx[0] = m_cfg_antiCollisionValidVoltage>>8;
    CanTx[1] = (uint8_t)m_cfg_antiCollisionValidVoltage;
  
    Can2SendExt(0x0B011050, CanTx, 2);
}

static void HandleRateData( uint8_t *fp_all_rate, uint8_t*fp_idx, uint8_t f_rate )
{    
    if( fp_all_rate != NULL && fp_idx!=NULL)
    {
        if( *fp_idx < 4 )
        {
            fp_all_rate[*fp_idx] = f_rate;
            (*fp_idx)++;
        }
        else //四字节已放满
        {
            for( int j=1;j<4;j++)
                fp_all_rate[j-1] = fp_all_rate[j];
            fp_all_rate[3] = f_rate;
        }
//        memcpy(g_test_rate, &fp_all_rate[0], 4);
    }
}

void receiveAntiCollisionData(uint32_t ExtId, uint8_t* Data)
{
	switch (ExtId)
	{
		case 0x0B011038:
		case 0x0B011039:
		case 0x0B01103A:
		case 0x0B01103B:
		case 0x0B01103C:
		case 0x0B01103D:
		case 0x0B01103E:
		case 0x0B01103F:        //读取到配置值
			m_antiCollisionInfo[ExtId - 0x0B011038].readCfgRate = Data[0] << 24 | Data[1] << 16 | Data[2] << 8 | Data[3];
			m_antiCollisionInfo[ExtId - 0x0B011038].readCfgPeriodTime_ms = Data[4] << 24 | Data[5] << 16 | Data[6] << 8 | Data[7];
			break;

		case 0x0B011048:
		case 0x0B011049:
		case 0x0B01104A:
		case 0x0B01104B:
		case 0x0B01104C:
		case 0x0B01104D:
		case 0x0B01104E:
		case 0x0B01104F:        //单传感器的实时变化率，触发值，断连值
			m_antiCollisionInfo[ExtId - 0x0B011048].rate = Data[0] << 24 | Data[1] << 16 | Data[2] << 8 | Data[3];
			break;

		case 0x0B011033:        //触发值
			gs_noDataCnt = 0;
			m_antiCollisionTriggerFlag = Data[0];
			m_antiCollisionDisconnectReceive = Data[1];
            if ( GetCurCtrMode() )        //自动模式下，刹车生效
            {
                for (int i=0; i<8; i++)
                {
                    if ((m_anti_collision_enable >> i) & 0x01 && (m_antiCollisionTriggerFlag >> i) & 0x01)   //使能+触发则刹车
                    m_antiCollisionStop |= (1<<i);
                }
				m_antiCollisionStop = m_antiCollisionStop&0xFC; //0 1通道作为接尘袋使用
            }
            for( int i=2; i<8; i++)
                HandleRateData((uint8_t*)&g_all_rate[i],&g_all_rate_idx[i],Data[i]);
			break;
        
        case 0x0B011031:   //版本号
            m_software_version[0] = Data[0];
            m_software_version[1] = Data[1];
            m_software_version[2] = Data[2];
            break;
        case 0x0B011051:
            m_cur_antiCollisionValidVoltage = (Data[0]<<8)|Data[1];
            break;

		default:
			break;
	}

}

/*
*  判断断连值
*/
uint8_t AntiCollision_Disconnect_Judge(void)
{
	uint8_t much_State = 0;
	uint8_t i;

	if (gs_noDataCnt > 30)
    {
        for (i=0; i<8; i++)
        {
            if ((m_anti_collision_enable >> i) & 0x01 )    //使能
            {
                much_State |= (1<<i);
            }
        }
		return much_State;             //超时回复全断连
    }

	for (i=0; i<8; i++)
	{
		if ((m_anti_collision_enable >> i) & 0x01 && (m_antiCollisionDisconnectReceive >> i) & 0x01)    //使能+断连则断连
		{
			much_State |= (1<<i);
		}
	}
	return much_State;
}

void HandleAntiCollisionData(void)
{
	uint8_t i;

	if ( GetCurCtrMode() )        //自动模式下，刹车生效
	{
		for (i=0; i<8; i++)
		{
			if ((m_anti_collision_enable >> i) & 0x01 && (m_antiCollisionTriggerFlag >> i) & 0x01)   //使能+触发则刹车
			m_antiCollisionStop |= (1<<i);
		}
	}
	else
		m_antiCollisionStop = 0;

	m_antiCollisionDisconnectFlag = AntiCollision_Disconnect_Judge();    //断连值
}

void ClearAntiCollisionTrigger(void)
{
	m_antiCollisionTriggerFlag = 0;
	m_antiCollisionStop = 0;
}

uint32_t GetAntiCollisionDisconectStatus(void)
{
	uint32_t status = 0;
	
	status = m_antiCollisionDisconnectFlag;
	
	return status;
}

uint32_t GetAntiCollisionReat(uint8_t f_idx)
{
	uint32_t current_rate = 0;
	if( f_idx<8 )
	{
		current_rate = m_antiCollisionInfo[f_idx].rate;
	}
	return current_rate;
}

uint8_t GetAntiCollisionStop(void)
{
	return m_antiCollisionStop;
}

uint16_t GetAntiCollisionEnable(void)
{
    return m_anti_collision_enable;
}

uint8_t GetAntiCollisionTriggle(void)
{
    return m_antiCollisionTriggerFlag;
}

void GetAntiCollisionData(antiCollisionInfo_t fp_data[] )
{
    if( NULL != fp_data )
    {
        for( int i=0;i<8;i++)
            fp_data[i] = m_antiCollisionInfo[i];
    }
}

void GetGcubVersion(uint32_t *fp_version )
{
    if( fp_version != NULL )
    {
        fp_version[1] = 1;
        fp_version[1] = m_software_version[0];
        fp_version[2] = m_software_version[1];
        fp_version[3] = m_software_version[2];
    }
}

void AntiCollisionTask(void)
{
	static uint8_t s_idx;
	static TickType_t last_read_tick = 0;
    static uint8_t s_cfg_delay = 0;
//	floatMemory_t floatMemory;

	/***************获取当前触发状态***************/
    if( m_anti_collision_enable )
        ReadAntiCollisionDataInfo();
	/***************获取当前配置***************/
	if( xTaskGetTickCount() - last_read_tick >= 100 )
	{
		last_read_tick = xTaskGetTickCount();
		gs_noDataCnt++;
        if(s_idx<8)
        {
            if ( ((m_anti_collision_enable >> s_idx) & 0x01) && 
			      (m_antiCollisionInfo[s_idx].readCfgRate != m_antiCollisionInfo[s_idx].cfgRate\
                || m_antiCollisionInfo[s_idx].readCfgPeriodTime_ms != m_antiCollisionInfo[s_idx].cfgPeriodTime_ms) )
            {
                CfgAntiCollisionInfo(s_idx);
            }
        }
        s_idx++;
        if( s_idx>=8)
            s_idx=0;
        if( m_anti_collision_enable )
        {
            ReadAntiCollisionCfgInfo();
            s_cfg_delay++;
            if( s_cfg_delay>= 10 )
            {
                s_cfg_delay = 0;
                if( m_cfg_antiCollisionValidVoltage!=m_cur_antiCollisionValidVoltage )
                    CfgAntiCollisionValidVoltage();
            }
        }
        
        if( !m_software_version[0] && !m_software_version[1] && !m_software_version[2] )
        {
            ReadSoftwareVersion();
        }
	}

	HandleAntiCollisionData();
}

void ConfigAntiCollision( uint16_t f_enable, uint32_t fp_period_time[], uint32_t fp_rate[], uint16_t f_voltage )
{
	m_anti_collision_enable = f_enable;
	if( fp_period_time != NULL && fp_rate != NULL )
	{
		m_anti_collision_enable|=0x03;
		for( int i=0; i<2; i++)   // 通道0 1 改为尘袋检测使用
		{
			m_antiCollisionInfo[i].cfgPeriodTime_ms = 6;
			m_antiCollisionInfo[i].cfgRate = 60;
		}
		
		for( int i=2; i<8; i++)  
		{
			m_antiCollisionInfo[i].cfgPeriodTime_ms = fp_period_time[i];
			m_antiCollisionInfo[i].cfgRate = fp_rate[i];
		}
		
	}
}

