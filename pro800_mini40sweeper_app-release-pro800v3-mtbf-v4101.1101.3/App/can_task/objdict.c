#include "objdict.h"
#include <stdio.h>
#include "drv_h25a15.h"
#include "drv_zlac8015.h"
#include "nanopb_tcpip.h"


#define OBJ_NUM    5

const Objdict_t *m_pobjdict[OBJ_NUM] = {NULL};
static uint8_t m_objdict_size[OBJ_NUM] = {0};//每个m_pobjdict指针下结构项的数量
static uint8_t m_objdict_idx = 0;

void ObjdictLoad( const Objdict_t* obj, uint8_t size )
{
		
    if( obj != NULL && size > 0 && m_objdict_idx < OBJ_NUM )
    {
		for(uint8_t i=0;i<m_objdict_idx;i++)  //有重复的ID注册,进行更新
		{
			if(m_pobjdict[i]->id == obj->id) 
			{
				m_pobjdict[i] = obj;
				m_objdict_size[i] = size;
				return ;
			}
		}

		m_pobjdict[m_objdict_idx] = obj;
		m_objdict_size[m_objdict_idx] = size;
		m_objdict_idx++;	


    }
}

void ObjdictDispatch( stc_can_rx_frame_t * f_rx_msg )
{
    uint8_t id = 0;
    uint16_t index = 0;
    uint8_t sub_index = 0;
    uint8_t frame_type = 0;
    uint8_t i = 0;
    uint8_t idx = 0;
    uint8_t* p_data = NULL;
    pFunction func = NULL;
    uint8_t  node_id;
	if(f_rx_msg->IDE != 0)
	{
		return ;
	}

    if ((ConfigCommandCMD.motor_driver_type == 4) && (is_motor_driver_type_configed == 1))
    {
        /* handle zlac pdo data */
        handle_zlac_pdo_data(f_rx_msg->u32ID, f_rx_msg->au8Data, f_rx_msg->DLC);
    }

    if ((ConfigCommandCMD.hardware_type == 0) && (is_hardware_type_configed == 1))
    {
        /* handle h4aa30 pdo data */
        handle_h4aa30_pdo_data(f_rx_msg->u32ID, f_rx_msg->au8Data, f_rx_msg->DLC);
    }
    
	if(f_rx_msg->u32ID == (0x580 + H25A_ID))  //自研驱动器心跳有问题临时已数据判断
	{
		H25a_Info.data_recv_time = HAL_GetTick();
	}
 
	if((f_rx_msg->u32ID&0x700)==0x700) //心跳包处理
	{
		node_id=f_rx_msg->u32ID-0x700;
		for(idx=0; idx<m_objdict_idx; idx++ )
		{
			for(i=0; i< m_objdict_size[idx]; i++ )
			{
				if( m_pobjdict[idx][i].index == 0xFFFF && m_pobjdict[idx][i].id == node_id )//找到该结构，跳出
				{
					break;
				}
			}
			if(i < m_objdict_size[idx])             				//找到结构
			{
				p_data =  m_pobjdict[idx][i].p_data;
				*(uint32_t*)p_data= HAL_GetTick();
				return;
			}
			else if( i >= m_objdict_size[idx]&& idx==m_objdict_idx)//未找到该结构，跳出
			{
				return ;
			}

		}			
	} 
	
    if( f_rx_msg->DLC != 8 || f_rx_msg->IDE != CAN_ID_STD_EXT ) 
    {
        return;
    }
    
    id = f_rx_msg->u32ID - 0x580;
    frame_type = f_rx_msg->au8Data[0];
    index = (uint16_t)f_rx_msg->au8Data[1] | ((uint16_t)f_rx_msg->au8Data[2]<<8);
    sub_index = f_rx_msg->au8Data[3];
    
    for(idx=0; idx<m_objdict_idx; idx++ )
    {
        for(i=0; i< m_objdict_size[idx]; i++ )
        {
            if( m_pobjdict[idx][i].index == index && m_pobjdict[idx][i].sub_index == sub_index && m_pobjdict[idx][i].id == id )//找到该结构，跳出
            {
                break;
            }
        }
        if( i < m_objdict_size[idx] )//查询号小于该数组帧的长度，则找到该结构
        {
            break;
        }
    }
    
    if( idx == m_objdict_idx )//未找到该项
    {
        return;
    }

    p_data =  m_pobjdict[idx][i].p_data;
    func = m_pobjdict[idx][i].func;
    
    if( p_data != NULL )
    {
        switch( frame_type )
        {
            case 0x4F://报文包含一个字节的有效数据
                if( m_pobjdict[idx][i].data_size >= 1 )
                {
                    p_data[0] = f_rx_msg->au8Data[4];
                }
                break;
            case 0x4B://报文包含两个字节的有效数据
                if( m_pobjdict[idx][i].data_size >= 2 )
                {
                    p_data[0] = f_rx_msg->au8Data[4];
                    p_data[1] = f_rx_msg->au8Data[5];
                }
                break;
            case 0x47://报文包含三个字节的有效数据
                if( m_pobjdict[idx][i].data_size >= 3 )
                {
                    p_data[0] = f_rx_msg->au8Data[4];
                    p_data[1] = f_rx_msg->au8Data[5];
                    p_data[2] = f_rx_msg->au8Data[6];
                }
                break;
            case 0x43://报文包含四个字节的有效数据
                if( m_pobjdict[idx][i].data_size >= 4 )
                {
                    p_data[0] = f_rx_msg->au8Data[4];
                    p_data[1] = f_rx_msg->au8Data[5];
                    p_data[2] = f_rx_msg->au8Data[6];
                    p_data[3] = f_rx_msg->au8Data[7];
                }
                break;
            case 0x60://写命令的回复
                break;
            default:
                return;
        }
    }
    
    if( func != NULL )
    {
        func( NULL );
    }
    
    return;
}


/******************以下给CAN2用*************************/
const Objdict_t *m_pobjdict2[OBJ_NUM] = {NULL};
static uint8_t m_objdict_size2[OBJ_NUM] = {0};//每个m_pobjdict指针下结构项的数量
static uint8_t m_objdict_idx2 = 0;

void ObjdictLoad2( const Objdict_t* obj, uint8_t size )
{
    if( obj != NULL && size > 0 && m_objdict_idx2 < OBJ_NUM )
    {
        m_pobjdict2[m_objdict_idx2] = obj;
        m_objdict_size2[m_objdict_idx2] = size;
        m_objdict_idx2++;
    }
}

void ObjdictDispatch2( stc_can_rx_frame_t * f_rx_msg )
{
    uint8_t id = 0;
    uint16_t index = 0;
    uint8_t sub_index = 0;
    uint8_t frame_type = 0;
    uint8_t i = 0;
    uint8_t idx = 0;
    uint8_t* p_data = NULL;
    pFunction func = NULL;
    uint8_t  node_id;

    if( f_rx_msg->DLC != 8 || !(f_rx_msg->u32ID&0x580)) 	
    {
        return;
    }
    
    id = f_rx_msg->u32ID - 0x580;
    frame_type = f_rx_msg->au8Data[0];
    index = (uint16_t)f_rx_msg->au8Data[1] | ((uint16_t)f_rx_msg->au8Data[2]<<8);
    sub_index = f_rx_msg->au8Data[3];
    
    for(idx=0; idx<m_objdict_idx2; idx++ )
    {
        for(i=0; i< m_objdict_size2[idx]; i++ )
        {
            if( m_pobjdict2[idx][i].index == index && m_pobjdict2[idx][i].sub_index == sub_index && m_pobjdict2[idx][i].id == id )//找到该结构，跳出
            {
                break;
            }
        }
        if( i < m_objdict_size2[idx] )//查询号小于该数组帧的长度，则找到该结构
        {
            break;
        }
    }
    
    if( idx == m_objdict_idx2 )//未找到该项
    {
        return;
    }

    p_data =  m_pobjdict2[idx][i].p_data;
    func = m_pobjdict2[idx][i].func;
    
    if( p_data != NULL )
    {
        switch( frame_type )
        {
            case 0x4F://报文包含一个字节的有效数据
                if( m_pobjdict2[idx][i].data_size >= 1 )
                {
                    p_data[0] = f_rx_msg->au8Data[4];
                }
                break;
            case 0x4B://报文包含两个字节的有效数据
                if( m_pobjdict2[idx][i].data_size >= 2 )
                {
                    p_data[0] = f_rx_msg->au8Data[4];
                    p_data[1] = f_rx_msg->au8Data[5];
                }
                break;
            case 0x47://报文包含三个字节的有效数据
                if( m_pobjdict2[idx][i].data_size >= 3 )
                {
                    p_data[0] = f_rx_msg->au8Data[4];
                    p_data[1] = f_rx_msg->au8Data[5];
                    p_data[2] = f_rx_msg->au8Data[6];
                }
                break;
            case 0x43://报文包含四个字节的有效数据
                if( m_pobjdict2[idx][i].data_size >= 4 )
                {
                    p_data[0] = f_rx_msg->au8Data[4];
                    p_data[1] = f_rx_msg->au8Data[5];
                    p_data[2] = f_rx_msg->au8Data[6];
                    p_data[3] = f_rx_msg->au8Data[7];
                }
                break;
            case 0x60://写命令的回复
                break;
            default:
                return;
        }
    }
    
    if( func != NULL )
    {
        func( NULL );
    }
    
    return;
}
void ObjdictDispatch3(stc_can_rx_frame_t *f_rx_msg)
{
    uint8_t id = 0;
    uint16_t index = 0;
    uint8_t sub_index = 0;
    uint8_t frame_type = 0;
    uint8_t i = 0;
    uint8_t idx = 0;
    uint8_t *p_data = NULL;
    pFunction func = NULL;

    if (f_rx_msg->DLC != 8 || !(f_rx_msg->u32ID & 0x580))
    {
        return;
    }

    id = f_rx_msg->u32ID - 0x600;
    frame_type = f_rx_msg->au8Data[0];
    index = (uint16_t)f_rx_msg->au8Data[1] | ((uint16_t)f_rx_msg->au8Data[2] << 8);
    sub_index = f_rx_msg->au8Data[3];

    for (idx = 0; idx < m_objdict_idx2; idx++)
    {
        for (i = 0; i < m_objdict_size2[idx]; i++)
        {
            if (m_pobjdict2[idx][i].index == index && m_pobjdict2[idx][i].sub_index == sub_index && m_pobjdict2[idx][i].id == id) // 找到该结构，跳出
            {
                break;
            }
        }
        if (i < m_objdict_size2[idx]) // 查询号小于该数组帧的长度，则找到该结构
        {
            break;
        }
    }

    if (idx == m_objdict_idx2) // 未找到该项
    {
        return;
    }

    p_data = m_pobjdict2[idx][i].p_data;
    func = m_pobjdict2[idx][i].func;

    if (p_data != NULL)
    {
        switch (frame_type)
        {
        case 0x2F: // 报文包含一个字节的有效数据
            if (m_pobjdict2[idx][i].data_size >= 1)
            {
                p_data[0] = f_rx_msg->au8Data[4];
            }
            break;
        case 0x2B: // 报文包含两个字节的有效数据
            if (m_pobjdict2[idx][i].data_size >= 2)
            {
                p_data[0] = f_rx_msg->au8Data[4];
                p_data[1] = f_rx_msg->au8Data[5];
            }
            break;
        case 0x27: // 报文包含三个字节的有效数据
            if (m_pobjdict2[idx][i].data_size >= 3)
            {
                p_data[0] = f_rx_msg->au8Data[4];
                p_data[1] = f_rx_msg->au8Data[5];
                p_data[2] = f_rx_msg->au8Data[6];
            }
            break;
        case 0x23: // 报文包含四个字节的有效数据
            if (m_pobjdict2[idx][i].data_size >= 4)
            {
                p_data[0] = f_rx_msg->au8Data[4];
                p_data[1] = f_rx_msg->au8Data[5];
                p_data[2] = f_rx_msg->au8Data[6];
                p_data[3] = f_rx_msg->au8Data[7];
            }
            break;
        case 0x60: // 写命令的回复
            break;
        default:
            return;
        }
    }
    if (func != NULL)
    {
        func(NULL);
    }
    return;
}
/**********************************************************************************************
*介	 绍	：字典里目标索引对应数据
*参  数 ：f_index四个字节如0x01200201 指ID为1的索引为0x2002 0x01的数据
***********************************************************************************************/
uint32_t GetValueByIndex(uint32_t f_index)
{
	uint8_t i=0;
	uint8_t idx=0;
	
	for( idx = 0; idx < m_objdict_idx; idx++ ) // m_objdict_idx 字典数目
	{
			if( m_pobjdict[idx][0].id == (f_index>>24) ) 
			{
				break;  //找到目标节点ID的字典 
			}
	}
	
	if(idx < m_objdict_idx)   //找到结构
	{
		uint32_t tem_index = 0;
		
		for(int i=0;i<m_objdict_size[idx];i++)
		{
			tem_index = (m_pobjdict[idx][i].index<<8)|m_pobjdict[idx][i].sub_index;
			if(tem_index == (f_index&0XFFFFFF))
			{
									
				if(m_pobjdict[idx][i].data_size ==1)
				{
					return *((uint8_t*)m_pobjdict[idx][i].p_data);
				}
				if(m_pobjdict[idx][i].data_size ==2)
				{
					return *((uint16_t*)m_pobjdict[idx][i].p_data);
				}
				if(m_pobjdict[idx][i].data_size ==4)
				{
					return *((uint32_t*)m_pobjdict[idx][i].p_data);
				}
				
			}
		}			
	}
	else  //未找到该结构，跳出
	{
		return 0 ;
	}
}	
