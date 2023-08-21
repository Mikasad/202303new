#ifndef _OBJDICT_H
#define _OBJDICT_H
//#include "stm32f4xx_hal.h"
//#include "can_task.h"
#include "main.h"

/** this are static defined datatypes taken fCODE the canopen standard. They
 *  are located at index 0x0001 to 0x001B. As described in the standard, they
 *  are in the object dictionary for definition purpose only. a device does not
 *  to support all of this datatypes.
 */
#define boolean         0x01
#define int8            0x02
#define int16           0x03
#define int32           0x04
#define uint8           0x05
#define uint16          0x06
#define uint32          0x07
#define real32          0x08
#define visible_string  0x09
#define octet_string    0x0A
#define unicode_string  0x0B
#define time_of_day     0x0C
#define time_difference 0x0D

#define domain          0x0F
#define int24           0x10
#define real64          0x11
#define int40           0x12
#define int48           0x13
#define int56           0x14
#define int64           0x15
#define uint24          0x16

#define uint40          0x18
#define uint48          0x19
#define uint56          0x1A
#define uint64          0x1B


#define RW     0x00  
#define WO     0x01
#define RO     0x02


typedef void (*pFunction)(void*);

typedef struct Objdict_s
{
    uint8_t id;
    uint16_t index;//索引
    uint8_t sub_index;//子索引
    uint8_t data_type;//数据类型
    uint8_t data_size;//数据长度，字节数
    uint8_t rw;//读写类型
    void* p_data;//返回数据存放的地址
    pFunction func;//回调函数地址，在收到从机回复时调用
}Objdict_t;


void ObjdictLoad( const Objdict_t* obj, uint8_t size );

void ObjdictDispatch( stc_can_rx_frame_t* f_rx_msg );

void ObjdictTest(void);



void ObjdictLoad2( const Objdict_t* obj, uint8_t size );

void ObjdictDispatch2( stc_can_rx_frame_t * f_rx_msg );

uint32_t GetValueByIndex(uint32_t f_index);
void ObjdictDispatch3( stc_can_rx_frame_t * f_rx_msg );
#endif
