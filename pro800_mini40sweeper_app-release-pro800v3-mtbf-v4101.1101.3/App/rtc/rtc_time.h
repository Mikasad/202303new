#ifndef _RTC_TIME_H
#define _RTC_TIME_H

#include "main.h"


typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
}RtcTime_t;



void SetTime( RtcTime_t time, uint8_t f_mode );

int SetTimeAscii( char* f_time );
void GetTime( RtcTime_t* f_time );

uint32_t GetUtc8(void);
void SetTimeUtc8( uint32_t f_utc );
uint32_t Time2Utc(uint16_t f_year, uint8_t f_month, uint8_t f_day, uint8_t f_hour, uint8_t f_min, uint8_t f_sec);
void Utc2Time(RtcTime_t* fp_time, uint32_t f_utc);
void SetSysUtc8(uint32_t f_utc8);


#endif
