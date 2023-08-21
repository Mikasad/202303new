#include "rtc_time.h"
#include "rtc.h"
#include "log.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>



void SetTime( RtcTime_t f_time, uint8_t f_mode )
{
	HAL_StatusTypeDef result;
	RTC_TimeTypeDef rtcTimeNow = {0};
	RTC_DateTypeDef rtcDateNow = {0};
    LogTime_t log = {0};
    
    log.head.byte.id = ID_SET_TIME;
    log.type = f_mode;
    log.utc_old = GetUtc8();

    result = HAL_RTC_GetTime(&hrtc, &rtcTimeNow, RTC_FORMAT_BIN);
    result = HAL_RTC_GetDate(&hrtc, &rtcDateNow, RTC_FORMAT_BIN);
    
    f_time.year = f_time.year%2010;

    rtcTimeNow.Hours = f_time.hour;
    rtcTimeNow.Minutes = f_time.min;
    rtcTimeNow.Seconds = f_time.sec;

    rtcTimeNow.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    rtcTimeNow.StoreOperation = RTC_STOREOPERATION_RESET;

    rtcDateNow.Year = f_time.year;
    rtcDateNow.Month = f_time.month;
    rtcDateNow.Date = f_time.day;
    rtcDateNow.WeekDay = RTC_WEEKDAY_FRIDAY;

    result = HAL_RTC_SetTime(&hrtc, &rtcTimeNow, RTC_FORMAT_BIN);
    result = HAL_RTC_SetDate(&hrtc, &rtcDateNow, RTC_FORMAT_BIN);
    (void)result;
    
    log.utc_new = GetUtc8();
    
    xQueueSend( g_log_queue, &log, 2);
}

int SetTimeAscii( char* f_time )
{
    char* p = NULL;
    RtcTime_t time = {0};
    
    p = strstr( f_time, "time:" );
    if( NULL != p )
    {
        p += 5;
        time.year = atoi( p );
        p = strstr( p, " " );
        if( NULL != p )
        {
            p +=1;
            time.month = atoi( p );
            p = strstr( p, " " );
            if( NULL != p )
            {
                p +=1;
                time.day = atoi( p );
                p = strstr( p, " " );
                if( NULL != p )
                {
                    p +=1;
                    time.hour = atoi( p );
                    p = strstr( p, " " );
                    if( NULL != p )
                    {
                        p +=1;
                        time.min = atoi( p );
                        p = strstr( p, " " );
                        if( NULL != p )
                        {
                            p +=1;
                            time.sec = atoi( p );
                            
                            if( time.year >= 2019 && time.month>0 && time.month<=12 && time.day>0 && time.day<=31 && time.hour<24 && time.min<60 && time.sec<60 )
                            {
                                SetTime( time, 1 );
                                return 1;
                            }
                        }
                    }
                }
            }
        }
    }
    
    return 0;
}

void GetTime( RtcTime_t* f_time )
{
    RTC_TimeTypeDef rtcTimeNow = {0};
	RTC_DateTypeDef rtcDateNow = {0};
    
    if( f_time != NULL )
    {
        HAL_RTC_GetTime(&hrtc, &rtcTimeNow, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &rtcDateNow, RTC_FORMAT_BIN);
        
        f_time->year = rtcDateNow.Year+2010;
        f_time->month = rtcDateNow.Month;
        f_time->day = rtcDateNow.Date;
        f_time->hour = rtcTimeNow.Hours;
        f_time->min = rtcTimeNow.Minutes;
        f_time->sec = rtcTimeNow.Seconds;
    }
}

uint32_t GetUtc8(void)
{
    RtcTime_t time = {0};
    GetTime(&time);
    return Time2Utc( time.year, time.month, time.day, time.hour, time.min, time.sec );
}

void SetTimeUtc8( uint32_t f_utc )
{
    RtcTime_t time = {0};
    Utc2Time( &time, f_utc );
    SetTime( time, 2 );
}

//日期时间转UTC
uint32_t Time2Utc(uint16_t f_year, uint8_t f_month, uint8_t f_day, uint8_t f_hour, uint8_t f_min, uint8_t f_sec)
{
	uint32_t utc = 0;
	uint16_t year = 1970;
	uint8_t month = 1;

	if (f_year < 1970 || f_month<1 || f_month>12 || f_hour>23 || f_min>59 || f_sec >59 || f_day<1)
	{
		return 0;
	}
    if( f_month==1 || f_month==3 || f_month==5 || f_month==7 || f_month==8 || f_month==10 || f_month==12 )
    {
        if( f_day > 31 )
        {
            return 0;
        }
    }
    else if( f_month==4 || f_month==6 || f_month==9 || f_month==11 )
    {
        if( f_day > 30 )
        {
            return 0;
        }
    }
    else
    {
        if ((((f_year % 4) == 0) && ((f_year % 100) != 0)) || ((f_year % 400) == 0))
        {
            if( f_day > 29 )
            {
                return 0;
            }
        }
        else
        {
            if( f_day > 28 )
            {
                return 0;
            }
        }
    }
    

	for (; year < f_year; year++)
	{
		if ((((year % 4) == 0) && ((year % 100) != 0)) || ((year % 400) == 0))
		{
			utc += 31622400;
		}
		else
		{
			utc += 31536000;
		}
	}
	for (; month < f_month; month++)
	{
		if (month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10)
		{
			utc += 2678400;
		}
		else if (month == 4 || month == 6 || month == 9 || month == 11)
		{
			utc += 2592000;
		}
		else
		{
			if ((((f_year % 4) == 0) && ((f_year % 100) != 0)) || ((f_year % 400) == 0))
			{
				utc += 2505600;
			}
			else
			{
				utc += 2419200;
			}
		}
	}
	utc += (f_day - 1) * 24 * 3600;
	utc += f_hour * 3600;
	utc += f_min * 60;
	utc += f_sec;

	return utc;
}

//utc转日期时间
void Utc2Time(RtcTime_t* fp_time, uint32_t f_utc)
{
	uint16_t year = 1970;
	uint8_t month = 1;
    
	if (fp_time == NULL)
	{
		return;
	}
    
	fp_time->sec = f_utc % 60;
	f_utc /= 60; //分钟
	fp_time->min = f_utc % 60;
	f_utc /= 60; //小时
	fp_time->hour = f_utc % 24;
	f_utc /= 24; //天

	while (f_utc >= 365)
	{
		if ((((year % 4) == 0) && ((year % 100) != 0)) || ((year % 400) == 0))
		{
			if (f_utc >= 366)
			{
				year++;
				f_utc -= 366;
			}
			else
			{
				break;
			}
			
		}
		else
		{
			year++;
			f_utc -= 365;
		}
	}
	while (f_utc)
	{
		if (month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10 || month == 12)
		{
			if (f_utc >= 31)
			{
				month++;
				f_utc -= 31;
			}
			else
			{
				break;
			}
		}
		else if (month == 4 || month == 6 || month == 9 || month == 11)
		{
			if (f_utc >= 30)
			{
				month++;
				f_utc -= 30;
			}
			else
			{
				break;
			}
		}
		else
		{
			if ((((year % 4) == 0) && ((year % 100) != 0)) || ((year % 400) == 0))
			{
				if (f_utc >= 29)
				{
					month++;
					f_utc -= 29;
				}
				else
				{
					break;
				}
			}
			else
			{
				if (f_utc >= 28)
				{
					month++;
					f_utc -= 28;
				}
				else
				{
					break;
				}
			}
		}
	}
	fp_time->year = year;
	fp_time->month = month;
	fp_time->day = f_utc + 1;
}

void SetSysUtc8(uint32_t f_utc8)
{
    uint32_t sys_utc = 0;
    
    sys_utc = GetUtc8();
    if( sys_utc+30 < f_utc8 || sys_utc > f_utc8+30 )
    {
        SetTimeUtc8( f_utc8 );
    }
}



