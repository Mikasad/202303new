/*
 * @Author: Jack Yi
 * @Date: 2023-07-01 15:28:40
 * @LastEditors: Jack Yi
 * @LastEditTime: 2023-07-01 15:30:17
 * @FilePath: \pro800_mini40sweeper_app\App\version.h
 * @Description: 
 * 
 * Copyright (c) 2023 by Jack Yi, All Rights Reserved. 
 */
#ifndef __VERSION_H_
#define __VERSION_H_

#include <stdint.h>
/*备注chassis没有范围�?, 但OTA要求，�?�果�?一位是1 那�??二位�?能是0-9, 如果前两位是1.9的话,�?三位�?能是0-9*/
//主版�?
#define MAIN_VERSION_TYPE 	0
#define MAIN_VERSION_MAJOR 	4101
#define MAIN_VERSION_MINOR 	1101                                            
#define MAIN_VERSION_BUILD 	3

//日期版本
#define DATE_VERSION_TYPE 	0x20
#define DATE_VERSION_MAJOR 	0x23
#define DATE_VERSION_MINOR 	0x07
#define DATE_VERSION_BUILD 	0x03

//字母版本
#define LETTER_VERSION_BASE 	"base"			//基板
#define LETTER_VERSION_ALPHA 	"alpha"			//初�?�版�?
#define LETTER_VERSION_BETA 	"beta"			//测试版本
#define LETTER_VERSION_RC 		"rc"			//候选版�?
#define LETTER_VERSION_RELEASE 	"release"		//发布版本

#define LETTER_VERSION LETTER_VERSION_BETA



typedef struct{
	uint32_t type;
	uint32_t major;
	uint32_t minor;
	uint32_t build;	
}version_t;




#endif
