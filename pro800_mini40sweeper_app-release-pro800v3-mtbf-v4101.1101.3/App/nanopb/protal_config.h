/********************************************************************************
*File Name   : protal_config.h
*Copyright   : Copyright (C) 2016-2016 Gaussian Robot,All Ringhts Reserved
*
*Create Date : 2016/10/24
*Author      : YiPeng
*Description : 定义上下位机通行的数据结构，nanopb流程为 
*              发送：  约定结构Message--结构赋值--压缩结构--发送压缩数据
*              接收：  收到压缩数据--接压缩数据--用约定的结构Message接受数据
*--------------------------------------------------------
*No  Version  Date       Recised By  Item    Description
*1   V0.1     16.10.24   Yipeng              约定上下位机通信的结构
*
*********************************************************************************/

#ifndef _PROTAL_CONFIG_H
#define _PROTAL_CONFIG_H
#include "data_type.h"

#pragma anon_unions  //为了定义匿名共用体
/**********************************************************************************************************************************
*本文件与nanopb.pb.h nanopb.pb.c为上下位机通用，有一方要更改此三个文件，更改后必须将此三个文件重新提交给另一方
*且要在本文件中写上更改说明；
*
*
*结构数据存放说明
*
*CommandMessage.cmd  31-24   32-16    15-8    7-0   
*                     预留  电机控制  手自动  刹车
*
* 说明：电机控制对应bit置1为运行，置0为停止命令；
*        手自动切换0：不做切换 1：切换手动  2：切换自动
*       刹车命令  0：抬起刹车  1：放下刹车
*
***********************************************************************************************************************************/
/*---------HeaderMsg CMD enmu--------*/
enum cmd
{
	CFG_MSG = 1, 
	COUNTS_MSG,
	RELAY_MSG,
	IMU_MSG,
	IO_OUT_MSG,
	MACHINE_STATUS_MSG,
	POSITION_MSG,
	POSITION_RETURN_MSG,
	SPEED_MSG,
	UWB_MSG,
	ULTRASONIC_MSG,
	VERIFY_MSG,
	VERSION_MSG
};

/*----------Motor CMD Bit Field----------*/







#endif 
