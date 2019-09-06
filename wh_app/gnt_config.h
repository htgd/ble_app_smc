/**@file	gnt_config.h
* @brief   	项目功能配置文件
* @details  主要包含协议应用栈程序框架，main函数入口
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2019-01-10
* @version 	V1.0
* @copyright	Copyright (c) 2018-2020  江苏亨通光网科技有限公司
**********************************************************************************
* @attention
* 硬件平台:nRF52832_QFAA \n
* SDK版本：nRF5_SDK_15.0.0
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/08/17  <td>1.0      <td>wanghuan  <td>创建初始版本
* </table>
*
**********************************************************************************
*/

/**@defgroup gnt_config Wh gnt_config
* @{
* @ingroup app_smc
* @brief   项目代码及功能配置宏开关.
* @details 主要是设备版本控制、NB通信参数配置、附加模块配置以及一些调试功能开关. \n
* @warning 部分配置不能同时设置，请留意配置注释
*/

#ifndef __GNT_CONFIG_H
#define	__GNT_CONFIG_H


// <<< Use Configuration Wizard in Context Menu >>>\n


//  <h> 功能配置 - Function config
//  <i> 功能配置
//  <i> 主要用来控制调试模式下的一些功能选项，如蓝牙打印开关、任务堆栈监测等
//==========================================================

//  <h> 设备版本管理
//      <o> 设备版本 - BOARD_GNT_VERSION
//      <i> 硬件版本
//      <i> 不同硬件版本其使用引脚或功能有不同，软件上作兼容
//      <1=> BOARD_GNT_V1 
//      <2=> BOARD_GNT_V2 
//      <3=> BOARD_GNT_V3 
//      <s0.15> 软件版本号 - DEVICE_SW_REVISION
//      <s1.15> 硬件版本号 - DEVICE_HW_REVISION
//  </h> 
#define BOARD_GNT_VERSION           3
#define DEVICE_SW_REVISION          "S02010041905301"
#define DEVICE_HW_REVISION          "HGTNBGNT0103001"


//  <h> 标签功能及参数配置

//  <o> NB默认云平台 - GNT_NB_DEFAULT_PLATFORM
//  <i> 1:NB默认所有BAND使用Oceanconnect平台;
//  <i> 2:移动NB卡注册OneNet，电信、联通NB卡注册电信Oceanconnect平台; 
//  <1=> NB_DEFAULT_PLATFORM_OCEANCONNECT 
//  <2=> NB_DEFAULT_PLATFORM_ONENET_OCEANCONNECT
#define GNT_NB_DEFAULT_PLATFORM  1

//  <o> 标签心跳上报周期 - GNT_DEFAULT_REPORT_PERIOD
//  <i> 上报周期单位ms，通过NB-IoT上报
//      <600000=> 10分钟（用于测试）
//      <1800000=> 30分钟（用于测试）
//      <3600000=> 1小时 
//      <7200000=> 2小时 
//      <10800000=> 3小时 
//      <14400000=> 4小时 
//      <18000000=> 5小时 
//      <21600000=> 6小时
//      <25200000=> 7小时
//      <28800000=> 8小时
//      <32400000=> 9小时
//      <36000000=> 10小时
//      <39600000=> 11小时
//      <43200000=> 12小时
#define GNT_DEFAULT_REPORT_PERIOD   (10800000)

// <o> 周期上报连续失败复位 - GNT_REPORT_CONTINUOUS_FAIL_TIMES_TO_RESET <1-10>
// <i> 周期上报连续失败复位
#define GNT_REPORT_CONTINUOUS_FAIL_TIMES_TO_RESET    ( 4 ) 

//  <q> 井盖告警队列功能 - SMC_ALRAM_QUEUE_EN
//  <i> 1使能井盖告警队列功能，0关闭井盖告警队列功能
#define SMC_ALRAM_QUEUE_EN	0

//  <q> 井盖关闭不上报 - SMC_CLOSE_REPORT_EN
//  <i> 1使能井盖告警队列功能，0关闭井盖告警队列功能
#define SMC_CLOSE_REPORT_EN	0

//  <e> 标签看门狗使能 - GNT_WDT_EN
//      <o1> 看门狗sleep和halt行为 - GNT_WDT_CONFIG_BEHAVIOUR
//      <i> WDT behavior in CPU SLEEP or HALT mode
//      <1=> Run in SLEEP, Pause in HALT 
//      <8=> Pause in SLEEP, Run in HALT 
//      <9=> Run in SLEEP and HALT 
//      <0=> Pause in SLEEP and HALT
//      <o2> 看门狗装载值 - GNT_WDT_CONFIG_RELOAD_VALUE <#*3600000>
//      <i> WDT Reload value 小时
//      <o3> 看门狗中断优先级  - GNT_WDT_CONFIG_IRQ_PRIORITY
//      <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
//      <0=> 0 (highest) 
//      <1=> 1 
//      <2=> 2 
//      <3=> 3 
//      <4=> 4 
//      <5=> 5 
//      <6=> 6 
//      <7=> 7
//  </e>
#define GNT_WDT_EN   1
#define GNT_WDT_CONFIG_BEHAVIOUR   1
#define GNT_WDT_CONFIG_RELOAD_VALUE   (10800000)
#define GNT_WDT_CONFIG_IRQ_PRIORITY   7

//  <e> 标签掉电比较器使能 - GNT_POF_EN
//  <i> <i> 1：使能蓝牙芯片掉电检测， 0：不使能
#define GNT_POF_EN   0
//  </e>

//  <e> 标签加速度传感器使能 - GNT_LIS3DH_EN
//  <i> 1使用加速度传感器， 0，设备未采用加速度传感器
#define GNT_LIS3DH_EN   0
//  </e>

//  <e> 标签GPS使能 - GNT_GPS_EN
//  <i> 1使用GPS功能， 0，设备未采用GPS功能
#define GNT_GPS_EN   0
//  <o> GPS默认定位模式 - GNT_GPS_DEFAULT_MODE
//  <i> 1:GPS默认工作模式GPS_ALONE,   2:  GPS默认工作模式AGPS
//  <i> 该参数设置flash保存
//  <1=> ME3616_GPS_MODE_ALONE 
//  <2=> ME3616_GPS_MODE_AGPS
#define GNT_GPS_DEFAULT_MODE  2
//  <q> GPS默认实时上报使能 - GNT_GPS_DEFAULT_REPORT_EN
#define GNT_GPS_DEFAULT_REPORT_EN 1
//  </e>
//==End of 标签GPS使能

//  </h>
//=====End of 标签功能及参数配置


//  <h> 蓝牙通信配置

//  </h>
//=====End of 蓝牙通信配置


//  <e> 固件升级 - DFU_SUPPORT
//      <q1> DFU_COM
//      <i> 1使能DFU服务及功能，程序不能直接下载，需通过DFU更新程序  
//      <i> 0关闭DFU服务及功能，适用于程序的修改调试
//  </e> 
#define DFU_SUPPORT     1
#define DFU_COM         0

//  <h> 调试功能
//      <q> 蓝牙日志打印 - GNT_BLE_LOG_EN
//      <i> 1使能蓝牙打印监测，0关闭蓝牙打印监测
#define GNT_BLE_LOG_EN          1

//      <q> 任务堆栈监测 - TASK_HEAP_MONITOR_EN
//      <i> 任务堆栈使用情况监测，1->使能，0->关闭
#define TASK_HEAP_MONITOR_EN    0


//      <q> 加速度传感器测试模式 - GNT_LIS3DH_TEST_EN
//      <i> 1：使能改模式， 0：不使能
//      <i> 该模式下NB相关任务不启动，用于快速测试LIS3DH功能是否正常
#define GNT_LIS3DH_TEST_EN  0

//      <q> NB信号测试模式 - GNT_NB_TEST_EN
//      <i> 1：使能改模式， 0：不使能
//      <i> 该模式下主要测试NB网络的注册情况和信号强度检测
//      <i> 注意不同测试模式不能同时使能
#define GNT_NB_TEST_EN  0

//  </h> 
//=====End of 调试功能

//  </h> 
//===========================================End of 功能配置



// <h> 电池保护电量阈值
// <i> 电量阈值
// <i> 标签各项功能在不同电量状态下进行省电或保护处理
//==========================================================
// <o> GNT_ENABLE_BATTERY_THRESHOLD <1-20>
// <i> 标签使能所需最低电量，当电量低于该值时标签不能使能，
// <i> 任何原因的复位当电量低于该值时标签自动切换为未使能状态
#define GNT_ENABLE_BATTERY_THRESHOLD            ( 3 ) 

// <o> GNT_REPORT_DISABLE_BATTERY_THRESHOLD <1-50>
// <i> 关闭NB上报电池电量阈值
#define GNT_REPORT_DISABLE_BATTERY_THRESHOLD    ( 1 ) 

// <o> GNT_REPORT_RESUME_BATTERY_THRESHOLD <1-50>
// <i> 恢复NB上报电池电量阈值
// <i> 当NB上报关闭后，电量需恢复一定值才使能上报
#define GNT_REPORT_RESUME_BATTERY_THRESHOLD     ( 15 ) 

// <o> GNT_GPS_DISABLE_BATTERY_THRESHOLD <5-50>
// <i> 关闭GPS定位电池电量阈值
#define GNT_GPS_DISABLE_BATTERY_THRESHOLD       ( 15 ) 

// <o> GNT_GPS_RESUME_BATTERY_THRESHOLD <1-50>
// <i> 恢复GPS定位电池电量阈值
// <i> 当GPS定位因电量低关闭后，电量需恢复一定值才恢复GPS定位功能
#define GNT_GPS_RESUME_BATTERY_THRESHOLD        ( 25 ) 


// </h> 
//======================End of 电池保护电量阈值



// <<< end of configuration section >>>



#endif	/* __GNT_CONFIG_H */

/** @} gnt_config*/

/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/

