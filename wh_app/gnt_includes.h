/**@file    gnt_includes.c
* @brief   	GNT设备公共包含头文件
* @details  头文件包含、全局结构体、全局变量、全局函数
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-08-23
* @version 	V1.0
* @copyright	Copyright (c) 2018-2020  江苏亨通光网科技有限公司
**********************************************************************************
* @attention
* 硬件平台:nRF52832_QFAA \n
* SDK版本：nRF5_SDK_15.0.0
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/08/23  <td>1.0      <td>wanghuan  <td>创建初始版本
* </table>
*/

#ifndef __GNT_INCLUDES_H
#define	__GNT_INCLUDES_H

/* Includes ------------------------------------------------------------------*/
#include "gnt_config.h"
//--标准库
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

//--协议栈
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "peer_manager.h"   //设备管理
#include "fds.h"
#include "bsp_btn_ble.h"

//--FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"
#include "queue.h"


#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"   //低功耗管理

//--日志打印
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include <SEGGER_RTT_Conf.h>
#include <SEGGER_RTT.h>

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
//--GNT使用组件与驱动
#include "fds.h"
#include "app_uart.h"           //串口驱动组件
//#include "ble_bas.h"          //蓝牙电池服务
#include "nrf_drv_saadc.h"      //ADC驱动组件
//#include "ble_gnts.h"         //GPS NB-IOT TAG服务
#include "ble_gnts_bu.h"
#include "nrf_drv_twi.h"        //I2C驱动组件
#include "bsp_gnt_function.h"   //应用多项功能
#if (GNT_LIS3DH_EN == 1)
#include "bsp_lis3dh.h"         //LIS3DH驱动
#include "nrf_lis3dh.h"
#endif
#include "bsp_me3616.h"
#include "bsp_fds.h"
#include "bsp_fstorage.h"
#include "nrf_temp.h"
//#include "nrf_power.h"

#if (DFU_SUPPORT == 1)
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "ble_dfu.h"
#include "nrf_power.h"
#include "nrf_bootloader_info.h"
#endif

#if (GNT_WDT_EN == 1)
#include "nrf_drv_wdt.h"
#endif


// 全局任务句柄
#define NB_THREAD_PRIO		        6      	///< 任务优先级
#define NB_REPORT_THREAD_PRIO		4      	///< 任务优先级
#define BLE_COMM_THREAD_PRIO		5      	///< 任务优先级

extern TaskHandle_t    m_nb_thread;      	///< 任务句柄
extern void nb_thread(void *pvParameters); 	///< 任务函数
extern TaskHandle_t    m_nb_report_thread;          
extern void nb_report_thread(void *pvParameters);   
extern TaskHandle_t    m_ble_comm_thread;          
extern void ble_comm_thread(void *pvParameters);

#if (TASK_HEAP_MONITOR_EN == 1)
#define HEAP_MONITOR_PRIO		    2
extern TaskHandle_t    m_heap_monitor_thread;          
extern void heap_monitor_thread(void *pvParameters);
#endif


// 全局队列句柄  
extern xQueueHandle    	BLE_MsgQueue; 	///< 蓝牙通信数据队列
#if	SMC_ALRAM_QUEUE_EN
extern xQueueHandle		Key_Queue; 		///< 按键值消息队列句柄
#endif

// 全局事件标志组
extern EventGroupHandle_t   GntEventGroup;	///< 设备全局事件组

// 事件标志位（传感、按键等中断触发事件）
#define KEY_EVENTBIT                    (1 << 0)	///< 按键按下事件
#define KEY_LONGPUSH_EVENTBIT           (1 << 1)	///< 按键长按事件
#define KEY_RELEASE_EVENTBIT            (1 << 2)	///< 按键释放事件
#define LISINT1_EVENTBIT                (1 << 3)	///< 加速度传感器中断1事件
#define LISINT2_EVENTBIT                (1 << 4)	///< 加速度传感器中断2事件
#define NB_PRERIOD_REPORT_EVENTBIT      (1 << 5)	///< NB周期超时事件
#define GNT_CLEAR_MOVEFLAG_EVENTBIT     (1 << 6)	///< 设备清除标志事件


// GNT标签的全局标志与信息
//#pragma pack(4)
//__attribute__((aligned (4)));
// flash操作的全局变量，如果是结构体成员，首地址为4字节倍数
/**@brief 设备全局flash参数信息结构体 */
typedef struct
{
    char    ble_name[16];               ///< 蓝牙名称
    uint32_t gnt_enable_flag;           ///< GNT标签功能配置使能（即使能NB和GPS）    0：未使能，1：使能
    uint32_t gnt_report_period;         ///< 标签NB心跳上报周期 （单位秒）
    uint32_t gnt_iot_platform;          ///< 标签云平台选择
#if (GNT_GPS_EN == 1)
    char    gnt_lock_latitude[12];      ///< 标签锁定纬度
    char    gnt_lock_longitude[12];     ///< 标签锁定经度
    uint32_t gnt_move_alarm_flag;       ///< 标签搬迁告警（超过锁定位置50m）
    uint32_t gnt_gps_mode_flag;         ///< 标签GPS定位模式
#endif
    
}GNT_flash_info_t;

/**@brief 设备全局参数信息结构体 */
typedef struct
{
    uint32_t gnt_err_state;             ///< 标签工作错误状态（0表示无错误）
    
    uint8_t gnt_usbvin_flag;            ///< USB充电插入标志 0：拔出/未插入，1：插入
    uint8_t gnt_ble_connect_flag;       ///< BLE连接状态   1:连接，0：断开
    uint8_t gnt_current_led_state;      ///< 当前LED状态
    uint8_t gnt_battery_level;          ///< 当前标签电量值（非实时）
    
    uint8_t smc_open_state;             ///< 当前井盖开启状态 0：正常， 1：异常翻开
    
#if (GNT_GPS_EN == 1)
    uint8_t gnt_gps_data_ready_flag;    ///< GPS定位是否成功 1：GPS定位有数据  0：其他（无数据或未启动定位）
    uint8_t gnt_gps_battery_low_flag;   ///< 标签启动GPS低电量标志 0：没有低电量， 1：启动GPS电量低，电池需回复一定电量阈值才能重新启动GPS
    uint8_t gnt_gps_realtime_report_flag;   ///< 标签GPS信息实时上报标志 0：关闭， 1：开启
    char    gnt_last_latitude[12];      ///< 标签最后定位的坐标纬度
    char    gnt_last_longitude[12];     ///< 标签最后定位的坐标经度
    int     gnt_last_latitude_com;      ///< 标签最后纬度用于计算偏离的值
    int     gnt_last_longitude_com;     ///< 标签最后经度用于计算偏离的值
    int     gnt_lock_latitude_com;      ///< 标签锁定纬度用于计算偏离的值
    int     gnt_lock_longitude_com;     ///< 标签锁定经度用于计算偏离的值
#endif
    
}GNT_info_t;
//#pragma pack()

extern GNT_flash_info_t     g_gnt_flash_info;
extern GNT_info_t           g_gnt_info;


// 应用FALSH存储fds
#define GNT_FILE                        (0xF010)	///< 设备flash文件标号
#define GNT_ENABLE_FLAG_REC_KEY         (0x7101)	///< 设备使能标志记录标号
#define GNT_BLE_NAME_REC_KEY            (0x7102)	///< 设备蓝牙名称记录标号
#define GNT_REPORT_PERIOD_REC_KEY       (0x7107)	///< 设备上报周期记录标号
#define GNT_IOT_PLATFORM_REC_KEY        (0x7108)	///< 设备云平台配置记录标号
#if (GNT_GPS_EN == 1)
#define GNT_LOCK_LATITUDE_REC_KEY       (0x7103)
#define GNT_LOCK_LONGTITUDE_REC_KEY     (0x7104)
#define GNT_MOVE_ALARM_FLAG_REC_KEY     (0x7105)
#define GNT_GPS_MODE_REC_KEY            (0x7106)
#endif


// 全局函数
extern void Ble_send(uint8_t*data, uint16_t len);
extern void Ble_log_send(uint8_t*data, uint16_t len);   
extern void gnt_change_report_period(uint32_t period);

// 工具函数
extern uint16_t CTin(uint8_t * data, uint16_t m);
extern uint16_t CTout(uint8_t * data, uint16_t m);
extern uint16_t GetCrc_16(uint8_t * Data, uint16_t len);


/**@brief BLE参数设置功能列表 */
typedef struct
{
    void (*bleTxPowerSet)(int8_t);
    void (*bleAdvStart)(void);
    void (*bleAdvStop)(void);
    void (*bleSetName)(char *, uint8_t);
    void (*bleSetAdvTimeout)(uint32_t);
	void (*bleDisconnect)(void);
    
}BLE_FxnTable;

extern BLE_FxnTable nRF52_FxnTable;		///< BLE参数设置操作句柄


/**@brief GNT标签的使能（功能类型） */
typedef enum
{
    GNT_DISABLE         = 0,    // 所有器件处于最低功耗状态（唤醒后支持蓝牙通信）
    GNT_ENABLE          = 1,    // 标签全功能工作
    GNT_FOR_NB_DFU      = 2,    // 用于NB模组固件升级模式（上电模组不开机）
    
} GNT_enable_type_t;



#endif	/* __GNT_INCLUDES_H */


















