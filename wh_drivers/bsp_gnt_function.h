/**@file  bsp_gnt_function.h
* @brief  GNT其他杂项功能的实现，功能列表： \n
* -# GNT使用IO按键配置
* -# 电量采集
* -# 温度采集
* -# GNT定制LED操作
* -# GNT位置信息锁定及附加操作
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2019-01-17
* @version 	V1.0
* @copyright	Copyright (c) 2016-2018  江苏亨通光网科技有限公司
**********************************************************************************
* @attention
* 硬件平台:nRF52832_QFAA \n
* SDK版本：nRF5_SDK_15.0.0
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2019/01/17  <td>1.0      <td>wanghuan  <td>创建初始版本
* </table>
*/

/**@defgroup bsp_gnt_function Bsp gnt_function module.
* @{
* @ingroup bsp_drivers
* @brief 设备杂项操作接口. \n
* -# GNT使用IO按键配置
* -# 电量采集
* -# 温度采集
* -# GNT定制LED操作
* -# GNT位置信息锁定及附加操作
*/

#ifndef __BSP_GNT_FUNCTION_H
#define	__BSP_GNT_FUNCTION_H

#include "gnt_includes.h"


/**@brief GNT标签的LED指示事件表 */
typedef enum
{
    LED_USB_IN         = 0,
    LED_USB_OUT,
    LED_BLE_CONNECT,
    LED_BLE_DISCONNECT,
    LED_GPS_DATA_READY,
    LED_GPS_DATA_NONE,
    LED_KEY,
    LED_NB_REGISTING,
    LED_NB_REGIST_OVER,
    
} GNT_led_event_t;


#if (GNT_WDT_EN == 1)
/**@brief 看门狗默认配置. */
#define GNT_WDT_DEAFULT_CONFIG                                               \
    {                                                                        \
        .behaviour          = (nrf_wdt_behaviour_t)GNT_WDT_CONFIG_BEHAVIOUR, \
        .reload_value       = GNT_WDT_CONFIG_RELOAD_VALUE,                   \
        .interrupt_priority = GNT_WDT_CONFIG_IRQ_PRIORITY,                   \
    }
#endif

/**@brief 设备使用按键配置 */
uint32_t bsp_btn_gnt_init(void);
	
/**@brief 电量采集ADC配置 */
uint8_t battery_adc_sample(void);

/**@brief   电池电量均值法采集，并转为电压值，再计算为百分比
* @return 	电量百分比值
*/
int32_t gnt_temp_get(void);

/**@brief   电池电量均值法采集，并转为电压值，再计算为百分比
* @param[in]  led_event   触发事件对应的LED状态
*/
void gnt_led_indication_set(GNT_led_event_t led_event);

/**@brief GNT位置信息锁定及偏离比较值计算
* @param[in]  *lock_latitude 	锁定维度指针
* @param[in]  *lock_longitude 	锁定经度指针
* @param[in]  funcType 	处理方式
* - 1：锁定GPS信息写入falsh，并计算lock偏离比较值；
* - 2：仅计算lock的com值
* - 3：仅计算last的com值
*/
void gnt_lock_location(char *lock_latitude, char *lock_longitude, uint8_t funcType);

/**@brief   看门狗初始化
* @param[in]  wdt_reload_value   看门狗配置周期
*/
void my_wdt_init(uint32_t wdt_reload_value);
	
/**@brief 看门狗喂狗 */
void my_wdt_feed(void);
    


#endif	/* __BSP_GNT_FUNCTION_H */







