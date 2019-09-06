/**@file    nrf_lis3dh.c
* @brief   	基于nRF的lis3DH驱动
* @details  主要配置其低功耗模式、采集速率、中断IO配置
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-08-17
* @version 	V1.0
* @copyright	Copyright (c) 2016-2018  江苏亨通光网科技有限公司
**********************************************************************************
* @attention
* 硬件平台:nRF52832_QFAA \n
* SDK版本：nRF5_SDK_15.0.0
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/08/17  <td>1.0      <td>wanghuan  <td>创建初始版本
* </table>
*/

/**@defgroup nrf_lis3dh Bsp lis3dh module.
* @{
* @ingroup bsp_drivers
* @brief 基于nRF的lis3DH驱动. \n
* 主要配置其低功耗模式、采集速率、中断IO配置 \n
* 驱动宏开关： GNT_LIS3DH_EN
*/

#ifndef __NRF_LIS3DH_H
#define	__NRF_LIS3DH_H


#include "gnt_includes.h"
#if (GNT_LIS3DH_EN == 1)

#if 0
// IODM事件
typedef enum
{
    IODM_EVENT_NOTHING = 0,                  /**< Assign this event to an action to prevent the action from generating an event (disable the action). */
    IODM_EVENT_DEFAULT,                      /**< Assign this event to an action to assign the default event to the action. */
    IODM_EVENT_DISCONNECT,                   /**< A link should be disconnected. */
    IODM_EVENT_ADVERTISING_START,            /**< The device should start advertising. */
    IODM_EVENT_ADVERTISING_STOP,             /**< The device should stop advertising. */
    IODM_EVENT_WHITELIST_OFF,                /**< The device should remove its advertising whitelist. */
    IODM_EVENT_BOND,                         /**< The device should bond to the currently connected peer. */
    IODM_EVENT_RESET,                        /**< The device should reset. */
    IODM_EVENT_SLEEP,                        /**< The device should enter sleep mode. */
    IODM_EVENT_WAKEUP,                       /**< The device should wake up from sleep mode. */
    IODM_EVENT_DFU,                          //设备进入DPU 升级模式
    IODM_EVENT_LISINT1,                      //门锁1干簧管事件
} gnt_event_t;


typedef void (* gnt_event_callback_t)(gnt_event_t);   //应用层板级事件回调


// 调用函数申明
void gnt_gpio_config(uint32_t ticks_per_100ms, gnt_event_callback_t callback);   //门禁IO输入、输出配置
#endif


/**@brief 开启I2C外设 */
void lis3dh_twi_enable(void);

/**@brief 关闭I2C外设 */
void lis3dh_twi_disable(void);

/**@brief lis3dh进入掉电模式 */
void lis3dh_power_off(void);

/**@brief lis3dh清中断锁存 */
void lis3dh_ResetInt1Latch(uint8_t err_try);

/**@brief lis3dh初始化 \n
* lis3dh运动检测中断配置
*/
void lis3dh_init(void);


#endif  // #if (GNT_LIS3DH_EN == 1)
#endif	/* __NRF_LIS3DH_H */

/** @} nrf_lis3dh*/

/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/



