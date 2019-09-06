/**@file    nrf_lis3dh.c
* @brief   	����nRF��lis3DH����
* @details  ��Ҫ������͹���ģʽ���ɼ����ʡ��ж�IO����
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-08-17
* @version 	V1.0
* @copyright	Copyright (c) 2016-2018  ���պ�ͨ�����Ƽ����޹�˾
**********************************************************************************
* @attention
* Ӳ��ƽ̨:nRF52832_QFAA \n
* SDK�汾��nRF5_SDK_15.0.0
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/08/17  <td>1.0      <td>wanghuan  <td>������ʼ�汾
* </table>
*/

/**@defgroup nrf_lis3dh Bsp lis3dh module.
* @{
* @ingroup bsp_drivers
* @brief ����nRF��lis3DH����. \n
* ��Ҫ������͹���ģʽ���ɼ����ʡ��ж�IO���� \n
* �����꿪�أ� GNT_LIS3DH_EN
*/

#ifndef __NRF_LIS3DH_H
#define	__NRF_LIS3DH_H


#include "gnt_includes.h"
#if (GNT_LIS3DH_EN == 1)

#if 0
// IODM�¼�
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
    IODM_EVENT_DFU,                          //�豸����DPU ����ģʽ
    IODM_EVENT_LISINT1,                      //����1�ɻɹ��¼�
} gnt_event_t;


typedef void (* gnt_event_callback_t)(gnt_event_t);   //Ӧ�ò�弶�¼��ص�


// ���ú�������
void gnt_gpio_config(uint32_t ticks_per_100ms, gnt_event_callback_t callback);   //�Ž�IO���롢�������
#endif


/**@brief ����I2C���� */
void lis3dh_twi_enable(void);

/**@brief �ر�I2C���� */
void lis3dh_twi_disable(void);

/**@brief lis3dh�������ģʽ */
void lis3dh_power_off(void);

/**@brief lis3dh���ж����� */
void lis3dh_ResetInt1Latch(uint8_t err_try);

/**@brief lis3dh��ʼ�� \n
* lis3dh�˶�����ж�����
*/
void lis3dh_init(void);


#endif  // #if (GNT_LIS3DH_EN == 1)
#endif	/* __NRF_LIS3DH_H */

/** @} nrf_lis3dh*/

/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/



