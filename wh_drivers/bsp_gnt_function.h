/**@file  bsp_gnt_function.h
* @brief  GNT��������ܵ�ʵ�֣������б� \n
* -# GNTʹ��IO��������
* -# �����ɼ�
* -# �¶Ȳɼ�
* -# GNT����LED����
* -# GNTλ����Ϣ���������Ӳ���
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2019-01-17
* @version 	V1.0
* @copyright	Copyright (c) 2016-2018  ���պ�ͨ�����Ƽ����޹�˾
**********************************************************************************
* @attention
* Ӳ��ƽ̨:nRF52832_QFAA \n
* SDK�汾��nRF5_SDK_15.0.0
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2019/01/17  <td>1.0      <td>wanghuan  <td>������ʼ�汾
* </table>
*/

/**@defgroup bsp_gnt_function Bsp gnt_function module.
* @{
* @ingroup bsp_drivers
* @brief �豸��������ӿ�. \n
* -# GNTʹ��IO��������
* -# �����ɼ�
* -# �¶Ȳɼ�
* -# GNT����LED����
* -# GNTλ����Ϣ���������Ӳ���
*/

#ifndef __BSP_GNT_FUNCTION_H
#define	__BSP_GNT_FUNCTION_H

#include "gnt_includes.h"


/**@brief GNT��ǩ��LEDָʾ�¼��� */
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
/**@brief ���Ź�Ĭ������. */
#define GNT_WDT_DEAFULT_CONFIG                                               \
    {                                                                        \
        .behaviour          = (nrf_wdt_behaviour_t)GNT_WDT_CONFIG_BEHAVIOUR, \
        .reload_value       = GNT_WDT_CONFIG_RELOAD_VALUE,                   \
        .interrupt_priority = GNT_WDT_CONFIG_IRQ_PRIORITY,                   \
    }
#endif

/**@brief �豸ʹ�ð������� */
uint32_t bsp_btn_gnt_init(void);
	
/**@brief �����ɼ�ADC���� */
uint8_t battery_adc_sample(void);

/**@brief   ��ص�����ֵ���ɼ�����תΪ��ѹֵ���ټ���Ϊ�ٷֱ�
* @return 	�����ٷֱ�ֵ
*/
int32_t gnt_temp_get(void);

/**@brief   ��ص�����ֵ���ɼ�����תΪ��ѹֵ���ټ���Ϊ�ٷֱ�
* @param[in]  led_event   �����¼���Ӧ��LED״̬
*/
void gnt_led_indication_set(GNT_led_event_t led_event);

/**@brief GNTλ����Ϣ������ƫ��Ƚ�ֵ����
* @param[in]  *lock_latitude 	����ά��ָ��
* @param[in]  *lock_longitude 	��������ָ��
* @param[in]  funcType 	����ʽ
* - 1������GPS��Ϣд��falsh��������lockƫ��Ƚ�ֵ��
* - 2��������lock��comֵ
* - 3��������last��comֵ
*/
void gnt_lock_location(char *lock_latitude, char *lock_longitude, uint8_t funcType);

/**@brief   ���Ź���ʼ��
* @param[in]  wdt_reload_value   ���Ź���������
*/
void my_wdt_init(uint32_t wdt_reload_value);
	
/**@brief ���Ź�ι�� */
void my_wdt_feed(void);
    


#endif	/* __BSP_GNT_FUNCTION_H */







