/**@file  bsp_gnt_function.c
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

/* Includes ------------------------------------------------------------------*/
#include "bsp_gnt_function.h"

/* Private variables ---------------------------------------------------------*/

/* function prototypes -------------------------------------------------------*/
uint32_t bsp_btn_gnt_init(void);    ///< GNTʹ�ð�������
uint8_t battery_adc_sample(void);   ///< �����ɼ�
int32_t gnt_temp_get(void);         ///< �¶Ȳɼ�
void gnt_led_indication_set(GNT_led_event_t led_event); ///< GNT LEDָʾ��
void gnt_lock_location(char *lock_latitude, char *lock_longitude, uint8_t funcType);///< GNTλ����Ϣ������ƫ��Ƚ�ֵ����


/**@brief   �¶Ȳɼ����ڲ��¶ȴ���SAADCӦ�ã�
* @return 	�¶�ֵ��ʵ���¶�ֵ/10 ����0.5��
*/
int32_t gnt_temp_get(void)
{
    int32_t temp;
    
    sd_temp_get(&temp);
    return (int32_t)((temp*10)/4);
}


// �����ɼ���SAADCӦ�ã�---------------------------------------------------------------------------------------
/**@brief saadc�жϴ�������������������ɼ� */
static void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{}

/**@brief �����ɼ�ADC���� */
static void battery_adc_configure(void)
{
    // adc��ʼ��
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);
    // adcͨ������
    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(BATTERY_ADC_CH);
    err_code = nrf_drv_saadc_channel_init(0, &config);          //adcͨ����ʼ��
    APP_ERROR_CHECK(err_code);
}

/**@brief   ��ص�����ֵ���ɼ�����תΪ��ѹֵ���ټ���Ϊ�ٷֱ�
* @return 	�����ٷֱ�ֵ
*/
uint8_t battery_adc_sample(void)
{
    uint8_t i;
    uint16_t voltage_val;   //��ص�ѹ
    uint8_t bat_percentage; //�����ٷֱ�
    nrf_saadc_value_t saadc_val;
    uint32_t saadc_val_sum = 0;
    
    nrf_gpio_cfg_output(ADC_BAT_EN);
    nrf_gpio_pin_clear(ADC_BAT_EN); //�õͣ�ʹ��ADC��ѹ�ɼ�
    battery_adc_configure();
    for(i=0; i<50; i++)
    {
        nrf_drv_saadc_sample_convert(0,&saadc_val);
        saadc_val_sum = saadc_val_sum + saadc_val;
    }
    saadc_val = saadc_val_sum/50;
    nrfx_saadc_uninit();
    nrf_gpio_cfg_default(ADC_BAT_EN);
    
    voltage_val = (saadc_val*3600*2)/1024;
    
    //�������Թ���
    if(voltage_val>=4200)
        bat_percentage = 100;
    else if(voltage_val>=4080)
        bat_percentage = 90+(voltage_val-4080)/12;
    else if(voltage_val>=4000)
        bat_percentage = 80+(voltage_val-4000)/8;
    else if(voltage_val>=3930)
        bat_percentage = 70+(voltage_val-3930)/7;
    else if(voltage_val>=3870)
        bat_percentage = 60+(voltage_val-3870)/6;
    else if(voltage_val>=3820)
        bat_percentage = 50+(voltage_val-3820)/5;
    else if(voltage_val>=3790)
        bat_percentage = 40+(voltage_val-3790)/3;
    else if(voltage_val>=3770)
        bat_percentage = 30+(voltage_val-3770)/2;
    else if(voltage_val>=3730)
        bat_percentage = 20+(voltage_val-3730)/4;
    else if(voltage_val>=3730)
        bat_percentage = 20+(voltage_val-3730)/4;
    else if(voltage_val>=3700)
        bat_percentage = 15+(voltage_val-3700)/3;
    else if(voltage_val>=3680)
        bat_percentage = 10+(voltage_val-3680)/2;
    else if(voltage_val>=3500)
        bat_percentage = 5+(voltage_val-3500)/18;
    else
        bat_percentage = 0;    
    
    return bat_percentage;
}


// GNT LEDָʾ��-----------------------------------------------------------------------------------------------
/**@brief GNT��ǩ��LEDָʾ�¼������ȼ����� */
typedef enum
{
    LED_USB         = 0,
    LED_BLE,
    LED_GPS,
    LED_NB,
    
} gnt_led_sort;

static uint8_t gnt_led_prio_state[4];	///< LED�¼�״̬�洢

/**@brief   ��ص�����ֵ���ɼ�����תΪ��ѹֵ���ټ���Ϊ�ٷֱ�
* @param[in]  led_event   �����¼���Ӧ��LED״̬
*/
void gnt_led_indication_set(GNT_led_event_t led_event)
{
	switch (led_event)
    {
        case LED_USB_IN:
		{
			gnt_led_prio_state[LED_USB] = 1;
			if((gnt_led_prio_state[LED_NB] != 1) && (gnt_led_prio_state[LED_GPS] != 1) && (gnt_led_prio_state[LED_BLE] != 1))
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_ON);
			}
		}break;
		
        case LED_USB_OUT:
		{
			gnt_led_prio_state[LED_USB] = 0;
			if(gnt_led_prio_state[LED_NB] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_REGIST);
			}
			else if(gnt_led_prio_state[LED_GPS] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_SLOW);
			}
			else if(gnt_led_prio_state[LED_BLE] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_FAST);
			}
			else
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
			}
		}break;
		
        case LED_BLE_CONNECT:
		{
			gnt_led_prio_state[LED_BLE] = 1;
			if((gnt_led_prio_state[LED_NB] != 1) && (gnt_led_prio_state[LED_GPS] != 1))
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_FAST);
			}
		}break;
		
        case LED_BLE_DISCONNECT:
		{
			gnt_led_prio_state[LED_BLE] = 0;
			if(gnt_led_prio_state[LED_NB] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_REGIST);
			}
			else if(gnt_led_prio_state[LED_GPS] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_SLOW);
			}
			else if(gnt_led_prio_state[LED_BLE] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_FAST);
			}
			else if(gnt_led_prio_state[LED_USB] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_ON);
			}
			else
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
			}
		}break;
		
        case LED_GPS_DATA_READY:
		{
			gnt_led_prio_state[LED_GPS] = 1;
			if((gnt_led_prio_state[LED_NB] != 1))
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_SLOW);
			}
		}break;
		
        case LED_GPS_DATA_NONE:
		{
			gnt_led_prio_state[LED_GPS] = 0;
			if(gnt_led_prio_state[LED_NB] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_REGIST);
			}
			else if(gnt_led_prio_state[LED_GPS] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_SLOW);
			}
			else if(gnt_led_prio_state[LED_BLE] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_FAST);
			}
			else if(gnt_led_prio_state[LED_USB] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_ON);
			}
			else
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
			}
		}break;
		
        case LED_KEY:
		{
			bsp_indication_set(BSP_INDICATE_GNT_STATE_ON);
			vTaskDelay(100);
			bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
			vTaskDelay(100);
			bsp_indication_set(BSP_INDICATE_GNT_STATE_ON);
			vTaskDelay(100);
			bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
			
			if(gnt_led_prio_state[LED_NB] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_REGIST);
			}
			else if(gnt_led_prio_state[LED_GPS] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_SLOW);
			}
			else if(gnt_led_prio_state[LED_BLE] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_FAST);
			}
			else if(gnt_led_prio_state[LED_USB] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_ON);
			}
			else
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
			}
		}break; 
        
        case LED_NB_REGISTING:
		{
			gnt_led_prio_state[LED_NB] = 1;
			bsp_indication_set(BSP_INDICATE_GNT_STATE_REGIST);
		}break;
		
        case LED_NB_REGIST_OVER:
        {
			gnt_led_prio_state[LED_NB] = 0;
			if(gnt_led_prio_state[LED_NB] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_REGIST);
			}
			else if(gnt_led_prio_state[LED_GPS] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_SLOW);
			}
			else if(gnt_led_prio_state[LED_BLE] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_FAST);
			}
			else if(gnt_led_prio_state[LED_USB] == 1)
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_ON);
			}
			else
			{
				bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
			}
		}break;
            
        default:
		{
			
		}break;
    }
}


#if (GNT_GPS_EN == 1)
/**@brief GNTλ����Ϣ������ƫ��Ƚ�ֵ����
* @param[in]  *lock_latitude 	����ά��ָ��
* @param[in]  *lock_longitude 	��������ָ��
* @param[in]  funcType 	����ʽ
* - 1������GPS��Ϣд��falsh��������lockƫ��Ƚ�ֵ��
* - 2��������lock��comֵ
* - 3��������last��comֵ
*/
void gnt_lock_location(char *lock_latitude, char *lock_longitude, uint8_t funcType)
{
    if(funcType == 1)
    {
        // ������ǰGPS��Ϣ���ϱ��澯����
        memcpy(g_gnt_flash_info.gnt_lock_latitude, lock_latitude, 12);
        memcpy(g_gnt_flash_info.gnt_lock_longitude, lock_longitude, 12);
        my_fds_write(GNT_FILE, GNT_LOCK_LATITUDE_REC_KEY, g_gnt_flash_info.gnt_lock_latitude, 12);
        my_fds_write(GNT_FILE, GNT_LOCK_LONGTITUDE_REC_KEY, g_gnt_flash_info.gnt_lock_longitude, 12);
        
        g_gnt_flash_info.gnt_move_alarm_flag = 0;
        my_fds_write(GNT_FILE, GNT_MOVE_ALARM_FLAG_REC_KEY, &(g_gnt_flash_info.gnt_move_alarm_flag), 1);
    }
    
    
    if((funcType == 1) || (funcType == 2))
    {
        // �ʲ�λ������ʱ����������ڼ���ƫ��澯�ıȽ�ֵ
        char *pColon = strchr(g_gnt_flash_info.gnt_lock_latitude, '.'); 
        if(pColon)
        {
            if((pColon-g_gnt_flash_info.gnt_lock_latitude) == 4 )
            {
                g_gnt_info.gnt_lock_latitude_com = (*(pColon-4)-0x30)*1000000 + (*(pColon-3)-0x30)*100000 +(*(pColon-2)-0x30)*10000    \
                                                 + (*(pColon-1)-0x30)*1000 + (*(pColon+1)-0x30)*100 + (*(pColon+2)-0x30)*10 + (*(pColon+3)-0x30);
            }
            else if( (pColon-g_gnt_flash_info.gnt_lock_latitude) == 5 )
            {
                g_gnt_info.gnt_lock_latitude_com = (*(pColon-5)-0x30)*10000000 + (*(pColon-4)-0x30)*1000000 + (*(pColon-3)-0x30)*100000 +(*(pColon-2)-0x30)*10000    \
                                                 + (*(pColon-1)-0x30)*1000 + (*(pColon+1)-0x30)*100 + (*(pColon+2)-0x30)*10 + (*(pColon+3)-0x30);
            }
        }
        else
        {
            g_gnt_info.gnt_lock_latitude_com = 0;
        }
        
        pColon = strchr(g_gnt_flash_info.gnt_lock_longitude, '.');
        if(pColon)
        {
            if((pColon-g_gnt_flash_info.gnt_lock_longitude) == 4 )
            {
                g_gnt_info.gnt_lock_longitude_com = (*(pColon-4)-0x30)*1000000 + (*(pColon-3)-0x30)*100000 +(*(pColon-2)-0x30)*10000    \
                                                 + (*(pColon-1)-0x30)*1000 + (*(pColon+1)-0x30)*100 + (*(pColon+2)-0x30)*10 + (*(pColon+3)-0x30);
            }
            else if( (pColon-g_gnt_flash_info.gnt_lock_longitude) == 5 )
            {
                g_gnt_info.gnt_lock_longitude_com = (*(pColon-5)-0x30)*10000000 + (*(pColon-4)-0x30)*1000000 + (*(pColon-3)-0x30)*100000 +(*(pColon-2)-0x30)*10000    \
                                                 + (*(pColon-1)-0x30)*1000 + (*(pColon+1)-0x30)*100 + (*(pColon+2)-0x30)*10 + (*(pColon+3)-0x30);
            }
        }
        else
        {
            g_gnt_info.gnt_lock_longitude_com = 0;
        }
    }
    else if(funcType == 3)
    {
        // �ʲ�λ������ʱ����������ڼ���ƫ��澯�ıȽ�ֵ
        char *pColon = strchr(g_gnt_info.gnt_last_latitude, '.'); 
        if(pColon)
        {
            if((pColon-g_gnt_info.gnt_last_latitude) == 4 )
            {
                g_gnt_info.gnt_last_latitude_com = (*(pColon-4)-0x30)*1000000 + (*(pColon-3)-0x30)*100000 +(*(pColon-2)-0x30)*10000    \
                                                 + (*(pColon-1)-0x30)*1000 + (*(pColon+1)-0x30)*100 + (*(pColon+2)-0x30)*10 + (*(pColon+3)-0x30);
            }
            else if( (pColon-g_gnt_flash_info.gnt_lock_latitude) == 5 )
            {
                g_gnt_info.gnt_last_latitude_com = (*(pColon-5)-0x30)*10000000 + (*(pColon-4)-0x30)*1000000 + (*(pColon-3)-0x30)*100000 +(*(pColon-2)-0x30)*10000    \
                                                 + (*(pColon-1)-0x30)*1000 + (*(pColon+1)-0x30)*100 + (*(pColon+2)-0x30)*10 + (*(pColon+3)-0x30);
            }
        }
        else
        {
            g_gnt_info.gnt_last_latitude_com = 0;
        }
        
        pColon = strchr(g_gnt_flash_info.gnt_lock_longitude, '.');
        if(pColon)
        {
            if((pColon-g_gnt_flash_info.gnt_lock_longitude) == 4 )
            {
                g_gnt_info.gnt_last_longitude_com = (*(pColon-4)-0x30)*1000000 + (*(pColon-3)-0x30)*100000 +(*(pColon-2)-0x30)*10000    \
                                                 + (*(pColon-1)-0x30)*1000 + (*(pColon+1)-0x30)*100 + (*(pColon+2)-0x30)*10 + (*(pColon+3)-0x30);
            }
            else if( (pColon-g_gnt_flash_info.gnt_lock_longitude) == 5 )
            {
                g_gnt_info.gnt_last_longitude_com = (*(pColon-5)-0x30)*10000000 + (*(pColon-4)-0x30)*1000000 + (*(pColon-3)-0x30)*100000 +(*(pColon-2)-0x30)*10000    \
                                                 + (*(pColon-1)-0x30)*1000 + (*(pColon+1)-0x30)*100 + (*(pColon+2)-0x30)*10 + (*(pColon+3)-0x30);
            }
        }
        else
        {
            g_gnt_info.gnt_last_longitude_com = 0;
        }
    }
}
#endif


/**@brief �豸ʹ�ð������� */
uint32_t bsp_btn_gnt_init(void)
{
    uint32_t err_code;
    
    err_code = bsp_event_to_button_action_assign(0, BSP_BUTTON_ACTION_RELEASE, BSP_EVENT_KEY_RELEASE);
//    err_code = bsp_event_to_button_action_assign(0, BSP_BUTTON_ACTION_LONG_PUSH, BSP_EVENT_KEY_LONGPUSH);
//    err_code = bsp_event_to_button_action_assign(1, BSP_BUTTON_ACTION_PUSH, BSP_EVENT_LISINT1);
    
    err_code = bsp_event_to_button_action_assign(1, BSP_BUTTON_ACTION_PUSH, BSP_EVENT_LISINT1);   
    err_code = bsp_event_to_button_action_assign(2, BSP_BUTTON_ACTION_PUSH, BSP_EVENT_LISINT2);
    
    err_code = bsp_event_to_button_action_assign(3, BSP_BUTTON_ACTION_PUSH, BSP_EVENT_USBVIN);
    err_code = bsp_event_to_button_action_assign(3, BSP_BUTTON_ACTION_RELEASE, BSP_EVENT_USBVOUT);

    return err_code;
}


// ���Ź�
#if (GNT_WDT_EN == 1)
nrf_drv_wdt_channel_id m_channel_id;	///< ���Ź������������

/**@brief ���Ź���ʱ�¼������� */
void wdt_event_handler(void)
{
    bsp_board_leds_off();
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

/**@brief   ���Ź���ʼ��
* @param[in]  wdt_reload_value   ���Ź���������
*/
void my_wdt_init(uint32_t wdt_reload_value)
{
    uint32_t err_code = NRF_SUCCESS;
    
    //Configure WDT.
    nrf_drv_wdt_config_t config = GNT_WDT_DEAFULT_CONFIG;
    config.reload_value = wdt_reload_value;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
    
//    nrf_drv_wdt_channel_feed(m_channel_id);
}

/**@brief ���Ź�ι�� */
void my_wdt_feed(void)
{
    nrf_drv_wdt_feed();
//    nrf_drv_wdt_channel_feed(m_channel_id);
}
#endif



/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/







