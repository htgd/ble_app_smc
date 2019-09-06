/**@file  bsp_gnt_function.c
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

/* Includes ------------------------------------------------------------------*/
#include "bsp_gnt_function.h"

/* Private variables ---------------------------------------------------------*/

/* function prototypes -------------------------------------------------------*/
uint32_t bsp_btn_gnt_init(void);    ///< GNT使用按键配置
uint8_t battery_adc_sample(void);   ///< 电量采集
int32_t gnt_temp_get(void);         ///< 温度采集
void gnt_led_indication_set(GNT_led_event_t led_event); ///< GNT LED指示灯
void gnt_lock_location(char *lock_latitude, char *lock_longitude, uint8_t funcType);///< GNT位置信息锁定及偏离比较值计算


/**@brief   温度采集（内部温度传感SAADC应用）
* @return 	温度值（实际温度值/10 精度0.5）
*/
int32_t gnt_temp_get(void)
{
    int32_t temp;
    
    sd_temp_get(&temp);
    return (int32_t)((temp*10)/4);
}


// 电量采集（SAADC应用）---------------------------------------------------------------------------------------
/**@brief saadc中断处理函数，自用与非阻塞采集 */
static void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{}

/**@brief 电量采集ADC配置 */
static void battery_adc_configure(void)
{
    // adc初始化
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);
    // adc通道配置
    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(BATTERY_ADC_CH);
    err_code = nrf_drv_saadc_channel_init(0, &config);          //adc通道初始化
    APP_ERROR_CHECK(err_code);
}

/**@brief   电池电量均值法采集，并转为电压值，再计算为百分比
* @return 	电量百分比值
*/
uint8_t battery_adc_sample(void)
{
    uint8_t i;
    uint16_t voltage_val;   //电池电压
    uint8_t bat_percentage; //电量百分比
    nrf_saadc_value_t saadc_val;
    uint32_t saadc_val_sum = 0;
    
    nrf_gpio_cfg_output(ADC_BAT_EN);
    nrf_gpio_pin_clear(ADC_BAT_EN); //置低，使能ADC分压采集
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
    
    //电量粗略估算
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


// GNT LED指示灯-----------------------------------------------------------------------------------------------
/**@brief GNT标签的LED指示事件按优先级排序 */
typedef enum
{
    LED_USB         = 0,
    LED_BLE,
    LED_GPS,
    LED_NB,
    
} gnt_led_sort;

static uint8_t gnt_led_prio_state[4];	///< LED事件状态存储

/**@brief   电池电量均值法采集，并转为电压值，再计算为百分比
* @param[in]  led_event   触发事件对应的LED状态
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
/**@brief GNT位置信息锁定及偏离比较值计算
* @param[in]  *lock_latitude 	锁定维度指针
* @param[in]  *lock_longitude 	锁定经度指针
* @param[in]  funcType 	处理方式
* - 1：锁定GPS信息写入falsh，并计算lock偏离比较值；
* - 2：仅计算lock的com值
* - 3：仅计算last的com值
*/
void gnt_lock_location(char *lock_latitude, char *lock_longitude, uint8_t funcType)
{
    if(funcType == 1)
    {
        // 锁定当前GPS信息，上报告警消除
        memcpy(g_gnt_flash_info.gnt_lock_latitude, lock_latitude, 12);
        memcpy(g_gnt_flash_info.gnt_lock_longitude, lock_longitude, 12);
        my_fds_write(GNT_FILE, GNT_LOCK_LATITUDE_REC_KEY, g_gnt_flash_info.gnt_lock_latitude, 12);
        my_fds_write(GNT_FILE, GNT_LOCK_LONGTITUDE_REC_KEY, g_gnt_flash_info.gnt_lock_longitude, 12);
        
        g_gnt_flash_info.gnt_move_alarm_flag = 0;
        my_fds_write(GNT_FILE, GNT_MOVE_ALARM_FLAG_REC_KEY, &(g_gnt_flash_info.gnt_move_alarm_flag), 1);
    }
    
    
    if((funcType == 1) || (funcType == 2))
    {
        // 资产位置锁定时，计算出用于计算偏离告警的比较值
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
        // 资产位置锁定时，计算出用于计算偏离告警的比较值
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


/**@brief 设备使用按键配置 */
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


// 看门狗
#if (GNT_WDT_EN == 1)
nrf_drv_wdt_channel_id m_channel_id;	///< 看门狗操作句柄定义

/**@brief 看门狗超时事件处理函数 */
void wdt_event_handler(void)
{
    bsp_board_leds_off();
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

/**@brief   看门狗初始化
* @param[in]  wdt_reload_value   看门狗配置周期
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

/**@brief 看门狗喂狗 */
void my_wdt_feed(void)
{
    nrf_drv_wdt_feed();
//    nrf_drv_wdt_channel_feed(m_channel_id);
}
#endif



/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/







