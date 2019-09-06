/**@file    nb_task.c
* @brief   	NBģ��ME3616ͨ�Ŵ�������
* @details  ��Ҫ�������ڳ�ʼ��������մ���NB����ע�ἰ���ʼ����NBͨ�Ŵ�����������ͬ���¼�����
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-09-20
* @version 	V1.0
* @copyright	Copyright (c) 2018-2020  ���պ�ͨ�����Ƽ����޹�˾
**********************************************************************************
* @attention
* Ӳ��ƽ̨:nRF52832_QFAA \n
* SDK�汾��nRF5_SDK_15.0.0
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/09/20  <td>1.0      <td>wanghuan  <td>������ʼ�汾
* </table>
*/

/**@defgroup nb_task Wh nb_task
* @{
* @brief   NBͨ�Ŵ�������ģ��.
* @details ��Ҫ����NBģ���ô��ڳ�ʼ��������մ���NB����ע�ἰ���ʼ����NBͨ�Ŵ������� \n
*          ����������NBģ��������ͬ���¼�����. 
* @warning Ŀǰ������֡����100�ֽڣ��ɸ����������
*/


#include "gnt_includes.h"

/**
* @name �ײ㴮�ڳ�ʼ��
* @{
* @brief ���ڳ�ʼ��������մ���NB�������ջص�
*/

// ���ڷ��͡�����FIFO��С
#define UART_TX_BUF_SIZE	256      	///< ���ڷ��ͻ����С.
#define UART_RX_BUF_SIZE   	512      	///< ���ڽ��ջ����С.

static nb_receive_cb  nb_receCb;        ///< ���崮�ڽ��ջص������ڴ���DMA������ʹ��

/**@brief ���ڽ����¼������� \n
* ���ڽ������ݲ�����ģ��ָ��Э����֡��Ȼ�����me3616����ע��Ļص�������
* @param[in]  *p_event 	���������¼�����ָ��
*/
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[200];
    static uint8_t index = 0;
//    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
        {
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//            app_uart_put(data_array[index]);    //���Է���
            if(index>200)
                index = 0;
            index++;
#if defined ME3616_EN	// ME3616Ӧ����֡����
            if (data_array[index - 1] == '\n')
            {
                if( (index>2) && (data_array[index - 2] == '\r') )  //�յ�\r\n���������㴦��
                {
//                    data_array[index] = '\0';
//                    NRF_LOG_INFO("%s", data_array);
                    nb_receCb((char*)data_array, index); //�ص�ȥ��֡ͷ֡β������
                    index = 0;
//                    memset(data_array, 0, 200);
                }
                else if( (index == 2) && (data_array[0] == '\r') )  //ȥ��֡ͷ\r\n
                {
                    index = 0;
                    memset(data_array, 0, 200);
                }
            }
#endif
        }   break;

        case APP_UART_COMMUNICATION_ERROR:	//���ڽ���ͨ�ų���
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:			//���ڽ���FIFOģ�����
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

/**@brief ���ڳ�ʼ�� \n
* ����ͨ�Ų������ã������ʡ�У��λ�����š�FIFO��
*/
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,  //RX_PIN_NUMBER  23/7
        .tx_pin_no    = TX_PIN_NUMBER,  //TX_PIN_NUMBER  24/6
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,	//���ؽ�ֹ
        .use_parity   = false,							//��У��
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200		//������115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_MID,     //APP_IRQ_PRIORITY_MID/APP_IRQ_PRIORITY_LOWEST
                       err_code);
    APP_ERROR_CHECK(err_code);
}

/**@brief ���ڷ��Ͷ��ֽ� \n
* @param[in]  *buffer 	�������ݻ���ָ��
* @param[in]  num 		���������ֽ���
*/
void uart_SendN( uint8_t *buffer, uint16_t num)
{
    uint16_t i;
    for(i=0; i<num; i++)
    {  
        app_uart_put(buffer[i]);
    }
}

#if 0
/**@brief ���ڷ����ַ��� \n
* @param[in]  *str 	�����ַ���ָ��
*/
void uart_SendString(char *str)
{
    uint16_t i=0;
    
    do 
    {
        app_uart_put(*(str + i));
        i++;
    } while(*(str + i)!='\0');
}

/**@brief ����printf \n
* @param[in]  *fmt 	��ʽ��
*/
void uart_printf( char *fmt, ...)
{
    char buffer[101]; // ���Ȱ�ʵ���������
    uint16_t i = 0;

    va_list arg_ptr;
    va_start(arg_ptr, fmt); 
    vsnprintf(buffer, 100+1, fmt, arg_ptr);
    
    while ((i < 100) && buffer[i])
    {
        app_uart_put(buffer[i++]);
    }

    va_end(arg_ptr);    
}
#endif
/** @} �ײ㴮�ڳ�ʼ��*/


/**@name NBģ��Ӳ�����ʼ��ע��
* @{
* @brief ��Ҫ�д��ڡ���ʱ����������Ӧ�û����ص�
*/
// NBģ�������ö�ʱ������
APP_TIMER_DEF(m_nb_timer_id);           ///< NBģ������ʹ�ö�ʱ��
static nb_timeout_cb  nb_timeoutCb;     ///< ��ʱ�ص������������ʱ����ʱ�����е���

int NB_MsgreportCb(NB_msg_types_t, char*, uint16_t);  ///< NBģ����Ϣ�ϱ��ص�

/**@brief NB���ڳ�ʼ��
* @param[in]  cb 	�������ڽ��ջص�
* @param[in]  baud 	���ڲ�����
*/
static void NB_UART_Init(nb_receive_cb cb, uint32_t baud)
{
    baud = baud;
    uart_init();
    nb_receCb = cb;	//�������ջص�����ע��
}

/**@brief NB���ڷ��ͺ���
* @param[in]  *buf 	��������ָ��
* @param[in]  len 	�������ݳ���
*/
static void NB_UART_Send(uint8_t*buf, uint16_t len)
{
    uart_SendN(buf, len);
}

/**@brief NB���ڹر� */
static void NB_UART_Close(void)
{
    app_uart_close();
}

/**@brief NB�����ö�ʱ����ʱ������ */
static void nb_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    nb_timeoutCb();
}

/**@brief NB�����ö�ʱ����ʼ�� */
static void NB_timer_init(nb_timeout_cb cb)
{
    ret_code_t  err_code;
    nb_timeoutCb = cb;  //��ʱ�ص�
    // ����NB��ʱ��
    err_code = app_timer_create(&m_nb_timer_id, APP_TIMER_MODE_SINGLE_SHOT, nb_timeout_handler);  
    APP_ERROR_CHECK(err_code);
}

/**@brief NB�����ö�ʱ������ */
static void NB_timer_start(uint32_t ms)
{
    ret_code_t  err_code;
    err_code = app_timer_start(m_nb_timer_id, APP_TIMER_TICKS(ms), NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief NB�����ö�ʱ��ֹͣ���������� */
static void NB_timer_stop(void)
{
    ret_code_t  err_code;
    err_code = app_timer_stop(m_nb_timer_id);
    APP_ERROR_CHECK(err_code);
}

///< NB����Ӳ�����ʼ��
static HW_FxnTable hw_fxn =
{
    .baudrate = 115200,
    .com_openFxn = NB_UART_Init,
    .com_sendFxn = NB_UART_Send,
    .com_closeFxn = NB_UART_Close,

    .initTimerFxn = NB_timer_init,
    .startTimerFxn = NB_timer_start,
    .stopTimerFxn = NB_timer_stop
};

///< NB���������ʼ��
NB_Config  nb_handle =
{
    .nb_fxnTablePtr = (void*)&ME3616_FxnTable,      ///< �ù��ܺ����б���������ʵ��
    .hw_fxnTablePtr = (void*)&hw_fxn,               ///< Ӳ����غ����б���Ӧ�ò�ʵ��
    .AppReceCB = NB_MsgreportCb						///< NB������Ϣ�ϱ�
};
/** @} NBģ��Ӳ�����ʼ��ע��*/


/**@name NB������ض���
* @{
* @brief ��Ҫ�д��ڡ���ʱ����������Ӧ�û����ص�
*/
// NBͨ��������
TaskHandle_t    m_nb_thread;        		///< NBͨ��������
void nb_thread(void *pvParameters); 		///< NBͨ��������
// �����ϱ����澯������
TaskHandle_t    m_nb_report_thread;         ///< NB�����ϱ�������
void nb_report_thread(void *pvParameters);  ///< NB�����ϱ�������
// �����ϱ���ʱ������
APP_TIMER_DEF(m_nb_report_timer_id);        ///< NB�����ϱ���ʱ��ʱ��
static void nb_report_timer_handler(void * p_context);	///< NB�����ϱ���ʱ�ص�����
/** @} NB������ض���*/


/**@brief ��������ϱ����� \n
* ��ϵͳ���Ź�ʹ��ʱ���ú���������ã���Ϊ�����������ڵ�ͬʱ���������ÿ��Ź����ڣ������Ź����������豸��λ����.
* @param[in]  period 	��������ֵ
*/
void gnt_change_report_period(uint32_t period)
{
    app_timer_stop(m_nb_report_timer_id);
    app_timer_start(m_nb_report_timer_id, g_gnt_flash_info.gnt_report_period, NULL);   //GPS��λ��ʱ��ʱ
}

/**@brief NBͨ������
* @details �豸�������������á�NBģ���ʼ���������ϱ����񴴽�����NB�����¼�����
*/
#if 1
void nb_thread(void *pvParameters)
{
    uint32_t err_code = NRF_SUCCESS;
    
    NRF_LOG_INFO("nb_thread start.");
    
    err_code = app_timer_create(&m_nb_report_timer_id, APP_TIMER_MODE_REPEATED, nb_report_timer_handler);
    APP_ERROR_CHECK(err_code);
    
    g_gnt_info.gnt_battery_level = battery_adc_sample();    //���������ɼ�
    
    if(g_gnt_flash_info.gnt_enable_flag == GNT_DISABLE)     //��ǩû��ʹ��,���ɼ�ģ�������Ϣ
    {
        nb_handle.nb_fxnTablePtr->nbModuleInit(&nb_handle, NULL, NB_REGIST_DISABLE, NULL);  //��ȡIMSI��IMEI
        
        nb_handle.nb_fxnTablePtr->nbComClose(&nb_handle);   //�رմ���
        nb_handle.nb_fxnTablePtr->nbPowerOff(&nb_handle);   //ģ��ػ�
        
        if(strstr(g_gnt_flash_info.ble_name, "HT_tag") == NULL)   //BLE����û������
        {
            snprintf(g_gnt_flash_info.ble_name, 13, "HT_tag_%s", &g_me3616_info.me3616_IMEI[10]);
            my_fds_write(GNT_FILE, GNT_BLE_NAME_REC_KEY, g_gnt_flash_info.ble_name, 12);  //BLE����д��Flash
            nRF52_FxnTable.bleSetName(g_gnt_flash_info.ble_name, 12);
//			NRF_LOG_INFO("BLE get name:%s\n", g_gnt_flash_info.ble_name);
//            NVIC_SystemReset(); //��λ���㲥������
        }
        vTaskSuspend(NULL);
    }
    else if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)  //��ǩʹ��
    {
        if(g_gnt_info.gnt_battery_level < GNT_ENABLE_BATTERY_THRESHOLD)
        {
            g_gnt_flash_info.gnt_enable_flag = GNT_DISABLE;
            my_fds_write(GNT_FILE, GNT_ENABLE_FLAG_REC_KEY, &(g_gnt_flash_info.gnt_enable_flag), 1);
            bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
            NVIC_SystemReset();
        }
        else
        {
            gnt_led_indication_set(LED_NB_REGISTING);       //��ƽ̨ע��״ָ̬ʾ
            
            if(strstr(g_gnt_flash_info.ble_name, "ht_TAG") == NULL)   //BLE����û������
            {
                nb_handle.nb_fxnTablePtr->nbModuleInit(&nb_handle, NULL, NB_REGIST_DISABLE, NULL);      //��ȡIMSI��IMEI
                
                snprintf(g_gnt_flash_info.ble_name, 13, "ht_TAG_%s", &g_me3616_info.me3616_IMEI[10]);
                my_fds_write(GNT_FILE, GNT_BLE_NAME_REC_KEY, g_gnt_flash_info.ble_name, 12);            //BLE����д��Flash
    //            NVIC_SystemReset(); //��λ���㲥������
                nRF52_FxnTable.bleSetName(g_gnt_flash_info.ble_name, 12);
                nb_handle.nb_fxnTablePtr->nbSet_IotRegist(&nb_handle, 3600*24, 3);
            }
            else
                nb_handle.nb_fxnTablePtr->nbModuleInit(&nb_handle, 3600*24, NB_REGIST_ENABLE, 3);   //ע��ƽ̨������ʱ��6Сʱ
        }
    }
    else
    {
        vTaskSuspend(NULL);
    }

    #if (GNT_NB_TEST_EN == 1)	// NBͨ�ż��źŲ���
    vTaskSuspend(NULL);
    #endif
    
    if (pdPASS != xTaskCreate(nb_report_thread, "NBR", 512, NULL, NB_REPORT_THREAD_PRIO, &m_nb_report_thread))
    {
        NRF_LOG_INFO("NRF_ERROR_NO_MEM.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
	
    
#if (GNT_GPS_EN == 1)   //��λ��������ƫ�Ƹ澯���ܣ�
    if(g_gnt_flash_info.gnt_lock_latitude[0] != 0)  //�ʲ�λ��������
    {
        gnt_lock_location(NULL, NULL, 2);   // ������lockƫ��Ƚ�ֵ
    }
#endif
                    
    
//    nRF52_FxnTable.bleAdvStart(); 
//    vTaskSuspend(NULL);
    
    while(1)
    {
        nb_handle.nb_fxnTablePtr->eventHandle(&nb_handle);
    }
    
}
#endif

#ifdef GNT_REPORT_CONTINUOUS_FAIL_TIMES_TO_RESET
static uint8_t gnt_report_continuous_fail_times = 0;	///< �豸�ϱ�����ʧ�ܴ���
#endif

/**@brief �����ϱ�����
* @details Ӧ�õ���NBģ���������¼�������Ҫ���������ϱ����������˶����GPS�ϱ���
*/
void nb_report_thread(void *pvParameters)
{
    EventBits_t uxBits;
    
    xEventGroupSetBits(GntEventGroup, NB_PRERIOD_REPORT_EVENTBIT);      //�ϵ��ϱ�һ��
    app_timer_start(m_nb_report_timer_id, g_gnt_flash_info.gnt_report_period, NULL);     //����3����һ���ϱ�����ʱ����λΪms��
    
    // ���Ź�ʹ�ܣ���Ҫʹ�ÿ��Ź�����������ϱ��������
#if (GNT_WDT_EN == 1)
    my_wdt_init(g_gnt_flash_info.gnt_report_period + 3600000);  //���ò��������Ź�����λms��
#endif
    
    // ����GPS��ʼ������ʽ
#if (GNT_GPS_EN == 1)
    #if (GNT_GPS_DEFAULT_REPORT_EN == 1)
    g_gnt_info.gnt_gps_realtime_report_flag = 1;
    #endif

    if(NB_ACK_OK != nb_handle.nb_fxnTablePtr->gpsInit(&nb_handle))
        NVIC_SystemReset();
#endif
    
#if (GNT_LIS3DH_EN == 1)
    xEventGroupClearBits(GntEventGroup, LISINT1_EVENTBIT|LISINT2_EVENTBIT);
    taskENTER_CRITICAL();   //�Դ������ĳ�ʼ�������ٽ籣��
    lis3dh_init();
    taskEXIT_CRITICAL();
    NRF_LOG_INFO("lis3dh init over");
    // ���Լ��ٶ�ֵ�ϱ�
#endif
    
    gnt_led_indication_set(LED_NB_REGIST_OVER);
    
    while(1)
    {
        uxBits = xEventGroupWaitBits(GntEventGroup, 
                                    NB_PRERIOD_REPORT_EVENTBIT|KEY_EVENTBIT|KEY_LONGPUSH_EVENTBIT|KEY_RELEASE_EVENTBIT, 
                                    pdTRUE, pdFALSE, portMAX_DELAY);
        
        if((uxBits & NB_PRERIOD_REPORT_EVENTBIT) == NB_PRERIOD_REPORT_EVENTBIT)   //NB�����ϱ��¼�
        {
            #if (GNT_WDT_EN == 1)
            my_wdt_feed();
            #endif
            
            char temp[50];
            memset(temp, 0, 50);
            
            g_gnt_info.gnt_battery_level = battery_adc_sample();
			
			if(g_gnt_info.gnt_battery_level > GNT_REPORT_DISABLE_BATTERY_THRESHOLD) //������ʱȡ���ϱ�
			{
				if(nrf_gpio_pin_read(BUTTON_1) == 1)
				{
					g_gnt_info.smc_open_state = 0;
				}
				else if(nrf_gpio_pin_read(BUTTON_1) == 0)
				{
					g_gnt_info.smc_open_state = 1;
				}
				nb_handle.nb_fxnTablePtr->getSign(&nb_handle);	//��ȡ�ź�ǿ��
				snprintf(temp, 50, "@03+%02d+%03d+%03d+%1d^", g_gnt_info.gnt_battery_level, gnt_temp_get(), g_me3616_info.me3616_rssi, g_gnt_info.smc_open_state); //�ϱ��������¶ȡ�����״̬
				Ble_log_send((uint8_t*)temp, strlen(temp)); //�������

                // �ϱ�+�����ͷ�RCC+����ע��
                nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_TRY);
				
				// �ϱ�ʧ�ܴ����ﵽһ��ֵʱ���������λ
				#ifdef GNT_REPORT_CONTINUOUS_FAIL_TIMES_TO_RESET
				if(g_me3616_info.iot_regist_status == 0)
					gnt_report_continuous_fail_times++;
				else
					gnt_report_continuous_fail_times = 0;
				if(gnt_report_continuous_fail_times == GNT_REPORT_CONTINUOUS_FAIL_TIMES_TO_RESET)
					NVIC_SystemReset();
				#endif					
            }
        }

        if((uxBits & KEY_EVENTBIT) == KEY_EVENTBIT) //ģ�⾮��״����
        {
            char temp[20];
            memset(temp, 0, 20);
            snprintf(temp, 20, "@02+1^");
            g_gnt_info.gnt_battery_level = battery_adc_sample();
            
            Ble_log_send((uint8_t*)"@02+1^", 6);
            
            if(g_gnt_info.gnt_battery_level > GNT_REPORT_DISABLE_BATTERY_THRESHOLD) //������ʱȡ���ϱ�
            {
                nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_TRY);
				Ble_log_send((uint8_t*)"notify", 6);
            }
//            vTaskDelay(10000);
        }
        if((uxBits & KEY_RELEASE_EVENTBIT) == KEY_RELEASE_EVENTBIT) //ģ�⾮��״�ر�
        {
			#if SMC_CLOSE_REPORT_EN
            char temp[20];
            memset(temp, 0, 20);
            snprintf(temp, 20, "@02+0^");
            g_gnt_info.gnt_battery_level = battery_adc_sample();
            
            Ble_log_send((uint8_t*)"@02+0^", 6);
            
            if(g_gnt_info.gnt_battery_level > GNT_REPORT_DISABLE_BATTERY_THRESHOLD) //������ʱȡ���ϱ�
            {
                nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_TRY);
            }
//            vTaskDelay(10000);
			#endif
        }
        
        if((uxBits & KEY_LONGPUSH_EVENTBIT) == KEY_LONGPUSH_EVENTBIT) //������������
        {
//            if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)    //NB����ʹ��
//            {
//                nb_handle.nb_fxnTablePtr->nbPowerOff(&nb_handle);   //NBģ��ػ�
//            }
//            lis3dh_power_off();
//            bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
//            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
        }
        
        if((uxBits & GNT_CLEAR_MOVEFLAG_EVENTBIT) == GNT_CLEAR_MOVEFLAG_EVENTBIT)   //�����ǩ�ƶ���־�¼�
        {
            char temp[20];
            memset(temp, 0, 20);
            snprintf(temp, 20, "@02+0^");
            if(g_gnt_info.gnt_battery_level > GNT_REPORT_DISABLE_BATTERY_THRESHOLD) //������ʱȡ���ϱ�
            {
                // �ϱ�+�����ͷ�RCC+����ע��
                nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_TRY); 
            }
        }
    }
}

/**@brief �����ϱ��ö�ʱ����ʱ������ */
static void nb_report_timer_handler(void * p_context)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(pdPASS == xEventGroupSetBitsFromISR(GntEventGroup, NB_PRERIOD_REPORT_EVENTBIT, &xHigherPriorityTaskWoken))
    {
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }  
}

/**@brief ME3616����������Ϣ�ϱ�->Ӧ�ûص�
* @param[in]  types 	��Ϣ����
* @param[in]  *msg 		��Ϣ����ָ��
* @param[in]  len 		��Ϣ���ݳ���
*/
int  NB_MsgreportCb(NB_msg_types_t types, char* msg, uint16_t len)
{
    switch (types)
    {
#if 0
        case MSG_GPS_GNSS_DATA_RECE:    //ģ��GPS���ݴ���
        {
            // $GPGLL,4004.74005,N,11614.19613,E,060845.00,A,A*5B<CR><LF>
//            NRF_LOG_INFO("%s  %d", msg, len);
            char* pColon = strchr(msg,'*');
            if(pColon)
            {
                pColon = pColon-3;
                if(strstr(pColon,"A,A"))    //������Ч
                {
                    char temp[100];
                    char longitude_data[20];    //����
                    char latitude_data[20];     //γ��
                    uint8_t longitude_len, latitude_len;
                    
                    strcpy(temp, msg);
                    strcpy(latitude_data, strtok(temp,","));        //��ȡ γ������
                    latitude_len = strlen(latitude_data);
                    if((latitude_len < 10)||(latitude_len > 11))    //���ݻ�ȡ����(���Ȳ���)��ȡ���˴��ϱ�
                        break;
                    else if(latitude_len == 10) //10λβ������
                        latitude_data[10] = '0';
                    
                    if(NULL == strchr(latitude_data,'.'))   //�������ݼ��
                        break;
                    
                    strcpy(&latitude_data[11], strtok(NULL,","));   //��ȡγ�ȷ���
                    latitude_len = strlen(latitude_data);
                    if(latitude_len != 12)      //11�ֽ�γ������+1�ֽڷ���
                        break;
//                    NRF_LOG_INFO("%s %d %s", latitude_data, latitude_len, &latitude_data[12]);//��ӡ���                                  
                    
                    
                    strcpy(longitude_data, strtok(NULL,","));       //��ȡ��������
                    longitude_len = strlen(longitude_data);
                    if((longitude_len < 10)||(longitude_len > 11))    //���ݻ�ȡ����(���Ȳ���)��ȡ���˴��ϱ�
                        break;
                    else if(longitude_len == 10) //10λβ������
                        longitude_data[10] = '0';
                    
                    if(NULL == strchr(longitude_data,'.'))   //�������ݼ��
                        break;
                    
                    strcpy(&longitude_data[11], strtok(NULL,","));  //��ȡ���ȷ���
                    longitude_len = strlen(longitude_data);
                    if(longitude_len != 12)      //11�ֽ�γ������+1�ֽڷ���
                        break;
//                    NRF_LOG_INFO("%s %d %s", longitude_data, longitude_len, &longitude_data[12]);
                    
                    
                    Ble_log_send((uint8_t*)latitude_data, 12);      //�������
                    Ble_log_send((uint8_t*)longitude_data, 12);     //�������
                    
                    // ��λ�ɹ�LEDָʾ
//                    NRF_LOG_INFO("get gps: %12.12s  %12.12s", longitude_data, latitude_data);
                    gnt_led_indication_set(LED_GPS_DATA_READY);
                    g_gnt_info.gnt_gps_data_ready_flag = 1;
                    
                    // ��¼��ǰGPS��Ϣ�����������/���µĵ�GPS����
                    memcpy(g_gnt_info.gnt_last_latitude, latitude_data, 12);
                    memcpy(g_gnt_info.gnt_last_longitude, longitude_data, 12);
                    
                    // ʹ���ƶ�ʱGPS��Ϣʵʱ�ϱ�
                    if(g_gnt_info.gnt_gps_realtime_report_flag == 1)
                    {
                        memset(temp, 0, 100);
                        snprintf(temp, 100, "@01+%11.11s+%1.1s+%11.11s+%1.1s^",     \
                                g_gnt_info.gnt_last_latitude, &g_gnt_info.gnt_last_latitude[11], g_gnt_info.gnt_last_longitude, &g_gnt_info.gnt_last_longitude[11]);
                         
                        nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_DISABLE, NB_REPFAIL_REG_NO_TRY);
                    }
                }
                else
                {
                    gnt_led_indication_set(LED_GPS_DATA_NONE);  //û���ź�ʱ��LEDָʾ
                    g_gnt_info.gnt_gps_data_ready_flag = 0;
                    
                    // ����GPS������
//                    nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)"@nogps^", 7, NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_DELAY_TRY);
//                    vTaskDelay(3000);
                    
                    Ble_log_send((uint8_t*)"nogps", 5);
                }
            }
//            g_gnt_info.gnt_battery_level = battery_adc_sample();
            vTaskDelay(5000);
        }break;
        
        case MSG_GPS_POSITION_TIMEOUT:
        {
            char temp[100];
            snprintf(temp, 100, "@01+%11.11s+%1.1s+%11.11s+%1.1s+0^",     \
                    g_gnt_info.gnt_last_latitude, &g_gnt_info.gnt_last_latitude[11], g_gnt_info.gnt_last_longitude, &g_gnt_info.gnt_last_longitude[11]);
            
            // ��ʱ��ʱ�ϱ���������
            gnt_led_indication_set(LED_GPS_DATA_NONE);  //��λ��ʱ���ʲ���ֹʱ��LEDָʾ
            g_gnt_info.gnt_gps_data_ready_flag = 0;     //GPS���ݱ�־����
            
            nb_handle.nb_fxnTablePtr->gpsStop(&nb_handle); //��ǩ��ֹ��GPS��λ��ʱ��ֹͣ��λ���ϴ�����¼������  
            nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_TRY); 
            
            
            #if 1   //��ֹʱ���ϱ�����������Ϣ
            gnt_lock_location(NULL, NULL, 3);   //��������¼��GPS��Ϣƫ�����ֵlast_com
            // ��ǩ��Ǩ����50m��û���ϱ����澯 ��λ��������ƫ�Ƹ澯���ܣ�
            if( ((abs(g_gnt_info.gnt_last_longitude_com - g_gnt_info.gnt_lock_longitude_com)>6) || (abs(g_gnt_info.gnt_last_latitude_com - g_gnt_info.gnt_lock_latitude_com)>6)) && (g_gnt_flash_info.gnt_move_alarm_flag == 0) )
            {
                g_gnt_flash_info.gnt_move_alarm_flag = 1;   
                my_fds_write(GNT_FILE, GNT_MOVE_ALARM_FLAG_REC_KEY, &(g_gnt_flash_info.gnt_move_alarm_flag), 1);
                snprintf(temp, 100, "@02+1^");
                nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_DELAY_TRY);
            }
            else if( ((abs(g_gnt_info.gnt_last_longitude_com - g_gnt_info.gnt_lock_longitude_com)<6) && (abs(g_gnt_info.gnt_last_latitude_com - g_gnt_info.gnt_lock_latitude_com)<6)) && (g_gnt_flash_info.gnt_move_alarm_flag == 1) )
            {
                g_gnt_flash_info.gnt_move_alarm_flag = 0;
                my_fds_write(GNT_FILE, GNT_MOVE_ALARM_FLAG_REC_KEY, &(g_gnt_flash_info.gnt_move_alarm_flag), 1);
                snprintf(temp, 100, "@02+0^");
                nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_DELAY_TRY); //��Ƶ���ϱ�
            }
            #endif
        }break;
        
        case MSG_GPS_NMEADATA_TIMEOUT:
        {
            if(g_me3616_info.gnss_run_state == 1)   //gps��λû�г�ʱʱִ��
            {
                Ble_log_send((uint8_t*)"gps_data_lost", 13);
                NRF_LOG_INFO("%s", "gps_data_lost");
                
                gnt_led_indication_set(LED_GPS_DATA_NONE);      //��λ��ʱ���ʲ���ֹʱ��LEDָʾ
                g_gnt_info.gnt_gps_data_ready_flag = 0;         //GPS���ݱ�־����
                nb_handle.nb_fxnTablePtr->gpsStop(&nb_handle);  //��ǩ��ֹ��GPS��λ��ʱ��ֹͣ��λ���ϴ�������
            }
        }break;
 #endif
        
        case MSG_OCEANCONNECT_DATA_RECE:
        {
            // +M2MCLIRECV:02000100004030415E��@0A^��
            char rply_head[10]; //���ڻظ�֡ǰ����messageid+mid+errcode
            char cmd_data[100]; //���ڻظ���ATָ��
            char dl_data[50];   //�·������� @0A^
            
            if(len < 10)
                break;
            
            memcpy(rply_head, msg, 6);  //����mid
            rply_head[1] = '3';         //Ӧ��messageid
            rply_head[6] = '0'; rply_head[7] = '0'; //errcode
            
            HexStrToStr(dl_data, &msg[10], (len-10)/2); //��ȡ�·������ַ���
            
//            if(strstr((char*)dl_data,"@0A")) //�����ǩ�ƶ���־��������ǰλ��
//            {
//                char temp[20];
//                memset(temp, 0, 20);
//                StrToHexStr(temp, "@0A+0^", 6); //0300010000064030412B305E
//                snprintf(cmd_data, 100, "AT+M2MCLISEND=%8.8s%04X%s", rply_head, 6, temp);
//                nb_handle.nb_fxnTablePtr->oceanconnectRply(&nb_handle, cmd_data);
//                // �ظ�ʧ����д�пͻ��˿���
//                
//                // ������ǰGPS��Ϣ���ϱ��澯����
//                #if (GNT_GPS_EN == 1)
//                gnt_lock_location(g_gnt_info.gnt_last_latitude, g_gnt_info.gnt_last_longitude, 1);
//                #endif
//                xEventGroupSetBits(GntEventGroup, GNT_CLEAR_MOVEFLAG_EVENTBIT); //�����ǩ�ƶ���־�¼�
//                break;
//            }
            if(strstr((char*)dl_data,"@0B")) //�����������ڣ���λСʱ��@0B+12^
            {
                uint8_t set_period;
                char temp[20];
                memset(temp, 0, 20);
                set_period = (dl_data[4]-0x30)*10 + (dl_data[5]-0x30);
                if((set_period < 1) || (set_period > 30))   // �������ڲ���������
                {
                    StrToHexStr(temp, "@0B+1^", 6);
                    snprintf(cmd_data, 100, "AT+M2MCLISEND=%8.8s%04X%s", rply_head, 6, temp);
                    nb_handle.nb_fxnTablePtr->oceanconnectRply(&nb_handle, cmd_data);
                }
                else
                {
                    StrToHexStr(temp, "@0B+0^", 6);
                    snprintf(cmd_data, 100, "AT+M2MCLISEND=%8.8s%04X%s", rply_head, 6, temp);
                    nb_handle.nb_fxnTablePtr->oceanconnectRply(&nb_handle, cmd_data);
                    
                    g_gnt_flash_info.gnt_report_period = set_period * 3600000;
                    my_fds_write(GNT_FILE, GNT_REPORT_PERIOD_REC_KEY, &(g_gnt_flash_info.gnt_report_period), 4);
                    
                    #if (GNT_WDT_EN == 1)   // ���Ź�ʹ����Ҫ��������
                    vTaskDelay(2000);
                    bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
                    NVIC_SystemReset();
                    #else
                    gnt_change_report_period(g_gnt_flash_info.gnt_report_period);
                    #endif
                }
                break;
            }
        }break;
        
        case MSG_ONENET_READ_REQ:
        {
            
        }break;
        
        case MSG_ONENET_WRITE_REQ:
        {
            if(strstr((char*)msg, "0A")) //�����ǩ�ƶ���־��������ǰλ��
            {
                char temp[20];  //���ڻظ���ATָ��
                memset(temp, 0, 20);
                snprintf(temp, 20, "@0A+0^"); //�ظ������ǩ�ƶ���־

                nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_TRY); 
            }
        }break;
        
        default:
            
        break;
    }
    
    
    return 0;
}


#if 0
void nb_thread(void *pvParameters)
{
    EventBits_t uxBits;
    AxesRaw_t data;
    
    vTaskDelay(3000);
    NRF_LOG_INFO("start lis3dh");
    lis3dh_init();
    NRF_LOG_INFO("start lis3dh over");
    while(1)
    {
        uxBits = xEventGroupWaitBits(GntEventGroup, KEY_EVENTBIT|LISINT2_EVENTBIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if((uxBits & LISINT2_EVENTBIT) == LISINT2_EVENTBIT)
        {
            vTaskDelay(2000);
            LIS3DH_ResetInt1Latch();
        }
        if((uxBits & KEY_EVENTBIT) == KEY_EVENTBIT) //��������
        {
            LIS3DH_ResetInt1Latch();
        }

//        LIS3DH_GetAccAxesRaw(&data);
//        NRF_LOG_INFO("X=%6d Y=%6d Z=%6d", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
//        vTaskDelay(1000);
    }
}
#endif

/** @} nb_task*/

/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/



