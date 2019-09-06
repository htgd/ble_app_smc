/**@file    nb_task.c
* @brief   	NB模组ME3616通信处理任务
* @details  主要包含串口初始化及其接收处理、NB驱动注册及其初始化、NB通信处理任务及其他同步事件处理
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-09-20
* @version 	V1.0
* @copyright	Copyright (c) 2018-2020  江苏亨通光网科技有限公司
**********************************************************************************
* @attention
* 硬件平台:nRF52832_QFAA \n
* SDK版本：nRF5_SDK_15.0.0
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/09/20  <td>1.0      <td>wanghuan  <td>创建初始版本
* </table>
*/

/**@defgroup nb_task Wh nb_task
* @{
* @brief   NB通信处理任务模块.
* @details 主要包含NB模组用串口初始化及其接收处理、NB驱动注册及其初始化、NB通信处理任务 \n
*          及其他调用NB模组驱动的同步事件处理. 
* @warning 目前最大接收帧长度100字节，可根据情况调节
*/


#include "gnt_includes.h"

/**
* @name 底层串口初始化
* @{
* @brief 串口初始化及其接收处理，NB驱动接收回调
*/

// 串口发送、接收FIFO大小
#define UART_TX_BUF_SIZE	256      	///< 串口发送缓冲大小.
#define UART_RX_BUF_SIZE   	512      	///< 串口接收缓冲大小.

static nb_receive_cb  nb_receCb;        ///< 定义串口接收回调，放在串口DMA接收中使用

/**@brief 串口接收事件处理函数 \n
* 串口接收数据并根据模组指令协议组帧，然后调用me3616驱动注册的回调处理函数
* @param[in]  *p_event 	串口外设事件类型指针
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
//            app_uart_put(data_array[index]);    //测试返回
            if(index>200)
                index = 0;
            index++;
#if defined ME3616_EN	// ME3616应答组帧处理
            if (data_array[index - 1] == '\n')
            {
                if( (index>2) && (data_array[index - 2] == '\r') )  //收到\r\n交由驱动层处理
                {
//                    data_array[index] = '\0';
//                    NRF_LOG_INFO("%s", data_array);
                    nb_receCb((char*)data_array, index); //回调去掉帧头帧尾的内容
                    index = 0;
//                    memset(data_array, 0, 200);
                }
                else if( (index == 2) && (data_array[0] == '\r') )  //去掉帧头\r\n
                {
                    index = 0;
                    memset(data_array, 0, 200);
                }
            }
#endif
        }   break;

        case APP_UART_COMMUNICATION_ERROR:	//串口接收通信出错
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:			//串口接收FIFO模块出错
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

/**@brief 串口初始化 \n
* 串口通信参数设置：波特率、校验位、引脚、FIFO等
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
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,	//流控禁止
        .use_parity   = false,							//无校验
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200		//波特率115200
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

/**@brief 串口发送多字节 \n
* @param[in]  *buffer 	发送数据缓冲指针
* @param[in]  num 		发送数据字节数
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
/**@brief 串口发送字符串 \n
* @param[in]  *str 	发送字符串指针
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

/**@brief 串口printf \n
* @param[in]  *fmt 	格式化
*/
void uart_printf( char *fmt, ...)
{
    char buffer[101]; // 长度按实际需求更改
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
/** @} 底层串口初始化*/


/**@name NB模组硬件层初始化注册
* @{
* @brief 主要有串口、定时器及驱动与应用互动回调
*/
// NB模组驱动用定时器定义
APP_TIMER_DEF(m_nb_timer_id);           ///< NB模组驱动使用定时器
static nb_timeout_cb  nb_timeoutCb;     ///< 超时回调，放在软件定时器超时函数中调用

int NB_MsgreportCb(NB_msg_types_t, char*, uint16_t);  ///< NB模块消息上报回调

/**@brief NB串口初始化
* @param[in]  cb 	驱动串口接收回调
* @param[in]  baud 	串口波特率
*/
static void NB_UART_Init(nb_receive_cb cb, uint32_t baud)
{
    baud = baud;
    uart_init();
    nb_receCb = cb;	//驱动接收回调函数注册
}

/**@brief NB串口发送函数
* @param[in]  *buf 	发送数据指针
* @param[in]  len 	发送数据长度
*/
static void NB_UART_Send(uint8_t*buf, uint16_t len)
{
    uart_SendN(buf, len);
}

/**@brief NB串口关闭 */
static void NB_UART_Close(void)
{
    app_uart_close();
}

/**@brief NB驱动用定时器超时处理函数 */
static void nb_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    nb_timeoutCb();
}

/**@brief NB驱动用定时器初始化 */
static void NB_timer_init(nb_timeout_cb cb)
{
    ret_code_t  err_code;
    nb_timeoutCb = cb;  //超时回调
    // 创建NB定时器
    err_code = app_timer_create(&m_nb_timer_id, APP_TIMER_MODE_SINGLE_SHOT, nb_timeout_handler);  
    APP_ERROR_CHECK(err_code);
}

/**@brief NB驱动用定时器启动 */
static void NB_timer_start(uint32_t ms)
{
    ret_code_t  err_code;
    err_code = app_timer_start(m_nb_timer_id, APP_TIMER_TICKS(ms), NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief NB驱动用定时器停止，计数清零 */
static void NB_timer_stop(void)
{
    ret_code_t  err_code;
    err_code = app_timer_stop(m_nb_timer_id);
    APP_ERROR_CHECK(err_code);
}

///< NB驱动硬件层初始化
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

///< NB驱动句柄初始化
NB_Config  nb_handle =
{
    .nb_fxnTablePtr = (void*)&ME3616_FxnTable,      ///< 该功能函数列表在驱动层实现
    .hw_fxnTablePtr = (void*)&hw_fxn,               ///< 硬件相关函数列表在应用层实现
    .AppReceCB = NB_MsgreportCb						///< NB驱动消息上报
};
/** @} NB模组硬件层初始化注册*/


/**@name NB任务相关定义
* @{
* @brief 主要有串口、定时器及驱动与应用互动回调
*/
// NB通信任务定义
TaskHandle_t    m_nb_thread;        		///< NB通信任务句柄
void nb_thread(void *pvParameters); 		///< NB通信任务函数
// 周期上报及告警任务定义
TaskHandle_t    m_nb_report_thread;         ///< NB周期上报任务句柄
void nb_report_thread(void *pvParameters);  ///< NB周期上报任务函数
// 周期上报定时器定义
APP_TIMER_DEF(m_nb_report_timer_id);        ///< NB周期上报计时定时器
static void nb_report_timer_handler(void * p_context);	///< NB周期上报超时回调函数
/** @} NB任务相关定义*/


/**@brief 变更心跳上报周期 \n
* 当系统看门狗使能时，该函数不会调用，因为调整心跳周期的同时需重新设置看门狗周期，而看门狗的设置需设备复位设置.
* @param[in]  period 	设置周期值
*/
void gnt_change_report_period(uint32_t period)
{
    app_timer_stop(m_nb_report_timer_id);
    app_timer_start(m_nb_report_timer_id, g_gnt_flash_info.gnt_report_period, NULL);   //GPS定位超时计时
}

/**@brief NB通信任务
* @details 设备开机的名称设置、NB模组初始化、周期上报任务创建、及NB驱动事件处理
*/
#if 1
void nb_thread(void *pvParameters)
{
    uint32_t err_code = NRF_SUCCESS;
    
    NRF_LOG_INFO("nb_thread start.");
    
    err_code = app_timer_create(&m_nb_report_timer_id, APP_TIMER_MODE_REPEATED, nb_report_timer_handler);
    APP_ERROR_CHECK(err_code);
    
    g_gnt_info.gnt_battery_level = battery_adc_sample();    //开机电量采集
    
    if(g_gnt_flash_info.gnt_enable_flag == GNT_DISABLE)     //标签没有使能,仅采集模组基础信息
    {
        nb_handle.nb_fxnTablePtr->nbModuleInit(&nb_handle, NULL, NB_REGIST_DISABLE, NULL);  //获取IMSI和IMEI
        
        nb_handle.nb_fxnTablePtr->nbComClose(&nb_handle);   //关闭串口
        nb_handle.nb_fxnTablePtr->nbPowerOff(&nb_handle);   //模组关机
        
        if(strstr(g_gnt_flash_info.ble_name, "HT_tag") == NULL)   //BLE名称没有设置
        {
            snprintf(g_gnt_flash_info.ble_name, 13, "HT_tag_%s", &g_me3616_info.me3616_IMEI[10]);
            my_fds_write(GNT_FILE, GNT_BLE_NAME_REC_KEY, g_gnt_flash_info.ble_name, 12);  //BLE名称写入Flash
            nRF52_FxnTable.bleSetName(g_gnt_flash_info.ble_name, 12);
//			NRF_LOG_INFO("BLE get name:%s\n", g_gnt_flash_info.ble_name);
//            NVIC_SystemReset(); //复位，广播新名称
        }
        vTaskSuspend(NULL);
    }
    else if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)  //标签使能
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
            gnt_led_indication_set(LED_NB_REGISTING);       //云平台注册状态指示
            
            if(strstr(g_gnt_flash_info.ble_name, "ht_TAG") == NULL)   //BLE名称没有设置
            {
                nb_handle.nb_fxnTablePtr->nbModuleInit(&nb_handle, NULL, NB_REGIST_DISABLE, NULL);      //获取IMSI和IMEI
                
                snprintf(g_gnt_flash_info.ble_name, 13, "ht_TAG_%s", &g_me3616_info.me3616_IMEI[10]);
                my_fds_write(GNT_FILE, GNT_BLE_NAME_REC_KEY, g_gnt_flash_info.ble_name, 12);            //BLE名称写入Flash
    //            NVIC_SystemReset(); //复位，广播新名称
                nRF52_FxnTable.bleSetName(g_gnt_flash_info.ble_name, 12);
                nb_handle.nb_fxnTablePtr->nbSet_IotRegist(&nb_handle, 3600*24, 3);
            }
            else
                nb_handle.nb_fxnTablePtr->nbModuleInit(&nb_handle, 3600*24, NB_REGIST_ENABLE, 3);   //注册平台，生存时间6小时
        }
    }
    else
    {
        vTaskSuspend(NULL);
    }

    #if (GNT_NB_TEST_EN == 1)	// NB通信及信号测试
    vTaskSuspend(NULL);
    #endif
    
    if (pdPASS != xTaskCreate(nb_report_thread, "NBR", 512, NULL, NB_REPORT_THREAD_PRIO, &m_nb_report_thread))
    {
        NRF_LOG_INFO("NRF_ERROR_NO_MEM.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
	
    
#if (GNT_GPS_EN == 1)   //（位置锁定及偏移告警功能）
    if(g_gnt_flash_info.gnt_lock_latitude[0] != 0)  //资产位置已锁定
    {
        gnt_lock_location(NULL, NULL, 2);   // 仅计算lock偏离比较值
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
static uint8_t gnt_report_continuous_fail_times = 0;	///< 设备上报连续失败次数
#endif

/**@brief 周期上报任务
* @details 应用调用NB模组驱动的事件处理、主要包含周期上报、按键、运动检测GPS上报等
*/
void nb_report_thread(void *pvParameters)
{
    EventBits_t uxBits;
    
    xEventGroupSetBits(GntEventGroup, NB_PRERIOD_REPORT_EVENTBIT);      //上电上报一次
    app_timer_start(m_nb_report_timer_id, g_gnt_flash_info.gnt_report_period, NULL);     //测试3分钟一次上报（定时器单位为ms）
    
    // 看门狗使能，主要使用看门狗来监测周期上报这个功能
#if (GNT_WDT_EN == 1)
    my_wdt_init(g_gnt_flash_info.gnt_report_period + 3600000);  //配置并启动看门狗（单位ms）
#endif
    
    // 配置GPS初始工作方式
#if (GNT_GPS_EN == 1)
    #if (GNT_GPS_DEFAULT_REPORT_EN == 1)
    g_gnt_info.gnt_gps_realtime_report_flag = 1;
    #endif

    if(NB_ACK_OK != nb_handle.nb_fxnTablePtr->gpsInit(&nb_handle))
        NVIC_SystemReset();
#endif
    
#if (GNT_LIS3DH_EN == 1)
    xEventGroupClearBits(GntEventGroup, LISINT1_EVENTBIT|LISINT2_EVENTBIT);
    taskENTER_CRITICAL();   //对传感器的初始化进行临界保护
    lis3dh_init();
    taskEXIT_CRITICAL();
    NRF_LOG_INFO("lis3dh init over");
    // 测试加速度值上报
#endif
    
    gnt_led_indication_set(LED_NB_REGIST_OVER);
    
    while(1)
    {
        uxBits = xEventGroupWaitBits(GntEventGroup, 
                                    NB_PRERIOD_REPORT_EVENTBIT|KEY_EVENTBIT|KEY_LONGPUSH_EVENTBIT|KEY_RELEASE_EVENTBIT, 
                                    pdTRUE, pdFALSE, portMAX_DELAY);
        
        if((uxBits & NB_PRERIOD_REPORT_EVENTBIT) == NB_PRERIOD_REPORT_EVENTBIT)   //NB周期上报事件
        {
            #if (GNT_WDT_EN == 1)
            my_wdt_feed();
            #endif
            
            char temp[50];
            memset(temp, 0, 50);
            
            g_gnt_info.gnt_battery_level = battery_adc_sample();
			
			if(g_gnt_info.gnt_battery_level > GNT_REPORT_DISABLE_BATTERY_THRESHOLD) //电量低时取消上报
			{
				if(nrf_gpio_pin_read(BUTTON_1) == 1)
				{
					g_gnt_info.smc_open_state = 0;
				}
				else if(nrf_gpio_pin_read(BUTTON_1) == 0)
				{
					g_gnt_info.smc_open_state = 1;
				}
				nb_handle.nb_fxnTablePtr->getSign(&nb_handle);	//获取信号强度
				snprintf(temp, 50, "@03+%02d+%03d+%03d+%1d^", g_gnt_info.gnt_battery_level, gnt_temp_get(), g_me3616_info.me3616_rssi, g_gnt_info.smc_open_state); //上报电量、温度、井盖状态
				Ble_log_send((uint8_t*)temp, strlen(temp)); //蓝牙监测

                // 上报+主动释放RCC+更新注册
                nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_TRY);
				
				// 上报失败次数达到一定值时进行软件复位
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

        if((uxBits & KEY_EVENTBIT) == KEY_EVENTBIT) //模拟井盖状开启
        {
            char temp[20];
            memset(temp, 0, 20);
            snprintf(temp, 20, "@02+1^");
            g_gnt_info.gnt_battery_level = battery_adc_sample();
            
            Ble_log_send((uint8_t*)"@02+1^", 6);
            
            if(g_gnt_info.gnt_battery_level > GNT_REPORT_DISABLE_BATTERY_THRESHOLD) //电量低时取消上报
            {
                nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_TRY);
				Ble_log_send((uint8_t*)"notify", 6);
            }
//            vTaskDelay(10000);
        }
        if((uxBits & KEY_RELEASE_EVENTBIT) == KEY_RELEASE_EVENTBIT) //模拟井盖状关闭
        {
			#if SMC_CLOSE_REPORT_EN
            char temp[20];
            memset(temp, 0, 20);
            snprintf(temp, 20, "@02+0^");
            g_gnt_info.gnt_battery_level = battery_adc_sample();
            
            Ble_log_send((uint8_t*)"@02+0^", 6);
            
            if(g_gnt_info.gnt_battery_level > GNT_REPORT_DISABLE_BATTERY_THRESHOLD) //电量低时取消上报
            {
                nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_TRY);
            }
//            vTaskDelay(10000);
			#endif
        }
        
        if((uxBits & KEY_LONGPUSH_EVENTBIT) == KEY_LONGPUSH_EVENTBIT) //按键长按测试
        {
//            if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)    //NB功能使能
//            {
//                nb_handle.nb_fxnTablePtr->nbPowerOff(&nb_handle);   //NB模组关机
//            }
//            lis3dh_power_off();
//            bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
//            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
        }
        
        if((uxBits & GNT_CLEAR_MOVEFLAG_EVENTBIT) == GNT_CLEAR_MOVEFLAG_EVENTBIT)   //清除标签移动标志事件
        {
            char temp[20];
            memset(temp, 0, 20);
            snprintf(temp, 20, "@02+0^");
            if(g_gnt_info.gnt_battery_level > GNT_REPORT_DISABLE_BATTERY_THRESHOLD) //电量低时取消上报
            {
                // 上报+主动释放RCC+更新注册
                nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_TRY); 
            }
        }
    }
}

/**@brief 周期上报用定时器超时处理函数 */
static void nb_report_timer_handler(void * p_context)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(pdPASS == xEventGroupSetBitsFromISR(GntEventGroup, NB_PRERIOD_REPORT_EVENTBIT, &xHigherPriorityTaskWoken))
    {
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }  
}

/**@brief ME3616的驱动层消息上报->应用回调
* @param[in]  types 	消息类型
* @param[in]  *msg 		消息数据指针
* @param[in]  len 		消息数据长度
*/
int  NB_MsgreportCb(NB_msg_types_t types, char* msg, uint16_t len)
{
    switch (types)
    {
#if 0
        case MSG_GPS_GNSS_DATA_RECE:    //模组GPS数据处理
        {
            // $GPGLL,4004.74005,N,11614.19613,E,060845.00,A,A*5B<CR><LF>
//            NRF_LOG_INFO("%s  %d", msg, len);
            char* pColon = strchr(msg,'*');
            if(pColon)
            {
                pColon = pColon-3;
                if(strstr(pColon,"A,A"))    //数据有效
                {
                    char temp[100];
                    char longitude_data[20];    //经度
                    char latitude_data[20];     //纬度
                    uint8_t longitude_len, latitude_len;
                    
                    strcpy(temp, msg);
                    strcpy(latitude_data, strtok(temp,","));        //获取 纬度数据
                    latitude_len = strlen(latitude_data);
                    if((latitude_len < 10)||(latitude_len > 11))    //数据获取有误(长度不对)，取消此次上报
                        break;
                    else if(latitude_len == 10) //10位尾部补零
                        latitude_data[10] = '0';
                    
                    if(NULL == strchr(latitude_data,'.'))   //数据内容检查
                        break;
                    
                    strcpy(&latitude_data[11], strtok(NULL,","));   //获取纬度方向
                    latitude_len = strlen(latitude_data);
                    if(latitude_len != 12)      //11字节纬度数据+1字节方向
                        break;
//                    NRF_LOG_INFO("%s %d %s", latitude_data, latitude_len, &latitude_data[12]);//打印监测                                  
                    
                    
                    strcpy(longitude_data, strtok(NULL,","));       //获取经度数据
                    longitude_len = strlen(longitude_data);
                    if((longitude_len < 10)||(longitude_len > 11))    //数据获取有误(长度不对)，取消此次上报
                        break;
                    else if(longitude_len == 10) //10位尾部补零
                        longitude_data[10] = '0';
                    
                    if(NULL == strchr(longitude_data,'.'))   //数据内容检查
                        break;
                    
                    strcpy(&longitude_data[11], strtok(NULL,","));  //获取经度方向
                    longitude_len = strlen(longitude_data);
                    if(longitude_len != 12)      //11字节纬度数据+1字节方向
                        break;
//                    NRF_LOG_INFO("%s %d %s", longitude_data, longitude_len, &longitude_data[12]);
                    
                    
                    Ble_log_send((uint8_t*)latitude_data, 12);      //蓝牙监测
                    Ble_log_send((uint8_t*)longitude_data, 12);     //蓝牙监测
                    
                    // 定位成功LED指示
//                    NRF_LOG_INFO("get gps: %12.12s  %12.12s", longitude_data, latitude_data);
                    gnt_led_indication_set(LED_GPS_DATA_READY);
                    g_gnt_info.gnt_gps_data_ready_flag = 1;
                    
                    // 记录当前GPS信息，即更新最后/最新的的GPS数据
                    memcpy(g_gnt_info.gnt_last_latitude, latitude_data, 12);
                    memcpy(g_gnt_info.gnt_last_longitude, longitude_data, 12);
                    
                    // 使能移动时GPS信息实时上报
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
                    gnt_led_indication_set(LED_GPS_DATA_NONE);  //没有信号时的LED指示
                    g_gnt_info.gnt_gps_data_ready_flag = 0;
                    
                    // 测试GPS功耗用
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
            
            // 定时超时上报最后的坐标
            gnt_led_indication_set(LED_GPS_DATA_NONE);  //定位超时即资产静止时的LED指示
            g_gnt_info.gnt_gps_data_ready_flag = 0;     //GPS数据标志清零
            
            nb_handle.nb_fxnTablePtr->gpsStop(&nb_handle); //标签静止且GPS定位超时，停止定位，上传最后记录的坐标  
            nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_TRY); 
            
            
            #if 1   //静止时才上报最后的坐标信息
            gnt_lock_location(NULL, NULL, 3);   //计算最后记录的GPS信息偏离计算值last_com
            // 标签搬迁超过50m且没有上报过告警 （位置锁定及偏移告警功能）
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
                nb_handle.nb_fxnTablePtr->nbNotify(&nb_handle, (uint8_t*)temp, strlen(temp), NB_RRC_DISABLE, NB_UPDATE_ENABLE, NB_REPFAIL_REG_DELAY_TRY); //高频率上报
            }
            #endif
        }break;
        
        case MSG_GPS_NMEADATA_TIMEOUT:
        {
            if(g_me3616_info.gnss_run_state == 1)   //gps定位没有超时时执行
            {
                Ble_log_send((uint8_t*)"gps_data_lost", 13);
                NRF_LOG_INFO("%s", "gps_data_lost");
                
                gnt_led_indication_set(LED_GPS_DATA_NONE);      //定位超时即资产静止时的LED指示
                g_gnt_info.gnt_gps_data_ready_flag = 0;         //GPS数据标志清零
                nb_handle.nb_fxnTablePtr->gpsStop(&nb_handle);  //标签静止且GPS定位超时，停止定位，上传空坐标
            }
        }break;
 #endif
        
        case MSG_OCEANCONNECT_DATA_RECE:
        {
            // +M2MCLIRECV:02000100004030415E（@0A^）
            char rply_head[10]; //用于回复帧前部分messageid+mid+errcode
            char cmd_data[100]; //用于回复的AT指令
            char dl_data[50];   //下发的数据 @0A^
            
            if(len < 10)
                break;
            
            memcpy(rply_head, msg, 6);  //复制mid
            rply_head[1] = '3';         //应答messageid
            rply_head[6] = '0'; rply_head[7] = '0'; //errcode
            
            HexStrToStr(dl_data, &msg[10], (len-10)/2); //获取下发数据字符串
            
//            if(strstr((char*)dl_data,"@0A")) //清除标签移动标志，锁定当前位置
//            {
//                char temp[20];
//                memset(temp, 0, 20);
//                StrToHexStr(temp, "@0A+0^", 6); //0300010000064030412B305E
//                snprintf(cmd_data, 100, "AT+M2MCLISEND=%8.8s%04X%s", rply_head, 6, temp);
//                nb_handle.nb_fxnTablePtr->oceanconnectRply(&nb_handle, cmd_data);
//                // 回复失败重写有客户端控制
//                
//                // 锁定当前GPS信息，上报告警消除
//                #if (GNT_GPS_EN == 1)
//                gnt_lock_location(g_gnt_info.gnt_last_latitude, g_gnt_info.gnt_last_longitude, 1);
//                #endif
//                xEventGroupSetBits(GntEventGroup, GNT_CLEAR_MOVEFLAG_EVENTBIT); //清除标签移动标志事件
//                break;
//            }
            if(strstr((char*)dl_data,"@0B")) //配置心跳周期（单位小时）@0B+12^
            {
                uint8_t set_period;
                char temp[20];
                memset(temp, 0, 20);
                set_period = (dl_data[4]-0x30)*10 + (dl_data[5]-0x30);
                if((set_period < 1) || (set_period > 30))   // 设置周期参数不合理
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
                    
                    #if (GNT_WDT_EN == 1)   // 看门狗使能需要重启配置
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
            if(strstr((char*)msg, "0A")) //清除标签移动标志，锁定当前位置
            {
                char temp[20];  //用于回复的AT指令
                memset(temp, 0, 20);
                snprintf(temp, 20, "@0A+0^"); //回复清除标签移动标志

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
        if((uxBits & KEY_EVENTBIT) == KEY_EVENTBIT) //按键测试
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



