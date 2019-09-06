/**@mainpage  智能井盖固件程序
* <table>
* <tr><th>Project  <td>ble_app_smc 
* <tr><th>Author   <td>wanghuan 
* <tr><th>Source   <td>E:\keil_workspace\NORDIC\nRF52832_htwh_sdk15.0\examples\ble_peripheral\ble_app_smc_freertos-doxygen
* </table>
* @section   项目详细描述
* 通过智能井盖管理系统的部署，管理人员通过手机APP与管理平台就能对辖区内井盖的安装、开闭、状态进行管理，出现异常情况及时通知维护人员进行检修，保障排水正常，保障市民安全。
*
* @section   功能描述  
* -# 本工程基于蓝牙新品nRF52832开发
* -# 本工程基于蓝牙协议栈开发，协议栈版本 SDK-15.0
* -# 智能井盖采用NB-IoT模组为ME3616
* 
* @section   用法描述 
* -# 智能井盖检测器安装指导
* -# 智能井盖检测器使用前需配置使能
* 
* @section   固件更新 
* <table>
* <tr><th>Date        <th>H_Version  <th>S_Version  <th>Author    <th>Description  </tr>
* <tr><td>2018/08/17  <td>1.0    <td>S02010041808171   <td>wanghuan  <td>创建初始版本 </tr>
* <tr><td>2019/06/24  <td>1.3    <td>S02010041906241   <td>wanghuan  <td>
* -# 电信平台增加上报需应答，应答超时时间默认40s；\n
*       代码宏：ME3616_NOTIFY_NEED_RPLY_EN
* -# 新增PSM进入超时处理，默认超时处理模组关机，超时时间默认200s；\n
*       代码宏：ME3616_PSM_TIMEOUT_HANDLE_EN
* -# 信号强度获取接口函数修改，增加可靠性，详见 me3616_getSignal()；
* -# 调试指令新增周期上报测试指令，710A-0D
* </tr>
* </table>
**********************************************************************************
*/

/**@file  main.c
* @brief   	项目主函数文件
* @details  主要包含协议应用栈程序框架，main函数入口
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-8-17
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

/**@defgroup app_smc Wh app_main
* @{
*
* @brief 智能井盖蓝牙应用主程序入口.
* @details 该模块主要为蓝牙协议栈初始化、协议栈各种参数的设置以及用户任务的创建等.
*/

#include "gnt_includes.h"

/**@name 协议栈用全局参数
* @brief 蓝牙5协议栈参数配置（广播、连接、安全等）相关宏定义，协议栈各模块句柄等全局参数
* @{
*/
#define DEVICE_NAME                         "HT_tag_00"                            	/**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "HTGD"                                  /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_SOC_OBSERVER_PRIO               1
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define TX_POWER_LEVEL                      (0)     								///< 初始发射功率

// 广播间隔与超时
#define APP_FAST_ADV_INTERVAL               MSEC_TO_UNITS(30, UNIT_0_625_MS)        /**< 快速广播间隔 (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_FAST_ADV_DURATION               MSEC_TO_UNITS(20000, UNIT_10_MS)        /**< 快速广播超时 (180 seconds) in units of 10 milliseconds. */
#define APP_SLOW_ADV_INTERVAL               MSEC_TO_UNITS(200, UNIT_0_625_MS)       /**< 慢速广播间隔 (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_SLOW_ADV_DURATION               MSEC_TO_UNITS(40000, UNIT_10_MS)        /**< 慢速广播超时 (180 seconds) in units of 10 milliseconds. */


// 连接间隔参数
#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(20, UNIT_1_25_MS)         /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(650, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */

// 蓝牙全局服务
NRF_BLE_GATT_DEF(m_gatt);                               /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                 /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                     /**< Advertising module instance. */
BLE_GNTS_DEF(m_gnts, NRF_SDH_BLE_TOTAL_LINK_COUNT);     ///< GPS_NB_TAG服务应用实例


static uint16_t m_conn_handle  = BLE_CONN_HANDLE_INVALID;       	/**< Handle of the current connection. */
static uint16_t m_ble_gnts_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;		///< 蓝牙最大发送字节数目,默认20字节


// 添加私有服务时，需更改NRF_SDH_BLE_VS_UUID_COUNT
static ble_uuid_t m_adv_uuids[] =                                   ///< Universally unique service identifiers.
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};
/** @} 协议栈用全局参数 */


#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                ///< Definition of Logger thread.
#endif
#if (TASK_HEAP_MONITOR_EN == 1)
TaskHandle_t    m_heap_monitor_thread;          					///< 堆栈监测任务句柄定义
void heap_monitor_thread(void *pvParameters);						///< 堆栈监测任务函数定义
#endif


// 事件标志组
EventGroupHandle_t   GntEventGroup = NULL;	///< 应用全局事件组
// 标签全局信息
GNT_flash_info_t    g_gnt_flash_info;		///< 全局设备flash信息结构体
GNT_info_t          g_gnt_info;   			///< 全局设备信息结构体

#if	SMC_ALRAM_QUEUE_EN
xQueueHandle		Key_Queue;				///< 按键事件队列
#endif

/**
* @name 函数前置定义
* @brief 
* @{
*/
static void advertising_init(void);
static void advertising_start(void * p_erase_bonds);
static void idle_state_handle(void);
static void tx_power_set(int8_t tx_power_level);
static void setDeviceName(char *name , uint8_t size);
static void setBleAdvTimeout(uint32_t timeout);
static void gnt_advertising_start(void);
static void gnt_advertising_stop(void);
/** @} 函数前置定义 */



///< 蓝牙参数设置功能函数列表
BLE_FxnTable nRF52_FxnTable = {
    .bleTxPowerSet          = tx_power_set,
    .bleAdvStart            = gnt_advertising_start,
    .bleAdvStop             = gnt_advertising_stop,
    .bleSetName             = setDeviceName,
    .bleSetAdvTimeout       = setBleAdvTimeout,
};

// 蓝牙按键变更名称计时器（超时回复名称）
APP_TIMER_DEF(m_key_BleNameChange_timer_id);        			///< 按键变更名称恢复定时器
static void key_BleNameChange_timer_handler(void * p_context);	///< 按键变更名称恢复超时回调函数


#if (DFU_SUPPORT == 1)
/**@brief 关机回调函数 \n
* 进入DFU，应用关机之前的一些操作，注册到power manager电源管理中
* @param[in]  event 	关机事件类型
*/
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            #if (GNT_WDT_EN == 1)
            my_wdt_feed();
            #endif
            
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

///< 注册应用关机事件处理函数（常用于关机前需要进行的一些操作：如flash操作，控制模块的关闭等）
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

/**@brief 协议栈状态观察回调函数
* @param[in]  state 	协议栈运行状态
*/
static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)	// 协议栈关闭
    {
        NRF_LOG_INFO("NRF_SDH_EVT_STATE_DISABLED to DFU mode.");
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);
        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

///< 协议栈状态观察者注册
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};

#endif	/* (DFU_SUPPORT == 1) */



/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&m_key_BleNameChange_timer_id, APP_TIMER_MODE_SINGLE_SHOT, key_BleNameChange_timer_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief 动态设置发射功率
* @param[in]  tx_power_level 	设置蓝牙发射功率值
*/
static void tx_power_set(int8_t tx_power_level)
{
    ret_code_t err_code;
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, tx_power_level);
    APP_ERROR_CHECK(err_code);
}

static uint8_t ble_name_changed_flag = 0;   ///< 设置蓝牙名称（有连接的情况下需要断开连接才能改名字），0:名字没有变更  1：按键触发变更

/**@brief 动态设置蓝牙名称
* @param[in]  *name 	蓝牙名称字符串指针
* @param[in]  size 		蓝牙名称字符串长度
*/
static void setDeviceName(char *name , uint8_t size)
{
    ret_code_t              err_code;
    ble_gap_conn_sec_mode_t sec_mode;
    
    if(ble_conn_state_status(m_conn_handle) != BLE_CONN_STATUS_CONNECTED)   //没有连接才能改名字
    {
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);  //连接模式，设置为开放模式
        err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)name, size);
        APP_ERROR_CHECK(err_code);
        
        gnt_advertising_stop(); //初始化广播包之前，必须保证广播停止
        advertising_init();
        gnt_advertising_start();
        ble_name_changed_flag = 1;
    }
}

/**@brief 开始广播 */
static void gnt_advertising_start(void)
{
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

/**@brief 停止广播 */
static void gnt_advertising_stop(void)
{
    sd_ble_gap_adv_stop(m_advertising.adv_handle);
}

/**@brief 动态变更蓝牙广播间隔(单位1ms)
* @param[in]  timeout 	蓝牙广播超时时间，0为不超时
*/
static void setBleAdvTimeout(uint32_t timeout)
{
    if(MSEC_TO_UNITS(timeout, UNIT_10_MS) < APP_SLOW_ADV_DURATION)
        timeout = APP_SLOW_ADV_DURATION;
    
    m_advertising.adv_modes_config.ble_adv_slow_timeout = MSEC_TO_UNITS(timeout, UNIT_10_MS);
    
    // 没有连接且不是广播空闲
    if((ble_conn_state_status(m_conn_handle) != BLE_CONN_STATUS_CONNECTED)&&(m_advertising.adv_evt != BLE_ADV_EVT_IDLE))
    {
        gnt_advertising_stop();
        gnt_advertising_start();    //重新开始广播
    }
}


/**@brief Function for the GAP initialization.通用访问规范（GAP）初始化
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    #if 1
    // 安全模式，安全权限设置
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);  //连接模式，设置为开放模式

    // 设备名称设置
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)g_gnt_flash_info.ble_name, strlen(g_gnt_flash_info.ble_name));
    APP_ERROR_CHECK(err_code);
    #endif

    // 应用图标设置                                        
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_TAG);
    APP_ERROR_CHECK(err_code);
																					
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    // 连接参数配置
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;  //最小连接间隔
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;  //最大连接间隔
    gap_conn_params.slave_latency     = SLAVE_LATENCY;      //从机延迟
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;   //连接超时监管（4s），超时时间必须大于（有效连接时间=连接间隔*（1+从机延迟值））

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);       //连接参数设置,主要是时间间隔设置和连接超时时间
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. \n
* 用于MTU长度协商，以此来设置蓝牙发送接收的最大长度
*/
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)//GATT事件处理函数
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))   //ATT_MTU size更新事件，即MTU协商
    {
        m_ble_gnts_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_gnts_max_data_len, m_ble_gnts_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT module. \n
* GATT初始化
*/
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
    
    // 下次作为从机连接的ATT_MTU长度设置
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the GPS_NB_TAG Service.
 *
 * @details This function will process the data received from the GPS_NB_TAG Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       GPS_NB_TAG Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void gnts_data_handler(ble_gnts_evt_t * p_evt)   //蓝牙接收数据处理（TX特征属性）
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (p_evt->type == BLE_GNTS_EVT_RX_DATA)    //蓝牙有数据接收
    {
//        NRF_LOG_INFO("Received data from BLE GNTS.");
//        NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        
        uint8_t rev_data[p_evt->params.rx_data.length+1];
        rev_data[0] = (uint8_t)p_evt->params.rx_data.length;
        memcpy(&rev_data[1], p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        
        if(pdFALSE == xQueueIsQueueFullFromISR(BLE_MsgQueue))   //队列is not full
        {
            if(pdPASS == xQueueSendFromISR(BLE_MsgQueue, rev_data, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
        }
//        ble_gnts_data_send(&m_gnts, (uint8_t*)p_evt->params.rx_data.p_data, &p_evt->params.rx_data.length, m_conn_handle);
    }

}
/**@snippet [Handling the data received over BLE] */


#if (DFU_SUPPORT == 1)
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");
            // YOUR_JOB: Disconnect all bonded devices that currently are connected.
            //           This is required to receive a service changed indication
            //           on bootup after a successful (or aborted) Device Firmware Update.
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}
#endif	// #if (DFU_SUPPORT == 1)


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
    
    
    // 添加GPS_NB_TAG服务
    ble_gnts_init_t     gnts_init;
    
    memset(&gnts_init, 0, sizeof(gnts_init));
    gnts_init.data_handler = gnts_data_handler;

    err_code = ble_gnts_init(&m_gnts, &gnts_init);
    APP_ERROR_CHECK(err_code);  
    
#if (DFU_SUPPORT == 1)
    ble_dfu_buttonless_init_t dfus_init = {0};
    // Initialize the async SVCI interface to bootloader.
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);

    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
#endif
    
}


/**@brief   Function for starting application timers.
 * @details Timers are run after the scheduler has started.
 */
static void application_timers_start(void)
{
//    ret_code_t err_code;
//    // Start application timers.
//    err_code = app_timer_start(m_led_timer_id, APP_TIMER_TICKS(3000), NULL);
//    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module. */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;   //m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

//    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.广播蓝牙事件处理函数
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
//    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
//            bsp_indication_set(BSP_INDICATE_ADVERTISING);
            break;

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("advertising time-out...");
//            bsp_indication_set(BSP_INDICATE_IDLE);
//            if(g_gnt_flash_info.gnt_enable_flag == GNT_DISABLE)
//            {
//                nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
//            }
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void app_ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:  //GAP断开事件
            NRF_LOG_INFO("Disconnected.");
            gnt_led_indication_set(LED_BLE_DISCONNECT); //蓝牙断开LED指示
            g_gnt_info.gnt_ble_connect_flag = 0;
        
            if(ble_name_changed_flag == 1)
            {
                ble_name_changed_flag = 0;
                if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)   //使能
                    nRF52_FxnTable.bleSetName(g_gnt_flash_info.ble_name, 12);
                else if(g_gnt_flash_info.gnt_enable_flag == GNT_DISABLE)
                    nRF52_FxnTable.bleSetName(g_gnt_flash_info.ble_name, 12);
            }
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:     //GAP连接事件
            NRF_LOG_INFO("Connected.");
            gnt_led_indication_set(LED_BLE_CONNECT); //蓝牙连接LED指示
            g_gnt_info.gnt_ble_connect_flag = 1;
        
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);   //将连接句柄分配给队列写模块的给定实例的函数
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT: //客户端超时
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            // 从机主动断开蓝牙连接
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT: //服务端超时
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

#if (GNT_POF_EN == 1)
/**@brief 蓝牙SOC事件应用回调函数. */
static void app_soc_evt_handler(uint32_t sys_evt, void * p_context)
{
    if ((p_context == NULL) || (sys_evt == NULL))
        return;

    switch (sys_evt)
    {
        // 掉电保护.
        case NRF_EVT_POWER_FAILURE_WARNING:
        {
            NRF_LOG_INFO("EVT_POWER_FAILURE.");
            
//            lis3dh_power_off();
//            nb_handle.nb_fxnTablePtr->nbPowerOff(&nb_handle);
//            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
        } break;

        default:
            // No implementation needed.
            break;
    }
}
#endif


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();    //协议栈回复使能应答，主要工作是设置系统时钟
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings. 使用默认配置设置协议栈
    // Fetch the start address of the application RAM.获取应用程序RAM的起始地址
    uint32_t ram_start = 0;
    // 采用默认配置从机，如需修改，在sdk_config修改NRF_SDH_BLE_PERIPHERAL_LINK_COUNT和NRF_SDH_BLE_CENTRAL_LINK_COUNT
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.使能协议栈
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.注册蓝牙处理事件，sdk15用观察者概念替代事件派发
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, app_ble_evt_handler, NULL);
    
    #if (GNT_POF_EN == 1)
	// 注册蓝牙SOC处理事件
    NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, app_soc_evt_handler, NULL);
    #endif
}



/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            bool delete_bonds = false;
            advertising_start(&delete_bonds);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

/**@brief Function for the Peer Manager initialization. */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage. */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality. */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;
//    int8_t tx_power_level = TX_POWER_LEVEL;     //设置广播发送功率

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;   //广播时的名称显示类型（全名）
    init.advdata.include_appearance      = true;                    //是否需要图标
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;     //蓝牙设备普通发现模式
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);    //广播UUID
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;
    
//    init.advdata.p_tx_power_level        = &tx_power_level;   //设置发射功率
    
    init.config.ble_adv_fast_enabled  = true;                       //快速广播使能
    init.config.ble_adv_fast_interval = APP_FAST_ADV_INTERVAL;      //快速广播间隔 30ms
    init.config.ble_adv_fast_timeout  = APP_FAST_ADV_DURATION;      //默认快速广播超时 20s
    
    init.config.ble_adv_slow_enabled  = true;                       //慢速广播使能
    init.config.ble_adv_slow_interval = APP_SLOW_ADV_INTERVAL;      //慢速广播间隔 200ms
//    init.config.ble_adv_slow_timeout  = APP_SLOW_ADV_DURATION;    //默认慢速广播超时 40s
    
    if(nrf_gpio_pin_read(USBVIN_CHECK) == 1)    //USB插入
    {
        gnt_led_indication_set(LED_USB_IN);
        g_gnt_info.gnt_usbvin_flag = 1;
        init.config.ble_adv_slow_timeout = 0;
    }
    else
    {
        gnt_led_indication_set(LED_USB_OUT);
        g_gnt_info.gnt_usbvin_flag = 0;
        init.config.ble_adv_slow_timeout  = APP_SLOW_ADV_DURATION;   //默认慢速广播超时 40s
    }

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief 按键变更名称恢复超时回调函数. */
static void key_BleNameChange_timer_handler(void * p_context)
{
    if(ble_conn_state_status(m_conn_handle) != BLE_CONN_STATUS_CONNECTED)   //没有连接就
    {
        if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)
            nRF52_FxnTable.bleSetName(g_gnt_flash_info.ble_name, 12);
        else if(g_gnt_flash_info.gnt_enable_flag == GNT_DISABLE)
            nRF52_FxnTable.bleSetName(g_gnt_flash_info.ble_name, 12);
    }
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
//    ret_code_t err_code;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	#if	SMC_ALRAM_QUEUE_EN
	uint8_t key_value;
	#endif

    switch (event)
    {
        case BSP_EVENT_WAKEUP:
            NRF_LOG_INFO("BSP_EVENT_WAKEUP.");
            gnt_led_indication_set(LED_KEY); //按键LED指示
            
//            g_gnt_info.smc_open_state = 1;  //井盖打开
            
            if(ble_conn_state_status(m_conn_handle) != BLE_CONN_STATUS_CONNECTED)
            {
                NRF_LOG_INFO("no connect and change name.");
                char temp[9];
                memcpy(temp, g_gnt_flash_info.ble_name, 7);
                memcpy(&temp[7], &g_gnt_flash_info.ble_name[10], 2);
                nRF52_FxnTable.bleSetName(temp, 9);    //缩短蓝牙名称以区分
                app_timer_stop(m_key_BleNameChange_timer_id);
                app_timer_start(m_key_BleNameChange_timer_id, 60000, NULL);
            }
        
            if(m_advertising.adv_evt == BLE_ADV_EVT_IDLE)
            {
                NRF_LOG_INFO("BLE_ADV_IDLE, go to start fast advertising.");
                gnt_advertising_start();
            }
            
			#if	SMC_ALRAM_QUEUE_EN
			// 使用覆盖式入队,不需要关注队列是否满
			key_value = 1;
			xQueueOverwriteFromISR(Key_Queue, &key_value, &xHigherPriorityTaskWoken);
			#endif
			
            if(pdPASS == xEventGroupSetBitsFromISR(GntEventGroup, KEY_EVENTBIT, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            } 

            break; // BSP_EVENT_WAKEUP
        
        case BSP_EVENT_SLEEP:
//            NRF_LOG_INFO("BSP_EVENT_SLEEP.");
//            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
//            NRF_LOG_INFO("BSP_EVENT_DISCONNECT.");
//            err_code = sd_ble_gap_disconnect(m_conn_handle,
//                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//            if (err_code != NRF_ERROR_INVALID_STATE)
//            {
//                APP_ERROR_CHECK(err_code);
//            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
//            NRF_LOG_INFO("BSP_EVENT_WHITELIST_OFF.");
//            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
//            {
//                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
//                if (err_code != NRF_ERROR_INVALID_STATE)
//                {
//                    APP_ERROR_CHECK(err_code);
//                }
//            }
            break; // BSP_EVENT_KEY_0
            
		case BSP_EVENT_LISINT1:
            NRF_LOG_INFO("BSP_EVENT_LISINT1");
//            gnt_led_indication_set(LED_KEY); //按键LED指示
            if(pdPASS == xEventGroupSetBitsFromISR(GntEventGroup, LISINT1_EVENTBIT, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            } 
            break;	
        
        case BSP_EVENT_LISINT2:
            NRF_LOG_INFO("BSP_EVENT_LISINT2.");
//            gnt_led_indication_set(LED_KEY); //按键LED指示
            if(pdPASS == xEventGroupSetBitsFromISR(GntEventGroup, LISINT2_EVENTBIT, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }    
            break;
        
        case BSP_EVENT_USBVIN:      //USB电源插入
            NRF_LOG_INFO("BSP_EVENT_USBVIN.");
            gnt_led_indication_set(LED_USB_IN); //USB插入LED指示
            g_gnt_info.gnt_usbvin_flag = 1;
            m_advertising.adv_modes_config.ble_adv_slow_timeout = 0;
            if(m_advertising.adv_evt == BLE_ADV_EVT_IDLE)
            {
                NRF_LOG_INFO("always start fast advertising.");
                gnt_advertising_start();
            }
            else if(ble_conn_state_status(m_conn_handle) != BLE_CONN_STATUS_CONNECTED)  //没有连接
            {
                gnt_advertising_stop();
                gnt_advertising_start();    //重新开始广播
            }
            break;
         
        case BSP_EVENT_USBVOUT:     //USB电源拔出
            NRF_LOG_INFO("BSP_EVENT_USBVOUT.");
            gnt_led_indication_set(LED_USB_OUT); //USB插入LED指示
            g_gnt_info.gnt_usbvin_flag = 0;
            m_advertising.adv_modes_config.ble_adv_slow_timeout = APP_SLOW_ADV_DURATION;    //40S+20S
            if(ble_conn_state_status(m_conn_handle) != BLE_CONN_STATUS_CONNECTED)
            {
                gnt_advertising_stop();
                gnt_advertising_start();    //重新开始广播
            }
            break;
        
        case BSP_EVENT_KEY_LONGPUSH:   
            NRF_LOG_INFO("BSP_EVENT_KEY_LONGPUSH");
            if(pdPASS == xEventGroupSetBitsFromISR(GntEventGroup, KEY_LONGPUSH_EVENTBIT, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            break;
            
        case BSP_EVENT_KEY_RELEASE:   
            NRF_LOG_INFO("BSP_EVENT_KEY_RELEASE");
//            g_gnt_info.smc_open_state = 0;  //井盖关闭
			#if	SMC_ALRAM_QUEUE_EN
			// 使用覆盖式入队,不需要关注队列是否满
			key_value = 0;
			xQueueOverwriteFromISR(Key_Queue, &key_value, &xHigherPriorityTaskWoken);
			#endif
		
            if(pdPASS == xEventGroupSetBitsFromISR(GntEventGroup, KEY_RELEASE_EVENTBIT, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            break;    
            
        default:
            break;
    }
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
    
    err_code = bsp_btn_gnt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run(); //等同于sd_app_evt_wait();
    }
}


/**@brief Function for starting advertising. */
static void advertising_start(void * p_erase_bonds)
{
    bool erase_bonds = *(bool*)p_erase_bonds;

    if (erase_bonds)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
		if(strstr(g_gnt_flash_info.ble_name, "HT_tag") || strstr(g_gnt_flash_info.ble_name, "ht_TAG"))
		{
			ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
			APP_ERROR_CHECK(err_code);
		}
    }
}


#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
     vTaskResume(m_logger_thread);
#endif
}

#if 0
/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}
#endif


SemaphoreHandle_t Ble_send_Semaphore;   ///< 蓝牙发送保护锁
TaskHandle_t    m_start_thread;         ///< 启动任务句柄定义 
void start_thread(void *pvParameters);	///< 启动任务函数定义

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
	
    // Initialize modules.
    log_init(); //日志打印初始化
    nrf_drv_clock_init();

    // Do not start any interrupt that uses system functions before system initialisation.
    // The best solution is to start the OS before any other initalisation.

    NRF_LOG_INFO("SMC FreeRTOS APP started.");
    
#if NRF_LOG_ENABLED
    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif
    
    // 协议栈初始化
    ble_stack_init();           //蓝牙协议栈配置和初始化
    power_management_init();    //低功耗管理初始化
    
    
    // 低功耗相关设置
    // Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;  //使支持WFI和WFE指令
   
    err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);  //使能DC/DC regulator，减少电流消耗（需增加外围LC器件）
    APP_ERROR_CHECK(err_code);
    err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    APP_ERROR_CHECK(err_code);

    timers_init();                      //应用定时器初始化
    buttons_leds_init(&erase_bonds);    //按键、LED初始化

#if (GNT_POF_EN == 1)
    // 掉电保护（即锂电池电量低时）
    err_code = sd_power_pof_enable(true);
    APP_ERROR_CHECK(err_code);
    err_code = sd_power_pof_threshold_set(NRF_POWER_THRESHOLD_V18);
    APP_ERROR_CHECK(err_code);
#endif
    
    
    //获取flash信息
    my_fds_init();
    my_fds_read(GNT_FILE, GNT_BLE_NAME_REC_KEY, g_gnt_flash_info.ble_name, 12);
    my_fds_read(GNT_FILE, GNT_ENABLE_FLAG_REC_KEY, &(g_gnt_flash_info.gnt_enable_flag), 1);
    my_fds_read(GNT_FILE, GNT_REPORT_PERIOD_REC_KEY, &(g_gnt_flash_info.gnt_report_period), 4); // 单位秒
    my_fds_read(GNT_FILE, GNT_IOT_PLATFORM_REC_KEY, &(g_gnt_flash_info.gnt_iot_platform), 1);
    if(g_gnt_flash_info.gnt_report_period < 60)     // 周期设置小于60s或未设置采用默认周期
    {
        g_gnt_flash_info.gnt_report_period = GNT_DEFAULT_REPORT_PERIOD;
    }
    if(g_gnt_flash_info.gnt_iot_platform == 1)
    {
        g_me3616_info.me3616_iot_platform = 1;
    }
    else
    {
        g_me3616_info.me3616_iot_platform = 0;
    }
    
#if (GNT_GPS_EN == 1)
    my_fds_read(GNT_FILE, GNT_LOCK_LATITUDE_REC_KEY, g_gnt_flash_info.gnt_lock_latitude, 12);
    my_fds_read(GNT_FILE, GNT_LOCK_LONGTITUDE_REC_KEY, g_gnt_flash_info.gnt_lock_longitude, 12);
    my_fds_read(GNT_FILE, GNT_MOVE_ALARM_FLAG_REC_KEY, &(g_gnt_flash_info.gnt_move_alarm_flag), 1);
    my_fds_read(GNT_FILE, GNT_GPS_MODE_REC_KEY, &(g_gnt_flash_info.gnt_gps_mode_flag), 1);
    //    g_me3616_info.gnss_run_mode = (uint8_t)g_gnt_flash_info.gnt_gps_mode_flag;
    if(g_gnt_flash_info.gnt_gps_mode_flag == 1)
        g_me3616_info.gnss_run_mode = 1;
    else
        g_me3616_info.gnss_run_mode = 0;
#endif
    
    NRF_LOG_INFO("gnt_enable_flag = %d", g_gnt_flash_info.gnt_enable_flag);
    NRF_LOG_INFO("ble_name = %s", g_gnt_flash_info.ble_name);
    
    // Initialize modules.
    gap_params_init();      //GAP通用访问规范，连接的安全模式、连接间隔设置
    gatt_init();            //GATT初始化
    advertising_init();     //广播初始化
    services_init();        //服务初始化
    conn_params_init();     //连接参数更新初始化
    peer_manager_init();    //设备管理初始化
    application_timers_start(); //应用定时器启动

    NRF_LOG_INFO("BLE init over.");
    tx_power_set(0);        //设置发射功率
    
//    if(nrf_gpio_pin_read(USBVIN_CHECK) == 1)    //初始上电USB插入(已经在advertising_init中检测)
//    {
//        gnt_led_indication_set(LED_USB_IN);
//        g_gnt_info.gnt_usbvin_flag = 1;
//        m_advertising.adv_modes_config.ble_adv_slow_timeout = 0;
//    }
    
    // Create a FreeRTOS task for the BLE stack.(softdevice_task)
    // The task will run advertising_start() before entering its loop.
    nrf_sdh_freertos_init(advertising_start, &erase_bonds); //设置不广播，创建sd任务
    
    if (pdPASS != xTaskCreate(start_thread, "STR", 128, NULL, 8, &m_start_thread))
    {
        NRF_LOG_INFO("NRF_ERROR_NO_MEM.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
	
    GntEventGroup = xEventGroupCreate();            //configUSE_16_BIT_TICKS为0  事件组24位
    if(GntEventGroup == NULL)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }   
    vSemaphoreCreateBinary(Ble_send_Semaphore);     //蓝牙发送保护锁
    if(Ble_send_Semaphore == NULL)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    BLE_MsgQueue = xQueueCreate( 10 , 21 );         //蓝牙数据接收缓存队列
    if(BLE_MsgQueue == NULL)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    #if	SMC_ALRAM_QUEUE_EN
	Key_Queue = xQueueCreate( 10 , 1 );				//按键值消息队列（对按键产生的事件进行排序）
	if(Key_Queue == NULL)
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	#endif
	
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();
    
	
    for (;;)
    {
        sleep_mode_enter();
        idle_state_handle();
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN); 	//不会到这一步
    }
}

/**@brief 启动GNT功能任务. */
void start_thread(void *pvParameters)
{
    // 创建NB通信处理任务，及串口通信任务
    if (pdPASS != xTaskCreate(nb_thread, "NB", 1536, NULL, NB_THREAD_PRIO, &m_nb_thread))
    {
        NRF_LOG_INFO("NRF_ERROR_NO_MEM.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    if(g_gnt_flash_info.gnt_enable_flag == GNT_FOR_NB_DFU)
    {
        vTaskSuspend(m_nb_thread);
    }
    
    // 创建BLE通信处理任务
    if (pdPASS != xTaskCreate(ble_comm_thread, "BLE", 512, NULL, BLE_COMM_THREAD_PRIO, &m_ble_comm_thread))
    {
        NRF_LOG_INFO("NRF_ERROR_NO_MEM.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
    // 测试加速度传感器
#if (GNT_LIS3DH_EN == 1)
    #if (GNT_LIS3DH_TEST_EN == 1)
    vTaskSuspend(m_nb_thread);
    
    taskENTER_CRITICAL();   //对传感器的初始化进行临界保护
    lis3dh_init();
    taskEXIT_CRITICAL();
    #endif
#endif
    
    // 创建任务堆栈监测任务
#if (TASK_HEAP_MONITOR_EN == 1)
    if (pdPASS != xTaskCreate(heap_monitor_thread, "HM", 64, NULL, HEAP_MONITOR_PRIO, &m_heap_monitor_thread))
    {
        NRF_LOG_INFO("NRF_ERROR_NO_MEM.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif
    
    vTaskDelete(NULL);	//启动任务作用已完成，删除自己
}


#if (TASK_HEAP_MONITOR_EN == 1)
///< 任务堆栈监测任务
void heap_monitor_thread(void *pvParameters)
{
    #if 1    
    UBaseType_t uxHighWaterMark;
    
    NRF_LOG_INFO("heap_monitor_thread start.");
    while (1)
    {
        vTaskDelay(10000);
        
        uxHighWaterMark = uxTaskGetStackHighWaterMark(xTaskGetIdleTaskHandle());
        NRF_LOG_INFO("IdleTask free stack size %d",(uint32_t)uxHighWaterMark);
        
        uxHighWaterMark = uxTaskGetStackHighWaterMark(xTaskGetHandle((const char *)"Tmr Svc"));
        NRF_LOG_INFO("TmrTask free stack size %d",(uint32_t)uxHighWaterMark);
        
        uxHighWaterMark = uxTaskGetStackHighWaterMark(xTaskGetHandle((const char *)"BLE"));
        NRF_LOG_INFO("SdTask free stack size %d",(uint32_t)uxHighWaterMark);
        
        uxHighWaterMark = uxTaskGetStackHighWaterMark(m_heap_monitor_thread);
        NRF_LOG_INFO("heap_monitor_thread free stack size %d",(uint32_t)uxHighWaterMark);
        
        uxHighWaterMark = uxTaskGetStackHighWaterMark(m_nb_thread);
        NRF_LOG_INFO("nb_thread free stack size %d",(uint32_t)uxHighWaterMark);
        
        uxHighWaterMark = uxTaskGetStackHighWaterMark(m_ble_comm_thread);
        NRF_LOG_INFO("ble_comm_thread free stack size %d",(uint32_t)uxHighWaterMark);
        
        uxHighWaterMark = uxTaskGetStackHighWaterMark(m_logger_thread);
        NRF_LOG_INFO("logger_thread free stack size %d",(uint32_t)uxHighWaterMark);
        
        if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)
        {
            uxHighWaterMark = uxTaskGetStackHighWaterMark(m_nb_report_thread);
            NRF_LOG_INFO("nb_report_thread free stack size %d",(uint32_t)uxHighWaterMark);
        }
    }
    #endif
    
    
    #if 0
    UBaseType_t   ArraySize;
    TaskStatus_t  *StatusArray;
    uint8_t       x;
    
    ArraySize = uxTaskGetNumberOfTasks(); //获取任务个数
    StatusArray = pvPortMalloc(ArraySize*sizeof(TaskStatus_t));
    
    while(1)
    {
        if(StatusArray != NULL){ //内存申请成功

            ArraySize = uxTaskGetSystemState( (TaskStatus_t *)  StatusArray,
                                              (UBaseType_t   )  ArraySize,
                                              (uint32_t *    )  &FreeRTOSRunTimeTicks );

            SEGGER_RTT_printf(0,"TaskName\t\t\tPriority\t\tTaskNumber\t\tMinStk\t\t\n");
            for(x = 0;x<ArraySize;x++){

                SEGGER_RTT_printf(0,"%s\t\t\t%d\t\t\t%d\t\t\t%d\t\t%d\r\n",
                        StatusArray[x].pcTaskName,
                        (int)StatusArray[x].uxCurrentPriority,
                        (int)StatusArray[x].xTaskNumber,
                        (int)StatusArray[x].usStackHighWaterMark,
                        (int)((float)StatusArray[x].ulRunTimeCounter/FreeRTOSRunTimeTicks*100));
            }
            SEGGER_RTT_printf(0,"\n\n");
        }
        vTaskDelay(2000);

    }
    #endif
}
#endif


/**@brief 蓝牙发送接口
* @param[in]  *data 	发送蓝牙数据指针
* @param[in]  len 		发送蓝牙数据长度
*/
void Ble_send(uint8_t*data, uint16_t len)
{
    xSemaphoreTake( Ble_send_Semaphore,  portMAX_DELAY);
    len = CTout(data, len);
    if(len<21)
    {
        ble_gnts_data_send(&m_gnts, data, &len, m_conn_handle);
    }
    else
    {
        uint8_t i;
        uint16_t slen;
        for(i=0; i<(len/20); i++)
        {
            slen = 20;
            ble_gnts_data_send(&m_gnts, &data[i*20], &slen, m_conn_handle);
        }
        slen = len%20;
        ble_gnts_data_send(&m_gnts, &data[(len/20)*20], &slen, m_conn_handle);
    }  
    xSemaphoreGive( Ble_send_Semaphore );
}

/**@brief 蓝牙调试接口
* @param[in]  *data 	发送蓝牙数据指针
* @param[in]  len 		发送蓝牙数据长度
*/
void Ble_log_send(uint8_t*data, uint16_t len)
{
#if (GNT_BLE_LOG_EN == 1)
	xSemaphoreTake( Ble_send_Semaphore,  portMAX_DELAY);
	//    len = CTout(data, len);	//调试接口禁止使用该语句，否则会影响发送的数据
    if(len<21)
    {
        ble_gnts_data_send(&m_gnts, data, &len, m_conn_handle);
    }
    else
    {
        uint8_t i;
        uint16_t slen;
        for(i=0; i<(len/20); i++)
        {
            slen = 20;
            ble_gnts_data_send(&m_gnts, &data[i*20], &slen, m_conn_handle);
        }
        slen = len%20;
        ble_gnts_data_send(&m_gnts, &data[(len/20)*20], &slen, m_conn_handle);
    }  
    xSemaphoreGive( Ble_send_Semaphore );
#endif
}


/** @} app_smc*/


/**@defgroup bsp_drivers Wh driver module.
* @{
* @brief 外设驱动.
*/
/** @} bsp_drivers*/

/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/





