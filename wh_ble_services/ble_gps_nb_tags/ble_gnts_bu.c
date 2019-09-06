/**@file    ble_gnts_bu.c
* @brief   	GPS-NB资产追踪服务(GPS NB-IOT TAG)
* @details  
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-08-17
* @version 	V1.0
* @copyright	Copyright (c) 2016-2018  江苏亨通光网科技有限公司
**********************************************************************************
* @attention
* OS: FreeRTOS 10.0
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/08/17  <td>1.0      <td>wanghuan  <td>创建初始版本
* </table>
*/

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_GNTS)
#include "ble.h"
#include "ble_gnts_bu.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

// 日志打印配置
#define NRF_LOG_MODULE_NAME ble_gnts
#if BLE_GNTS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BLE_GNTS_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_GNTS_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_GNTS_CONFIG_DEBUG_COLOR
#else // BLE_GNTS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // BLE_GNTS_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


// 基于SIG-UUID
#define BLE_UUID_GNTS_CHARACTERISTIC        0xFFE1      /**< The UUID of the TX Characteristic. */

#define BLE_UUID_GNTS_LOG_CHARACTERISTIC    0xFFE2      //蓝牙日志调试接口


/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 *
 * @param[in] p_nus     GPS_NB_TAG Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_gnts_t * p_gnts, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    ble_gnts_evt_t             evt;
    ble_gatts_value_t          gatts_val;
    uint8_t                    cccd_value[2];
    ble_gnts_client_context_t * p_client = NULL;

    err_code = blcm_link_ctx_get(p_gnts->p_link_ctx_storage,
                                 p_ble_evt->evt.gap_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gap_evt.conn_handle);
    }

    /* Check the hosts CCCD value to inform of readiness to send data using the RX characteristic */
    memset(&gatts_val, 0, sizeof(ble_gatts_value_t));
    gatts_val.p_value = cccd_value;
    gatts_val.len     = sizeof(cccd_value);
    gatts_val.offset  = 0;

    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_gnts->gnt_handles.cccd_handle,
                                      &gatts_val);

    if ((err_code == NRF_SUCCESS)     &&
        (p_gnts->data_handler != NULL) &&
        ble_srv_is_notification_enabled(gatts_val.p_value))
    {
        if (p_client != NULL)
        {
            p_client->is_notification_enabled = true;
        }

        memset(&evt, 0, sizeof(ble_gnts_evt_t));
        evt.type        = BLE_GNTS_EVT_COMM_STARTED;
        evt.p_gnts      = p_gnts;
        evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        evt.p_link_ctx  = p_client;

        p_gnts->data_handler(&evt);
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_gnts    GPS_NB_TAG Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_gnts_t * p_gnts, ble_evt_t const * p_ble_evt)
{
    ret_code_t                    err_code;
    ble_gnts_evt_t                 evt;
    ble_gnts_client_context_t    * p_client;
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    err_code = blcm_link_ctx_get(p_gnts->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
    }

    memset(&evt, 0, sizeof(ble_gnts_evt_t));
    evt.p_gnts      = p_gnts;
    evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
    evt.p_link_ctx  = p_client;

    if ((p_evt_write->handle == p_gnts->gnt_handles.cccd_handle) &&
        (p_evt_write->len == 2))
    {
        if (p_client != NULL)
        {
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                p_client->is_notification_enabled = true;
                evt.type                          = BLE_GNTS_EVT_COMM_STARTED;
            }
            else
            {
                p_client->is_notification_enabled = false;
                evt.type                          = BLE_GNTS_EVT_COMM_STOPPED;
            }

            if (p_gnts->data_handler != NULL)
            {
                p_gnts->data_handler(&evt);
            }

        }
    }
    else if ((p_evt_write->handle == p_gnts->gnt_handles.value_handle) &&
             (p_gnts->data_handler != NULL))
    {
        evt.type                  = BLE_GNTS_EVT_RX_DATA;
        evt.params.rx_data.p_data = p_evt_write->data;
        evt.params.rx_data.length = p_evt_write->len;

        p_gnts->data_handler(&evt);
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_HVN_TX_COMPLETE event from the SoftDevice.
 *
 * @param[in] p_gnts    GPS_NB_TAG Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_hvx_tx_complete(ble_gnts_t * p_gnts, ble_evt_t const * p_ble_evt)
{
    ret_code_t                 err_code;
    ble_gnts_evt_t              evt;
    ble_gnts_client_context_t * p_client;

    err_code = blcm_link_ctx_get(p_gnts->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
        return;
    }

    if (p_client->is_notification_enabled)
    {
        memset(&evt, 0, sizeof(ble_gnts_evt_t));
        evt.type        = BLE_GNTS_EVT_TX_RDY;
        evt.p_gnts       = p_gnts;
        evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
        evt.p_link_ctx  = p_client;

        p_gnts->data_handler(&evt);
    }
}


// GNT标签蓝牙通信收发特征添加
static uint32_t gnt_char_add(ble_gnts_t * p_gnts, ble_gnts_init_t const * p_gnts_init)
{
    ble_gatts_char_md_t char_md;    //attribute全部都用char_md表示
    ble_gatts_attr_t    attr_char_value;    //数据的值
    ble_uuid_t          ble_uuid;           //特征类型用uuid表示
    ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_md_t cccd_md;    //CCCD信息

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char gnt_desc[] = "GNT";
    char_md.p_char_user_desc = (uint8_t *)gnt_desc;
    char_md.char_user_desc_size = strlen(gnt_desc);
    char_md.char_user_desc_max_size =strlen(gnt_desc);
    
    char_md.char_props.read          = 1;   //可读
    char_md.char_props.notify        = 1;   //可通知
    char_md.char_props.write         = 1;   //可写
    char_md.char_props.write_wo_resp = 1;   //无回复写
//    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = &cccd_md;
    char_md.p_sccd_md                = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_GNTS_CHARACTERISTIC);

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_GNTS_MAX_DATA_LEN;

    return sd_ble_gatts_characteristic_add(p_gnts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_gnts->gnt_handles);
}

// 蓝牙日志打印特征添加
#if 0
static uint32_t gnt_log_char_add(ble_gnts_t * p_gnts, ble_gnts_init_t const * p_gnts_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read          = 0;   //可读
    char_md.char_props.notify        = 1;   //可通知
    char_md.char_props.write         = 0;   //可写
    char_md.char_props.write_wo_resp = 0;   //无回复写
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_GNTS_LOG_CHARACTERISTIC);

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_GNTS_MAX_DATA_LEN;

    return sd_ble_gatts_characteristic_add(p_gnts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_gnts->gnt_log_handles);
}
#endif


// GPS_NB_TAG服务蓝牙事件处理函数
void ble_gnts_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_gnts_t * p_gnts = (ble_gnts_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_gnts, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_gnts, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            on_hvx_tx_complete(p_gnts, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

#if 0
// GPS_NB_TAG服务系统事件处理函数
void ble_gnts_on_sys_evt(uint32_t sys_evt, void * p_context)
{
    if ((p_context == NULL) || (sys_evt == NULL))
    {
        return;
    }

    switch (sys_evt)
    {
        // 掉电保护.
        case NRF_EVT_POWER_FAILURE_WARNING:
        {
            
        } break;

        default:
            // No implementation needed.
            break;
    }
}
#endif


// GPS_NB_TAG服务初始化
uint32_t ble_gnts_init(ble_gnts_t * p_gnts, ble_gnts_init_t const * p_gnts_init)
{
    ret_code_t    err_code;
    ble_uuid_t    ble_uuid;
//    ble_uuid128_t gnts_base_uuid = GNTS_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_gnts);
    VERIFY_PARAM_NOT_NULL(p_gnts_init);

    // Initialize the service structure.
    p_gnts->data_handler = p_gnts_init->data_handler;
    
    // 添加基于通用SIG-UUID的GNT服务
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_GNTS_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_gnts->service_handle);
    VERIFY_SUCCESS(err_code);

    // 添加GNT服务特征字节
    err_code = gnt_char_add(p_gnts, p_gnts_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

// 蓝牙数据发送
uint32_t ble_gnts_data_send(ble_gnts_t * p_gnts, uint8_t * p_data, uint16_t * p_length, uint16_t conn_handle)
{
    ret_code_t                 err_code;
    ble_gatts_hvx_params_t     hvx_params;
    ble_gnts_client_context_t * p_client;

    VERIFY_PARAM_NOT_NULL(p_gnts);

    err_code = blcm_link_ctx_get(p_gnts->p_link_ctx_storage, conn_handle, (void *) &p_client);  //获取RX通知是否使能
    VERIFY_SUCCESS(err_code);

    if ((conn_handle == BLE_CONN_HANDLE_INVALID) || (p_client == NULL))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (!p_client->is_notification_enabled)//通知使能了，没有使能通知返回错误，不蓝牙发送
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (*p_length > BLE_GNTS_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_gnts->gnt_handles.value_handle;//数据处理
    hvx_params.p_data = p_data;     //数据
    hvx_params.p_len  = p_length;   //长度
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;//通知

    return sd_ble_gatts_hvx(conn_handle, &hvx_params);
}

// 蓝牙日志数据发送
#if 0
uint32_t ble_gnts_log_data_send(ble_gnts_t * p_gnts, uint8_t * p_data, uint16_t * p_length, uint16_t conn_handle)
{
    ret_code_t                 err_code;
    ble_gatts_hvx_params_t     hvx_params;
    ble_gnts_client_context_t * p_client;

    VERIFY_PARAM_NOT_NULL(p_gnts);

    err_code = blcm_link_ctx_get(p_gnts->p_link_ctx_storage, conn_handle, (void *) &p_client);  //获取RX通知是否使能
    VERIFY_SUCCESS(err_code);

    if ((conn_handle == BLE_CONN_HANDLE_INVALID) || (p_client == NULL))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (!p_client->is_notification_enabled)//通知使能了，没有使能通知返回错误，不蓝牙发送
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (*p_length > BLE_GNTS_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_gnts->gnt_handles.value_handle;//数据处理
    hvx_params.p_data = p_data;     //数据
    hvx_params.p_len  = p_length;   //长度
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;//通知

    return sd_ble_gatts_hvx(conn_handle, &hvx_params);
}
#endif

#endif // NRF_MODULE_ENABLED(BLE_GNTS)


/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/




