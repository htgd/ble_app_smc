/**
  **********************************************************************************
  * @file    ble_gnts.c
  * @author  wanghuan  any question please send mail to 371463817@qq.com
  * @version V1.0
  * @date    2018-08-15
  * @brief   GPS-NB资产追踪服务(GPS NB-IOT TAG)
  **********************************************************************************
  * @attention
  *
  * 版权说明:Copyright (c) 2018-2020   江苏亨通光网科技有限公司
  * 硬件平台:nRF52832_QFAA
  * SDK版本：nRF5_SDK_15.0.0
  * 修改日志:
  *
  **********************************************************************************
  */

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_GNTS)
#include "ble.h"
#include "ble_gnts.h"
#include "ble_srv_common.h"

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


#define BLE_UUID_GNTS_TX_CHARACTERISTIC 0x0002                      /**< The UUID of the TX Characteristic. */
#define BLE_UUID_GNTS_RX_CHARACTERISTIC 0x0003                      /**< The UUID of the RX Characteristic. */

#define BLE_GNTS_MAX_RX_CHAR_LEN        BLE_GNTS_MAX_DATA_LEN        /**< Maximum length of the RX Characteristic (in bytes)244. */
#define BLE_GNTS_MAX_TX_CHAR_LEN        BLE_GNTS_MAX_DATA_LEN        /**< Maximum length of the TX Characteristic (in bytes). */

#define GNTS_BASE_UUID                  {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */



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
                                      p_gnts->rx_handles.cccd_handle,
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

    if ((p_evt_write->handle == p_gnts->tx_handles.cccd_handle) &&
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
    else if ((p_evt_write->handle == p_gnts->rx_handles.value_handle) &&
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


/**@brief Function for adding TX characteristic. 添加TX 特征字节
 *
 * @param[in] p_gnts       GPS_NB_TAG Service structure.
 * @param[in] p_gnts_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t tx_char_add(ble_gnts_t * p_gnts, ble_gnts_init_t const * p_gnts_init)
{
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_gnts->uuid_type;
    ble_uuid.uuid = BLE_UUID_GNTS_TX_CHARACTERISTIC;

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
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_GNTS_MAX_TX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_gnts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_gnts->tx_handles);
    /**@snippet [Adding proprietary characteristic to the SoftDevice] */
}


/**@brief Function for adding RX characteristic. 添加RX特征字节
 *
 * @param[in] p_gnts       GPS_NB_TAG Service structure.
 * @param[in] p_gnts_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rx_char_add(ble_gnts_t * p_gnts, const ble_gnts_init_t * p_gnts_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_gnts->uuid_type;
    ble_uuid.uuid = BLE_UUID_GNTS_RX_CHARACTERISTIC;

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
    attr_char_value.max_len   = BLE_GNTS_MAX_RX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_gnts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_gnts->rx_handles);
}


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


// GPS_NB_TAG服务初始化
uint32_t ble_gnts_init(ble_gnts_t * p_gnts, ble_gnts_init_t const * p_gnts_init)
{
    ret_code_t    err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t gnts_base_uuid = GNTS_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_gnts);
    VERIFY_PARAM_NOT_NULL(p_gnts_init);

    // Initialize the service structure.
    p_gnts->data_handler = p_gnts_init->data_handler;

    /**@snippet [Adding proprietary Service to the SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&gnts_base_uuid, &p_gnts->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_gnts->uuid_type;
    ble_uuid.uuid = BLE_UUID_GNTS_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_gnts->service_handle);
    /**@snippet [Adding proprietary Service to the SoftDevice] */
    VERIFY_SUCCESS(err_code);

    // Add the RX Characteristic.
    err_code = rx_char_add(p_gnts, p_gnts_init);
    VERIFY_SUCCESS(err_code);

    // Add the TX Characteristic.
    err_code = tx_char_add(p_gnts, p_gnts_init);
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

    hvx_params.handle = p_gnts->tx_handles.value_handle;//数据处理
    hvx_params.p_data = p_data;     //数据
    hvx_params.p_len  = p_length;   //长度
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;//通知

    return sd_ble_gatts_hvx(conn_handle, &hvx_params);
}



#endif // NRF_MODULE_ENABLED(BLE_GNTS)







