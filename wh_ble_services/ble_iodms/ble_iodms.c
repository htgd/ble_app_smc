/**
  **********************************************************************************
  * @file    ble_iodms.c
  * @author  wanghuan  any question please send mail to 371463817@qq.com
  * @version V1.0
  * @date    2018-07-31
  * @brief   门禁蓝牙通信处理收发服务
  **********************************************************************************
  * @attention
  *
  * 版权说明:Copyright (c) 2018-2020   江苏亨通光网科技有限公司
  * 硬件平台:nRF52832_QFAA
  * 修改日志:
  *
  **********************************************************************************
  */

#include "ble_iodms.h"
#include "ble_srv_common.h"
#include "sdk_common.h"

#define BLE_UUID_IODMS_TX_CHARACTERISTIC 0x0002                      /**< The UUID of the TX Characteristic. */
#define BLE_UUID_IODMS_RX_CHARACTERISTIC 0x0003                      /**< The UUID of the RX Characteristic. */

#define BLE_IODMS_MAX_RX_CHAR_LEN        BLE_IODMS_MAX_DATA_LEN        /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_IODMS_MAX_TX_CHAR_LEN        BLE_IODMS_MAX_DATA_LEN        /**< Maximum length of the TX Characteristic (in bytes). */

#define IODMS_BASE_UUID                  {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S110 SoftDevice.
 *
 * @param[in] p_iodms     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_iodms_t * p_iodms, ble_evt_t * p_ble_evt)    //蓝牙连接处理
{
    p_iodms->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S110 SoftDevice.
 *
 * @param[in] p_iodms     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_iodms_t * p_iodms, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_iodms->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S110 SoftDevice.
 *
 * @param[in] p_iodms     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_iodms_t * p_iodms, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (
        (p_evt_write->handle == p_iodms->rx_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_iodms->is_notification_enabled = true;
        }
        else
        {
            p_iodms->is_notification_enabled = false;
        }
    }
    else if (
             (p_evt_write->handle == p_iodms->tx_handles.value_handle)
             &&
             (p_iodms->data_handler != NULL)
            )
    {
        p_iodms->data_handler(p_iodms, p_evt_write->data, p_evt_write->len);    //交由应用层data_handler处理（写特性）
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}


/**@brief Function for adding RX characteristic. 接收特征值配置
 *
 * @param[in] p_iodms       Nordic UART Service structure.
 * @param[in] p_iodms_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rx_char_add(ble_iodms_t * p_iodms, const ble_iodms_init_t * p_iodms_init)
{
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
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

    // GATT特征值参数组
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_iodms->uuid_type;
    ble_uuid.uuid = BLE_UUID_IODMS_RX_CHARACTERISTIC;   //接收特征字节UUID

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm); //可读可写
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    //属性参数组
    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    //GATT属性参数组
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_IODMS_MAX_RX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_iodms->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_iodms->rx_handles);
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
}


/**@brief Function for adding TX characteristic.
 *
 * @param[in] p_iodms       Nordic UART Service structure.
 * @param[in] p_iodms_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t tx_char_add(ble_iodms_t * p_iodms, const ble_iodms_init_t * p_iodms_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;   //可写
    char_md.char_props.write_wo_resp = 1;   //写不需要回复
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_iodms->uuid_type;
    ble_uuid.uuid = BLE_UUID_IODMS_TX_CHARACTERISTIC;

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
    attr_char_value.max_len   = BLE_IODMS_MAX_TX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_iodms->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_iodms->tx_handles);
}


// （门禁服务）协议栈事件处理函数
void ble_iodms_on_ble_evt(ble_iodms_t * p_iodms, ble_evt_t * p_ble_evt)
{
    if ((p_iodms == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:     //连接处理
            on_connect(p_iodms, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:  //断开处理
            on_disconnect(p_iodms, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:   //写特性发生
            on_write(p_iodms, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

// 门禁蓝牙通信服务初始化
uint32_t ble_iodms_init(ble_iodms_t * p_iodms, const ble_iodms_init_t * p_iodms_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t iodms_base_uuid = IODMS_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_iodms);
    VERIFY_PARAM_NOT_NULL(p_iodms_init);

    // Initialize the service structure.
    p_iodms->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_iodms->data_handler            = p_iodms_init->data_handler;  //蓝牙接收数据处理
    p_iodms->is_notification_enabled = false;

    /**@snippet [Adding proprietary Service to S110 SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&iodms_base_uuid, &p_iodms->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_iodms->uuid_type;
    ble_uuid.uuid = BLE_UUID_IODMS_SERVICE;

    // Add a service declaration to the local server ATT table（添加GATT服务）
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,    //主服务
                                        &ble_uuid,
                                        &p_iodms->service_handle);
    /**@snippet [Adding proprietary Service to S110 SoftDevice] */
    VERIFY_SUCCESS(err_code);

    // Add the RX Characteristic.添加接收特征值
    err_code = rx_char_add(p_iodms, p_iodms_init);
    VERIFY_SUCCESS(err_code);

    // Add the TX Characteristic.添加发送特征值
    err_code = tx_char_add(p_iodms, p_iodms_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


// 蓝牙发送数据函数
uint32_t ble_iodms_string_send(ble_iodms_t * p_iodms, uint8_t * p_string, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;

    VERIFY_PARAM_NOT_NULL(p_iodms);

    if ((p_iodms->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_iodms->is_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_IODMS_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_iodms->rx_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_iodms->conn_handle, &hvx_params);
}


