/**
  **********************************************************************************
  * @file    ble_iodms.h
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

#ifndef BLE_IODMS_H__
#define BLE_IODMS_H__

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>

#define BLE_UUID_IODMS_SERVICE  0x0001                          //门禁蓝牙通信服务UUID
#define BLE_IODMS_MAX_DATA_LEN  (GATT_MTU_SIZE_DEFAULT - 3)     /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/* Forward declaration of the ble_iodms_t type. */
typedef struct ble_iodms_s ble_iodms_t;

/**@brief Nordic UART Service event handler type. */
typedef void (*ble_iodms_data_handler_t) (ble_iodms_t * p_iodms, uint8_t * p_data, uint16_t length);

/**@brief Nordic UART Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_iodms_init
 *          function.
 */
typedef struct
{
    ble_iodms_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_iodms_init_t;

/**@brief Nordic UART Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_iodms_s
{
    uint8_t                  uuid_type;               // 蓝牙通信服务UUID类型 /**< UUID type for Nordic UART Service Base UUID. */
    uint16_t                 service_handle;          /**< Handle of Nordic UART Service (as provided by the SoftDevice). */
    ble_gatts_char_handles_t tx_handles;              /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t rx_handles;              /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
    uint16_t                 conn_handle;             /**< Handle of the current connection (as provided by the SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool                     is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
    ble_iodms_data_handler_t   data_handler;            /**< Event handler to be called for handling received data. */
};

/**@brief Function for initializing the Nordic UART Service.
 *
 * @param[out] p_iodms      Nordic UART Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_iodms_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_iodms or p_iodms_init is NULL.
 */
uint32_t ble_iodms_init(ble_iodms_t * p_iodms, const ble_iodms_init_t * p_iodms_init);

/**@brief Function for handling the Nordic UART Service's BLE events.
 *
 * @details The Nordic UART Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the Nordic UART Service event handler of the
 * application if necessary.
 *
 * @param[in] p_iodms       Nordic UART Service structure.
 * @param[in] p_ble_evt   Event received from the SoftDevice.
 */
void ble_iodms_on_ble_evt(ble_iodms_t * p_iodms, ble_evt_t * p_ble_evt);

/**@brief Function for sending a string to the peer.
 *
 * @details This function sends the input string as an RX characteristic notification to the
 *          peer.
 *
 * @param[in] p_iodms       Pointer to the Nordic UART Service structure.
 * @param[in] p_string    String to be sent.
 * @param[in] length      Length of the string.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_iodms_string_send(ble_iodms_t * p_iodms, uint8_t * p_string, uint16_t length);


#endif // BLE_IODMS_H__

/** @} */
