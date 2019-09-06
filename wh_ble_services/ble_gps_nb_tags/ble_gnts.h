/**
  **********************************************************************************
  * @file    ble_gnts.h
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

#ifndef BLE_GNTS_H__
#define BLE_GNTS_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_gnts instance.
 *
 * @param     _name            Name of the instance.
 * @param[in] _nus_max_clients Maximum number of NUS clients connected at a time.
 * @hideinitializer
 */
#define BLE_GNTS_DEF(_name, _gnts_max_clients)                      \
    BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),  \
                             (_gnts_max_clients),                  \
                             sizeof(ble_gnts_client_context_t));   \
    static ble_gnts_t _name =                                      \
    {                                                             \
        .p_link_ctx_storage = &CONCAT_2(_name, _link_ctx_storage) \
    };                                                            \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
                         BLE_GNTS_BLE_OBSERVER_PRIO,               \
                         ble_gnts_on_ble_evt,                      \
                         &_name)

    
#define BLE_UUID_GNTS_SERVICE 0x0001    //GPS NB-iot 资产标签追踪服务UUID

#define OPCODE_LENGTH        1
#define HANDLE_LENGTH        2

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the GPS_NB_TAG service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_GNTS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_GNTS_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif


/**@brief   GPS_NB_TAG Service event types. */
typedef enum
{
    BLE_GNTS_EVT_RX_DATA,      /**< Data received. */
    BLE_GNTS_EVT_TX_RDY,       /**< Service is ready to accept new data to be transmitted. */
    BLE_GNTS_EVT_COMM_STARTED, /**< Notification has been enabled. */
    BLE_GNTS_EVT_COMM_STOPPED, /**< Notification has been disabled. */
} ble_gnts_evt_type_t;

/* Forward declaration of the ble_nus_t type. */
typedef struct ble_gnts_s ble_gnts_t;


/**@brief   GPS_NB_TAG Service @ref BLE_GNTS_EVT_RX_DATA event data.蓝牙接收数据事件数据结构体
 *
 * @details This structure is passed to an event when @ref BLE_GNTS_EVT_RX_DATA occurs.
 */
typedef struct
{
    uint8_t const * p_data; /**< A pointer to the buffer with received data. */
    uint16_t        length; /**< Length of received data. */
} ble_gnts_evt_rx_data_t;


/**@brief GPS_NB_TAG Service client context structure.
 *
 * @details This structure contains state context related to hosts.
 */
typedef struct
{
    bool is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
} ble_gnts_client_context_t;

/**@brief   GPS_NB_TAG Service event structure.
 *
 * @details This structure is passed to an event coming from service.
 */
typedef struct
{
    ble_gnts_evt_type_t         type;           /**< Event type. */
    ble_gnts_t                  * p_gnts;       /**< A pointer to the instance. */
    uint16_t                    conn_handle;    /**< Connection handle. */
    ble_gnts_client_context_t   * p_link_ctx;   /**< A pointer to the link context. */
    union
    {
        ble_gnts_evt_rx_data_t  rx_data;        /**< @ref BLE_GNTS_EVT_RX_DATA event data. */
    } params;
} ble_gnts_evt_t;


/**@brief GPS_NB_TAG Service event handler type. */
typedef void (* ble_gnts_data_handler_t) (ble_gnts_evt_t * p_evt);


/**@brief   GPS_NB_TAG Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_nus_init
 *          function.
 */
typedef struct
{
    ble_gnts_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_gnts_init_t;


/**@brief   GPS_NB_TAG Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_gnts_s
{
    uint8_t                         uuid_type;          /**< UUID type for Nordic UART Service Base UUID. */
    uint16_t                        service_handle;     /**< Handle of Nordic UART Service (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        tx_handles;         /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        rx_handles;         /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
    blcm_link_ctx_storage_t * const p_link_ctx_storage; /**< Pointer to link context storage with handles of all current connections and its context. */
    ble_gnts_data_handler_t         data_handler;       /**< Event handler to be called for handling received data. */
};


/**@brief   Function for initializing the GPS_NB_TAG Service.
 *
 * @param[out] p_gnts     GPS_NB_TAG Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_gnts_init Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_nus or p_gnts_init is NULL.
 */
uint32_t ble_gnts_init(ble_gnts_t * p_gnts, ble_gnts_init_t const * p_gnts_init);


/**@brief   Function for handling the GPS_NB_TAG Service's BLE events.
 *
 * @details The GPS_NB_TAG Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the GPS_NB_TAG Service event handler of the
 * application if necessary.
 *
 * @param[in] p_ble_evt     Event received from the SoftDevice.
 * @param[in] p_context     GPS_NB_TAG Service structure.
 */
void ble_gnts_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief   Function for sending a data to the peer.
 *
 * @details This function sends the input string as an RX characteristic notification to the
 *          peer.
 *
 * @param[in]     p_gnts      Pointer to the GPS_NB_TAG Service structure.
 * @param[in]     p_data      String to be sent.
 * @param[in,out] p_length    Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_gnts_data_send(ble_gnts_t * p_gnts,
                           uint8_t   * p_data,
                           uint16_t  * p_length,
                           uint16_t    conn_handle);


#ifdef __cplusplus
}
#endif

#endif // BLE_GNTS_H__

/** @} */
