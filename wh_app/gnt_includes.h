/**@file    gnt_includes.c
* @brief   	GNT�豸��������ͷ�ļ�
* @details  ͷ�ļ�������ȫ�ֽṹ�塢ȫ�ֱ�����ȫ�ֺ���
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-08-23
* @version 	V1.0
* @copyright	Copyright (c) 2018-2020  ���պ�ͨ�����Ƽ����޹�˾
**********************************************************************************
* @attention
* Ӳ��ƽ̨:nRF52832_QFAA \n
* SDK�汾��nRF5_SDK_15.0.0
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/08/23  <td>1.0      <td>wanghuan  <td>������ʼ�汾
* </table>
*/

#ifndef __GNT_INCLUDES_H
#define	__GNT_INCLUDES_H

/* Includes ------------------------------------------------------------------*/
#include "gnt_config.h"
//--��׼��
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

//--Э��ջ
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "peer_manager.h"   //�豸����
#include "fds.h"
#include "bsp_btn_ble.h"

//--FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"
#include "queue.h"


#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"   //�͹��Ĺ���

//--��־��ӡ
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include <SEGGER_RTT_Conf.h>
#include <SEGGER_RTT.h>

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
//--GNTʹ�����������
#include "fds.h"
#include "app_uart.h"           //�����������
//#include "ble_bas.h"          //������ط���
#include "nrf_drv_saadc.h"      //ADC�������
//#include "ble_gnts.h"         //GPS NB-IOT TAG����
#include "ble_gnts_bu.h"
#include "nrf_drv_twi.h"        //I2C�������
#include "bsp_gnt_function.h"   //Ӧ�ö����
#if (GNT_LIS3DH_EN == 1)
#include "bsp_lis3dh.h"         //LIS3DH����
#include "nrf_lis3dh.h"
#endif
#include "bsp_me3616.h"
#include "bsp_fds.h"
#include "bsp_fstorage.h"
#include "nrf_temp.h"
//#include "nrf_power.h"

#if (DFU_SUPPORT == 1)
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "ble_dfu.h"
#include "nrf_power.h"
#include "nrf_bootloader_info.h"
#endif

#if (GNT_WDT_EN == 1)
#include "nrf_drv_wdt.h"
#endif


// ȫ��������
#define NB_THREAD_PRIO		        6      	///< �������ȼ�
#define NB_REPORT_THREAD_PRIO		4      	///< �������ȼ�
#define BLE_COMM_THREAD_PRIO		5      	///< �������ȼ�

extern TaskHandle_t    m_nb_thread;      	///< ������
extern void nb_thread(void *pvParameters); 	///< ������
extern TaskHandle_t    m_nb_report_thread;          
extern void nb_report_thread(void *pvParameters);   
extern TaskHandle_t    m_ble_comm_thread;          
extern void ble_comm_thread(void *pvParameters);

#if (TASK_HEAP_MONITOR_EN == 1)
#define HEAP_MONITOR_PRIO		    2
extern TaskHandle_t    m_heap_monitor_thread;          
extern void heap_monitor_thread(void *pvParameters);
#endif


// ȫ�ֶ��о��  
extern xQueueHandle    	BLE_MsgQueue; 	///< ����ͨ�����ݶ���
#if	SMC_ALRAM_QUEUE_EN
extern xQueueHandle		Key_Queue; 		///< ����ֵ��Ϣ���о��
#endif

// ȫ���¼���־��
extern EventGroupHandle_t   GntEventGroup;	///< �豸ȫ���¼���

// �¼���־λ�����С��������жϴ����¼���
#define KEY_EVENTBIT                    (1 << 0)	///< ���������¼�
#define KEY_LONGPUSH_EVENTBIT           (1 << 1)	///< ���������¼�
#define KEY_RELEASE_EVENTBIT            (1 << 2)	///< �����ͷ��¼�
#define LISINT1_EVENTBIT                (1 << 3)	///< ���ٶȴ������ж�1�¼�
#define LISINT2_EVENTBIT                (1 << 4)	///< ���ٶȴ������ж�2�¼�
#define NB_PRERIOD_REPORT_EVENTBIT      (1 << 5)	///< NB���ڳ�ʱ�¼�
#define GNT_CLEAR_MOVEFLAG_EVENTBIT     (1 << 6)	///< �豸�����־�¼�


// GNT��ǩ��ȫ�ֱ�־����Ϣ
//#pragma pack(4)
//__attribute__((aligned (4)));
// flash������ȫ�ֱ���������ǽṹ���Ա���׵�ַΪ4�ֽڱ���
/**@brief �豸ȫ��flash������Ϣ�ṹ�� */
typedef struct
{
    char    ble_name[16];               ///< ��������
    uint32_t gnt_enable_flag;           ///< GNT��ǩ��������ʹ�ܣ���ʹ��NB��GPS��    0��δʹ�ܣ�1��ʹ��
    uint32_t gnt_report_period;         ///< ��ǩNB�����ϱ����� ����λ�룩
    uint32_t gnt_iot_platform;          ///< ��ǩ��ƽ̨ѡ��
#if (GNT_GPS_EN == 1)
    char    gnt_lock_latitude[12];      ///< ��ǩ����γ��
    char    gnt_lock_longitude[12];     ///< ��ǩ��������
    uint32_t gnt_move_alarm_flag;       ///< ��ǩ��Ǩ�澯����������λ��50m��
    uint32_t gnt_gps_mode_flag;         ///< ��ǩGPS��λģʽ
#endif
    
}GNT_flash_info_t;

/**@brief �豸ȫ�ֲ�����Ϣ�ṹ�� */
typedef struct
{
    uint32_t gnt_err_state;             ///< ��ǩ��������״̬��0��ʾ�޴���
    
    uint8_t gnt_usbvin_flag;            ///< USB�������־ 0���γ�/δ���룬1������
    uint8_t gnt_ble_connect_flag;       ///< BLE����״̬   1:���ӣ�0���Ͽ�
    uint8_t gnt_current_led_state;      ///< ��ǰLED״̬
    uint8_t gnt_battery_level;          ///< ��ǰ��ǩ����ֵ����ʵʱ��
    
    uint8_t smc_open_state;             ///< ��ǰ���ǿ���״̬ 0�������� 1���쳣����
    
#if (GNT_GPS_EN == 1)
    uint8_t gnt_gps_data_ready_flag;    ///< GPS��λ�Ƿ�ɹ� 1��GPS��λ������  0�������������ݻ�δ������λ��
    uint8_t gnt_gps_battery_low_flag;   ///< ��ǩ����GPS�͵�����־ 0��û�е͵����� 1������GPS�����ͣ������ظ�һ��������ֵ������������GPS
    uint8_t gnt_gps_realtime_report_flag;   ///< ��ǩGPS��Ϣʵʱ�ϱ���־ 0���رգ� 1������
    char    gnt_last_latitude[12];      ///< ��ǩ���λ������γ��
    char    gnt_last_longitude[12];     ///< ��ǩ���λ�����꾭��
    int     gnt_last_latitude_com;      ///< ��ǩ���γ�����ڼ���ƫ���ֵ
    int     gnt_last_longitude_com;     ///< ��ǩ��󾭶����ڼ���ƫ���ֵ
    int     gnt_lock_latitude_com;      ///< ��ǩ����γ�����ڼ���ƫ���ֵ
    int     gnt_lock_longitude_com;     ///< ��ǩ�����������ڼ���ƫ���ֵ
#endif
    
}GNT_info_t;
//#pragma pack()

extern GNT_flash_info_t     g_gnt_flash_info;
extern GNT_info_t           g_gnt_info;


// Ӧ��FALSH�洢fds
#define GNT_FILE                        (0xF010)	///< �豸flash�ļ����
#define GNT_ENABLE_FLAG_REC_KEY         (0x7101)	///< �豸ʹ�ܱ�־��¼���
#define GNT_BLE_NAME_REC_KEY            (0x7102)	///< �豸�������Ƽ�¼���
#define GNT_REPORT_PERIOD_REC_KEY       (0x7107)	///< �豸�ϱ����ڼ�¼���
#define GNT_IOT_PLATFORM_REC_KEY        (0x7108)	///< �豸��ƽ̨���ü�¼���
#if (GNT_GPS_EN == 1)
#define GNT_LOCK_LATITUDE_REC_KEY       (0x7103)
#define GNT_LOCK_LONGTITUDE_REC_KEY     (0x7104)
#define GNT_MOVE_ALARM_FLAG_REC_KEY     (0x7105)
#define GNT_GPS_MODE_REC_KEY            (0x7106)
#endif


// ȫ�ֺ���
extern void Ble_send(uint8_t*data, uint16_t len);
extern void Ble_log_send(uint8_t*data, uint16_t len);   
extern void gnt_change_report_period(uint32_t period);

// ���ߺ���
extern uint16_t CTin(uint8_t * data, uint16_t m);
extern uint16_t CTout(uint8_t * data, uint16_t m);
extern uint16_t GetCrc_16(uint8_t * Data, uint16_t len);


/**@brief BLE�������ù����б� */
typedef struct
{
    void (*bleTxPowerSet)(int8_t);
    void (*bleAdvStart)(void);
    void (*bleAdvStop)(void);
    void (*bleSetName)(char *, uint8_t);
    void (*bleSetAdvTimeout)(uint32_t);
	void (*bleDisconnect)(void);
    
}BLE_FxnTable;

extern BLE_FxnTable nRF52_FxnTable;		///< BLE�������ò������


/**@brief GNT��ǩ��ʹ�ܣ��������ͣ� */
typedef enum
{
    GNT_DISABLE         = 0,    // ��������������͹���״̬�����Ѻ�֧������ͨ�ţ�
    GNT_ENABLE          = 1,    // ��ǩȫ���ܹ���
    GNT_FOR_NB_DFU      = 2,    // ����NBģ��̼�����ģʽ���ϵ�ģ�鲻������
    
} GNT_enable_type_t;



#endif	/* __GNT_INCLUDES_H */


















