/**@file    bsp_me3616.h
* @brief   	Э���ֽڽ���
* @details  IODH������֡��0x7Eת���2�ֽ����У�0x7D��0x5E��������֡��0x7D ת���2�ֽ����У�0x7D��0x5D��
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-8-17
* @version 	V1.0
* @copyright	Copyright (c) 2016-2018  ���պ�ͨ�����Ƽ����޹�˾
**********************************************************************************
* @attention
* OS: FreeRTOS 10.0
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/08/17  <td>1.0      <td>wanghuan  <td>������ʼ�汾
* </table>
*/

/**@defgroup bsp_me3616 Bsp me3616 driver module.
* @{
* @ingroup bsp_drivers
* @brief ʹ�ø�����֮ǰ���Ƚ������������ʵ��ע��. \n
* ME3616����֧����ƽ̨Onenet��OceanConnect \n
* ��ʹ��GPS����ʹ��ʱ��֧��GPS���� 
*/

#ifndef __BSP_ME3616_H
#define	__BSP_ME3616_H

#include <stdint.h>
#include "gnt_includes.h"

// <<< Use Configuration Wizard in Context Menu >>>\n

// <e> �������� - ME3616_EN
// <i> ��������
// <i> ��Ҫ�������������Ĵ�����
#define ME3616_EN   	1

// <e> GPS����ʹ�� - ME3616_GPS_EN
// <i> 1��ʹ�ܣ� 0����ʹ��
#define ME3616_GPS_EN   0
// </e>

// <e> notify����Ӧ����� - ME3616_NOTIFY_NEED_RPLY_EN
// <i> notify�յ�ƽ̨Ӧ�����ж��ϱ��ɹ�(�ݽ�������OceanConnectƽ̨)
#define ME3616_NOTIFY_NEED_RPLY_EN	1
// <o> notify�յ�ƽ̨Ӧ��ʱʱ��(��λ��) - ME3616_NOTIFY_RPLY_TIMEOUT_VALUE <20-300><#*1000>
// <i> �ж�notify�յ�ƽ̨Ӧ��ʱ��ʱ��ֵ
#define ME3616_NOTIFY_RPLY_TIMEOUT_VALUE	40000
// </e>

// <e> PSM��ʱ���� - ME3616_PSM_TIMEOUT_HANDLE_EN
// <i> ����PSM��ʱ����PSM�����ϱ��쳣����NB��δ����PSMģʽ������йػ�����������
#define ME3616_PSM_TIMEOUT_HANDLE_EN	1
// <o> PSM��ʱʱ��(��λ��) - ME3616_PSM_TIMEOUT_VALUE <30-300><#*1000>
// <i> �жϽ���PSM��ʱ��ʱ��ֵ
#define ME3616_PSM_TIMEOUT_VALUE	200000
// </e>

// <e> �źŲ�� - ME3616_SIGNAL_BAD_HANDLE_EN
// <i> �źŲ�ʱ������ĳЩͨ�Ź��̽����Ż��������ܻ�Ӱ���ϱ��ɹ����ʣ�
#define ME3616_SIGNAL_BAD_HANDLE_EN		0
// <o> ��ȡ�ź�ǿ�����Դ��� - ME3616_SIGNAL_TRY_TIMES <10-200>
// <i> ��ȡ�ź�ǿ�����Դ���,ÿ�ξ��ȴ�1s
#define ME3616_SIGNAL_TRY_TIMES	30
// <o> �źŲ�ע�����Դ��� - ME3616_SIGNALBAD_REG_TRY_TIMES <1-5>
// <i> �źŲ�ע�����Դ���
#define ME3616_SIGNALBAD_REG_TRY_TIMES	1
// <o> �źŲ�ע�ᳬʱʱ�� - ME3616_SIGNALBAD_REG_TIMEOUT <60-200><#*1000>
// <i> �źŲ�ע�ᳬʱʱ�䵥λ��
#define ME3616_SIGNALBAD_REG_TIMEOUT	90000
// <o> �ź�ǿ�Ȳ���ֵ - ME3616_SIGNAL_TRY_TIMES <51-113>
// <i> �ź�ǿ�Ȳ���ֵ,255Ϊû���źţ�113Ϊ�źż���,���㹫ʽ 113-s*2��10 -> 93��
#define ME3616_SIGNAL_BAD_THRESHOLD	93
// </e>

// </e> 
//===========================================End of ��������

// <<< end of configuration section >>>


#if (ME3616_EN)

#if (ME3616_GPS_EN)
    #if (GNT_GPS_DEFAULT_MODE == 1)
    #define ME3616_GPS_MODE_ALONE       0
    #define ME3616_GPS_MODE_AGPS        1
    #elif (GNT_GPS_DEFAULT_MODE == 2)
    #define ME3616_GPS_MODE_ALONE       1
    #define ME3616_GPS_MODE_AGPS        0
    #endif
#endif

// NBĬ����ƽ̨
#if (GNT_NB_DEFAULT_PLATFORM == 1)
#define ME3616_IOT_PLATFORM_OCEANCONNECT            0
#define ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT     1
#elif (GNT_NB_DEFAULT_PLATFORM == 2)
#define ME3616_IOT_PLATFORM_OCEANCONNECT            1
#define ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT     0
#endif

// BANDƵ��
#define BAND_3      3   ///< ��ͨ
#define BAND_5      5   ///< ����
#define BAND_8      8   ///< �ƶ�


#define HEX_MODE    0
#define TEXT_MODE   1
#define OCEANCONNECT_DATA_MODE          HEX_MODE   ///< ֧������ͨ�Ų���textģʽ���ַ�����AT+M2MCLICFG=1,1��ME3616C1AV1.0B07��ME3616G1AV0.0B01�����ϰ汾֧�֣�
#define OCEANCONNECT_REMOTE_INFO        "180.101.147.115,5683"	///< ƽ̨��ַ��Ϣ


/**@enum NB_msg_types_t
* @brief ���������ϱ�Ӧ����Ϣ����
*/
typedef enum
{
    MSG_NONE            = 0,

#if (ME3616_GPS_EN)    
    MSG_GPS_GNSS_DATA_RECE,				///< ��gps��λ����
    MSG_GPS_POSITION_TIMEOUT,			///< gps��λ��ʱ
    MSG_GPS_NMEADATA_TIMEOUT,			///< gps��ȡ���ݳ�ʱ
#endif
    
    MSG_OCEANCONNECT_DATA_RECE,			///< OCEANCONNECT ƽ̨���ݽ���
    
    MSG_ONENET_READ_REQ,				///< ONENET ƽ̨������
    MSG_ONENET_WRITE_REQ,				///< ONENET ƽ̨д����
    
    MSG_END
} NB_msg_types_t;


/**@enum NB_err_types_t
* @brief ����ָ�����Ӧ���������
*/
typedef enum
{
    NB_ACK_NULL         = 0,
    NB_ACK_OK           = 1,			///< Ӧ��ɹ�
    NB_ACK_NOT_EXPECT   = 2,			///< Ӧ��ֵ��������ֵ
    NB_ACK_ERROR        = 3,			///< Ӧ�����
    NB_ACK_TIMEOUT      = 4,			///< Ӧ��ʱ
    NB_CMD_ERROR        = 5,			///< �������
    
    NB_IOT_REGIST_SUCCESS,				///< ��ƽ̨ע��ɹ�
    NB_IOT_REGISTING,					///< ��ƽ̨ע����
    NB_IOT_REGIST_FAILED,				///< ��ƽ̨ע��ʧ��
    
    NB_ONENET_OBSERVE_WAIT_TIMEOUT,		///< onenet �۲�ע�ᳬʱ
    NB_ONENET_DISCOVER_WAIT_TIMEOUT,	///< onenet �����ֳ�ʱ
    NB_ONENET_UPDATE_SUCCESS,			///< onenet ����ע��ɹ�
    NB_ONENET_UPDATE_FAIL,				///< onenet ����ע��ʧ��
    
    NB_NOTIFY_SUCCESS,	///< �ϱ�ƽָ̨��ɹ�
    NB_NOTIFY_FAIL,		///< �ϱ�ƽָ̨��ʧ��
    
    NB_RPLY_SUCCESS,	///< Ӧ��ƽָ̨��ɹ�
    NB_RPLY_FAIL,		///< Ӧ��ƽָ̨��ʧ��

#if (ME3616_GPS_EN)     
    NB_AGPS_DATA_READY,	///< GPS�������ݾ���
    NB_AGPS_DATA_NOT_READY,	///< GPS��������δ����
#endif
    
} NB_err_types_t;


/**@struct ME3616_info_t
* @brief ME3616��Ϣ�ṹ�� \n
* ����洢ME3616����Ϣ
*/
typedef struct
{
    uint8_t     me3616_matready;            ///< ME3616����AT����
    uint8_t     me3616_cfun;                ///< ME3616��Ƶ״̬
    uint8_t     me3616_cpin;                ///< ME3616��������״̬
    uint8_t     me3616_getip;               ///< ME3616����ע���ȡIP״̬
    
    uint8_t     me3616_SwRevision[17];      ///< ME3616ģ��̼��汾��
    uint8_t     me3616_IMSI[16];            ///< �ƶ��豸�����
    uint8_t     me3616_IMEI[16];            ///< ģ�����к�

    uint8_t     me3616_rssi;                ///< ģ������ź�ǿ�ȣ�0~255��
    uint8_t     me3616_band;                ///< ��ǰNB����BAND
    uint8_t     me3616_psmisset;            ///< PSM�Ƿ����ã� 0������PSM  1��ʹ��PSM
    uint8_t     me3616_zslr;                ///< ���߹����Ƿ����ã� 0���ر�����  1�������߿���
    
    volatile uint8_t     me3616_powerOn_flag;       ///< ģ�鿪��״̬��0���ػ���  1������     
    uint8_t     me3616_info_collection_flag;        ///< ģ�������Ϣ�ɼ���ɱ�־
    volatile uint8_t     me3616_enterPSM_flag;      ///< ģ���Ƿ����PSM��0��û�н���  1������PSM
    volatile uint8_t     me3616_comOpen_flag;       ///< ģ�鴮���Ƿ�ʹ�ܣ�0�����ڹر�   1�����ڴ�
    
    uint8_t     iot_regist_status;          ///< ������ƽ̨ע��״̬
    uint8_t     me3616_register_status;     ///< ���總��ע��״̬
    
    uint32_t    regist_lifetime;            ///< ƽ̨ע������ʱ��
    
    uint8_t     me3616_iot_platform;        ///< NB��ƽ̨ѡ��

#if (ME3616_GPS_EN) 
    volatile uint8_t     gnss_run_state;	///< gps����״̬
    uint8_t     agps_data_ready;			///< gps������λ�����Ƿ����
    uint8_t     gnss_run_mode;				///< ����ˮ��λģʽ
#endif

//    volatile uint8_t     oceanconnect_data_mode;     ///< ����ƽ̨����ͨ��ģʽ��0��Hex mode�� 1��Text mode
    
} ME3616_info_t;


///< NBģ����������������
typedef struct NB_Conf *NB_Handle;

//******************************************************************************
/**@struct NB_FxnTable
* @brief ����NBģ��Ҫʵ�ֵĹ��ܺ����б�ṹ��
*/
typedef struct
{
    void (*nbComOpen)(NB_Handle);       ///< NBʹ�ô��ڡ���ʱ����ʼ�����ص�ע��
    void (*nbComClose)(NB_Handle);      ///< NBʹ�ô��ڹر�
    uint8_t (*nbSet_IotRegist)(NB_Handle, uint32_t, uint8_t);       ///< NBģ���ʼ�����ü�IOTƽ̨ע��
    uint8_t (*nbModuleInit)(NB_Handle, uint32_t, uint8_t, uint8_t); ///< ��NBģ���ʼ������
    
    uint8_t (*cmdSend)(NB_Handle, char *, uint16_t, uint8_t);   	///< NBָ�������Ӧ���
    void (*nbRrcRelease)(NB_Handle);                ///< NB�����ͷ�RRC����
    uint8_t (*psmSet)(NB_Handle, char *, char *);   ///< NBģ��PSM����
    uint8_t (*getModuleInfo)(NB_Handle, uint8_t, uint8_t, uint8_t); 	///< ��ȡNBģ�����Ϣ
    uint8_t (*getSign)(NB_Handle);                  ///< ��ȡNBģ���ź�ǿ��
    
    void (*nbWakeUp)(NB_Handle);                ///< NBģ�黽��
    void (*nbReset)(NB_Handle);                 ///< ��NBģ�鸴λ����
    void (*nbPowerOn)(NB_Handle);               ///< NBģ�鿪��
    void (*nbPowerOff)(NB_Handle);              ///< NBģ��ػ�
    
    // ����ƽ̨oceanconnect��������
    uint8_t (*oceanconnectRegist)(NB_Handle, uint32_t);	///< ����ƽ̨ע��
    uint8_t (*oceanconnectDeregist)(NB_Handle);			///< ����ƽ̨ȥע��
//    uint8_t (*oceanconnectSend)(NB_Handle, uint8_t*, uint16_t);
    uint8_t (*oceanconnectRply)(NB_Handle, char*);		///< ����ƽ̨����Ӧ��
    
    // �ƶ�onenetƽ̨��������
    uint8_t (*onenetRegist)(NB_Handle, uint32_t);		///< �ƶ�ƽ̨ע��
    uint8_t (*onenetDeregist)(NB_Handle);				///< �ƶ�ƽ̨ȥע��
    uint8_t (*onenetUpdate)(NB_Handle, uint32_t, uint8_t);	///< �ƶ�ƽ̨����ע��
    
    uint8_t (*nbNotify)(NB_Handle, uint8_t*, uint16_t, uint8_t, uint8_t, uint8_t);	///< ��ƽ̨֪ͨ�ϱ�
    
    // GPS��������
#if (ME3616_GPS_EN) 
    uint8_t (*gpsInit)(NB_Handle);	///< GPS���ܳ�ʼ��
    uint8_t (*gpsRun)(NB_Handle);	///< GPS������λ
    uint8_t (*gpsStop)(NB_Handle);	///< GPSֹͣ��λ
#endif
    // ��������
    void (*eventHandle)(NB_Handle);	///< NB�����¼�������

    // �͹��Ĳ���
    #if defined ME3616_LP_TEST
    void (*lpTest)(NB_Handle);
    #endif
    
    #if (GNT_NB_TEST_EN == 1)
    void (*nbTest)(NB_Handle);
    #endif
} NB_FxnTable;

extern const NB_FxnTable ME3616_FxnTable;	///< NB�������ܺ����б�
extern  ME3616_info_t    g_me3616_info;		///< ME3616����Ϣ


///< ��������UART���ջص�
typedef void (*nb_receive_cb)(char*, uint16_t);
///< ������ʱ����ʱ�ص�
typedef void (*nb_timeout_cb)(void);


/**@struct HW_FxnTable
* @brief NBģ��Ĵ����붨ʱ��Ӳ����ؽӿڽṹ��
*/
typedef struct
{
    const uint32_t  baudrate;                       ///< ���ڲ�����
    void (*com_openFxn)(nb_receive_cb, uint32_t); 	///< ��Ӳ������
    void (*com_sendFxn)(uint8_t*,uint16_t);			///< ���ڷ���
    void (*com_closeFxn)(void);						///< ���ڹر�

    void (*initTimerFxn)(nb_timeout_cb);			///< ��ʱ����ʼ��
    void (*startTimerFxn)(uint32_t);				///< ������ʱ��
    void (*stopTimerFxn)(void);						///< �رն�ʱ��
} HW_FxnTable;

typedef int (*NB_ReceCB)(NB_msg_types_t, char*, uint16_t);	///< Ӧ���ϱ��ص���������

//******************************************************************************
/**@struct NB_Config
* @brief NBģ�����������ṹ��
*/
typedef struct NB_Conf
{
    NB_FxnTable         *nb_fxnTablePtr;    ///< NB���ܺ����б�
    HW_FxnTable         *hw_fxnTablePtr;    ///< NB uart����ʱ������ָ��
    NB_ReceCB           AppReceCB;          ///< NB��Ӧ����Ӧ�ûص�
} NB_Config;

extern NB_Config  nb_handle;				///< ME3616��������ṹ


/**@name ME3616�������¼���־��
* @brief ME3616�������¼���־�飬���ڸ�Чͬ��
* @{
*/
///< �ڲ����¼���
extern EventGroupHandle_t   Me3616EventGroup;
#define NB_CPIN_EVENTBIT                            (1 << 0)   	///< NB����NB������
#define NB_GETIP_EVENTBIT                           (1 << 1)    ///< ��ȡIP��NBģ�鸽������
#define NB_ACK_OK_EVENTBIT                          (1 << 2)    ///< NBָ�����OK
#define NB_ACK_ERROR_EVENTBIT                       (1 << 3)    ///< NBָ���������

#define NB_OCEANCONNECT_REG_SUCCESS_EVENTBIT        (1 << 4)    ///< ����ƽ̨ע��ɹ�
#define NB_OCEANCONNECT_REG_FAILED_EVENTBIT         (1 << 5)    ///< ����ƽ̨ע��ʧ��
#define NB_OCEANCONNECT_OB_SUCCESS_EVENTBIT         (1 << 6)    ///< ����ƽ̨observe success
#define NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT     (1 << 7)    ///< ����ƽ̨֪ͨ�ɹ�
#define NB_OCEANCONNECT_NOTIFY_FAILED_EVENTBIT      (1 << 8)    ///< ����ƽ̨֪ͨʧ��
#define NB_OCEANCONNECT_NOTIFY_AND_RPLY_SUCCESS_EVENTBIT      (1 << 9)    ///< ����ƽ̨֪ͨ��Ӧ��ɹ�

#define NB_ONENET_CONNECT_SUCCESS_EVENTBIT          (1 << 10)  	///< �ƶ�ƽ̨���ӳɹ�
#define NB_ONENET_REG_SUCCESS_EVENTBIT              (1 << 11)   ///< �ƶ�ƽ̨ע��ɹ�
#define NB_ONENET_REG_FAILED_EVENTBIT               (1 << 12)   ///< �ƶ�ƽ̨ע��ʧ��
#define NB_ONENET_OBSERVE_REQ_EVENTBIT              (1 << 13)   ///< �ƶ�ƽ̨observe����
#define NB_ONENET_DISCOVER_REQ_EVENTBIT             (1 << 14)   ///< �ƶ�ƽ̨discover����
#define NB_ONENET_UPDATE_SUCCESS_EVENTBIT           (1 << 15)   ///< �ƶ�ƽ̨ע����³ɹ�

///< ��Ӧ�ûص�ͬ�����¼���
extern EventGroupHandle_t   Me3616AppEventGroup;
#define NB_ENTERPSM_EVENTBIT                        (1 << 1)    ///< ģ�����PSM
#define NB_OCEANCONNECT_RECV_EVENTBIT               (1 << 2)   	///< ����ƽ̨���յ�����
#define NB_ONENET_READ_REQ_EVENTBIT                 (1 << 3)   	///< �ƶ�ƽ̨read����
#define NB_ONENET_WRITE_REQ_EVENTBIT                (1 << 4)   	///< �ƶ�ƽ̨write����
#define NB_GNSS_RECE_EVENTBIT                       (1 << 5)   	///< GNSS�ɼ����ݽ���
#define NB_AGPS_DATAREADY_EVENTBIT                  (1 << 6)   	///< AGPS�����������
#define NB_GPS_POSITION_TIMEOUT_EVENTBIT            (1 << 7)   	///< GPS��λ��ʱ
#define NB_GPS_NMEADATA_TIMEOUT_EVENTBIT            (1 << 8)   	///< GPS����NMEA�ϱ����ݳ�ʱ��10s��
#define NB_PSM_ENTER_TIMEOUT_EVENTBIT               (1 << 9)   	///< PSM���볬ʱ

/**@} ME3616�������¼���־�� */



// ʹ�ܱ�־Ϊ�궨��
#define NB_REGIST_ENABLE            1   ///< ��ʼ��ʹ��ƽ̨ע��
#define NB_REGIST_DISABLE           0	///< ��ʼ����ֹƽ̨ע��

#define NB_RRC_ENABLE               1   ///< �ϱ�ʹ��RRC�ͷ�
#define NB_RRC_DISABLE              0	///< �ϱ���ֹRRC�ͷ�

#define NB_UPDATE_ENABLE            1   ///< �ϱ�ʹ�ܸ���ע��
#define NB_UPDATE_DISABLE           0	///< �ϱ���ֹ����ע��

#define NB_REPFAIL_REG_TRY          1	///< �ϱ�ʧ������
#define NB_REPFAIL_REG_DELAY_TRY    2	///< �ϱ�ʧ���ӳ�����
#define NB_REPFAIL_REG_NO_TRY       3	///< �ϱ�ʧ�ܲ����� 


/**
* @brief 16�����ַ���ת�ַ���	\n
* @param[in]  *pSrc    	Դ��ַ
* @param[in]  nLen    	�ַ����ĳ��ȣ�����pSrc�ĳ��ȣ�
* @param[out] *pDest   	Ŀ�ĵ�ַ
* @see 16�����ַ���ת�ַ��� :: HexStrToStr
*/
void StrToHexStr(char *pDest, char *pSrc, uint16_t nLen);

/**
* @brief 16�����ַ���ת�ַ���	\n
* @param[in]  *pSrc    	Դ��ַ
* @param[in]  nLen    	Ҫת���ĳ��ȣ�16�����ַ����ĳ���/2������pDest�ĳ��ȣ�
* @param[out] *pDest   	Ŀ�ĵ�ַ
* @see �ַ���ת16�����ַ��� :: StrToHexStr
*/
void HexStrToStr(char *pDest, char *pSrc, uint16_t nLen);

#endif	/* #if (ME3616_EN) */
#endif	/* __BSP_ME3616_H */


/** @} bsp_me3616*/

/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/





