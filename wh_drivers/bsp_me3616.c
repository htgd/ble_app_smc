/**@file    bsp_me3616.c
* @brief   	ME3616_GPS NB-iot����
* @details  
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-08-17
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

/* Includes ------------------------------------------------------------------*/
#include "bsp_me3616.h"
#if (ME3616_EN)

/* Private variables ---------------------------------------------------------*/
/**
* @name Driverȫ�ֱ�����ṹ��
* @brief �˳������ڶ����ŷ���ָ�������С������Ŀǰ��෢��ָ��ᳬ��128���ֽڣ�
* ��udp��coap������Ϣ���������Ϣ������û���Ҫ���ͳ�������ָ����ԸĴ����ֵ
* @{
*/
//����ȫ�ֻ��������ME3616֧������ͽ���512�ֽڣ�
#define NB_UART_RECE_BUF_MAX_LEN        256			///< NB���ڽ�����󻺴��ֽ���
#define NB_UART_SEND_BUF_MAX_LEN        256			///< NB���ڷ�����󻺴��ֽ���
#define NB_DATA_RECE_BUF_MAX_LEN        256			///< NB����������󻺴��ֽ���
#if (ME3616_GPS_EN) 
#define NB_GPSDATA_RECE_BUF_MAX_LEN     256			///< NB����GPS��������ֽ���
#endif

/**@struct ReceBuf_t
* @brief ���ջ���ռ�
*/
typedef struct
{
    char        Buf[NB_UART_RECE_BUF_MAX_LEN];		///< ���ջ���
    uint16_t  len;                          		///< ��Ч���ݳ���
} ReceBuf_t;

/**@struct SendBuf_t
* @brief ���ͻ���ռ�
*/
typedef struct
{
    char        Buf[NB_UART_SEND_BUF_MAX_LEN];		///< ���ͻ���
    uint16_t    len;                             	///< ��Ч���ݳ���
} SendBuf_t;

/**@struct DataReceBuf_t
* @brief NB�������ݻ���ռ�
*/
typedef struct
{
    char        Buf[NB_DATA_RECE_BUF_MAX_LEN];		///< NB�������ݻ���
    uint16_t    len;                            	///< ��Ч���ݳ���
} DataReceBuf_t;

#if (ME3616_GPS_EN) 
/**@struct GpsDataReceBuf_t
* @brief GPS�������ݻ���ռ�
*/
typedef struct
{
    char        Buf[NB_GPSDATA_RECE_BUF_MAX_LEN];	///< GPS�������ݻ���
    uint16_t    len;                               	///< ��Ч���ݳ���
} GpsDataReceBuf_t;
#endif

/**@struct OnenetMsg_t
* @brief ONENET ָ������ṹ��
*/
typedef struct
{
    char        observemsgid[10];   ///< ƽ̨���� msgid
    char        msgid[10];          ///< ������Ϣ�� message id
    char        objectid[10];       ///< ����� object id
    char        instanceid[2];		///< ����� instance id
    char        resourceid[5];		///< ����� resource id
} OnenetMsg_t;


EventGroupHandle_t   Me3616EventGroup;      ///< ME3616�����¼�ͬ��
EventGroupHandle_t   Me3616AppEventGroup; 	///< ME3616Ӧ�������¼�ͬ��

//==============================================================================
// NB�����û��涨��
static ReceBuf_t            gNBReceBuf;		///< NB���ڽ��ջ���ṹ�嶨��
static SendBuf_t            gNBSendBuf;		///< NB���ڷ��ͻ���ṹ�嶨��
static DataReceBuf_t        gNBDataReceBuf;	///< NBģ�����ݽ��ջ���ṹ�嶨��
#if (ME3616_GPS_EN) 
static GpsDataReceBuf_t     gNBGpsDataReceBuf;	///< NBģ��GPS���ݽ��ջ���ṹ�嶨��
#endif
static OnenetMsg_t          gNBOnenetMsg;	///< NBģ��onenet��Ϣ�嶨��

// ���ڱ�ʾ�����¼���־
ME3616_info_t    g_me3616_info;          	///< NBģ����Ϣ�����ڱ�ʾ�����¼���־����Ϣ
/** @} Driverȫ�ֱ�����ṹ��*/


/* function prototypes ------------------------------------------------------*/
/**@name Driver����ǰ�ö���
* @brief ME3616_FxnTable�ĺ����б�ĺ���ӳ��
* @{
*/
static void me3616_ComOpen(NB_Handle handle);		///< NBģ��ʹ�ô��ڡ���ʱ����ʼ�����ص�ע��
static void me3616_ComClose(NB_Handle handle);		///< NBģ��ʹ�ô��ڹر�
static uint8_t me3616_Set_IotRegist(NB_Handle handle, uint32_t lifetime, uint8_t regist_try);	///< ģ���ʼ�����ü�IOTƽ̨ע�ᣨ��������ע�ᣩ
static uint8_t me3616_init(NB_Handle handle, uint32_t lifetime, uint8_t regist_enabled, uint8_t regist_try);	///< ģ���ʼ����������Ϣ�Ļ�ȡ�����ߵ����ã�

static uint8_t me3616_send_cmd(NB_Handle handle, char *cmd, uint16_t waittime, uint8_t cmd_try);	///< ģ��ָ���
static void me3616_RRC_Release(NB_Handle handle);								///< ģ��RRC�����ͷţ����ڿ��ٽ�������
static uint8_t me3616_psm_set(NB_Handle handle, char *t3412, char *t3324);		///< ģ��PSM���ýӿ�
static uint8_t me3616_getModuleInfo(NB_Handle handle, uint8_t InfoType, uint8_t IfReset, uint8_t err_try);	///< ��ȡģ����Ϣ IMSI��IMEI�� PSM���á� BAND�� �̼��汾��
static uint8_t me3616_getSignal(NB_Handle handle);	///< ��ȡģ���ź�ǿ��

static void me3616_WakeUp(NB_Handle handle);		///< NBģ�黽��
static void me3616_Reset(NB_Handle handle);			///< NBģ�鸴λ
static void me3616_PowerOn(NB_Handle handle);		///< NBģ�鿪��
static void me3616_PowerOff(NB_Handle handle);		///< NBģ��ػ�

// ����ƽ̨oceanconnect��������
static uint8_t me3616_oceanconnect_regist(NB_Handle handle, uint32_t lifetime);	///< oceanconnectƽ̨ע��
static uint8_t me3616_oceanconnect_deregist(NB_Handle handle);					///< oceanconnectƽ̨ȥע��
//static uint8_t me3616_oceanconnect_send(NB_Handle handle, uint8_t *data, uint16_t len);
static uint8_t me3616_oceanconnect_rply(NB_Handle handle, char *cmd);			///< oceanconnectƽ̨Ӧ��

// �ƶ�ƽ̨onenet��������
static uint8_t me3616_onenet_regist(NB_Handle handle, uint32_t lifetime);		///< onenetƽ̨ע��
static uint8_t me3616_onenet_deregist(NB_Handle handle);						///< onenetƽ̨ȥע��
static uint8_t me3616_onenet_update(NB_Handle handle, uint32_t lifetime, uint8_t update_try);	///< onenetƽ̨����ע��

static uint8_t me3616_notify(NB_Handle handle, uint8_t *data, uint16_t len, uint8_t rcc_enabled, uint8_t update_enabled, uint8_t report_fail_try_type);	///< NBģ����Ϣ�ϱ�

#if (ME3616_GPS_EN) 
// GPS��λ��������
APP_TIMER_DEF(m_gps_position_timer_id);        	///< GPS��λ��ʱ��ʱ����120s��λʱ��
static void gps_position_timeout_handler(void * p_context);	///< GPS��λ��ʱ������
APP_TIMER_DEF(m_gpsRevData_timer_id);           ///< GPS������λ�����ϱ���NMEA�ϱ����ݳ�ʱ��ʱ��
static void gpsRevData_timeout_handler(void * p_context);   ///< GPS���ݽ��ճ�ʱ������

static uint8_t me3616_gps_init(NB_Handle handle);	///< ģ��GPS���ܳ�ʼ��
static uint8_t me3616_gps_run(NB_Handle handle);	///< ģ��GPS������λ
static uint8_t me3616_gps_stop(NB_Handle handle);	///< ģ��GPSֹͣ��λ
#endif
/** @} Driver����ǰ�ö���*/


// PSM�ϱ���ʱ����
#if (ME3616_PSM_TIMEOUT_HANDLE_EN)
APP_TIMER_DEF(m_psm_timer_id);        				///< PSM�ϱ���ʱ��ʱ��
static void psm_timeout_handler(void * p_context);	///< PSM�ϱ���ʱ������
#endif

#if (GNT_NB_TEST_EN == 1)
static void me3616_Test(NB_Handle handle);			///< ģ�����
#endif

// �ϱ�����ʱע���ʱ���������ڸ�Ƶ�ϱ�������
APP_TIMER_DEF(m_reportFail_regDelayTry_timer_id); 	///< �ϱ�����ʱע���ʱ����15min��ʱʱ��
static void reportFail_regDelayTry_timeout_handler(void * p_context);	///< �ӻ�ע�ᳬʱ������


static void me3616_event_handle(NB_Handle handle); 	///< ME3616����ȫ���¼�������

#if defined ME3616_LP_TEST
static void me3616_lpTest(NB_Handle handle); 		///< ģ��͹��Ĳ���
#endif

SemaphoreHandle_t Nb_sendCmd_Semaphore;				///< ME3616ָ����������ź���
SemaphoreHandle_t Nb_Notify_Semaphore;				///< ME3616��Ϣ�ϱ������ź���


//******************************************************************************
/**@defgroup bsp_me3616 Bsp me3616 driver module.
* @{
* @ingroup bsp_drivers
*/
///< NB�������ܺ����б�
const NB_FxnTable ME3616_FxnTable = {
    // ͨ�ò�������
    .nbComOpen              = me3616_ComOpen,       ///< NBģ��ʹ�ô��ڡ���ʱ����ʼ�����ص�ע��
    .nbComClose             = me3616_ComClose,      ///< NBģ��ʹ�ô��ڹر�
    .nbSet_IotRegist        = me3616_Set_IotRegist, ///< ģ���ʼ�����ü�IOTƽ̨ע�ᣨ��������ע�ᣩ
    .nbModuleInit           = me3616_init,          ///< ģ���ʼ����������Ϣ�Ļ�ȡ�����ߵ����ã�
 
    .cmdSend                = me3616_send_cmd,      ///< ģ��ָ���
    .nbRrcRelease           = me3616_RRC_Release,	///< ģ��RRC�����ͷţ����ڿ��ٽ�������
    .psmSet                 = me3616_psm_set,       ///< ģ��PSM���ýӿ�
    .getModuleInfo          = me3616_getModuleInfo, ///< ��ȡģ����Ϣ IMSI��IMEI�� PSM���á� BAND�� �̼��汾��
    .getSign                = me3616_getSignal,		///< ��ȡ�ź�ǿ��

    .nbWakeUp               = me3616_WakeUp,		///< ģ�黽��
    .nbReset                = me3616_Reset,         ///< ģ�鸴λ
    .nbPowerOn              = me3616_PowerOn,       ///< ģ���ϵ�
    .nbPowerOff             = me3616_PowerOff,      ///< ģ��ػ�
    
    // ����ƽ̨oceanconnect��������
    .oceanconnectRegist     = me3616_oceanconnect_regist,	///< oceanconnectƽ̨ע��
    .oceanconnectDeregist   = me3616_oceanconnect_deregist,	///< oceanconnectƽ̨ȥע��
//    .oceanconnectSend       = me3616_oceanconnect_send,
    .oceanconnectRply       = me3616_oceanconnect_rply,		///< oceanconnectƽ̨Ӧ��
    
    // �ƶ�ƽ̨onenet��������
    .onenetRegist           = me3616_onenet_regist,			///< onenetƽ̨ע��
    .onenetDeregist         = me3616_onenet_deregist,		///< onenetƽ̨ȥע��
//    .onenetNotify           = me3616_onenet_notify,
    .onenetUpdate           = me3616_onenet_update,			///< onenetƽ̨����ע��
    
    .nbNotify               = me3616_notify,				///< NBģ����Ϣ�ϱ�
    
#if (ME3616_GPS_EN) 
    // GNSS��λ��������
    .gpsInit                = me3616_gps_init,				///< ģ��GPS���ܳ�ʼ��
    .gpsRun                 = me3616_gps_run,				///< ģ��GPS������λ
    .gpsStop                = me3616_gps_stop, 				///< ģ��GPSֹͣ��λ
#endif
    
    // ģ�����¼������������ݲ�����������������ѭ��
    .eventHandle            = me3616_event_handle,			///< ME3616����ȫ���¼�������
    
    // �͹��Ĳ���
    #if defined ME3616_LP_TEST
    .lpTest                 = me3616_lpTest,
    #endif
    
    #if (GNT_NB_TEST_EN == 1)
    .nbTest                 = me3616_Test,
    #endif
};
/** @} bsp_me3616*/

/* function prototypes ------------------------------------------------------*/
static void me3616_receCb(char* buf, uint16_t len);   	///< ���ڽ������ݻص�
//static void me3616_timeoutCb(void);                   ///< ��ʱ����ʱ�ص�
//static void nbsend_msg_app(NB_Handle handle, uint8_t**buf,uint8_t isOk);   ///< Ӧ�ûص���������Ӧ�ò㷢��ָ�����


#if (GNT_NB_TEST_EN == 1)
static void me3616_Test(NB_Handle handle)
{}
#endif


//******************************************************************************
/**@brief ��me3616ʹ�õ�uart����
* @param[in]  handle 			NBģ���������
* @see :: ME3616_FxnTable :: me3616_ComClose
*/
static void me3616_ComOpen(NB_Handle handle)
{
	if(g_me3616_info.me3616_comOpen_flag == 0)
	{
		//ע�ᴮ�ڽ��ջص�����
		handle->hw_fxnTablePtr->com_openFxn(me3616_receCb, handle->hw_fxnTablePtr->baudrate);
		g_me3616_info.me3616_comOpen_flag = 1;
		//ע�ᶨʱ����ʱ�ص�����
//		handle->hw_fxnTablePtr->initTimerFxn(me3616_timeoutCb);
		NRF_LOG_INFO("uart open");
	}
}

/**
* @brief �ر�me3616ʹ�õ�uart����
* @param[in]  handle 			NBģ���������
* @see :: ME3616_FxnTable :: me3616_ComOpen
*/
static void me3616_ComClose(NB_Handle handle)
{
	if(g_me3616_info.me3616_comOpen_flag == 1)
	{
		handle->hw_fxnTablePtr->com_closeFxn();
//	    nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_PULLUP);
//	    nrf_gpio_cfg_input(TX_PIN_NUMBER, NRF_GPIO_PIN_PULLUP);
		g_me3616_info.me3616_comOpen_flag = 0;
		NRF_LOG_INFO("uart close");
	}
}

/**
* @brief me3616����ƽ̨ע�� \n
* ģ�鵥�θ�λ����ʼ����ע��
* @param[in]  handle 			NBģ���������
* @param[in]  lifetime 			������ƽ̨����ʱ��
* @param[in]  IfReset 			ע���Ƿ�λ��0������λ��1����λ
* @return  ����ָ��ִ�н��
* - NB_IOT_REGIST_SUCCESS  	 ִ�гɹ�
* - NB_IOT_REGIST_FAILED     ִ��ʧ��
* @see :: me3616_Set_IotRegist
*/
static uint8_t me3616_Set_IotRegist_once(NB_Handle handle, uint32_t lifetime, uint8_t IfReset)
{
    uint8_t ret;
    EventBits_t uxBits;
    
    if(g_me3616_info.me3616_comOpen_flag == 0)	//�������PSM���´��ڹر�
        me3616_ComOpen(handle);
    
nb_restart: 
//    memset(&g_me3616_info, 0, sizeof(ME3616_info_t) );  //����Ϣ��״̬
    
    if(IfReset == 1)
    {
        xEventGroupClearBits(Me3616EventGroup, NB_GETIP_EVENTBIT);
        g_me3616_info.me3616_getip = 0;
        g_me3616_info.me3616_band = 0;
        g_me3616_info.me3616_psmisset = 0;
        g_me3616_info.me3616_zslr = 0;
        
        g_me3616_info.me3616_enterPSM_flag = 0;
        g_me3616_info.iot_regist_status = 0;
        g_me3616_info.me3616_register_status = 0;
        #if (ME3616_GPS_EN) 
        g_me3616_info.gnss_run_state = 0;
        #endif
        
        me3616_Reset(handle);   //Ӳ�����Ÿ�λ
        NRF_LOG_INFO("ME3616 reset over");
    }
	
	// �ȴ��Զ���ȡIP(��ʱ�ȴ�60s)
    uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_GETIP_EVENTBIT, pdTRUE, pdFALSE, 60000);
	
    if((uxBits & NB_GETIP_EVENTBIT) == NB_GETIP_EVENTBIT)
    {
        g_me3616_info.me3616_register_status = 1;   //��վ���總��״̬
        NRF_LOG_INFO("NB get ip OK");
    }
    else
    {
        NRF_LOG_INFO("NB get ip timeout");
        return NB_IOT_REGIST_FAILED;
    }
    
    if(IfReset == 1)
        me3616_send_cmd(handle, "ATE0", 2000, 2);       //��ֹ����
    
//******************************��ȡģ�������Ϣ************************************************* 
    if(NB_ACK_OK != handle->nb_fxnTablePtr->getModuleInfo(handle, 2, 0, NULL))   // ��ȡ�豸��Ϣ
        return NB_IOT_REGIST_FAILED;
    
//***********************************�͹�������************************************************* 
    if(g_me3616_info.me3616_psmisset == 0)  //PSMû�����ã�����PSM ��TAU = 6Сʱ  Active-Time = 30s��
    {
        ret = handle->nb_fxnTablePtr->psmSet(handle, "00100110", "00001010");
        if(ret == NB_ACK_OK)
        {
            NRF_LOG_INFO("PSM set OK");
        }
        else
        {
            NRF_LOG_INFO("PSM set ERROR %d", ret);
            return NB_IOT_REGIST_FAILED;
        }
    }
    me3616_send_cmd(handle, "AT+CEDRXS=0", 1000, 1);
    
    if(g_me3616_info.me3616_zslr == 0)  //���߿���û�д򿪣���������
    {
        ret = me3616_send_cmd(handle, "AT+ZSLR=1", 2000, 1);    //����ָ������ģ����Ч
        ret = me3616_send_cmd(handle, "AT+SETWAKETIME=20", 2000, 2);
        if(ret == NB_ACK_OK)
        {
            NRF_LOG_INFO("ZSLR set OK");
            IfReset = 1;    //������
            goto nb_restart;
        }
        else
        {
            NRF_LOG_INFO("ZSLR set ERROR %d", ret);
            return NB_IOT_REGIST_FAILED;
        }
    }
    
    ret = me3616_send_cmd(handle, "AT*MNBIOTEVENT=1,1", 2000, 2); //ʹ��PSM״̬�����ϱ�
    
    // NB����ģʽ������ע��
    #if (GNT_NB_TEST_EN == 1)
    return NB_IOT_REGIST_SUCCESS;
    #endif
    
//***********************************ƽ̨ע��*************************************************    
    // ����Oceanconnectƽ̨ע�� AT+M2MCLINEW=180.101.147.115,5683,"868613030006275",90
    if(g_me3616_info.me3616_iot_platform == ME3616_IOT_PLATFORM_OCEANCONNECT)
    {
        ret = handle->nb_fxnTablePtr->oceanconnectRegist(handle, lifetime);
        if(ret == NB_IOT_REGIST_SUCCESS)
        {
            NRF_LOG_INFO("Oceanconnect regist success");
        }
        else
        {
            NRF_LOG_INFO("Oceanconnect regist failed ->%d", ret);
            return NB_IOT_REGIST_FAILED;
        }
    }
    else if(g_me3616_info.me3616_iot_platform == ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT)
    {
        if((g_me3616_info.me3616_band == BAND_5) || (g_me3616_info.me3616_band == BAND_3))
        {
            #if (OCEANCONNECT_DATA_MODE == TEXT_MODE)
    //        if(g_me3616_info.oceanconnect_data_mode == HEX_MODE)
    //        {
    //            if(NB_ACK_OK != me3616_send_cmd(handle, "AT+M2MCLICFG=1,1", 2000, 2))
    //                return NB_IOT_REGIST_FAILED;
    //        }
            #elif (OCEANCONNECT_DATA_MODE == HEX_MODE)
    //        if(g_me3616_info.oceanconnect_data_mode == TEXT_MODE)
    //        {
    //            if(NB_ACK_OK != me3616_send_cmd(handle, "AT+M2MCLICFG=0,0", 2000, 2))
    //                    return NB_IOT_REGIST_FAILED;
    //        }
            #endif
            
            ret = handle->nb_fxnTablePtr->oceanconnectRegist(handle, lifetime);
            if(ret == NB_IOT_REGIST_SUCCESS)
            {
                NRF_LOG_INFO("Oceanconnect regist success");
            }
            else
            {
                NRF_LOG_INFO("Oceanconnect regist failed ->%d", ret);
            }
        
    //        handle->nb_fxnTablePtr->oceanconnectSend(handle, NULL, 0);
        }
        else if(g_me3616_info.me3616_band == BAND_8) // �ƶ�OneNetƽ̨ע��
        {
            ret = handle->nb_fxnTablePtr->onenetRegist(handle, lifetime);
            if(ret == NB_IOT_REGIST_SUCCESS)
            {
                NRF_LOG_INFO("onenet regist success");
            }
            else
            {
                NRF_LOG_INFO("onenet regist failed ->%d", ret);
            }
        }
        else
        {
            return NB_IOT_REGIST_FAILED;
        }
    }
    return ret; 
}

/**@brief me3616ƽ̨ע�� \n
* me3616��λ����ʼ�����á�ƽ̨ע��
* @param[in]  handle 			NBģ���������
* @param[in]  lifetime 			������ƽ̨����ʱ��
* @param[in]  regist_try 		ע��������Դ���
* @return  ����ָ��ִ�н��
* - NB_IOT_REGIST_SUCCESS  	 ִ�гɹ�
* - NB_IOT_REGIST_FAILED     ִ��ʧ��
* @see :: ME3616_FxnTable :: me3616_init
*/
static uint8_t me3616_Set_IotRegist(NB_Handle handle, uint32_t lifetime, uint8_t regist_try)
{
    uint8_t ret;
    g_me3616_info.iot_regist_status = 2;    //ע����
    
//    vTaskDelay(5000);	//���ο������踴λ���ʱ����
    ret = me3616_Set_IotRegist_once(handle, lifetime, 1);   //�״�ע�᲻��λ
    if( ret != NB_IOT_REGIST_SUCCESS )   
    {
		#if (ME3616_SIGNAL_BAD_HANDLE_EN)
		if(g_me3616_info.me3616_rssi > ME3616_SIGNAL_BAD_THRESHOLD)	//�źŽϲ<10�������ӵ���ע��ʱ�䣬����ע�����
		{
			regist_try = ME3616_SIGNALBAD_REG_TRY_TIMES;
		}
		#endif
        do{
            ret = me3616_Set_IotRegist_once(handle, lifetime, 1);   //��ע�Ḵλ
        }while ( (regist_try--) && (ret != NB_IOT_REGIST_SUCCESS) );
    }
    
    if(ret == NB_IOT_REGIST_SUCCESS)
    {
        g_me3616_info.iot_regist_status = 1;    //ƽ̨ע��ɹ���־
    }
    else
        g_me3616_info.iot_regist_status = 0;    //δע���ע��ʧ��
    return ret;
}

/**@brief me3616������ʼ��
* @param[in]  handle 			NBģ���������
* @param[in]  lifetime 			������ƽ̨����ʱ��
* @param[in]  regist_enabled 	�Ƿ�ʹ��ƽ̨������ǩ�Ƿ�ʹ�ܣ�
* @param[in]  regist_try 		ע��������Դ���
* @return  ����ָ��ִ�н��
* - NB_ACK_OK  	 ִ�гɹ�
* - NB_ACK_ERROR ִ�д���
* - NB_ACK_TIMEOUT ִ�г�ʱ
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_init(NB_Handle handle, uint32_t lifetime, uint8_t regist_enabled, uint8_t regist_try)
{
    uint8_t ret;
    // ME3616ģ��ʹ��IO��ʼ��
    nrf_gpio_cfg_output(NB_PWRON);  //����NB_PWRON���
    nrf_gpio_pin_set(NB_PWRON);
    nrf_gpio_cfg_output(NB_RESET);
    nrf_gpio_pin_set(NB_RESET);
//    nrf_gpio_cfg_output(NB_WKUP);
//    nrf_gpio_pin_set(NB_WKUP);
    
    Me3616EventGroup = xEventGroupCreate();         //me3616����ʹ���¼��� (configUSE_16_BIT_TICKSΪ0  �¼���24λ)
	Me3616AppEventGroup = xEventGroupCreate();    	//me3616Ӧ������ʹ���¼��飨�����¼�λ�϶���ֿ�ʹ�ã�
    vSemaphoreCreateBinary(Nb_sendCmd_Semaphore);   //NBָ����������ź���
    vSemaphoreCreateBinary(Nb_Notify_Semaphore);    //֪ͨ��
    
    //�ϱ�����ʱע���ʱ����15min��ʱʱ��
    app_timer_create(&m_reportFail_regDelayTry_timer_id, APP_TIMER_MODE_SINGLE_SHOT, reportFail_regDelayTry_timeout_handler);
	
	#if (ME3616_PSM_TIMEOUT_HANDLE_EN)	//PSM�ϱ���ʱ����
	app_timer_create(&m_psm_timer_id, APP_TIMER_MODE_SINGLE_SHOT, psm_timeout_handler);
	#endif
    
    me3616_ComOpen(handle); //������ʼ��ʹ�ܴ���
    g_me3616_info.regist_lifetime = lifetime;
    xEventGroupClearBits(Me3616EventGroup, NB_CPIN_EVENTBIT|NB_GETIP_EVENTBIT);
    me3616_PowerOn(handle);     //����������NB_PWRON����1s
    if(regist_enabled == NB_REGIST_DISABLE) //��ǩû��ʹ�ܣ���ע�����ȡģ�������Ϣ
    {
        // �ȴ�ģ��ͨ�š�������������
        EventBits_t uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_CPIN_EVENTBIT, pdTRUE, pdFALSE, 10000);
        if((uxBits & NB_CPIN_EVENTBIT) == NB_CPIN_EVENTBIT)
        {
            NRF_LOG_INFO("CPIN ready");
            if(NB_ACK_OK != me3616_send_cmd(handle, "ATE0", 3000, 1))   //��ֹ����
            {
                ret = handle->nb_fxnTablePtr->getModuleInfo(handle, 1, 1, 5);
            }
            else
            {
                // ��ȡģ�������Ϣ
                if(NB_ACK_OK != handle->nb_fxnTablePtr->getModuleInfo(handle, 1, 0, NULL))
                {
                    ret = handle->nb_fxnTablePtr->getModuleInfo(handle, 1, 1, 5);
                }
            }
        }
        else
        {
            ret = handle->nb_fxnTablePtr->getModuleInfo(handle, 1, 1, 5);
        }
    }
    else if(regist_enabled == NB_REGIST_ENABLE)    //��ǩʹ�ܣ�ע����ƽ̨
    {
        vTaskDelay(5000);	//���ο������踴λ���ʱ����
        ret = me3616_Set_IotRegist(handle, lifetime, regist_try);
    } 
    return ret;
}


/**@brief NBģ�黽��
* @param[in]  handle 	NBģ���������
* @see :: ME3616_FxnTable
*/
static void me3616_WakeUp(NB_Handle handle)
{
    me3616_ComOpen(handle);	//������ڹرգ���Ҫ�򿪴���
    
    if(NB_ACK_OK == me3616_send_cmd(handle, "AT", 1000, 1)) //�ѻ���
    {
        /* ģ���ѻ��� */
    }
    else    //ͨ��NB_PWRON����
    {
        xEventGroupClearBits(Me3616EventGroup, NB_CPIN_EVENTBIT|NB_ONENET_CONNECT_SUCCESS_EVENTBIT);
        nrf_gpio_pin_write(NB_PWRON, 0);
        vTaskDelay(1000);
        nrf_gpio_pin_write(NB_PWRON, 1);
        NRF_LOG_INFO("NB_PWRON WKUP");
        
        EventBits_t uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_CPIN_EVENTBIT, pdTRUE, pdFALSE, 6000); 
        if((uxBits & NB_CPIN_EVENTBIT) == NB_CPIN_EVENTBIT)
        {
            if((g_me3616_info.me3616_band == BAND_8) && (g_me3616_info.me3616_iot_platform == ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT))
            {
                uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_ONENET_CONNECT_SUCCESS_EVENTBIT, pdTRUE, pdFALSE, 10000);
                if((uxBits & NB_ONENET_CONNECT_SUCCESS_EVENTBIT) == NB_ONENET_CONNECT_SUCCESS_EVENTBIT)
                    NRF_LOG_INFO("ONENET_CONNECT_SUCCESS");
            }
            else    // ME3616_IOT_PLATFORM_OCEANCONNECT
            {
                vTaskDelay(5000);   // ����ʱָ������
//                me3616_send_cmd(handle, "AT+M2MCLISEND=00", 1000, 1);   // ���Ϳ������˳�PSM
//                me3616_send_cmd(handle, "AT", 1000, 2)
            }
        }
    }
}

/**@brief NBģ�鸴λ
* @param[in]  handle 	NBģ���������
* @see :: ME3616_FxnTable
*/
static void me3616_Reset(NB_Handle handle)
{
    me3616_WakeUp(handle);      		//����״̬�²���reset
    NRF_LOG_INFO("me3616 reset");
    nrf_gpio_pin_write(NB_RESET, 0);    //Ӳ�����Ÿ�λ
    vTaskDelay(500);
    nrf_gpio_pin_write(NB_RESET, 1);
    g_me3616_info.me3616_powerOn_flag = 1;
}

/**@brief NBģ�鿪��
* @param[in]  handle 	NBģ���������
* @see :: ME3616_FxnTable :: me3616_PowerOff
*/
static void me3616_PowerOn(NB_Handle handle)
{
    nrf_gpio_pin_write(NB_PWRON, 0);
    vTaskDelay(1000);
    nrf_gpio_pin_write(NB_PWRON, 1);
    g_me3616_info.me3616_powerOn_flag = 1;
    NRF_LOG_INFO("me3616_PowerOn");
}

/**@brief NBģ��ػ�
* @param[in]  handle 	NBģ���������
* @see :: ME3616_FxnTable :: me3616_PowerOn
*/
static void me3616_PowerOff(NB_Handle handle)
{
    #if 0
    me3616_WakeUp(handle);
    if(NB_ACK_OK != me3616_send_cmd(handle, "AT+ZTURNOFF", 1000, 2))
    {
        // ��ػ�ʧ�ܲ���Ӳ���ػ�
        nrf_gpio_pin_write(NB_PWRON, 0);
        vTaskDelay(5000);
        nrf_gpio_pin_write(NB_PWRON, 1);
    }
    #endif
    NRF_LOG_INFO("me3616_PowerOff");
    nrf_gpio_pin_write(NB_PWRON, 0);
    vTaskDelay(1000);
    nrf_gpio_pin_write(NB_PWRON, 1);
	vTaskDelay(1000);
    nrf_gpio_pin_write(NB_PWRON, 0);
    vTaskDelay(5000);
    nrf_gpio_pin_write(NB_PWRON, 1);
    
    g_me3616_info.me3616_powerOn_flag = 0;
    NRF_LOG_INFO("me3616_PowerOff over");
}

/**@brief NBģ�������ͷ�RRC����
* @param[in]  handle 	NBģ���������
* @see :: ME3616_FxnTable
*/
static void me3616_RRC_Release(NB_Handle handle)
{
    me3616_send_cmd(handle, "AT*MNBIOTRAI=1", 2000, 1);
}


/**@brief ME3616��PSM���ã��Ĳ������籣��(�ڲ�����)
* @param[in]  handle 	NBģ���������
* @param[in]  t3412 	TAU��ʱ��
* @param[in]  t3324 	Active��ʱ��
* @return  ����ָ��ִ�н��
* - NB_ACK_OK  	 ִ�гɹ�
* - NB_ACK_ERROR ִ�д���
* - NB_ACK_TIMEOUT ִ�г�ʱ
* @par ʾ��:
* @code
*	// TAU = 6Сʱ  Active-Time = 30s
*	ret = handle->nb_fxnTablePtr->psmSet(handle, "00100110", "00001010");
* @endcode
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_psm_set(NB_Handle handle, char *t3412, char *t3324)
{
    uint8_t ret;
    char psm_set_s[40] = "AT+CPSMS=1,,,\"";
    
    if( (strlen(t3412)!=8) || (strlen(t3324)!=8) )
    {
        return NB_CMD_ERROR;
    }
    
    strcat(psm_set_s, t3412);
    strcat(psm_set_s, "\",\"");
    strcat(psm_set_s, t3324);
    strcat(psm_set_s, "\"");
    
    ret = me3616_send_cmd(handle, psm_set_s, 2000, 1); 
    return ret;
}

/**@brief ��ȡģ����Ϣ IMSI��IMEI�� PSM���á� BAND�� �̼��汾��(�ڲ�����)
* @param[in]  handle 	NBģ���������
* @param[in]  InfoType 	��ȡ��Ϣ������
* - 1����IMEI��IMSI
* - 2��PSM��ZSLR��BAND��GMR(��������)
* @return  ����ָ��ִ�н��
* - NB_ACK_OK  	 ִ�гɹ�
* - NB_ACK_ERROR ִ�д���
* - NB_ACK_TIMEOUT ִ�г�ʱ
* @see :: me3616_getNbInfo
*/
static uint8_t me3616_getNbInfo(NB_Handle handle, uint8_t InfoType)
{
    uint8_t ret;
    // ��ѯ�����ƶ�̨�豸��ʶIMSI
//    ret = me3616_send_cmd(handle, "ATI", 2000, 0);

	// ��ѯ��Ʒ���к�IMEI   869662030455593
    ret = me3616_send_cmd(handle, "AT+GSN", 2000, 1);
    if(ret == NB_ACK_OK)
    {
		memcpy(g_me3616_info.me3616_IMEI, gNBReceBuf.Buf, 15);
		g_me3616_info.me3616_IMEI[15] = '\0';
    }
    else
    {
        return NB_ACK_ERROR;
    }
    // ��ѯ�����ƶ�̨�豸��ʶIMSI  460111172164930
    ret = me3616_send_cmd(handle, "AT+CIMI", 2000, 1);
    if(ret == NB_ACK_OK)
    {
		memcpy(g_me3616_info.me3616_IMSI, gNBReceBuf.Buf, 15);
		g_me3616_info.me3616_IMSI[15] = '\0';
    }
    else
    {
        return NB_ACK_ERROR;
    }

    if(InfoType == 2)
    {
        // ��ѯPSM�Ƿ�����  �ظ�  +CPSMS: 0
        ret = me3616_send_cmd(handle, "AT+CPSMS?", 2000, 1);
        if(ret == NB_ACK_OK)
        {
            char* pColon = strchr(gNBReceBuf.Buf,':'); 
            if(pColon)
            {
                pColon = pColon+2;
                g_me3616_info.me3616_psmisset = (*pColon - 0x30);    //����ֲ�˵��
            }
        }
        // ��ѯ���߹����Ƿ��� +ZSLR:0
        ret = me3616_send_cmd(handle, "AT+ZSLR?", 2000, 1);
        if(ret == NB_ACK_OK)
        {
            char* pColon = strchr(gNBReceBuf.Buf,':'); 
            if(pColon)
            {
                pColon = pColon+1;
                g_me3616_info.me3616_zslr = (*pColon - 0x30);    //����ֲ�˵��
            }
        }
        // ��ѯBAND  *MBAND: 5
        ret = me3616_send_cmd(handle, "AT*MBAND?", 2000, 1);
        if(ret == NB_ACK_OK)
        {
            char* pColon = strchr(gNBReceBuf.Buf,':'); 
            if(pColon)
            {
                pColon = pColon+2;
                g_me3616_info.me3616_band = (*pColon - 0x30);    //����ֲ�˵��
            }
//            if((g_me3616_info.me3616_band!=5)&&(g_me3616_info.me3616_band!=8)&&(g_me3616_info.me3616_band!=3))
//                return NB_ACK_ERROR;
        }
        else
            return NB_ACK_ERROR;
        
        // ��ѯģ��̼��汾��  ME3616G1AV0.0B01
        ret = me3616_send_cmd(handle, "AT+GMR", 2000, 1);
        if(ret == NB_ACK_OK)
        {
            memcpy(g_me3616_info.me3616_SwRevision, gNBReceBuf.Buf, 16);
        }
        // ��ѯ�ź�ǿ��
        ret = me3616_send_cmd(handle, "AT+CSQ", 2000, 1);
        if(ret == NB_ACK_OK)
        {
            char* pColon = strchr(gNBReceBuf.Buf,':'); 
            if(pColon)
            {
                pColon = pColon+2;
                g_me3616_info.me3616_rssi = (uint8_t)strtol(pColon, NULL, 10);   //��ֵת������ֲ�˵��
            }
        }
        
        // ����ƽ̨���ݷ��ͺ��ϱ�ģʽ���� +M2MCLICFG: 0,0
//        if(g_me3616_info.me3616_band == 5)
//        {
//            ret = me3616_send_cmd(handle, "AT+M2MCLICFG?", 2000, 1);
//            if(ret == NB_ACK_OK)
//            {
//                char* pColon = strchr(gNBReceBuf.Buf,',');
//                if(pColon)
//                {
//                    pColon = pColon+1;
//                    g_me3616_info.oceanconnect_data_mode = (*pColon - 0x30);    //����ֲ�˵��
//                }
//                if((g_me3616_info.oceanconnect_data_mode!=0)&&(g_me3616_info.oceanconnect_data_mode!=1))
//                    return NB_ACK_ERROR;
//            }
//            else
//                return NB_ACK_ERROR;
//        }
    }
    return ret;
}

/**@brief ��ȡģ����Ϣ \n
* ��Ϣ������IMSI��IMEI�� PSM���á� BAND�� �̼��汾��
* @param[in]  handle 	NBģ���������
* @param[in]  InfoType 	��ȡ��Ϣ������
* - 1����IMEI��IMSI
* - 2��PSM��ZSLR��BAND��GMR(��������)
* @param[in]  IfReset 	�Ƿ���Ҫ����  0�������������Դ�����Ч���� 1������
* @param[in]  err_try 	�������Դ���
* @return  ����ָ��ִ�н��
* - NB_ACK_OK  	 ִ�гɹ�
* - NB_ACK_ERROR ִ�д���
* - NB_ACK_TIMEOUT ִ�г�ʱ
* @see :: ME3616_FxnTable :: me3616_getNbInfo
*/
static uint8_t me3616_getModuleInfo(NB_Handle handle, uint8_t InfoType, uint8_t IfReset, uint8_t err_try)
{
    uint8_t ret;
    if(IfReset == 0)
    {
        ret = me3616_getNbInfo(handle, InfoType);
        if(ret == NB_ACK_OK)
            g_me3616_info.me3616_info_collection_flag = 1;
    }
    else if( (IfReset == 1) && (InfoType == 1) )
    {
        do{
        
            xEventGroupClearBits(Me3616EventGroup, NB_CPIN_EVENTBIT);
            me3616_Reset(handle);   //Ӳ�����Ÿ�λ
            NRF_LOG_INFO("ME3616 reset over");
            xEventGroupWaitBits(Me3616EventGroup, NB_CPIN_EVENTBIT, pdTRUE, pdFALSE, 10000);
            ret = me3616_send_cmd(handle, "ATE0", 3000, 1); //��ֹ����
            if(ret == NB_ACK_OK)
            {
                ret = me3616_getNbInfo(handle, InfoType);   // ��ȡģ�������Ϣ
            }
        }while ( (err_try--) && (ret != NB_ACK_OK) );
        
        if(ret == NB_ACK_OK)
            g_me3616_info.me3616_info_collection_flag = 1;
    }
    return ret;    
}

/**@brief ��ȡģ��RSSI�ź�ǿ��
* @param[in]  handle 	NBģ���������
* @return  ����ָ��ִ�н��
* - NB_ACK_OK  	 ִ�гɹ�
* - NB_ACK_ERROR ִ�д���
* - NB_ACK_TIMEOUT ִ�г�ʱ
* @par ʾ��:
* @code
*	AT+CSQ
* @endcode
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_getSignal(NB_Handle handle)
{
    uint8_t ret;
	uint8_t try_times = 0;
	
    me3616_WakeUp(handle);
//	vTaskDelay(1000);
	if(g_me3616_info.me3616_enterPSM_flag == 1)
	{
		me3616_send_cmd(handle, "AT+M2MCLISEND=00", 3000, 2);   // ���Ϳ������˳�PSM
	}
	do{
		// ��ѯ�ź�ǿ�� +CSQ: 17,99
		ret = me3616_send_cmd(handle, "AT+CSQ", 2000, 1);
		if(ret == NB_ACK_OK)
		{
			char* pColon = strchr(gNBReceBuf.Buf,':'); 
			if(pColon)
			{
				pColon = pColon+2;
				g_me3616_info.me3616_rssi = (uint8_t)strtol(pColon, NULL, 10);   //��ֵת������ֲ�˵��
				if(g_me3616_info.me3616_rssi < 32)
				{
					g_me3616_info.me3616_rssi = 113 - (g_me3616_info.me3616_rssi*2);
				}
				else
				{
					g_me3616_info.me3616_rssi = 255;	//û���ź�
					vTaskDelay(1000);
				}
			}
		}
		else
		{
			vTaskDelay(1000);
		}
		try_times++;
	}while((try_times < ME3616_SIGNAL_TRY_TIMES) && (g_me3616_info.me3616_rssi > 113));
    return ret;
}


/**@brief ME3616����ָ��ͺ���
* @param[in]  handle 	NBģ���������
* @param[in]  cmd 		ָ�������ַ���ָ��
* @param[in]  waittime 	ָ����Ӧ��ʱʱ�䣬��λms
* @return  ����ָ��ִ�н��
* - NB_ACK_OK  	 ִ�гɹ�
* - NB_ACK_ERROR ִ�д���
* - NB_ACK_TIMEOUT ִ�г�ʱ
* @see :: me3616_send_cmd
*/
static uint8_t me3616_send_cmd_once(NB_Handle handle, char *cmd, uint16_t waittime)
{
    EventBits_t uxBits;
    
    memset(&gNBReceBuf, 0, sizeof(ReceBuf_t)); //�������
    // �ַ���ָ�벻����sizeof�� �ַ����Ƽ�ʹ��strlen������ͬʱʹ��snprintfʱ���ȼǵ�+1
    snprintf(gNBSendBuf.Buf, strlen((char*)cmd)+3, "%s\r\n", cmd); //��ӽ���λ\r\n
    
    xEventGroupClearBits(Me3616EventGroup, NB_ACK_OK_EVENTBIT|NB_ACK_ERROR_EVENTBIT);
    handle->hw_fxnTablePtr->com_sendFxn((uint8_t*)gNBSendBuf.Buf, strlen((char*)cmd)+2);   //����ָ��
    NRF_LOG_INFO("%s", gNBSendBuf.Buf); //����������
//    handle->hw_fxnTablePtr->startTimerFxn(waittime);    //�������ʱ��ʱ��
    uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_ACK_OK_EVENTBIT|NB_ACK_ERROR_EVENTBIT, pdTRUE, pdFALSE, waittime);
    if((uxBits & NB_ACK_OK_EVENTBIT) == NB_ACK_OK_EVENTBIT)
    {
        return NB_ACK_OK;
    }
    else if((uxBits & NB_ACK_ERROR_EVENTBIT) == NB_ACK_ERROR_EVENTBIT)
    {
        return NB_ACK_ERROR;
    }

    return NB_ACK_TIMEOUT;
}

/**@brief ME3616ָ��ͺ���
* @param[in]  handle 	NBģ���������
* @param[in]  cmd 		ָ�������ַ���ָ��
* @param[in]  waittime 	ָ����Ӧ��ʱʱ�䣬��λms
* @param[in]  cmd_try 	�������Դ���
* @return  ����ָ��ִ�н��
* - NB_ACK_OK  	 ִ�гɹ�
* - NB_ACK_ERROR ִ�д���
* - NB_ACK_TIMEOUT ִ�г�ʱ
* @see :: ME3616_FxnTable :: me3616_send_cmd_once
*/
static uint8_t me3616_send_cmd(NB_Handle handle, char *cmd, uint16_t waittime, uint8_t cmd_try)
{
    uint8_t cmd_result;
    uint8_t cmd_tried = 0;
    xSemaphoreTake( Nb_sendCmd_Semaphore,  portMAX_DELAY);
    do{
        cmd_result = me3616_send_cmd_once(handle, cmd, waittime*(cmd_tried+1));
        cmd_tried++;
    }while( (cmd_result != NB_ACK_OK) && (cmd_tried < (cmd_try+1)) );
    xSemaphoreGive( Nb_sendCmd_Semaphore );
    return cmd_result;
}


//******************************************************************************
/**@brief ����ME3616�첽���ص�֪ͨ
* @param[in]  buf 	��Ϣ����ָ��
* @param[in]  len 	��Ϣ����
* @return 
* - 0  ���첽֪ͨ
* - !0 �����첽֪ͨ
* @see :: me3616_receCb
*/
static uint8_t me3616_AsyncNotification(char* buf, uint16_t len)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t isAsync = false;
    char* position_addr_start = NULL;   //�ַ�����ʼ��ַ 
    
    if(g_me3616_info.me3616_getip == 0) //ME3616�Զ�������û�����
    {
        if((position_addr_start = strstr(buf, "*MATREADY: 1")) != NULL) //����׼���ã����շ�AT
        {
            g_me3616_info.me3616_matready = 1;
            return true;
        }
        if((position_addr_start = strstr(buf, "+CFUN: 1")) != NULL)     //��Ƶ����
        {
            g_me3616_info.me3616_cfun = 1;
            return true;
        }
        if((position_addr_start = strstr(buf, "+CPIN: READY")) != NULL) //������
        {
            if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_CPIN_EVENTBIT, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            g_me3616_info.me3616_cpin = 1;
            return true;
        }
        if((position_addr_start = strstr(buf, "+IP:")) != NULL)         //�Զ���ȡIP ��ַ
        {
            if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_GETIP_EVENTBIT, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            g_me3616_info.me3616_getip = 1;
            return true;
        } 
    }
    else    //�������֮����첽֪ͨ����
    {
        // +CPIN: READY
        if((position_addr_start = strstr(buf, "+CPIN: READY")) != NULL) //�����˳�PSM
        {
            if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_CPIN_EVENTBIT, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            return true;
        }
//        if((position_addr_start = strstr(buf, "+IP:")) != NULL)         //�Զ���ȡIP ��ַ
//        {
//            if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_GETIP_EVENTBIT, &xHigherPriorityTaskWoken))
//            {
//                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//            }
//            g_me3616_info.me3616_getip = 1;
//            return true;
//        }
        // +CEREG: 0,1
//        if((position_addr_start = strstr(buf, "+CEREG")) != NULL)       //����ע��״̬����
//        {
//            char* pColon = strchr(buf,','); //,1���������ע��״̬
//            if(pColon)
//            {
//                pColon++;
//                g_me3616_info.me3616_register_status = (*pColon - 0x30);    //����ֲ�˵��
//            }
//            
//            return true;
//        }
        
        
        //�ƶ�onenetƽ̨
        if((g_me3616_info.me3616_band == BAND_8) && (g_me3616_info.me3616_iot_platform == ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT))
        {
            // +MIPLEVENT ģ��״̬�ϱ�    
            if((position_addr_start = strstr(buf, "+MIPLEVENT")) != NULL)  //�ϱ�ģ�鵱ǰ״̬
            {
                // onenet����ʱ״̬
                if((position_addr_start = strstr(buf, "0, 4")) != NULL)    //4 CONNECT_SUCCESS
                {
                    if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_ONENET_CONNECT_SUCCESS_EVENTBIT, &xHigherPriorityTaskWoken))
                    {
                        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                    }
                }
                if((position_addr_start = strstr(buf, "0, 5")) != NULL)    //5 CONNECT_FAILED
                {
                    if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_ONENET_REG_FAILED_EVENTBIT, &xHigherPriorityTaskWoken))
                    {
                        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                    }
                }
                if((position_addr_start = strstr(buf, "0, 6")) != NULL)    //6 REG_SUCCESS
                {
                    if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_ONENET_REG_SUCCESS_EVENTBIT, &xHigherPriorityTaskWoken))
                    {
                        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                    }
                }
                if((position_addr_start = strstr(buf, "0, 7")) != NULL)    //7 REG_FAILED
                {
                    if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_ONENET_REG_FAILED_EVENTBIT, &xHigherPriorityTaskWoken))
                    {
                        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                    }
                }
                if((position_addr_start = strstr(buf, "0, 11")) != NULL)   //11 UPDATE_SUCCESS
                {
                    if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_ONENET_UPDATE_SUCCESS_EVENTBIT, &xHigherPriorityTaskWoken))
                    {
                        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                    }
                }
                
                
                return true;
            }
            // +MIPLOBSERVE OneNETƽ̨��ģ�鷢��observe����
            // +MIPLOBSERVE: 0, 69171, 1, 10250, 0, -1
            if((position_addr_start = strstr(buf, "+MIPLOBSERVE")) != NULL)  //observe����
            {
                char* pColon = strchr(position_addr_start,':');
                if(pColon)
                {
                    pColon = pColon+2;
                    memset(&gNBDataReceBuf, 0, sizeof(DataReceBuf_t));
                    gNBDataReceBuf.len = len-(pColon - position_addr_start);
                    memcpy(gNBDataReceBuf.Buf, pColon, gNBDataReceBuf.len);
                }
                if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_ONENET_OBSERVE_REQ_EVENTBIT, &xHigherPriorityTaskWoken))
                {
                    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                }
                return true;
            }
            // +MIPLDISCOVER OneNETƽ̨��ģ�鷢��discover����
            // +MIPLDISCOVER: 0, 65310, 3303
            if((position_addr_start = strstr(buf, "+MIPLDISCOVER")) != NULL)  //discover����
            {
                char* pColon = strchr(position_addr_start,':');
                if(pColon)
                {
                    pColon = pColon+2;
                    memset(&gNBDataReceBuf, 0, sizeof(DataReceBuf_t));
                    gNBDataReceBuf.len = len-(pColon - position_addr_start);
                    memcpy(gNBDataReceBuf.Buf, pColon, gNBDataReceBuf.len);
                }
                if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_ONENET_DISCOVER_REQ_EVENTBIT, &xHigherPriorityTaskWoken))
                {
                    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                }
                return true;
            }
            // +MIPLREAD OneNETƽ̨��ģ�鷢��read����
            // +MIPLREAD: 0, 638, 3300, 0, 5750
            if((position_addr_start = strstr(buf, "+MIPLREAD")) != NULL)    //read����
            {
                char* pColon = strchr(position_addr_start,':');
                if(pColon)
                {
                    pColon = pColon+2;
                    memset(&gNBDataReceBuf, 0, sizeof(DataReceBuf_t));
                    gNBDataReceBuf.len = len-(pColon - position_addr_start);
                    memcpy(gNBDataReceBuf.Buf, pColon, gNBDataReceBuf.len);
                }
                if(pdPASS == xEventGroupSetBitsFromISR(Me3616AppEventGroup, NB_ONENET_READ_REQ_EVENTBIT, &xHigherPriorityTaskWoken))
                {
                    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                }
                return true;
            }
            // +MIPLWRITE OneNETƽ̨��ģ�鷢��write����
            // +MIPLWRITE: 0, 65316, 3300, 0, 5750, 1, 4, HTGD, 0, 0
            if((position_addr_start = strstr(buf, "+MIPLWRITE")) != NULL)   //write����
            {
                char* pColon = strchr(position_addr_start,':');
                if(pColon)
                {
                    pColon = pColon+2;
                    memset(&gNBDataReceBuf, 0, sizeof(DataReceBuf_t));
                    gNBDataReceBuf.len = len-(pColon - position_addr_start);
                    memcpy(gNBDataReceBuf.Buf, pColon, gNBDataReceBuf.len);
                }
                if(pdPASS == xEventGroupSetBitsFromISR(Me3616AppEventGroup, NB_ONENET_WRITE_REQ_EVENTBIT, &xHigherPriorityTaskWoken))
                {
                    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                }
                return true;
            }
        }
        else 	//����oceanconnectƽ̨
        {
            // +M2MCLI:register success
            if((position_addr_start = strstr(buf, "+M2MCLI:")) != NULL)  //�ϱ�ģ�鵱ǰ״̬
            {
                if((position_addr_start = strstr(buf, "notify success")) != NULL)
                {
                    if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT, &xHigherPriorityTaskWoken))
                    {
                        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                    }
                }
                else if((position_addr_start = strstr(buf, "notify failed")) != NULL)
                {
                    if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_OCEANCONNECT_NOTIFY_FAILED_EVENTBIT, &xHigherPriorityTaskWoken))
                    {
                        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                    }
                }
                else if((position_addr_start = strstr(buf, "register success")) != NULL)
                {
                    if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_OCEANCONNECT_REG_SUCCESS_EVENTBIT, &xHigherPriorityTaskWoken))
                    {
                        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                    }
                }
                else if((position_addr_start = strstr(buf, "register failed")) != NULL)
                {
                    if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_OCEANCONNECT_REG_FAILED_EVENTBIT, &xHigherPriorityTaskWoken))
                    {
                        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                    }
                }
                else if((position_addr_start = strstr(buf, "observe success")) != NULL)
                {
                    if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_OCEANCONNECT_OB_SUCCESS_EVENTBIT, &xHigherPriorityTaskWoken))
                    {
                        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                    }
                }
//                else if((position_addr_start = strstr(buf, "register update success")) != NULL)
//                {
//                    
//                }
//                else if((position_addr_start = strstr(buf, "deregister success")) != NULL)
//                {
//                    
//                }
                return true;
            }
            // +M2MCLIRECV:0200004030415E       Hex mode
            // +M2MCLIRECV:12345678             Text mode
            if((position_addr_start = strstr(buf, "+M2MCLIRECV")) != NULL)  //ƽ̨���������ϱ�
            {
                char* pColon = strchr(position_addr_start,':');
                if(pColon)
                {
                    pColon = pColon+1;;
                    memset(&gNBDataReceBuf, 0, sizeof(DataReceBuf_t));
                    gNBDataReceBuf.len = len-(pColon - position_addr_start);
                    memcpy(gNBDataReceBuf.Buf, pColon, gNBDataReceBuf.len);
                }
                if(pdPASS == xEventGroupSetBitsFromISR(Me3616AppEventGroup, NB_OCEANCONNECT_RECV_EVENTBIT, &xHigherPriorityTaskWoken))
                {
                    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                }
                return true;
            }
        }
        
        // PSM״̬���
        if((position_addr_start = strstr(buf, "ENTER PSM")) != NULL)       //����PSM
        {
            g_me3616_info.me3616_enterPSM_flag = 1;
            if(pdPASS == xEventGroupSetBitsFromISR(Me3616AppEventGroup, NB_ENTERPSM_EVENTBIT, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            return true;
        }    
        if((position_addr_start = strstr(buf, "EXIT PSM")) != NULL)       //�˳�PSM
        {
            g_me3616_info.me3616_enterPSM_flag = 0;
            return true;
        }
    }
    
    #if (ME3616_GPS_EN) 
    // GNSS���ݷ���
    // $GPGLL,4004.74005,N,11614.19613,E,060845.00,A,A*5B<CR><LF>
    // $GNGLL,,,,,,V,N*7A
    if(g_me3616_info.gnss_run_state == 1)
    {
        if((position_addr_start = strstr(buf, "$GNGLL")) != NULL) //����׼���ã����շ�AT
        {
            char* pColon = strchr(position_addr_start,',');
            if(pColon)
            {
                pColon = pColon+1;
                memset(&gNBGpsDataReceBuf, 0, sizeof(gNBGpsDataReceBuf));
                gNBGpsDataReceBuf.len = len-(pColon - position_addr_start);
                memcpy(gNBGpsDataReceBuf.Buf, pColon, gNBGpsDataReceBuf.len);
            }
            if(pdPASS == xEventGroupSetBitsFromISR(Me3616AppEventGroup, NB_GNSS_RECE_EVENTBIT, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            return true;
        }
    }
    // +ZGPS: DATA DOWNLOAD SUCCESS
//    if(g_me3616_info.agps_data_ready == 0)
//    {
//        if((position_addr_start = strstr(buf, "DOWNLOAD SUCCESS")) != NULL)
//        {
//            NRF_LOG_INFO("AGPS data download");
//            g_me3616_info.agps_data_ready = 1;
//            if(pdPASS == xEventGroupSetBitsFromISR(Me3616AppEventGroup, NB_AGPS_DATAREADY_EVENTBIT, &xHigherPriorityTaskWoken))
//            {
//                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//            }
//        }
//    }
    #endif
    
    return isAsync;
}


/**@brief ME3616�������ݽ��ջص����� \n
* ����Ϣ���з��࣬�첽��Ϣ����:: me3616_AsyncNotification ���д���
* @param[in]  buf 	���յ����ݻ����ַ
* @param[in]  len 	���ݳ���
* @see :: ME3616_FxnTable
*/
static void me3616_receCb(char* buf, uint16_t len)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if(len > NB_UART_RECE_BUF_MAX_LEN)  //�������ջ��泤��
        return;
    
    buf[len] = '\0';
    NRF_LOG_INFO("%s", buf);
    
#if 1
    if(!me3616_AsyncNotification(buf, len))  //���첽��Ϣ(�����á�ִ�С���ȡָ��)
    {
        if((gNBReceBuf.len + len) < NB_UART_RECE_BUF_MAX_LEN)
        {
            memcpy(gNBReceBuf.Buf + gNBReceBuf.len, buf, len);
            gNBReceBuf.len += len;
        }
        if(strstr((char*)buf,"OK"))
        {
            if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_ACK_OK_EVENTBIT, &xHigherPriorityTaskWoken))        //��CMD����OK�¼�λ
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
        }
        else if (strstr((char*)buf,"ERROR"))
        {
            if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_ACK_ERROR_EVENTBIT, &xHigherPriorityTaskWoken))     //��CMD���������¼�λ
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
        }
    }
    else
    {
        memset(&gNBReceBuf, 0, sizeof(ReceBuf_t));   //���첽��Ϣ�����
    }
#endif
}


// me3616��ʱ�ó�ʱ������
//static void me3616_timeoutCb(void)
//{
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_ACK_TIMEOUT_EVENTBIT, &xHigherPriorityTaskWoken))    //�ó�ʱ�¼���־
//    {
//        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//    }
//}




/*******************����OceanConnectƽ̨��������*****************************************************************************/
/**@brief ����ƽ̨ע��
* @param[in]  handle 	NBģ���������
* @param[in]  lifetime 	ƽ̨����ʱ�䣬ģ��ᶨ�ڸ���ע��
* @return  ����ִ�н��
* - NB_IOT_REGIST_SUCCESS  	ִ�гɹ�
* - NB_IOT_REGIST_FAILED  	ִ��ʧ��
* @par ʾ��:
* @code
*	
* @endcode
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_oceanconnect_regist(NB_Handle handle, uint32_t lifetime)
{
    uint8_t ret;
    EventBits_t uxBits;
    
    char cmd_data[100];
    
    snprintf(cmd_data, 100, "%s,\"%15s\",%d", "AT+M2MCLINEW=180.101.147.115,5683", g_me3616_info.me3616_IMEI, lifetime);
    xEventGroupClearBits(Me3616EventGroup, NB_OCEANCONNECT_REG_SUCCESS_EVENTBIT|NB_OCEANCONNECT_OB_SUCCESS_EVENTBIT);
    ret = me3616_send_cmd(handle, cmd_data, 2000, 3);  // ����ƽ̨ע��
    if(ret == NB_ACK_OK)
    {
        uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_OCEANCONNECT_REG_SUCCESS_EVENTBIT | NB_OCEANCONNECT_OB_SUCCESS_EVENTBIT, pdTRUE, pdTRUE, 30000);
        if((uxBits & (NB_OCEANCONNECT_REG_SUCCESS_EVENTBIT | NB_OCEANCONNECT_OB_SUCCESS_EVENTBIT) ) == (NB_OCEANCONNECT_REG_SUCCESS_EVENTBIT | NB_OCEANCONNECT_OB_SUCCESS_EVENTBIT))
            return NB_IOT_REGIST_SUCCESS;
        else
            return NB_IOT_REGIST_FAILED;
    }
    else
    {
        return NB_IOT_REGIST_FAILED;
    }
}

/**@brief ����ƽ̨ȥע��
* @param[in]  handle 	NBģ���������
* @return  ����ִ�н��
* - NB_ACK_OK  	ִ�гɹ�
* - Others  	ִ��ʧ��
* @par ʾ��:
* @code
*	AT+M2MCLIDEL
* @endcode
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_oceanconnect_deregist(NB_Handle handle)
{
    uint8_t ret;
    ret = me3616_send_cmd(handle, "AT+M2MCLIDEL", 1000, 1);  // ����ƽ̨ȥע��
    return ret;
}

#if 1
/**@brief ����ƽ̨�����·��Ļظ�
* @param[in]  handle    	NBģ���������
* @param[in]  cmd    		Ҫ�ظ�������ָ��
* @return  ����ִ�н��
* - NB_RPLY_SUCCESS  	ִ�гɹ�
* - NB_RPLY_FAIL  	ִ��ʧ��
* @par ʾ��:
* @code
*	AT+M2MCLISEND=000101
* @endcode
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_oceanconnect_rply(NB_Handle handle, char *cmd)
{
    EventBits_t uxBits;
    uint8_t ret;
    uint8_t rply_result;
    
    ret = me3616_send_cmd(handle, cmd, 2000, 2); //�����ϱ�����
    if(ret == NB_ACK_OK)
    {
        uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT|NB_OCEANCONNECT_NOTIFY_FAILED_EVENTBIT, pdTRUE, pdFALSE, 30000);
        if((uxBits & NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT) == NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT)
            rply_result = NB_RPLY_SUCCESS;
        else
            rply_result = NB_RPLY_FAIL;
    }
    else
        rply_result = NB_RPLY_FAIL;
    
    return rply_result;
}
#endif


#if 0
/**@brief ����ƽ̨��������
* @param[in]  handle    	NBģ���������
* @param[in]  data    		Ҫ��������ָ��
* @param[in]  len  			Ҫ�������ݳ���
* @return  ����ִ�н��
* - NB_OCEANCONNECT_SEND_SUCCESS  	ִ�гɹ�
* - NB_OCEANCONNECT_SEND_FAIL  	ִ��ʧ��
* @par ʾ��:
* @code
*	AT+M2MCLISEND=000101
* @endcode
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_oceanconnect_send(NB_Handle handle, uint8_t *data, uint16_t len)
{
    EventBits_t uxBits;
    uint8_t ret;
    char cmd_data[100];
    
    if(len>70)
        return NB_ACK_ERROR;
    
    snprintf(cmd_data, 100, "%s%04X%s", "AT+M2MCLISEND=01", len, data);
    xEventGroupClearBits(Me3616EventGroup, NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT|NB_OCEANCONNECT_NOTIFY_FAILED_EVENTBIT);
    ret = me3616_send_cmd(handle, cmd_data, 2000, 3);
    if(ret == NB_ACK_OK)
    {
        uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT|NB_OCEANCONNECT_NOTIFY_FAILED_EVENTBIT, pdTRUE, pdFALSE, 30000);
        if((uxBits & NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT) == NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT)
        {
            return NB_OCEANCONNECT_SEND_SUCCESS;
        }
        else
            return NB_OCEANCONNECT_SEND_FAIL;
    }
    else
    {
        return NB_OCEANCONNECT_SEND_FAIL;
    }
}
#endif


/*******************�ƶ�OneNetƽ̨��������*****************************************************************************/

/**@brief ��ȡonenetͨ����Ϣ��message id
* @param[in]  *data    	ԭʼָ������ָ��
* @param[out] *mid 		message idָ��
* @par ʾ��:
* @code
*	+MIPLOBSERVE: 0, 130843, 1, 10250, 0, -1
* @endcode
* @see :: me3616_onenet_regist  :: me3616_event_handle
*/
static void me3616_onenet_getid(char *data, char *mid)
{
    char temp[100];
    memset(gNBOnenetMsg.msgid, 0, 10);
    strcpy(temp, (char*)data);
    strtok(temp,",");
    strcpy(mid,strtok(NULL,",")+1);
}

/**@brief ����onenet��write���������
* @param[in]  *data    	ԭʼָ������ָ��
* @param[in]  *len    	ԭʼָ�����ݳ���ָ��
* @param[out] *appdata 	��������ָ��
* @par ʾ��:
* @code
*	+MIPLWRITE: 0, 65316, 3300, 0, 5750, 1, 4, 7E01, 0, 0
* @endcode
* @see :: me3616_event_handle
*/
static void me3616_onenet_getWriteData(char *data, char *len, char *appdata)
{
    char temp[100];
    memset(gNBOnenetMsg.msgid, 0, 10);
    strcpy(temp, (char*)data);
    strtok(temp,",");   //0
    strtok(NULL,",");   //65316
    strtok(NULL,",");   //3300
    strtok(NULL,",");   //0
    strtok(NULL,",");   //5750  
    strtok(NULL,",");   //1 �ַ���
    
    strncpy(len, strtok(NULL,",")+1, 4); //4 ����
    strcpy(appdata, strtok(NULL,",")+1);
}

/**@brief onenetע���ʼ����
* @param[in]  handle    	NBģ���������
* @param[in]  lifetime    	ƽ̨����ʱ�䣬ģ��ᶨ�ڸ���ע��
* @return  ����ִ�н��
* - NB_IOT_REGIST_SUCCESS  	ִ�гɹ�
* - NB_IOT_REGIST_FAILED  	ִ��ʧ��
* - Others  	ִ��ʧ��
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_onenet_regist(NB_Handle handle, uint32_t lifetime)
{
    uint8_t ret;
    EventBits_t uxBits;
    char cmd_data[50];
    
    // ����OneNET instance
    ret = me3616_send_cmd(handle, "AT+MIPLCREATE", 2000, 2);
    if(ret != NB_ACK_OK)
        return NB_IOT_REGIST_FAILED;
    // ����object id (һ��ʵ�� resource id 5750	Application Type RW)
    ret = me3616_send_cmd(handle, "AT+MIPLADDOBJ=0,3300,1,\"1\",1,0", 2000, 2);
    if(ret != NB_ACK_OK)
        return NB_IOT_REGIST_FAILED;
    // �豸ע�ᵽOneNETƽ̨ AT+MIPLOPEN=0,3600
    xEventGroupClearBits(Me3616EventGroup, NB_ONENET_REG_SUCCESS_EVENTBIT|NB_ONENET_OBSERVE_REQ_EVENTBIT|NB_ONENET_DISCOVER_REQ_EVENTBIT);  //���¼�λ
    snprintf(cmd_data, 50, "AT+MIPLOPEN=0,%d", lifetime);
    ret = me3616_send_cmd(handle, cmd_data, 2000, 2);
    if(ret != NB_ACK_OK)
        return NB_IOT_REGIST_FAILED;
    
    // 120s�ȴ�ע��ɹ�
    uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_ONENET_REG_SUCCESS_EVENTBIT, pdTRUE, pdFALSE, 120000);
    if((uxBits & NB_ONENET_REG_SUCCESS_EVENTBIT) == NB_ONENET_REG_SUCCESS_EVENTBIT) //onenetע��ɹ�
    {
        uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_ONENET_OBSERVE_REQ_EVENTBIT, pdTRUE, pdFALSE, 30000);
        if((uxBits & NB_ONENET_OBSERVE_REQ_EVENTBIT) == NB_ONENET_OBSERVE_REQ_EVENTBIT)
        {
            me3616_onenet_getid(gNBDataReceBuf.Buf, gNBOnenetMsg.observemsgid);
            snprintf(cmd_data, 50, "AT+MIPLOBSERVERSP=0,%s,1", gNBOnenetMsg.observemsgid);
            ret = me3616_send_cmd(handle, cmd_data, 2000, 2);
            if(ret == NB_ACK_OK)
            {
                NRF_LOG_INFO("OBSERVERSP OK");
            }
            else
            {
                NRF_LOG_INFO("OBSERVERSP FAIL");
                return NB_IOT_REGIST_FAILED;
            }
        }
        else
        {
            return NB_IOT_REGIST_FAILED;
        }
        
        // +MIPLDISCOVER: 0, 65310, 10250
        uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_ONENET_DISCOVER_REQ_EVENTBIT, pdTRUE, pdFALSE, 30000);
        if((uxBits & NB_ONENET_DISCOVER_REQ_EVENTBIT) == NB_ONENET_DISCOVER_REQ_EVENTBIT)
        {
            me3616_onenet_getid(gNBDataReceBuf.Buf, gNBOnenetMsg.msgid);
            // AT+MIPLDISCOVERRSP=0,65310,1,3,"0;1"
            snprintf(cmd_data, 50, "AT+MIPLDISCOVERRSP=0,%s,1,4,\"5750\"", gNBOnenetMsg.msgid);
            ret = me3616_send_cmd(handle, cmd_data, 2000, 2);
            if(ret == NB_ACK_OK)
            {
                NRF_LOG_INFO("DISCOVERRSP OK");
            }
            else
            {
                NRF_LOG_INFO("DISCOVERRSP FAIL");
                return NB_IOT_REGIST_FAILED;
            }
        }
        else
        {
            return NB_IOT_REGIST_FAILED;
        }
        
//        me3616_onenet_update(handle, lifetime, 2); //����ע��
        
        return NB_IOT_REGIST_SUCCESS;
    }
    else
    {
        return NB_IOT_REGIST_FAILED;
    }  
}

/**@brief �ƶ�ƽ̨����ע��
* @param[in]  handle    	NBģ���������
* @param[in]  lifetime    	����ƽ̨����ʱ��
* @param[in]  update_try  	ʧ�����Դ���
* @return  ����ִ�н��
* - NB_ACK_OK  	ִ�гɹ�
* - Others  	ִ��ʧ��
* @par ʾ��:
* @code
*	AT+MIPLUPDATE=0,3600,1
* @endcode
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_onenet_update(NB_Handle handle, uint32_t lifetime, uint8_t update_try)
{
    uint8_t ret;
    EventBits_t uxBits;
    char cmd_data[50];
    
    if(g_me3616_info.me3616_enterPSM_flag == 1) //ģ�������ߣ���Ҫ����
        me3616_WakeUp(handle);
    
    snprintf(cmd_data, 50, "AT+MIPLUPDATE=0,%d,1", lifetime);
    ret = NB_ONENET_UPDATE_FAIL;
    xEventGroupClearBits(Me3616EventGroup, NB_ONENET_UPDATE_SUCCESS_EVENTBIT);
    if( me3616_send_cmd(handle, cmd_data, 2000, 2) == NB_ACK_OK )
    {
        uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_ONENET_UPDATE_SUCCESS_EVENTBIT, pdTRUE, pdFALSE, 20000);
        if((uxBits & NB_ONENET_UPDATE_SUCCESS_EVENTBIT) == NB_ONENET_UPDATE_SUCCESS_EVENTBIT)
        {
            g_me3616_info.iot_regist_status = 1;
            ret = NB_ONENET_UPDATE_SUCCESS;
        }
    }
    
    if(ret != NB_ONENET_UPDATE_SUCCESS) //����ʧ������ע��
    {
        ret = me3616_Set_IotRegist(handle, g_me3616_info.regist_lifetime, 2);   //ƽ̨ע��
        if(ret != NB_IOT_REGIST_SUCCESS)    //�ٴγ���ע��ʧ��
        {
            return NB_ONENET_UPDATE_FAIL;
        }
    }
    
    return NB_ONENET_UPDATE_SUCCESS;
}

/**@brief �ƶ�ƽ̨ȥע��
* @param[in]  handle    	NBģ���������
* @return  ����ִ�н��
* - NB_ACK_OK  	ִ�гɹ�
* - Others  	ִ��ʧ��
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_onenet_deregist(NB_Handle handle)
{
    uint8_t ret;
    ret = me3616_send_cmd(handle, "AT+MIPLCLOSE=0", 2000, 1);  // ONENETƽ̨ȥע��
    return ret;
}


#if(ME3616_PSM_TIMEOUT_HANDLE_EN)
/**@brief ����PSM��ʱ������
*/
static void psm_timeout_handler(void * p_context)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(pdPASS == xEventGroupSetBitsFromISR(Me3616AppEventGroup, NB_PSM_ENTER_TIMEOUT_EVENTBIT, &xHigherPriorityTaskWoken))        //��PSM���볬ʱ�¼�λ
    {
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}
#endif

static uint8_t nb_reportFail_delayTry_flag = 0;    ///< ��ʱע��������־

/**@brief �ϱ�ʧ����ʱ����ע�ᳬʱ������
* @ref NB_REPFAIL_REG_DELAY_TRY �����ӻ����ԣ����ӳ��ڼ���������������ӻ��������ڸ�Ƶ���ϱ����ϱ�ʧ������ע�ᳬʱ15min��
* @see :: me3616_notify
*/
static void reportFail_regDelayTry_timeout_handler(void * p_context)
{
    nb_reportFail_delayTry_flag = 2;
}

/**@brief NBģ������ƽ̨�ϱ�����
* @param[in]  handle  			NBģ���������
* @param[in]  *data    			�ϱ�����ָ��
* @param[in]  len    			�ϱ����ݳ���
* @param[in]  rcc_enabled  		�ϱ�ʱ�Ƿ������ͷ�RCC����
* @param[in]  update_enabled    �ϱ�ʱ�Ƿ����ע��(ֻ������onenet)
* @param[in]  report_fail_try_type	�ϱ�ʧ������ע������ \n
* @ref NB_REPFAIL_REG_TRY ������������	\n
* @ref NB_REPFAIL_REG_DELAY_TRY �����ӻ����ԣ����ӳ��ڼ���������������ӻ��������ڸ�Ƶ���ϱ����ϱ�ʧ������ע�ᳬʱ15min�� \n
* @ref NB_REPFAIL_REG_NO_TRY ��������
* @return  ����ִ�н��
* - NB_NOTIFY_SUCCESS  	�ϱ��ɹ�
* - NB_NOTIFY_FAIL		�ϱ�ʧ��
* - NB_IOT_REGIST_FAILED ע��ʧ�ܷ���
* - Others  ��������
* @par ʾ��:
* @code
*	�ƶ�ƽ̨�������� AT+MIPLNOTIFY=0,122553,3308,0,5900,4,4,50,0,0
*	����ƽ̨�������� AT+M2MCLISEND=000101
* @endcode
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_notify(NB_Handle handle, uint8_t *data, uint16_t len, uint8_t rcc_enabled, uint8_t update_enabled, uint8_t report_fail_try_type)
{
    EventBits_t uxBits;
    uint8_t ret;
    uint8_t notify_result;
    
    char cmd_data[220];
    char notify_data[100];
    
    if(len > 99)
        return NB_NOTIFY_FAIL;
    
	#if(ME3616_PSM_TIMEOUT_HANDLE_EN)
	app_timer_stop(m_psm_timer_id);	// ֹͣPSM��ʱ��ʱ
	#endif
	
    xSemaphoreTake( Nb_Notify_Semaphore,  600000);    //���ȴ�10����
//    if(g_me3616_info.me3616_enterPSM_flag == 1) //ģ�������ߣ���Ҫ����
//    {
//        me3616_WakeUp(handle);
//    }
    me3616_WakeUp(handle);  //Ϊ��֤�����ϱ��ɿ���ÿ�������Ѳ���������PSM��־ʧЧ�������
    
	if(g_me3616_info.me3616_enterPSM_flag == 1)
	{
		me3616_send_cmd(handle, "AT+M2MCLISEND=00", 2000, 1);   // ���Ϳ������˳�PSM
	}

//    me3616_getSignal(handle);
	
    //�ϱ�֮ǰ��ƽ̨֮ǰû��ע��ɹ����ٴ�ע��(�����ڵ�Ƶ�����ϱ��Ŀɿ���Ҫ��ߵ��ϱ�����)
#if 1	//�����Ż��棬��������NBͨ��ʱ��
    if((g_me3616_info.iot_regist_status == 0) && (report_fail_try_type == NB_REPFAIL_REG_TRY))
    {
        ret = me3616_Set_IotRegist(handle, g_me3616_info.regist_lifetime, 3);   //ƽ̨ע��
        if(ret != NB_IOT_REGIST_SUCCESS)    //�ٴγ���ע��ʧ��
        {
            xSemaphoreGive( Nb_Notify_Semaphore );
            return ret;
        }
    }
#endif
	
    //�ƶ�OneNet�ϱ���������ע�ᣨ����ʧ�ܻ�����ע�ᣩ
    if((update_enabled == NB_UPDATE_ENABLE) && (g_me3616_info.me3616_band == BAND_8) && (g_me3616_info.me3616_iot_platform == ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT))
    {
        ret = me3616_onenet_update(handle, g_me3616_info.regist_lifetime, 3);
        if(ret != NB_ONENET_UPDATE_SUCCESS)
        {
            xSemaphoreGive( Nb_Notify_Semaphore );
            return ret;
        }
    }
    
    memcpy(notify_data, data, len);
    notify_data[len] = '\0';
    // �������
    if((g_me3616_info.me3616_band == BAND_8) && (g_me3616_info.me3616_iot_platform == ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT)) //�ƶ��ϱ�
    {
        snprintf(cmd_data, 220, "AT+MIPLNOTIFY=0,%s,3300,0,5750,1,%d,%s,0,0", gNBOnenetMsg.observemsgid, len, notify_data);
    }
    else    //�����ϱ�
    {
        #if (OCEANCONNECT_DATA_MODE == HEX_MODE)
        char temp[200];
        memset(temp, 0, 200);
        StrToHexStr(temp, notify_data, len);
        snprintf(cmd_data, 220, "AT+M2MCLISEND=01%04X%s", len, temp);
        #elif (OCEANCONNECT_DATA_MODE == TEXT_MODE)
        snprintf(cmd_data, 220, "AT+M2MCLISEND=\"01%04X%s\"", len, notify_data);
        #endif
		
		#if (ME3616_NOTIFY_NEED_RPLY_EN)
		xEventGroupClearBits(Me3616EventGroup, NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT|NB_OCEANCONNECT_NOTIFY_FAILED_EVENTBIT|NB_OCEANCONNECT_NOTIFY_AND_RPLY_SUCCESS_EVENTBIT);
		#else
		xEventGroupClearBits(Me3616EventGroup, NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT|NB_OCEANCONNECT_NOTIFY_FAILED_EVENTBIT);
		#endif
    }
	
    ret = me3616_send_cmd(handle, cmd_data, 3000, 3); //�����ϱ�����
    if((g_me3616_info.me3616_band == BAND_8) && (g_me3616_info.me3616_iot_platform == ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT))
    {
        if(ret == NB_ACK_OK)
            notify_result = NB_NOTIFY_SUCCESS;
        else
            notify_result = NB_NOTIFY_FAIL;
    }
    else
    {
        if(ret == NB_ACK_OK)
        {
			#if (ME3616_NOTIFY_NEED_RPLY_EN)
			uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_OCEANCONNECT_NOTIFY_AND_RPLY_SUCCESS_EVENTBIT|NB_OCEANCONNECT_NOTIFY_FAILED_EVENTBIT, pdTRUE, pdFALSE, 30000);
            if((uxBits & NB_OCEANCONNECT_NOTIFY_AND_RPLY_SUCCESS_EVENTBIT) == NB_OCEANCONNECT_NOTIFY_AND_RPLY_SUCCESS_EVENTBIT)
			#else
			uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT|NB_OCEANCONNECT_NOTIFY_FAILED_EVENTBIT, pdTRUE, pdFALSE, 30000);
            if((uxBits & NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT) == NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT)
			#endif
            {
                NRF_LOG_INFO("NB_NOTIFY_SUCCESS");
                notify_result = NB_NOTIFY_SUCCESS;
            }
            else
                notify_result = NB_NOTIFY_FAIL;
        }
        else
            notify_result = NB_NOTIFY_FAIL;
    }
    
    if(notify_result == NB_NOTIFY_FAIL) //�ϱ�ʧ�ܣ���Ҫ����ע��
    {
        if( (report_fail_try_type == NB_REPFAIL_REG_TRY) || (nb_reportFail_delayTry_flag == 2) )  //ʧ������ע�ᣨ�����ڵ�Ƶ���ɿ�������
        {
            ret = me3616_Set_IotRegist(handle, g_me3616_info.regist_lifetime, 2);   //�ϱ�ʧ������ƽ̨ע��
            if(ret == NB_IOT_REGIST_SUCCESS)
            {
                if((g_me3616_info.me3616_band == BAND_8) && (g_me3616_info.me3616_iot_platform == ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT))
                {
                    // OneNet����ע��msgid����
                    snprintf(cmd_data, 220, "AT+MIPLNOTIFY=0,%s,3300,0,5750,1,%d,%s,0,0", gNBOnenetMsg.observemsgid, len, notify_data);
//                    if(rcc_enabled == 1)  //�ƶ�����RRC���ɹ�
//                        me3616_RRC_Release(handle);
                    ret = me3616_send_cmd(handle, cmd_data, 2000, 2);
                    if(ret == NB_ACK_OK)
                        notify_result = NB_NOTIFY_SUCCESS;
                    else
                        notify_result = NB_NOTIFY_FAIL;
                }
                else
                {
					#if (ME3616_NOTIFY_NEED_RPLY_EN)
					xEventGroupClearBits(Me3616EventGroup, NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT|NB_OCEANCONNECT_NOTIFY_FAILED_EVENTBIT|NB_OCEANCONNECT_NOTIFY_AND_RPLY_SUCCESS_EVENTBIT);
					#else
					xEventGroupClearBits(Me3616EventGroup, NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT|NB_OCEANCONNECT_NOTIFY_FAILED_EVENTBIT);
					#endif
                    ret = me3616_send_cmd(handle, cmd_data, 3000, 3);
                    if(ret == NB_ACK_OK)
                    {
						#if (ME3616_NOTIFY_NEED_RPLY_EN)
						uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_OCEANCONNECT_NOTIFY_AND_RPLY_SUCCESS_EVENTBIT|NB_OCEANCONNECT_NOTIFY_FAILED_EVENTBIT, pdTRUE, pdFALSE, 30000);
						if((uxBits & NB_OCEANCONNECT_NOTIFY_AND_RPLY_SUCCESS_EVENTBIT) == NB_OCEANCONNECT_NOTIFY_AND_RPLY_SUCCESS_EVENTBIT)
						#else
						uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT|NB_OCEANCONNECT_NOTIFY_FAILED_EVENTBIT, pdTRUE, pdFALSE, 30000);
						if((uxBits & NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT) == NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT)
						#endif
                        {
							NRF_LOG_INFO("NB_NOTIFY_SUCCESS");
                            notify_result = NB_NOTIFY_SUCCESS;
                            if(rcc_enabled == 1)
                            {
                                me3616_RRC_Release(handle);
                                me3616_send_cmd(handle, "AT+M2MCLISEND=00", 2000, 1);
                            }
                        }
                        else
                        {
                            notify_result = NB_NOTIFY_FAIL;
                        }
                    }
                    else
                        notify_result = NB_NOTIFY_FAIL;
                }
            }
            if(nb_reportFail_delayTry_flag == 2)
            {
                nb_reportFail_delayTry_flag = 0;
            }
        }
        else if(report_fail_try_type == NB_REPFAIL_REG_DELAY_TRY)  //ʧ�ܳ�ʱ����ע�ᣨ�����ڸ�Ƶ�ϱ�������
        {
            if(nb_reportFail_delayTry_flag == 0)
            {
                nb_reportFail_delayTry_flag = 1;    //��ʱע�Ὺʼ��ʱ��־
                app_timer_start(m_reportFail_regDelayTry_timer_id, 15*60*1000, NULL);   //GPS��λ��ʱ��ʱ
            }
        }     
    }
    else    //�ϱ��ɹ�
    {
        if(report_fail_try_type == NB_REPFAIL_REG_DELAY_TRY)
        {
            if(nb_reportFail_delayTry_flag != 0)
            {
                nb_reportFail_delayTry_flag = 0;
                app_timer_stop(m_reportFail_regDelayTry_timer_id);
            }
        }
        // RRC�ͷſ��ٽ���PSM
        if(rcc_enabled == NB_RRC_ENABLE)
        {
            me3616_RRC_Release(handle);
            me3616_send_cmd(handle, "AT+M2MCLISEND=00", 2000, 1);
        }
        
    }
    
    xSemaphoreGive( Nb_Notify_Semaphore );
	
	// PSM��ʱ�ݴ���
	#if(ME3616_PSM_TIMEOUT_HANDLE_EN)
	app_timer_start(m_psm_timer_id, ME3616_PSM_TIMEOUT_VALUE, NULL);   //����PSM��ʱ��ʱ
	#endif
	
    return notify_result;
}


/*******************GNSS��������*****************************************************************************/
#if (ME3616_GPS_EN) 

/**@brief GPS��λ��ʱ������ (����Ҫ��ֹ����120s��ֹͣGPS��λ)��ͨ���¼�λ:: NB_GPS_POSITION_TIMEOUT_EVENTBIT
* ��ͬ��:: me3616_event_handle
*/
static void gps_position_timeout_handler(void * p_context)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(pdPASS == xEventGroupSetBitsFromISR(Me3616AppEventGroup, NB_GPS_POSITION_TIMEOUT_EVENTBIT, &xHigherPriorityTaskWoken))        //��CMD����OK�¼�λ
    {
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

/**@brief GPS���ݽ��ճ�ʱ��������ͨ���¼�λ:: NB_GPS_NMEADATA_TIMEOUT_EVENTBIT
* ��ͬ��:: me3616_event_handle
*/
static void gpsRevData_timeout_handler(void * p_context)
{
    if(g_me3616_info.gnss_run_state == 1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if(pdPASS == xEventGroupSetBitsFromISR(Me3616AppEventGroup, NB_GPS_NMEADATA_TIMEOUT_EVENTBIT, &xHigherPriorityTaskWoken))        //��CMD����OK�¼�λ
        {
            portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        }
    }
}


#if 0
/**@brief AGPS������ȡ������λ����,���ж�AGNSS�����Ƿ���Ч����Ч��������
* @param[in]  handle    	NBģ���������
* @param[in]  zgdata_try    ���س������Դ���
* @return  ����ִ�н��
* - NB_AGPS_DATA_READY  ��ȡ�ɹ�
* - Others  ��ȡʧ��
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_agps_zgdata(NB_Handle handle, uint8_t zgdata_try)
{
    uint8_t ret;
    EventBits_t uxBits;
    do{
        //��ѯAGNSS�����Ƿ���Ч +ZGDATA: READY
        ret = NB_AGPS_DATA_NOT_READY;
        if( me3616_send_cmd(handle, "AT+ZGDATA?", 2000, 1) == NB_ACK_OK )
        {
            if(strstr(gNBReceBuf.Buf, "+ZGDATA: READY") == NULL)    //AGNSS����ʧЧ��NOT READY��
            {
                xEventGroupClearBits(Me3616AppEventGroup, NB_AGPS_DATAREADY_EVENTBIT);
                if(me3616_send_cmd(handle, "AT+ZGDATA", 1000, 1) != NB_ACK_OK)   //��������AGPS����,��������ʾGPS������δ���
                {
                    vTaskDelay(10000);  //���ش�����ʱ�ȴ����������
                }
                else
                {
                    g_me3616_info.gnss_run_state = 1;
                    uxBits = xEventGroupWaitBits(Me3616AppEventGroup, NB_AGPS_DATAREADY_EVENTBIT, pdTRUE, pdFALSE, 30000);
                    g_me3616_info.gnss_run_state = 0;
                    if((uxBits & NB_AGPS_DATAREADY_EVENTBIT) == NB_AGPS_DATAREADY_EVENTBIT)
                        return NB_AGPS_DATA_READY;
                    else
                        ret = NB_AGPS_DATA_NOT_READY;
                }
            }
            else
                return NB_AGPS_DATA_READY;
        }
        
    }while( (zgdata_try--) && (ret != NB_AGPS_DATA_READY) );
    return ret;
}
#endif

/**@brief GPS���ܳ�ʼ��
* @details ����GPS��λģʽΪAGPS��ʹ���Զ���ȡ��λ������Ϣ
* @param[in]  handle    NBģ���������
* @return  ����ִ�н��
* - NB_ACK_OK  	ִ�гɹ�
* - Others  	ִ��ʧ��
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_gps_init(NB_Handle handle)
{
    uint8_t ret;
    
    #if (BOARD_GNT_VERSION ==3)
    nrf_gpio_cfg_output(GPS_LNA_EN);  //����NB_PWRON���
    nrf_gpio_pin_write(GPS_LNA_EN, 0);
    #endif
    
    app_timer_create(&m_gps_position_timer_id, APP_TIMER_MODE_SINGLE_SHOT, gps_position_timeout_handler);   //��λ��ʱ
    app_timer_create(&m_gpsRevData_timer_id, APP_TIMER_MODE_SINGLE_SHOT, gpsRevData_timeout_handler);       //��λ�������ݶ�ʧ��ʱ
    
    
    // AGPS��λ
#if 1
    if(g_me3616_info.gnss_run_mode == ME3616_GPS_MODE_AGPS)
    {   
        ret = me3616_send_cmd(handle, "AT+ZGMODE=1", 2000, 3);  //���ö�λģʽΪAGPS
        ret = me3616_send_cmd(handle, "AT+ZGAUTO=1,10", 2000, 2);
    }
    else
    {
        ret = me3616_send_cmd(handle, "AT+ZGMODE=2", 2000, 3);  //���ö�λģʽΪStandAlone
        ret = me3616_send_cmd(handle, "AT+ZGAUTO=0,10", 2000, 2);
    }
    
//    ret = me3616_send_cmd(handle, "AT+ZGNMEA=32", 2000, 2); //����NMEA�ϱ���ϢΪGLL
//    
//    ret = me3616_agps_zgdata(handle, 2);
    
//    ret = me3616_send_cmd(handle, "AT+ZGRUN=1", 1000, 2);   //�������ζ�λģʽ
//    ret = me3616_send_cmd(handle, "AT+ZGLOCK=0", 1000, 2);   //�������ζ�λģʽ 
//    ret = me3616_send_cmd(handle, "AT+ZGTMOUT?", 1000, 1);   //��ѯ���ζ�λ��ʱʱ��
//    if(ret == NB_ACK_OK)
//    {
//        if(strstr(gNBReceBuf.Buf, "+ZGTMOUT: 60") == NULL)
//        {
//            ret = me3616_send_cmd(handle, "AT+ZGTMOUT=60", 1000, 2);    //���ζ�λ��ʱʱ��60S
//        }
//    }
    
//    ret = me3616_send_cmd(handle, "AT+ZGPSR=1", 1000, 2);  //ʹ�ܶ�λ����ϱ�
    
#endif  

    // GPS��λ
#if 0
    ret = me3616_send_cmd(handle, "AT+ZGMODE=2", 1000, 2);  //���ö�λģʽΪStandAlone
    ret = me3616_send_cmd(handle, "AT+ZGNMEA=32", 2000, 2);  //����NMEA�ϱ���ϢΪGLL
#endif  

    return ret;
}

/**@brief ����GPS����
* @details ÿ��������λ����������ö�λģʽ�����ݸ�ʽ����������λ������������λģʽ
* @param[in]  handle    NBģ���������
* @return  ����ִ�н��
* - NB_ACK_OK  	ִ�гɹ�
* - Others  	ִ��ʧ��
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_gps_run(NB_Handle handle)
{
    uint8_t ret = NB_ACK_OK;
    
    if(g_me3616_info.gnss_run_state == 0)   //û��ʹ��GPS��λ
    {
        me3616_WakeUp(handle); 
        
        if(g_me3616_info.gnss_run_mode == ME3616_GPS_MODE_AGPS)
        {
            me3616_send_cmd(handle, "AT+ZGMODE=1", 1000, 2);
        }
        else
        {
            me3616_send_cmd(handle, "AT+ZGMODE=2", 1000, 2);
        }
        if(NB_ACK_OK != me3616_send_cmd(handle, "AT+ZGNMEA=32", 2000, 2))
            return NB_ACK_ERROR;
//        ret = me3616_agps_zgdata(handle, 3);
        
        ret = me3616_send_cmd(handle, "AT+ZGRUN=2", 2000, 2);
        if(ret == NB_ACK_OK)
        {
            #if (BOARD_GNT_VERSION ==3)
            nrf_gpio_pin_write(GPS_LNA_EN, 1);  //GPS LNAʹ��
            #endif
            
            g_me3616_info.gnss_run_state = 1;
            app_timer_stop(m_gps_position_timer_id);
            app_timer_start(m_gps_position_timer_id, 60000, NULL);   //GPS��λ��ʱ��ʱ
            app_timer_stop(m_gpsRevData_timer_id);
            app_timer_start(m_gpsRevData_timer_id, 20000, NULL);
            
//            if(g_me3616_info.me3616_band == BAND_5) //������Ҫ�ֶ�����PSM
//            {
//                me3616_RRC_Release(handle);
//                me3616_send_cmd(handle, "AT+M2MCLISEND=00", 2000, 1);
//            }
        }
    }
    else if(g_me3616_info.gnss_run_state == 1)  //��λ�������У���ʼ��GPS��λ��ʱ��ʱ��
    {
        app_timer_stop(m_gps_position_timer_id);
        app_timer_start(m_gps_position_timer_id, 60000, NULL);   //GPS��λ��ʱ��ʱ
    }

    return ret;
}

/**@brief �ر�GPS����
* @details ͨ��ָ��ֹͣGPS��λ������ʧ����GPS�������ڴ˲���
* @param[in]  handle    NBģ���������
* @return  ����ִ�н��
* - NB_ACK_OK  	ִ�гɹ�
* - Others  	ִ��ʧ��
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_gps_stop(NB_Handle handle)
{
    uint8_t ret;
    
    if(g_me3616_info.gnss_run_state == 1)
    {
        #if (BOARD_GNT_VERSION ==3)
        nrf_gpio_pin_write(GPS_LNA_EN, 0);  //GPS LNAʹ��
        #endif
        
        me3616_WakeUp(handle);
    //    app_timer_stop(m_gps_position_timer_id);
        ret = me3616_send_cmd(handle, "AT+ZGRUN=0", 2000, 2);
        if(ret == NB_ACK_OK)
        {
            g_me3616_info.gnss_run_state = 0;
        }
        else
        {
    //        me3616_WakeUp(handle);
            me3616_send_cmd(handle, "AT+ZGRST=2", 2000, 2); //������GPS
            vTaskDelay(2000);
            me3616_send_cmd(handle, "AT+ZGRUN=0", 2000, 2);
            g_me3616_info.gnss_run_state = 0;
            Ble_log_send((uint8_t *)"stop_err", 8);
        }
    }
    
    return ret;
}
#endif


/**@brief ME3616ȫ���¼�������
* @details ��ƽ̨���ݽ����¼��ͽ���PSM�¼��������ݸ�Ӧ�ûص�����
* @param[in]  handle    NBģ���������
* @return  
* @see :: ME3616_FxnTable
*/
static void me3616_event_handle(NB_Handle handle)
{
    EventBits_t uxBits;
    
    uxBits = xEventGroupWaitBits(Me3616AppEventGroup, 	\
                                 NB_ENTERPSM_EVENTBIT|NB_ONENET_READ_REQ_EVENTBIT|NB_ONENET_WRITE_REQ_EVENTBIT|NB_OCEANCONNECT_RECV_EVENTBIT	\
								 |NB_GNSS_RECE_EVENTBIT|NB_GPS_POSITION_TIMEOUT_EVENTBIT|NB_GPS_NMEADATA_TIMEOUT_EVENTBIT	\
								 |NB_PSM_ENTER_TIMEOUT_EVENTBIT, 	\
                                 pdTRUE, pdFALSE, portMAX_DELAY);
	
    
    if((uxBits & NB_ONENET_READ_REQ_EVENTBIT) == NB_ONENET_READ_REQ_EVENTBIT)   //onenet������ 3300.0.5750
    {
        char cmd_data[100];
        me3616_onenet_getid(gNBDataReceBuf.Buf, gNBOnenetMsg.msgid);
        snprintf(cmd_data, 100, "AT+MIPLREADRSP=0,%s,1,3300,0,5750,1,4,HTGW,0,0", gNBOnenetMsg.msgid);
        me3616_send_cmd(handle, cmd_data, 1000, 2);
    }
    // +MIPLWRITE: 0, 65316, 3300, 0, 5750, 1, 4, @0A^, 0, 0
    if((uxBits & NB_ONENET_WRITE_REQ_EVENTBIT) == NB_ONENET_WRITE_REQ_EVENTBIT)   //onenetд���� 3300.0.5750
    {
        char cmd_data[100];
        if(strstr((char*)gNBDataReceBuf.Buf,"3300, 0, 5750"))
        {
            me3616_onenet_getid(gNBDataReceBuf.Buf, gNBOnenetMsg.msgid);
            snprintf(cmd_data, 100, "AT+MIPLWRITERSP=0,%s,2", gNBOnenetMsg.msgid);
            me3616_send_cmd(handle, cmd_data, 1000, 2);
            
            char app_clen[4];
            uint16_t app_len;
            char app_data[50];
            if( strlen(gNBDataReceBuf.Buf) > 100)
                return;
            
            me3616_onenet_getWriteData(gNBDataReceBuf.Buf, app_clen, app_data);
            if( strlen(app_clen) > 4)
                return;
            app_len = atoi(app_clen);
            NRF_LOG_INFO("Write Data len=%d, data=%s", app_len, app_data);
            if( (app_len < 60) && ((app_len > 10)) )
                handle->AppReceCB(MSG_ONENET_WRITE_REQ, app_data, app_len);//�ϱ�Ӧ�ô���
        }
            
    }
    
    // +M2MCLIRECV:0200004030415E ����ƽ̨�����·���DL_data��
    if((uxBits & NB_OCEANCONNECT_RECV_EVENTBIT) == NB_OCEANCONNECT_RECV_EVENTBIT)   //oceanconnect���ݽ���
    {
        if(strstr(gNBDataReceBuf.Buf, "AAAA0000") != NULL)  //notify��ƽ̨�ظ�
        {
//            NRF_LOG_INFO("rev_aaaa0000")
			#if (ME3616_NOTIFY_NEED_RPLY_EN)
            xEventGroupSetBits(Me3616EventGroup, NB_OCEANCONNECT_NOTIFY_AND_RPLY_SUCCESS_EVENTBIT);
			#endif
        }
        else
        {
            char app_data[60];
            uint16_t app_len;
            taskENTER_CRITICAL();   //�ٽ��������жϸı�����
            app_len = strlen(gNBDataReceBuf.Buf);
            if(app_len < 60)
                memcpy(app_data, gNBDataReceBuf.Buf, app_len);
            taskEXIT_CRITICAL();
            NRF_LOG_INFO("Write Data len=%d, data=%s", app_len, app_data);
            if( (app_len < 60) && ((app_len > 10)) && (app_data[0] == '0') && (app_data[1] == '2') &&((app_len%2) == 0))
                handle->AppReceCB(MSG_OCEANCONNECT_DATA_RECE, app_data, app_len);//�ϱ�Ӧ�ô���
        }
    }
    
    #if (ME3616_GPS_EN) 
    // $GNGLL,,,,,,V,N*7A       $GPGLL,4004.74005,N,11614.19613,E,060845.00,A,A*5B<CR><LF>
    if((uxBits & NB_GNSS_RECE_EVENTBIT) == NB_GNSS_RECE_EVENTBIT)       //GPS����NMEA�ϱ���Ϣ���ϱ�Ӧ�ý�����
    {
        app_timer_stop(m_gpsRevData_timer_id);
        
        char temp[60];
        uint16_t temp_len;
        
        taskENTER_CRITICAL();   //�ٽ��������жϸı�����
        temp_len = strlen(gNBGpsDataReceBuf.Buf);
        if(temp_len < 60)
        {
            memset(temp, 0, 60);
            memcpy(temp, gNBGpsDataReceBuf.Buf, temp_len);
        }
        taskEXIT_CRITICAL();
        
        if(temp_len < 60)
            handle->AppReceCB(MSG_GPS_GNSS_DATA_RECE, temp, temp_len);//�ϱ�Ӧ�ô���
        
        if(g_me3616_info.gnss_run_state == 1)
            app_timer_start(m_gpsRevData_timer_id, 20000, NULL);
        
        xEventGroupClearBits(Me3616EventGroup, NB_GNSS_RECE_EVENTBIT);  //�����־�Ա�֤��־��λ�����µ�
    }
    if((uxBits & NB_GPS_POSITION_TIMEOUT_EVENTBIT) == NB_GPS_POSITION_TIMEOUT_EVENTBIT) //��λ��ʱ
    {
        app_timer_stop(m_gpsRevData_timer_id);  //��λ��ʱ�ر�NMEA��ⶨʱ��
        handle->AppReceCB(MSG_GPS_POSITION_TIMEOUT, NULL, NULL);    //�ϱ�Ӧ�ô���
    }
    if((uxBits & NB_GPS_NMEADATA_TIMEOUT_EVENTBIT) == NB_GPS_NMEADATA_TIMEOUT_EVENTBIT)  //NMEA�ϱ���ʱ
    {
        handle->AppReceCB(MSG_GPS_NMEADATA_TIMEOUT, NULL, NULL);    //�ϱ�Ӧ�ô���
    }
    #endif
    
    if((uxBits & NB_ENTERPSM_EVENTBIT) == NB_ENTERPSM_EVENTBIT) //ģ�����PSM
    {
        #if (ME3616_GPS_EN) 
        if(g_me3616_info.gnss_run_state == 0)
        {
            me3616_ComClose(handle);    	// �������PSM��GPS����������رմ���
			#if (ME3616_PSM_TIMEOUT_HANDLE_EN)
			app_timer_stop(m_psm_timer_id);	// ֹͣPSM��ʱ��ʱ
			#endif
        }
        #else
        me3616_ComClose(handle);        // �������PSM����رմ���
		#if (ME3616_PSM_TIMEOUT_HANDLE_EN)
		app_timer_stop(m_psm_timer_id);	// ֹͣPSM��ʱ��ʱ
		#endif
        #endif
    }
	
	#if (ME3616_PSM_TIMEOUT_HANDLE_EN)
	if((uxBits & NB_PSM_ENTER_TIMEOUT_EVENTBIT) == NB_PSM_ENTER_TIMEOUT_EVENTBIT) //ģ�����PSM��ʱ
    {
        #if (ME3616_GPS_EN) 
        if(g_me3616_info.gnss_run_state == 0)
        {
            me3616_ComClose(handle);    //�������PSM��ʱ��GPS����������رմ���
			me3616_PowerOff(handle);	//�������PSM��ʱ����ģ��ػ�
        }
        #else
        me3616_ComClose(handle);        //�������PSM��ʱ����رմ���
		me3616_PowerOff(handle);		//�������PSM��ʱ����ģ��ػ�
        #endif
    }
	#endif
    
    
}


#if (OCEANCONNECT_DATA_MODE == HEX_MODE)
// ���ߺ���---------------------------------------------------------------------------------------
/**
* @brief 16�����ַ���ת�ַ���	\n
* @param[in]  *pSrc    	Դ��ַ
* @param[in]  nLen    	�ַ����ĳ��ȣ�����pSrc�ĳ��ȣ�
* @param[out] *pDest   	Ŀ�ĵ�ַ
* @see 16�����ַ���ת�ַ��� :: HexStrToStr
*/
void StrToHexStr(char *pDest, char *pSrc, uint16_t nLen)
{
    uint16_t i;
    
    for(i=0; i<nLen; i++)
    {
        snprintf(pDest, 3, "%02X", pSrc[i]);
        pDest = pDest+2;
    }
}

/**
* @brief 16�����ַ���ת�ַ���	\n
* @param[in]  *pSrc    	Դ��ַ
* @param[in]  nLen    	Ҫת���ĳ��ȣ�16�����ַ����ĳ���/2������pDest�ĳ��ȣ�
* @param[out] *pDest   	Ŀ�ĵ�ַ
* @see �ַ���ת16�����ַ��� :: StrToHexStr
*/
void HexStrToStr(char *pDest, char *pSrc, uint16_t nLen)
{
    uint16_t i;
    
    for(i=0; i<nLen; i++)
    {
        pDest[i] = (pSrc[2*i] - 0x30)*16 + (pSrc[2*i+1] - 0x30);
    }
}
#endif

#endif	/* #if (ME3616_EN) */





/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/



