/**@file    bsp_me3616.c
* @brief   	ME3616_GPS NB-iot驱动
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

/* Includes ------------------------------------------------------------------*/
#include "bsp_me3616.h"
#if (ME3616_EN)

/* Private variables ---------------------------------------------------------*/
/**
* @name Driver全局变量与结构体
* @brief 此常量用于定义存放发送指令缓存区大小。考虑目前大多发送指令不会超过128个字节，
* 除udp与coap发送消息，与接收消息。如果用户需要发送长度数据指令，可以改大此数值
* @{
*/
//定义全局缓存变量（ME3616支持最大发送接收512字节）
#define NB_UART_RECE_BUF_MAX_LEN        256			///< NB串口接收最大缓存字节数
#define NB_UART_SEND_BUF_MAX_LEN        256			///< NB串口发送最大缓存字节数
#define NB_DATA_RECE_BUF_MAX_LEN        256			///< NB接收数据最大缓存字节数
#if (ME3616_GPS_EN) 
#define NB_GPSDATA_RECE_BUF_MAX_LEN     256			///< NB接收GPS数据最大字节数
#endif

/**@struct ReceBuf_t
* @brief 接收缓存空间
*/
typedef struct
{
    char        Buf[NB_UART_RECE_BUF_MAX_LEN];		///< 接收缓存
    uint16_t  len;                          		///< 有效数据长度
} ReceBuf_t;

/**@struct SendBuf_t
* @brief 发送缓存空间
*/
typedef struct
{
    char        Buf[NB_UART_SEND_BUF_MAX_LEN];		///< 发送缓存
    uint16_t    len;                             	///< 有效数据长度
} SendBuf_t;

/**@struct DataReceBuf_t
* @brief NB接收数据缓存空间
*/
typedef struct
{
    char        Buf[NB_DATA_RECE_BUF_MAX_LEN];		///< NB接收数据缓存
    uint16_t    len;                            	///< 有效数据长度
} DataReceBuf_t;

#if (ME3616_GPS_EN) 
/**@struct GpsDataReceBuf_t
* @brief GPS接收数据缓存空间
*/
typedef struct
{
    char        Buf[NB_GPSDATA_RECE_BUF_MAX_LEN];	///< GPS接收数据缓存
    uint16_t    len;                               	///< 有效数据长度
} GpsDataReceBuf_t;
#endif

/**@struct OnenetMsg_t
* @brief ONENET 指令参数结构体
*/
typedef struct
{
    char        observemsgid[10];   ///< 平台订阅 msgid
    char        msgid[10];          ///< 请求消息的 message id
    char        objectid[10];       ///< 请求的 object id
    char        instanceid[2];		///< 请求的 instance id
    char        resourceid[5];		///< 请求的 resource id
} OnenetMsg_t;


EventGroupHandle_t   Me3616EventGroup;      ///< ME3616驱动事件同步
EventGroupHandle_t   Me3616AppEventGroup; 	///< ME3616应用驱动事件同步

//==============================================================================
// NB驱动用缓存定义
static ReceBuf_t            gNBReceBuf;		///< NB串口接收缓存结构体定义
static SendBuf_t            gNBSendBuf;		///< NB串口发送缓存结构体定义
static DataReceBuf_t        gNBDataReceBuf;	///< NB模组数据接收缓存结构体定义
#if (ME3616_GPS_EN) 
static GpsDataReceBuf_t     gNBGpsDataReceBuf;	///< NB模组GPS数据接收缓存结构体定义
#endif
static OnenetMsg_t          gNBOnenetMsg;	///< NB模组onenet信息体定义

// 用于表示保存事件标志
ME3616_info_t    g_me3616_info;          	///< NB模组信息，用于表示保存事件标志和信息
/** @} Driver全局变量与结构体*/


/* function prototypes ------------------------------------------------------*/
/**@name Driver函数前置定义
* @brief ME3616_FxnTable的函数列表的函数映射
* @{
*/
static void me3616_ComOpen(NB_Handle handle);		///< NB模组使用串口、定时器初始化及回调注册
static void me3616_ComClose(NB_Handle handle);		///< NB模组使用串口关闭
static uint8_t me3616_Set_IotRegist(NB_Handle handle, uint32_t lifetime, uint8_t regist_try);	///< 模组初始化设置及IOT平台注册（可用于重注册）
static uint8_t me3616_init(NB_Handle handle, uint32_t lifetime, uint8_t regist_enabled, uint8_t regist_try);	///< 模组初始化（基本信息的获取、休眠的设置）

static uint8_t me3616_send_cmd(NB_Handle handle, char *cmd, uint16_t waittime, uint8_t cmd_try);	///< 模组指令发送
static void me3616_RRC_Release(NB_Handle handle);								///< 模组RRC链接释放，用于快速进入休眠
static uint8_t me3616_psm_set(NB_Handle handle, char *t3412, char *t3324);		///< 模组PSM设置接口
static uint8_t me3616_getModuleInfo(NB_Handle handle, uint8_t InfoType, uint8_t IfReset, uint8_t err_try);	///< 获取模组信息 IMSI、IMEI、 PSM设置、 BAND、 固件版本号
static uint8_t me3616_getSignal(NB_Handle handle);	///< 获取模组信号强度

static void me3616_WakeUp(NB_Handle handle);		///< NB模组唤醒
static void me3616_Reset(NB_Handle handle);			///< NB模组复位
static void me3616_PowerOn(NB_Handle handle);		///< NB模组开机
static void me3616_PowerOff(NB_Handle handle);		///< NB模组关机

// 电信平台oceanconnect操作函数
static uint8_t me3616_oceanconnect_regist(NB_Handle handle, uint32_t lifetime);	///< oceanconnect平台注册
static uint8_t me3616_oceanconnect_deregist(NB_Handle handle);					///< oceanconnect平台去注册
//static uint8_t me3616_oceanconnect_send(NB_Handle handle, uint8_t *data, uint16_t len);
static uint8_t me3616_oceanconnect_rply(NB_Handle handle, char *cmd);			///< oceanconnect平台应答

// 移动平台onenet操作函数
static uint8_t me3616_onenet_regist(NB_Handle handle, uint32_t lifetime);		///< onenet平台注册
static uint8_t me3616_onenet_deregist(NB_Handle handle);						///< onenet平台去注册
static uint8_t me3616_onenet_update(NB_Handle handle, uint32_t lifetime, uint8_t update_try);	///< onenet平台更新注册

static uint8_t me3616_notify(NB_Handle handle, uint8_t *data, uint16_t len, uint8_t rcc_enabled, uint8_t update_enabled, uint8_t report_fail_try_type);	///< NB模组消息上报

#if (ME3616_GPS_EN) 
// GPS定位操作函数
APP_TIMER_DEF(m_gps_position_timer_id);        	///< GPS定位计时定时器，120s定位时间
static void gps_position_timeout_handler(void * p_context);	///< GPS定位超时处理函数
APP_TIMER_DEF(m_gpsRevData_timer_id);           ///< GPS启动定位开启上报，NMEA上报数据超时定时器
static void gpsRevData_timeout_handler(void * p_context);   ///< GPS数据接收超时处理函数

static uint8_t me3616_gps_init(NB_Handle handle);	///< 模组GPS功能初始化
static uint8_t me3616_gps_run(NB_Handle handle);	///< 模组GPS启动定位
static uint8_t me3616_gps_stop(NB_Handle handle);	///< 模组GPS停止定位
#endif
/** @} Driver函数前置定义*/


// PSM上报超时处理
#if (ME3616_PSM_TIMEOUT_HANDLE_EN)
APP_TIMER_DEF(m_psm_timer_id);        				///< PSM上报超时定时器
static void psm_timeout_handler(void * p_context);	///< PSM上报超时处理函数
#endif

#if (GNT_NB_TEST_EN == 1)
static void me3616_Test(NB_Handle handle);			///< 模组测试
#endif

// 上报出错超时注册计时器（适用于高频上报场景）
APP_TIMER_DEF(m_reportFail_regDelayTry_timer_id); 	///< 上报出错超时注册计时器，15min超时时间
static void reportFail_regDelayTry_timeout_handler(void * p_context);	///< 延缓注册超时处理函数


static void me3616_event_handle(NB_Handle handle); 	///< ME3616驱动全局事件处理函数

#if defined ME3616_LP_TEST
static void me3616_lpTest(NB_Handle handle); 		///< 模组低功耗测试
#endif

SemaphoreHandle_t Nb_sendCmd_Semaphore;				///< ME3616指令操作保护信号量
SemaphoreHandle_t Nb_Notify_Semaphore;				///< ME3616消息上报保护信号量


//******************************************************************************
/**@defgroup bsp_me3616 Bsp me3616 driver module.
* @{
* @ingroup bsp_drivers
*/
///< NB驱动功能函数列表
const NB_FxnTable ME3616_FxnTable = {
    // 通用操作函数
    .nbComOpen              = me3616_ComOpen,       ///< NB模组使用串口、定时器初始化及回调注册
    .nbComClose             = me3616_ComClose,      ///< NB模组使用串口关闭
    .nbSet_IotRegist        = me3616_Set_IotRegist, ///< 模组初始化设置及IOT平台注册（可用于重注册）
    .nbModuleInit           = me3616_init,          ///< 模组初始化（基本信息的获取、休眠的设置）
 
    .cmdSend                = me3616_send_cmd,      ///< 模组指令发送
    .nbRrcRelease           = me3616_RRC_Release,	///< 模组RRC链接释放，用于快速进入休眠
    .psmSet                 = me3616_psm_set,       ///< 模组PSM设置接口
    .getModuleInfo          = me3616_getModuleInfo, ///< 获取模组信息 IMSI、IMEI、 PSM设置、 BAND、 固件版本号
    .getSign                = me3616_getSignal,		///< 获取信号强度

    .nbWakeUp               = me3616_WakeUp,		///< 模组唤醒
    .nbReset                = me3616_Reset,         ///< 模组复位
    .nbPowerOn              = me3616_PowerOn,       ///< 模组上电
    .nbPowerOff             = me3616_PowerOff,      ///< 模组关机
    
    // 电信平台oceanconnect操作函数
    .oceanconnectRegist     = me3616_oceanconnect_regist,	///< oceanconnect平台注册
    .oceanconnectDeregist   = me3616_oceanconnect_deregist,	///< oceanconnect平台去注册
//    .oceanconnectSend       = me3616_oceanconnect_send,
    .oceanconnectRply       = me3616_oceanconnect_rply,		///< oceanconnect平台应答
    
    // 移动平台onenet操作函数
    .onenetRegist           = me3616_onenet_regist,			///< onenet平台注册
    .onenetDeregist         = me3616_onenet_deregist,		///< onenet平台去注册
//    .onenetNotify           = me3616_onenet_notify,
    .onenetUpdate           = me3616_onenet_update,			///< onenet平台更新注册
    
    .nbNotify               = me3616_notify,				///< NB模组消息上报
    
#if (ME3616_GPS_EN) 
    // GNSS定位操作函数
    .gpsInit                = me3616_gps_init,				///< 模组GPS功能初始化
    .gpsRun                 = me3616_gps_run,				///< 模组GPS启动定位
    .gpsStop                = me3616_gps_stop, 				///< 模组GPS停止定位
#endif
    
    // 模组主事件处理函数（数据操作），放在任务中循环
    .eventHandle            = me3616_event_handle,			///< ME3616驱动全局事件处理函数
    
    // 低功耗测试
    #if defined ME3616_LP_TEST
    .lpTest                 = me3616_lpTest,
    #endif
    
    #if (GNT_NB_TEST_EN == 1)
    .nbTest                 = me3616_Test,
    #endif
};
/** @} bsp_me3616*/

/* function prototypes ------------------------------------------------------*/
static void me3616_receCb(char* buf, uint16_t len);   	///< 串口接收数据回调
//static void me3616_timeoutCb(void);                   ///< 定时器超时回调
//static void nbsend_msg_app(NB_Handle handle, uint8_t**buf,uint8_t isOk);   ///< 应用回调函数，向应用层发送指令参数


#if (GNT_NB_TEST_EN == 1)
static void me3616_Test(NB_Handle handle)
{}
#endif


//******************************************************************************
/**@brief 打开me3616使用的uart串口
* @param[in]  handle 			NB模组驱动句柄
* @see :: ME3616_FxnTable :: me3616_ComClose
*/
static void me3616_ComOpen(NB_Handle handle)
{
	if(g_me3616_info.me3616_comOpen_flag == 0)
	{
		//注册串口接收回调函数
		handle->hw_fxnTablePtr->com_openFxn(me3616_receCb, handle->hw_fxnTablePtr->baudrate);
		g_me3616_info.me3616_comOpen_flag = 1;
		//注册定时器超时回调函数
//		handle->hw_fxnTablePtr->initTimerFxn(me3616_timeoutCb);
		NRF_LOG_INFO("uart open");
	}
}

/**
* @brief 关闭me3616使用的uart串口
* @param[in]  handle 			NB模组驱动句柄
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
* @brief me3616单次平台注册 \n
* 模组单次复位、初始化和注册
* @param[in]  handle 			NB模组驱动句柄
* @param[in]  lifetime 			物联网平台生存时间
* @param[in]  IfReset 			注册是否复位，0：不复位，1：复位
* @return  返回指令执行结果
* - NB_IOT_REGIST_SUCCESS  	 执行成功
* - NB_IOT_REGIST_FAILED     执行失败
* @see :: me3616_Set_IotRegist
*/
static uint8_t me3616_Set_IotRegist_once(NB_Handle handle, uint32_t lifetime, uint8_t IfReset)
{
    uint8_t ret;
    EventBits_t uxBits;
    
    if(g_me3616_info.me3616_comOpen_flag == 0)	//避免进入PSM导致串口关闭
        me3616_ComOpen(handle);
    
nb_restart: 
//    memset(&g_me3616_info, 0, sizeof(ME3616_info_t) );  //清信息、状态
    
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
        
        me3616_Reset(handle);   //硬件引脚复位
        NRF_LOG_INFO("ME3616 reset over");
    }
	
	// 等待自动获取IP(超时等待60s)
    uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_GETIP_EVENTBIT, pdTRUE, pdFALSE, 60000);
	
    if((uxBits & NB_GETIP_EVENTBIT) == NB_GETIP_EVENTBIT)
    {
        g_me3616_info.me3616_register_status = 1;   //基站网络附着状态
        NRF_LOG_INFO("NB get ip OK");
    }
    else
    {
        NRF_LOG_INFO("NB get ip timeout");
        return NB_IOT_REGIST_FAILED;
    }
    
    if(IfReset == 1)
        me3616_send_cmd(handle, "ATE0", 2000, 2);       //禁止回显
    
//******************************获取模组基础信息************************************************* 
    if(NB_ACK_OK != handle->nb_fxnTablePtr->getModuleInfo(handle, 2, 0, NULL))   // 获取设备信息
        return NB_IOT_REGIST_FAILED;
    
//***********************************低功耗设置************************************************* 
    if(g_me3616_info.me3616_psmisset == 0)  //PSM没有设置，设置PSM （TAU = 6小时  Active-Time = 30s）
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
    
    if(g_me3616_info.me3616_zslr == 0)  //休眠开关没有打开，设置休眠
    {
        ret = me3616_send_cmd(handle, "AT+ZSLR=1", 2000, 1);    //设置指令重启模组生效
        ret = me3616_send_cmd(handle, "AT+SETWAKETIME=20", 2000, 2);
        if(ret == NB_ACK_OK)
        {
            NRF_LOG_INFO("ZSLR set OK");
            IfReset = 1;    //需重启
            goto nb_restart;
        }
        else
        {
            NRF_LOG_INFO("ZSLR set ERROR %d", ret);
            return NB_IOT_REGIST_FAILED;
        }
    }
    
    ret = me3616_send_cmd(handle, "AT*MNBIOTEVENT=1,1", 2000, 2); //使能PSM状态主动上报
    
    // NB测试模式不进行注册
    #if (GNT_NB_TEST_EN == 1)
    return NB_IOT_REGIST_SUCCESS;
    #endif
    
//***********************************平台注册*************************************************    
    // 电信Oceanconnect平台注册 AT+M2MCLINEW=180.101.147.115,5683,"868613030006275",90
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
        else if(g_me3616_info.me3616_band == BAND_8) // 移动OneNet平台注册
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

/**@brief me3616平台注册 \n
* me3616复位、初始化设置、平台注册
* @param[in]  handle 			NB模组驱动句柄
* @param[in]  lifetime 			物联网平台生存时间
* @param[in]  regist_try 		注册出错重试次数
* @return  返回指令执行结果
* - NB_IOT_REGIST_SUCCESS  	 执行成功
* - NB_IOT_REGIST_FAILED     执行失败
* @see :: ME3616_FxnTable :: me3616_init
*/
static uint8_t me3616_Set_IotRegist(NB_Handle handle, uint32_t lifetime, uint8_t regist_try)
{
    uint8_t ret;
    g_me3616_info.iot_regist_status = 2;    //注册中
    
//    vTaskDelay(5000);	//初次开机如需复位最低时间间隔
    ret = me3616_Set_IotRegist_once(handle, lifetime, 1);   //首次注册不复位
    if( ret != NB_IOT_REGIST_SUCCESS )   
    {
		#if (ME3616_SIGNAL_BAD_HANDLE_EN)
		if(g_me3616_info.me3616_rssi > ME3616_SIGNAL_BAD_THRESHOLD)	//信号较差（<10），增加单次注册时间，减少注册次数
		{
			regist_try = ME3616_SIGNALBAD_REG_TRY_TIMES;
		}
		#endif
        do{
            ret = me3616_Set_IotRegist_once(handle, lifetime, 1);   //再注册复位
        }while ( (regist_try--) && (ret != NB_IOT_REGIST_SUCCESS) );
    }
    
    if(ret == NB_IOT_REGIST_SUCCESS)
    {
        g_me3616_info.iot_regist_status = 1;    //平台注册成功标志
    }
    else
        g_me3616_info.iot_regist_status = 0;    //未注册或注册失败
    return ret;
}

/**@brief me3616开机初始化
* @param[in]  handle 			NB模组驱动句柄
* @param[in]  lifetime 			物联网平台生存时间
* @param[in]  regist_enabled 	是否使能平台（即标签是否使能）
* @param[in]  regist_try 		注册出错重试次数
* @return  返回指令执行结果
* - NB_ACK_OK  	 执行成功
* - NB_ACK_ERROR 执行错误
* - NB_ACK_TIMEOUT 执行超时
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_init(NB_Handle handle, uint32_t lifetime, uint8_t regist_enabled, uint8_t regist_try)
{
    uint8_t ret;
    // ME3616模组使用IO初始化
    nrf_gpio_cfg_output(NB_PWRON);  //设置NB_PWRON输出
    nrf_gpio_pin_set(NB_PWRON);
    nrf_gpio_cfg_output(NB_RESET);
    nrf_gpio_pin_set(NB_RESET);
//    nrf_gpio_cfg_output(NB_WKUP);
//    nrf_gpio_pin_set(NB_WKUP);
    
    Me3616EventGroup = xEventGroupCreate();         //me3616驱动使用事件组 (configUSE_16_BIT_TICKS为0  事件组24位)
	Me3616AppEventGroup = xEventGroupCreate();    	//me3616应用驱动使用事件组（由于事件位较多需分开使用）
    vSemaphoreCreateBinary(Nb_sendCmd_Semaphore);   //NB指令操作保护信号量
    vSemaphoreCreateBinary(Nb_Notify_Semaphore);    //通知锁
    
    //上报出错超时注册计时器，15min超时时间
    app_timer_create(&m_reportFail_regDelayTry_timer_id, APP_TIMER_MODE_SINGLE_SHOT, reportFail_regDelayTry_timeout_handler);
	
	#if (ME3616_PSM_TIMEOUT_HANDLE_EN)	//PSM上报超时处理
	app_timer_create(&m_psm_timer_id, APP_TIMER_MODE_SINGLE_SHOT, psm_timeout_handler);
	#endif
    
    me3616_ComOpen(handle); //开机初始化使能串口
    g_me3616_info.regist_lifetime = lifetime;
    xEventGroupClearBits(Me3616EventGroup, NB_CPIN_EVENTBIT|NB_GETIP_EVENTBIT);
    me3616_PowerOn(handle);     //开机，拉低NB_PWRON引脚1s
    if(regist_enabled == NB_REGIST_DISABLE) //标签没有使能，不注册仅获取模组基础信息
    {
        // 等待模组通信、物联网卡就绪
        EventBits_t uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_CPIN_EVENTBIT, pdTRUE, pdFALSE, 10000);
        if((uxBits & NB_CPIN_EVENTBIT) == NB_CPIN_EVENTBIT)
        {
            NRF_LOG_INFO("CPIN ready");
            if(NB_ACK_OK != me3616_send_cmd(handle, "ATE0", 3000, 1))   //禁止回显
            {
                ret = handle->nb_fxnTablePtr->getModuleInfo(handle, 1, 1, 5);
            }
            else
            {
                // 获取模组基础信息
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
    else if(regist_enabled == NB_REGIST_ENABLE)    //标签使能，注册云平台
    {
        vTaskDelay(5000);	//初次开机如需复位最低时间间隔
        ret = me3616_Set_IotRegist(handle, lifetime, regist_try);
    } 
    return ret;
}


/**@brief NB模组唤醒
* @param[in]  handle 	NB模组驱动句柄
* @see :: ME3616_FxnTable
*/
static void me3616_WakeUp(NB_Handle handle)
{
    me3616_ComOpen(handle);	//如果串口关闭，需要打开串口
    
    if(NB_ACK_OK == me3616_send_cmd(handle, "AT", 1000, 1)) //已唤醒
    {
        /* 模组已唤醒 */
    }
    else    //通过NB_PWRON唤醒
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
                vTaskDelay(5000);   // 不延时指令会出错
//                me3616_send_cmd(handle, "AT+M2MCLISEND=00", 1000, 1);   // 发送空数据退出PSM
//                me3616_send_cmd(handle, "AT", 1000, 2)
            }
        }
    }
}

/**@brief NB模组复位
* @param[in]  handle 	NB模组驱动句柄
* @see :: ME3616_FxnTable
*/
static void me3616_Reset(NB_Handle handle)
{
    me3616_WakeUp(handle);      		//唤醒状态下才能reset
    NRF_LOG_INFO("me3616 reset");
    nrf_gpio_pin_write(NB_RESET, 0);    //硬件引脚复位
    vTaskDelay(500);
    nrf_gpio_pin_write(NB_RESET, 1);
    g_me3616_info.me3616_powerOn_flag = 1;
}

/**@brief NB模组开机
* @param[in]  handle 	NB模组驱动句柄
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

/**@brief NB模组关机
* @param[in]  handle 	NB模组驱动句柄
* @see :: ME3616_FxnTable :: me3616_PowerOn
*/
static void me3616_PowerOff(NB_Handle handle)
{
    #if 0
    me3616_WakeUp(handle);
    if(NB_ACK_OK != me3616_send_cmd(handle, "AT+ZTURNOFF", 1000, 2))
    {
        // 软关机失败采用硬件关机
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

/**@brief NB模组主动释放RRC连接
* @param[in]  handle 	NB模组驱动句柄
* @see :: ME3616_FxnTable
*/
static void me3616_RRC_Release(NB_Handle handle)
{
    me3616_send_cmd(handle, "AT*MNBIOTRAI=1", 2000, 1);
}


/**@brief ME3616的PSM设置，改参数掉电保存(内部调用)
* @param[in]  handle 	NB模组驱动句柄
* @param[in]  t3412 	TAU定时器
* @param[in]  t3324 	Active定时器
* @return  返回指令执行结果
* - NB_ACK_OK  	 执行成功
* - NB_ACK_ERROR 执行错误
* - NB_ACK_TIMEOUT 执行超时
* @par 示例:
* @code
*	// TAU = 6小时  Active-Time = 30s
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

/**@brief 获取模组信息 IMSI、IMEI、 PSM设置、 BAND、 固件版本号(内部调用)
* @param[in]  handle 	NB模组驱动句柄
* @param[in]  InfoType 	获取信息的类型
* - 1：仅IMEI、IMSI
* - 2：PSM、ZSLR、BAND、GMR(不能重启)
* @return  返回指令执行结果
* - NB_ACK_OK  	 执行成功
* - NB_ACK_ERROR 执行错误
* - NB_ACK_TIMEOUT 执行超时
* @see :: me3616_getNbInfo
*/
static uint8_t me3616_getNbInfo(NB_Handle handle, uint8_t InfoType)
{
    uint8_t ret;
    // 查询国际移动台设备标识IMSI
//    ret = me3616_send_cmd(handle, "ATI", 2000, 0);

	// 查询产品序列号IMEI   869662030455593
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
    // 查询国际移动台设备标识IMSI  460111172164930
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
        // 查询PSM是否设置  回复  +CPSMS: 0
        ret = me3616_send_cmd(handle, "AT+CPSMS?", 2000, 1);
        if(ret == NB_ACK_OK)
        {
            char* pColon = strchr(gNBReceBuf.Buf,':'); 
            if(pColon)
            {
                pColon = pColon+2;
                g_me3616_info.me3616_psmisset = (*pColon - 0x30);    //详见手册说明
            }
        }
        // 查询休眠功能是否开启 +ZSLR:0
        ret = me3616_send_cmd(handle, "AT+ZSLR?", 2000, 1);
        if(ret == NB_ACK_OK)
        {
            char* pColon = strchr(gNBReceBuf.Buf,':'); 
            if(pColon)
            {
                pColon = pColon+1;
                g_me3616_info.me3616_zslr = (*pColon - 0x30);    //详见手册说明
            }
        }
        // 查询BAND  *MBAND: 5
        ret = me3616_send_cmd(handle, "AT*MBAND?", 2000, 1);
        if(ret == NB_ACK_OK)
        {
            char* pColon = strchr(gNBReceBuf.Buf,':'); 
            if(pColon)
            {
                pColon = pColon+2;
                g_me3616_info.me3616_band = (*pColon - 0x30);    //详见手册说明
            }
//            if((g_me3616_info.me3616_band!=5)&&(g_me3616_info.me3616_band!=8)&&(g_me3616_info.me3616_band!=3))
//                return NB_ACK_ERROR;
        }
        else
            return NB_ACK_ERROR;
        
        // 查询模组固件版本号  ME3616G1AV0.0B01
        ret = me3616_send_cmd(handle, "AT+GMR", 2000, 1);
        if(ret == NB_ACK_OK)
        {
            memcpy(g_me3616_info.me3616_SwRevision, gNBReceBuf.Buf, 16);
        }
        // 查询信号强度
        ret = me3616_send_cmd(handle, "AT+CSQ", 2000, 1);
        if(ret == NB_ACK_OK)
        {
            char* pColon = strchr(gNBReceBuf.Buf,':'); 
            if(pColon)
            {
                pColon = pColon+2;
                g_me3616_info.me3616_rssi = (uint8_t)strtol(pColon, NULL, 10);   //数值转换详见手册说明
            }
        }
        
        // 电信平台数据发送和上报模式设置 +M2MCLICFG: 0,0
//        if(g_me3616_info.me3616_band == 5)
//        {
//            ret = me3616_send_cmd(handle, "AT+M2MCLICFG?", 2000, 1);
//            if(ret == NB_ACK_OK)
//            {
//                char* pColon = strchr(gNBReceBuf.Buf,',');
//                if(pColon)
//                {
//                    pColon = pColon+1;
//                    g_me3616_info.oceanconnect_data_mode = (*pColon - 0x30);    //详见手册说明
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

/**@brief 获取模组信息 \n
* 信息包含：IMSI、IMEI、 PSM设置、 BAND、 固件版本号
* @param[in]  handle 	NB模组驱动句柄
* @param[in]  InfoType 	获取信息的类型
* - 1：仅IMEI、IMSI
* - 2：PSM、ZSLR、BAND、GMR(不能重启)
* @param[in]  IfReset 	是否需要重启  0：不重启（重试次数无效）， 1：重启
* @param[in]  err_try 	出错重试次数
* @return  返回指令执行结果
* - NB_ACK_OK  	 执行成功
* - NB_ACK_ERROR 执行错误
* - NB_ACK_TIMEOUT 执行超时
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
            me3616_Reset(handle);   //硬件引脚复位
            NRF_LOG_INFO("ME3616 reset over");
            xEventGroupWaitBits(Me3616EventGroup, NB_CPIN_EVENTBIT, pdTRUE, pdFALSE, 10000);
            ret = me3616_send_cmd(handle, "ATE0", 3000, 1); //禁止回显
            if(ret == NB_ACK_OK)
            {
                ret = me3616_getNbInfo(handle, InfoType);   // 获取模组基础信息
            }
        }while ( (err_try--) && (ret != NB_ACK_OK) );
        
        if(ret == NB_ACK_OK)
            g_me3616_info.me3616_info_collection_flag = 1;
    }
    return ret;    
}

/**@brief 获取模组RSSI信号强度
* @param[in]  handle 	NB模组驱动句柄
* @return  返回指令执行结果
* - NB_ACK_OK  	 执行成功
* - NB_ACK_ERROR 执行错误
* - NB_ACK_TIMEOUT 执行超时
* @par 示例:
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
		me3616_send_cmd(handle, "AT+M2MCLISEND=00", 3000, 2);   // 发送空数据退出PSM
	}
	do{
		// 查询信号强度 +CSQ: 17,99
		ret = me3616_send_cmd(handle, "AT+CSQ", 2000, 1);
		if(ret == NB_ACK_OK)
		{
			char* pColon = strchr(gNBReceBuf.Buf,':'); 
			if(pColon)
			{
				pColon = pColon+2;
				g_me3616_info.me3616_rssi = (uint8_t)strtol(pColon, NULL, 10);   //数值转换详见手册说明
				if(g_me3616_info.me3616_rssi < 32)
				{
					g_me3616_info.me3616_rssi = 113 - (g_me3616_info.me3616_rssi*2);
				}
				else
				{
					g_me3616_info.me3616_rssi = 255;	//没有信号
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


/**@brief ME3616单次指令发送函数
* @param[in]  handle 	NB模组驱动句柄
* @param[in]  cmd 		指令内容字符串指针
* @param[in]  waittime 	指令响应超时时间，单位ms
* @return  返回指令执行结果
* - NB_ACK_OK  	 执行成功
* - NB_ACK_ERROR 执行错误
* - NB_ACK_TIMEOUT 执行超时
* @see :: me3616_send_cmd
*/
static uint8_t me3616_send_cmd_once(NB_Handle handle, char *cmd, uint16_t waittime)
{
    EventBits_t uxBits;
    
    memset(&gNBReceBuf, 0, sizeof(ReceBuf_t)); //接收清空
    // 字符串指针不能用sizeof， 字符串推荐使用strlen函数，同时使用snprintf时长度记得+1
    snprintf(gNBSendBuf.Buf, strlen((char*)cmd)+3, "%s\r\n", cmd); //添加结束位\r\n
    
    xEventGroupClearBits(Me3616EventGroup, NB_ACK_OK_EVENTBIT|NB_ACK_ERROR_EVENTBIT);
    handle->hw_fxnTablePtr->com_sendFxn((uint8_t*)gNBSendBuf.Buf, strlen((char*)cmd)+2);   //发送指令
    NRF_LOG_INFO("%s", gNBSendBuf.Buf); //发送命令监测
//    handle->hw_fxnTablePtr->startTimerFxn(waittime);    //启动命令超时计时器
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

/**@brief ME3616指令发送函数
* @param[in]  handle 	NB模组驱动句柄
* @param[in]  cmd 		指令内容字符串指针
* @param[in]  waittime 	指令响应超时时间，单位ms
* @param[in]  cmd_try 	出错重试次数
* @return  返回指令执行结果
* - NB_ACK_OK  	 执行成功
* - NB_ACK_ERROR 执行错误
* - NB_ACK_TIMEOUT 执行超时
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
/**@brief 处理ME3616异步返回的通知
* @param[in]  buf 	消息内容指针
* @param[in]  len 	消息长度
* @return 
* - 0  是异步通知
* - !0 不是异步通知
* @see :: me3616_receCb
*/
static uint8_t me3616_AsyncNotification(char* buf, uint16_t len)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t isAsync = false;
    char* position_addr_start = NULL;   //字符串起始地址 
    
    if(g_me3616_info.me3616_getip == 0) //ME3616自动附网还没有完成
    {
        if((position_addr_start = strstr(buf, "*MATREADY: 1")) != NULL) //串口准备好，可收发AT
        {
            g_me3616_info.me3616_matready = 1;
            return true;
        }
        if((position_addr_start = strstr(buf, "+CFUN: 1")) != NULL)     //射频正常
        {
            g_me3616_info.me3616_cfun = 1;
            return true;
        }
        if((position_addr_start = strstr(buf, "+CPIN: READY")) != NULL) //卡正常
        {
            if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_CPIN_EVENTBIT, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            g_me3616_info.me3616_cpin = 1;
            return true;
        }
        if((position_addr_start = strstr(buf, "+IP:")) != NULL)         //自动获取IP 地址
        {
            if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_GETIP_EVENTBIT, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            g_me3616_info.me3616_getip = 1;
            return true;
        } 
    }
    else    //附网完成之后的异步通知处理
    {
        // +CPIN: READY
        if((position_addr_start = strstr(buf, "+CPIN: READY")) != NULL) //唤醒退出PSM
        {
            if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_CPIN_EVENTBIT, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            return true;
        }
//        if((position_addr_start = strstr(buf, "+IP:")) != NULL)         //自动获取IP 地址
//        {
//            if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_GETIP_EVENTBIT, &xHigherPriorityTaskWoken))
//            {
//                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//            }
//            g_me3616_info.me3616_getip = 1;
//            return true;
//        }
        // +CEREG: 0,1
//        if((position_addr_start = strstr(buf, "+CEREG")) != NULL)       //网络注册状态更新
//        {
//            char* pColon = strchr(buf,','); //,1后的是网络注册状态
//            if(pColon)
//            {
//                pColon++;
//                g_me3616_info.me3616_register_status = (*pColon - 0x30);    //详见手册说明
//            }
//            
//            return true;
//        }
        
        
        //移动onenet平台
        if((g_me3616_info.me3616_band == BAND_8) && (g_me3616_info.me3616_iot_platform == ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT))
        {
            // +MIPLEVENT 模组状态上报    
            if((position_addr_start = strstr(buf, "+MIPLEVENT")) != NULL)  //上报模组当前状态
            {
                // onenet唤醒时状态
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
            // +MIPLOBSERVE OneNET平台向模组发起observe请求
            // +MIPLOBSERVE: 0, 69171, 1, 10250, 0, -1
            if((position_addr_start = strstr(buf, "+MIPLOBSERVE")) != NULL)  //observe请求
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
            // +MIPLDISCOVER OneNET平台向模组发起discover请求
            // +MIPLDISCOVER: 0, 65310, 3303
            if((position_addr_start = strstr(buf, "+MIPLDISCOVER")) != NULL)  //discover请求
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
            // +MIPLREAD OneNET平台向模组发起read请求
            // +MIPLREAD: 0, 638, 3300, 0, 5750
            if((position_addr_start = strstr(buf, "+MIPLREAD")) != NULL)    //read请求
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
            // +MIPLWRITE OneNET平台向模组发起write请求
            // +MIPLWRITE: 0, 65316, 3300, 0, 5750, 1, 4, HTGD, 0, 0
            if((position_addr_start = strstr(buf, "+MIPLWRITE")) != NULL)   //write请求
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
        else 	//电信oceanconnect平台
        {
            // +M2MCLI:register success
            if((position_addr_start = strstr(buf, "+M2MCLI:")) != NULL)  //上报模组当前状态
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
            if((position_addr_start = strstr(buf, "+M2MCLIRECV")) != NULL)  //平台接收数据上报
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
        
        // PSM状态监测
        if((position_addr_start = strstr(buf, "ENTER PSM")) != NULL)       //进入PSM
        {
            g_me3616_info.me3616_enterPSM_flag = 1;
            if(pdPASS == xEventGroupSetBitsFromISR(Me3616AppEventGroup, NB_ENTERPSM_EVENTBIT, &xHigherPriorityTaskWoken))
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            return true;
        }    
        if((position_addr_start = strstr(buf, "EXIT PSM")) != NULL)       //退出PSM
        {
            g_me3616_info.me3616_enterPSM_flag = 0;
            return true;
        }
    }
    
    #if (ME3616_GPS_EN) 
    // GNSS数据分析
    // $GPGLL,4004.74005,N,11614.19613,E,060845.00,A,A*5B<CR><LF>
    // $GNGLL,,,,,,V,N*7A
    if(g_me3616_info.gnss_run_state == 1)
    {
        if((position_addr_start = strstr(buf, "$GNGLL")) != NULL) //串口准备好，可收发AT
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


/**@brief ME3616串口数据接收回调函数 \n
* 对消息进行分类，异步消息交由:: me3616_AsyncNotification 进行处理
* @param[in]  buf 	接收的数据缓存地址
* @param[in]  len 	数据长度
* @see :: ME3616_FxnTable
*/
static void me3616_receCb(char* buf, uint16_t len)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if(len > NB_UART_RECE_BUF_MAX_LEN)  //超出接收缓存长度
        return;
    
    buf[len] = '\0';
    NRF_LOG_INFO("%s", buf);
    
#if 1
    if(!me3616_AsyncNotification(buf, len))  //非异步消息(即设置、执行、读取指令)
    {
        if((gNBReceBuf.len + len) < NB_UART_RECE_BUF_MAX_LEN)
        {
            memcpy(gNBReceBuf.Buf + gNBReceBuf.len, buf, len);
            gNBReceBuf.len += len;
        }
        if(strstr((char*)buf,"OK"))
        {
            if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_ACK_OK_EVENTBIT, &xHigherPriorityTaskWoken))        //置CMD操作OK事件位
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
        }
        else if (strstr((char*)buf,"ERROR"))
        {
            if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_ACK_ERROR_EVENTBIT, &xHigherPriorityTaskWoken))     //置CMD操作错误事件位
            {
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
        }
    }
    else
    {
        memset(&gNBReceBuf, 0, sizeof(ReceBuf_t));   //是异步消息，清空
    }
#endif
}


// me3616计时用超时处理函数
//static void me3616_timeoutCb(void)
//{
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    if(pdPASS == xEventGroupSetBitsFromISR(Me3616EventGroup, NB_ACK_TIMEOUT_EVENTBIT, &xHigherPriorityTaskWoken))    //置超时事件标志
//    {
//        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//    }
//}




/*******************电信OceanConnect平台操作函数*****************************************************************************/
/**@brief 电信平台注册
* @param[in]  handle 	NB模组驱动句柄
* @param[in]  lifetime 	平台生存时间，模组会定期更新注册
* @return  函数执行结果
* - NB_IOT_REGIST_SUCCESS  	执行成功
* - NB_IOT_REGIST_FAILED  	执行失败
* @par 示例:
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
    ret = me3616_send_cmd(handle, cmd_data, 2000, 3);  // 电信平台注册
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

/**@brief 电信平台去注册
* @param[in]  handle 	NB模组驱动句柄
* @return  函数执行结果
* - NB_ACK_OK  	执行成功
* - Others  	执行失败
* @par 示例:
* @code
*	AT+M2MCLIDEL
* @endcode
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_oceanconnect_deregist(NB_Handle handle)
{
    uint8_t ret;
    ret = me3616_send_cmd(handle, "AT+M2MCLIDEL", 1000, 1);  // 电信平台去注册
    return ret;
}

#if 1
/**@brief 电信平台命令下发的回复
* @param[in]  handle    	NB模组驱动句柄
* @param[in]  cmd    		要回复的命令指针
* @return  函数执行结果
* - NB_RPLY_SUCCESS  	执行成功
* - NB_RPLY_FAIL  	执行失败
* @par 示例:
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
    
    ret = me3616_send_cmd(handle, cmd, 2000, 2); //发送上报数据
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
/**@brief 电信平台发送数据
* @param[in]  handle    	NB模组驱动句柄
* @param[in]  data    		要发送数据指针
* @param[in]  len  			要发送数据长度
* @return  函数执行结果
* - NB_OCEANCONNECT_SEND_SUCCESS  	执行成功
* - NB_OCEANCONNECT_SEND_FAIL  	执行失败
* @par 示例:
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


/*******************移动OneNet平台操作函数*****************************************************************************/

/**@brief 获取onenet通信消息的message id
* @param[in]  *data    	原始指令数据指针
* @param[out] *mid 		message id指针
* @par 示例:
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

/**@brief 解析onenet的write请求的数据
* @param[in]  *data    	原始指令数据指针
* @param[in]  *len    	原始指令数据长度指针
* @param[out] *appdata 	解析数据指针
* @par 示例:
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
    strtok(NULL,",");   //1 字符串
    
    strncpy(len, strtok(NULL,",")+1, 4); //4 长度
    strcpy(appdata, strtok(NULL,",")+1);
}

/**@brief onenet注册初始流程
* @param[in]  handle    	NB模组驱动句柄
* @param[in]  lifetime    	平台生存时间，模组会定期更新注册
* @return  函数执行结果
* - NB_IOT_REGIST_SUCCESS  	执行成功
* - NB_IOT_REGIST_FAILED  	执行失败
* - Others  	执行失败
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_onenet_regist(NB_Handle handle, uint32_t lifetime)
{
    uint8_t ret;
    EventBits_t uxBits;
    char cmd_data[50];
    
    // 创建OneNET instance
    ret = me3616_send_cmd(handle, "AT+MIPLCREATE", 2000, 2);
    if(ret != NB_ACK_OK)
        return NB_IOT_REGIST_FAILED;
    // 新增object id (一个实例 resource id 5750	Application Type RW)
    ret = me3616_send_cmd(handle, "AT+MIPLADDOBJ=0,3300,1,\"1\",1,0", 2000, 2);
    if(ret != NB_ACK_OK)
        return NB_IOT_REGIST_FAILED;
    // 设备注册到OneNET平台 AT+MIPLOPEN=0,3600
    xEventGroupClearBits(Me3616EventGroup, NB_ONENET_REG_SUCCESS_EVENTBIT|NB_ONENET_OBSERVE_REQ_EVENTBIT|NB_ONENET_DISCOVER_REQ_EVENTBIT);  //清事件位
    snprintf(cmd_data, 50, "AT+MIPLOPEN=0,%d", lifetime);
    ret = me3616_send_cmd(handle, cmd_data, 2000, 2);
    if(ret != NB_ACK_OK)
        return NB_IOT_REGIST_FAILED;
    
    // 120s等待注册成功
    uxBits = xEventGroupWaitBits(Me3616EventGroup, NB_ONENET_REG_SUCCESS_EVENTBIT, pdTRUE, pdFALSE, 120000);
    if((uxBits & NB_ONENET_REG_SUCCESS_EVENTBIT) == NB_ONENET_REG_SUCCESS_EVENTBIT) //onenet注册成功
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
        
//        me3616_onenet_update(handle, lifetime, 2); //更新注册
        
        return NB_IOT_REGIST_SUCCESS;
    }
    else
    {
        return NB_IOT_REGIST_FAILED;
    }  
}

/**@brief 移动平台更新注册
* @param[in]  handle    	NB模组驱动句柄
* @param[in]  lifetime    	更新平台生存时间
* @param[in]  update_try  	失败重试次数
* @return  函数执行结果
* - NB_ACK_OK  	执行成功
* - Others  	执行失败
* @par 示例:
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
    
    if(g_me3616_info.me3616_enterPSM_flag == 1) //模组已休眠，需要唤醒
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
    
    if(ret != NB_ONENET_UPDATE_SUCCESS) //更新失败重新注册
    {
        ret = me3616_Set_IotRegist(handle, g_me3616_info.regist_lifetime, 2);   //平台注册
        if(ret != NB_IOT_REGIST_SUCCESS)    //再次尝试注册失败
        {
            return NB_ONENET_UPDATE_FAIL;
        }
    }
    
    return NB_ONENET_UPDATE_SUCCESS;
}

/**@brief 移动平台去注册
* @param[in]  handle    	NB模组驱动句柄
* @return  函数执行结果
* - NB_ACK_OK  	执行成功
* - Others  	执行失败
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_onenet_deregist(NB_Handle handle)
{
    uint8_t ret;
    ret = me3616_send_cmd(handle, "AT+MIPLCLOSE=0", 2000, 1);  // ONENET平台去注册
    return ret;
}


#if(ME3616_PSM_TIMEOUT_HANDLE_EN)
/**@brief 进入PSM超时处理函数
*/
static void psm_timeout_handler(void * p_context)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(pdPASS == xEventGroupSetBitsFromISR(Me3616AppEventGroup, NB_PSM_ENTER_TIMEOUT_EVENTBIT, &xHigherPriorityTaskWoken))        //置PSM进入超时事件位
    {
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}
#endif

static uint8_t nb_reportFail_delayTry_flag = 0;    ///< 延时注册启动标志

/**@brief 上报失败延时重新注册超时处理函数
* @ref NB_REPFAIL_REG_DELAY_TRY 出错延缓重试，在延迟期间如果正常则重新延缓，适用于高频率上报（上报失败重新注册超时15min）
* @see :: me3616_notify
*/
static void reportFail_regDelayTry_timeout_handler(void * p_context)
{
    nb_reportFail_delayTry_flag = 2;
}

/**@brief NB模组向云平台上报数据
* @param[in]  handle  			NB模组驱动句柄
* @param[in]  *data    			上报数据指针
* @param[in]  len    			上报数据长度
* @param[in]  rcc_enabled  		上报时是否主动释放RCC链接
* @param[in]  update_enabled    上报时是否更新注册(只适用于onenet)
* @param[in]  report_fail_try_type	上报失败重新注册类型 \n
* @ref NB_REPFAIL_REG_TRY 出错立即重试	\n
* @ref NB_REPFAIL_REG_DELAY_TRY 出错延缓重试，在延迟期间如果正常则重新延缓，适用于高频率上报（上报失败重新注册超时15min） \n
* @ref NB_REPFAIL_REG_NO_TRY 出错不重试
* @return  函数执行结果
* - NB_NOTIFY_SUCCESS  	上报成功
* - NB_NOTIFY_FAIL		上报失败
* - NB_IOT_REGIST_FAILED 注册失败返回
* - Others  其他错误
* @par 示例:
* @code
*	移动平台发送数据 AT+MIPLNOTIFY=0,122553,3308,0,5900,4,4,50,0,0
*	电信平台发送数据 AT+M2MCLISEND=000101
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
	app_timer_stop(m_psm_timer_id);	// 停止PSM超时计时
	#endif
	
    xSemaphoreTake( Nb_Notify_Semaphore,  600000);    //最多等待10分钟
//    if(g_me3616_info.me3616_enterPSM_flag == 1) //模组已休眠，需要唤醒
//    {
//        me3616_WakeUp(handle);
//    }
    me3616_WakeUp(handle);  //为保证周期上报可靠，每次做唤醒操作（避免PSM标志失效的情况）
    
	if(g_me3616_info.me3616_enterPSM_flag == 1)
	{
		me3616_send_cmd(handle, "AT+M2MCLISEND=00", 2000, 1);   // 发送空数据退出PSM
	}

//    me3616_getSignal(handle);
	
    //上报之前，平台之前没有注册成功，再次注册(适用于低频周期上报的可靠性要求高的上报场景)
#if 1	//功耗优化版，尽量减少NB通信时间
    if((g_me3616_info.iot_regist_status == 0) && (report_fail_try_type == NB_REPFAIL_REG_TRY))
    {
        ret = me3616_Set_IotRegist(handle, g_me3616_info.regist_lifetime, 3);   //平台注册
        if(ret != NB_IOT_REGIST_SUCCESS)    //再次尝试注册失败
        {
            xSemaphoreGive( Nb_Notify_Semaphore );
            return ret;
        }
    }
#endif
	
    //移动OneNet上报附带更新注册（更新失败会重新注册）
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
    // 命令组成
    if((g_me3616_info.me3616_band == BAND_8) && (g_me3616_info.me3616_iot_platform == ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT)) //移动上报
    {
        snprintf(cmd_data, 220, "AT+MIPLNOTIFY=0,%s,3300,0,5750,1,%d,%s,0,0", gNBOnenetMsg.observemsgid, len, notify_data);
    }
    else    //电信上报
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
	
    ret = me3616_send_cmd(handle, cmd_data, 3000, 3); //发送上报数据
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
    
    if(notify_result == NB_NOTIFY_FAIL) //上报失败，需要重新注册
    {
        if( (report_fail_try_type == NB_REPFAIL_REG_TRY) || (nb_reportFail_delayTry_flag == 2) )  //失败重新注册（适用于低频、可靠场景）
        {
            ret = me3616_Set_IotRegist(handle, g_me3616_info.regist_lifetime, 2);   //上报失败重新平台注册
            if(ret == NB_IOT_REGIST_SUCCESS)
            {
                if((g_me3616_info.me3616_band == BAND_8) && (g_me3616_info.me3616_iot_platform == ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT))
                {
                    // OneNet重新注册msgid会变更
                    snprintf(cmd_data, 220, "AT+MIPLNOTIFY=0,%s,3300,0,5750,1,%d,%s,0,0", gNBOnenetMsg.observemsgid, len, notify_data);
//                    if(rcc_enabled == 1)  //移动测试RRC不成功
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
        else if(report_fail_try_type == NB_REPFAIL_REG_DELAY_TRY)  //失败超时重新注册（适用于高频上报场景）
        {
            if(nb_reportFail_delayTry_flag == 0)
            {
                nb_reportFail_delayTry_flag = 1;    //超时注册开始计时标志
                app_timer_start(m_reportFail_regDelayTry_timer_id, 15*60*1000, NULL);   //GPS定位超时计时
            }
        }     
    }
    else    //上报成功
    {
        if(report_fail_try_type == NB_REPFAIL_REG_DELAY_TRY)
        {
            if(nb_reportFail_delayTry_flag != 0)
            {
                nb_reportFail_delayTry_flag = 0;
                app_timer_stop(m_reportFail_regDelayTry_timer_id);
            }
        }
        // RRC释放快速进入PSM
        if(rcc_enabled == NB_RRC_ENABLE)
        {
            me3616_RRC_Release(handle);
            me3616_send_cmd(handle, "AT+M2MCLISEND=00", 2000, 1);
        }
        
    }
    
    xSemaphoreGive( Nb_Notify_Semaphore );
	
	// PSM超时容错处理
	#if(ME3616_PSM_TIMEOUT_HANDLE_EN)
	app_timer_start(m_psm_timer_id, ME3616_PSM_TIMEOUT_VALUE, NULL);   //进入PSM超时计时
	#endif
	
    return notify_result;
}


/*******************GNSS操作函数*****************************************************************************/
#if (ME3616_GPS_EN) 

/**@brief GPS定位超时处理函数 (即需要静止保持120s就停止GPS定位)，通过事件位:: NB_GPS_POSITION_TIMEOUT_EVENTBIT
* 来同步:: me3616_event_handle
*/
static void gps_position_timeout_handler(void * p_context)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(pdPASS == xEventGroupSetBitsFromISR(Me3616AppEventGroup, NB_GPS_POSITION_TIMEOUT_EVENTBIT, &xHigherPriorityTaskWoken))        //置CMD操作OK事件位
    {
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

/**@brief GPS数据接收超时处理函数，通过事件位:: NB_GPS_NMEADATA_TIMEOUT_EVENTBIT
* 来同步:: me3616_event_handle
*/
static void gpsRevData_timeout_handler(void * p_context)
{
    if(g_me3616_info.gnss_run_state == 1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if(pdPASS == xEventGroupSetBitsFromISR(Me3616AppEventGroup, NB_GPS_NMEADATA_TIMEOUT_EVENTBIT, &xHigherPriorityTaskWoken))        //置CMD操作OK事件位
        {
            portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        }
    }
}


#if 0
/**@brief AGPS主动获取辅助定位数据,先判断AGNSS数据是否有效，无效则开启下载
* @param[in]  handle    	NB模组驱动句柄
* @param[in]  zgdata_try    下载出错重试次数
* @return  函数执行结果
* - NB_AGPS_DATA_READY  获取成功
* - Others  获取失败
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_agps_zgdata(NB_Handle handle, uint8_t zgdata_try)
{
    uint8_t ret;
    EventBits_t uxBits;
    do{
        //查询AGNSS数据是否有效 +ZGDATA: READY
        ret = NB_AGPS_DATA_NOT_READY;
        if( me3616_send_cmd(handle, "AT+ZGDATA?", 2000, 1) == NB_ACK_OK )
        {
            if(strstr(gNBReceBuf.Buf, "+ZGDATA: READY") == NULL)    //AGNSS数据失效（NOT READY）
            {
                xEventGroupClearBits(Me3616AppEventGroup, NB_AGPS_DATAREADY_EVENTBIT);
                if(me3616_send_cmd(handle, "AT+ZGDATA", 1000, 1) != NB_ACK_OK)   //发起下载AGPS数据,命令错误表示GPS冷启动未完成
                {
                    vTaskDelay(10000);  //下载错误，延时等待冷启动完成
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

/**@brief GPS功能初始化
* @details 设置GPS定位模式为AGPS，使能自动获取定位辅助信息
* @param[in]  handle    NB模组驱动句柄
* @return  函数执行结果
* - NB_ACK_OK  	执行成功
* - Others  	执行失败
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_gps_init(NB_Handle handle)
{
    uint8_t ret;
    
    #if (BOARD_GNT_VERSION ==3)
    nrf_gpio_cfg_output(GPS_LNA_EN);  //设置NB_PWRON输出
    nrf_gpio_pin_write(GPS_LNA_EN, 0);
    #endif
    
    app_timer_create(&m_gps_position_timer_id, APP_TIMER_MODE_SINGLE_SHOT, gps_position_timeout_handler);   //定位超时
    app_timer_create(&m_gpsRevData_timer_id, APP_TIMER_MODE_SINGLE_SHOT, gpsRevData_timeout_handler);       //定位接收数据丢失超时
    
    
    // AGPS定位
#if 1
    if(g_me3616_info.gnss_run_mode == ME3616_GPS_MODE_AGPS)
    {   
        ret = me3616_send_cmd(handle, "AT+ZGMODE=1", 2000, 3);  //设置定位模式为AGPS
        ret = me3616_send_cmd(handle, "AT+ZGAUTO=1,10", 2000, 2);
    }
    else
    {
        ret = me3616_send_cmd(handle, "AT+ZGMODE=2", 2000, 3);  //设置定位模式为StandAlone
        ret = me3616_send_cmd(handle, "AT+ZGAUTO=0,10", 2000, 2);
    }
    
//    ret = me3616_send_cmd(handle, "AT+ZGNMEA=32", 2000, 2); //开启NMEA上报信息为GLL
//    
//    ret = me3616_agps_zgdata(handle, 2);
    
//    ret = me3616_send_cmd(handle, "AT+ZGRUN=1", 1000, 2);   //启动单次定位模式
//    ret = me3616_send_cmd(handle, "AT+ZGLOCK=0", 1000, 2);   //启动单次定位模式 
//    ret = me3616_send_cmd(handle, "AT+ZGTMOUT?", 1000, 1);   //查询单次定位超时时间
//    if(ret == NB_ACK_OK)
//    {
//        if(strstr(gNBReceBuf.Buf, "+ZGTMOUT: 60") == NULL)
//        {
//            ret = me3616_send_cmd(handle, "AT+ZGTMOUT=60", 1000, 2);    //单次定位超时时间60S
//        }
//    }
    
//    ret = me3616_send_cmd(handle, "AT+ZGPSR=1", 1000, 2);  //使能定位结果上报
    
#endif  

    // GPS定位
#if 0
    ret = me3616_send_cmd(handle, "AT+ZGMODE=2", 1000, 2);  //设置定位模式为StandAlone
    ret = me3616_send_cmd(handle, "AT+ZGNMEA=32", 2000, 2);  //开启NMEA上报信息为GLL
#endif  

    return ret;
}

/**@brief 开启GPS服务
* @details 每次启动定位最好重新设置定位模式、数据格式后在启动定位，采用连续定位模式
* @param[in]  handle    NB模组驱动句柄
* @return  函数执行结果
* - NB_ACK_OK  	执行成功
* - Others  	执行失败
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_gps_run(NB_Handle handle)
{
    uint8_t ret = NB_ACK_OK;
    
    if(g_me3616_info.gnss_run_state == 0)   //没有使能GPS定位
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
            nrf_gpio_pin_write(GPS_LNA_EN, 1);  //GPS LNA使能
            #endif
            
            g_me3616_info.gnss_run_state = 1;
            app_timer_stop(m_gps_position_timer_id);
            app_timer_start(m_gps_position_timer_id, 60000, NULL);   //GPS定位超时计时
            app_timer_stop(m_gpsRevData_timer_id);
            app_timer_start(m_gpsRevData_timer_id, 20000, NULL);
            
//            if(g_me3616_info.me3616_band == BAND_5) //电信需要手动进入PSM
//            {
//                me3616_RRC_Release(handle);
//                me3616_send_cmd(handle, "AT+M2MCLISEND=00", 2000, 1);
//            }
        }
    }
    else if(g_me3616_info.gnss_run_state == 1)  //定位已在运行，初始化GPS定位超时计时器
    {
        app_timer_stop(m_gps_position_timer_id);
        app_timer_start(m_gps_position_timer_id, 60000, NULL);   //GPS定位超时计时
    }

    return ret;
}

/**@brief 关闭GPS功能
* @details 通过指令停止GPS定位，操作失败则GPS热重启在此操作
* @param[in]  handle    NB模组驱动句柄
* @return  函数执行结果
* - NB_ACK_OK  	执行成功
* - Others  	执行失败
* @see :: ME3616_FxnTable
*/
static uint8_t me3616_gps_stop(NB_Handle handle)
{
    uint8_t ret;
    
    if(g_me3616_info.gnss_run_state == 1)
    {
        #if (BOARD_GNT_VERSION ==3)
        nrf_gpio_pin_write(GPS_LNA_EN, 0);  //GPS LNA使能
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
            me3616_send_cmd(handle, "AT+ZGRST=2", 2000, 2); //热启动GPS
            vTaskDelay(2000);
            me3616_send_cmd(handle, "AT+ZGRUN=0", 2000, 2);
            g_me3616_info.gnss_run_state = 0;
            Ble_log_send((uint8_t *)"stop_err", 8);
        }
    }
    
    return ret;
}
#endif


/**@brief ME3616全局事件处理函数
* @details 云平台数据接收事件和进入PSM事件，并传递给应用回调函数
* @param[in]  handle    NB模组驱动句柄
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
	
    
    if((uxBits & NB_ONENET_READ_REQ_EVENTBIT) == NB_ONENET_READ_REQ_EVENTBIT)   //onenet读请求 3300.0.5750
    {
        char cmd_data[100];
        me3616_onenet_getid(gNBDataReceBuf.Buf, gNBOnenetMsg.msgid);
        snprintf(cmd_data, 100, "AT+MIPLREADRSP=0,%s,1,3300,0,5750,1,4,HTGW,0,0", gNBOnenetMsg.msgid);
        me3616_send_cmd(handle, cmd_data, 1000, 2);
    }
    // +MIPLWRITE: 0, 65316, 3300, 0, 5750, 1, 4, @0A^, 0, 0
    if((uxBits & NB_ONENET_WRITE_REQ_EVENTBIT) == NB_ONENET_WRITE_REQ_EVENTBIT)   //onenet写请求 3300.0.5750
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
                handle->AppReceCB(MSG_ONENET_WRITE_REQ, app_data, app_len);//上报应用处理
        }
            
    }
    
    // +M2MCLIRECV:0200004030415E 电信平台命令下发（DL_data）
    if((uxBits & NB_OCEANCONNECT_RECV_EVENTBIT) == NB_OCEANCONNECT_RECV_EVENTBIT)   //oceanconnect数据接收
    {
        if(strstr(gNBDataReceBuf.Buf, "AAAA0000") != NULL)  //notify的平台回复
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
            taskENTER_CRITICAL();   //临界区保护中断改变数据
            app_len = strlen(gNBDataReceBuf.Buf);
            if(app_len < 60)
                memcpy(app_data, gNBDataReceBuf.Buf, app_len);
            taskEXIT_CRITICAL();
            NRF_LOG_INFO("Write Data len=%d, data=%s", app_len, app_data);
            if( (app_len < 60) && ((app_len > 10)) && (app_data[0] == '0') && (app_data[1] == '2') &&((app_len%2) == 0))
                handle->AppReceCB(MSG_OCEANCONNECT_DATA_RECE, app_data, app_len);//上报应用处理
        }
    }
    
    #if (ME3616_GPS_EN) 
    // $GNGLL,,,,,,V,N*7A       $GPGLL,4004.74005,N,11614.19613,E,060845.00,A,A*5B<CR><LF>
    if((uxBits & NB_GNSS_RECE_EVENTBIT) == NB_GNSS_RECE_EVENTBIT)       //GPS开启NMEA上报信息（上报应用解析）
    {
        app_timer_stop(m_gpsRevData_timer_id);
        
        char temp[60];
        uint16_t temp_len;
        
        taskENTER_CRITICAL();   //临界区保护中断改变数据
        temp_len = strlen(gNBGpsDataReceBuf.Buf);
        if(temp_len < 60)
        {
            memset(temp, 0, 60);
            memcpy(temp, gNBGpsDataReceBuf.Buf, temp_len);
        }
        taskEXIT_CRITICAL();
        
        if(temp_len < 60)
            handle->AppReceCB(MSG_GPS_GNSS_DATA_RECE, temp, temp_len);//上报应用处理
        
        if(g_me3616_info.gnss_run_state == 1)
            app_timer_start(m_gpsRevData_timer_id, 20000, NULL);
        
        xEventGroupClearBits(Me3616EventGroup, NB_GNSS_RECE_EVENTBIT);  //清除标志以保证标志置位是最新的
    }
    if((uxBits & NB_GPS_POSITION_TIMEOUT_EVENTBIT) == NB_GPS_POSITION_TIMEOUT_EVENTBIT) //定位超时
    {
        app_timer_stop(m_gpsRevData_timer_id);  //定位超时关闭NMEA检测定时器
        handle->AppReceCB(MSG_GPS_POSITION_TIMEOUT, NULL, NULL);    //上报应用处理
    }
    if((uxBits & NB_GPS_NMEADATA_TIMEOUT_EVENTBIT) == NB_GPS_NMEADATA_TIMEOUT_EVENTBIT)  //NMEA上报超时
    {
        handle->AppReceCB(MSG_GPS_NMEADATA_TIMEOUT, NULL, NULL);    //上报应用处理
    }
    #endif
    
    if((uxBits & NB_ENTERPSM_EVENTBIT) == NB_ENTERPSM_EVENTBIT) //模组进入PSM
    {
        #if (ME3616_GPS_EN) 
        if(g_me3616_info.gnss_run_state == 0)
        {
            me3616_ComClose(handle);    	// 如果进入PSM且GPS不工作，则关闭串口
			#if (ME3616_PSM_TIMEOUT_HANDLE_EN)
			app_timer_stop(m_psm_timer_id);	// 停止PSM超时计时
			#endif
        }
        #else
        me3616_ComClose(handle);        // 如果进入PSM，则关闭串口
		#if (ME3616_PSM_TIMEOUT_HANDLE_EN)
		app_timer_stop(m_psm_timer_id);	// 停止PSM超时计时
		#endif
        #endif
    }
	
	#if (ME3616_PSM_TIMEOUT_HANDLE_EN)
	if((uxBits & NB_PSM_ENTER_TIMEOUT_EVENTBIT) == NB_PSM_ENTER_TIMEOUT_EVENTBIT) //模组进入PSM超时
    {
        #if (ME3616_GPS_EN) 
        if(g_me3616_info.gnss_run_state == 0)
        {
            me3616_ComClose(handle);    //如果进入PSM超时且GPS不工作，则关闭串口
			me3616_PowerOff(handle);	//如果进入PSM超时，则模组关机
        }
        #else
        me3616_ComClose(handle);        //如果进入PSM超时，则关闭串口
		me3616_PowerOff(handle);		//如果进入PSM超时，则模组关机
        #endif
    }
	#endif
    
    
}


#if (OCEANCONNECT_DATA_MODE == HEX_MODE)
// 工具函数---------------------------------------------------------------------------------------
/**
* @brief 16进制字符串转字符串	\n
* @param[in]  *pSrc    	源地址
* @param[in]  nLen    	字符串的长度（即是pSrc的长度）
* @param[out] *pDest   	目的地址
* @see 16进制字符串转字符串 :: HexStrToStr
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
* @brief 16进制字符串转字符串	\n
* @param[in]  *pSrc    	源地址
* @param[in]  nLen    	要转换的长度，16进制字符串的长度/2（即是pDest的长度）
* @param[out] *pDest   	目的地址
* @see 字符串转16进制字符串 :: StrToHexStr
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



