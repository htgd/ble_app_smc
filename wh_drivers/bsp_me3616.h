/**@file    bsp_me3616.h
* @brief   	协议字节解析
* @details  IODH的数据帧中0x7E转变成2字节序列（0x7D，0x5E），数据帧中0x7D 转变成2字节序列（0x7D，0x5D）
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-8-17
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

/**@defgroup bsp_me3616 Bsp me3616 driver module.
* @{
* @ingroup bsp_drivers
* @brief 使用该驱动之前，先进行驱动句柄的实例注册. \n
* ME3616驱动支持云平台Onenet和OceanConnect \n
* 当使能GPS驱动使能时，支持GPS操作 
*/

#ifndef __BSP_ME3616_H
#define	__BSP_ME3616_H

#include <stdint.h>
#include "gnt_includes.h"

// <<< Use Configuration Wizard in Context Menu >>>\n

// <e> 驱动配置 - ME3616_EN
// <i> 驱动配置
// <i> 主要用来控制驱动的代码量
#define ME3616_EN   	1

// <e> GPS驱动使能 - ME3616_GPS_EN
// <i> 1：使能， 0：不使能
#define ME3616_GPS_EN   0
// </e>

// <e> notify增加应答机制 - ME3616_NOTIFY_NEED_RPLY_EN
// <i> notify收到平台应答后才判定上报成功(暂仅适用于OceanConnect平台)
#define ME3616_NOTIFY_NEED_RPLY_EN	1
// <o> notify收到平台应答超时时间(单位秒) - ME3616_NOTIFY_RPLY_TIMEOUT_VALUE <20-300><#*1000>
// <i> 判断notify收到平台应答超时的时间值
#define ME3616_NOTIFY_RPLY_TIMEOUT_VALUE	40000
// </e>

// <e> PSM超时处理 - ME3616_PSM_TIMEOUT_HANDLE_EN
// <i> 进入PSM超时，或PSM主动上报异常，或NB卡未设置PSM模式，则进行关机或其他处理
#define ME3616_PSM_TIMEOUT_HANDLE_EN	1
// <o> PSM超时时间(单位秒) - ME3616_PSM_TIMEOUT_VALUE <30-300><#*1000>
// <i> 判断进入PSM超时的时间值
#define ME3616_PSM_TIMEOUT_VALUE	200000
// </e>

// <e> 信号差处理 - ME3616_SIGNAL_BAD_HANDLE_EN
// <i> 信号差时对驱动某些通信过程进行优化处理（可能会影响上报成功几率）
#define ME3616_SIGNAL_BAD_HANDLE_EN		0
// <o> 获取信号强度重试次数 - ME3616_SIGNAL_TRY_TIMES <10-200>
// <i> 获取信号强度重试次数,每次均等待1s
#define ME3616_SIGNAL_TRY_TIMES	30
// <o> 信号差注册重试次数 - ME3616_SIGNALBAD_REG_TRY_TIMES <1-5>
// <i> 信号差注册重试次数
#define ME3616_SIGNALBAD_REG_TRY_TIMES	1
// <o> 信号差注册超时时间 - ME3616_SIGNALBAD_REG_TIMEOUT <60-200><#*1000>
// <i> 信号差注册超时时间单位秒
#define ME3616_SIGNALBAD_REG_TIMEOUT	90000
// <o> 信号强度差阈值 - ME3616_SIGNAL_TRY_TIMES <51-113>
// <i> 信号强度差阈值,255为没有信号，113为信号极低,计算公式 113-s*2（10 -> 93）
#define ME3616_SIGNAL_BAD_THRESHOLD	93
// </e>

// </e> 
//===========================================End of 功能配置

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

// NB默认云平台
#if (GNT_NB_DEFAULT_PLATFORM == 1)
#define ME3616_IOT_PLATFORM_OCEANCONNECT            0
#define ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT     1
#elif (GNT_NB_DEFAULT_PLATFORM == 2)
#define ME3616_IOT_PLATFORM_OCEANCONNECT            1
#define ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT     0
#endif

// BAND频段
#define BAND_3      3   ///< 联通
#define BAND_5      5   ///< 电信
#define BAND_8      8   ///< 移动


#define HEX_MODE    0
#define TEXT_MODE   1
#define OCEANCONNECT_DATA_MODE          HEX_MODE   ///< 支持数据通信采用text模式及字符串（AT+M2MCLICFG=1,1，ME3616C1AV1.0B07、ME3616G1AV0.0B01及以上版本支持）
#define OCEANCONNECT_REMOTE_INFO        "180.101.147.115,5683"	///< 平台地址信息


/**@enum NB_msg_types_t
* @brief 定义驱动上报应用消息类型
*/
typedef enum
{
    MSG_NONE            = 0,

#if (ME3616_GPS_EN)    
    MSG_GPS_GNSS_DATA_RECE,				///< 有gps定位数据
    MSG_GPS_POSITION_TIMEOUT,			///< gps定位超时
    MSG_GPS_NMEADATA_TIMEOUT,			///< gps获取数据超时
#endif
    
    MSG_OCEANCONNECT_DATA_RECE,			///< OCEANCONNECT 平台数据接收
    
    MSG_ONENET_READ_REQ,				///< ONENET 平台读请求
    MSG_ONENET_WRITE_REQ,				///< ONENET 平台写请求
    
    MSG_END
} NB_msg_types_t;


/**@enum NB_err_types_t
* @brief 定义指令操作应答错误类型
*/
typedef enum
{
    NB_ACK_NULL         = 0,
    NB_ACK_OK           = 1,			///< 应答成功
    NB_ACK_NOT_EXPECT   = 2,			///< 应答值不是期望值
    NB_ACK_ERROR        = 3,			///< 应答错误
    NB_ACK_TIMEOUT      = 4,			///< 应答超时
    NB_CMD_ERROR        = 5,			///< 命令错误
    
    NB_IOT_REGIST_SUCCESS,				///< 云平台注册成功
    NB_IOT_REGISTING,					///< 云平台注册中
    NB_IOT_REGIST_FAILED,				///< 云平台注册失败
    
    NB_ONENET_OBSERVE_WAIT_TIMEOUT,		///< onenet 观察注册超时
    NB_ONENET_DISCOVER_WAIT_TIMEOUT,	///< onenet 服务发现超时
    NB_ONENET_UPDATE_SUCCESS,			///< onenet 更新注册成功
    NB_ONENET_UPDATE_FAIL,				///< onenet 更新注册失败
    
    NB_NOTIFY_SUCCESS,	///< 上报平台指令成功
    NB_NOTIFY_FAIL,		///< 上报平台指令失败
    
    NB_RPLY_SUCCESS,	///< 应答平台指令成功
    NB_RPLY_FAIL,		///< 应答平台指令失败

#if (ME3616_GPS_EN)     
    NB_AGPS_DATA_READY,	///< GPS辅助数据就绪
    NB_AGPS_DATA_NOT_READY,	///< GPS辅助数据未就绪
#endif
    
} NB_err_types_t;


/**@struct ME3616_info_t
* @brief ME3616信息结构体 \n
* 定义存储ME3616的信息
*/
typedef struct
{
    uint8_t     me3616_matready;            ///< ME3616串口AT就绪
    uint8_t     me3616_cfun;                ///< ME3616射频状态
    uint8_t     me3616_cpin;                ///< ME3616物联网卡状态
    uint8_t     me3616_getip;               ///< ME3616附网注册获取IP状态
    
    uint8_t     me3616_SwRevision[17];      ///< ME3616模组固件版本号
    uint8_t     me3616_IMSI[16];            ///< 移动设备身份码
    uint8_t     me3616_IMEI[16];            ///< 模块序列号

    uint8_t     me3616_rssi;                ///< 模块接收信号强度（0~255）
    uint8_t     me3616_band;                ///< 当前NB网络BAND
    uint8_t     me3616_psmisset;            ///< PSM是否设置， 0：禁用PSM  1：使能PSM
    uint8_t     me3616_zslr;                ///< 休眠功能是否设置， 0：关闭休眠  1：打开休眠开关
    
    volatile uint8_t     me3616_powerOn_flag;       ///< 模组开机状态。0：关机，  1：开机     
    uint8_t     me3616_info_collection_flag;        ///< 模组基础信息采集完成标志
    volatile uint8_t     me3616_enterPSM_flag;      ///< 模组是否进入PSM，0：没有进入  1：进入PSM
    volatile uint8_t     me3616_comOpen_flag;       ///< 模组串口是否使能，0：串口关闭   1：串口打开
    
    uint8_t     iot_regist_status;          ///< 物联网平台注册状态
    uint8_t     me3616_register_status;     ///< 网络附着注册状态
    
    uint32_t    regist_lifetime;            ///< 平台注册生存时间
    
    uint8_t     me3616_iot_platform;        ///< NB云平台选择

#if (ME3616_GPS_EN) 
    volatile uint8_t     gnss_run_state;	///< gps运行状态
    uint8_t     agps_data_ready;			///< gps辅助定位数据是否就绪
    uint8_t     gnss_run_mode;				///< 给排水定位模式
#endif

//    volatile uint8_t     oceanconnect_data_mode;     ///< 电信平台数据通信模式，0：Hex mode， 1：Text mode
    
} ME3616_info_t;


///< NB模组对象驱动句柄类型
typedef struct NB_Conf *NB_Handle;

//******************************************************************************
/**@struct NB_FxnTable
* @brief 定义NB模组要实现的功能函数列表结构体
*/
typedef struct
{
    void (*nbComOpen)(NB_Handle);       ///< NB使用串口、定时器初始化及回调注册
    void (*nbComClose)(NB_Handle);      ///< NB使用串口关闭
    uint8_t (*nbSet_IotRegist)(NB_Handle, uint32_t, uint8_t);       ///< NB模组初始化设置及IOT平台注册
    uint8_t (*nbModuleInit)(NB_Handle, uint32_t, uint8_t, uint8_t); ///< 对NB模块初始化操作
    
    uint8_t (*cmdSend)(NB_Handle, char *, uint16_t, uint8_t);   	///< NB指令发送与响应检查
    void (*nbRrcRelease)(NB_Handle);                ///< NB主动释放RRC连接
    uint8_t (*psmSet)(NB_Handle, char *, char *);   ///< NB模组PSM设置
    uint8_t (*getModuleInfo)(NB_Handle, uint8_t, uint8_t, uint8_t); 	///< 获取NB模块的信息
    uint8_t (*getSign)(NB_Handle);                  ///< 获取NB模块信号强度
    
    void (*nbWakeUp)(NB_Handle);                ///< NB模组唤醒
    void (*nbReset)(NB_Handle);                 ///< 对NB模块复位操作
    void (*nbPowerOn)(NB_Handle);               ///< NB模组开机
    void (*nbPowerOff)(NB_Handle);              ///< NB模组关机
    
    // 电信平台oceanconnect操作函数
    uint8_t (*oceanconnectRegist)(NB_Handle, uint32_t);	///< 电信平台注册
    uint8_t (*oceanconnectDeregist)(NB_Handle);			///< 电信平台去注册
//    uint8_t (*oceanconnectSend)(NB_Handle, uint8_t*, uint16_t);
    uint8_t (*oceanconnectRply)(NB_Handle, char*);		///< 电信平台命令应答
    
    // 移动onenet平台操作函数
    uint8_t (*onenetRegist)(NB_Handle, uint32_t);		///< 移动平台注册
    uint8_t (*onenetDeregist)(NB_Handle);				///< 移动平台去注册
    uint8_t (*onenetUpdate)(NB_Handle, uint32_t, uint8_t);	///< 移动平台更新注册
    
    uint8_t (*nbNotify)(NB_Handle, uint8_t*, uint16_t, uint8_t, uint8_t, uint8_t);	///< 云平台通知上报
    
    // GPS操作函数
#if (ME3616_GPS_EN) 
    uint8_t (*gpsInit)(NB_Handle);	///< GPS功能初始化
    uint8_t (*gpsRun)(NB_Handle);	///< GPS启动定位
    uint8_t (*gpsStop)(NB_Handle);	///< GPS停止定位
#endif
    // 主任务函数
    void (*eventHandle)(NB_Handle);	///< NB驱动事件处理函数

    // 低功耗测试
    #if defined ME3616_LP_TEST
    void (*lpTest)(NB_Handle);
    #endif
    
    #if (GNT_NB_TEST_EN == 1)
    void (*nbTest)(NB_Handle);
    #endif
} NB_FxnTable;

extern const NB_FxnTable ME3616_FxnTable;	///< NB驱动功能函数列表
extern  ME3616_info_t    g_me3616_info;		///< ME3616的信息


///< 声明处理UART接收回调
typedef void (*nb_receive_cb)(char*, uint16_t);
///< 声明定时器超时回调
typedef void (*nb_timeout_cb)(void);


/**@struct HW_FxnTable
* @brief NB模组的串口与定时器硬件相关接口结构体
*/
typedef struct
{
    const uint32_t  baudrate;                       ///< 串口波特率
    void (*com_openFxn)(nb_receive_cb, uint32_t); 	///< 打开硬件串口
    void (*com_sendFxn)(uint8_t*,uint16_t);			///< 串口发送
    void (*com_closeFxn)(void);						///< 串口关闭

    void (*initTimerFxn)(nb_timeout_cb);			///< 定时器初始化
    void (*startTimerFxn)(uint32_t);				///< 启动定时器
    void (*stopTimerFxn)(void);						///< 关闭定时器
} HW_FxnTable;

typedef int (*NB_ReceCB)(NB_msg_types_t, char*, uint16_t);	///< 应用上报回调函数类型

//******************************************************************************
/**@struct NB_Config
* @brief NB模组的驱动句柄结构体
*/
typedef struct NB_Conf
{
    NB_FxnTable         *nb_fxnTablePtr;    ///< NB功能函数列表
    HW_FxnTable         *hw_fxnTablePtr;    ///< NB uart及定时器对像指针
    NB_ReceCB           AppReceCB;          ///< NB响应参数应用回调
} NB_Config;

extern NB_Config  nb_handle;				///< ME3616驱动句柄结构


/**@name ME3616驱动用事件标志组
* @brief ME3616驱动用事件标志组，用于高效同步
* @{
*/
///< 内部用事件组
extern EventGroupHandle_t   Me3616EventGroup;
#define NB_CPIN_EVENTBIT                            (1 << 0)   	///< NB开机NB卡就绪
#define NB_GETIP_EVENTBIT                           (1 << 1)    ///< 获取IP，NB模组附网就绪
#define NB_ACK_OK_EVENTBIT                          (1 << 2)    ///< NB指令操作OK
#define NB_ACK_ERROR_EVENTBIT                       (1 << 3)    ///< NB指令操作错误

#define NB_OCEANCONNECT_REG_SUCCESS_EVENTBIT        (1 << 4)    ///< 电信平台注册成功
#define NB_OCEANCONNECT_REG_FAILED_EVENTBIT         (1 << 5)    ///< 电信平台注册失败
#define NB_OCEANCONNECT_OB_SUCCESS_EVENTBIT         (1 << 6)    ///< 电信平台observe success
#define NB_OCEANCONNECT_NOTIFY_SUCCESS_EVENTBIT     (1 << 7)    ///< 电信平台通知成功
#define NB_OCEANCONNECT_NOTIFY_FAILED_EVENTBIT      (1 << 8)    ///< 电信平台通知失败
#define NB_OCEANCONNECT_NOTIFY_AND_RPLY_SUCCESS_EVENTBIT      (1 << 9)    ///< 电信平台通知且应答成功

#define NB_ONENET_CONNECT_SUCCESS_EVENTBIT          (1 << 10)  	///< 移动平台连接成功
#define NB_ONENET_REG_SUCCESS_EVENTBIT              (1 << 11)   ///< 移动平台注册成功
#define NB_ONENET_REG_FAILED_EVENTBIT               (1 << 12)   ///< 移动平台注册失败
#define NB_ONENET_OBSERVE_REQ_EVENTBIT              (1 << 13)   ///< 移动平台observe请求
#define NB_ONENET_DISCOVER_REQ_EVENTBIT             (1 << 14)   ///< 移动平台discover请求
#define NB_ONENET_UPDATE_SUCCESS_EVENTBIT           (1 << 15)   ///< 移动平台注册更新成功

///< 与应用回调同步用事件组
extern EventGroupHandle_t   Me3616AppEventGroup;
#define NB_ENTERPSM_EVENTBIT                        (1 << 1)    ///< 模组进入PSM
#define NB_OCEANCONNECT_RECV_EVENTBIT               (1 << 2)   	///< 电信平台接收到数据
#define NB_ONENET_READ_REQ_EVENTBIT                 (1 << 3)   	///< 移动平台read请求
#define NB_ONENET_WRITE_REQ_EVENTBIT                (1 << 4)   	///< 移动平台write请求
#define NB_GNSS_RECE_EVENTBIT                       (1 << 5)   	///< GNSS采集数据接收
#define NB_AGPS_DATAREADY_EVENTBIT                  (1 << 6)   	///< AGPS数据下载完成
#define NB_GPS_POSITION_TIMEOUT_EVENTBIT            (1 << 7)   	///< GPS定位超时
#define NB_GPS_NMEADATA_TIMEOUT_EVENTBIT            (1 << 8)   	///< GPS开启NMEA上报数据超时（10s）
#define NB_PSM_ENTER_TIMEOUT_EVENTBIT               (1 << 9)   	///< PSM进入超时

/**@} ME3616驱动用事件标志组 */



// 使能标志为宏定义
#define NB_REGIST_ENABLE            1   ///< 初始化使能平台注册
#define NB_REGIST_DISABLE           0	///< 初始化禁止平台注册

#define NB_RRC_ENABLE               1   ///< 上报使能RRC释放
#define NB_RRC_DISABLE              0	///< 上报禁止RRC释放

#define NB_UPDATE_ENABLE            1   ///< 上报使能更新注册
#define NB_UPDATE_DISABLE           0	///< 上报禁止更新注册

#define NB_REPFAIL_REG_TRY          1	///< 上报失败重试
#define NB_REPFAIL_REG_DELAY_TRY    2	///< 上报失败延迟重试
#define NB_REPFAIL_REG_NO_TRY       3	///< 上报失败不重试 


/**
* @brief 16进制字符串转字符串	\n
* @param[in]  *pSrc    	源地址
* @param[in]  nLen    	字符串的长度（即是pSrc的长度）
* @param[out] *pDest   	目的地址
* @see 16进制字符串转字符串 :: HexStrToStr
*/
void StrToHexStr(char *pDest, char *pSrc, uint16_t nLen);

/**
* @brief 16进制字符串转字符串	\n
* @param[in]  *pSrc    	源地址
* @param[in]  nLen    	要转换的长度，16进制字符串的长度/2（即是pDest的长度）
* @param[out] *pDest   	目的地址
* @see 字符串转16进制字符串 :: StrToHexStr
*/
void HexStrToStr(char *pDest, char *pSrc, uint16_t nLen);

#endif	/* #if (ME3616_EN) */
#endif	/* __BSP_ME3616_H */


/** @} bsp_me3616*/

/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/





