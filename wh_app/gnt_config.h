/**@file	gnt_config.h
* @brief   	��Ŀ���������ļ�
* @details  ��Ҫ����Э��Ӧ��ջ�����ܣ�main�������
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2019-01-10
* @version 	V1.0
* @copyright	Copyright (c) 2018-2020  ���պ�ͨ�����Ƽ����޹�˾
**********************************************************************************
* @attention
* Ӳ��ƽ̨:nRF52832_QFAA \n
* SDK�汾��nRF5_SDK_15.0.0
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/08/17  <td>1.0      <td>wanghuan  <td>������ʼ�汾
* </table>
*
**********************************************************************************
*/

/**@defgroup gnt_config Wh gnt_config
* @{
* @ingroup app_smc
* @brief   ��Ŀ���뼰�������ú꿪��.
* @details ��Ҫ���豸�汾���ơ�NBͨ�Ų������á�����ģ�������Լ�һЩ���Թ��ܿ���. \n
* @warning �������ò���ͬʱ���ã�����������ע��
*/

#ifndef __GNT_CONFIG_H
#define	__GNT_CONFIG_H


// <<< Use Configuration Wizard in Context Menu >>>\n


//  <h> �������� - Function config
//  <i> ��������
//  <i> ��Ҫ�������Ƶ���ģʽ�µ�һЩ����ѡ���������ӡ���ء������ջ����
//==========================================================

//  <h> �豸�汾����
//      <o> �豸�汾 - BOARD_GNT_VERSION
//      <i> Ӳ���汾
//      <i> ��ͬӲ���汾��ʹ�����Ż����в�ͬ�������������
//      <1=> BOARD_GNT_V1 
//      <2=> BOARD_GNT_V2 
//      <3=> BOARD_GNT_V3 
//      <s0.15> ����汾�� - DEVICE_SW_REVISION
//      <s1.15> Ӳ���汾�� - DEVICE_HW_REVISION
//  </h> 
#define BOARD_GNT_VERSION           3
#define DEVICE_SW_REVISION          "S02010041905301"
#define DEVICE_HW_REVISION          "HGTNBGNT0103001"


//  <h> ��ǩ���ܼ���������

//  <o> NBĬ����ƽ̨ - GNT_NB_DEFAULT_PLATFORM
//  <i> 1:NBĬ������BANDʹ��Oceanconnectƽ̨;
//  <i> 2:�ƶ�NB��ע��OneNet�����š���ͨNB��ע�����Oceanconnectƽ̨; 
//  <1=> NB_DEFAULT_PLATFORM_OCEANCONNECT 
//  <2=> NB_DEFAULT_PLATFORM_ONENET_OCEANCONNECT
#define GNT_NB_DEFAULT_PLATFORM  1

//  <o> ��ǩ�����ϱ����� - GNT_DEFAULT_REPORT_PERIOD
//  <i> �ϱ����ڵ�λms��ͨ��NB-IoT�ϱ�
//      <600000=> 10���ӣ����ڲ��ԣ�
//      <1800000=> 30���ӣ����ڲ��ԣ�
//      <3600000=> 1Сʱ 
//      <7200000=> 2Сʱ 
//      <10800000=> 3Сʱ 
//      <14400000=> 4Сʱ 
//      <18000000=> 5Сʱ 
//      <21600000=> 6Сʱ
//      <25200000=> 7Сʱ
//      <28800000=> 8Сʱ
//      <32400000=> 9Сʱ
//      <36000000=> 10Сʱ
//      <39600000=> 11Сʱ
//      <43200000=> 12Сʱ
#define GNT_DEFAULT_REPORT_PERIOD   (10800000)

// <o> �����ϱ�����ʧ�ܸ�λ - GNT_REPORT_CONTINUOUS_FAIL_TIMES_TO_RESET <1-10>
// <i> �����ϱ�����ʧ�ܸ�λ
#define GNT_REPORT_CONTINUOUS_FAIL_TIMES_TO_RESET    ( 4 ) 

//  <q> ���Ǹ澯���й��� - SMC_ALRAM_QUEUE_EN
//  <i> 1ʹ�ܾ��Ǹ澯���й��ܣ�0�رվ��Ǹ澯���й���
#define SMC_ALRAM_QUEUE_EN	0

//  <q> ���ǹرղ��ϱ� - SMC_CLOSE_REPORT_EN
//  <i> 1ʹ�ܾ��Ǹ澯���й��ܣ�0�رվ��Ǹ澯���й���
#define SMC_CLOSE_REPORT_EN	0

//  <e> ��ǩ���Ź�ʹ�� - GNT_WDT_EN
//      <o1> ���Ź�sleep��halt��Ϊ - GNT_WDT_CONFIG_BEHAVIOUR
//      <i> WDT behavior in CPU SLEEP or HALT mode
//      <1=> Run in SLEEP, Pause in HALT 
//      <8=> Pause in SLEEP, Run in HALT 
//      <9=> Run in SLEEP and HALT 
//      <0=> Pause in SLEEP and HALT
//      <o2> ���Ź�װ��ֵ - GNT_WDT_CONFIG_RELOAD_VALUE <#*3600000>
//      <i> WDT Reload value Сʱ
//      <o3> ���Ź��ж����ȼ�  - GNT_WDT_CONFIG_IRQ_PRIORITY
//      <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
//      <0=> 0 (highest) 
//      <1=> 1 
//      <2=> 2 
//      <3=> 3 
//      <4=> 4 
//      <5=> 5 
//      <6=> 6 
//      <7=> 7
//  </e>
#define GNT_WDT_EN   1
#define GNT_WDT_CONFIG_BEHAVIOUR   1
#define GNT_WDT_CONFIG_RELOAD_VALUE   (10800000)
#define GNT_WDT_CONFIG_IRQ_PRIORITY   7

//  <e> ��ǩ����Ƚ���ʹ�� - GNT_POF_EN
//  <i> <i> 1��ʹ������оƬ�����⣬ 0����ʹ��
#define GNT_POF_EN   0
//  </e>

//  <e> ��ǩ���ٶȴ�����ʹ�� - GNT_LIS3DH_EN
//  <i> 1ʹ�ü��ٶȴ������� 0���豸δ���ü��ٶȴ�����
#define GNT_LIS3DH_EN   0
//  </e>

//  <e> ��ǩGPSʹ�� - GNT_GPS_EN
//  <i> 1ʹ��GPS���ܣ� 0���豸δ����GPS����
#define GNT_GPS_EN   0
//  <o> GPSĬ�϶�λģʽ - GNT_GPS_DEFAULT_MODE
//  <i> 1:GPSĬ�Ϲ���ģʽGPS_ALONE,   2:  GPSĬ�Ϲ���ģʽAGPS
//  <i> �ò�������flash����
//  <1=> ME3616_GPS_MODE_ALONE 
//  <2=> ME3616_GPS_MODE_AGPS
#define GNT_GPS_DEFAULT_MODE  2
//  <q> GPSĬ��ʵʱ�ϱ�ʹ�� - GNT_GPS_DEFAULT_REPORT_EN
#define GNT_GPS_DEFAULT_REPORT_EN 1
//  </e>
//==End of ��ǩGPSʹ��

//  </h>
//=====End of ��ǩ���ܼ���������


//  <h> ����ͨ������

//  </h>
//=====End of ����ͨ������


//  <e> �̼����� - DFU_SUPPORT
//      <q1> DFU_COM
//      <i> 1ʹ��DFU���񼰹��ܣ�������ֱ�����أ���ͨ��DFU���³���  
//      <i> 0�ر�DFU���񼰹��ܣ������ڳ�����޸ĵ���
//  </e> 
#define DFU_SUPPORT     1
#define DFU_COM         0

//  <h> ���Թ���
//      <q> ������־��ӡ - GNT_BLE_LOG_EN
//      <i> 1ʹ��������ӡ��⣬0�ر�������ӡ���
#define GNT_BLE_LOG_EN          1

//      <q> �����ջ��� - TASK_HEAP_MONITOR_EN
//      <i> �����ջʹ�������⣬1->ʹ�ܣ�0->�ر�
#define TASK_HEAP_MONITOR_EN    0


//      <q> ���ٶȴ���������ģʽ - GNT_LIS3DH_TEST_EN
//      <i> 1��ʹ�ܸ�ģʽ�� 0����ʹ��
//      <i> ��ģʽ��NB����������������ڿ��ٲ���LIS3DH�����Ƿ�����
#define GNT_LIS3DH_TEST_EN  0

//      <q> NB�źŲ���ģʽ - GNT_NB_TEST_EN
//      <i> 1��ʹ�ܸ�ģʽ�� 0����ʹ��
//      <i> ��ģʽ����Ҫ����NB�����ע��������ź�ǿ�ȼ��
//      <i> ע�ⲻͬ����ģʽ����ͬʱʹ��
#define GNT_NB_TEST_EN  0

//  </h> 
//=====End of ���Թ���

//  </h> 
//===========================================End of ��������



// <h> ��ر���������ֵ
// <i> ������ֵ
// <i> ��ǩ������ڲ�ͬ����״̬�½���ʡ��򱣻�����
//==========================================================
// <o> GNT_ENABLE_BATTERY_THRESHOLD <1-20>
// <i> ��ǩʹ��������͵��������������ڸ�ֵʱ��ǩ����ʹ�ܣ�
// <i> �κ�ԭ��ĸ�λ���������ڸ�ֵʱ��ǩ�Զ��л�Ϊδʹ��״̬
#define GNT_ENABLE_BATTERY_THRESHOLD            ( 3 ) 

// <o> GNT_REPORT_DISABLE_BATTERY_THRESHOLD <1-50>
// <i> �ر�NB�ϱ���ص�����ֵ
#define GNT_REPORT_DISABLE_BATTERY_THRESHOLD    ( 1 ) 

// <o> GNT_REPORT_RESUME_BATTERY_THRESHOLD <1-50>
// <i> �ָ�NB�ϱ���ص�����ֵ
// <i> ��NB�ϱ��رպ󣬵�����ָ�һ��ֵ��ʹ���ϱ�
#define GNT_REPORT_RESUME_BATTERY_THRESHOLD     ( 15 ) 

// <o> GNT_GPS_DISABLE_BATTERY_THRESHOLD <5-50>
// <i> �ر�GPS��λ��ص�����ֵ
#define GNT_GPS_DISABLE_BATTERY_THRESHOLD       ( 15 ) 

// <o> GNT_GPS_RESUME_BATTERY_THRESHOLD <1-50>
// <i> �ָ�GPS��λ��ص�����ֵ
// <i> ��GPS��λ������͹رպ󣬵�����ָ�һ��ֵ�Żָ�GPS��λ����
#define GNT_GPS_RESUME_BATTERY_THRESHOLD        ( 25 ) 


// </h> 
//======================End of ��ر���������ֵ



// <<< end of configuration section >>>



#endif	/* __GNT_CONFIG_H */

/** @} gnt_config*/

/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/

