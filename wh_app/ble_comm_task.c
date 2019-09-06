/**@file   	ble_comm_task.c
* @brief   	设备蓝牙通信协议处理任务
* @details  主要包含蓝牙数据的组帧、协议解析、协议处理及应答
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-09-20
* @version 	V1.0
* @copyright	Copyright (c) 2018-2020  江苏亨通光网科技有限公司
**********************************************************************************
* @attention
* 硬件平台:nRF52832_QFAA \n
* SDK版本：nRF5_SDK_15.0.0
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/09/20  <td>1.0      <td>wanghuan  <td>创建初始版本
* </table>
*
**********************************************************************************
*/

/**@defgroup ble_comm_task Wh ble_comm_task
* @{
* @brief   蓝牙数据通信协议处理任务模块.
* @details 主要包含蓝牙数据的组帧、协议解析、协议处理及应答. 
* @warning 目前最大接收帧长度100字节，可根据情况调节
*/

#include "gnt_includes.h"

TaskHandle_t    m_ble_comm_thread;          ///< 蓝牙通信任务句柄
void ble_comm_thread(void *pvParameters);	///< 蓝牙通信任务函数

xQueueHandle    BLE_MsgQueue;           	///< 蓝牙通信数据接收队列


static void ble_comm_rev_package(uint8_t * Data,uint16_t len);  ///< 数据接收组帧
static void ble_comm_handle(uint8_t * Data,uint16_t len);       ///< 数据处理

/**@brief 蓝牙通信任务
* @details 蓝牙数据的组帧、协议解析、协议处理及应答
*/
void ble_comm_thread(void *pvParameters)
{
    uint8_t temp_buffer[100];
    
    NRF_LOG_INFO("ble_comm_thread start.");
    
    while(1)
    {
        memset(temp_buffer, 0, 100);
        // 无限制等待使用portMAX_DELAY   100/portTICK_RATE_MS
        if( xQueueReceive( BLE_MsgQueue, temp_buffer, portMAX_DELAY ) == pdPASS ) 
        {
            NRF_LOG_HEXDUMP_INFO(&temp_buffer[1], temp_buffer[0]);
            ble_comm_rev_package(&temp_buffer[1], temp_buffer[0]);
        }
    }
}

/**@brief 蓝牙接收数据组包
* @param[in]  *Data 	接收蓝牙数据指针
* @param[in]  len 		接收蓝牙数据长度
*/
static void ble_comm_rev_package(uint8_t * Data,uint16_t len)
{
    uint16_t i;
    static uint16_t frameCount=0;   //组帧计数器
    static uint8_t buffer[100];     //数据帧长
    //根据静态变量frameCount组帧
    
    for(i=0;i<len;i++){
        switch(frameCount)
    {
        case 0:    //帧头第一个数据
        {
            if (Data[i]==0x7E)
            {
                buffer[0]=Data[i];
                frameCount++;
            }
            break;
        }
        case 1:    //帧头第2个数据
        {
            if (Data[i]==0x00)
            {
                buffer[1]=Data[i];
                frameCount++;
            }
            else  
            {
                if(Data[i]==0x7E)
                {
                    buffer[0]=Data[i];
                    frameCount=1;
                }
                else
                {
                    frameCount=0;
                }
            }
            break;
        }
        case 2:    //帧头第3个数据
        {
            if (Data[i]==0x10)
            {
                buffer[2]=Data[i];
                frameCount++;
            }
            else 
            { 
                if(Data[i]==0x7E)
                {
                    buffer[0]=Data[i];
                    frameCount=1;
                }
                else
                {
                    frameCount=0;
                }
            }
            break;
        }
        default:    //其他情况
        {
            //序号+预留+命令码+状态吗（暂时不做判断）
            if(frameCount>=2 && frameCount<=9)
            {
                buffer[frameCount]=Data[i];
                frameCount++;
                break;
            }
            if(frameCount>=10 && Data[i]!=0x7E) //还没有收到帧尾
            {
                buffer[frameCount]=Data[i];
                frameCount++;
                if(frameCount>=100) {frameCount=0;} //帧长超过限制，重新组帧
                break;
            }
            if(frameCount>=10 && Data[i]==0x7E) //收到帧尾
            {
                uint16_t rlen;
                buffer[frameCount]=Data[i];

                //接收一帧完毕，组建消息发送到I2数据处理任务
                rlen = CTin(buffer, frameCount+1);
                
                ble_comm_handle(buffer, rlen);
                
                frameCount=0;
                break;
            }
        }
    }
    }
}

/**@brief 蓝牙数据协议分析处理
* @param[in]  *Data 	组帧之后数据指针
* @param[in]  len 		组帧之后数据长度
*/
static void ble_comm_handle(uint8_t * Data,uint16_t len)
{
    uint16_t crc_v;         //用于CRC16校验计算
    uint8_t Ble_rply[60];   //用于接口协议通信回复
    
    memset(Ble_rply, 0, 60);  
    memcpy(Ble_rply, Data, 9); //复制帧前部分
    Ble_rply[9] = 0;   //状态码
    
    switch((uint16_t)(((uint16_t)(Data[8]<<8))|Data[7]))
    {
        case 0x7101:    //读标签加密验证码
        {
            
        }break;
        
        case 0x7102:    //读imei和imsi指令
        {
            Ble_rply[10] = 0x10;
            if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)
                Ble_rply[43] = 0x6C;
            else
                Ble_rply[43] = 0x00;
            
            if(g_me3616_info.me3616_info_collection_flag == 1)
            {
                Ble_rply[11] = 15;
                memcpy(&Ble_rply[12], g_me3616_info.me3616_IMEI, 15);
                Ble_rply[27] = 15;
                memcpy(&Ble_rply[28], g_me3616_info.me3616_IMSI, 15);
            }
            else
            {
                memset(&Ble_rply[27], 0, 16);
            }
            crc_v=GetCrc_16(Ble_rply, 47);
            Ble_rply[44]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[45]=crc_v>>8;
            Ble_rply[46]=0x7E;
            Ble_send(Ble_rply, 47);
        }break;
        
        case 0x7103:    //读标签告警和状态（电量+注册状态+井盖状态）
        {
            Ble_rply[10] = battery_adc_sample();
            if(g_gnt_flash_info.gnt_enable_flag != GNT_ENABLE)
            {
                Ble_rply[11] = 0xFF;
                Ble_rply[12] = 0x00;
            }
            else
            {
//                NRF_LOG_INFO("iot_regist_status = %d ",g_me3616_info.iot_regist_status);
                if(g_me3616_info.iot_regist_status == 1)
                    Ble_rply[11] = 0x00;
                else
                    Ble_rply[11] = 0xFF;
                
                if(nrf_gpio_pin_read(BUTTON_1) == 0)  //井盖状态
                    Ble_rply[12] = 0x03;
                else
                    Ble_rply[12] = 0x00;
            }
            
            crc_v=GetCrc_16(Ble_rply, 16);
            Ble_rply[13]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[14]=crc_v>>8;
            Ble_rply[15]=0x7E;
            Ble_send(Ble_rply, 16);    //应答
        }break;
        
        case 0x7104:    //读标签软硬件版本号
        {
            memcpy(&Ble_rply[10], DEVICE_HW_REVISION, 15);
            memcpy(&Ble_rply[26], DEVICE_SW_REVISION, 15);
            
            crc_v=GetCrc_16(Ble_rply, 45);
            Ble_rply[42]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[43]=crc_v>>8;
            Ble_rply[44]=0x7E;
            Ble_send(Ble_rply, 45);    //应答
        }break;
        
        case 0x7105:    //写标签启用标志(标签复位，重命名蓝牙名称，开启NB平台注册、搬动检测、GPS定位等全功能)
        {
            if(len<30)  //排除错误帧
            {
                Ble_rply[10] = 0x01;    //帧格式错误
            }
            else
            {
                char check_id[18];
                snprintf(check_id, 18, "%s", &Data[10]);
                if(strstr(check_id, (char*)g_me3616_info.me3616_IMEI) != NULL)    //校验ID号
                {
                    if(Data[27] == 0x00)    //标签出厂重置
                    {
                        my_fds_delete_file(GNT_FILE);
                        g_gnt_flash_info.gnt_enable_flag = GNT_DISABLE;
                        my_fds_write(GNT_FILE, GNT_ENABLE_FLAG_REC_KEY, &(g_gnt_flash_info.gnt_enable_flag), 1);
                        
                        Ble_rply[10] = 0;
                        crc_v=GetCrc_16(Ble_rply, 14);
                        Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
                        Ble_rply[12]=crc_v>>8;
                        Ble_rply[13]=0x7E;
                        Ble_send(Ble_rply, 14);
                        
                        vTaskDelay(2000);
                        
//                        if(g_me3616_info.me3616_powerOn_flag == 1)  //如果模组开机状态则关机
//                            nb_handle.nb_fxnTablePtr->nbPowerOff(&nb_handle);       //NB模组关机
                        
                        bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
                        
//                        if(g_gnt_info.gnt_usbvin_flag == 0)
//                            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);   //BLE关机
//                        else
                            NVIC_SystemReset();
                    }
                    else if(Data[27] == 0xFF)   //标签使能
                    {
                        g_gnt_info.gnt_battery_level = battery_adc_sample();
                        if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)
                        {
                            Ble_rply[10] = 0x04;    //标签已使能，无需再使能
                        }
                        else if(g_gnt_info.gnt_battery_level < GNT_ENABLE_BATTERY_THRESHOLD)
                        {
                            Ble_rply[10] = 0x05;    //标签电量低使能失败
                        }
                        else
                        {
                            g_gnt_flash_info.gnt_enable_flag = GNT_ENABLE;
                            my_fds_write(GNT_FILE, GNT_ENABLE_FLAG_REC_KEY, &(g_gnt_flash_info.gnt_enable_flag), 1);
                            
                            Ble_rply[10] = 0;
                            crc_v=GetCrc_16(Ble_rply, 14);
                            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
                            Ble_rply[12]=crc_v>>8;
                            Ble_rply[13]=0x7E;
                            Ble_send(Ble_rply, 14);
                            vTaskDelay(2000);
                            NVIC_SystemReset();
                        }
                    }
                }
                else
                {
                    Ble_rply[10] = 0x02;    //ID不匹配
                }
                
                #if 0
                if(Data[27] == 0x03)   //设置GPS定位模式为AGPS
                {
                    g_me3616_info.gnss_run_mode = g_gnt_flash_info.gnt_gps_mode_flag = ME3616_GPS_MODE_AGPS;
                    my_fds_write(GNT_FILE, GNT_GPS_MODE_REC_KEY, &(g_gnt_flash_info.gnt_gps_mode_flag), 1);
                    
                    Ble_rply[10] = 0;
                    crc_v=GetCrc_16(Ble_rply, 14);
                    Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
                    Ble_rply[12]=crc_v>>8;
                    Ble_rply[13]=0x7E;
                    Ble_send(Ble_rply, 14);
                    
                    lis3dh_power_off();
                    vTaskDelay(2000);
                    bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
                    NVIC_SystemReset();
                }
                else if(Data[27] == 0x04)   //设置GPS定位模式为StandAlone
                {
                    g_me3616_info.gnss_run_mode = g_gnt_flash_info.gnt_gps_mode_flag = ME3616_GPS_MODE_ALONE;
                    my_fds_write(GNT_FILE, GNT_GPS_MODE_REC_KEY, &(g_gnt_flash_info.gnt_gps_mode_flag), 1);
                    
                    Ble_rply[10] = 0;
                    crc_v=GetCrc_16(Ble_rply, 14);
                    Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
                    Ble_rply[12]=crc_v>>8;
                    Ble_rply[13]=0x7E;
                    Ble_send(Ble_rply, 14);
                    
                    lis3dh_power_off();
                    vTaskDelay(2000);
                    bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
                    NVIC_SystemReset();
                }
                else if(Data[27] == 0x05)   //使能GPS信息实时上报
                {
                    g_gnt_info.gnt_gps_realtime_report_flag = 1;
                    Ble_rply[10] = 0;
                }
                else if(Data[27] == 0x06)   //关闭GPS信息实时上报
                {
                    g_gnt_info.gnt_gps_realtime_report_flag = 0;
                    Ble_rply[10] = 0;
                }
                
                #endif
            }
            
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);
        }break;
        
        case 0x7106:    //读NB_Iot信号强度
        {
//            if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)
//                Ble_rply[10] = nb_handle.nb_fxnTablePtr->getSign(&nb_handle);
//            else
//                Ble_rply[10] = 99;
			nb_handle.nb_fxnTablePtr->getSign(&nb_handle);
			Ble_rply[10] = g_me3616_info.me3616_rssi;
			
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);    //应答
//            NRF_LOG_INFO("nb rssi = %d.", Ble_rply[10]);
        }break;
        
        case 0x7107:    //读NB_Iot平台注册状态
        {
            if(g_me3616_info.iot_regist_status == 1)
                Ble_rply[10] = 0x00;
            else
                Ble_rply[10] = 0xFF;
            
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);    //应答
        }break;
        
        case 0x7108:    //标签快速指示
        {
            ret_code_t err_code = bsp_indication_set(BSP_INDICATE_GNT_STATE_FAST);
            APP_ERROR_CHECK(err_code);
            Ble_rply[10] = 0x00;
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);
        }
        
        case 0x7109:    //清除标签移动标志
        {
            Ble_rply[10] = 0x00;    //标签未使能，不支持这条指令
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);    //应答
        }break;
        
        case 0x7111:    //终端GPS数据传递给标签
        {
            Ble_rply[10] = 0x00;
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);    //应答       
        }break; 
        
        case 0x7121:    //更改当前蓝牙广播时长
        {
            Ble_rply[10] = 0x00;
            if(Data[10] == 0xFF)            //蓝牙广播时长延长到十分钟
            {
                if(g_gnt_info.gnt_usbvin_flag == 1)
                    Ble_rply[10] = 0x01;
                else if(g_gnt_info.gnt_usbvin_flag == 0)
                    nRF52_FxnTable.bleSetAdvTimeout(10*60*1000);
            }
            else if(Data[10] == 0x00)       //广播时长恢复到一分钟
            {
                if(g_gnt_info.gnt_usbvin_flag == 1)
                    Ble_rply[10] = 0x01;
                else if(g_gnt_info.gnt_usbvin_flag == 0)
                    nRF52_FxnTable.bleSetAdvTimeout(40*1000);
            }
                
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);
        }break;
        
        
        case 0x7A01:    //标签功能配置（测试用） 1：不启动NB任务  2：NB初始化不启动周期任务
        {
            switch (Data[10])
            {
                // 标签使能清除
                case 0x00:
                {
                    my_fds_delete_file(GNT_FILE);
                    g_gnt_flash_info.gnt_enable_flag = GNT_DISABLE;
                    my_fds_write(GNT_FILE, GNT_ENABLE_FLAG_REC_KEY, &(g_gnt_flash_info.gnt_enable_flag), 1);
                    
                    Ble_rply[10] = 0;
                    crc_v=GetCrc_16(Ble_rply, 14);
                    Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
                    Ble_rply[12]=crc_v>>8;
                    Ble_rply[13]=0x7E;
                    Ble_send(Ble_rply, 14);
                    
                    vTaskDelay(2000); 
                    
    //                if(g_me3616_info.me3616_powerOn_flag == 1)  //如果模组开机状态则关机
    //                    nb_handle.nb_fxnTablePtr->nbPowerOff(&nb_handle);       //NB模组关机
                    
                    bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
                    
    //                if(g_gnt_info.gnt_usbvin_flag == 0)
    //                    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);   //BLE关机
    //                else
                        NVIC_SystemReset();
                }break;
                
                // 标签使能
                case 0xFF:
                {
                    g_gnt_info.gnt_battery_level = battery_adc_sample();
                    if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)
                    {
                        Ble_rply[10] = 0x04;    //标签已使能，无需再使能
                    }
                    else if(g_gnt_info.gnt_battery_level < GNT_ENABLE_BATTERY_THRESHOLD)
                    {
                        Ble_rply[10] = 0x05;    //标签电量低使能失败
                    }
                    else
                    {
                        g_gnt_flash_info.gnt_enable_flag = GNT_ENABLE;
                        my_fds_write(GNT_FILE, GNT_ENABLE_FLAG_REC_KEY, &(g_gnt_flash_info.gnt_enable_flag), 1);
                        
                        Ble_rply[10] = 0;
                        crc_v=GetCrc_16(Ble_rply, 14);
                        Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
                        Ble_rply[12]=crc_v>>8;
                        Ble_rply[13]=0x7E;
                        Ble_send(Ble_rply, 14);
                        vTaskDelay(2000);
                        NVIC_SystemReset();
                    }
                }break;
                
                // 配置标签进入可升级NB模组模式（需再次下发指令恢复）
                case 0x02:
                {
                    g_gnt_flash_info.gnt_enable_flag = GNT_FOR_NB_DFU;
                    my_fds_write(GNT_FILE, GNT_ENABLE_FLAG_REC_KEY, &(g_gnt_flash_info.gnt_enable_flag), 1);
                    
                    Ble_rply[10] = 0;
                    crc_v=GetCrc_16(Ble_rply, 14);
                    Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
                    Ble_rply[12]=crc_v>>8;
                    Ble_rply[13]=0x7E;
                    Ble_send(Ble_rply, 14);
                    
                    vTaskDelay(2000);
                    bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
                    NVIC_SystemReset();
                }break;
                
#if 0 
                // 设置GPS定位模式（0：AGPS， 1：StandAlone）
                case 0x03:
                {
                    if(Data[11] == 1)
                    {
                        g_me3616_info.gnss_run_mode = g_gnt_flash_info.gnt_gps_mode_flag = ME3616_GPS_MODE_ALONE;
                    }
                    else
                    {
                        g_me3616_info.gnss_run_mode = g_gnt_flash_info.gnt_gps_mode_flag = ME3616_GPS_MODE_AGPS;
                    }
                    my_fds_write(GNT_FILE, GNT_GPS_MODE_REC_KEY, &(g_gnt_flash_info.gnt_gps_mode_flag), 1);
                    
                    Ble_rply[10] = 0;
                    crc_v=GetCrc_16(Ble_rply, 14);
                    Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
                    Ble_rply[12]=crc_v>>8;
                    Ble_rply[13]=0x7E;
                    Ble_send(Ble_rply, 14);
                    
                    lis3dh_power_off();
                    vTaskDelay(2000);
                    bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
                    NVIC_SystemReset();
                }break;
                
                // 读取GPS定位模式（0：AGPS， 1：StandAlone）
                case 0x04:
                {
                    Ble_rply[10] = g_gnt_flash_info.gnt_gps_mode_flag;
                }break;
                
                // 设置GPS信息实时上报（0：取消， 1：使能）
                case 0x05:
                {
                    if(Data[11] == 1)
                    {
                        g_gnt_info.gnt_gps_realtime_report_flag = 1;
                    }
                    else
                    {
                        g_gnt_info.gnt_gps_realtime_report_flag = 0;
                    }
                    Ble_rply[10] = 0;
                }break;
                
                // 读取GPS信息实时上报标志
                case 0x06:
                {
                    Ble_rply[10] = g_gnt_info.gnt_gps_realtime_report_flag;
                }break;
#endif
                // 配置NB云平台(0:ME3616_IOT_PLATFORM_OCEANCONNECT, 1:ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT)
                case 0x07:
                {
                    if(len != 15)
                    {
                        Ble_rply[10] = 0xFF;
                    }
                    else
                    {
                        if(g_gnt_flash_info.gnt_iot_platform == Data[11])   //重复配置
                        {
                            Ble_rply[10] = 0x01;
                            break;
                        }
                        else
                        {
                            if(Data[11] == 0)
                            {
                                g_gnt_flash_info.gnt_iot_platform = ME3616_IOT_PLATFORM_OCEANCONNECT;
                            }
                            else
                            {
                                g_gnt_flash_info.gnt_iot_platform = ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT;
                            }
                            my_fds_write(GNT_FILE, GNT_IOT_PLATFORM_REC_KEY, &(g_gnt_flash_info.gnt_iot_platform), 1);
                            
                            Ble_rply[10] = 0;
                            crc_v=GetCrc_16(Ble_rply, 14);
                            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
                            Ble_rply[12]=crc_v>>8;
                            Ble_rply[13]=0x7E;
                            Ble_send(Ble_rply, 14);
                            
//                            lis3dh_power_off();
                            vTaskDelay(2000);
                            bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
                            NVIC_SystemReset();
                        }
                    }
                }break;
                
                // 读取NB云平台配置
                case 0x08:
                {
                    Ble_rply[10] = g_gnt_flash_info.gnt_iot_platform;
                }break;
                
                // 配置心跳上报周期(4字节小端)
                case 0x09:
                {
                    if(len != 18)
                    {
                        Ble_rply[10] = 0x01;
                    }
                    else
                    {
                        memcpy(&(g_gnt_flash_info.gnt_report_period), &Data[11], 4);
//                        g_gnt_flash_info.gnt_report_period = Data[11]|(Data[12]<<8)|(Data[13]<<16)|(Data[14]<<24);
                        if(g_gnt_flash_info.gnt_report_period < 60)
                        {
                            Ble_rply[10] = 0x02;
                            break;
                        }
                        else
                        {
                            my_fds_write(GNT_FILE, GNT_REPORT_PERIOD_REC_KEY, &(g_gnt_flash_info.gnt_report_period), 4);
                            Ble_rply[10] = 0;
                            
                            #if (GNT_WDT_EN == 1)   // 看门狗使能需要重启配置
                            crc_v=GetCrc_16(Ble_rply, 14);
                            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
                            Ble_rply[12]=crc_v>>8;
                            Ble_rply[13]=0x7E;
                            Ble_send(Ble_rply, 14);
                            vTaskDelay(2000);
                            bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
                            NVIC_SystemReset();
                            #else
                            gnt_change_report_period(g_gnt_flash_info.gnt_report_period);
                            #endif
                        }
                    }
                }break;
                
				// 读取配置心跳上报周期
                case 0x0A:
                {
                    memcpy(&Ble_rply[10], &(g_gnt_flash_info.gnt_report_period), 4);
                    crc_v=GetCrc_16(Ble_rply, 17);
                    Ble_rply[14]=(uint8_t)(crc_v & 0x00ff);
                    Ble_rply[15]=crc_v>>8;
                    Ble_rply[16]=0x7E;
                    Ble_send(Ble_rply, 17); 
                    return;
                }
				
                // 清除标签移动标志事件
                case 0x0B:
                {
					if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)
					{
						// 锁定当前GPS信息，上报告警消除
						#if (GNT_GPS_EN == 1)
						gnt_lock_location(g_gnt_info.gnt_last_latitude, g_gnt_info.gnt_last_longitude, 1);
						#endif
						
						xEventGroupSetBits(GntEventGroup, GNT_CLEAR_MOVEFLAG_EVENTBIT); //清除标签移动标志事件
						Ble_rply[10] = 0x00;
					}
					else
					{
						Ble_rply[10] = 0xFF;    	//标签未使能，不支持这条指令
					}
                }break;
				
				// 标签复位
                case 0x0C:
                {
					Ble_rply[10] = 0;
                    crc_v=GetCrc_16(Ble_rply, 14);
					Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
					Ble_rply[12]=crc_v>>8;
					Ble_rply[13]=0x7E;
					Ble_send(Ble_rply, 14);
					vTaskDelay(2000);
					bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
					NVIC_SystemReset();
                }break;
				
                // 主动进行一次周期上报（测试）
                case 0x0D:
                {
                    if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)
					{
						xEventGroupSetBits(GntEventGroup, NB_PRERIOD_REPORT_EVENTBIT); // 置周期上报事件
						Ble_rply[10] = 0x00;
					}
					else
					{
						Ble_rply[10] = 0xFF;    	// 标签未使能，不支持这条指令
					}
                }break;
                
				// 配置电量阈值
                case 0x0E:
                {
                    
                }break;
				
                // 读取电量阈值
                case 0x0F:
                {
                    
                }break;
				
				// 
                case 0x10:
                {
                    
                }break;
                
                default:
                {
                    Ble_rply[10] = 0xFE;    //不支持指令
                }break;
            }
            
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);         
        }break;    
        
        // 仅测试时使用（标签使能、初始化完成）
        case 0x7A02:    //加速度传感器中断阈值设置
        {
            
        }break;
        
        // 读取设备信息或配置
        // 01-> 设备使能标志
        // 02-> GPS
        // 03-> GPS定位模式
        // 04-> GPS实时上报状态
        // 05-> GPS定位是否使能
        case 0x7A03:    //读取设备信息或配置
        {
            Ble_rply[10] = gnt_temp_get();
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);
        }break;
        
        case 0x7A04:    //设备复位
        {
            Ble_rply[10] = 0;
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);
			vTaskDelay(2000);
            NVIC_SystemReset();
        }break;
    }
}




#if 0
/** 内部温度传感器********************************************************************************************************/
void gnt_temp_init(void)
{
    nrf_temp_init();
}

int32_t gnt_temp_get(void)
{
    int32_t temp;
    
    NRF_TEMP->TASKS_START = 1;  //开始温度测量
    
    while (NRF_TEMP->EVENTS_DATARDY == 0)   //等待温度测量完成
    {
        
    }
    NRF_TEMP->EVENTS_DATARDY = 0;   //请标志
    
    temp = ((nrf_temp_read()*10) / 4);   //转换为摄氏度
    
    NRF_TEMP->TASKS_STOP = 1;   //停止测量
    
    return temp;    
}
#endif



/** @} ble_comm_task*/

/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/



