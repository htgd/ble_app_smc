/**@file   	ble_comm_task.c
* @brief   	�豸����ͨ��Э�鴦������
* @details  ��Ҫ�����������ݵ���֡��Э�������Э�鴦��Ӧ��
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-09-20
* @version 	V1.0
* @copyright	Copyright (c) 2018-2020  ���պ�ͨ�����Ƽ����޹�˾
**********************************************************************************
* @attention
* Ӳ��ƽ̨:nRF52832_QFAA \n
* SDK�汾��nRF5_SDK_15.0.0
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/09/20  <td>1.0      <td>wanghuan  <td>������ʼ�汾
* </table>
*
**********************************************************************************
*/

/**@defgroup ble_comm_task Wh ble_comm_task
* @{
* @brief   ��������ͨ��Э�鴦������ģ��.
* @details ��Ҫ�����������ݵ���֡��Э�������Э�鴦��Ӧ��. 
* @warning Ŀǰ������֡����100�ֽڣ��ɸ����������
*/

#include "gnt_includes.h"

TaskHandle_t    m_ble_comm_thread;          ///< ����ͨ��������
void ble_comm_thread(void *pvParameters);	///< ����ͨ��������

xQueueHandle    BLE_MsgQueue;           	///< ����ͨ�����ݽ��ն���


static void ble_comm_rev_package(uint8_t * Data,uint16_t len);  ///< ���ݽ�����֡
static void ble_comm_handle(uint8_t * Data,uint16_t len);       ///< ���ݴ���

/**@brief ����ͨ������
* @details �������ݵ���֡��Э�������Э�鴦��Ӧ��
*/
void ble_comm_thread(void *pvParameters)
{
    uint8_t temp_buffer[100];
    
    NRF_LOG_INFO("ble_comm_thread start.");
    
    while(1)
    {
        memset(temp_buffer, 0, 100);
        // �����Ƶȴ�ʹ��portMAX_DELAY   100/portTICK_RATE_MS
        if( xQueueReceive( BLE_MsgQueue, temp_buffer, portMAX_DELAY ) == pdPASS ) 
        {
            NRF_LOG_HEXDUMP_INFO(&temp_buffer[1], temp_buffer[0]);
            ble_comm_rev_package(&temp_buffer[1], temp_buffer[0]);
        }
    }
}

/**@brief ���������������
* @param[in]  *Data 	������������ָ��
* @param[in]  len 		�����������ݳ���
*/
static void ble_comm_rev_package(uint8_t * Data,uint16_t len)
{
    uint16_t i;
    static uint16_t frameCount=0;   //��֡������
    static uint8_t buffer[100];     //����֡��
    //���ݾ�̬����frameCount��֡
    
    for(i=0;i<len;i++){
        switch(frameCount)
    {
        case 0:    //֡ͷ��һ������
        {
            if (Data[i]==0x7E)
            {
                buffer[0]=Data[i];
                frameCount++;
            }
            break;
        }
        case 1:    //֡ͷ��2������
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
        case 2:    //֡ͷ��3������
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
        default:    //�������
        {
            //���+Ԥ��+������+״̬����ʱ�����жϣ�
            if(frameCount>=2 && frameCount<=9)
            {
                buffer[frameCount]=Data[i];
                frameCount++;
                break;
            }
            if(frameCount>=10 && Data[i]!=0x7E) //��û���յ�֡β
            {
                buffer[frameCount]=Data[i];
                frameCount++;
                if(frameCount>=100) {frameCount=0;} //֡���������ƣ�������֡
                break;
            }
            if(frameCount>=10 && Data[i]==0x7E) //�յ�֡β
            {
                uint16_t rlen;
                buffer[frameCount]=Data[i];

                //����һ֡��ϣ��齨��Ϣ���͵�I2���ݴ�������
                rlen = CTin(buffer, frameCount+1);
                
                ble_comm_handle(buffer, rlen);
                
                frameCount=0;
                break;
            }
        }
    }
    }
}

/**@brief ��������Э���������
* @param[in]  *Data 	��֮֡������ָ��
* @param[in]  len 		��֮֡�����ݳ���
*/
static void ble_comm_handle(uint8_t * Data,uint16_t len)
{
    uint16_t crc_v;         //����CRC16У�����
    uint8_t Ble_rply[60];   //���ڽӿ�Э��ͨ�Żظ�
    
    memset(Ble_rply, 0, 60);  
    memcpy(Ble_rply, Data, 9); //����֡ǰ����
    Ble_rply[9] = 0;   //״̬��
    
    switch((uint16_t)(((uint16_t)(Data[8]<<8))|Data[7]))
    {
        case 0x7101:    //����ǩ������֤��
        {
            
        }break;
        
        case 0x7102:    //��imei��imsiָ��
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
        
        case 0x7103:    //����ǩ�澯��״̬������+ע��״̬+����״̬��
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
                
                if(nrf_gpio_pin_read(BUTTON_1) == 0)  //����״̬
                    Ble_rply[12] = 0x03;
                else
                    Ble_rply[12] = 0x00;
            }
            
            crc_v=GetCrc_16(Ble_rply, 16);
            Ble_rply[13]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[14]=crc_v>>8;
            Ble_rply[15]=0x7E;
            Ble_send(Ble_rply, 16);    //Ӧ��
        }break;
        
        case 0x7104:    //����ǩ��Ӳ���汾��
        {
            memcpy(&Ble_rply[10], DEVICE_HW_REVISION, 15);
            memcpy(&Ble_rply[26], DEVICE_SW_REVISION, 15);
            
            crc_v=GetCrc_16(Ble_rply, 45);
            Ble_rply[42]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[43]=crc_v>>8;
            Ble_rply[44]=0x7E;
            Ble_send(Ble_rply, 45);    //Ӧ��
        }break;
        
        case 0x7105:    //д��ǩ���ñ�־(��ǩ��λ���������������ƣ�����NBƽ̨ע�ᡢ�ᶯ��⡢GPS��λ��ȫ����)
        {
            if(len<30)  //�ų�����֡
            {
                Ble_rply[10] = 0x01;    //֡��ʽ����
            }
            else
            {
                char check_id[18];
                snprintf(check_id, 18, "%s", &Data[10]);
                if(strstr(check_id, (char*)g_me3616_info.me3616_IMEI) != NULL)    //У��ID��
                {
                    if(Data[27] == 0x00)    //��ǩ��������
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
                        
//                        if(g_me3616_info.me3616_powerOn_flag == 1)  //���ģ�鿪��״̬��ػ�
//                            nb_handle.nb_fxnTablePtr->nbPowerOff(&nb_handle);       //NBģ��ػ�
                        
                        bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
                        
//                        if(g_gnt_info.gnt_usbvin_flag == 0)
//                            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);   //BLE�ػ�
//                        else
                            NVIC_SystemReset();
                    }
                    else if(Data[27] == 0xFF)   //��ǩʹ��
                    {
                        g_gnt_info.gnt_battery_level = battery_adc_sample();
                        if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)
                        {
                            Ble_rply[10] = 0x04;    //��ǩ��ʹ�ܣ�������ʹ��
                        }
                        else if(g_gnt_info.gnt_battery_level < GNT_ENABLE_BATTERY_THRESHOLD)
                        {
                            Ble_rply[10] = 0x05;    //��ǩ������ʹ��ʧ��
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
                    Ble_rply[10] = 0x02;    //ID��ƥ��
                }
                
                #if 0
                if(Data[27] == 0x03)   //����GPS��λģʽΪAGPS
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
                else if(Data[27] == 0x04)   //����GPS��λģʽΪStandAlone
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
                else if(Data[27] == 0x05)   //ʹ��GPS��Ϣʵʱ�ϱ�
                {
                    g_gnt_info.gnt_gps_realtime_report_flag = 1;
                    Ble_rply[10] = 0;
                }
                else if(Data[27] == 0x06)   //�ر�GPS��Ϣʵʱ�ϱ�
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
        
        case 0x7106:    //��NB_Iot�ź�ǿ��
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
            Ble_send(Ble_rply, 14);    //Ӧ��
//            NRF_LOG_INFO("nb rssi = %d.", Ble_rply[10]);
        }break;
        
        case 0x7107:    //��NB_Iotƽ̨ע��״̬
        {
            if(g_me3616_info.iot_regist_status == 1)
                Ble_rply[10] = 0x00;
            else
                Ble_rply[10] = 0xFF;
            
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);    //Ӧ��
        }break;
        
        case 0x7108:    //��ǩ����ָʾ
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
        
        case 0x7109:    //�����ǩ�ƶ���־
        {
            Ble_rply[10] = 0x00;    //��ǩδʹ�ܣ���֧������ָ��
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);    //Ӧ��
        }break;
        
        case 0x7111:    //�ն�GPS���ݴ��ݸ���ǩ
        {
            Ble_rply[10] = 0x00;
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);    //Ӧ��       
        }break; 
        
        case 0x7121:    //���ĵ�ǰ�����㲥ʱ��
        {
            Ble_rply[10] = 0x00;
            if(Data[10] == 0xFF)            //�����㲥ʱ���ӳ���ʮ����
            {
                if(g_gnt_info.gnt_usbvin_flag == 1)
                    Ble_rply[10] = 0x01;
                else if(g_gnt_info.gnt_usbvin_flag == 0)
                    nRF52_FxnTable.bleSetAdvTimeout(10*60*1000);
            }
            else if(Data[10] == 0x00)       //�㲥ʱ���ָ���һ����
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
        
        
        case 0x7A01:    //��ǩ�������ã������ã� 1��������NB����  2��NB��ʼ����������������
        {
            switch (Data[10])
            {
                // ��ǩʹ�����
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
                    
    //                if(g_me3616_info.me3616_powerOn_flag == 1)  //���ģ�鿪��״̬��ػ�
    //                    nb_handle.nb_fxnTablePtr->nbPowerOff(&nb_handle);       //NBģ��ػ�
                    
                    bsp_indication_set(BSP_INDICATE_GNT_STATE_OFF);
                    
    //                if(g_gnt_info.gnt_usbvin_flag == 0)
    //                    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);   //BLE�ػ�
    //                else
                        NVIC_SystemReset();
                }break;
                
                // ��ǩʹ��
                case 0xFF:
                {
                    g_gnt_info.gnt_battery_level = battery_adc_sample();
                    if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)
                    {
                        Ble_rply[10] = 0x04;    //��ǩ��ʹ�ܣ�������ʹ��
                    }
                    else if(g_gnt_info.gnt_battery_level < GNT_ENABLE_BATTERY_THRESHOLD)
                    {
                        Ble_rply[10] = 0x05;    //��ǩ������ʹ��ʧ��
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
                
                // ���ñ�ǩ���������NBģ��ģʽ�����ٴ��·�ָ��ָ���
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
                // ����GPS��λģʽ��0��AGPS�� 1��StandAlone��
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
                
                // ��ȡGPS��λģʽ��0��AGPS�� 1��StandAlone��
                case 0x04:
                {
                    Ble_rply[10] = g_gnt_flash_info.gnt_gps_mode_flag;
                }break;
                
                // ����GPS��Ϣʵʱ�ϱ���0��ȡ���� 1��ʹ�ܣ�
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
                
                // ��ȡGPS��Ϣʵʱ�ϱ���־
                case 0x06:
                {
                    Ble_rply[10] = g_gnt_info.gnt_gps_realtime_report_flag;
                }break;
#endif
                // ����NB��ƽ̨(0:ME3616_IOT_PLATFORM_OCEANCONNECT, 1:ME3616_IOT_PLATFORM_ONENET_OCEANCONNECT)
                case 0x07:
                {
                    if(len != 15)
                    {
                        Ble_rply[10] = 0xFF;
                    }
                    else
                    {
                        if(g_gnt_flash_info.gnt_iot_platform == Data[11])   //�ظ�����
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
                
                // ��ȡNB��ƽ̨����
                case 0x08:
                {
                    Ble_rply[10] = g_gnt_flash_info.gnt_iot_platform;
                }break;
                
                // ���������ϱ�����(4�ֽ�С��)
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
                            
                            #if (GNT_WDT_EN == 1)   // ���Ź�ʹ����Ҫ��������
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
                
				// ��ȡ���������ϱ�����
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
				
                // �����ǩ�ƶ���־�¼�
                case 0x0B:
                {
					if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)
					{
						// ������ǰGPS��Ϣ���ϱ��澯����
						#if (GNT_GPS_EN == 1)
						gnt_lock_location(g_gnt_info.gnt_last_latitude, g_gnt_info.gnt_last_longitude, 1);
						#endif
						
						xEventGroupSetBits(GntEventGroup, GNT_CLEAR_MOVEFLAG_EVENTBIT); //�����ǩ�ƶ���־�¼�
						Ble_rply[10] = 0x00;
					}
					else
					{
						Ble_rply[10] = 0xFF;    	//��ǩδʹ�ܣ���֧������ָ��
					}
                }break;
				
				// ��ǩ��λ
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
				
                // ��������һ�������ϱ������ԣ�
                case 0x0D:
                {
                    if(g_gnt_flash_info.gnt_enable_flag == GNT_ENABLE)
					{
						xEventGroupSetBits(GntEventGroup, NB_PRERIOD_REPORT_EVENTBIT); // �������ϱ��¼�
						Ble_rply[10] = 0x00;
					}
					else
					{
						Ble_rply[10] = 0xFF;    	// ��ǩδʹ�ܣ���֧������ָ��
					}
                }break;
                
				// ���õ�����ֵ
                case 0x0E:
                {
                    
                }break;
				
                // ��ȡ������ֵ
                case 0x0F:
                {
                    
                }break;
				
				// 
                case 0x10:
                {
                    
                }break;
                
                default:
                {
                    Ble_rply[10] = 0xFE;    //��֧��ָ��
                }break;
            }
            
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);         
        }break;    
        
        // ������ʱʹ�ã���ǩʹ�ܡ���ʼ����ɣ�
        case 0x7A02:    //���ٶȴ������ж���ֵ����
        {
            
        }break;
        
        // ��ȡ�豸��Ϣ������
        // 01-> �豸ʹ�ܱ�־
        // 02-> GPS
        // 03-> GPS��λģʽ
        // 04-> GPSʵʱ�ϱ�״̬
        // 05-> GPS��λ�Ƿ�ʹ��
        case 0x7A03:    //��ȡ�豸��Ϣ������
        {
            Ble_rply[10] = gnt_temp_get();
            crc_v=GetCrc_16(Ble_rply, 14);
            Ble_rply[11]=(uint8_t)(crc_v & 0x00ff);
            Ble_rply[12]=crc_v>>8;
            Ble_rply[13]=0x7E;
            Ble_send(Ble_rply, 14);
        }break;
        
        case 0x7A04:    //�豸��λ
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
/** �ڲ��¶ȴ�����********************************************************************************************************/
void gnt_temp_init(void)
{
    nrf_temp_init();
}

int32_t gnt_temp_get(void)
{
    int32_t temp;
    
    NRF_TEMP->TASKS_START = 1;  //��ʼ�¶Ȳ���
    
    while (NRF_TEMP->EVENTS_DATARDY == 0)   //�ȴ��¶Ȳ������
    {
        
    }
    NRF_TEMP->EVENTS_DATARDY = 0;   //���־
    
    temp = ((nrf_temp_read()*10) / 4);   //ת��Ϊ���϶�
    
    NRF_TEMP->TASKS_STOP = 1;   //ֹͣ����
    
    return temp;    
}
#endif



/** @} ble_comm_task*/

/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/



