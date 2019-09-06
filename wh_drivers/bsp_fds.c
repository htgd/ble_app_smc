/**@file    bsp_fds.c
* @brief   	nRF52832的flash文件系统操作驱动程序
* @details  flash操作基于nRF-SDK的fds库
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-09-25
* @version 	V1.0
* @copyright	Copyright (c) 2018-2020  江苏亨通光网科技有限公司
**********************************************************************************
* @attention
* 硬件平台:nRF52832_QFAA \n
* SDK版本：nRF5_SDK_15.0.0
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/09/15  <td>1.0      <td>wanghuan  <td>创建初始版本
* </table>
*/

/* Includes ------------------------------------------------------------------*/
#include "bsp_fds.h"

// FDS事件完成标志
static bool volatile m_fds_evt_init;
static bool volatile m_fds_evt_write;
static bool volatile m_fds_evt_update;
static bool volatile m_fds_evt_del_record;
static bool volatile m_fds_evt_del_file;
static bool volatile m_fds_evt_gc;


/**@brief FDS事件处理函数
* @param[in]  *p_evt   	FDS事件指针
* @return none
* @note
*/
static void my_fds_evt_handler(fds_evt_t const * p_evt)
{
    switch (p_evt->id)
    {
        case FDS_EVT_INIT:  //FDS初始化事件
            if (p_evt->result == FDS_SUCCESS)
            {
                m_fds_evt_init = true;
            }
            break;

        case FDS_EVT_WRITE: //FDS写记录事件
        {
            if (p_evt->result == FDS_SUCCESS)
            {
                NRF_LOG_INFO("Write Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
                m_fds_evt_write = true;
            }
        } break;
        
        case FDS_EVT_UPDATE:	//FDS更新事件，即写falsh操作
        {
            if (p_evt->result == FDS_SUCCESS)
            {
                m_fds_evt_update = true;
            }
        } break;

        case FDS_EVT_DEL_RECORD:	//FDS删除记录事件
        {
            if (p_evt->result == FDS_SUCCESS)
            {
                NRF_LOG_INFO("Delete Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
                m_fds_evt_del_record = true;
            }
        } break;
        
        case FDS_EVT_DEL_FILE:		//FDS删除整个文件事件
        {
            if (p_evt->result == FDS_SUCCESS)
            {
                m_fds_evt_del_file = true;
            }
        } break;
        
        case FDS_EVT_GC:			//FDS垃圾回收事件
        {
            if (p_evt->result == FDS_SUCCESS)
            {
                m_fds_evt_gc = true;
            }
        } break;

        default:
            break;
    }
}

/**@brief FDS注册事件句柄和初始化程序 */
void my_fds_init(void)
{
    //注册FDS事件句柄，在调用fds_init()函数之前，一定要先注册
    ret_code_t rc = fds_register(my_fds_evt_handler);
    APP_ERROR_CHECK(rc);
    
    //FDS模块初始化
    m_fds_evt_init = false;
    rc = fds_init();
    APP_ERROR_CHECK(rc);
    //等待fds初始化完成
    while (!m_fds_evt_init)
    {
        sd_app_evt_wait();
    }
    
    fds_stat_t stat = {0};
    rc = fds_stat(&stat);
    APP_ERROR_CHECK(rc);

    NRF_LOG_INFO("Found %d valid records.", stat.valid_records);
    NRF_LOG_INFO("Found %d dirty records (ready to be garbage collected).", stat.dirty_records);
    if(stat.dirty_records>0)
    {
        m_fds_evt_gc = false;
        rc = fds_gc();
        APP_ERROR_CHECK(rc);
        while (!m_fds_evt_gc)
        {
            sd_app_evt_wait();
        }
    }
}

/**@brief FDS写记录函数
* @param[in] fid		文件 ID
* @param[in] key  		记录标号
* @param[in] *p_data   	要记录的数据指针
* @param[in] len   		要记录的数据长度
* @return 	 none
*/
void my_fds_write(uint32_t fid, uint32_t key, void const * p_data, uint32_t len)
{
    my_fds_find_and_delete(fid, key);
    
    fds_record_t const rec =
    {
        .file_id           = fid,
        .key               = key,
        .data.p_data       = p_data,
        .data.length_words = (len + 3) / sizeof(uint32_t)
    };
    m_fds_evt_write = false;
    ret_code_t rc = fds_record_write(NULL, &rec);
    if (rc != FDS_SUCCESS)
    {
        NRF_LOG_INFO("error: fds_record_write %d", rc);
    }
    else
    {
        while (!m_fds_evt_write)
        {
            sd_app_evt_wait();
        }
    }
}

/**@brief FDS读记录函数
* @param[in] fid		文件 ID
* @param[in] key  		记录标号
* @param[in] *p_data   	读取存储数据指针
* @param[in] len   		读取数据长度
* @return 	 none
*/
void my_fds_read(uint32_t fid, uint32_t key, void * p_data, uint32_t len)
{
    fds_record_desc_t desc = {0};
    fds_find_token_t  ftok = {0};
    if (fds_record_find(fid, key, &desc, &ftok) == FDS_SUCCESS)
    {
        fds_flash_record_t config = {0};    //存储读取的flash数据
        ret_code_t rc = fds_record_open(&desc, &config);//打开记录读取数据
        APP_ERROR_CHECK(rc);
        
        NRF_LOG_INFO("find record id = %d", desc.record_id);
        
        memcpy(p_data, config.p_data, len);
        
        rc = fds_record_close(&desc);   //关闭记录
        APP_ERROR_CHECK(rc);
    }
}

/**@brief FDS查找并删除记录标号的所有内容
* @param[in] fid		文件 ID
* @param[in] key  		记录标号
* @note 	 一个记录标号可以记录多个内容
*/
void my_fds_find_and_delete(uint32_t fid, uint32_t key)
{
    fds_record_desc_t desc = {0};
    fds_find_token_t  ftok = {0};
    while (fds_record_find(fid, key, &desc, &ftok) == FDS_SUCCESS)
    {
        m_fds_evt_del_record = false;
        fds_record_delete(&desc);
        while (!m_fds_evt_del_record)
        {
            sd_app_evt_wait();
        }
    }
    
    fds_stat_t stat = {0};
    ret_code_t rc = fds_stat(&stat);
    APP_ERROR_CHECK(rc);
    if(stat.dirty_records>0)
    {
        m_fds_evt_gc = false;
        rc = fds_gc();	//回收资源
        APP_ERROR_CHECK(rc);
        while (!m_fds_evt_gc)
        {
            sd_app_evt_wait();
        }
    }
}

/**@brief FDS删除整个文件
* @param[in] fid	文件 ID
*/
void my_fds_delete_file(uint32_t fid)
{
    ret_code_t rc;
    m_fds_evt_del_file = false;
    fds_file_delete(fid);
    while (!m_fds_evt_del_file)
    {
        sd_app_evt_wait();
    }
    
    m_fds_evt_gc = false;
    rc = fds_gc();
    APP_ERROR_CHECK(rc);
    while (!m_fds_evt_gc)
    {
        sd_app_evt_wait();
    }
}







/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/



