/**
  **********************************************************************************
  * @file    bsp_fstorage.c
  * @author  wanghuan  any question please send mail to 371463817@qq.com
  * @version V1.0
  * @date    2018-09-25
  * @brief   nRF52832的flash操作驱动程序
  **********************************************************************************
  * @attention
  *
  * 版权说明:Copyright (c) 2018-2020   江苏亨通光网科技有限公司
  * 硬件平台:nRF52832_QFAA
  * 修改日志:
  *
  **********************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "bsp_fstorage.h"

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    .evt_handler = fstorage_evt_handler,
    .start_addr = 0x3e000,
    .end_addr   = 0x3ffff,
};


static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}


static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = NRF_UICR->NRFFW[0];
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}


void wait_for_flash_ready(void)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(&fstorage))
    {
       sd_app_evt_wait();
    }
}


void my_fstorage_init(void)
{
    nrf_fstorage_api_t * p_fs_api;
    p_fs_api = &nrf_fstorage_sd;
    ret_code_t rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);
    
    NRF_LOG_INFO("========| flash info |========");
    NRF_LOG_INFO("erase unit: \t%d bytes",      fstorage.p_flash_info->erase_unit);
    NRF_LOG_INFO("program unit: \t%d bytes",    fstorage.p_flash_info->program_unit);
    NRF_LOG_INFO("==============================");
}

// 注意：write_addr-> 写flash的地址应为4整数倍
// p_data-> 数据起始地址应为4整数倍   len->数据长度为4字节倍数
void my_fstorage_write(uint32_t write_addr, void const * p_data, uint32_t len)
{
    len = ((len+3)/4)*4;
    
    
    ret_code_t rc = nrf_fstorage_write(&fstorage, write_addr, p_data, len, NULL);
    APP_ERROR_CHECK(rc);
}

void my_fstorage_read(uint32_t read_addr, void * p_data, uint32_t len)
{
    nrf_fstorage_read(&fstorage, read_addr, p_data, len);
}











/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/




