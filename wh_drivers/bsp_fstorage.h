#ifndef __BSP_FSTORAGE_H
#define	__BSP_FSTORAGE_H

#include "nrf_fstorage.h"
#if (FDS_BACKEND == NRF_FSTORAGE_SD)
#include "nrf_fstorage_sd.h"
#endif

#if (FDS_CRC_CHECK_ON_READ)
#include "crc16.h"
#endif
#include "gnt_includes.h"



void my_fstorage_init(void);
void my_fstorage_write(uint32_t write_addr, void const * p_data, uint32_t len);
void my_fstorage_read(uint32_t read_addr, void * p_data, uint32_t len);



#endif	/* __BSP_FSTORAGE_H */


