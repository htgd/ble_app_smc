/**@file    bsp_fds.h
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

/**@defgroup bsp_fds Bsp fds module.
* @{
* @ingroup bsp_drivers
* @brief falsh文件系统操作驱动. \n
* flash操作注意事件同步，且写操作应当在临界区执行
*/

#ifndef __BSP_FDS_H
#define	__BSP_FDS_H

#include "gnt_includes.h"

/**@brief FDS注册事件句柄和初始化程序 */
void my_fds_init(void);

/**@brief FDS写记录函数
* @param[in] fid		文件 ID
* @param[in] key  		记录标号
* @param[in] *p_data   	要记录的数据指针
* @param[in] len   		要记录的数据长度
* @return 	 none
*/
void my_fds_write(uint32_t fid, uint32_t key, void const * p_data, uint32_t len);

/**@brief FDS读记录函数
* @param[in] fid		文件 ID
* @param[in] key  		记录标号
* @param[in] *p_data   	读取存储数据指针
* @param[in] len   		读取数据长度
* @return 	 none
*/
void my_fds_read(uint32_t fid, uint32_t key, void * p_data, uint32_t len);

/**@brief FDS查找并删除记录标号的所有内容
* @param[in] fid		文件 ID
* @param[in] key  		记录标号
* @note 	 一个记录标号可以记录多个内容
*/
void my_fds_find_and_delete(uint32_t fid, uint32_t key);

/**@brief FDS删除整个文件
* @param[in] fid	文件 ID
*/
void my_fds_delete_file(uint32_t fid);


#endif	/* __BSP_FDS_H */

/** @} bsp_fds*/

/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/




