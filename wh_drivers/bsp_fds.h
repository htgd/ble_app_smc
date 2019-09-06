/**@file    bsp_fds.h
* @brief   	nRF52832��flash�ļ�ϵͳ������������
* @details  flash��������nRF-SDK��fds��
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-09-25
* @version 	V1.0
* @copyright	Copyright (c) 2018-2020  ���պ�ͨ�����Ƽ����޹�˾
**********************************************************************************
* @attention
* Ӳ��ƽ̨:nRF52832_QFAA \n
* SDK�汾��nRF5_SDK_15.0.0
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/09/15  <td>1.0      <td>wanghuan  <td>������ʼ�汾
* </table>
*/

/**@defgroup bsp_fds Bsp fds module.
* @{
* @ingroup bsp_drivers
* @brief falsh�ļ�ϵͳ��������. \n
* flash����ע���¼�ͬ������д����Ӧ�����ٽ���ִ��
*/

#ifndef __BSP_FDS_H
#define	__BSP_FDS_H

#include "gnt_includes.h"

/**@brief FDSע���¼�����ͳ�ʼ������ */
void my_fds_init(void);

/**@brief FDSд��¼����
* @param[in] fid		�ļ� ID
* @param[in] key  		��¼���
* @param[in] *p_data   	Ҫ��¼������ָ��
* @param[in] len   		Ҫ��¼�����ݳ���
* @return 	 none
*/
void my_fds_write(uint32_t fid, uint32_t key, void const * p_data, uint32_t len);

/**@brief FDS����¼����
* @param[in] fid		�ļ� ID
* @param[in] key  		��¼���
* @param[in] *p_data   	��ȡ�洢����ָ��
* @param[in] len   		��ȡ���ݳ���
* @return 	 none
*/
void my_fds_read(uint32_t fid, uint32_t key, void * p_data, uint32_t len);

/**@brief FDS���Ҳ�ɾ����¼��ŵ���������
* @param[in] fid		�ļ� ID
* @param[in] key  		��¼���
* @note 	 һ����¼��ſ��Լ�¼�������
*/
void my_fds_find_and_delete(uint32_t fid, uint32_t key);

/**@brief FDSɾ�������ļ�
* @param[in] fid	�ļ� ID
*/
void my_fds_delete_file(uint32_t fid);


#endif	/* __BSP_FDS_H */

/** @} bsp_fds*/

/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/




