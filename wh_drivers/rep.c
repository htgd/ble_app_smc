/**@file    rep.c
* @brief   	Э���ֽڽ���
* @details  IODH������֡��0x7Eת���2�ֽ����У�0x7D��0x5E��������֡��0x7D ת���2�ֽ����У�0x7D��0x5D��
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2016-2-18
* @version 	V1.0
* @copyright	Copyright (c) 2016-2018  ���պ�ͨ�����Ƽ����޹�˾
**********************************************************************************
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2016/02/18  <td>1.0      <td>wanghuan  <td>������ʼ�汾
* </table>
*/

/**@defgroup bsp_rep Bsp rep module.
* @{
* @ingroup bsp_drivers
* @brief Э���ֽڽ���. \n
* ����֡��0x7Eת���2�ֽ����У�0x7D��0x5E��������֡��0x7D ת���2�ֽ����У�0x7D��0x5D��
*/

#include "gnt_includes.h"

/**
* �Ի�ȡ�������к���0x7D��0x7E�������������������ݰ�
* @param[in]  *data		Ҫ������������ָ��
* @param[in]  m    		����֡����
* @return ��������֡��n
* @note
*/
uint16_t CTin(uint8_t * data, uint16_t m)
{
	uint16_t n;
	uint16_t i;
	uint16_t k=0; //ת����0x7D.0x7E������
	uint8_t temp[1024];
	for(i=3;i<m-1;i++)
	{
		if(data[i]!=0x7D)
		{
			temp[i-k]=data[i];
		}
		if(data[i]==0x7D)
		{
			if((data[i+1]==0x5E)||(data[i+1]==0x5D))
			{
				k++; 
				i++;
				if(data[i]==0x5D)
				{
					temp[i-k]=0x7D;
				}
				if(data[i]==0x5E)
				{
					temp[i-k]=0x7E;
				}
			}
			else
			{
				temp[i-k]=0x7D;
			}
		}
	}
	n=m-k;
	for(i=3;i<n-1;i++)
	{
		data[i]=temp[i];
	}
	data[n-1]=0x7E;
	return n;
}


/**
* ��Ҫ���͵������к���0x7D��0x7E���룬���������ݰ�
* @param[in]  *data		Ҫ������������ָ��
* @param[in]  m    		Ҫ����֡����
* @return ��������֡��n
* @note
*/
uint16_t CTout(uint8_t * data, uint16_t m)
{
	uint16_t n;
	uint16_t i;
	uint16_t k=0; //ת����0x7D.0x7E������
	uint8_t temp[1024];
	for(i=3;i<m-1;i++)
	{
		if((data[i]==0x7D) || (data[i]==0x7E))
		{
			if(data[i]==0x7D)
			{
				temp[i+k]=0x7D;
				temp[i+k+1]=0x5D;
			}
			if(data[i]==0x7E)
			{
				temp[i+k]=0x7D;
				temp[i+k+1]=0x5E;
			}
			k++;
		}
		else
		{
			temp[i+k]=data[i];
		}
	}
	n=m+k;
	for(i=3;i<n-1;i++)
	{
		data[i]=temp[i];
	}
	data[n-1]=0x7E;
	return n;
}

/** @} bsp_rep*/

/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/


