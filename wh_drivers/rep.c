/**@file    rep.c
* @brief   	协议字节解析
* @details  IODH的数据帧中0x7E转变成2字节序列（0x7D，0x5E），数据帧中0x7D 转变成2字节序列（0x7D，0x5D）
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2016-2-18
* @version 	V1.0
* @copyright	Copyright (c) 2016-2018  江苏亨通光网科技有限公司
**********************************************************************************
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2016/02/18  <td>1.0      <td>wanghuan  <td>创建初始版本
* </table>
*/

/**@defgroup bsp_rep Bsp rep module.
* @{
* @ingroup bsp_drivers
* @brief 协议字节解析. \n
* 数据帧中0x7E转变成2字节序列（0x7D，0x5E），数据帧中0x7D 转变成2字节序列（0x7D，0x5D）
*/

#include "gnt_includes.h"

/**
* 对获取的数据中含有0x7D、0x7E解析出来，并重组数据包
* @param[in]  *data		要解析的数据首指针
* @param[in]  m    		接收帧长度
* @return 重组后的总帧长n
* @note
*/
uint16_t CTin(uint8_t * data, uint16_t m)
{
	uint16_t n;
	uint16_t i;
	uint16_t k=0; //转换的0x7D.0x7E的数量
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
* 对要发送的数据中含有0x7D、0x7E编码，并重组数据包
* @param[in]  *data		要解析的数据首指针
* @param[in]  m    		要发送帧长度
* @return 重组后的总帧长n
* @note
*/
uint16_t CTout(uint8_t * data, uint16_t m)
{
	uint16_t n;
	uint16_t i;
	uint16_t k=0; //转换的0x7D.0x7E的数量
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


