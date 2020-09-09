/**
  ******************************************************************************
  * @author  泽耀科技 ASHINING
  * @version V3.0
  * @date    2016-10-08
  * @brief   AS62配置H文件
  ******************************************************************************
  * @attention
  *
  * 官网	:	http://www.ashining.com
  * 淘宝	:	https://shop105912646.taobao.com
  * 阿里巴巴:	https://cdzeyao.1688.com
  ******************************************************************************
  */
  
  
#ifndef __DRV_AS62_H__
#define __DRV_AS62_H__


#include "drv_uart.h"


typedef enum 
{
	ASxx_Write_OK = 0,		//写入成功
	ASxx_Write_ERROR		//写入失败
}ASxxWriteStatusType;



ASxxWriteStatusType ASxx_param_init( void );
void ASxx_read_param( uint8_t *pReadBuffer );
void ASxx_read_version( uint8_t *pReadBuffer );
void ASxx_read_voltage( uint8_t *pReadBuffer );
void ASxx_reset( void );

#endif
