/**
  ******************************************************************************
  * @author  泽耀科技 ASHINING
  * @version V3.0
  * @date    2016-10-08
  * @brief   AS62配置C文件
  ******************************************************************************
  * @attention
  *
  * 官网	:	http://www.ashining.com
  * 淘宝	:	https://shop105912646.taobao.com
  * 阿里巴巴:	https://cdzeyao.1688.com
  ******************************************************************************
  */
  
  
#include "drv_AS62.h"


//模块配置参数数组
//改变模块参数，只需改变参数数组值，然后初始化即可
const uint8_t g_ASxx_Param_Config[ 6 ] = { 0xC0, 0x12, 0x34, 0x1A, 0x17, 0xC4 };			//定点模式
const uint8_t  g_ASxx_Config_Status_OK[ ] = { 0x4F, 0x4B, 0x0D, 0x0A };


/**
  * @brief :ASxx模块初始化
  * @param :无
  * @note  :按照默认参数初始化，修改默认参数表即可改变模块初始化参数
  * @retval:
  *        @ASxx_Write_OK 写入成功
  *        @ASxx_Write_ERROR 写入失败
  */
ASxxWriteStatusType ASxx_param_init( void )
{
	uint8_t i = 0;
	uint8_t Read_Buff[ 5 ] = { 0 };
	
	drv_uart_tx_bytes((uint8_t *)g_ASxx_Param_Config, 6 );
	drv_uart_rx_bytes( Read_Buff );
	
	for( i = 0; i < 4; i++ )
	{
		if( Read_Buff[ i ] != g_ASxx_Config_Status_OK[ i ] )
		{
			break;
		}
	}
	
	if( 4 == i )
	{
		return ASxx_Write_OK;		//配置成功
	}
	else
	{
		return ASxx_Write_ERROR;	//配置失败
	}
	
}

/**
  * @brief :ASxx模块读配置参数
  * @param :
  *         @pReadBuffer：参数返回地址
  * @note  :无
  * @retval:无
  */
void ASxx_read_param( uint8_t *pReadBuffer )
{
	uint8_t Read_Cmd[ 3 ] = { 0xC1, 0xC1, 0xC1 };

	drv_uart_tx_bytes( Read_Cmd, 3 );
	drv_uart_rx_bytes( pReadBuffer );
	
}

/**
  * @brief :ASxx模块读取硬件版本号
  * @param :
  *         @pReadBuffer：硬件版本号返回地址
  * @note  :无
  * @retval:无
  */
void ASxx_read_version( uint8_t *pReadBuffer )
{
	uint8_t Read_Cmd[ 3 ] = { 0xC3, 0xC3, 0xC3 };

	drv_uart_tx_bytes( Read_Cmd, 3 );
	drv_uart_rx_bytes( pReadBuffer );
}

/**
  * @brief :ASxx模块读取实际电压值
  * @param :
  *         @pReadBuffer：电压值返回地址
  * @note  :无
  * @retval:无
  */
void ASxx_read_voltage( uint8_t *pReadBuffer )
{
	uint8_t Read_Cmd[ 3 ] = { 0xC5, 0xC5, 0xC5 };

	drv_uart_tx_bytes( Read_Cmd, 3 );
	drv_uart_rx_bytes( pReadBuffer );
}

/**
  * @brief :ASxx模块复位
  * @param :无
  * @note  :无
  * @retval:无
  */
void ASxx_reset( void )
{
	uint8_t Read_Cmd[ 3 ] = { 0xC4, 0xC4, 0xC4 };

	drv_uart_tx_bytes( Read_Cmd, 3 );
}

/**
  * @brief :ASxx模块发送数据（定点模式）
  * @param :
  *        @Addr_H：地址高位
  *        @Addr_L：地址低位
  *        @Channel：信道
  *        @pTxBuff：发送数据地址
  *        @Length：发送数据个数
  * @note  :定点模式 数据个数最29个
  * @retval:无
  */
void ASxx_tx_packet( uint8_t Addr_H, uint8_t Addr_L, uint8_t Channel, uint8_t *pTxBuff, uint8_t Length )
{
	uint8_t Header[ 3 ] = { 0 };
	
	Header[ 0 ] = Addr_H;
	Header[ 1 ] = Addr_L;
	Header[ 2 ] = Channel;
	
	drv_uart_tx_bytes( Header, 3 );
	//发送数据
	drv_uart_tx_bytes( pTxBuff, Length );
}

/**
  * @brief :ASxx模块接收数据（定点模式）
  * @param :无
  * @note  :定点模式 数据个数最29个
  * @retval:无
  */
uint8_t ASxx_rx_packet( uint8_t *pRxBuff )
{
	uint8_t Length = 0;
	
	Length = drv_uart_rx_bytes( pRxBuff );
	
	return Length;
}


