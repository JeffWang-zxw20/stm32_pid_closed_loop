/**
  ******************************************************************************
  * @author  泽耀科技 ASHINING
  * @version V3.0
  * @date    2016-10-08
  * @brief   主函数
  ******************************************************************************
  * @attention
  *
  * 官网	:	http://www.ashining.com
  * 淘宝	:	https://shop105912646.taobao.com
  * 阿里巴巴:	https://cdzeyao.1688.com
  ******************************************************************************
  */
 

#include "drv_delay.h"
#include "drv_uart.h"
#include "drv_led.h"
#include "drv_AS62.h"
  

extern uint8_t g_ASxx_Param_Config[ 6 ];


/**
  * @brief :主函数
  * @param :无
  * @note  :无
  * @retval:无
  */
int main( void )
{
	uint8_t i = 0;
	uint8_t Read_Buff[ 6 ] = { 0 };
	
	//串口初始化  根据模块波特率变化
	drv_uart_init( 9600 );
	
	//ASXX配置 
	//配置时 MD0 MD1应该悬空
	//配置完成后应读出参数比较是否于配置参数一致
	Read_Buff[ 0 ] = ASxx_param_init( );

	//延时初始化
	drv_delay_init( );
	
	//LED初始化
	drv_led_init( );
	
	while( 1 )
	{
		if( ASxx_Write_OK == Read_Buff[ 0 ] )	//配置返回正确值
		{
			ASxx_read_param( Read_Buff );		//读出参数 和写入的参数比较，确认配置正确
			
			for( i = 0; i < 6; i++ )
			{
				if( Read_Buff[ i ] != g_ASxx_Param_Config[ i ] )
				{
					break;
				}
			}
			
			if( 6 == i )
			{
				//配置成功 
				led_red_flashing( );
				led_green_flashing( );
				drv_delay_500Ms( 1 );
				led_red_flashing( );
				led_green_flashing( );
				
				while( 1 )
				{
					//执行相应操作函数
				}
				
			}
			else
			{
				Read_Buff[ 0 ] = ASxx_param_init( );	//重新配置，直到配置成功，如果一直配置不成功，检测是否是因为MDO MD1没有悬空
			}
		}
		else		//配置返回错误值
		{
			Read_Buff[ 0 ] = ASxx_param_init( );		//重新配置，直到配置成功，如果一直配置不成功，检测是否是因为MDO MD1没有悬空
		}
		
		
	}
}

