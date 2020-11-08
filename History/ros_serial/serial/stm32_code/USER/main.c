
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "mbotLinuxUsart.h"//引用该头文件是使用，通信协议的前提

//测试发送变量
short testSend1   =5000;
short testSend2   =2000;
short testSend3   =1000;
unsigned char testSend4 = 0x05;

//测试接收变量
int testRece1     =0;
int testRece2     =0;
unsigned char testRece3 = 0x00;


int main(void)
{	
//======================================硬件初始化====================================================
	delay_init();	    	                        //延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置中断优先级分组2
	uart_init(115200);	                            //串口初始化为115200
	
//=======================================循环程序=====================================================
	while(1)
	{
		//将需要发送到ROS的数据，从该函数发出，前三个数据范围（-32768 - +32767），第四个数据的范围(0 - 255)
		usartSendData(testSend1,testSend2,testSend3,testSend4);
		//必须的延时
		delay_ms(13);
	} 
}

//====================================串口中断服务程序=================================================
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
		 USART_ClearITPendingBit(USART1,USART_IT_RXNE);//首先清除中断标志位
		 //从ROS接收到的数据，存放到下面三个变量中
		 usartReceiveOneData(&testRece1,&testRece2,&testRece3);
	 }
}
//===========================================END=======================================================
