#include "dma.h"
#include "lcd.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//DMA驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/4/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

DMA_HandleTypeDef  UART1TxDMA_Handler;      //DMA句柄
//在 HAL 库中，任何一个可以使用 DMA 的外设，它的初始化结构体句柄都会有一个 DMA_HandleTypeDef 
//指针类型的成员变量，是 HAL 库用来做相关指向的。
//Hdmatx 就是 DMA_HandleTypeDef 结构体指针类型


//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMA通道选择,@ref DMA_channel DMA_CHANNEL_0~DMA_CHANNEL_7
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx)
{ 
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
        __HAL_RCC_DMA2_CLK_ENABLE();//DMA2时钟使能	
	}else 
	{
        __HAL_RCC_DMA1_CLK_ENABLE();//DMA1时钟使能 
	}
    
	
	//Hdmatx 就是 DMA_HandleTypeDef 结构体指针类型
    __HAL_LINKDMA(&UART1_Handler,hdmatx,UART1TxDMA_Handler);    //将DMA与USART1联系起来(发送DMA)
    
    //Tx DMA配置
    UART1TxDMA_Handler.Instance=DMA_Streamx;                            //数据流选择
    UART1TxDMA_Handler.Init.Channel=chx;                                //通道选择
    UART1TxDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             //存储器到外设
	//例如本实验我们要用到 DMA2_Stream7 的 DMA_CHANNEL_4，把内存中数组的值发送到串口外设发送寄存器 DR，
	//所以方向为存储器到外设 DMA_MEMORY_TO_PERIPH，一个一个字节发送，
	//需要数字 索引自动增加，所以是存储器增量模式 DMA_MINC_ENABLE
    UART1TxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
    UART1TxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
    UART1TxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
    UART1TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
    UART1TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //外设普通模式
    UART1TxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
    UART1TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
    UART1TxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
    UART1TxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
    UART1TxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
    
    HAL_DMA_DeInit(&UART1TxDMA_Handler);   
    HAL_DMA_Init(&UART1TxDMA_Handler);
} 


//开启一次DMA传输
//huart:串口句柄
//pData：传输的数据指针
//Size:传输的数据量
//hdmatx 是外设句柄结构体的成员变量，在这里实际就是 UART1_Handler 的 成员变量
//。在 HAL 库中，任何一个可以使用 DMA 的外设，它的初始化结构体句柄都会有一个 DMA_HandleTypeDef 
//指针类型的成员变量，是 HAL 库用来做相关指向的。
//Hdmatx 就是 DMA_HandleTypeDef 结构体指针类型
void MYDMA_USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	
	// 使能 DMA 数据流的函数
	//第一个参数是 DMA 句柄，第二个是传输源地址，第三个是传输目标地址(here is 串口控制寄存器 CR3 的位 7 )
	//This is because we use dma to send data from memory to uart, so destination of dma trans is the 
	// cr register of uart
	
	//第四个是传输的数据长度。
    HAL_DMA_Start(huart->hdmatx, (u32)pData, (uint32_t)&huart->Instance->DR, Size);//开启DMA传输
    
    huart->Instance->CR3 |= USART_CR3_DMAT;//使能串口DMA发送
	//串口 1 的 DMA 发送实际是串口控制寄存器 CR3 的位 7 来控制的
	//这里我们可以通过 直接操作寄存器方式来实现： 
	//data has been transported from memory to cr register in the uart
	//now we send the data from uart to pc 
}	  

 
