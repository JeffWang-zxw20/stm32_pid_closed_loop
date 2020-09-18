/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
//成员变量 Instance 是用来设置寄存器基地址，例如要设置为 DMA2 的数据流 7，那么取值 为 DMA2_Stream
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 100000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB7     ------> USART1_RX
    PA9     ------> USART1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC11     ------> USART3_RX
    PC10     ------> USART3_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB7     ------> USART1_RX
    PA9     ------> USART1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC11     ------> USART3_RX
    PC10     ------> USART3_TX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11|GPIO_PIN_10);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//成员变量 Instance 是用来设置寄存器基地址，例如要设置为 DMA2 的数据流 7，那么取值 为 DMA2_Stream

//数据接收暂存区
unsigned char  receiveBuff[16] = {0};         
//通信协议常量
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};

//发送数据（左轮速、右轮速、角度）共用体（-32767 - +32768）
union sendData
{
	short d;
	unsigned char data[2];
}leftVelNow,rightVelNow,angleNow;

//左右轮速控制速度共用体
union receiveData
{
	short d;
	unsigned char data[2];
}leftVelSet,rightVelSet;

/**************************************************************************
函数功能：通过串口中断服务函数，获取上位机发送的左右轮控制速度、预留控制标志位，分别存入参数中
入口参数：左轮轮速控制地址、右轮轮速控制地址、预留控制标志位
返回  值：无特殊意义
**************************************************************************/
int usartReceiveOneData(int *p_leftSpeedSet,int *p_rightSpeedSet,unsigned char *p_crtlFlag)
{
	unsigned char USART_Receiver              = 0;          //接收数据
	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //一帧数据传送开始标志位
	static short dataLength                   = 0;

	//USART_Receiver = USART_ReceiveData(USART1);   //@@@@@#####如果你使用不是USART1更改成相应的，比如USART3
	//接收消息头
	if(Start_Flag == START)
	{
		if(USART_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)        //数据头两位 //buf[0]
			{
				Start_Flag = !START;              //收到数据头，开始接收数据
				//printf("header ok\n");
				receiveBuff[0]=header[0];         //buf[0]
				receiveBuff[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //缓冲区初始化
				checkSum = 0x00;				  //校验和初始化
			}
		}
		else 
		{
			USARTReceiverFront = USART_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://接收左右轮速度数据的长度
				receiveBuff[2] = USART_Receiver;
				dataLength     = receiveBuff[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://接收所有数据，并赋值处理 
				receiveBuff[j + 3] = USART_Receiver;        //buf[3] - buf[7]					
				j++;
				if(j >= dataLength)                         
				{
					j = 0;
					USARTBufferIndex++;
				}
				break;
			case 2://接收校验值信息
				receiveBuff[3 + dataLength] = USART_Receiver;
				checkSum = getCrc8(receiveBuff, 3 + dataLength);
				  // 检查信息校验值
				if (checkSum != receiveBuff[3 + dataLength]) //buf[8]
				{
					//printf("Received data check sum error!");
					return 0;
				}
				USARTBufferIndex++;
				break;
				
			case 3://接收信息尾
				if(k==0)
				{
					//数据0d     buf[9]  无需判断
					k++;
				}
				else if (k==1)
				{
					//数据0a     buf[10] 无需判断

					//进行速度赋值操作					
					 for(k = 0; k < 2; k++)
					{
						leftVelSet.data[k]  = receiveBuff[k + 3]; //buf[3]  buf[4]
						rightVelSet.data[k] = receiveBuff[k + 5]; //buf[5]  buf[6]
					}				
					
					//速度赋值操作
					*p_leftSpeedSet  = (int)leftVelSet.d;
					*p_rightSpeedSet = (int)rightVelSet.d;
					
					//ctrlFlag
					*p_crtlFlag = receiveBuff[7];                //buf[7]
					
					//-----------------------------------------------------------------
					//完成一个数据包的接收，相关变量清零，等待下一字节数据
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}
/**************************************************************************
函数功能：将左右轮速和角度数据、控制信号进行打包，通过串口发送给Linux
入口参数：实时左轮轮速、实时右轮轮速、实时角度、控制信号（如果没有角度也可以不发）
返回  值：无
**************************************************************************/
void usartSendData(short leftVel, short rightVel,short angle,unsigned char ctrlFlag)
{
	// 协议数据缓存数组
	unsigned char buf[13] = {0};
	int i, length = 0;

	// 计算左右轮期望速度
	leftVelNow.d  = leftVel;
	rightVelNow.d = rightVel;
	angleNow.d    = angle;
	
	// 设置消息头
	for(i = 0; i < 2; i++)
		buf[i] = header[i];                      // buf[0] buf[1] 
	
	// 设置机器人左右轮速度、角度
	length = 7;
	buf[2] = length;                             // buf[2]
	for(i = 0; i < 2; i++)
	{
		buf[i + 3] = leftVelNow.data[i];         // buf[3] buf[4]
		buf[i + 5] = rightVelNow.data[i];        // buf[5] buf[6]
		buf[i + 7] = angleNow.data[i];           // buf[7] buf[8]
	}
	// 预留控制指令
	buf[3 + length - 1] = ctrlFlag;              // buf[9]
	
	// 设置校验值、消息尾
	buf[3 + length] = getCrc8(buf, 3 + length);  // buf[10]
	buf[3 + length + 1] = ender[0];              // buf[11]
	buf[3 + length + 2] = ender[1];              // buf[12]
	
	//发送字符串数据
	//USART_Send_String(buf,sizeof(buf));
	HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&buf,sizeof(buf));
}
/**************************************************************************
函数功能：发送指定大小的字符数组，被usartSendData函数调用
入口参数：数组地址、数组大小
返回  值：无
**************************************************************************/
void USART_Send_String(uint8_t *p,uint16_t sendSize)
{ 
	static int length =0;
	while(length<sendSize)
	{   
		//@@@@@#####如果你使用不是USART1更改成相应的，比如USART3，这里有两处修改
		while( !(USART1->SR&(0x01<<7)) );//发送缓冲区为空
		USART1->DR=*p;                   
		p++;
		length++;
	}
	length =0;
}
/**************************************************************************
函数功能：计算八位循环冗余校验，被usartSendData和usartReceiveOneData函数调用
入口参数：数组地址、数组大小
返回  值：无
**************************************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
