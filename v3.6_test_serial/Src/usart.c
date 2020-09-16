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



//-----------------------------ROS_commu-------------------------------------------------------------//
/*--------------------------------发送协议-----------------------------------
//-----55 aa    size   			00 00 00 00 00        crc8       0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/

/*--------------------------------接收协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/


/**************************************************************************
通信的发送函数和接收函数必须的一些常量、变量、共用体对象
**************************************************************************/
//received data buffer
unsigned char recebuff[16] = {0};
//protocol constant
const unsigned char header[2] = {0x55, 0xaa}; ///数据头55aa 
const unsigned char ender[2]  = {0x0d, 0x0a}; //数据尾0d0a




//Send data union type
/*
https://docs.microsoft.com/en-us/cpp/cpp/unions?view=vs-2019

A union is a user-defined type in which all members share the same memory location. 
This definition means that at any given time, a union can contain no more than
one object from its list of members.

It also means that no matter how many members a union has,
it always uses only enough memory to store the largest member.
*/
union receData_union
{
	short d_what;
	unsigned char data_union[2];
}left_set, right_set;



//rece data union type
union sendData_union
{
	short d_send;
	unsigned char data_s_union[2];
}gyro_x, gyro_y, gyro_z;








//-----------------------------ROS_commu_END-------------------------------------------------------------//
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
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

    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream5;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

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
    HAL_DMA_DeInit(uartHandle->hdmarx);

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




/**************************************************************************
函数功能：通过串口中断服务函数，获取上位机发送的左右轮控制速度、预留控制标志位，分别存入参数中
入口参数：左轮轮速控制地址、右轮轮速控制地址、预留控制标志位
返回  值：无特殊意义
**************************************************************************/

//Type Name		Bytes				Other Names						Range of Values
//short	   		2bytes 				short int, signed short int	     -32,768 to 32,767
//char			1						none						-128 to 127 by default

/*
static : 
https://www.geeksforgeeks.org/static-variables-in-c/


ONE!--------------------------------------------
Static variables have a property of preserving 
their value even after they are out of their scope!

static variables preserve their previous value 
in their previous scope 
and are not initialized AGAIN in the new scope.

For example, we can use static int to 
count a number of times a function is called, 


TWO!---------------------------------------------
RECALL WAHT YOU STRUGGLED IN MARS LANDER
int fun() 
{ 
  static int count = 0; 
  count++; 
  return count; 
} 
   
int main() 
{ 
  printf("%d ", fun()); 
  printf("%d ", fun()); 
  return 0; 
}

Output will be 1 2, static int count = 0 will not init again.


THREE!---------------------------------------------------
Static variables (like global variables) 
are initialized as 0 
if not initialized explicitly. 


FOUR!----------------------------------------------------
Static variables should not be declared inside structure.
BUT
It is possible to declare structure 
inside the function (stack segment) 
or
allocate memory dynamically(heap segment)
*/
int usartReceoneData(int *ptr_leftset, int *ptr_rightset, unsigned char *ptr_flag)
{
	unsigned char usart_receiver    		 	= 0;  //receive data
	static unsigned char check_sum   			= 0;
	static unsigned char usart_buff_index 		= 0;
	static short j_=0, k_=0;
	static unsigned char usart_rece_front 		= 0;
	static unsigned char start_flag  			= START;    //一帧数据传送开始标志位
	static short data_len                      	= 0;
	
	usart_receiver = (uint16_t)(huart1.Instance->DR & (uint16_t)0x01FF); //huart3.Instance = USART_TypeDef
	//confused ----------& -----must figure out --- ask other															 //                  =USARTx
	
	//GET header
	
/*--------------------------------发送协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/

/*--------------------------------接收协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/
	if(start_flag == START)
	{
		if(usart_receiver == 0xaa)  //buff[1]
		{
			if(usart_rece_front ==0x55) //数据头两位 //buf[0]
			{
						
				start_flag = !START;  /////////////////////////////// !?? check
				recebuff[0] = header[0];
				recebuff[1] = header[1];
				usart_buff_index = 0;  //init buffer
				check_sum = 0x00;      //init checking
			}

		}
		else
		{
			usart_rece_front = usart_receiver;
		}
	}
	else
	{
		switch(usart_buff_index)
		{
			case 0: //get data length
				recebuff[2] = usart_receiver;
				data_len    = recebuff[2]; //数据头55 + aa + 数据字节数size ---- so [2]
				usart_buff_index++;
			case 1: //get all the data and assign value
				recebuff [j_ + 3] = usart_receiver;  //j_ = 0 init  
													//+3 is because the first three 
													// bypes are 0x55, 0xaa and data length
				j_++;
				if(j_ >= data_len)
				{
					j_=0;
					usart_buff_index++;
				}
				break;
		    case 2: //get checking info
				recebuff[3+ data_len] = usart_receiver;
				check_sum = getcrc8(recebuff, 3+data_len);
				//check crc8
				if(check_sum != recebuff[3+ data_len])
				{
					//printf("received data check sum erro");
					return 0;
				}
				usart_buff_index++;
				break;
				
			case 3: // get end of the info
				if(k_ == 0)
				{
					//数据0d     buf[9]  无需判断
					k_++;
				}
				else if(k_==1)
				{
					//数据0a     buf[10] 无需判断
					for(k_=0; k_<2; k_++)
					{
						//进行赋值操作
						left_set.data_union[k_] = recebuff[k_ +3]; //+3 is because the first three 
																	// bypes are 0x55, 0xaa and data length
						right_set.data_union[k_] = recebuff[k_ + 5]; //+5 because the previous left_set
																	//is of size 2bytes--- NEED TO CHANGE 
					}
					//赋值操作
					/*
					//Typecasting is making a variable of one type, such as an int, 
					act like another type, a char, for one single operation. 
					cout<< (char)65 <<"\n"; 
										  // The (char) is a typecast, telling the computer to interpret the 65 as a
										  //  character, not as a number. 
					
					A type cast is basically a conversion from one type to another. 
					It can be implicit (i.e., done automatically by the compiler, 
					perhaps losing info in the process) 
					or explicit (i.e., specified by the developer in the code).
					*/
					*ptr_leftset = (int) left_set.d_what;
					*ptr_rightset = (int) right_set.d_what;
					
					//flag
					*ptr_flag = recebuff[7];  ///?????????????why[7]?????
					
					//-----------------------------------------------------------------
					//完成一个数据包的接收，相关变量清零，等待下一字节数据
					usart_buff_index         =0;
					usart_rece_front         =0;
					start_flag               =START;
					check_sum                =0;
					data_len                 =0;
					j_=0;
					k_=0;
				}
				break;
				
			default:
				break;	
		}
	}
	return 0;
}




/**************************************************************************
函数功能：将左右轮速和角度数据、控制信号进行打包，通过串口发送给Linux
入口参数：实时左轮轮速、实时右轮轮速、实时角度、控制信号（如果没有角度也可以不发）
返回  值：无
**************************************************************************/
void usartSendData(short valone, short valtwo, short valthree, unsigned char flag)
{
	//buffer 
	unsigned char buff[13] = {0};
	int i_, len_ = 0;
	
	gyro_x.d_send = valone;
	gyro_y.d_send = valtwo;
	gyro_z.d_send = valthree;
	
	//set header
	for(i_=0; i_<2; i_++)
		{
			buff[i_] = header[i_];
		}
	
	len_ = 7; /////////////////////////////need to change accodingly 
	buff[2] = len_;
	for(i_=0;i_<2;i_++)
		{
			buff[i_ + 3] = gyro_x.data_s_union[i_];//+3 is because the first three 
												 // bypes are 0x55, 0xaa and data length
			buff[i_ + 5] = gyro_y.data_s_union[i_];
			buff[i_ + 7] = gyro_y.data_s_union[i_];
		}
	// 预留控制指令
	buff[3 + len_ -1 ] = flag;   //buff[9]
		
	// 设置校验值、消息尾
	buff[3 + len_] = getcrc8(buff, 3 + len_);      //buff[10]
	buff[3 + len_ + 1] = ender[0];  //fill the ender   //buff[11]
	buff[3 + len_ + 1] = ender[1];                   //buff[12]
	
	//send via dma  ----------------possible bugs (uint8_t *)&buff
	HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&buff,sizeof(buff));
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
