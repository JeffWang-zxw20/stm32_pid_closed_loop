/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "remote_control.h" 
#include "usart.h"  //so that we can use some rece_it variables defined in usart.c
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//HAL_RECE_IT__defined in usart.c  ------the code below is unnecessary??????????????????????????????????
//extern uint8_t USART_RX_BUF[USART_REC_LEN];//接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
//extern uint16_t USART_RX_STA;              //接收状态标记	
//extern uint8_t aRxBuffer[RXBUFFRSIZE]; //HAL库USART接收Buffer


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */


//RC_controller decode
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




//-----------------------------------RC_CONTROLLER---------------------------------//
/* USER CODE BEGIN TD */
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
//remote control data 
RC_ctrl_t rc_ctrl;

//receive data, 18 bytes one frame, but set 36 bytes 
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

void remote_control_init(void)
{	//void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);//receive enable 
}

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;  //return address
}
//-----------------------------------RC_CONTROLLER_END---------------------------------//




/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	//didadida,dadada,lalala,hahaha
	//http://www.mculover666.cn/posts/4283984198/
	//HAL_IncTick函数会把当前系统中定义的计数值变量递加，在stm32l4xx_hal.c文件中，
	//HAL库中还定义了一个函数 HAL_GetTick()，使用此API可以获取到当前系统中的计数值


	//死循环比较，直到到达延时值，每次1ms，1000次为1s --->because default freq for this clock is 1khz
    //while ((HAL_GetTick() - tickstart) < 1000);
//	//上面的这种应用方式比较准确，但系统进入了死循环，还有一种比较灵活的使用方式：
//	 if(HAL_GetTick() % 1000 == 0)
//    {
//        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//    }
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  uint32_t timeout=0;
  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)//等待就绪
  {
	 timeout++; 
	 if(timeout>HAL_MAX_DELAY)
		 break; //TIMEout 
  }
  
  timeout=0;
  while(HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFRSIZE) != HAL_OK) 
  {//一次处理完成之后，重新开启中断并设置RxXferCount为1
	timeout++;
	if(timeout>HAL_MAX_DELAY)
		 break; //TIMEout 
  }
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
/*
//在完成初始化之后，每当 USART3 产生空闲中断时
//就会进入 USART3_IRQHandler 进行处 理，
//在 USART3_IRQHandler 中，进行寄存器中断标志位的处理，
//然后判断进行接收的缓冲区是 1 号缓冲区还是 2 号缓冲区，
//使用设定长度减去剩余长度，获取本次 DMA 得到的数 据的长度，
//判断是否与一帧数据（18 字节）长度相等，
//如果相等则调用函数 sbus_to_rc 进 行遥控器数据的解码
	
	
//HAL_UART_IRQHandler
//该函数一般在中断服务函数中 调用，
//作为串口中断处理的通用入口。一般调用方法为： 
//void USART1_IRQHandler(void)                 
//	{  HAL_UART_IRQHandler(&UART1_Handler); //调用 HAL 库中断处理公用函数      
//…//中断处理完成后的结束工作 } 
//	
//也就是说，真正的串口中断处理逻辑我们会最终在函数HAL_UART_IRQHandler内部执行。
//而该函数是 HAL 库已经定义好，而且用户一般不能随意修改。
//这个时候大家会问，那么我们 的 中 断 控 制 逻 辑 编 写 在 哪 里 呢 ？ 
	
//从代码逻辑可以看出，在函数 HAL_UART_IRQHandler 内部通过判断中断类型是否
//为接收 完成中断，确定是否调用 HAL 另外一个函数 UART_Receive_IT()。
//函数 UART_Receive_IT()的 作用是把每次中断接收到的字符保存在
//串口句柄的缓存指针 pRxBuffPtr 中，
//同时每次接收一个 字符，其计数器 RxXferCount 减 1，
//直到接收完成 RxXferSize 个字符之后 RxXferCount 设置为 0，
//同时调用接收完成回调函数 HAL_UART_RxCpltCallback 进行处理。

//e.g in UART_Receive_IT():
//  //Call legacy weak Rx complete callback//
//     HAL_UART_RxCpltCallback(huart);
//Note HAL_UART_RxCpltCallback will only be called when transmission is completed
//and this func __weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
//is a weak func, we need to override it.
*/

/*
这里，我们再把串口接收中断的一般流程进行概括：当接收到一个字符之后，

在函数
UART_Receive_IT (inside  HAL_UART_IRQHandler)

中会把数据保存在串口句柄的成员变量pRxBuffPtr缓存中，
同时RxXferCount 计数器减 1。如果我们设置 RxXferSize=10,
那么当接收到 10 个字符之后，RxXferCount 会由 10 
减到 0（RxXferCount 初始值等于 RxXferSize），
这个时候再调用接收完成

回调函数 
HAL_UART_RxCpltCallback(weak func, inside UART_Receive_IT)

进行处理。
*/


  /* USER CODE END USART3_IRQn 0 */
  //HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
  
  //huart3.Instance->SR: Status register (USART_SR) 
  /*
  RXNE: Read data register not empty This bit is set by hardware when the content of 
  the RDR shift register has been transferred to the USART_DR register.
  An interrupt is generated if RXNEIE=1 in the USART_CR1 register. 
  It is cleared by a read to the USART_DR register
  */
  if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据 &here is bit wise and
										// if the RXNE in the SR register 
										// and the UART_FLAG_RXNE
										//are both one, means this bit is set???? don't make sense
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
	
	
    else if(USART3->SR & UART_FLAG_IDLE)
		/* IDLE: IDLE line detected This bit is set by hardware when 
			an Idle Line is detected. An interrupt is generated if the IDLEIE=1 in
			the USART_CR1 register. It is cleared by a software sequence 
			(an read to the USART_SR register followed by a read to the USART_DR register).
		*/
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

		
		// CR is /*!< DMA stream x configuration register      */
	    //so CR is an abbreviation of 
	    //DMA stream x configuration register (DMA_SxCR) (x = 0..7) 
		/*
		CT: Current target (only in double buffer mode) This bits is set and cleared by hardware.
		It can also be written by software.
		0: The current target memory is 
		Memory 0 (addressed by the DMA_SxM0AR pointer) 
		1: The current target memory is Memory 1 (addressed by the DMA_SxM1AR pointer) 
		This bit can be written only if EN is ‘0’ to indicate the target memory area of 
		the first transfer. Once the stream is enabled, this bit operates as a status flag 
		indicating which memory area is the current target. 
		*/
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
			//NDTR----------DMA stream x number of data register (DMA_SxNDTR) (x = 0..7)
			// NDT[15:0]: Number of data items to transfer
			//Number of data items 
			//to be transferred (0 up to 65535). 
			//This register can be written only when the stream is disable
			//data length
			//数据长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM8 capture compare interrupt.
  */
void TIM8_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_CC_IRQn 0 */

  /* USER CODE END TIM8_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_CC_IRQn 1 */

  /* USER CODE END TIM8_CC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

//---------------------RECE_IT---------------------------------------------------------------//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{							
										/*	USART_RX_STA 16bits 接收状态
											bit15，	接收完成标志  1 = 0x8000
											bit14，	接收到0x0d   1= 0x4000
											bit13~0，	接收到的有效字节数目
														full = 0x3fff = 0011 1111 1111 1111
											protocol is: end with 0x0d 0x0a
	
				当接收到从电脑发过来的数据，把接收到的数据保存在 USART_RX_BUF 中，同时在接收状态
				寄存器（USART_RX_STA）中计数接收到的有效数据个数，当收到回车（回车的表示由 2 个字
				节组成：0X0D 和 0X0A）的第一个字节 0X0D 时，计数器将不再增加，等待 0X0A 的到来，而
				如果 0X0A 没有来到，则认为这次接收失败，重新开始下一次接收。如果顺利接收到 0X0A，
				则标记 USART_RX_STA 的第 15 位，这样完成一次接收，并等待该位被其他程序清除，从而开
				始下一次的接收，而如果迟迟没有收到0X0D，那么在接收数据超过USART_REC_LEN的时候，
				则会丢弃前面的数据，重新接收。

										*/
	if(huart->Instance == USART1)
	{
		if( (USART_RX_STA&0x8000) == 0) //接收未完成  //0x8000 = 1000 0000 0000 0000 = bit15  &is bitwise AND
		{
			if(USART_RX_STA&0x4000)  // if already接收到了0x0dbefore    0x4000 = 0100 0000 0000 0000
			{	
					if(aRxBuffer[0] != 0x0a) //if this time we get is not 0x0a 
											//then error, since last time we got 0x0d, this time
											// must got 0x0a
											//aRxBuffer is of size one, every time an it occure, one byte is received
					{
						USART_RX_STA = 0;  //接收数据错误,重新开始接收
					}
					else   //this time we got 0x0a, rece complete
					{
						USART_RX_STA |= 0x8000;
					}
			}
			else //还没收到0X0D
			{
				if(aRxBuffer[0] == 0x0d)  //收到0X0D
					USART_RX_STA |= 0x4000; //set that we have received 0x0d
				else  //GET other bits --- this bit is data bits
				{
					USART_RX_BUF[USART_RX_STA&0x3fff] = aRxBuffer[0];
					/*
					Essense is here
					& is bit wise 
					suppose I have received 5 bytes of data, 
					so USART_RX_STA will be 
					00 00 0000 0000 0101 
					USART_RX_STA&0x3fff will be (can verify it with windows built in calculator)
					00 00 0000 0000 0101
					so USART_RX_BUF[USART_RX_STA&0x3fff] 
					= USART_RX_BUF[5]
					Note when USART_RX_BUF[USART_RX_STA&0x3fff] = aRxBuffer[0] is executed
					we are receiving the 6th(start from 1st) data
					so we put the 6th data at USART_RX_BUF[5]
					make sense
					very clever design
					*/
					USART_RX_STA++;  //the number of bits stored ++
					if(USART_RX_STA > (USART_REC_LEN-1)) //#define USART_REC_LEN  200 定义最大接收字节数 200
						USART_RX_STA=0; //接收数据错误,重新开始接收-- beccause exploded
				}
			}
		}
	}
}
//---------------------RECE_IT---------------------------------------------------------------//

//-----------------------------------RC_CONTROLLER---------------------------------//
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
       rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

//-----------------------------------RC_CONTROLLER---------------------------------//
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
