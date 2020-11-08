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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//remote control data 
RC_ctrl_t rc_ctrl;

//receive data, 18 bytes one frame, but set 36 bytes 
//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];




void remote_control_init(void)
{	//void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);//receive enable 
}

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;  //return address
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_usart1_tx;
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

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
//����ɳ�ʼ��֮��ÿ�� USART3 ���������ж�ʱ
//�ͻ���� USART3_IRQHandler ���д� ��
//�� USART3_IRQHandler �У����мĴ����жϱ�־λ�Ĵ���
//Ȼ���жϽ��н��յĻ������� 1 �Ż��������� 2 �Ż�������
//ʹ���趨���ȼ�ȥʣ�೤�ȣ���ȡ���� DMA �õ����� �ݵĳ��ȣ�
//�ж��Ƿ���һ֡���ݣ�18 �ֽڣ�������ȣ�
//����������ú��� sbus_to_rc �� ��ң�������ݵĽ���
	
	
//HAL_UART_IRQHandler
//�ú���һ�����жϷ������� ���ã�
//��Ϊ�����жϴ����ͨ����ڡ�һ����÷���Ϊ�� 
//void USART1_IRQHandler(void)                 
//	{  HAL_UART_IRQHandler(&UART1_Handler); //���� HAL ���жϴ����ú���      
//��//�жϴ�����ɺ�Ľ������� } 
//	
//Ҳ����˵�������Ĵ����жϴ����߼����ǻ������ں���HAL_UART_IRQHandler�ڲ�ִ�С�
//���ú����� HAL ���Ѿ�����ã������û�һ�㲻�������޸ġ�
//���ʱ���һ��ʣ���ô���� �� �� �� �� �� �� �� �� д �� �� �� �� �� 
	
//�Ӵ����߼����Կ������ں��� HAL_UART_IRQHandler �ڲ�ͨ���ж��ж������Ƿ�
//Ϊ���� ����жϣ�ȷ���Ƿ���� HAL ����һ������ UART_Receive_IT()��
//���� UART_Receive_IT()�� �����ǰ�ÿ���жϽ��յ����ַ�������
//���ھ���Ļ���ָ�� pRxBuffPtr �У�
//ͬʱÿ�ν���һ�� �ַ���������� RxXferCount �� 1��
//ֱ��������� RxXferSize ���ַ�֮�� RxXferCount ����Ϊ 0��
//ͬʱ���ý�����ɻص����� HAL_UART_RxCpltCallback ���д���

//e.g in UART_Receive_IT():
//  /*Call legacy weak Rx complete callback*/
//     HAL_UART_RxCpltCallback(huart);
//Note HAL_UART_RxCpltCallback will only be called when transmission is completed
//and this func __weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
//is a weak func, we need to override it.


/*
��������ٰѴ��ڽ����жϵ�һ�����̽��и����������յ�һ���ַ�֮��

�ں���
UART_Receive_IT (inside  HAL_UART_IRQHandler)

�л�����ݱ����ڴ��ھ���ĳ�Ա����pRxBuffPtr�����У�
ͬʱRxXferCount �������� 1������������� RxXferSize=10,
��ô�����յ� 10 ���ַ�֮��RxXferCount ���� 10 
���� 0��RxXferCount ��ʼֵ���� RxXferSize����
���ʱ���ٵ��ý������

�ص����� 
HAL_UART_RxCpltCallback(weak func, inside UART_Receive_IT)

���д���
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
  if(huart3.Instance->SR & UART_FLAG_RXNE)//���յ����� &here is bit wise and
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
		This bit can be written only if EN is ��0�� to indicate the target memory area of 
		the first transfer. Once the stream is enabled, this bit operates as a status flag 
		indicating which memory area is the current target. 
		*/
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
			//NDTR----------DMA stream x number of data register (DMA_SxNDTR) (x = 0..7)
			// NDT[15:0]: Number of data items to transfer
			//Number of data items 
			//to be transferred (0 up to 65535). 
			//This register can be written only when the stream is disable
			//data length
			//���ݳ���
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
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
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
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


/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
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
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
