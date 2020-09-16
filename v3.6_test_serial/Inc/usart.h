/**
  ******************************************************************************
  * File Name          : USART.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
#define START 0X11
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

//Type Name		Bytes				Other Names						Range of Values
//short	   		2bytes 				short int, signed short int	     -32,768 to 32,767
//char			1						none						-128 to 127 by default

//��linux���ղ��������ݵ�������ַ��
extern int usartReceoneData(int *ptr_leftset, int *ptr_rightset, unsigned char *ptr_flag);
//��װ���ݣ�����USART1_Send_String�����ݷ��͸�linux
extern void usartSendData(short valone, short valtwo, short valthree, unsigned char flag);
//����ָ���ַ�����ĺ���
void usart_send_string(unsigned char *ptr_str, unsigned short send_size);
//�����λѭ������У�飬�õ�У��ֵ��һ���̶�����֤���ݵ���ȷ��
unsigned char getcrc8(unsigned char *ptr_input, unsigned short len);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
