/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern unsigned char t8ch1_cap_sta;  
extern unsigned int t8ch1_cap_val;
extern uint32_t cap1;
double  e_now;
double	e_pre;
double	I_e;
double	delta_t;
double	delta_e;
double RPM_ref = 1000;  //motor speed
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint32_t IC_Value1 = 0;
uint32_t IC_Value2 = 0;
double Difference = 0;
double Frequency = 0;
uint8_t Is_First_Captured = 0;  // 0- not captured, 1- captured

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Thank this author!!!!!!
//https://controllerstech.com/how-to-use-input-capture-in-stm32/
//Thank you!!!!!
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if interrput source is channel 1
	{
		if (Is_First_Captured==0)  // is the first value captured ? 
		{
			IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // capture the first value
			Is_First_Captured =1;  // set the first value captured as true
		}

		else if (Is_First_Captured)  // if the first is captured
		{
			IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // capture second value

			if (IC_Value2 > IC_Value1)  
			{
				Difference = IC_Value2-IC_Value1;   // calculate the difference
			}

			else if (IC_Value2 < IC_Value1)
			{
				Difference = ((0xffff-IC_Value1)+IC_Value2) +1;
			}

			else
			{
				Error_Handler();
			}

			Frequency = 2333333.33333/Difference;  // calculate frequency
			Is_First_Captured = 0;  // reset the first captured
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//long long temp=0;
	
	int temp=0;
	double RPM_now=0;
	double pwm_out;
	double motor_out= 1500;
	double kp=30;
	double ki=0.1;
	double kd=0.1;
	char str[12];
	extern float freq;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1500);
	HAL_Delay(3000);
	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_UART_Transmit(&huart1, "100000----\r\n", 12, 100);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1600);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		///test uarst

	/*	
		//motor tim1 ch1, servo tim1 ch4
		20ms ARR 20000
		wheel
		left max-----1800 ----1.8ms
		right max ---- 1100  --- 1.2ms

		motor
		1500 zero ----------------------1.5ms
		2000 max ------------------------2ms -- forward
		could be higher
		!!!!!NEVER put motor value SMALLER than 1500!!!!!;
	*/\
		
		sprintf(str, "%f", Frequency);
		HAL_UART_Transmit(&huart1, str, 12, 100);
		HAL_Delay(100);
//		sprintf(str, "%f", freq);
//		HAL_UART_Transmit(&huart1, str, 12, 100);
	  
	  
//		if(t8ch1_cap_sta&0X80)        //means finish the whole capture, ready to calculate data
//		{
//			temp=t8ch1_cap_sta&0X3F; //Note OX3F means from bit0 to 5, not the single bit on bit 5
//											// so it refers to how many overflows 
//			temp*=0XFFFFFFFF;		 	    	//65536 is OXFFFF	
//			temp+=(t8ch1_cap_val);      //THE time when the falling edge has occured
//			sprintf(str, "%d", temp);
//			HAL_UART_Transmit(&huart1, str, 12, 100);
//		}
			//printf("HIGH:%lld us\r\n",temp);//print to chuangkou via uart
//			//pid part RPM = 60/T
//			RPM_now = 60/(temp*1e-6);  //possible bugs here as temp is long
//			e_pre = e_now; // a possible bug here
//			
//			e_now = RPM_ref - RPM_now;  //positive means low speed, so kp is positive
//			I_e += e_now;   //ki positive 
//			delta_t = temp*1e-6;   //possible bugs here.
//			delta_e = (e_now - e_pre)/delta_t;   /// watch sign   ---- possible bugs here 
//			//if the gradient is increasing

//			pwm_out = kp*e_now + ki*I_e + kd*delta_e;  
//			if(pwm_out>1.0)
//			{
//				pwm_out = 1.0;
//			}				
//			if(pwm_out<-1.0) pwm_out=-1.0;	
//			
//			pwm_out =1500+ pwm_out*500; //motor moves forward range: 1500 to 2000  (1.5ms to 2ms) 
//			if(pwm_out<1500) pwm_out=1500;
//			if(pwm_out>2000) pwm_out=2000;
//			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, motor_out);
//			
//			t8ch1_cap_sta=0;          //start the next cpature 

//		}

//  e_now = 0;
//	e_pre=0;
//	I_e =0;
//	delta_t=0;
//	delta_e=0;
//	while(1)
//    {
//        //delay_ms(10);
//		//TIM_SetTIM3Compare2(TIM_GetTIM3Capture2()+1); 
//		//if(TIM_GetTIM3Capture2()==300)TIM_SetTIM3Compare2(0);  ---- just clear  
//		//TIm3 is irrelevant here!!!! ignore 
//		
//        if(TIM5CH1_CAPTURE_STA&0X80)        //means finish the whole capture, ready to calculate data
//		{
//			temp=TIM5CH1_CAPTURE_STA&0X3F; //Note OX3F means from bit0 to 5, not the single bit on bit 5
//											// so it refers to how many overflows 
//			temp*=65535;		 	    	//65536 is OXFFFF	
//			temp+=(TIM5CH1_CAPTURE_VAL);      //THE time when the falling edge has occured
//			
//			printf("HIGH:%lld us\r\n",temp);//print to chuangkou via uart
//			
//			
//			
//			//pid part RPM = 60/T
//			RPM_now = 60/(temp*1e-6);  //possible bugs here as temp is long
//			e_pre = e_now; // a possible bug here
//			
//			e_now = RPM_ref - RPM_now;  //positive means low speed, so kp is positive
//			I_e += e_now;   //ki positive 
//			delta_t = temp*1e-6;
//			delta_e = (e_now - e_pre)/delta_t;   /// watch sign   ---- possible bugs here 
//			//if the gradient is increasing

//			pwm_out = kp*e_now + ki*I_e + kd*delta_e;  
//			if(pwm_out>1.0)
//			{
//				pwm_out = 1.0;
//			}				
//			if(pwm_out<0.0) pwm_out=0.0;	
//			TIM_SetTIM3Compare2(pwm_out);  //generate the pwm wave 
//			
//			TIM5CH1_CAPTURE_STA=0;          //start the next cpature 
//		}
    }
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
