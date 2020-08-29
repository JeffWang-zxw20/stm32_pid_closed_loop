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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);
//	HAL_Delay(1000);
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2000);
//	HAL_Delay(1000);
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1500);
//int motorh=500;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//motor right
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1500);
//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1200);
//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1100); //right max
//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1000);
//        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 950);
//        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 900);
//        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 800);
//	  
//		//motorh = motorh + 100;
        HAL_Delay(4000);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1600);
	  		HAL_Delay(5000);
//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1500);//////////!!!yes
//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1600);
//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1700);
//        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 600);
//        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 550);
//        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1800);

//	    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1100);
//	    HAL_Delay(5000);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1800);
	    HAL_Delay(5000);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2000);
	    HAL_Delay(10000);
//	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1500);  ////////mid value
//        HAL_Delay(3000);
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1400);   ///1600 motor: low back , servo small to right
//	    HAL_Delay(10000);
//	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1400);
//	    HAL_Delay(2000);


//20ms ARR 20000
//1600 motor: low back , wheel small to left (driver view) ----- 1.6ms ----------never give motor value greater than 1500
//1400 motor: low forward, wheel small to left -------------------1.4ms 


//wheel
//left max-----1800 ----1.8ms
//right max ---- 1100  --- 1.2ms

//motor
//1500 zero ----------------------1.5ms
//2000 max ------------------------2ms -- forward
//could be higher
//never put value smaller than 1500;
  }
}
//  /* USER CODE END 3 */


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
  /** Initializes the CPU, AHB and APB busses clocks
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
  /** Initializes the CPU, AHB and APB busses clocks
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
