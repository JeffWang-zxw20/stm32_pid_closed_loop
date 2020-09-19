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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include "string.h"
#include "BMI088driver.h"
#include "ANO.h"
#include "host.h"
#include "mbotLinuxUsart.h"//���ø�ͷ�ļ���ʹ�ã�ͨ��Э���ǰ��
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

//I'd like to thank this author who did me such a huge favour in pwm capture.
//https://controllerstech.com/how-to-use-input-capture-in-stm32/
//for ANO, I learnt from this tutorial
//https://www.bilibili.com/video/BV1cJ411N7P4/?spm_id_from=333.788.videocard.6

//----------------------------IMU-----------------------------------------------------//
fp32 gyro[3], accel[3], temp;


//--------------------------ros-----------------------------------------------------//
//���Է��ͱ���
short testSend1   =5000;
short testSend2   =2000;
short testSend3   =1000;
unsigned char testSend4 = 0x05;

//���Խ��ձ���
int testRece1     =0;
int testRece2     =0;
unsigned char testRece3 = 0x00;

union sendData_t
{
	short d;
	unsigned char data[2];
}leftVel,rightVel,angle;

//�������ٿ����ٶȹ�����
union receiveData_t
{
	short d;
	unsigned char data[2];
}left,right;

//--------------------------ros_END-----------------------------------------------------//



//-------------------------------------------------ANO--------------------------------------------------------------------------------------//

int trans_scale_gyro = 1e5;
int trans_scale_acce = 1e3;
#define BYTE0(dwTemp) (*( (char *)(&dwTemp)     ))
#define BYTE1(dwTemp) (*( (char *)(&dwTemp)+1     ))
#define BYTE2(dwTemp) (*( (char *)(&dwTemp)+2    ))
#define BYTE3(dwTemp) (*( (char *)(&dwTemp)+3    ))

//signed int ANO
uint8_t testdatatosend[50];
void TestSendData(uint8_t *dataToSend, uint8_t length);
void Test_Send_User (int16_t data1, int16_t data2, int16_t data3,
					int16_t data4, int16_t data5, int16_t data6,
					int16_t data7,int16_t data8,int16_t data9,
					int16_t data10);

//-------------------------------------------ANO_END---------------------------------------//






//---------------------------------------------MODBUS--------------------------------------//
MODBUS_TABLE modbus_table = {0};
void modbus_send(void);
//---------------------------------------------MODBUS_END--------------------------------------//


//-----------------------------------------UART_RECEIVE_TEST----------------------------//
uint8_t len;	
uint16_t times_rece_it=0;
//-----------------------------------------UART_RECEIVE_TEST_END----------------------------//


//testing
uint16_t cont1 =0;
uint8_t flip=0;
uint8_t times=0;
float t_pwm = 1600;
uint16_t ref_hz = 300;

//capture freq
char str[12];
char str2[12];
float main_clock_freq = 168e6;
uint16_t psc_val = 2564;     ///range = 1hz to 13kHZ
uint16_t ARR_val = 0xffff;
float timer_counting_freq;
float IC_Value1=0.0;
float IC_Value2=0.0;
float Diff = 0.0;
float freq = 0.0;
uint8_t Is_first_capture=0; //0-not captured, 1- captured


//pid part
float freq_to_m_rpm;
float motor_to_wheel;
float e_pre=0;
float e_now=0;
float delta_e=0;
float delta_t=1;  //This is changing, hard to capture?
float I_e=0;
float kp=0.0001;
float ki=0.00005;
float kd=0.00001;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) //FOR capturing motor pwm. tim8 ch1
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) //If tim8 ch1 prompts an interrupt
	{
		if(Is_first_capture==0) //if first captured?
		{
			IC_Value1=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1); //Read the timestamp
			Is_first_capture=1; 
		}
		
		else if (Is_first_capture) //not the first capture
		{
			IC_Value2 = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1); //Record the second capture
			if(IC_Value2 > IC_Value1) 
								      /*
										Since we have adjusted psc to make sure 
										the time difference between the two values are smaller
										than the time needed for counter to count from 0 to arr,
										then there are only two cases: IC2 and IC1 are on the same slope
										in which IC2 >ic1
										OR, IC2 and ic1 are on different slope in which ic2 MUST be smaller
										than ic1
										*/
			{
				Diff = IC_Value2 - IC_Value1;
			}
			else if(IC_Value2 < IC_Value1) //On different slope
			{
				Diff=((ARR_val-IC_Value1)+IC_Value2) +1; //i don't understand why plus 1; 
			}
			else
			{
				Error_Handler();  //Built in hal func for which i have no idea what it is doing
			}
			//freq = (168e6/72)/Diff;
			freq = (timer_counting_freq)/Diff;
									/*
									//for psc = 72
									//168e6/72 is the freq of the timer, where 72 is the psc
									//168e6/72 gives how quick the counter is counting from 0 to ARR
									//So the timer counts every (72/168e6=4.2e-7) seconds and takes  
									// 4.2e-7 *ARR(65536) = 0.028 seconds to reach ARR
									//So we need to make sure that the time difference between two 
									//rising edge is within 0.028 seconds(as in this mode
									//when rising edge is detected and this function is called, 
									// the current counting number will not be reset to zero)
									//otherwise the algo will fail
									//in other words, the minimum freq that we can detect
									// is (168e6/72)/ARR = 35.6 = 1/0.028
									//If the minimum freq required is even lower, we can adjust
									// the psc (72 here) to lower the mini detectable freq
									//The max detectable freq is the counting freq of the tim
									//or for a better precision 2 or 3 or 5times lower than
									// the couting freq of the tim
									//so here maxf = (168e6/72)/accurate_factor(5) = 466 kHz.
									//so detectable range is 35.6 to 466 khz

									//for psc = 2564  range = 1hz to 13kHZ
									//Might change to this psc in the future---- did --very good result
									*/
			
		
			Is_first_capture =0; //reset ready for next capture.	
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
  MX_DMA_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
	while(BMI088_init())  //Init the IMU
    {
        ;
    }
	
	
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1500);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1500);
	//Tim8 ch1 for capture 
	HAL_TIM_IC_Start_IT(&htim8,TIM_CHANNEL_1);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_TC); /////Must have this !!!!!!!!
	timer_counting_freq = main_clock_freq/psc_val;
	HAL_Delay(3000);
	//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1600);
	//HAL_Delay(3000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  
/*	  
//20ms ARR 20000
//1600 motor: low back , wheel small to left (driver view) ----- 1.6ms ----------never give motor value greater than 1500
//1400 motor: low forward, wheel small to left -------------------1.4ms 


//wheel
//left max-----1800 ----1.8ms
//right max ---- 1100  --- 1.2ms

//motor
//1500 zero ----------------------1.5ms   ------- 0hz
//2000 max ------------------------2ms -- forward  ---- 753~761hz
//could be higher
//never put value smaller than 1500;
  */

	  
	  
	  
	  
	  
	  
	  
	  
/*
//---------------------------------------Old_print_func-------------------------------------------//

//		sprintf(str2, "%d", ref_hz);
//		sprintf(str, "%f", freq);
//		
//		//HAL_UART_Transmit(&huart1, "Actual:", 12, 100);
//		HAL_UART_Transmit(&huart1, str, 12, 100);
//		HAL_UART_Transmit(&huart1, "    \r\n", 12, 100);
//		
//		//HAL_UART_Transmit(&huart1, "ref:", 12, 100);
//		HAL_UART_Transmit(&huart1, str2, 12, 100);
//		HAL_UART_Transmit(&huart1, "    \r\n", 12, 100);
//---------------------------------------Old_print_func-------------------------------------------//
*/








		






//-------------------------------------PID_adjust----------------------------------------------//
//	if(flip==0)
//	{
//		cont1 ++;
//		if (cont1 >100)
//			{
//				ref_hz +=600;
//				if(ref_hz<700)
//				{
//					ref_hz = ref_hz;
//				}
//				else
//				{
//					ref_hz =400;
//				}
//				
//				times ++;
//				
//				if(times >1)
//				{
//					flip=1;
//					times = 0;
//				}
//					
//				
//				cont1 = 0;
//			}
//		
//	}
//	
//	if(flip==1)
//	{
//		cont1 ++;
//		if (cont1 >100)
//			{
//				ref_hz -=600;
//				if(ref_hz<701 && ref_hz>200)
//				{
//					ref_hz = ref_hz;
//				}
//				else
//				{
//					ref_hz =200;
//				}
//				
//				times ++;
//				
//				if(times >1)
//				{
//					flip=0;
//					times = 0;
//				}
//				
//				cont1 = 0;
//			}
//			
//	}
		
//--------------------------------------------PID_adjust------------------------------------------------//
		
		
		
	
	
	
	
	
	
	
	
	
//-------------------------------------PID----------------------------------------------------//

//	e_pre = e_now; // a possible bug here
//	
//	e_now = ref_hz - freq;  //positive means low speed, so kp is positive
//	I_e += e_now;   //ki positive 

//	delta_e = (e_now - e_pre)/delta_t;   /// watch sign   ---- possible bugs here 
//	//if the gradient is increasing

//	t_pwm = kp*e_now + ki*I_e + kd*delta_e;  
//	if(t_pwm>1.0)
//	{
//		t_pwm = 1.0;
//	}				
//	if(t_pwm<-1.0) t_pwm=-1.0;	
//	
//	t_pwm =1500 + t_pwm*500; //motor moves forward range: 1500 to 2000  (1.5ms to 2ms) 
//	if(t_pwm<1500) t_pwm=1500;
//	if(t_pwm>2000) t_pwm=2000;
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, t_pwm);
//-------------------------------------PID_END----------------------------------------------------//








	
//---------------------------------------IMU----------------------------------------------//
    BMI088_read(gyro, accel, &temp);


//---------------------------------------IMU_END----------------------------------------------//








//--------------------------------------ANO-----------------------------------------------//
//	HAL_Delay(20);  //20ms
//	gyro[0] *= trans_scale_gyro;
//	gyro[1] *= trans_scale_gyro;
//	gyro[2] *= trans_scale_gyro;
//	accel[0] *= trans_scale_acce;
//	accel[1] *= trans_scale_acce;
//	accel[2] *= trans_scale_acce;
//	
	//Test_Send_User(ref_hz,freq,gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2],temp,10);

//--------------------------------------ANO_END-----------------------------------------------//






	
//-------------------------------------------MODBUS---------------------------------------//
	
	//modbus_send();
	//HAL_Delay(500);
//-------------------------------------------MODBUS_END---------------------------------------//






//---------------------------------TEST_UART_RECEIVE--------------------------------------------//
       if(USART_RX_STA&0x8000)
		{			
			//uint8_t USART_Receiver[200];          //��������
			//len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			//printf("\r\nThe msg you are sending is:\r\n");
			//HAL_UART_Transmit(&huart1,(uint8_t*)&USART_RX_BUF,len,1);	//���ͽ��յ�������
			//while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		//�ȴ����ͽ���
			//printf("\r\n\r\n");//���뻻��
			//USART_RX_STA=0;
			//testRece1++;
			//USART_Receiver = *USART_RX_BUF;
			//usartReceiveOneData(&testRece1,&testRece2,&testRece3);
			//usartSendData(USART_Receiver[1],USART_RX_BUF[1],USART_RX_BUF[5],testSend4);
			//usartSendData(testRece1,testRece2,testRece3,testSend4);
			
			//unsigned char USART_Receiver              = 0;          //��������
			static unsigned char checkSum             = 0;
			static short j=0,k=0;
			static short dataLength                   = 0;
			bool yes =1;
			
		    unsigned char  receiveBuff[16] = {0};    //16 bytes each byte has 8 bits  [i]means one byte area    
			//ͨ��Э�鳣��
		    const unsigned char header[2]  = {0x55, 0xaa};
			//const unsigned char ender[2]   = {0x0d, 0x0a};
			
			
			receiveBuff[0]=header[0];         //buf[0]
			receiveBuff[1]=header[1];         //buf[1]
			checkSum = 0x00;				  //У��ͳ�ʼ��
			receiveBuff[2]=USART_RX_BUF[2];         //buf[1] //���ݵĳ���
			dataLength = receiveBuff[2];
			for(j=0;j<dataLength;j++)
				receiveBuff[j+3] = USART_RX_BUF[j+3];  //data length is 5 bytes  two var have two bytes each
														// one variable has one byte: the ctrlFlag
			receiveBuff[3+dataLength] = USART_RX_BUF[3+dataLength];  //buff[8]
			checkSum = getCrc8(receiveBuff,3 + dataLength);  //try getCrc8(&receiveBuff
			if(checkSum != receiveBuff[3+dataLength])
				{
					printf("crc8 error");
					yes = 0;
					return 0;
				}
			
			
			if(yes)
			{
				receiveBuff[3+dataLength+1] = USART_RX_BUF[3+dataLength+1];  //get 0x0d
				receiveBuff[3+dataLength+2] = USART_RX_BUF[3+dataLength+2];  //get 0x0a
				for (k=0; k<2; k++)
					{
						left.data[k] = receiveBuff[k+3];
						right.data[k] = receiveBuff[k+5];
					}
				testRece1 = (int) left.d;
				testRece2 = (int) right.d;
				testRece3 = receiveBuff[3+dataLength-1];
			}
			
			
			//-----------------------------------------------------------------
			//���һ�����ݰ��Ľ��գ���ر������㣬�ȴ���һ�ֽ�����
			checkSum = 0;
			dataLength=0;
			j=0;
			k=0;
			yes=1;
			
			
			
			usartSendData(testRece1,testRece2,testRece3,testSend4);
		}
		
		
		//else
		//{
//			times++;
//			if(times%5000==0)
//			{
//				printf("\r\nALIENTEK\r\n");
//				printf("ALIENTEK\r\n\r\n\r\n");
//			}
//			if(times%2==0)printf("please input msg\r\n");  
//			//if(times%30==0)LED0=!LED0;//��˸LED,��ʾϵͳ��������.   
//			HAL_Delay(1000);
		//}
	HAL_Delay(20);
//---------------------------------TEST_UART_RECEIVE_END--------------------------------------------//




//while end scope 		
	}
  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
  

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






//--------------------------------------ANO-----------------------------------------------//

///*
void TestSendData(uint8_t *dataToSend, uint8_t length)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&testdatatosend, length, 0xffff);
}

void Test_Send_User (int16_t data1, int16_t data2, int16_t data3,
					int16_t data4, int16_t data5, int16_t data6,
					int16_t data7,int16_t data8,int16_t data9,
					int16_t data10)
{
	uint8_t _cnt=0;
	testdatatosend[_cnt++]=0xAA; //head 
	testdatatosend[_cnt++]=0x05;  //target address
	testdatatosend[_cnt++]=0xAF; //receriver add ??  these two are default -- don't need to change 
	testdatatosend[_cnt++]=0xF1;  //function id --- just choice F1
	testdatatosend[_cnt++]=0;   // length of data --  set to zero for now 
	
	testdatatosend[_cnt++]=BYTE1(data1);
	testdatatosend[_cnt++]=BYTE0(data1);
	
	testdatatosend[_cnt++]=BYTE1(data2);
	testdatatosend[_cnt++]=BYTE0(data2);
	
	testdatatosend[_cnt++]=BYTE1(data3);
	testdatatosend[_cnt++]=BYTE0(data3);
	
	testdatatosend[_cnt++]=BYTE1(data4);
	testdatatosend[_cnt++]=BYTE0(data4);
	
	testdatatosend[_cnt++]=BYTE1(data5);
	testdatatosend[_cnt++]=BYTE0(data5);
	
	testdatatosend[_cnt++]=BYTE1(data6);
	testdatatosend[_cnt++]=BYTE0(data6);
	
	testdatatosend[_cnt++]=BYTE1(data7);
	testdatatosend[_cnt++]=BYTE0(data7);
	
	testdatatosend[_cnt++]=BYTE1(data8);
	testdatatosend[_cnt++]=BYTE0(data8);
	
	testdatatosend[_cnt++]=BYTE1(data9);
	testdatatosend[_cnt++]=BYTE0(data9);
	
	testdatatosend[_cnt++]=BYTE1(data10);
	testdatatosend[_cnt++]=BYTE0(data10);
	
	testdatatosend[4] = _cnt - 5;    //the fifth bit [4] stores the length of data, we set this to zero initially, now we need to calculate
									//_cnt stores the length of the whole signal up till now, we minus 5(the first five bits eg header, target address balalla to get the length of data
									//see
									//https://www.bilibili.com/video/BV1TJ411r7a3?from=search&seid=16506637512618591906
									//
	
	uint8_t sum = 0;
	
	for(uint8_t i=0; i<_cnt; i++)
		sum += testdatatosend[i];
		
	testdatatosend[_cnt++]=sum;   //sum is the sum check bits, the last bits in the signal
	
	TestSendData(testdatatosend, _cnt);
}











//--------------------------------------ANO_END-----------------------------------------------//











//---------------------------------MODBUS-----------------------------------------------------//
void modbus_send(void)
{ 
	static uint8_t feedback_buff[128] = {0};
    static uint32_t bufflen = 0;
	modbus_table.degree = temp;
	modbus_table.acce_x = accel[0]*trans_scale_acce;
	modbus_table.acce_y = accel[1]*trans_scale_acce;
	modbus_table.acce_z = accel[2]*trans_scale_acce;
	//modbus_table.
	modbus_feedback(&modbus_table, 0x02, 6, feedback_buff, &bufflen);
	HAL_UART_Transmit(&huart1, feedback_buff, bufflen, 5000);
}
//---------------------------------MODBUS_END-----------------------------------------------------//












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