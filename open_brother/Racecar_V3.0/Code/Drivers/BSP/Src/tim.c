
#include "tim.h"



TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

TIM_OC_InitTypeDef sConfigOC12;
TIM_OC_InitTypeDef sConfigOC34;

__weak void MY_SYSCLK_Callback(void)
{
    return;
}

//PWM3 PWM4
void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    __HAL_RCC_TIM1_CLK_ENABLE();
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 168 - 1;                 //定时器分频系数
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;    //向上计数
    htim1.Init.Period = 5000 - 1;                   //自动重装载值
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    HAL_TIM_Base_Init(&htim1);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }

    sConfigOC34.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC34.Pulse        = 1500;                 //脉宽  单位：us
    sConfigOC34.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC34.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC34.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC34.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC34, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC34, TIM_CHANNEL_3);

    HAL_TIM_PWM_MspInit(&htim1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

//PWM1 PWM2
void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;                  //84MHz，不分频
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;    //向上计数
    htim2.Init.Period = 8400 - 1;                    //自动重装载值，频率为10kHz，精度为-8400到+8400
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.RepetitionCounter = 0;
    HAL_TIM_Base_Init(&htim2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }

    sConfigOC12.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC12.Pulse        = 0;                     //脉宽  单位：us
    sConfigOC12.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC12.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC12.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC12.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC12, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC12, TIM_CHANNEL_3);

    HAL_TIM_PWM_MspInit(&htim2);
}

//谁用CUBE生成代码谁就是傻逼
//编码器脉冲计数
void MX_TIM3_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
   
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    TIM3->SMCR |= 0x07;         //SMS = 111 外部时钟模式1
    TIM3->SMCR |= 0x06 << 4;    //TS = 110 触发选择：滤波后的定时器输入2 (TI2FP2)
    TIM3->CR1 |= 1 << 0;        //CEN = 1 计数器使能
}

//系统定时器
//1ms一次中断
void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 84 - 1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 1000 - 1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }
    HAL_TIM_Base_Start_IT(&htim6);
}

void PWM1_Setus(uint32_t us)
{
    sConfigOC12.Pulse = us;

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC12, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void PWM2_Setus(uint32_t us)
{
    sConfigOC12.Pulse = us;

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC12, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void PWM3_Setus(uint32_t us)
{
    sConfigOC34.Pulse = us;

    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC34, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}

void PWM4_Setus(uint32_t us)
{
    sConfigOC34.Pulse = us;

    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC34, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)
    {
        GPIO_InitTypeDef GPIO_InitStruct;

        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;

        GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if(htim->Instance == TIM2)
    {
        GPIO_InitTypeDef GPIO_InitStruct;

        __HAL_RCC_TIM2_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

        GPIO_InitStruct.Pin = GPIO_PIN_15;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if(htim->Instance == TIM3)
    {
        GPIO_InitTypeDef GPIO_InitStruct;

        __HAL_RCC_TIM3_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;

        GPIO_InitStruct.Pin = GPIO_PIN_4;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(tim_baseHandle->Instance==TIM5)
    {
        /* TIM5 clock enable */
        __HAL_RCC_TIM5_CLK_ENABLE();
    
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM5 GPIO Configuration    
        PA0-WKUP     ------> TIM5_CH1 
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if(tim_baseHandle->Instance==TIM6)
    {
        __HAL_RCC_TIM6_CLK_ENABLE();

        HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }
}


