#include "encoder.h"


//编码器方向引脚IO初始化
void Encoder_DIR_IO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

//编码器零位引脚IO初始化
void Encoder_ZERO_IO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);    
}

void EncoderInit(void)
{
    MX_TIM3_Init(); //编码器脉冲计数
    Encoder_DIR_IO_Init();
    Encoder_ZERO_IO_Init();
}


inline int8_t EncoderReadDir(void)
{
    if((GPIOC->IDR & 0x10) == 0)
    {
        return -1;
    }
    else
    {
        return 1;
    }
}

inline void EncoderUpdatePulse(P_ENCODER phdl)
{
    int16_t count_pulse;

    count_pulse = TIM3->CNT;
    TIM3->CNT = 0;
    if(EncoderReadDir() == 1)
    {
        phdl->pulse = count_pulse;
    }
    else if(EncoderReadDir() == -1)
    {
        phdl->pulse = -count_pulse;
    }
}

