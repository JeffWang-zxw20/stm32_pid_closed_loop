#include "motor.h"

void Motor_Init(void)
{
    MX_TIM1_Init();
    MX_TIM2_Init();
    PWM3_Setus(1500);
    PWM4_Setus(1500 - DIR_OFFSET_US);
}

//设置油门PWM脉宽 us -500~500
void motor_set_acce_us(int16_t us)
{
    if(us > 500)
    {
        us = 500;
    }
    else if(us < -500)
    {
        us = -500;
    }
    PWM3_Setus(1500 + us);
}


//设置转向舵机PWM脉宽 -500~500
void motor_set_dir_us(int16_t us)
{
    if(us > 500)
    {
        us = 500;
    }
    else if(us < -500)
    {
        us = -500;
    }

    us = us + DIR_OFFSET_US;
    PWM4_Setus(1500 - us);
}

void PWM1_IO_SHUTDOWN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void PWM2_IO_SHUTDOWN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void PWM1_IO_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void PWM2_IO_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void motor_io_delay(void)
{
    volatile int i = 0;
    for(i = 0; i <= 50; i++)
    {

    }
}

//值为-1000 ~ 1000
void motor_set(int16_t value)
{
    static int16_t last_value = 0;
    
    value = -value;

    if(value > 8000)
    {
        value = 8000;
    }
    else if(value < -8000)
    {
        value = -8000;
    }

    if(value == 0)
    {
        PWM1_Setus(0);
        PWM2_Setus(0);
        PWM1_IO_SHUTDOWN();
        PWM2_IO_SHUTDOWN();
    }
    else if(value > 0)
    {
        if(last_value == 0)
        {
            PWM2_Setus(0);
            PWM2_IO_SHUTDOWN();
            PWM1_IO_INIT();
            PWM1_Setus(value);
        }
        else if(last_value > 0)
        {        
            PWM1_Setus(value);
        }
        else if(last_value < 0)
        {
            PWM2_Setus(0);
            PWM2_IO_SHUTDOWN();
            motor_io_delay();
            PWM1_IO_INIT();
            PWM1_Setus(value);
        }
    }
    else if(value < 0)
    {
        if(last_value == 0)
        {
            PWM1_Setus(0);
            PWM1_IO_SHUTDOWN();
            PWM2_IO_INIT();
            PWM2_Setus(-value);
        }
        else if(last_value > 0)
        {
            PWM1_Setus(0);
            PWM1_IO_SHUTDOWN();
            motor_io_delay();
            PWM2_IO_INIT();
            PWM2_Setus(-value);
        }
        else if(last_value < 0)
        {
            PWM2_Setus(-value);
        }
    }
    last_value = value;
}

