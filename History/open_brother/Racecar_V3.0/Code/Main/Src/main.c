
#include "main.h"

//轮子一圈编码器10圈
//轮子外径6cm
uint8_t u5rxdata[UART_DMA_RX_SIZE] = {0};
uint8_t u6rxdata[UART_DMA_RX_SIZE] = {0};
DBUS_MSG dbus_msg = {0};
DBUS_CTRL dbus_ctrl = {0};
ENCODER encoder = {0};
MAIN_CTRL main_ctrl = {0};
CLOCK clock = {0};
MODBUS_MSG modbus_msg = {0};
MODBUS_TABLE modbus_table = {0};

PID_HANDLE hspeed = {0};
int rxlen = 0;

void MY_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5)
    {
        if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);
            rxlen = UART_DMA_RX_SIZE - DMA1_Stream0->NDTR;

            HAL_UART_DMAStop(huart);
            HAL_UART_Receive_DMA(huart, u5rxdata, UART_DMA_RX_SIZE);
            if(rxlen == 18)
            {
                dbusGetMsg(&dbus_msg, u5rxdata);
                dbusGetCtrl(&dbus_msg, &dbus_ctrl);
            }

        }
    }
    else if(huart->Instance == USART6) ///---------------------------------receive data
    {
        if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);
   

            HAL_UART_DMAStop(huart);
            HAL_UART_Receive_DMA(huart, u6rxdata, UART_DMA_RX_SIZE);

            if(modbus_pack_analize(u6rxdata, rxlen, &modbus_msg) == 0)
            {
                if(modbus_msg.order == MODBUS_WRITE)
                {
                    table_write(modbus_msg.offset, modbus_msg.len, modbus_msg.data, &modbus_table);
                }
            }
        }        
    }
    
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5)
    {

    }
}


 void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 {
 	if(huart->Instance == UART5)
 	{
        __HAL_UART_CLEAR_OREFLAG(&huart5);
 	}
 	else if(huart->Instance == USART6)
 	{
        __HAL_UART_CLEAR_OREFLAG(&huart6);
 	}     
 }

void MY_SYSCLK_Callback(void)
{
    clock.clock++;
    clock.clock2ms++;
    clock.clock5ms++;
    clock.clock10ms++;
    clock.clock50ms++;
    clock.clock100ms++;
}
 

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // if(GPIO_Pin == GPIO_PIN_5)
    // {
    //     if( (GPIOC->IDR & (1 << 5) ) != 0 ) //如果此时PC3确实是高电平
    //     {

    //仅此一个外部中断，没必要判断了
    if(EncoderReadDir() == 1)
    {
        encoder.cycle++;
    }
    else
    {
        encoder.cycle--;
    }
    //     }
    // }
}


int main(void)
{    
    HAL_Init();
    SystemClock_Config();

    pid_init(&hspeed);

    LED_Core_Init();
    MX_GPIO_Init();  ///wifi part need to study
    MX_DMA_Init();
    MX_IWDG_Init();
    MX_USART1_UART_Init();  //核心板输出
    MX_USART2_UART_Init();  //无线串口
    // MX_USART3_UART_Init();
    MX_UART5_Init();        //接收机
    MX_USART6_UART_Init();  //上位机  pc
    MX_TIM6_Init();

    Motor_Init();
    EncoderInit();

    HAL_UART_Receive_DMA(&huart5, u5rxdata, UART_DMA_RX_SIZE);
    HAL_UART_Receive_DMA(&huart6, u6rxdata, UART_DMA_RX_SIZE);
    
    while(1)
    {
        if(clock.clock2ms >= 2)
        {
            clock.clock2ms = 0;
            clock.count2ms++;
            clock_2ms_callback();
        }
        if(clock.clock5ms >= 5) //Send data to pc
        {
            clock.clock5ms = 0;
            clock.count5ms++;
            clock_5ms_callback();
        }
        if(clock.clock10ms >= 10)  //send AN0 for visulisation
        {
            clock.clock10ms = 0;            
            clock.count10ms++;
            clock_10ms_callback();
        }
        if(clock.clock50ms >= 50)
        {
            clock.clock50ms = 0;            
            clock.count50ms++;
            clock_50ms_callback();
        }        
        if(clock.clock100ms >= 100)
        {
            clock.clock100ms = 0;            
            clock.count100ms++;
            clock_100ms_callback();
        }            
    }
}

void clock_2ms_callback(void)
{


    return;
}

void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);


void MY_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    // if(__HAL_DMA_GET_FLAG(huart->hdmatx, DMA_FLAG_TCIF2_6) == RESET)
    // {
        huart->hdmatx->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);
        huart->hdmatx->Instance->NDTR = Size;
        huart->hdmatx->Instance->M0AR = (uint32_t)pData;        
        huart->hdmatx->Instance->PAR = (uint32_t)(&huart->Instance->DR);        
        __HAL_DMA_ENABLE(huart->hdmatx);
        // __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);

    // }
    // else
    // {
    //     __HAL_DMA_CLEAR_FLAG(huart->hdmatx, DMA_FLAG_TCIF2_6);
    //     __HAL_DMA_DISABLE(huart->hdmatx);
    // }
}



void clock_5ms_callback(void)
{
    static uint8_t feedback_buff[128] = {0};
    static uint32_t bufflen = 0;

    EncoderUpdatePulse(&encoder);
    modbus_table.pulse = encoder.pulse;  //table is a struct storing data
    modbus_table.cycle = encoder.cycle;

    modbus_feedback(&modbus_table, 0x02, 6, feedback_buff, &bufflen);
    // HAL_UART_Transmit_DMA(&huart6, feedback_buff, bufflen);
    HAL_UART_Transmit(&huart6, feedback_buff, bufflen, 5000);
    // MY_UART_Transmit_DMA(&huart6, feedback_buff, bufflen);
    
    switch (dbus_ctrl.mode)
    {
    case CTRL_MODE_STOP:
        main_ctrl.acce_us = 0;
        main_ctrl.dir_us = 0;
        break;

    case CTRL_MODE_MANUAL_FAST:
        main_ctrl.acce_us = dbus_ctrl.ch_acce / 660.0 * 8000.0;
        main_ctrl.dir_us = dbus_ctrl.ch_dir / 660.0 * 500.0;
        break;


    case CTRL_MODE_MANUAL_SLOW:
        main_ctrl.acce_us = dbus_ctrl.ch_acce / 660.0 * 4000.0;
        main_ctrl.dir_us = dbus_ctrl.ch_dir / 660.0 * 500.0;
        break;

    case CTRL_MODE_AUTO_DIR:
        main_ctrl.acce_us = 0;
        main_ctrl.dir_us = modbus_table.trg_dir_us;
        break;

    case CTRL_MODE_AUTO_ALL:
        main_ctrl.target_speed = modbus_table.trg_speed;
        main_ctrl.dir_us = modbus_table.trg_dir_us;
        goto SPEED_CTRL;
    
    case CTRL_MODE_MANUAL_SPEED:
        main_ctrl.target_speed = (int16_t)((float)dbus_ctrl.ch_acce / 660.0 * 1000.0);
        main_ctrl.dir_us = dbus_ctrl.ch_dir / 660.0 * 500.0;
SPEED_CTRL:

        main_ctrl.acce_us = (int16_t)pidProcess(&hspeed, (float)main_ctrl.target_speed, encoder.pulse);
        
        break;
    
    default:
        break;
    }
    // motor_set_acce_us(main_ctrl.acce_us);
    motor_set(main_ctrl.acce_us);
    motor_set_dir_us(main_ctrl.dir_us);
}

void clock_10ms_callback(void)
{
    static uint8_t buff[128] = {0};
    static uint16_t len;    
    static int16_t data[8];

    data[0] = encoder.pulse;
    data[1] = main_ctrl.target_speed;
    data[2] = main_ctrl.acce_us;
    ANOPrintS16(1, data, 3, buff, &len);
    HAL_UART_Transmit_DMA(&huart2, buff, len);    
    return;
}

void clock_50ms_callback(void)
{
    static uint32_t last_dbus_count = 0;

    if(dbus_msg.count > last_dbus_count)
    {
        last_dbus_count = dbus_msg.count;
        HAL_IWDG_Refresh(&hiwdg);
    }

    return;
}


void clock_100ms_callback(void)
{
    LED_Core_Turn();
    return;
}








