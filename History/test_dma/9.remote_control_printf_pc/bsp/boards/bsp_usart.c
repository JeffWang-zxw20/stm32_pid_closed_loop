#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
void usart1_tx_dma_init(void)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
	////使能串口DMA发送
	//串口 1 的 DMA 发送实际是串口控制寄存器 CR3 的位 7 来控制的
	//这里我们可以通过 直接操作寄存器方式来实现： 
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len) //data is a pointer
{

    //disable DMA //we clean it and disable it temporarily because we want to transmit new stuff
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);
    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    //clear flag
    //清除标志位
	////TCIF3 is the transfer complete interrupt flag of stream3, _7means channel7
	//note stream3 means the 4th see zhengdian's pdf p352
	//Here is just cleaning the transmit complete(TCIF) and half transmit complete(HTIF)flag
	//we clean it because we want to transmit new stuff
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_HTIF7);

    //set data address
    //设置数据地址
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data); //data is a pointer
    //set data length
    //设置数据长度
    hdma_usart1_tx.Instance->NDTR = len;

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}


