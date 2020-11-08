#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
void usart1_tx_dma_init(void)
{
    //enable the DMA transfer for the receiver request
    //ʹ��DMA���ڽ���
	////ʹ�ܴ���DMA����
	//���� 1 �� DMA ����ʵ���Ǵ��ڿ��ƼĴ��� CR3 ��λ 7 �����Ƶ�
	//�������ǿ���ͨ�� ֱ�Ӳ����Ĵ�����ʽ��ʵ�֣� 
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len) //data is a pointer
{

    //disable DMA //we clean it and disable it temporarily because we want to transmit new stuff
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);
    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    //clear flag
    //�����־λ
	////TCIF3 is the transfer complete interrupt flag of stream3, _7means channel7
	//note stream3 means the 4th see zhengdian's pdf p352
	//Here is just cleaning the transmit complete(TCIF) and half transmit complete(HTIF)flag
	//we clean it because we want to transmit new stuff
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_HTIF7);

    //set data address
    //�������ݵ�ַ
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data); //data is a pointer
    //set data length
    //�������ݳ���
    hdma_usart1_tx.Instance->NDTR = len;

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}


