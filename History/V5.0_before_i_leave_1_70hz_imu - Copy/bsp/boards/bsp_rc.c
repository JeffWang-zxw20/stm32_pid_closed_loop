#include "bsp_rc.h"
#include "main.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

//ROS//
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;


void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
	//Select DMA enable (DMAR) in USART_CR3 if Multi buffer 
	//Communication is to take place. 
	//Configure the DMA register as explained in multibuffer communication. 
	
	//SET_BIT this func set the bit USART_CR3_DMAR in SxCR register to 1 ?? guess 
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);  //NO idea why use

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
	
	
	
	
	
	
	//// CR is /*!< DMA stream x configuration register      */
	//so CR is an abbreviation of 
	//DMA stream x configuration register (DMA_SxCR) (x = 0..7) 
	//Memory-to-peripheral mode: DMA_SxCR_EN
	//When this mode is enabled (by setting the EN bit in the DMA_SxCR register
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);   //first disable, then modify, then enable to receive again
    }

	
	
	
	
	//成员变量 Instance 是用来设置寄存器基地址，例如要设置为 DMA2 的数据流 7，那么取值 为 DMA2_Stream
	//PAR----DMA stream x peripheral address register (DMA_SxPAR) (x = 0..7)
	//Base address of the peripheral data register from/to 
	//which the data will be read/written. These bits are write-protected and 
	//can be written only when bit EN = '0' in the DMA_SxCR register.--- So need to disable first
    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR); /// &is getting address
	//& get the address of the Base address of the peripheral data register of USART3(&(USART3->DR))
	//USART3->DR---------Data register (USART_DR) 
	//Contains the Received or Transmitted data character,
	//depending on whether it is read from or written to.
	//When receiving with the parity enabled, the value read in the MSB bit is the received parity bit.

	
	
	
	
	//M0AR-----DMA stream x memory 0 address register (DMA_SxM0AR) (x = 0..7) 
	//Base address of Memory area 0 from/to which the data will be read/written. 
	//These bits are write-protected. 
	//They can be written only if: – the stream is 
	//disabled (bit EN= '0' in the DMA_SxCR register)
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf); //rx1_buf is the address of memory area 
	
	
	
	
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
	
	
	
	
	
	
	
	
	//NDTR----------DMA stream x number of data register (DMA_SxNDTR) (x = 0..7)
	// NDT[15:0]: Number of data items to transfer
	//Number of data items 
	//to be transferred (0 up to 65535). 
	//This register can be written only when the stream is disable
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num; //write buff number
	
	
	
	
	
	
	
	
	
    //enable double memory buffer
    //使能双缓冲区
	//set the DBM bits in the SxCR register
	//it is Bit 18 DBM: Double buffer mode 
	//This bits is set and cleared by software. 
	//0: No buffer switching at the end of transfer 
	//1: Memory target switched at the end of the DMA transfer. 
	//This bit is protected and can be written only if EN is ‘0’. 
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);




    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

}


void ROS_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	//enable the DMA transfer for the receiver request
    //使能DMA串口接收
	//Select DMA enable (DMAR) in USART_CR3 if Multi buffer 
	//Communication is to take place. 
	//Configure the DMA register as explained in multibuffer communication. 
	
	//SET_BIT this func set the bit USART_CR3_DMAR in SxCR register to 1 ?? guess 
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  //NO idea why use

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
	
	
	
	
	
	
	//// CR is /*!< DMA stream x configuration register      */
	//so CR is an abbreviation of 
	//DMA stream x configuration register (DMA_SxCR) (x = 0..7) 
	//Memory-to-peripheral mode: DMA_SxCR_EN
	//When this mode is enabled (by setting the EN bit in the DMA_SxCR register
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);   //first disable, then modify, then enable to receive again
    }

	
	
	
	
	//成员变量 Instance 是用来设置寄存器基地址，例如要设置为 DMA2 的数据流 7，那么取值 为 DMA2_Stream
	//PAR----DMA stream x peripheral address register (DMA_SxPAR) (x = 0..7)
	//Base address of the peripheral data register from/to 
	//which the data will be read/written. These bits are write-protected and 
	//can be written only when bit EN = '0' in the DMA_SxCR register.--- So need to disable first
    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR); /// &is getting address
	//& get the address of the Base address of the peripheral data register of USART3(&(USART3->DR))
	//USART3->DR---------Data register (USART_DR) 
	//Contains the Received or Transmitted data character,
	//depending on whether it is read from or written to.
	//When receiving with the parity enabled, the value read in the MSB bit is the received parity bit.

	
	
	
	
	//M0AR-----DMA stream x memory 0 address register (DMA_SxM0AR) (x = 0..7) 
	//Base address of Memory area 0 from/to which the data will be read/written. 
	//These bits are write-protected. 
	//They can be written only if: – the stream is 
	//disabled (bit EN= '0' in the DMA_SxCR register)
    //memory buffer 1
    //内存缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf); //rx1_buf is the address of memory area 
	
	
	
	
    //memory buffer 2
    //内存缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
	
	
	
	
	
	
	
	
	//NDTR----------DMA stream x number of data register (DMA_SxNDTR) (x = 0..7)
	// NDT[15:0]: Number of data items to transfer
	//Number of data items 
	//to be transferred (0 up to 65535). 
	//This register can be written only when the stream is disable
    //data length
    //数据长度
    hdma_usart1_rx.Instance->NDTR = dma_buf_num; //write buff number
	
	
	
	
	
	
	
	
	
    //enable double memory buffer
    //使能双缓冲区
	//set the DBM bits in the SxCR register
	//it is Bit 18 DBM: Double buffer mode 
	//This bits is set and cleared by software. 
	//0: No buffer switching at the end of transfer 
	//1: Memory target switched at the end of the DMA transfer. 
	//This bit is protected and can be written only if EN is ‘0’. 
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);




    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
	//in_rx_it = 1;
}






