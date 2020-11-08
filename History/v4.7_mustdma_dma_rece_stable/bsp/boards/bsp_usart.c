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
	//#define USART_CR3_DMAT   USART_CR3_DMAT_Msk   /*!<DMA Enable Transmitter      */
	
}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len) //data is a pointer
{

    //disable DMA //we clean it and disable it temporarily because we want to transmit new stuff
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);
    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)  // // wait until DMA completion?? 
													//see https://community.st.com/s/question/0D50X00009XkaAtSAJ/how-to-clear-pending-dma-request
	
	// & here is bitwise and???
		
	//，DMA_SxCR 控制数据流到底使用哪一个通道，
	//每个数据流有 8 个通道可 供选择，每次只能选择其中一个通道进行 DMA 传输
	
	
	//Memory-to-peripheral mode: DMA_SxCR_EN
	//When this mode is enabled (by setting the EN bit in the DMA_SxCR register
	// CR is /*!< DMA stream x configuration register      */
	//so CR is an abbreviation of 
	//DMA stream x configuration register (DMA_SxCR) (x = 0..7) 
	
	
	
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);  //clear flag since //DMA cannot be reenabled until the transfer completed flag in LISR is cleared
    }
    //清除标志位
	////TCIF3 is the transfer complete interrupt flag of stream3, _7means channel7
	//note stream3 means the 4th see zhengdian's pdf p352
	//Here is just cleaning the transmit complete(TCIF) and half transmit complete(HTIF)flag
	//we clean it because we want to transmit new stuff
	
	//第一个是 DMA 中断状态寄存器，该寄存器总共有 2 个：DMA_LISR 和 DMA_HISR，
	//每个寄存器 管理 4 数据流（总共 8 个），DMA_LISR 寄存器用于管理数据流 0~3，
	//而 DMA_HISR 用于管理数据 流 4~7。这两个寄存器各位描述都完全一模一样，只是管理的数据流不一样。
	//here for Usart1_tx, we use stream7, so we use HISR
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);  //cleaning the transmit complete(TCIF) 
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_HTIF7); //CLEAN half transmit complete(HTIF)flag

	
	
	
    //set data address
    //设置数据地址
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data); //data is a pointer
	//DMA stream x memory 0 address register (DMA_SxM0AR) (x = 0..7) 
	//Base address of Memory area 0 from/to which the data will be read/written. 
	//These bits are write-protected. They can be written only if: 
	//– the stream is disabled (bit EN= '0' in the DMA_SxCR register) 
	//So that is why we disable the DMA first
	
	
    //set data length
    //设置数据长度
    hdma_usart1_tx.Instance->NDTR = len;
	//DMA stream x number of data register (DMA_SxNDTR) (x = 0..7)
	//Number of data items to be transferred (0 up to 65535).
	//This register can be written only when the stream is disabled 
	//--- so  we need to disable dma first

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_tx);
	//this is a define line
	//it actually set bits
	//((__HANDLE__)->Instance->CR |=  DMA_SxCR_EN)  // |= is Bitwise inclusive OR and assignment operator.
}


