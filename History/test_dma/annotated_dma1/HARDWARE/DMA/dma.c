#include "dma.h"
#include "lcd.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//DMA��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/4/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

DMA_HandleTypeDef  UART1TxDMA_Handler;      //DMA���
//�� HAL ���У��κ�һ������ʹ�� DMA �����裬���ĳ�ʼ���ṹ����������һ�� DMA_HandleTypeDef 
//ָ�����͵ĳ�Ա�������� HAL �����������ָ��ġ�
//Hdmatx ���� DMA_HandleTypeDef �ṹ��ָ������


//DMAx�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMAͨ��ѡ��,@ref DMA_channel DMA_CHANNEL_0~DMA_CHANNEL_7
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx)
{ 
	if((u32)DMA_Streamx>(u32)DMA2)//�õ���ǰstream������DMA2����DMA1
	{
        __HAL_RCC_DMA2_CLK_ENABLE();//DMA2ʱ��ʹ��	
	}else 
	{
        __HAL_RCC_DMA1_CLK_ENABLE();//DMA1ʱ��ʹ�� 
	}
    
	
	//Hdmatx ���� DMA_HandleTypeDef �ṹ��ָ������
    __HAL_LINKDMA(&UART1_Handler,hdmatx,UART1TxDMA_Handler);    //��DMA��USART1��ϵ����(����DMA)
    
    //Tx DMA����
    UART1TxDMA_Handler.Instance=DMA_Streamx;                            //������ѡ��
    UART1TxDMA_Handler.Init.Channel=chx;                                //ͨ��ѡ��
    UART1TxDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             //�洢��������
	//���籾ʵ������Ҫ�õ� DMA2_Stream7 �� DMA_CHANNEL_4�����ڴ��������ֵ���͵��������跢�ͼĴ��� DR��
	//���Է���Ϊ�洢�������� DMA_MEMORY_TO_PERIPH��һ��һ���ֽڷ��ͣ�
	//��Ҫ���� �����Զ����ӣ������Ǵ洢������ģʽ DMA_MINC_ENABLE
    UART1TxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //���������ģʽ
    UART1TxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //�洢������ģʽ
    UART1TxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //�������ݳ���:8λ
    UART1TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //�洢�����ݳ���:8λ
    UART1TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //������ͨģʽ
    UART1TxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //�е����ȼ�
    UART1TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
    UART1TxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
    UART1TxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //�洢��ͻ�����δ���
    UART1TxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //����ͻ�����δ���
    
    HAL_DMA_DeInit(&UART1TxDMA_Handler);   
    HAL_DMA_Init(&UART1TxDMA_Handler);
} 


//����һ��DMA����
//huart:���ھ��
//pData�����������ָ��
//Size:�����������
//hdmatx ���������ṹ��ĳ�Ա������������ʵ�ʾ��� UART1_Handler �� ��Ա����
//���� HAL ���У��κ�һ������ʹ�� DMA �����裬���ĳ�ʼ���ṹ����������һ�� DMA_HandleTypeDef 
//ָ�����͵ĳ�Ա�������� HAL �����������ָ��ġ�
//Hdmatx ���� DMA_HandleTypeDef �ṹ��ָ������
void MYDMA_USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	
	// ʹ�� DMA �������ĺ���
	//��һ�������� DMA ������ڶ����Ǵ���Դ��ַ���������Ǵ���Ŀ���ַ(here is ���ڿ��ƼĴ��� CR3 ��λ 7 )
	//This is because we use dma to send data from memory to uart, so destination of dma trans is the 
	// cr register of uart
	
	//���ĸ��Ǵ�������ݳ��ȡ�
    HAL_DMA_Start(huart->hdmatx, (u32)pData, (uint32_t)&huart->Instance->DR, Size);//����DMA����
    
    huart->Instance->CR3 |= USART_CR3_DMAT;//ʹ�ܴ���DMA����
	//���� 1 �� DMA ����ʵ���Ǵ��ڿ��ƼĴ��� CR3 ��λ 7 �����Ƶ�
	//�������ǿ���ͨ�� ֱ�Ӳ����Ĵ�����ʽ��ʵ�֣� 
	//data has been transported from memory to cr register in the uart
	//now we send the data from uart to pc 
}	  

 
