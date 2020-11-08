
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "mbotLinuxUsart.h"//���ø�ͷ�ļ���ʹ�ã�ͨ��Э���ǰ��

//���Է��ͱ���
short testSend1   =5000;
short testSend2   =2000;
short testSend3   =1000;
unsigned char testSend4 = 0x05;

//���Խ��ձ���
int testRece1     =0;
int testRece2     =0;
unsigned char testRece3 = 0x00;


int main(void)
{	
//======================================Ӳ����ʼ��====================================================
	delay_init();	    	                        //��ʱ������ʼ��	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�����ж����ȼ�����2
	uart_init(115200);	                            //���ڳ�ʼ��Ϊ115200
	
//=======================================ѭ������=====================================================
	while(1)
	{
		//����Ҫ���͵�ROS�����ݣ��Ӹú���������ǰ�������ݷ�Χ��-32768 - +32767�������ĸ����ݵķ�Χ(0 - 255)
		usartSendData(testSend1,testSend2,testSend3,testSend4);
		//�������ʱ
		delay_ms(13);
	} 
}

//====================================�����жϷ������=================================================
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
		 USART_ClearITPendingBit(USART1,USART_IT_RXNE);//��������жϱ�־λ
		 //��ROS���յ������ݣ���ŵ���������������
		 usartReceiveOneData(&testRece1,&testRece2,&testRece3);
	 }
}
//===========================================END=======================================================
