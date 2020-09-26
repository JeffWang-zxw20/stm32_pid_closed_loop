#include "mbotLinuxUsart.h"
#include "usart.h"         //包含printf

/*--------------------------------发送协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/

/*--------------------------------接收协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/


/**************************************************************************
通信的发送函数和接收函数必须的一些常量、变量、共用体对象
**************************************************************************/

extern uint8_t  USART_RX_BUF[USART_REC_LEN]; 


//数据接收暂存区
unsigned char  receiveBuff[16] = {0};         
//通信协议常量
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};

//发送数据（左轮速、右轮速、角度）共用体（-32767 - +32768）
union sendData
{
	short d;
	unsigned char data[2];
}leftVelNow,rightVelNow,angleNow,acc_x,acc_y,acc_z;



//左右轮速控制速度共用体
union receiveData
{
	short d;
	unsigned char data[2];
}leftVelSet,rightVelSet;



union imu_send
{
	float d;
	unsigned char data[4];
}gx,gy,gz,ax,ay,az;
/**************************************************************************
函数功能：通过串口中断服务函数，获取上位机发送的左右轮控制速度、预留控制标志位，分别存入参数中
入口参数：左轮轮速控制地址、右轮轮速控制地址、预留控制标志位
返回  值：无特殊意义
**************************************************************************/
int usartReceiveOneData(int *p_leftSpeedSet,int *p_rightSpeedSet,unsigned char *p_crtlFlag)
{
	unsigned char USART_Receiver              = 0;          //接收数据
	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //一帧数据传送开始标志位
	static short dataLength                   = 0;

	//USART_Receiver = *USART_RX_BUF;   //@@@@@#####如果你使用不是USART1更改成相应的，比如USART3
	//接收消息头
	if(Start_Flag == START)
	{
		if(USART_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)        //数据头两位 //buf[0]
			{
				Start_Flag = !START;              //收到数据头，开始接收数据
				//printf("header ok\n");
				receiveBuff[0]=header[0];         //buf[0]
				receiveBuff[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //缓冲区初始化
				checkSum = 0x00;				  //校验和初始化
			}
		}
		else 
		{
			USARTReceiverFront = USART_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://接收左右轮速度数据的长度
				receiveBuff[2] = USART_Receiver;
				dataLength     = receiveBuff[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://接收所有数据，并赋值处理 
				receiveBuff[j + 3] = USART_Receiver;        //buf[3] - buf[7]					
				j++;
				if(j >= dataLength)                         
				{
					j = 0;
					USARTBufferIndex++;
				}
				break;
			case 2://接收校验值信息
				receiveBuff[3 + dataLength] = USART_Receiver;
				checkSum = getCrc8(receiveBuff, 3 + dataLength);
				  // 检查信息校验值
				if (checkSum != receiveBuff[3 + dataLength]) //buf[8]
				{
					//printf("Received data check sum error!");
					return 0;
				}
				USARTBufferIndex++;
				break;
				
			case 3://接收信息尾
				if(k==0)
				{
					//数据0d     buf[9]  无需判断
					k++;
				}
				else if (k==1)
				{
					//数据0a     buf[10] 无需判断

					//进行速度赋值操作					
					 for(k = 0; k < 2; k++)
					{
						leftVelSet.data[k]  = receiveBuff[k + 3]; //buf[3]  buf[4]
						rightVelSet.data[k] = receiveBuff[k + 5]; //buf[5]  buf[6]
					}				
					
					//速度赋值操作
					*p_leftSpeedSet  = (int)leftVelSet.d;
					*p_rightSpeedSet = (int)rightVelSet.d;
					
					//ctrlFlag
					*p_crtlFlag = receiveBuff[7];                //buf[7]
					
					//-----------------------------------------------------------------
					//完成一个数据包的接收，相关变量清零，等待下一字节数据
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}
/**************************************************************************
函数功能：将左右轮速和角度数据、控制信号进行打包，通过串口发送给Linux
入口参数：实时左轮轮速、实时右轮轮速、实时角度、控制信号（如果没有角度也可以不发）
返回  值：无
**************************************************************************/
void usartSendData(short leftVel, short rightVel,short angle,unsigned char ctrlFlag)
{
	// 协议数据缓存数组
	unsigned char buf[13] = {0};
	int i, length = 0;

	// 计算左右轮期望速度
	leftVelNow.d  = leftVel;
	rightVelNow.d = rightVel;
	angleNow.d    = angle;
	
	// 设置消息头
	for(i = 0; i < 2; i++)
		buf[i] = header[i];                      // buf[0] buf[1] 
	
	// 设置机器人左右轮速度、角度
	length = 7;
	buf[2] = length;                             // buf[2]
	for(i = 0; i < 2; i++)
	{
		buf[i + 3] = leftVelNow.data[i];         // buf[3] buf[4]
		buf[i + 5] = rightVelNow.data[i];        // buf[5] buf[6]
		buf[i + 7] = angleNow.data[i];           // buf[7] buf[8]
	}
	// 预留控制指令
	buf[3 + length - 1] = ctrlFlag;              // buf[9]
	
	// 设置校验值、消息尾
	buf[3 + length] = getCrc8(buf, 3 + length);  // buf[10]
	buf[3 + length + 1] = ender[0];              // buf[11]
	buf[3 + length + 2] = ender[1];              // buf[12]
	
	//发送字符串数据
	//HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&buf,sizeof(buf));
	USART_Send_String(buf,sizeof(buf));
}

void usartSendDouble(double leftVel, double rightVel,double angle,double accx, double accy, double accz, unsigned char ctrlFlag)
{
	// 协议数据缓存数组
	unsigned char buf[19] = {0};
	int i, length = 0;

	// 计算左右轮期望速度
	leftVelNow.d  = leftVel;
	rightVelNow.d = rightVel;
	angleNow.d    = angle;
	acc_x.d       = accx;
	acc_y.d       = accy;
	acc_z.d       = accz;
	// 设置消息头
//	for(i = 0; i < 2; i++)           
//		buf[i] = header[i];                      // buf[0] buf[1] 
	buf[0] = 0x55;
	buf[1] = 0xaa;
	
	// 设置机器人左右轮速度、角度
	//length = 7;
	length = 13;
	buf[2] = length;                             // buf[2]
	for(i = 0; i < 2; i++)
	{
		buf[i + 3] = leftVelNow.data[i];         // buf[3] buf[4]
		buf[i + 5] = rightVelNow.data[i];        // buf[5] buf[6]
		buf[i + 7] = angleNow.data[i];           // buf[7] buf[8]
		buf[i + 9] = acc_x.data[i];
		buf[i + 11] = acc_y.data[i];
		buf[i + 13] = acc_z.data[i];
		
	}
	// 预留控制指令
	buf[3 + length - 1] = ctrlFlag;              // buf[9]
	
	// 设置校验值、消息尾
	buf[3 + length] = getCrc8(buf, 3 + length);  // buf[10]
	buf[3 + length + 1] = ender[0];              // buf[11]
	buf[3 + length + 2] = ender[1];              // buf[12]
	
	//发送字符串数据
	//HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&buf,sizeof(buf));
	USART_Send_String(buf,sizeof(buf));
}



void usartSendprecison(float ggx, float ggy,float ggz,float aax, float aay, float aaz, unsigned char ctrlFlag)
{
		// 协议数据缓存数组
	uint8_t data_number = 6;
	uint8_t pre_data_size = 4;
//	uint8_t total_size = 3 +data_number*pre_data_size+ 1 + 3 ;  // = 31
	
//	union imu_send
//{
//	float d;
//	unsigned char data[4];
//}gx,gy,gz,ax,ay,az;
	unsigned char buf[31] = {0};
	int i, length = 0;

	// 计算左右轮期望速度
	gx.d  = ggx;
	gy.d = ggy;
	gz.d    = ggz;
	ax.d       = aax;
	ay.d       = aay;
	az.d       = aaz;
	// 设置消息头
//	for(i = 0; i < 2; i++)           
//		buf[i] = header[i];                      // buf[0] buf[1] 
	buf[0] = 0x55;
	buf[1] = 0xaa;
	
	//lenth is the length of real data
	length = data_number*pre_data_size+ 1;      //1byte is the ctrlFlag = 25
	buf[2] = length;                             // buf[2]
	for(i = 0; i < 4; i++)
	{
		buf[i + 3] = gx.data[i];         // buf[3] buf[4]
		buf[i + 7] = gy.data[i];        // buf[5] buf[6]
		buf[i + 11] = gz.data[i];           // buf[7] buf[8]
		buf[i + 15] = ax.data[i];
		buf[i + 19] = ay.data[i];
		buf[i + 23] = az.data[i];  //buf[23] - buf[26]
		
	}
	// 预留控制指令
	buf[3 + length - 1] = ctrlFlag;              // buf[27]
	
	// 设置校验值、消息尾
	buf[3 + length] = getCrc8(buf, 3 + length);  // buf[28]
	buf[3 + length + 1] = ender[0];              // buf[29]
	buf[3 + length + 2] = ender[1];              // buf[30]  --since start from buf[0] so total 31 bytes
	
	//发送字符串数据
	//HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&buf,sizeof(buf));
	USART_Send_String(buf,sizeof(buf));
}



/**************************************************************************
函数功能：发送指定大小的字符数组，被usartSendData函数调用
入口参数：数组地址、数组大小
返回  值：无
**************************************************************************/
void USART_Send_String(uint8_t *p,uint16_t sendSize)
{ 
	static int length =0;
	while(length<sendSize)
	{   
		//@@@@@#####如果你使用不是USART1更改成相应的，比如USART3，这里有两处修改
		while( !(USART1->SR&(0x01<<7)) );//发送缓冲区为空
		USART1->DR=*p;                   
		p++;
		length++;
	}
	length =0;
}
/**************************************************************************
函数功能：计算八位循环冗余校验，被usartSendData和usartReceiveOneData函数调用
入口参数：数组地址、数组大小
返回  值：无
**************************************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}
/**********************************END***************************************/







