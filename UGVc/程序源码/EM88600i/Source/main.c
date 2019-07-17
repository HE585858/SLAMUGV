/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "EMB8600I_IOConfig.h"
#include "user_vars.h"
#include <stdio.h>
#include <string.h>

/************************************************************************************/
// 常量及全局变量定义
/************************************************************************************/

// USART发送接收数据缓存定义 
uint8_t  USART1_RxBuffer[USART1_RXBUF_SIZE]; 	// USART1接收数据缓存
uint8_t  USART1_TxBuffer[USART1_TXBUF_SIZE]; 	// USART1发送数据缓存

uint8_t  USART2_RxBuffer[USART2_RXBUF_SIZE]; 	// USART2接收数据缓存
uint8_t  USART2_TxBuffer[USART2_TXBUF_SIZE]; 	// USART2发送数据缓存

uint8_t  USART3_RxBuffer[USART3_RXBUF_SIZE]; 	// USART3接收数据缓存
uint8_t  USART3_TxBuffer[USART3_TXBUF_SIZE]; 	// USART3发送数据缓存

uint8_t  UART4_RxBuffer[UART4_RXBUF_SIZE]; 		// UART4接收数据缓存
uint8_t  UART4_TxBuffer[UART4_TXBUF_SIZE]; 		// UART4发送数据缓存

uint8_t  UART5_RxBuffer[UART5_RXBUF_SIZE];	 	// UART5接收数据缓存
uint8_t  UART5_TxBuffer[UART5_TXBUF_SIZE]; 		// UART5发送数据缓存


// USART发送接收数据长度变量 
__IO uint16_t USART1_RxLen;
__IO uint16_t USART1_TxLen;

__IO uint16_t USART2_RxLen;
__IO uint16_t USART2_TxLen;

__IO uint16_t USART3_RxLen;
__IO uint16_t USART3_TxLen;

__IO uint16_t UART4_RxLen;
__IO uint16_t UART4_TxLen;

__IO uint16_t UART5_RxLen;
__IO uint16_t UART5_TxLen;

/************************************************************************************/

 /***********************************************************************************
* Function: Delay_us;
*
* Description: 延时函数,单位微秒;
* 
* Input:  nCount: 
*
* Output: none;
*          		
* Return: none;
*
* Note:   none;
************************************************************************************/
void Delay_us(__IO uint32_t nCount)
{
    uint32_t Time;
	Time = nCount*1000/126;
	while(--Time);
}

 /***********************************************************************************
* Function: Delay_ms;
*
* Description: 延时函数,单位毫秒;
* 
* Input:  nCount: 
*
* Output: none;
*          		
* Return: none;
*
* Note:   none;
************************************************************************************/
void Delay_ms(__IO uint32_t nCount)
{
	uint32_t Time;
	Time = nCount*1000000/126;
	while(--Time);
}


 /***********************************************************************************
* Function: NVIC_Configuration;
*
* Description: NVIC初始化配置;
* 
* Input:  none;
*
* Output: none;
*          		
* Return: none;
*
* Note:   none;
************************************************************************************/
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		   
	
	// 设置USART1中断
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			     	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	// 设置USART2中断
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			   
	NVIC_Init(&NVIC_InitStructure);
	// 设置USART3中断
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;			     
	NVIC_Init(&NVIC_InitStructure);
	// 设置UART4中断
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;		
	NVIC_Init(&NVIC_InitStructure);
	// 设置UART5中断
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;			     
	NVIC_Init(&NVIC_InitStructure);
}

 /***********************************************************************************
* Function: GPIO_RemapConfig;
*
* Description: GPIO重映射配置;
* 
* Input:  none;
*
* Output: none;
*          		
* Return: none;
*
* Note:   none;
************************************************************************************/
void GPIO_RemapConfig(void)
{
	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);			// USART1重映射	
	GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);			// USART2重映射	
	GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);		// USART3完全重映射	
}

 /***********************************************************************************
* Function: RCC_Configuration;
*
* Description: RCC时钟配置;
* 
* Input:  none;
*
* Output: none;
*          		
* Return: none;
*
* Note:   none;
************************************************************************************/
void RCC_Configuration(void)
{   
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO,  ENABLE);  // 使能USART1，AFIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,  ENABLE);  	// 使能USART2时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,  ENABLE);  	// 使能USART3时钟  	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,  ENABLE);  	// 使能UART4时钟  			
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,  ENABLE);   	// 使能UART5时钟 		
}
 /***********************************************************************************
* Function: GPIO_Configuration;
*
* Description: GPIO初始化配置;
* 
* Input:  none;
*
* Output: none;
*          		
* Return: none;
*
* Note:   none;
************************************************************************************/
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE);	

    GPIO_InitStructure.GPIO_Pin = IO_LED;				    // LED(RUN) 配置为通用推挽输出  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		// 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// 翻转速度为50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);					 

	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     	// 复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// 翻转速度50MHZ
	GPIO_InitStructure.GPIO_Pin = IO_TXD1;	         	    // USART1  TX
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = IO_TXD2|IO_TXD3;	   		// UART2，UART3  TX 
	GPIO_Init(GPIOD, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = IO_TXD4|IO_TXD5;	         // UART4，UART5  TX
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 	// 浮空输入
	GPIO_InitStructure.GPIO_Pin = IO_RXD1;	           		// USART1 RX
	GPIO_Init(GPIOB, &GPIO_InitStructure);		

	GPIO_InitStructure.GPIO_Pin = IO_RXD2|IO_RXD3|IO_RXD5;	// UART2，UART3，UART5 RX
	GPIO_Init(GPIOD, &GPIO_InitStructure);	      	 		
	
	GPIO_InitStructure.GPIO_Pin = IO_RXD4;	           		// UART4 RX
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;     	// 开漏输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// 翻转速度50MHZ
	GPIO_InitStructure.GPIO_Pin = IO_DIR4;	         	    // UART4转485方向IO
	GPIO_Init(GPIOE, &GPIO_InitStructure);	

	GPIO_ResetBits(GPIOD, IO_DIR4);							// 置低，485设置成接收状态	
}

 /***********************************************************************************
* Function: USART_Configuration;
*
* Description: USART初始化配置;
* 
* Input:  USARTx: USART1-USAET3,UART4,UART5;
*
* Output: none;
*          		
* Return: none;
*
* Note:   none;
************************************************************************************/
void USART_Configuration(USART_TypeDef* USARTx)
{
    USART_InitTypeDef USART_InitStructure;
	
	USART_InitStructure.USART_BaudRate = 115200;					//波特率115200
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//数据位8位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位1位
    USART_InitStructure.USART_Parity = USART_Parity_No;				//无校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USARTx, &USART_InitStructure);						//配置串口参数函数
  
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);                  //使能接收中断
    //USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);					//使能发送缓冲空中断
    //USART_ITConfig(USARTx, USART_IT_TC, ENABLE);					//使能发送完成中断
    USART_Cmd(USARTx, ENABLE);	 // 使能USARTx 	
 }

 /***********************************************************************************
* Function: Led_Ctrl;
*
* Description: Led命令控制;
* 
* Input:  Cmd,  Led控制命令: LED_ON, LED打开;
*				LED_OFF, LED关闭;
*				LED_NEG, LED取反;	
*
* Output: none;
*          		
* Return: none;
*
* Note:   none;
************************************************************************************/
void Led_Ctrl(uint8_t Cmd)
{
	switch (Cmd)
	{
		case LED_ON:												
			GPIO_SetBits(GPIOB, IO_LED);		 						
			break;		
		case LED_OFF:
			GPIO_ResetBits(GPIOB, IO_LED);		 	 
			break;	
		case LED_NEG:
			if (GPIO_ReadOutputDataBit(GPIOB, IO_LED) == Bit_SET)		// 判断输出IO是否为高电平
			{
				GPIO_ResetBits(GPIOB, IO_LED);		 	
			}
			else
			{
				GPIO_SetBits(GPIOB, IO_LED);		 	  
			}
			break;
		default:
			break;
	}
}
 /***********************************************************************************
* Function: USART_WriteBuffer;
*
* Description: USART数据缓存写入;
* 
* Input:  USARTx, 串口索引标识: (USART1~USART3, UART4,UART5);
*		  pbuf, 需要写入的数据缓存指针;	
*		  data_length, 数据长度；
*
* Output: none;
*          		
* Return: none;
*
* Note:   none;
************************************************************************************/
void USART_WriteBuffer(USART_TypeDef* USARTx, uint8_t *pbuf, uint16_t data_length)
{
	
	if (USARTx == UART4)										// UART4
	{
		GPIO_SetBits(GPIOE, IO_DIR4);							// 置高，485设置成发送状态
	}
	while(data_length--)
	{		
		USART_SendData(USARTx, *pbuf++);						// 发送数据
		while(!USART_GetFlagStatus(USARTx,USART_FLAG_TXE));	    // 判断数据是否发送完成
	}
	if (USARTx == UART4)										// UART4
	{
		Delay_ms(1);
		GPIO_ResetBits(GPIOE, IO_DIR4);							// 置低，485设置成接收状态
	}
}

 /***********************************************************************************
* Function: USART_ReadBuffer;
*
* Description: USART数据缓存读取;
* 
* Input:  USARTx, 串口索引标识: (USART1~USART3, UART4,UART5);
*		  pbuf, 需要读取的数据缓存指针;	
*		  data_length, 数据长度；
*
* Output: none;
*          		
* Return: none;
*
* Note:   none;
************************************************************************************/
void USART_ReadBuffer(USART_TypeDef* USARTx, uint8_t *pbuf, uint16_t data_length)
{
	if (USARTx==USART1)
	{
		memcpy(pbuf, USART1_RxBuffer, data_length);			// 获取接收缓存中的数据
		USART1_RxLen = 0;									// 清除接收缓存长度
	}
	else if (USARTx==USART2)
	{
		memcpy(pbuf, USART2_RxBuffer, data_length);			// 获取接收缓存中的数据
		USART2_RxLen = 0;									// 清除接收缓存长度
	}
	else if (USARTx==USART3)
	{
		memcpy(pbuf, USART3_RxBuffer, data_length);			// 获取接收缓存中的数据
		USART3_RxLen = 0;									// 清除接收缓存长度
	}
	else if (USARTx==UART4)
	{
		memcpy(pbuf, UART4_RxBuffer, data_length);			// 获取接收缓存中的数据
		UART4_RxLen = 0;									// 清除接收缓存长度
	}
	else if (USARTx==UART5)
	{
		memcpy(pbuf, UART5_RxBuffer, data_length);			// 获取接收缓存中的数据
		UART5_RxLen = 0;									// 清除接收缓存长度
	}
}

 /***********************************************************************************
* Function: USART_GetRxBufLen;
*
* Description: 读取USART数据缓存当前长度;
* 
* Input:  USARTx, 串口索引标识: (USART1~USART3, UART4,UART5);
*
* Output: none;
*          		
* Return: none;
*
* Note:   none;
************************************************************************************/
uint16_t USART_GetRxBufLen(USART_TypeDef* USARTx)
{
	uint16_t rBufferLen;
	
	if (USARTx==USART1)
	{
		rBufferLen = USART1_RxLen;							// 获取接收缓存长度
	}
	else if (USARTx==USART2)
	{
		rBufferLen = USART2_RxLen;							// 获取接收缓存长度
	}
	else if (USARTx==USART3)
	{
		rBufferLen = USART3_RxLen;							// 获取接收缓存长度
	}
	else if (USARTx==UART4)
	{
		rBufferLen = UART4_RxLen;							// 获取接收缓存长度
	}
	else if (USARTx==UART5)
	{
		rBufferLen = UART5_RxLen;							// 获取接收缓存长度
	}
	return rBufferLen;
}
/***********************************************************************************
* Function: USART_Test;
*
* Description: USART测试;
* 
* Input:  none;
*
* Output: none;
*          		
* Return: none;
*
* Note:   none;
************************************************************************************/
void USART_Test(void)
{
	#define  USART_BUFFER_LEN	64
	static uint16_t  rUsart1Len,rUsart2Len,rUsart3Len,rUart4Len,rUart5Len;  // USART1-USART3,UART4, UART5接收数据长度
	uint8_t   buf[USART_BUFFER_LEN];
	uint16_t  len;
	
	// USART1
	len = USART_GetRxBufLen(USART1);					// 读取接收数据长度
	if ((len == rUsart1Len)&&(len>0))                   // 两次读取缓存数据长度相等但大于0: 表示接收到一帧完整的数据
	{
		if (len>USART_BUFFER_LEN)                       // 超出缓存长度处理
		{
			len = USART_BUFFER_LEN;
		}
		USART_ReadBuffer(USART1, buf, len);          	// 读取数据到缓存
		USART_WriteBuffer(USART1, buf, len);          	// 原样返回读到的数据
		rUsart1Len -= len;                              // 计算缓存剩余数据长度
	}
	else
	{
	  rUsart1Len = len;
	}
	
	// USART2
	len = USART_GetRxBufLen(USART2);					// 读取接收数据长度
	if ((len == rUsart2Len)&&(len>0))                   // 两次读取缓存数据长度相等但大于0: 表示接收到一帧完整的数据
	{
		if (len>USART_BUFFER_LEN)                       // 超出缓存长度处理
		{
			len = USART_BUFFER_LEN;
		}
		USART_ReadBuffer(USART2, buf, len);         	// 读取数据到缓存
		USART_WriteBuffer(USART2, buf, len);        	// 原样返回读到的数据
		rUsart2Len -= len;                              // 计算缓存剩余数据长度
	}
	else
	{
	  rUsart2Len = len;
	}
	
	// USART3
	len = USART_GetRxBufLen(USART3);					// 读取接收数据长度
	if ((len == rUsart3Len)&&(len>0))                   // 两次读取缓存数据长度相等但大于0: 表示接收到一帧完整的数据
	{
		if (len>USART_BUFFER_LEN)                       // 超出缓存长度处理
		{
			len = USART_BUFFER_LEN;
		}
		USART_ReadBuffer(USART3, buf, len);          	// 读取数据到缓存
		USART_WriteBuffer(USART3, buf, len);          	// 原样返回读到的数据
		rUsart3Len -= len;                              // 计算缓存剩余数据长度
	}
	else
	{
	  rUsart3Len = len;
	}
	
	// UART4
	len = USART_GetRxBufLen(UART4);						// 读取接收数据长度
	if ((len == rUart4Len)&&(len>0))                    // 两次读取缓存数据长度相等但大于0: 表示接收到一帧完整的数据
	{
		if (len>USART_BUFFER_LEN)                       // 超出缓存长度处理
		{
			len = USART_BUFFER_LEN;
		}
		USART_ReadBuffer(UART4, buf, len);           	// 读取数据到缓存
		USART_WriteBuffer(UART4, buf, len);         	// 原样返回读到的数据
		rUart4Len -= len;                               // 计算缓存剩余数据长度
	}
	else
	{
	  rUart4Len	= len;
	}
	
	// UART5
	len = USART_GetRxBufLen(UART5);						// 读取接收数据长度
	if ((len == rUart5Len)&&(len>0))                    // 两次读取缓存数据长度相等但大于0: 表示接收到一帧完整的数据
	{
		if (len>USART_BUFFER_LEN)                       // 超出缓存长度处理
		{
			len = USART_BUFFER_LEN;
		}
		USART_ReadBuffer(UART5, buf, len);           	// 读取数据到缓存
		USART_WriteBuffer(UART5, buf, len);         	// 原样返回读到的数据
		rUart5Len -= len;                               // 计算缓存剩余数据长度
	}
	else
	{
	  rUart5Len	= len;
	}
	
}

int main(void)
{	
	uint32_t cnt;
	RCC_Configuration();						// RCC配置	
	GPIO_Configuration();						// GPIO配置	
	GPIO_RemapConfig();							// GPIO重映射
	NVIC_Configuration();						// NVIC配置

	USART_Configuration(USART1);				// USART1初始化
	USART_Configuration(USART2);				// USART2初始化
	USART_Configuration(USART3);				// USART3初始化
	USART_Configuration(UART4);					// UART4初始化
	USART_Configuration(UART5);					// UART5初始化
	
	cnt = 0;
	while (1)
	{		
		Delay_ms(20);							// 延时20ms	
		USART_Test(); 							// USART测试
		cnt++;
		
		if ((cnt%50)==0)						
		{
			Led_Ctrl(LED_NEG);					// 翻转LED灯
		}
	}
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
