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
// ������ȫ�ֱ�������
/************************************************************************************/

// USART���ͽ������ݻ��涨�� 
uint8_t  USART1_RxBuffer[USART1_RXBUF_SIZE]; 	// USART1�������ݻ���
uint8_t  USART1_TxBuffer[USART1_TXBUF_SIZE]; 	// USART1�������ݻ���

uint8_t  USART2_RxBuffer[USART2_RXBUF_SIZE]; 	// USART2�������ݻ���
uint8_t  USART2_TxBuffer[USART2_TXBUF_SIZE]; 	// USART2�������ݻ���

uint8_t  USART3_RxBuffer[USART3_RXBUF_SIZE]; 	// USART3�������ݻ���
uint8_t  USART3_TxBuffer[USART3_TXBUF_SIZE]; 	// USART3�������ݻ���

uint8_t  UART4_RxBuffer[UART4_RXBUF_SIZE]; 		// UART4�������ݻ���
uint8_t  UART4_TxBuffer[UART4_TXBUF_SIZE]; 		// UART4�������ݻ���

uint8_t  UART5_RxBuffer[UART5_RXBUF_SIZE];	 	// UART5�������ݻ���
uint8_t  UART5_TxBuffer[UART5_TXBUF_SIZE]; 		// UART5�������ݻ���


// USART���ͽ������ݳ��ȱ��� 
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
* Description: ��ʱ����,��λ΢��;
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
* Description: ��ʱ����,��λ����;
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
* Description: NVIC��ʼ������;
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
	
	// ����USART1�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			     	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	// ����USART2�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			   
	NVIC_Init(&NVIC_InitStructure);
	// ����USART3�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;			     
	NVIC_Init(&NVIC_InitStructure);
	// ����UART4�ж�
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;		
	NVIC_Init(&NVIC_InitStructure);
	// ����UART5�ж�
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;			     
	NVIC_Init(&NVIC_InitStructure);
}

 /***********************************************************************************
* Function: GPIO_RemapConfig;
*
* Description: GPIO��ӳ������;
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
	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);			// USART1��ӳ��	
	GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);			// USART2��ӳ��	
	GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);		// USART3��ȫ��ӳ��	
}

 /***********************************************************************************
* Function: RCC_Configuration;
*
* Description: RCCʱ������;
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
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO,  ENABLE);  // ʹ��USART1��AFIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,  ENABLE);  	// ʹ��USART2ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,  ENABLE);  	// ʹ��USART3ʱ��  	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,  ENABLE);  	// ʹ��UART4ʱ��  			
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,  ENABLE);   	// ʹ��UART5ʱ�� 		
}
 /***********************************************************************************
* Function: GPIO_Configuration;
*
* Description: GPIO��ʼ������;
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

    GPIO_InitStructure.GPIO_Pin = IO_LED;				    // LED(RUN) ����Ϊͨ���������  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		// �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// ��ת�ٶ�Ϊ50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);					 

	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     	// �����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// ��ת�ٶ�50MHZ
	GPIO_InitStructure.GPIO_Pin = IO_TXD1;	         	    // USART1  TX
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = IO_TXD2|IO_TXD3;	   		// UART2��UART3  TX 
	GPIO_Init(GPIOD, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = IO_TXD4|IO_TXD5;	         // UART4��UART5  TX
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 	// ��������
	GPIO_InitStructure.GPIO_Pin = IO_RXD1;	           		// USART1 RX
	GPIO_Init(GPIOB, &GPIO_InitStructure);		

	GPIO_InitStructure.GPIO_Pin = IO_RXD2|IO_RXD3|IO_RXD5;	// UART2��UART3��UART5 RX
	GPIO_Init(GPIOD, &GPIO_InitStructure);	      	 		
	
	GPIO_InitStructure.GPIO_Pin = IO_RXD4;	           		// UART4 RX
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;     	// ��©���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// ��ת�ٶ�50MHZ
	GPIO_InitStructure.GPIO_Pin = IO_DIR4;	         	    // UART4ת485����IO
	GPIO_Init(GPIOE, &GPIO_InitStructure);	

	GPIO_ResetBits(GPIOD, IO_DIR4);							// �õͣ�485���óɽ���״̬	
}

 /***********************************************************************************
* Function: USART_Configuration;
*
* Description: USART��ʼ������;
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
	
	USART_InitStructure.USART_BaudRate = 115200;					//������115200
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//����λ8λ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;			//ֹͣλ1λ
    USART_InitStructure.USART_Parity = USART_Parity_No;				//��У��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //��Ӳ������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USARTx, &USART_InitStructure);						//���ô��ڲ�������
  
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);                  //ʹ�ܽ����ж�
    //USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);					//ʹ�ܷ��ͻ�����ж�
    //USART_ITConfig(USARTx, USART_IT_TC, ENABLE);					//ʹ�ܷ�������ж�
    USART_Cmd(USARTx, ENABLE);	 // ʹ��USARTx 	
 }

 /***********************************************************************************
* Function: Led_Ctrl;
*
* Description: Led�������;
* 
* Input:  Cmd,  Led��������: LED_ON, LED��;
*				LED_OFF, LED�ر�;
*				LED_NEG, LEDȡ��;	
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
			if (GPIO_ReadOutputDataBit(GPIOB, IO_LED) == Bit_SET)		// �ж����IO�Ƿ�Ϊ�ߵ�ƽ
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
* Description: USART���ݻ���д��;
* 
* Input:  USARTx, ����������ʶ: (USART1~USART3, UART4,UART5);
*		  pbuf, ��Ҫд������ݻ���ָ��;	
*		  data_length, ���ݳ��ȣ�
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
		GPIO_SetBits(GPIOE, IO_DIR4);							// �øߣ�485���óɷ���״̬
	}
	while(data_length--)
	{		
		USART_SendData(USARTx, *pbuf++);						// ��������
		while(!USART_GetFlagStatus(USARTx,USART_FLAG_TXE));	    // �ж������Ƿ������
	}
	if (USARTx == UART4)										// UART4
	{
		Delay_ms(1);
		GPIO_ResetBits(GPIOE, IO_DIR4);							// �õͣ�485���óɽ���״̬
	}
}

 /***********************************************************************************
* Function: USART_ReadBuffer;
*
* Description: USART���ݻ����ȡ;
* 
* Input:  USARTx, ����������ʶ: (USART1~USART3, UART4,UART5);
*		  pbuf, ��Ҫ��ȡ�����ݻ���ָ��;	
*		  data_length, ���ݳ��ȣ�
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
		memcpy(pbuf, USART1_RxBuffer, data_length);			// ��ȡ���ջ����е�����
		USART1_RxLen = 0;									// ������ջ��泤��
	}
	else if (USARTx==USART2)
	{
		memcpy(pbuf, USART2_RxBuffer, data_length);			// ��ȡ���ջ����е�����
		USART2_RxLen = 0;									// ������ջ��泤��
	}
	else if (USARTx==USART3)
	{
		memcpy(pbuf, USART3_RxBuffer, data_length);			// ��ȡ���ջ����е�����
		USART3_RxLen = 0;									// ������ջ��泤��
	}
	else if (USARTx==UART4)
	{
		memcpy(pbuf, UART4_RxBuffer, data_length);			// ��ȡ���ջ����е�����
		UART4_RxLen = 0;									// ������ջ��泤��
	}
	else if (USARTx==UART5)
	{
		memcpy(pbuf, UART5_RxBuffer, data_length);			// ��ȡ���ջ����е�����
		UART5_RxLen = 0;									// ������ջ��泤��
	}
}

 /***********************************************************************************
* Function: USART_GetRxBufLen;
*
* Description: ��ȡUSART���ݻ��浱ǰ����;
* 
* Input:  USARTx, ����������ʶ: (USART1~USART3, UART4,UART5);
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
		rBufferLen = USART1_RxLen;							// ��ȡ���ջ��泤��
	}
	else if (USARTx==USART2)
	{
		rBufferLen = USART2_RxLen;							// ��ȡ���ջ��泤��
	}
	else if (USARTx==USART3)
	{
		rBufferLen = USART3_RxLen;							// ��ȡ���ջ��泤��
	}
	else if (USARTx==UART4)
	{
		rBufferLen = UART4_RxLen;							// ��ȡ���ջ��泤��
	}
	else if (USARTx==UART5)
	{
		rBufferLen = UART5_RxLen;							// ��ȡ���ջ��泤��
	}
	return rBufferLen;
}
/***********************************************************************************
* Function: USART_Test;
*
* Description: USART����;
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
	static uint16_t  rUsart1Len,rUsart2Len,rUsart3Len,rUart4Len,rUart5Len;  // USART1-USART3,UART4, UART5�������ݳ���
	uint8_t   buf[USART_BUFFER_LEN];
	uint16_t  len;
	
	// USART1
	len = USART_GetRxBufLen(USART1);					// ��ȡ�������ݳ���
	if ((len == rUsart1Len)&&(len>0))                   // ���ζ�ȡ�������ݳ�����ȵ�����0: ��ʾ���յ�һ֡����������
	{
		if (len>USART_BUFFER_LEN)                       // �������泤�ȴ���
		{
			len = USART_BUFFER_LEN;
		}
		USART_ReadBuffer(USART1, buf, len);          	// ��ȡ���ݵ�����
		USART_WriteBuffer(USART1, buf, len);          	// ԭ�����ض���������
		rUsart1Len -= len;                              // ���㻺��ʣ�����ݳ���
	}
	else
	{
	  rUsart1Len = len;
	}
	
	// USART2
	len = USART_GetRxBufLen(USART2);					// ��ȡ�������ݳ���
	if ((len == rUsart2Len)&&(len>0))                   // ���ζ�ȡ�������ݳ�����ȵ�����0: ��ʾ���յ�һ֡����������
	{
		if (len>USART_BUFFER_LEN)                       // �������泤�ȴ���
		{
			len = USART_BUFFER_LEN;
		}
		USART_ReadBuffer(USART2, buf, len);         	// ��ȡ���ݵ�����
		USART_WriteBuffer(USART2, buf, len);        	// ԭ�����ض���������
		rUsart2Len -= len;                              // ���㻺��ʣ�����ݳ���
	}
	else
	{
	  rUsart2Len = len;
	}
	
	// USART3
	len = USART_GetRxBufLen(USART3);					// ��ȡ�������ݳ���
	if ((len == rUsart3Len)&&(len>0))                   // ���ζ�ȡ�������ݳ�����ȵ�����0: ��ʾ���յ�һ֡����������
	{
		if (len>USART_BUFFER_LEN)                       // �������泤�ȴ���
		{
			len = USART_BUFFER_LEN;
		}
		USART_ReadBuffer(USART3, buf, len);          	// ��ȡ���ݵ�����
		USART_WriteBuffer(USART3, buf, len);          	// ԭ�����ض���������
		rUsart3Len -= len;                              // ���㻺��ʣ�����ݳ���
	}
	else
	{
	  rUsart3Len = len;
	}
	
	// UART4
	len = USART_GetRxBufLen(UART4);						// ��ȡ�������ݳ���
	if ((len == rUart4Len)&&(len>0))                    // ���ζ�ȡ�������ݳ�����ȵ�����0: ��ʾ���յ�һ֡����������
	{
		if (len>USART_BUFFER_LEN)                       // �������泤�ȴ���
		{
			len = USART_BUFFER_LEN;
		}
		USART_ReadBuffer(UART4, buf, len);           	// ��ȡ���ݵ�����
		USART_WriteBuffer(UART4, buf, len);         	// ԭ�����ض���������
		rUart4Len -= len;                               // ���㻺��ʣ�����ݳ���
	}
	else
	{
	  rUart4Len	= len;
	}
	
	// UART5
	len = USART_GetRxBufLen(UART5);						// ��ȡ�������ݳ���
	if ((len == rUart5Len)&&(len>0))                    // ���ζ�ȡ�������ݳ�����ȵ�����0: ��ʾ���յ�һ֡����������
	{
		if (len>USART_BUFFER_LEN)                       // �������泤�ȴ���
		{
			len = USART_BUFFER_LEN;
		}
		USART_ReadBuffer(UART5, buf, len);           	// ��ȡ���ݵ�����
		USART_WriteBuffer(UART5, buf, len);         	// ԭ�����ض���������
		rUart5Len -= len;                               // ���㻺��ʣ�����ݳ���
	}
	else
	{
	  rUart5Len	= len;
	}
	
}

int main(void)
{	
	uint32_t cnt;
	RCC_Configuration();						// RCC����	
	GPIO_Configuration();						// GPIO����	
	GPIO_RemapConfig();							// GPIO��ӳ��
	NVIC_Configuration();						// NVIC����

	USART_Configuration(USART1);				// USART1��ʼ��
	USART_Configuration(USART2);				// USART2��ʼ��
	USART_Configuration(USART3);				// USART3��ʼ��
	USART_Configuration(UART4);					// UART4��ʼ��
	USART_Configuration(UART5);					// UART5��ʼ��
	
	cnt = 0;
	while (1)
	{		
		Delay_ms(20);							// ��ʱ20ms	
		USART_Test(); 							// USART����
		cnt++;
		
		if ((cnt%50)==0)						
		{
			Led_Ctrl(LED_NEG);					// ��תLED��
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
