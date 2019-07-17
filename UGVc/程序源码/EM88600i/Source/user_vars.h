/*************************************************************************************/
#ifndef USER_VARS_H 
#define USER_VARS_H 

#include "stm32f10x.h"
#include <stdio.h>

/************************************************************************************/
// ������ȫ�ֱ�������
/************************************************************************************/

// LED���Ʋ�������
#define  LED_ON			0			// LED��			
#define  LED_OFF		1			// LED�ر�
#define  LED_NEG		2			// LEDȡ��

// USART���ͽ������ݻ��泤�ȶ��� 
#define  USART1_RXBUF_SIZE		256		// USART1���ջ��泤��
#define  USART1_TXBUF_SIZE		256		// USART1���ͻ��泤��

#define  USART2_RXBUF_SIZE		256		// USART2���ջ��泤��
#define  USART2_TXBUF_SIZE		256		// USART2���ͻ��泤��

#define  USART3_RXBUF_SIZE		256		// USART3���ջ��泤��
#define  USART3_TXBUF_SIZE		256		// USART3���ͻ��泤��

#define  UART4_RXBUF_SIZE		256		// UART4���ջ��泤��
#define  UART4_TXBUF_SIZE		256		// UART4���ͻ��泤��

#define  UART5_RXBUF_SIZE		256		// UART5���ջ��泤��
#define  UART5_TXBUF_SIZE		256		// UART5���ͻ��泤��

// USART���ͽ������ݻ��涨�� 
extern uint8_t  USART1_RxBuffer[]; 		// USART1�������ݻ���
extern uint8_t  USART1_TxBuffer[]; 		// USART1�������ݻ���

extern uint8_t  USART2_RxBuffer[]; 		// USART2�������ݻ���
extern uint8_t  USART2_TxBuffer[]; 		// USART2�������ݻ���
	
extern uint8_t  USART3_RxBuffer[]; 		// USART3�������ݻ���
extern uint8_t  USART3_TxBuffer[];	 	// USART3�������ݻ���

extern uint8_t  UART4_RxBuffer[]; 		// UART4�������ݻ���
extern uint8_t  UART4_TxBuffer[]; 		// UART4�������ݻ���

extern uint8_t  UART5_RxBuffer[];	 	// UART5�������ݻ���
extern uint8_t  UART5_TxBuffer[]; 		// UART5�������ݻ���

// USART���ͽ������ݳ��ȱ��� 
extern __IO uint16_t USART1_RxLen;
extern __IO uint16_t USART1_TxLen;

extern __IO uint16_t USART2_RxLen;
extern __IO uint16_t USART2_TxLen;

extern __IO uint16_t USART3_RxLen;
extern __IO uint16_t USART3_TxLen;

extern __IO uint16_t UART4_RxLen;
extern __IO uint16_t UART4_TxLen;

extern __IO uint16_t UART5_RxLen;
extern __IO uint16_t UART5_TxLen;

#endif
/***********************************************************************************/
// �ļ�����
/***********************************************************************************/
