/*************************************************************************************/
#ifndef USER_VARS_H 
#define USER_VARS_H 

#include "stm32f10x.h"
#include <stdio.h>

/************************************************************************************/
// 常量及全局变量定义
/************************************************************************************/

// LED控制参数定义
#define  LED_ON			0			// LED打开			
#define  LED_OFF		1			// LED关闭
#define  LED_NEG		2			// LED取反

// USART发送接收数据缓存长度定义 
#define  USART1_RXBUF_SIZE		256		// USART1接收缓存长度
#define  USART1_TXBUF_SIZE		256		// USART1发送缓存长度

#define  USART2_RXBUF_SIZE		256		// USART2接收缓存长度
#define  USART2_TXBUF_SIZE		256		// USART2发送缓存长度

#define  USART3_RXBUF_SIZE		256		// USART3接收缓存长度
#define  USART3_TXBUF_SIZE		256		// USART3发送缓存长度

#define  UART4_RXBUF_SIZE		256		// UART4接收缓存长度
#define  UART4_TXBUF_SIZE		256		// UART4发送缓存长度

#define  UART5_RXBUF_SIZE		256		// UART5接收缓存长度
#define  UART5_TXBUF_SIZE		256		// UART5发送缓存长度

// USART发送接收数据缓存定义 
extern uint8_t  USART1_RxBuffer[]; 		// USART1接收数据缓存
extern uint8_t  USART1_TxBuffer[]; 		// USART1发送数据缓存

extern uint8_t  USART2_RxBuffer[]; 		// USART2接收数据缓存
extern uint8_t  USART2_TxBuffer[]; 		// USART2发送数据缓存
	
extern uint8_t  USART3_RxBuffer[]; 		// USART3接收数据缓存
extern uint8_t  USART3_TxBuffer[];	 	// USART3发送数据缓存

extern uint8_t  UART4_RxBuffer[]; 		// UART4接收数据缓存
extern uint8_t  UART4_TxBuffer[]; 		// UART4发送数据缓存

extern uint8_t  UART5_RxBuffer[];	 	// UART5接收数据缓存
extern uint8_t  UART5_TxBuffer[]; 		// UART5发送数据缓存

// USART发送接收数据长度变量 
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
// 文件结束
/***********************************************************************************/
