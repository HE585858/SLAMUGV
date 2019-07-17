/*************************************************************************************/
#ifndef EMB8600I_IOCONFIG_H 
#define EMB8600I_IOCONFIG_H 

/************************************************************************************/
//  EMB8600I 硬件版本
/************************************************************************************/
#define   HW_VERSION	108

/************************************************************************************/
//  EMB8600I IO输入输出定义
/************************************************************************************/
// LED IO端口定义
#define IO_LED      GPIO_Pin_2   // PB2, 运行LED指示灯 

// ALARM IO端口定义
#define IO_ALARM    GPIO_Pin_2   // PE2, 蜂鸣器控制IO

// ADC IO端口定义(JP14, JP15)
// JP14
#define IO_ADC1     GPIO_Pin_0   // PA0
#define IO_ADC2     GPIO_Pin_3   // PA3
#define IO_ADC3     GPIO_Pin_6	 // PA6
#define IO_ADC4     GPIO_Pin_0 	 // PB0
// JP15
#define IO_ADC5     GPIO_Pin_1 	 // PB1
#define IO_ADC6     GPIO_Pin_0 	 // PC0
#define IO_ADC7     GPIO_Pin_2	 // PC2
#define IO_ADC8     GPIO_Pin_3	 // PC3

// DAC输出IO端口定义(JP13)
#define IO_DAC1     GPIO_Pin_4   // PA4
#define IO_DAC2     GPIO_Pin_5   // PA5	  

// UART1~UART5 IO定义(JP8,JP9)
// JP8: RS232
#define IO_TXD1     GPIO_Pin_6	 // PB6
#define IO_RXD1     GPIO_Pin_7	 // PB7
#define IO_TXD2     GPIO_Pin_5	 // PD5
#define IO_RXD2     GPIO_Pin_6	 // PD6
#define IO_TXD3     GPIO_Pin_8	 // PD8
#define IO_RXD3     GPIO_Pin_9	 // PD9
#define IO_TXD5     GPIO_Pin_12	 // PC12
#define IO_RXD5     GPIO_Pin_2	 // PD2
// JP9  RS485
#define IO_TXD4     GPIO_Pin_10	 // PC10
#define IO_RXD4     GPIO_Pin_11	 // PC11

// UART4转成RS485 方向控制IO定义
#define IO_DIR4     GPIO_Pin_3	 // PE3, UART4转成RS485方向控制IO

// CAN1占用IO定义(JP11)
#define IO_CAN1RX   GPIO_Pin_0   // PD0 
#define IO_CAN1TX   GPIO_Pin_1	 // PD1

// EEPROM IO定义					   
#define IO_SCL 		GPIO_Pin_8	 // PB8
#define IO_SDA  	GPIO_Pin_9	 // PB9

// SD卡(JP5)
#define IO_SDCS     GPIO_Pin_5	 // PE14
#define IO_SDINR    GPIO_Pin_6	 // PE6
#define IO_SDPWR    GPIO_Pin_7	 // PE7
#define IO_SDWP     GPIO_Pin_13	 // PC13

// 脉冲FCLK输入(JP6)
// JP10  FCLK1
#define IO_FCLK1CH1 GPIO_Pin_12	 // PD12, FCLK1
#define IO_FCLK1CH2 GPIO_Pin_13	 // PD13, FCLK2

// PWM输出(JP7)
#define IO_PWM1CH1  GPIO_Pin_6	 // PC6,  PWM1
#define IO_PWM1CH2  GPIO_Pin_7	 // PC7,  PWM2

// SPI IO配置
#define IO_SCLK     GPIO_Pin_3   // PB3	
#define IO_MISO     GPIO_Pin_4   // PB4
#define IO_MOSI     GPIO_Pin_5   // PB5

// SPI FLASH片选
#define IO_SPIFLASH_CS GPIO_Pin_15 // PA15

// 输入端口(JP4)
#define IO_DIN1     GPIO_Pin_14	 // PD14
#define IO_DIN2     GPIO_Pin_15	 // PD15
#define IO_DIN3     GPIO_Pin_14	 // PB14
#define IO_DIN4     GPIO_Pin_15	 // PB15
#define IO_DIN5     GPIO_Pin_7	 // PD7
#define IO_DIN6     GPIO_Pin_4	 // PE4
#define IO_DIN7     GPIO_Pin_8	 // PC8
#define IO_DIN8     GPIO_Pin_9	 // PC9

// 输出端口(JP3)
#define IO_DOUT1    GPIO_Pin_8	 // PE8
#define IO_DOUT2    GPIO_Pin_9	 // PE9
#define IO_DOUT3    GPIO_Pin_10	 // PE10
#define IO_DOUT4    GPIO_Pin_11	 // PE11
#define IO_DOUT5    GPIO_Pin_12	 // PE12
#define IO_DOUT6    GPIO_Pin_13	 // PE13
#define IO_DOUT7    GPIO_Pin_14	 // PE14
#define IO_DOUT8    GPIO_Pin_15	 // PE15

// 拨码开关
#define IO_SW1  	GPIO_Pin_0	 // PE0 
#define IO_SW2 	    GPIO_Pin_1	 // PE1

// USB HOST 电源控制IO
#define IO_USBPWR   GPIO_Pin_4	 // PD4

#endif

/***********************************************************************************/
// 文件结束
/***********************************************************************************/
