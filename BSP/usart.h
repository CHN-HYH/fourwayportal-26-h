#ifndef	__USART_H__
#define __USART_H__

#include "ti_msp_dl_config.h"


/* 初始化 UART0、UART1、UART3 的中断。 */
void USART_Init(void);
/* 通过 UART0 发送一个字节。 */
void USART_SendData(unsigned char data);

#endif
