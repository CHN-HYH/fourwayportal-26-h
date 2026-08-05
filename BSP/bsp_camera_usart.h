#ifndef __BSP_CAMERA_USART_H_
#define __BSP_CAMERA_USART_H_

#include "ti_msp_dl_config.h"

#define CAMERA_UART_RX_BUFFER_LEN    (256)

// 摄像头串口接收缓冲区。camera_uart_rx_length 是当前有效字节数。
extern volatile uint8_t camera_uart_rx_buffer[CAMERA_UART_RX_BUFFER_LEN];
extern volatile uint16_t camera_uart_rx_length;
extern volatile uint8_t camera_uart_rx_flag;

void Camera_Uart_ClearBuffer(void);

#endif
