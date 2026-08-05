#include "bsp_camera_usart.h"

volatile uint8_t camera_uart_rx_buffer[CAMERA_UART_RX_BUFFER_LEN] = {0};
volatile uint16_t camera_uart_rx_length = 0;
volatile uint8_t camera_uart_rx_flag = 0;

void Camera_Uart_ClearBuffer(void)
{
    camera_uart_rx_length = 0;
    camera_uart_rx_flag = 0;
    camera_uart_rx_buffer[0] = '\0';
}

void UART3_IRQHandler(void)
{
    uint8_t receivedData;

    switch (DL_UART_getPendingInterrupt(UART_3_INST))
    {
        case DL_UART_IIDX_RX:
            receivedData = DL_UART_Main_receiveData(UART_3_INST);
            if (camera_uart_rx_length < CAMERA_UART_RX_BUFFER_LEN - 1)
            {
                camera_uart_rx_buffer[camera_uart_rx_length++] = receivedData;
                camera_uart_rx_buffer[camera_uart_rx_length] = '\0';
                camera_uart_rx_flag = 1;
            }
            else
            {
                camera_uart_rx_length = 0;
                camera_uart_rx_flag = 0;
                camera_uart_rx_buffer[0] = '\0';
            }
            break;

        default:
            break;
    }
}
