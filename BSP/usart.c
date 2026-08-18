#include "usart.h"
#include "stdio.h"

#define UART0_RX_MAX	(128U) /* UART0 调试接收缓冲区字节数。 */

static volatile uint8_t s_rx[UART0_RX_MAX] = {0}; /* UART0 接收缓冲区。 */
static volatile uint16_t s_rx_len = 0U;           /* 当前已接收的字节数。 */
static volatile uint8_t s_rx_ready = 0U;          /* UART0 收到数据后的标志。 */

void USART_Init(void)
{
	/* 系统外设已由 main 中的 SYSCFG_DL_init() 统一初始化。 */
	/* 清除串口中断标志。 */
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_3_INST_INT_IRQN);
	/* 使能串口中断。 */
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
}

//串口发送一个字节
//The serial port sends a byte
void USART_SendData(unsigned char data)
{
	//当串口0忙的时候等待
	//Wait when serial port 0 is busy
	while( DL_UART_isBusy(UART_0_INST) == true );
	//发送
	//send
	DL_UART_Main_transmitData(UART_0_INST, data);
}


#if !defined(__MICROLIB)
//不使用微库的话就需要添加下面的函数
//If you don't use the micro library, you need to add the following function
#if (__ARMCLIB_VERSION <= 6000000)
//如果编译器是AC5  就定义下面这个结构体
//If the compiler is AC5, define the following structure
struct __FILE
{
	int handle; /* C 库文件句柄占位字段，UART 重定向不使用其值。 */
};
#endif

FILE __stdout;

//定义_sys_exit()以避免使用半主机模式
//Define _sys_exit() to avoid using semihosting mode
void _sys_exit(int status)
{
	status = status;
}
#endif


//printf函数重定义
//printf function redefinition
int fputc(int ch, FILE *stream)
{
	//当串口0忙的时候等待，不忙的时候再发送传进来的字符
	//Wait when serial port 0 is busy, and send the incoming characters when it is not busy
	while( DL_UART_isBusy(UART_0_INST) == true );
	
	DL_UART_Main_transmitData(UART_0_INST, ch);
	
	return ch;
}

//串口的中断服务函数
//Serial port interrupt service function
void UART_0_INST_IRQHandler(void)
{
	uint8_t rx = 0U; /* UART0 硬件刚接收的字节。 */
	
	//如果产生了串口中断
	//If a serial port interrupt occurs
	switch( DL_UART_getPendingInterrupt(UART_0_INST) )
	{
		case DL_UART_IIDX_RX://如果是接收中断	If it is a receive interrupt
			
			// 接收发送过来的数据保存	Receive and save the data sent
			rx = DL_UART_Main_receiveData(UART_0_INST);
		
			// 检查缓冲区是否已满	Check if the buffer is full
			if (s_rx_len < UART0_RX_MAX - 1U)
			{
				s_rx[s_rx_len++] = rx;
			}
			else
			{
				s_rx_len = 0U;
			}

			// 标记接收标志	Mark receiving flag
			s_rx_ready = 1U;
		
			break;
		
		default://其他的串口中断	Other serial port interrupts
			break;
	}
}
