#include "bsp_motor_usart.h"

/************************************************
函数名称 ： Send_Motor_U8		Function name: Send_Motor_U8
功    能 ： USART1发送一个字符	Function: USART1 sends a character
参    数 ： Data --- 数据		Parameter: Data --- data
返 回 值 ： 无					Return value: None
*************************************************/
void Send_Motor_U8(uint8_t Data)
{
	while( DL_UART_isBusy(UART_1_INST) == true );
	DL_UART_Main_transmitData(UART_1_INST, Data);
}

/************************************************
函数名称 ： Send_Motor_ArrayU8	Function name: Send_Motor_ArrayU8
功    能 ： 串口1发送N个字符		Function: Serial port 1 sends N characters
参    数 ： pData ---- 字符串	Parameter: pData ---- string
            Length --- 长度		Length --- length
返 回 值 ： 无					Return value: None
*************************************************/
void Send_Motor_ArrayU8(uint8_t *pData, uint16_t Length)
{
	while (Length--)
	{
		Send_Motor_U8(*pData);
		pData++;
	}
}


/*  串口中断接收处理 */
/* Serial port interrupt reception processing */
void UART_1_INST_IRQHandler(void)
{
	uint8_t Rx2_Temp = 0;
	
	switch( DL_UART_getPendingInterrupt(UART_1_INST) )
	{
		case DL_UART_IIDX_RX://如果是接收中断	If it is a receive interrupt
			
			// 接收发送过来的数据保存	Receive and save the data sent
			Rx2_Temp = DL_UART_Main_receiveData(UART_1_INST);
			//处理	deal with
			Deal_Control_Rxtemp(Rx2_Temp);
			break;
		
		default://其他的串口中断	Other serial port interrupts
			break;
	}	
	

}
