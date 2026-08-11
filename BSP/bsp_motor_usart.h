#ifndef __BSP_MOTOR_USART_H_
#define __BSP_MOTOR_USART_H_

#include "ti_msp_dl_config.h"
#include "app_motor_usart.h"

/* 初始化电机驱动串口，当前只有声明，尚未实现。 */
void Motor_Usart_init(void);
/* 向电机驱动串口发送一个字节。 */
void Send_Motor_U8(uint8_t data);
/* 向电机驱动串口发送 len 个字节。 */
void Send_Motor_ArrayU8(uint8_t *buf, uint16_t len);



#endif

