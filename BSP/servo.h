#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdint.h>

/* 初始化 TIMA0 CCP0 的 PB8 舵机输出，并回到中位。 */
void Servo_Init(void);
/* 直接设置舵机 PWM 比较值，范围限制为 20 到 100 CC。 */
void Servo_SetCc(uint16_t cc);
/* 设置相对 1500 us 中位的脉宽偏移。 */
void Servo_SetPos(int offset_us);
/* 设置 0 到 180 度的舵机角度。 */
void Set_Servo_Angle(unsigned int angle);

#endif
