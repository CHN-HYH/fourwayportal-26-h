#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdint.h>

/* 初始化 TIMA0 CCP0 的 50 Hz 舵机 PWM 输出。 */
void Servo_Init(void);
/* 设置目标角度，超出 0 到 180 度时自动限幅。 */
void Servo_SetAngle(uint16_t angle);

#endif
