#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdint.h>

/* 初始化 TIMA0 CCP0 的 50 Hz 舵机 PWM 输出。 */
void Servo_Init(void);
/* 直接设置高电平脉宽，超出 500 到 2500 us 时自动限幅。 */
void Servo_SetPulseUs(uint16_t pulse_us);
/* 设置目标角度，超出 0 到 180 度时自动限幅。 */
void Servo_SetAngle(uint16_t angle);

#endif
