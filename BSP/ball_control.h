#ifndef __BALL_CONTROL_H__
#define __BALL_CONTROL_H__

#include <stdint.h>

/* 初始化控球器和舵机，默认目标沿用当前调参目标。 */
void Ball_Control_Init(void);
/* 清除滤波、速度和临时推力状态，目标位置保持不变。 */
void Ball_Control_Reset(void);
/* 使用最新视觉位置更新舵机，target_cm 为相对摆杆中心的位置。 */
void Ball_Control_Update(float target_cm);
/* 停止闭环并让摆杆回到当前标定的平衡角。 */
void Ball_Control_Stop(void);

float Ball_Control_GetTarget(void);
float Ball_Control_GetPosition(void);
float Ball_Control_GetVelocity(void);
uint8_t Ball_Control_IsUsable(void);
uint8_t Ball_Control_IsStable(float tolerance_cm);

#endif
