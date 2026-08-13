#ifndef __CAR_WITH_BALL_H__
#define __CAR_WITH_BALL_H__

#include <stdint.h>

/* 开始一次新的巡线控球组合过程，并记录缓启动起点。 */
void Car_WithBall_Reset(uint32_t start_ms);
/* 同时更新四路循迹和钢珠控制。 */
void Car_WithBall_Update(int16_t speed, float target_cm);
/* 停止底盘，并让控球器回到任务原始目标。 */
void Car_WithBall_StopCar(float target_cm);
float Car_WithBall_GetCompensatedTarget(void);

#endif
