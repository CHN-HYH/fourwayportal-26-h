#ifndef __APP_MOTOR_H_
#define __APP_MOTOR_H_

#include "ti_msp_dl_config.h"
#include "app_motor_usart.h"
#include "delay.h"

/* 小车底盘电机间距之和的一半，单位 mm。 */
#define Car_APB          				(188.0f) /* (228 + 148) / 2 */

/* 按预设型号配置电机驱动参数。 */
void Set_Motor(int type);
/* 直接下发底盘 PWM：vx 前后、vy 预留横移、vz 转向。 */
void Motion_Car_Control(int16_t vx, int16_t vy, int16_t vz);


#endif
