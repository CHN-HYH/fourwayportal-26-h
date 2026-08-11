#ifndef __FOUR_LINEWALKING_H__
#define __FOUR_LINEWALKING_H__	
#include "ti_msp_dl_config.h"
#include "app_motor_usart.h"
#include "app_motor.h"
/*

	从车身后面往前看： 左侧到右边巡线传感器顺序为  L2  L1  R1  R2
	Looking forward from the rear of the vehicle: The order of the line-following sensors from left to right is L2  L1  R1  R2

*/

#define LineWalk_L1_IN		( ( ( DL_GPIO_readPins(Sensor_PORT,Sensor_X2_PIN) & Sensor_X2_PIN ) > 0 ) ? 1 : 0 )
#define LineWalk_L2_IN		( ( ( DL_GPIO_readPins(Sensor_PORT,Sensor_X1_PIN) & Sensor_X1_PIN ) > 0 ) ? 1 : 0 )
#define LineWalk_R1_IN		( ( ( DL_GPIO_readPins(Sensor_PORT,Sensor_X3_PIN) & Sensor_X3_PIN ) > 0 ) ? 1 : 0 )
#define LineWalk_R2_IN		( ( ( DL_GPIO_readPins(Sensor_PORT,Sensor_X4_PIN) & Sensor_X4_PIN ) > 0 ) ? 1 : 0 )

#define LOW		(0)
#define HIGH	(1)

/* 预留的左锐角循迹状态。 */
extern int Left_rui;
/* 预留的右锐角循迹状态。 */
extern int Right_rui;

/* 读取四路红外传感器并输出底盘转向控制量。 */
void Four_LineWalking(void);


#endif


