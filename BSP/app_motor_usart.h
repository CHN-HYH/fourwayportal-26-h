#ifndef __APP_MOTOR_USART_H_
#define __APP_MOTOR_USART_H_

#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "bsp_motor_usart.h"
#include "string.h"
#include "stdlib.h"

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t


//外部声明区	External declaration area
typedef enum _motor_type  //此类型用做判断死区	This type is used to determine the dead zone
{
	MOTOR_TYPE_NONE = 0x00,       // 保留	reserve
	MOTOR_520 ,       //520电机 包含L型	520 motor including L type
	MOTOR_310 ,       //310电机	310 motor
	MOTOR_TT_Encoder ,//tt电机,带编码器	tt motor with encoder
	MOTOR_TT , 		  // tt电机,不带编码器	tt motor, without encoder

	Motor_TYPE_MAX    // 最后一个电机类型，仅作为判断	The last motor type is for judgment only
} motor_type_t;



/* 四路电机的 10 ms 编码器增量，供外部读取。 */
extern int Encoder_Offset[4];
/* 四路电机的累计编码器值，供外部读取。 */
extern int Encoder_Now[4];
/* 四路电机的实时速度，供外部读取。 */
extern float g_Speed[4];
/* 收到完整驱动板帧后置位，由主循环按需清除。 */
extern volatile uint8_t g_recv_flag;


void send_motor_type(motor_type_t data);
void send_motor_deadzone(uint16_t data);
void send_pulse_line(uint16_t data);
void send_pulse_phase(uint16_t data);
void send_wheel_diameter(float data);
/* 设置电机驱动板 PID 的比例、积分和微分参数。 */
void send_motor_PID(float p, float i, float d);
/* 配置上传总编码器、10 ms 编码器增量和速度数据的开关。 */
void send_upload_data(bool all, bool step, bool speed);
/* 下发四路电机的速度控制量。 */
void Contrl_Speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
/* 下发四路电机的 PWM 控制量。 */
void Contrl_Pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

void Deal_Control_Rxtemp(uint8_t rxtemp);
void Deal_data_real(void);
/* 返回成功解析的速度帧累计数量。 */
uint32_t Motor_GetSpeedUpdateCount(void);

#endif

