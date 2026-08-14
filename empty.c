#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdio.h>

// /* UniFlash 串口 BSL 要求 Flash 镜像长度按 8 字节对齐，当前工程补 4 字节。 */
// __attribute__((used))
// const uint32_t g_uniflash_padding = 0xFFFFFFFFU;
#include "delay.h"
#include "key.h"
#include "usart.h"
#include "bsp_camera_usart.h"
#include "vision_servo_test.h"
#include "app_motor_usart.h"
#include "Four_linewalking.h"
#include "app_motor.h"

#define MOTOR_TYPE 2   //1:520电机 2:310电机 3:测速码盘TT电机 4:TT直流减速电机 5:L型520电机
                       //1:520 motor 2:310 motor 3:speed code disc TT motor 4:TT DC reduction motor 5:L type 520 motor

int main(void)
{
	VisionServoResult control_result; /* 当前有效帧的钢珠控制结果。 */
	KeyEvent key_event;         /* 当前检测到的一次性按键事件。 */
	float target_cm = SV_TEST_ORIGIN_TARGET_CM; /* 当前阶段目标位置。 */
	uint8_t stage = 0U;       /* 0：原点等待；1：确认原点；2：前往 5 cm；3：前往 -5 cm；4：保持 -5 cm。 */
	uint8_t timeout_printed = 0U; /* 是否已经输出过超时提示。 */
	uint32_t control_start_ms; /* K4 启动动作的时刻，单位 ms。 */
	uint32_t elapsed_ms;       /* K4 启动后已经使用的时间，单位 ms。 */

	/* 初始化系统时钟、GPIO、定时器和 PWM 外设。 */
	SYSCFG_DL_init();
	/* 初始化调试串口与摄像头接收。 */
	USART_Init();
	Camera_Vision_Init();
	control_start_ms = Camera_Vision_GetTimeMs();
	/* 初始化钢珠闭环控制，舵机先回到已调整好的水平中位。 */
	Vision_Servo_Test_Init();
	printf("[CONTROL] target=0.0cm waiting key4 camera_center_x=158\r\n");
    // Set_Motor(MOTOR_TYPE);
    
    // //修改电机PID，这里的参数是为四驱310底盘配置的，其他底盘需要自己测试修改
    // //Modify the motor PID, the parameters here are configured for the 4WD 310 chassis, other chassis need to test and modify their own!
	// send_motor_PID(1.9,0.2,0.8);
    
    // delay_ms(100);

	while(1)
	{
		key_event = Key_GetEvent();
		if (key_event == KEY_EVENT_K4)
		{
			target_cm = SV_TEST_ORIGIN_TARGET_CM;
			stage = 1U;
			timeout_printed = 0U;
			control_start_ms = Camera_Vision_GetTimeMs();
			printf("[CONTROL] key4 sequence target=0.0cm\r\n");
		}

		/* 先解析摄像头数据，再用每个新有效帧更新一次闭环控制。 */
		Camera_Vision_Process();
		// Four_LineWalking();//四路巡线，启动！	Four-way line patrol, start!
		control_result = Vision_Servo_Test_Update(target_cm);
		elapsed_ms = Camera_Vision_GetTimeMs() - control_start_ms;

		if ((stage >= 1U) && (stage <= 3U) &&
		    (control_result == VISION_SERVO_REACHED))
		{
			if (stage == 1U)
			{
				stage = 2U;
				target_cm = SV_TEST_FIRST_TARGET_CM;
				printf("[CONTROL] target=5.0cm elapsed=%lums\r\n",
				       (unsigned long)elapsed_ms);
			}
			else if (stage == 2U)
			{
				stage = 3U;
				target_cm = SV_TEST_SECOND_TARGET_CM;
				printf("[CONTROL] target=-5.0cm elapsed=%lums\r\n",
				       (unsigned long)elapsed_ms);
			}
			else
			{
				stage = 4U;
				printf("[CONTROL] target=-5.0cm stable elapsed=%lums\r\n",
				       (unsigned long)elapsed_ms);
			}
		}

		if ((stage >= 1U) && (stage <= 3U) &&
		    (timeout_printed == 0U) &&
		    (elapsed_ms >= SV_TEST_TIMEOUT_MS))
		{
			timeout_printed = 1U;
			printf("[CONTROL] timeout elapsed=%lums stage=%u\r\n",
			       (unsigned long)elapsed_ms,
			       (unsigned int)stage);
		}
	}
	
}
