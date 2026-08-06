#include "ti_msp_dl_config.h"
#include <stdint.h>

/* UniFlash 串口 BSL 要求 Flash 镜像长度按 8 字节对齐，当前工程补 4 字节。 */
__attribute__((used))
const uint32_t g_uniflash_padding = 0xFFFFFFFFU;
#include "delay.h"
#include "usart.h"
#include "bsp_camera_usart.h"
#include "app_motor_usart.h"
#include "Four_linewalking.h"
#include "app_motor.h"

#define MOTOR_TYPE 2   //1:520电机 2:310电机 3:测速码盘TT电机 4:TT直流减速电机 5:L型520电机
                       //1:520 motor 2:310 motor 3:speed code disc TT motor 4:TT DC reduction motor 5:L type 520 motor

int main(void)
{	
	USART_Init();
	Camera_Vision_Init();
	printf("please wait...");
    
    Set_Motor(MOTOR_TYPE);
    
    //修改电机PID，这里的参数是为四驱310底盘配置的，其他底盘需要自己测试修改
    //Modify the motor PID, the parameters here are configured for the 4WD 310 chassis, other chassis need to test and modify their own!
	send_motor_PID(1.9,0.2,0.8);
    
    delay_ms(100);

	while(1)
	{
		Camera_Vision_Process();
		Four_LineWalking();//四路巡线，启动！	Four-way line patrol, start!
	}
	
}
