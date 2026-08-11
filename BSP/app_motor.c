#include "app_motor.h"

static float s_lr = 0.0f;  /* 预留横移分量，当前底盘未参与混控。 */
static float s_fb = 0.0f;  /* 前后行驶的 PWM 控制量。 */
static float s_turn = 0.0f; /* 转向叠加到左右轮的 PWM 控制量。 */
static int s_l1 = 0;       /* 左前轮的 PWM 控制量。 */
static int s_l2 = 0;       /* 左后轮的 PWM 控制量。 */
static int s_r1 = 0;       /* 右前轮的 PWM 控制量。 */
static int s_r2 = 0;       /* 右后轮的 PWM 控制量。 */

/* 返回当前小车轮子轴间距和的一半。 */
static float get_apb(void)
{
    return Car_APB;
}

void Set_Motor(int type)
{
    if(type == 1)
    {
        send_motor_type(1);//配置电机类型	Configure motor type
        delay_ms(100);
        send_pulse_phase(30);//配置减速比 查电机手册得出	Configure the reduction ratio. Check the motor manual to find out
        delay_ms(100);
        send_pulse_line(11);//配置磁环线 查电机手册得出	Configure the magnetic ring wire. Check the motor manual to get the result.
        delay_ms(100);
        send_wheel_diameter(67.00);//配置轮子直径,测量得出		Configure the wheel diameter and measure it
        delay_ms(100);
        send_motor_deadzone(1900);//配置电机死区,实验得出	Configure the motor dead zone, and the experiment shows
        delay_ms(100);
    }
    
    else if(type == 2)
    {
        send_motor_type(2);
        delay_ms(100);
        send_pulse_phase(20);
        delay_ms(100);
        send_pulse_line(13);
        delay_ms(100);
        send_wheel_diameter(48.00);
        delay_ms(100);
        send_motor_deadzone(1900);
        delay_ms(100);
    }
    
    else if(type == 3)
    {
        send_motor_type(3);
        delay_ms(100);
        send_pulse_phase(45);
        delay_ms(100);
        send_pulse_line(13);
        delay_ms(100);
        send_wheel_diameter(68.00);
        delay_ms(100);
        send_motor_deadzone(1600);
        delay_ms(100);
    }
    
    else if(type == 4)
    {
        send_motor_type(4);
        delay_ms(100);
        send_pulse_phase(48);
        delay_ms(100);
        send_motor_deadzone(1000);
        delay_ms(100);
    }
    
    else if(type == 5)
    {
        send_motor_type(1);
        delay_ms(100);
        send_pulse_phase(40);
        delay_ms(100);
        send_pulse_line(11);
        delay_ms(100);
        send_wheel_diameter(67.00);
        delay_ms(100);
        send_motor_deadzone(1900);
        delay_ms(100);
    }
}

//直接控制pwm
void Motion_Car_Control(int16_t vx, int16_t vy, int16_t vz)
{
	float apb = get_apb(); /* 底盘轴距和的一半，单位与配置值一致。 */
	s_lr = 0.0f;
    s_fb = vx;
    s_turn = (vz / 1000.0f) * apb;
    if (vx == 0 && vy == 0 && vz == 0)
    {
        Contrl_Speed(0,0,0,0);
        return;
    }

    s_l1 = s_fb + s_turn;
    s_l2 = s_fb + s_turn;
    s_r1 = s_fb - s_turn;
    s_r2 = s_fb - s_turn;
		
    if (s_l1 > 1000) s_l1 = 1000;
    if (s_l1 < -1000) s_l1 = -1000;
    if (s_l2 > 1000) s_l2 = 1000;
    if (s_l2 < -1000) s_l2 = -1000;
    if (s_r1 > 1000) s_r1 = 1000;
    if (s_r1 < -1000) s_r1 = -1000;
    if (s_r2 > 1000) s_r2 = 1000;
    if (s_r2 < -1000) s_r2 = -1000;
    
    //printf("%d\t,%d\t,%d\t,%d\r\n",s_l1,s_l2,s_r1,s_r2);
    
    Contrl_Speed(s_l1, s_l2, s_r1, s_r2);
		
}

