#include "Four_linewalking.h"
#include "bsp_camera_usart.h"
#include "usart.h"

#define IR_KP (450) /* 红外循迹位置环比例系数。 */
#define IR_KI (0)   /* 红外循迹位置环积分系数。 */
#define IR_KD (0)   /* 红外循迹位置环微分系数。 */
#define LINE_UPDATE_MS    (10U) /* 循迹和电机指令的固定更新周期。 */
#define LINE_SHARP_HOLD_MS (80U) /* 锐角状态的非阻塞保持时间。 */
#define LINE_EDGE_HOLD_MS  (10U) /* 最外侧探头状态的非阻塞保持时间。 */
#define LINE_LOST_LIMIT       (20U) /* 连续全白达到约 200 ms 后停车。 */

static int16_t s_out = 0;      /* 循迹 PID 输出的转向控制量。 */
static float s_err = 0.0f;     /* 当前循迹位置误差。 */
static int16_t s_spd = 300;    /* 当前直线速度控制量。 */
static int8_t s_last = 0;      /* 上次循迹误差，供微分项使用。 */
static float s_sum = 0.0f;     /* 循迹误差积分累计值。 */
static uint8_t s_pattern = 0U; /* 四路传感器电平，bit3~0 对应 L2/L1/R1/R2。 */
static uint8_t s_black = 0U;   /* 当前检测到黑线的传感器数量。 */
static uint32_t s_update_ms = 0U; /* 上一次循迹更新时刻。 */
static uint32_t s_hold_until = 0U; /* 特殊转向保持结束时刻。 */
static uint8_t s_lost_n = 0U;      /* 连续全白的循迹更新次数。 */

int Left_rui = 0;  /* 预留左锐角标志，当前未参与控制。 */
int Right_rui = 0; /* 预留右锐角标志，当前未参与控制。 */
int turn = 0;      /* 预留转向状态，当前未参与控制。 */

/* 根据当前循迹误差计算位置式 PID 转向量。 */
static float pid(float val)
{
	float out = 0.0f;       /* 本次 PID 计算得到的转向量。 */
	int8_t err;             /* 当前离散循迹误差。 */

	err = val;
	s_sum += err;
	if (s_sum > 100.0f) s_sum = 100.0f;
	if (s_sum < -100.0f) s_sum = -100.0f;
	
	//位置式pid    Positional pid
	out = err * IR_KP + s_sum * IR_KI + (err - s_last) * IR_KD;
	s_last = err;
	return out;
}

//获取X1X2X3X4的引脚电平	Get the pin levels of X1X2X3X4
/* 采样四路红外循迹传感器的当前电平。 */
static void read_line(int *l1, int *l2, int *r1, int *r2)
{
	*l1 = LineWalk_L1_IN;
	*l2 = LineWalk_L2_IN;
	*r1 = LineWalk_R1_IN;
	*r2 = LineWalk_R2_IN;
}

void Four_Line_Init(void)
{
	s_spd = 300;
	Four_Line_Reset();
}

void Four_Line_Reset(void)
{
	s_out = 0;
	s_err = 0.0f;
	s_last = 0;
	s_sum = 0.0f;
	s_pattern = 0U;
	s_black = 0U;
	s_update_ms = 0U;
	s_hold_until = 0U;
	s_lost_n = 0U;
}

void Four_Line_SetSpeed(int16_t speed)
{
	if (speed < 0) speed = 0;
	if (speed > 1000) speed = 1000;
	s_spd = speed;
}

void Four_Line_Update(void)
{
	int l1 = 0, l2 = 0, r1 = 0, r2 = 0; /* 四路红外传感器的当前电平。 */
	uint32_t now = Camera_Vision_GetTimeMs(); /* 当前毫秒时间。 */

	if ((s_update_ms != 0U) &&
	    ((uint32_t)(now - s_update_ms) < LINE_UPDATE_MS))
	{
		return;
	}
	s_update_ms = now;
	read_line(&l1, &l2, &r1, &r2);//获取黑线检测状态	Get black line detection status
	s_pattern = (uint8_t)((l2 << 3) | (l1 << 2) | (r1 << 1) | r2);
	s_black = (uint8_t)((l1 == LOW) + (l2 == LOW) +
	                    (r1 == LOW) + (r2 == LOW));
	if (s_black == 0U)
	{
		if (s_lost_n < LINE_LOST_LIMIT)
		{
			s_lost_n++;
		}
		if (s_lost_n >= LINE_LOST_LIMIT)
		{
			Four_Line_Stop();
			return;
		}
	}
	else
	{
		s_lost_n = 0U;
	}
    //debug
//    printf("L1:%d L2:%d R1:%d R2:%d\r\n",l1, l2, r1, r2);

    // 0 0 X 0
    // 1 0 X 0
    // 0 1 X 0
    //处理右锐角和右直角的转动
    //Processing the right acute angle and the right right angle rotation
    /* 全黑优先视为停止线，避免被左右锐角条件抢先匹配。 */
	if(l1 == LOW && l2 == LOW && r1 == LOW && r2 == LOW)
    {
		s_err = 0.0f;
		s_hold_until = now;
    }
	else if( (l1 == LOW || l2 == LOW) && r2 == LOW)
    {
		s_err = 13.0f;
		s_hold_until = now + LINE_SHARP_HOLD_MS;
    }
   // 0 X 0 0       
   // 0 X 0 1 
   // 0 X 1 0       
   //处理左锐角和左直角的转动
    //Handling left acute angle and left right angle rotation
    else if (l1 == LOW && (r1 == LOW || r2 == LOW))
	{ 
		s_err = -13.0f;
		s_hold_until = now + LINE_SHARP_HOLD_MS;
    }
	else if ((int32_t)(now - s_hold_until) < 0)
	{
		/* 保持上一次特殊转向，但主循环和视觉解析继续运行。 */
	}
    // 0 X X X
   //最左边检测到
    //Most left detected
    else if(l1 == LOW)
    {  
		s_err = -9.0f;
		s_hold_until = now + LINE_EDGE_HOLD_MS;
	}
    // X X X 0
   //最右边检测到
    //Most right detected
    else if(r2 == LOW)
    {  
		s_err = 9.0f;
		s_hold_until = now + LINE_EDGE_HOLD_MS;
//		Contrl_Speed(500,500,-500,-500);
	}
    // X 0 1 X
   //处理左小弯
    //Processing of the left hand chicane
    else if (l2 == LOW && r1 == HIGH) //中间黑线上的传感器微调车左转  Sensor on the black line in the center fine tunes the car to turn left
    {   
		s_err = -1.0f;
	}
    // X 1 0 X  
   //处理右小弯
    //Processing of the right-hand chicane
	else if (l2 == HIGH && r1 == LOW) //中间黑线上的传感器微调车右转  The sensor on the center black line fine tunes the car to turn right
    {   
		s_err = 1.0f;
	}
    // X 0 0 X
   //处理直线
    //Processing straight lines
    else if(l2 == LOW && r1 == LOW) // 都是黑色, 加速前进   It's all black, so speed up.
    {  
		s_err = 0.0f;
	}	
    //当为1 1 1 1时小车保持上一个小车运行状态
    //When it is 1 1 1 1 the trolley keeps the previous trolley in operation 
    
	 s_out = (int16_t)pid(s_err);
	
	Motion_Car_Control(s_spd, 0, s_out);
}

void Four_Line_Stop(void)
{
	Motion_Car_Control(0, 0, 0);
}

int16_t Four_Line_GetSpeed(void)
{
	return s_spd;
}

int16_t Four_Line_GetTurn(void)
{
	return s_out;
}

uint8_t Four_Line_GetPattern(void)
{
	return s_pattern;
}

uint8_t Four_Line_GetBlackCount(void)
{
	return s_black;
}

uint8_t Four_Line_IsLost(void)
{
	return (uint8_t)(s_lost_n >= LINE_LOST_LIMIT);
}

void Four_LineWalking(void)
{
	Four_Line_Update();
}
