#include "Four_linewalking.h"
#include "usart.h"

#define IR_KP (450) /* 红外循迹位置环比例系数。 */
#define IR_KI (0)   /* 红外循迹位置环积分系数。 */
#define IR_KD (0)   /* 红外循迹位置环微分系数。 */

static int s_out = 0;   /* 循迹 PID 输出的转向控制量。 */
static float s_err = 0; /* 当前循迹位置误差。 */

static int s_spd = 300; /* 初始直线速度的 PWM 控制量。 */

int Left_rui = 0;  /* 预留左锐角标志，当前未参与控制。 */
int Right_rui = 0; /* 预留右锐角标志，当前未参与控制。 */
int turn = 0;      /* 预留转向状态，当前未参与控制。 */

/* 根据当前循迹误差计算位置式 PID 转向量。 */
static float pid(float val)
{

	float out = 0.0f;       /* 本次 PID 计算得到的转向量。 */
	int8_t err;             /* 当前离散循迹误差。 */
	static int8_t last = 0; /* 上次误差，供微分项使用。 */
	static float sum;       /* 误差积分累计值。 */
	

	err = val;
	
	sum += err;
	
	//位置式pid    Positional pid
	out = err * IR_KP + sum * IR_KI + (err - last) * IR_KD;
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

void Four_LineWalking(void)
{
	int l1 = 0, l2 = 0, r1 = 0, r2 = 0; /* 四路红外传感器的当前电平。 */
	read_line(&l1, &l2, &r1, &r2);//获取黑线检测状态	Get black line detection status
    //debug
//    printf("L1:%d L2:%d R1:%d R2:%d\r\n",l1, l2, r1, r2);

    // 0 0 X 0
    // 1 0 X 0
    // 0 1 X 0
    //处理右锐角和右直角的转动
    //Processing the right acute angle and the right right angle rotation
	if( (l1 == LOW || l2 == LOW) && r2 == LOW)
    {
		s_err = 13.0f;
		delay_ms(80);
    }
   // 0 X 0 0       
   // 0 X 0 1 
   // 0 X 1 0       
   //处理左锐角和左直角的转动
    //Handling left acute angle and left right angle rotation
    else if (l1 == LOW && (r1 == LOW || r2 == LOW))
	{ 
		s_err = -13.0f;
		delay_ms(80);
    }
    // 0 X X X
   //最左边检测到
    //Most left detected
    else if(l1 == LOW)
    {  
		s_err = -9.0f;
		delay_ms(10);
	}
    // X X X 0
   //最右边检测到
    //Most right detected
    else if(r2 == LOW)
    {  
		s_err = 9.0f;
//		Contrl_Speed(500,500,-500,-500);
		delay_ms(10);
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
    // 0 0 0 0
    else if(l1 == LOW && l2 == LOW && r1 == LOW && r2 == LOW) // 都是黑色, 加速前进 It's all black, so speed up.
    {  
		s_err = 0.0f;
	}
    
    //当为1 1 1 1时小车保持上一个小车运行状态
    //When it is 1 1 1 1 the trolley keeps the previous trolley in operation 
    
	 s_out = (int)(pid(s_err));
	
	Motion_Car_Control(s_spd, 0, s_out);
    
}


