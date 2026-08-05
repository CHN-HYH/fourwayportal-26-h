#include "Four_linewalking.h"
#include "usart.h"

#define IRTrack_Trun_KP (450)
#define IRTrack_Trun_KI (0) 
#define IRTrack_Trun_KD (0) 

int pid_output_IRR = 0;
float err = 0;

int IRR_SPEED = 300;//初始直线速度    Initial linear velocity

int Left_rui = 0;
int Right_rui = 0;
int turn = 0;

float APP_IR_PID_Calc(float actual_value)
{

	float IRTrackTurn = 0;
	int8_t error;
	static int8_t error_last=0;
	static float IRTrack_Integral;//积分  Integral
	

	error=actual_value;
	
	IRTrack_Integral +=error;
	
	//位置式pid    Positional pid
	IRTrackTurn=error*IRTrack_Trun_KP
							+IRTrack_Trun_KI*IRTrack_Integral
							+(error - error_last)*IRTrack_Trun_KD;
	return IRTrackTurn;
}

//获取X1X2X3X4的引脚电平	Get the pin levels of X1X2X3X4
void Four_GetLineWalking(int *LineL1, int *LineL2, int *LineR1, int *LineR2)
{
	*LineL1 = LineWalk_L1_IN;
	*LineL2 = LineWalk_L2_IN;
	*LineR1 = LineWalk_R1_IN;
	*LineR2 = LineWalk_R2_IN;
}

void Four_LineWalking(void)
{
	int LineL1 = 0, LineL2 = 0, LineR1 = 0, LineR2 = 0;
	Four_GetLineWalking(&LineL1, &LineL2, &LineR1, &LineR2);//获取黑线检测状态	Get black line detection status
    //debug
//    printf("L1:%d L2:%d R1:%d R2:%d\r\n",LineL1, LineL2, LineR1, LineR2);

    // 0 0 X 0
    // 1 0 X 0
    // 0 1 X 0
    //处理右锐角和右直角的转动
    //Processing the right acute angle and the right right angle rotation
	if( (LineL1 == LOW || LineL2 == LOW) && LineR2 == LOW) 
    {
        err=13;
		delay_ms(80);
    }
   // 0 X 0 0       
   // 0 X 0 1 
   // 0 X 1 0       
   //处理左锐角和左直角的转动
    //Handling left acute angle and left right angle rotation
    else if ( LineL1 == LOW && (LineR1 == LOW || LineR2 == LOW)) 
	{ 
        err=-13;
		delay_ms(80);
    }
    // 0 X X X
   //最左边检测到
    //Most left detected
    else if( LineL1 == LOW )
    {  
        err=-9;
		delay_ms(10);
	}
    // X X X 0
   //最右边检测到
    //Most right detected
    else if ( LineR2 == LOW)
    {  
        err=9;
//		Contrl_Speed(500,500,-500,-500);
		delay_ms(10);
	}
    // X 0 1 X
   //处理左小弯
    //Processing of the left hand chicane
    else if (LineL2 == LOW && LineR1 == HIGH) //中间黑线上的传感器微调车左转  Sensor on the black line in the center fine tunes the car to turn left
    {   
		err=-1;
	}
    // X 1 0 X  
   //处理右小弯
    //Processing of the right-hand chicane
	else if (LineL2 == HIGH && LineR1 == LOW) //中间黑线上的传感器微调车右转  The sensor on the center black line fine tunes the car to turn right
    {   
		err=1;
	}
    // X 0 0 X
   //处理直线
    //Processing straight lines
    else if(LineL2 == LOW && LineR1 == LOW) // 都是黑色, 加速前进   It's all black, so speed up.
    {  
        err=0;
	}	
    // 0 0 0 0
    else if(LineL1 == LOW && LineL2 == LOW && LineR1 == LOW && LineR2 == LOW) // 都是黑色, 加速前进 It's all black, so speed up.
    {  
		err = 0;
	}
    
    //当为1 1 1 1时小车保持上一个小车运行状态 
    //When it is 1 1 1 1 the trolley keeps the previous trolley in operation 
    
    pid_output_IRR = (int)(APP_IR_PID_Calc(err));
	
	Motion_Car_Control(IRR_SPEED, 0, pid_output_IRR);
    
}


