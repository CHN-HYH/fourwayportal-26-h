#include "app_motor_usart.h"

#define RX_N 256 /* 电机驱动板协议帧的最大有效长度。 */

static uint8_t s_tx[50]; /* 电机驱动板命令的格式化发送缓冲区。 */

float g_Speed[4];     /* 四路电机的实时速度。 */
int Encoder_Offset[4]; /* 四路电机的 10 ms 编码器增量。 */
int Encoder_Now[4];   /* 四路电机的累计编码器值。 */

uint8_t g_recv_flag; /* 接收到一帧完整驱动板数据的标志。 */
static uint8_t s_rx[RX_N];      /* ISR 正在接收的协议帧内容。 */
static uint8_t s_rx_done[RX_N]; /* 收到结束符后的完整协议帧内容。 */

//////////********************发送部分********************///////////
//////////******************Sending part*****************///////////

//发送电机类型	Transmitter motor type
void send_motor_type(motor_type_t data)
{
	sprintf((char*)s_tx,"$mtype:%d#",data);
	Send_Motor_ArrayU8(s_tx, strlen((char*)s_tx));
	
}

//发送电机死区	Send motor dead zone
void send_motor_deadzone(uint16_t data)
{
	sprintf((char*)s_tx,"$deadzone:%d#",data);
	Send_Motor_ArrayU8(s_tx, strlen((char*)s_tx));
}

//发送电机磁环脉冲	Send motor magnetic ring pulse
void send_pulse_line(uint16_t data)
{
	sprintf((char*)s_tx,"$mline:%d#",data);
	Send_Motor_ArrayU8(s_tx, strlen((char*)s_tx));
}

//发送电机减速比	Transmitting motor reduction ratio
void send_pulse_phase(uint16_t data)
{
	sprintf((char*)s_tx,"$mphase:%d#",data);
	Send_Motor_ArrayU8(s_tx, strlen((char*)s_tx));
}

//发送轮子直径	Send wheel diameter
void send_wheel_diameter(float data)
{
	sprintf((char*)s_tx,"$wdiameter:%.3f#",data);
	Send_Motor_ArrayU8(s_tx, strlen((char*)s_tx));
}

//发送PID参数	Send PID parameters
void send_motor_PID(float p, float i, float d)
{
	sprintf((char*)s_tx,"$mpid:%.3f,%.3f,%.3f#",p,i,d);
	Send_Motor_ArrayU8(s_tx, strlen((char*)s_tx));
}

//需要接收数据的开关	Switch that needs to receive data
void send_upload_data(bool all, bool step, bool speed)
{
	sprintf((char*)s_tx,"$upload:%d,%d,%d#",all,step,speed);
	Send_Motor_ArrayU8(s_tx, strlen((char*)s_tx));
}

//控制速度	Controlling Speed
void Contrl_Speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
	sprintf((char*)s_tx,"$spd:%d,%d,%d,%d#",m1,m2,m3,m4);
	Send_Motor_ArrayU8(s_tx, strlen((char*)s_tx));
}


//控制pwm	Control PWM
void Contrl_Pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
	sprintf((char*)s_tx,"$pwm:%d,%d,%d,%d#",m1,m2,m3,m4);
	Send_Motor_ArrayU8(s_tx, strlen((char*)s_tx));
}


//////////********************接收部分********************///////////
//////////*****************Receiving part****************///////////

//传入参数：保留的字符串(指针数组)  原始字符串  分隔符号
//Incoming parameters: reserved string (pointer array) original string separator
static void split(char *part[], char *str, const char *sep)
{
    char *tok = strtok(str, sep); /* 当前分割得到的字段。 */
    int n = 0;                    /* 已保存的字段数。 */
	
    while (tok != NULL && n < 10)
    {
        part[n++] = tok;
        tok = strtok(NULL, sep);
    }

    if (n < 10)
    {
        part[n] = NULL;
    }
}

//检验从驱动板发送过来的数据，符合通讯协议的数据则保存下来
//Check the data sent from the driver board, and save the data that meets the communication protocol
void Deal_Control_Rxtemp(uint8_t byte)
{
	static u16 n = 0;       /* 当前帧已接收的字节数。 */
	static u8 started = 0;  /* 是否已经收到帧起始符 '$'。 */

	if(byte == '$' && started == 0)
	{
		started = 1;
		memset(s_rx,0,RX_N);//清空数据	Clear data
	}
	
	else if(started == 1)
	{
			if(byte == '#')
			{
				started = 0;
				n = 0;
				g_recv_flag = 1;
				memcpy(s_rx_done,s_rx,RX_N); //只有正确才会赋值	Only correct ones will be assigned
			}
			else
			{
				if(n >= RX_N - 1)
				{
					started = 0;
					n = 0;
					memset(s_rx,0,RX_N);//清空接收数据	Clear received data
				}
				else
				{
					s_rx[n] = byte;
					n++;
				}
			}
	}
	
}

//将从驱动板保存到的数据进行格式处理，然后准备打印
//Format the data saved from the driver board and prepare it for printing
void Deal_data_real(void)
{
	static uint8_t buf[RX_N]; /* 去掉协议头后的可分割数据。 */
	uint8_t len = 0;          /* 当前数据区长度。 */
	uint16_t frame = 0;       /* 已接收完整帧的实际长度。 */

	while (frame < RX_N && s_rx_done[frame] != '\0')
	{
		frame++;
	}
	if (frame < 5 || frame >= RX_N || s_rx_done[4] != ':')
	{
		return;
	}
	
	//总体的编码器	Overall encoder
	 if ((strncmp("MAll",(char*)s_rx_done,4)==0))
    {
        len = (uint8_t)(frame - 5);
        for (uint8_t i = 0; i < len; i++)
        {
            buf[i] = s_rx_done[i+5]; //去掉冒号	Remove the colon
        }  
				buf[len] = '\0';

					
				char *part[10];//指针数组 长度根据分割号定义  char 1字节   char* 4字节	 Pointer array The length is defined by the split number char 1 byte char* 4 bytes
				char num[4][10] = {'\0'}; /* 保存每路编码器文本的临时数组。 */
				split(part,(char*)buf, ", ");//以逗号切割	Split by comma
				for (int i = 0; i < 4; i++)
				{
						if (part[i] == NULL || strlen(part[i]) >= sizeof(num[i]))
						{
							return;
						}
						strcpy(num[i],part[i]);
						Encoder_Now[i] = atoi(num[i]);
				}
				
		}
		//10ms的实时编码器数据	10ms real-time encoder data
		else if	((strncmp("MTEP",(char*)s_rx_done,4)==0))
    {
        len = (uint8_t)(frame - 5);
        for (uint8_t i = 0; i < len; i++)
        {
            buf[i] = s_rx_done[i+5]; //去掉冒号	Remove the colon
        }  
				buf[len] = '\0';

				char *part[10];//指针数组 长度根据分割号定义  char 1字节   char* 4字节		Pointer array The length is defined by the split number char 1 byte char* 4 bytes
				char num[4][10] = {'\0'}; /* 保存每路编码器文本的临时数组。 */
				split(part,(char*)buf, ", ");//以逗号切割	Split by comma
				for (int i = 0; i < 4; i++)
				{
						if (part[i] == NULL || strlen(part[i]) >= sizeof(num[i]))
						{
							return;
						}
						strcpy(num[i],part[i]);
						Encoder_Offset[i] = atoi(num[i]);
				}
		}
		//速度	Speed
		else if	((strncmp("MSPD",(char*)s_rx_done,4)==0))
    {
        len = (uint8_t)(frame - 5);
        for (uint8_t i = 0; i < len; i++)
        {
            buf[i] = s_rx_done[i+5]; //去掉冒号	Remove the colon
        }  
				buf[len] = '\0';
				
				char *part[10];//指针数组 长度根据分割号定义  char 1字节   char* 4字节		Pointer array The length is defined by the split number char 1 byte char* 4 bytes
				char num[4][10] = {'\0'}; /* 保存每路速度文本的临时数组。 */
				split(part,(char*)buf, ", ");//以逗号切割	Split by comma
				for (int i = 0; i < 4; i++)
				{
						if (part[i] == NULL || strlen(part[i]) >= sizeof(num[i]))
						{
							return;
						}
						strcpy(num[i],part[i]);
						g_Speed[i] = atof(num[i]);
				}
		}
}
