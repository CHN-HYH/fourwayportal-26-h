#include "servo.h"
#include "ti_msp_dl_config.h"

void Servo_SetCc(uint16_t cc)
{
    if (cc < 20U)
    {
        cc = 20U;
    }
    if (cc > 100U)
    {
        cc = 100U;
    }
    DL_TimerA_setCaptureCompareValue(PWM_Servo_INST,
                                     cc,
                                     DL_TIMER_CC_0_INDEX);
}

void Servo_Init(void)
{
    Servo_SetCc(60U);
    DL_TimerA_startCounter(PWM_Servo_INST);
}

void Set_Servo_Angle(unsigned int angle)
{
    float cc;

    if (angle > 180U)
    {
        angle = 180U;
    }
    cc = 20.0f + ((float)angle / 180.0f) * 80.0f;
    Servo_SetCc((uint16_t)(cc + 0.5f));
}

void Servo_SetPos(int offset_us)
{
    int us = 1500 + offset_us; /* 目标高电平时间，单位 us。 */

    if (us < 500)
    {
        us = 500;
    }
    if (us > 2500)
    {
        us = 2500;
    }
    Servo_SetCc((uint16_t)((us + 12) / 25));
}
