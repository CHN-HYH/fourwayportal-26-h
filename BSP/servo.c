#include "servo.h"
#include "ti_msp_dl_config.h"

void Servo_Init(void)
{
    DL_TimerA_startCounter(PWM_Servo_INST);
    DL_TimerA_setCaptureCompareValue(PWM_Servo_INST, 60U, DL_TIMER_CC_0_INDEX);
}

void Set_Servo_Angle(unsigned int angle)
{
    float cc;
    float delta;
    float max_step = 10.0f; /* 单次最大变化量，单位 CC。 */
    static float cc_last = 60.0f; /* 上一次舵机位置，单位 CC。 */

    if (angle > 180U)
    {
        angle = 180U;
    }
    cc = 20.0f + ((float)angle / 180.0f) * 80.0f;
    delta = cc - cc_last;
    if ((delta > -3.0f) && (delta < 3.0f))
    {
        return;
    }
    if (delta > max_step)
    {
        cc = cc_last + max_step;
    }
    if (delta < -max_step)
    {
        cc = cc_last - max_step;
    }
    cc_last = cc;

    if (cc < 20.0f)
    {
        cc = 20.0f;
    }
    if (cc > 100.0f)
    {
        cc = 100.0f;
    }
    DL_TimerA_setCaptureCompareValue(PWM_Servo_INST,
                                     (unsigned int)(cc + 0.5f),
                                     DL_TIMER_CC_0_INDEX);
    DL_TimerA_startCounter(PWM_Servo_INST);
}

void Servo_SetPos(int offset_us)
{
    int us = 1500 + offset_us; /* 目标高电平时间，单位 us。 */
    float cc;
    float delta;
    float max_step = 10.0f; /* 单次最大变化量，单位 CC。 */
    static float cc_last = 60.0f; /* 上一次舵机位置，单位 CC。 */

    if (us < 500)
    {
        us = 500;
    }
    if (us > 2500)
    {
        us = 2500;
    }
    cc = (float)us / 25.0f;
    delta = cc - cc_last;
    if ((delta > -3.0f) && (delta < 3.0f))
    {
        return;
    }
    if (delta > max_step)
    {
        cc = cc_last + max_step;
    }
    if (delta < -max_step)
    {
        cc = cc_last - max_step;
    }
    cc_last = cc;

    if (cc < 20.0f)
    {
        cc = 20.0f;
    }
    if (cc > 100.0f)
    {
        cc = 100.0f;
    }
    DL_TimerA_setCaptureCompareValue(PWM_Servo_INST,
                                     (unsigned int)(cc + 0.5f),
                                     DL_TIMER_CC_0_INDEX);
    DL_TimerA_startCounter(PWM_Servo_INST);
}
