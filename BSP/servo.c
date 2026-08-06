#include "servo.h"
#include "ti_msp_dl_config.h"

#define SERVO_MIN_PULSE_US      (500U)
#define SERVO_MAX_PULSE_US      (2500U)
#define SERVO_MAX_ANGLE         (180U)
#define SERVO_PERIOD_TICKS      (20000U)

void Servo_Init(void)
{
    Servo_SetAngle(0U);
    DL_TimerA_startCounter(SERVO_PWM_INST);
}

void Servo_SetAngle(uint16_t angle)
{
    uint32_t pulse_us;

    if (angle > SERVO_MAX_ANGLE)
    {
        angle = SERVO_MAX_ANGLE;
    }

    pulse_us = SERVO_MIN_PULSE_US +
               (((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) /
                SERVO_MAX_ANGLE);

    DL_TimerA_setCaptureCompareValue(SERVO_PWM_INST,
                                     SERVO_PERIOD_TICKS - pulse_us,
                                     DL_TIMER_CC_1_INDEX);
}
