#include "servo.h"
#include "ti_msp_dl_config.h"

#define SERVO_MIN_PULSE_US      (500U)   /* 0 度对应的高电平宽度，单位 us。 */
#define SERVO_MAX_PULSE_US      (2500U)  /* 180 度对应的高电平宽度，单位 us。 */
#define SERVO_MAX_ANGLE         (180U)   /* 支持的最大输入角度，单位度。 */
#define SERVO_PERIOD_TICKS      (20000U) /* 50 Hz PWM 周期，定时器时钟为 1 MHz。 */

void Servo_Init(void)
{
    Servo_SetAngle(0U);
    DL_TimerA_startCounter(SERVO_PWM_INST);
}

void Servo_SetAngle(uint16_t angle)
{
    uint32_t pulse; /* 根据目标角度换算出的高电平宽度，单位 us。 */

    if (angle > SERVO_MAX_ANGLE)
    {
        angle = SERVO_MAX_ANGLE;
    }

    pulse = SERVO_MIN_PULSE_US +
               (((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) /
                SERVO_MAX_ANGLE);

    DL_TimerA_setCaptureCompareValue(SERVO_PWM_INST,
                                     SERVO_PERIOD_TICKS - pulse,
                                     DL_TIMER_CC_1_INDEX);
}
