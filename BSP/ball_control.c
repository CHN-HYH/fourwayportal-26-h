#include "ball_control.h"
#include "bsp_camera_usart.h"
#include "vision_servo_test.h"

#define BALL_TARGET_MIN_CM      (-12.0f) /* 钢珠中心允许的最小目标位置。 */
#define BALL_TARGET_MAX_CM       (12.0f) /* 钢珠中心允许的最大目标位置。 */
#define BALL_STABLE_VEL_CM_S      (0.50f) /* 稳定判定允许的最大绝对速度。 */

static float absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

static float clamp_target(float target_cm)
{
    if (target_cm < BALL_TARGET_MIN_CM)
    {
        return BALL_TARGET_MIN_CM;
    }
    if (target_cm > BALL_TARGET_MAX_CM)
    {
        return BALL_TARGET_MAX_CM;
    }
    return target_cm;
}

void Ball_Control_Init(void)
{
    Vision_Servo_Test_Init();
}

void Ball_Control_Reset(void)
{
    Vision_Servo_Test_Reset();
}

void Ball_Control_Update(float target_cm)
{
    Vision_Servo_Test_SetTarget(clamp_target(target_cm));
    Vision_Servo_Test_Update();
}

void Ball_Control_Stop(void)
{
    Vision_Servo_Test_Stop();
}

float Ball_Control_GetTarget(void)
{
    return Vision_Servo_Test_GetTarget();
}

float Ball_Control_GetPosition(void)
{
    return Vision_Servo_Test_GetPosition();
}

float Ball_Control_GetVelocity(void)
{
    return Vision_Servo_Test_GetVelocity();
}

uint8_t Ball_Control_IsUsable(void)
{
    return (uint8_t)((Camera_Vision_IsUsable() != 0U) &&
        (Vision_Servo_Test_HasPosition() != 0U));
}

uint8_t Ball_Control_IsStable(float tolerance_cm)
{
    float err;

    if ((tolerance_cm < 0.0f) || (Ball_Control_IsUsable() == 0U))
    {
        return 0U;
    }

    err = Ball_Control_GetTarget() - Ball_Control_GetPosition();
    return (uint8_t)((absf(err) <= tolerance_cm) &&
        (absf(Ball_Control_GetVelocity()) <= BALL_STABLE_VEL_CM_S));
}
