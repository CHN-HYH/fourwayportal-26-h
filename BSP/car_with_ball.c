#include "car_with_ball.h"
#include "ball_control.h"
#include "bsp_camera_usart.h"
#include "Four_linewalking.h"

#define CAR_RAMP_MS            (3000U) /* 底盘从低速爬升至目标速度的时间。 */
#define CAR_RAMP_MIN            (0.15f) /* 起步时保留的最小速度比例。 */
#define BALL_FWD_COMP_CM       (-0.80f) /* 前进时目标位置的基础补偿。 */
#define BALL_TURN_COMP_CM      (-0.80f) /* 转弯时随转向量变化的补偿。 */
#define BALL_START_COMP_CM      (0.00f) /* 起步附加补偿，待整车实测。 */
#define BALL_STOP_COMP_CM       (0.00f) /* 停车附加补偿，待整车实测。 */
#define LINE_TURN_MAX           (6000.0f) /* 转向补偿归一化使用的参考上限。 */

static uint32_t s_start_ms = 0U; /* 当前组合任务的开始时刻。 */
static float s_target = 0.0f;    /* 补偿后实际交给控球器的目标。 */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void Car_WithBall_Reset(uint32_t start_ms)
{
    s_start_ms = start_ms;
    s_target = 0.0f;
    Four_Line_Reset();
}

void Car_WithBall_Update(int16_t speed, float target_cm)
{
    uint32_t elapsed;
    float ramp = 1.0f;
    float compensated = target_cm;
    int16_t actual_speed = speed;
    int16_t turn;

    elapsed = (uint32_t)(Camera_Vision_GetTimeMs() - s_start_ms);
    if (elapsed < CAR_RAMP_MS)
    {
        ramp = (float)elapsed / (float)CAR_RAMP_MS;
        ramp = clampf(ramp, CAR_RAMP_MIN, 1.0f);
        actual_speed = (int16_t)((float)speed * ramp);
        compensated += BALL_START_COMP_CM;
    }

    Four_Line_SetSpeed(actual_speed);
    Four_Line_Update();
    turn = Four_Line_GetTurn();

    compensated += BALL_FWD_COMP_CM;
    compensated += BALL_TURN_COMP_CM *
        clampf((float)turn / LINE_TURN_MAX, -1.0f, 1.0f);
    s_target = clampf(compensated, -12.0f, 12.0f);
    Ball_Control_Update(s_target);
}

void Car_WithBall_StopCar(float target_cm)
{
    Four_Line_Stop();
    s_target = clampf(target_cm + BALL_STOP_COMP_CM, -12.0f, 12.0f);
    Ball_Control_Update(s_target);
}

float Car_WithBall_GetCompensatedTarget(void)
{
    return s_target;
}
