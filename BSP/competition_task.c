#include "competition_task.h"
#include "app_motor.h"
#include "app_motor_usart.h"
#include "ball_control.h"
#include "bsp_camera_usart.h"
#include "car_with_ball.h"
#include "Four_linewalking.h"

#define TASK_LINE_MAX_MS       (20000U) /* 纯循迹一圈的最长时间。 */
#define TASK_BALL_MAX_MS        (5000U) /* 钢珠往返控制的最长时间。 */
#define TASK_AB_MAX_MS          (8000U) /* A 到 B 同时控球的最长时间。 */
#define TASK_LAP_MAX_MS        (30000U) /* 带球一圈任务的最长时间。 */

#define TASK_LINE_SPEED           (316) /* 纯循迹任务的目标速度。 */
#define TASK_AB_SPEED             (240) /* A 到 B 带球任务的目标速度。 */
#define TASK_LAP_SPEED            (225) /* 带球一圈任务的目标速度。 */

#define TASK_BALL_POS_CM          (5.0f) /* 钢珠往返任务的首个目标。 */
#define TASK_BALL_CENTER_CM       (0.0f) /* 行驶稳定任务的钢珠目标。 */
#define TASK_BALL_TOL_CM          (0.8f) /* 往返目标的稳定位置容差。 */
#define TASK_BALL_STABLE_FRAMES       (3U) /* 连续满足稳定条件的有效视觉帧数。 */
#define TASK_AB_DISTANCE_MM       (1500.0f) /* A 到 B 的名义里程，需按实车标定。 */
#define TASK_LAP_DISTANCE_MM      (5000.0f) /* 回到 A 前要求的最小整圈里程。 */
#define MARK_ENTER_MS                 (20U) /* 全黑持续达到该时间才确认停止线。 */
#define MARK_LEAVE_MS                 (50U) /* 离开全黑达到该时间后重新允许触发。 */
#define MARK_MIN_GAP_MS              (700U) /* 相邻停止线事件的最小时间间隔。 */
#define MARK_MIN_GAP_MM            (300.0f) /* 起点附近重复检测的最小距离间隔。 */

static CompetitionTask s_task = COMP_TASK_LINE_LAP; /* 当前选择的竞赛任务。 */
static CompetitionStatus s_status = COMP_STATUS_READY; /* 当前任务状态。 */
static uint32_t s_start_ms = 0U;   /* 当前任务开始时刻。 */
static uint32_t s_pause_ms = 0U;   /* 最近一次暂停的开始时刻。 */
static uint32_t s_paused_ms = 0U;  /* 当前任务累计暂停时长。 */
static uint32_t s_end_ms = 0U;     /* 完成或超时时的时刻。 */
static uint8_t s_waypoint = 0U;    /* 已通过的途经点数量。 */
static uint8_t s_ball_phase = 0U;  /* 钢珠往返任务阶段。 */
static uint8_t s_ball_stable_n = 0U; /* 当前阶段连续稳定的视觉帧数。 */
static uint32_t s_ball_frame = 0U; /* 上次参与稳定判定的有效视觉帧。 */
static float s_ball_pos_cm = 0.0f; /* 指定位置任务的目标，单位 cm。 */
static float s_ball_target = 0.0f; /* 当前实际交给控球器的目标。 */
static float s_odom_mm = 0.0f;     /* 本次任务累计行驶距离，单位 mm。 */
static float s_mark_odom_mm = 0.0f; /* 上次停止线事件时的累计距离。 */
static uint32_t s_speed_n = 0U;    /* 上次处理的速度帧计数。 */
static uint32_t s_speed_ms = 0U;   /* 上次速度帧参与积分的时刻。 */
static uint32_t s_mark_ms = 0U;    /* 上次停止线事件时刻。 */
static uint32_t s_mark_enter_ms = 0U; /* 本次进入全黑状态的时刻。 */
static uint32_t s_mark_leave_ms = 0U; /* 本次离开全黑状态的时刻。 */
static uint8_t s_mark_on = 0U;     /* 当前是否处于确认后的停止线上。 */
static uint8_t s_mark_armed = 0U;  /* 离开起点后是否允许识别下一停止线。 */
static uint8_t s_vision_hold = 0U; /* 行驶任务是否因视觉失效而临时停车。 */

static uint32_t now_ms(void)
{
    return Camera_Vision_GetTimeMs();
}

static uint32_t timeout_ms(CompetitionTask task)
{
    switch (task)
    {
        case COMP_TASK_LINE_LAP:
            return TASK_LINE_MAX_MS;
        case COMP_TASK_BALL_ROUND_TRIP:
            return TASK_BALL_MAX_MS;
        case COMP_TASK_AB_WITH_BALL:
            return TASK_AB_MAX_MS;
        case COMP_TASK_LAP_WITH_BALL:
        case COMP_TASK_LAP_WITH_POSITION:
            return TASK_LAP_MAX_MS;
        default:
            return 0U;
    }
}

static void stop_car(void)
{
    Motion_Car_Control(0, 0, 0);
}

static float absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

static void reset_route_state(void)
{
    uint32_t now = now_ms();

    s_waypoint = 0U;
    s_odom_mm = 0.0f;
    s_mark_odom_mm = 0.0f;
    s_speed_n = Motor_GetSpeedUpdateCount();
    s_speed_ms = now;
    s_mark_ms = now;
    s_mark_enter_ms = 0U;
    s_mark_leave_ms = now;
    s_mark_on = 0U;
    s_mark_armed = 0U;
}

static void update_odometry(void)
{
    uint32_t count = Motor_GetSpeedUpdateCount();
    uint32_t now;
    uint32_t dt;
    float speed;

    if (count == s_speed_n)
    {
        return;
    }
    now = now_ms();
    dt = (uint32_t)(now - s_speed_ms);
    s_speed_n = count;
    s_speed_ms = now;
    if ((dt == 0U) || (dt > 500U))
    {
        return;
    }

    speed = (g_Speed[0] + g_Speed[1] + g_Speed[2] + g_Speed[3]) * 0.25f;
    s_odom_mm += absf(speed) * (float)dt / 1000.0f;
}

static uint8_t update_stop_mark(void)
{
    uint32_t now = now_ms();
    uint8_t full_black = (uint8_t)(Four_Line_GetBlackCount() == 4U);

    if (full_black != 0U)
    {
        s_mark_leave_ms = 0U;
        if (s_mark_enter_ms == 0U)
        {
            s_mark_enter_ms = now;
        }
        if ((s_mark_on == 0U) &&
            ((uint32_t)(now - s_mark_enter_ms) >= MARK_ENTER_MS))
        {
            s_mark_on = 1U;
            if ((s_mark_armed != 0U) &&
                ((uint32_t)(now - s_mark_ms) >= MARK_MIN_GAP_MS) &&
                ((s_odom_mm - s_mark_odom_mm) >= MARK_MIN_GAP_MM))
            {
                s_mark_armed = 0U;
                s_mark_ms = now;
                s_mark_odom_mm = s_odom_mm;
                return 1U;
            }
        }
    }
    else
    {
        s_mark_enter_ms = 0U;
        if (s_mark_leave_ms == 0U)
        {
            s_mark_leave_ms = now;
        }
        if ((uint32_t)(now - s_mark_leave_ms) >= MARK_LEAVE_MS)
        {
            s_mark_on = 0U;
            s_mark_armed = 1U;
        }
    }
    return 0U;
}

static void enter_done(void)
{
    s_end_ms = now_ms();
    s_status = COMP_STATUS_DONE;
    if ((s_task == COMP_TASK_AB_WITH_BALL) ||
        (s_task == COMP_TASK_LAP_WITH_BALL) ||
        (s_task == COMP_TASK_LAP_WITH_POSITION))
    {
        Car_WithBall_StopCar(s_ball_target);
    }
    else
    {
        stop_car();
        if (s_task == COMP_TASK_LINE_LAP)
        {
            Ball_Control_Stop();
        }
    }
}

static void enter_timeout(void)
{
    s_end_ms = now_ms();
    s_status = COMP_STATUS_TIMEOUT;
    if ((s_task == COMP_TASK_AB_WITH_BALL) ||
        (s_task == COMP_TASK_LAP_WITH_BALL) ||
        (s_task == COMP_TASK_LAP_WITH_POSITION))
    {
        Car_WithBall_StopCar(s_ball_target);
    }
    else
    {
        stop_car();
        if (s_task == COMP_TASK_LINE_LAP)
        {
            Ball_Control_Stop();
        }
    }
}

static uint8_t update_ball_round_trip(void)
{
    uint32_t frame;

    s_ball_target = (s_ball_phase == 0U) ?
        TASK_BALL_POS_CM : -TASK_BALL_POS_CM;
    Ball_Control_Update(s_ball_target);

    frame = vision.valid_n;
    if (frame == s_ball_frame)
    {
        return 0U;
    }
    s_ball_frame = frame;

    if (Ball_Control_IsStable(TASK_BALL_TOL_CM) != 0U)
    {
        if (s_ball_stable_n < TASK_BALL_STABLE_FRAMES)
        {
            s_ball_stable_n++;
        }
    }
    else
    {
        s_ball_stable_n = 0U;
    }

    if (s_ball_stable_n < TASK_BALL_STABLE_FRAMES)
    {
        return 0U;
    }

    s_ball_stable_n = 0U;
    if (s_ball_phase == 0U)
    {
        s_ball_phase = 1U;
        return 0U;
    }
    return 1U;
}

void Competition_Task_Init(void)
{
    s_task = COMP_TASK_LINE_LAP;
    s_status = COMP_STATUS_READY;
    s_start_ms = 0U;
    s_pause_ms = 0U;
    s_paused_ms = 0U;
    s_end_ms = 0U;
    s_waypoint = 0U;
    s_ball_phase = 0U;
    s_ball_stable_n = 0U;
    s_ball_frame = vision.valid_n;
    s_ball_pos_cm = 0.0f;
    s_ball_target = 0.0f;
    reset_route_state();
    stop_car();
    Ball_Control_Stop();
    Four_Line_Init();
}

void Competition_Task_Select(CompetitionTask task)
{
    if ((task >= COMP_TASK_COUNT) ||
        (s_status == COMP_STATUS_RUNNING) ||
        (s_status == COMP_STATUS_PAUSED))
    {
        return;
    }
    s_task = task;
    s_status = COMP_STATUS_READY;
    stop_car();
    Ball_Control_Stop();
}

void Competition_Task_SelectPrevious(void)
{
    CompetitionTask task;

    if ((s_status == COMP_STATUS_RUNNING) || (s_status == COMP_STATUS_PAUSED))
    {
        return;
    }
    task = (s_task == COMP_TASK_LINE_LAP) ?
        (CompetitionTask)(COMP_TASK_COUNT - 1) : (CompetitionTask)(s_task - 1);
    Competition_Task_Select(task);
}

void Competition_Task_SelectNext(void)
{
    CompetitionTask task;

    if ((s_status == COMP_STATUS_RUNNING) || (s_status == COMP_STATUS_PAUSED))
    {
        return;
    }
    task = (CompetitionTask)((s_task + 1) % COMP_TASK_COUNT);
    Competition_Task_Select(task);
}

void Competition_Task_Start(void)
{
    s_start_ms = now_ms();
    s_pause_ms = 0U;
    s_paused_ms = 0U;
    s_end_ms = 0U;
    reset_route_state();
    s_ball_phase = 0U;
    s_ball_stable_n = 0U;
    s_ball_frame = vision.valid_n;
    if (s_task == COMP_TASK_BALL_ROUND_TRIP)
    {
        s_ball_target = TASK_BALL_POS_CM;
    }
    else if (s_task == COMP_TASK_LAP_WITH_POSITION)
    {
        s_ball_target = s_ball_pos_cm;
    }
    else
    {
        s_ball_target = TASK_BALL_CENTER_CM;
    }
    s_status = COMP_STATUS_RUNNING;
    s_vision_hold = 0U;
    Four_Line_Reset();
    Ball_Control_Reset();
    Car_WithBall_Reset(s_start_ms);
    if (s_task == COMP_TASK_BALL_ROUND_TRIP)
    {
        stop_car();
    }
}

void Competition_Task_Pause(void)
{
    if (s_status != COMP_STATUS_RUNNING)
    {
        return;
    }
    s_pause_ms = now_ms();
    s_status = COMP_STATUS_PAUSED;
    if ((s_task == COMP_TASK_AB_WITH_BALL) ||
        (s_task == COMP_TASK_LAP_WITH_BALL) ||
        (s_task == COMP_TASK_LAP_WITH_POSITION))
    {
        Car_WithBall_StopCar(s_ball_target);
    }
    else
    {
        stop_car();
    }
}

void Competition_Task_Resume(void)
{
    uint32_t now;
    uint32_t elapsed;

    if (s_status != COMP_STATUS_PAUSED)
    {
        return;
    }
    now = now_ms();
    s_paused_ms += (uint32_t)(now - s_pause_ms);
    s_pause_ms = 0U;
    s_status = COMP_STATUS_RUNNING;
    elapsed = Competition_Task_GetElapsedMs();
    if ((s_task == COMP_TASK_AB_WITH_BALL) ||
        (s_task == COMP_TASK_LAP_WITH_BALL) ||
        (s_task == COMP_TASK_LAP_WITH_POSITION))
    {
        Car_WithBall_Reset((uint32_t)(now - elapsed));
    }
}

void Competition_Task_Stop(void)
{
    s_status = COMP_STATUS_READY;
    s_start_ms = 0U;
    s_pause_ms = 0U;
    s_paused_ms = 0U;
    s_end_ms = 0U;
    s_waypoint = 0U;
    s_ball_phase = 0U;
    s_ball_stable_n = 0U;
    s_vision_hold = 0U;
    stop_car();
    Ball_Control_Stop();
}

void Competition_Task_SetBallPosition(float target_cm)
{
    if (target_cm < -12.0f)
    {
        target_cm = -12.0f;
    }
    if (target_cm > 12.0f)
    {
        target_cm = 12.0f;
    }
    s_ball_pos_cm = target_cm;
}

void Competition_Task_Update(void)
{
    uint32_t limit;

    Deal_data_real();
    update_odometry();

    if (s_status == COMP_STATUS_PAUSED)
    {
        if (s_task != COMP_TASK_LINE_LAP)
        {
            Ball_Control_Update(s_ball_target);
        }
        return;
    }
    if ((s_status == COMP_STATUS_DONE) || (s_status == COMP_STATUS_TIMEOUT))
    {
        if (s_task != COMP_TASK_LINE_LAP)
        {
            Ball_Control_Update(s_ball_target);
        }
        return;
    }
    if (s_status != COMP_STATUS_RUNNING)
    {
        return;
    }

    limit = timeout_ms(s_task);
    if ((limit > 0U) && (Competition_Task_GetElapsedMs() >= limit))
    {
        enter_timeout();
        return;
    }

    if ((s_task == COMP_TASK_AB_WITH_BALL) ||
        (s_task == COMP_TASK_LAP_WITH_BALL) ||
        (s_task == COMP_TASK_LAP_WITH_POSITION))
    {
        if (Camera_Vision_IsUsable() == 0U)
        {
            if (s_vision_hold == 0U)
            {
                s_vision_hold = 1U;
                Car_WithBall_StopCar(s_ball_target);
            }
            Ball_Control_Update(s_ball_target);
            return;
        }
        if (s_vision_hold != 0U)
        {
            s_vision_hold = 0U;
            Car_WithBall_Reset((uint32_t)(now_ms() - Competition_Task_GetElapsedMs()));
        }
    }

    switch (s_task)
    {
        case COMP_TASK_LINE_LAP:
            Four_Line_SetSpeed(TASK_LINE_SPEED);
            Four_Line_Update();
            if ((update_stop_mark() != 0U) &&
                (s_odom_mm >= TASK_LAP_DISTANCE_MM))
            {
                s_waypoint++;
                enter_done();
            }
            break;

        case COMP_TASK_BALL_ROUND_TRIP:
            if (update_ball_round_trip() != 0U)
            {
                enter_done();
            }
            break;

        case COMP_TASK_AB_WITH_BALL:
            s_ball_target = TASK_BALL_CENTER_CM;
            Car_WithBall_Update(TASK_AB_SPEED, s_ball_target);
            (void)update_stop_mark();
            if (s_odom_mm >= TASK_AB_DISTANCE_MM)
            {
                s_waypoint++;
                enter_done();
            }
            break;

        case COMP_TASK_LAP_WITH_BALL:
            s_ball_target = TASK_BALL_CENTER_CM;
            Car_WithBall_Update(TASK_LAP_SPEED, s_ball_target);
            if ((update_stop_mark() != 0U) &&
                (s_odom_mm >= TASK_LAP_DISTANCE_MM))
            {
                s_waypoint++;
                enter_done();
            }
            break;

        case COMP_TASK_LAP_WITH_POSITION:
            s_ball_target = s_ball_pos_cm;
            Car_WithBall_Update(TASK_LAP_SPEED, s_ball_target);
            if ((update_stop_mark() != 0U) &&
                (s_odom_mm >= TASK_LAP_DISTANCE_MM))
            {
                s_waypoint++;
                enter_done();
            }
            break;

        default:
            Competition_Task_Stop();
            break;
    }
}

CompetitionTask Competition_Task_GetSelected(void)
{
    return s_task;
}

CompetitionStatus Competition_Task_GetStatus(void)
{
    return s_status;
}

uint32_t Competition_Task_GetElapsedMs(void)
{
    uint32_t end;

    if (s_start_ms == 0U)
    {
        return 0U;
    }
    if (s_status == COMP_STATUS_PAUSED)
    {
        end = s_pause_ms;
    }
    else if ((s_status == COMP_STATUS_DONE) ||
             (s_status == COMP_STATUS_TIMEOUT))
    {
        end = s_end_ms;
    }
    else
    {
        end = now_ms();
    }
    return (uint32_t)(end - s_start_ms - s_paused_ms);
}

uint8_t Competition_Task_GetWaypointCount(void)
{
    return s_waypoint;
}

uint8_t Competition_Task_GetBallPhase(void)
{
    return s_ball_phase;
}

float Competition_Task_GetBallTarget(void)
{
    return s_ball_target;
}
