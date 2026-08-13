#include "vision_servo_test.h"
#include "bsp_camera_usart.h"
#include "servo.h"
#include <stdint.h>
#include <stdio.h>

static uint32_t s_frame = 0U;       /* 上次已处理的有效视觉帧计数。 */
static uint32_t s_ms = 0U;          /* 上一有效视觉帧时间戳，单位 ms。 */
static float s_pos = 0.0f;          /* 上一有效帧的钢珠位置，单位 cm。 */
static float s_vel = 0.0f;          /* 低通后的钢珠速度，单位 cm/s。 */
static float s_push = 0.0f;         /* 静止时克服摩擦的临时推力角，单位度。 */
static float s_ang = SV_ANG0;       /* 上次下发的浮点舵机角度。 */
static float s_target = SV_POS_REF;  /* 当前钢珠目标位置，单位 cm。 */
static uint32_t s_still_ms = 0U;    /* 连续低速且未到位的累计时间，单位 ms。 */
static uint8_t s_has_pos = 0U;      /* 是否已有位置可用于计算速度。 */
static uint8_t s_lost = 0U;         /* 当前是否处于视觉短暂失效保持状态。 */
static uint32_t s_lost_ms = 0U;     /* 视觉失效开始时刻，单位 ms。 */

/* 将浮点控制量限制在给定范围内。 */
static float clampf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

/* 返回浮点数绝对值，避免引入额外数学库依赖。 */
static float absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

/* 按角度限幅、限速后更新 PWM。 */
static void set_ang(float req)
{
    float ang = clampf(req, SV_ANG_MIN, SV_ANG_MAX); /* 限幅后的舵机角度。 */

    if (ang > s_ang)
    {
        ang = clampf(ang, s_ang, s_ang + SV_STEP_UP);
    }
    else
    {
        ang = clampf(ang, s_ang - SV_STEP_DN, s_ang);
    }

    if (absf(ang - s_ang) >= 0.5f)
    {
        Servo_SetAngle((uint16_t)(ang + 0.5f));
        s_ang = ang;
    }
}

/* 临时推力释放时直接更新角度，避免残余倾角继续加速钢珠。 */
static void set_ang_now(float req)
{
    float ang = clampf(req, SV_ANG_MIN, SV_ANG_MAX); /* 立即下发的舵机角度。 */

    Servo_SetAngle((uint16_t)(ang + 0.5f));
    s_ang = ang;
}

/* 清除控制器历史状态，并让摆杆立即回到平衡基准。 */
void Vision_Servo_Test_Reset(void)
{
    s_frame = vision.valid_n;
    s_ms = Camera_Vision_GetTimeMs();
    s_pos = 0.0f;
    s_vel = 0.0f;
    s_push = 0.0f;
    s_still_ms = 0U;
    s_ang = SV_ANG0;
    s_has_pos = 0U;
    s_lost = 0U;
    s_lost_ms = 0U;
    Servo_SetAngle((uint16_t)SV_ANG0);
}

/* 初始化 PWM、默认目标和控制状态。 */
void Vision_Servo_Test_Init(void)
{
    Servo_Init();
    s_target = SV_POS_REF;
    Vision_Servo_Test_Reset();
}

/* 更新业务层给出的目标位置，目标换向时释放旧的静摩擦推力。 */
void Vision_Servo_Test_SetTarget(float target_cm)
{
    if (absf(target_cm - s_target) > 0.001f)
    {
        s_push = 0.0f;
        s_still_ms = 0U;
    }
    s_target = target_cm;
}

/* 主循环调用：执行位置、静摩擦补偿和速度制动控制。 */
void Vision_Servo_Test_Update(void)
{
    uint32_t now;       /* 当前时间戳，单位 ms。 */
    uint32_t frame;     /* 当前有效视觉帧计数。 */
    uint32_t dt = 0U;   /* 相邻有效帧时间间隔，单位 ms。 */
    float pos;          /* 当前钢珠位置，单位 cm。 */
    float vel;          /* 当前帧差分速度，单位 cm/s。 */
    float err;          /* 目标位置与当前位置的误差，单位 cm。 */
    float ctl;          /* 死区处理后的比例控制误差，单位 cm。 */
    float dt_s = 0.0f;  /* 相邻有效帧时间间隔，单位 s。 */
    float req;          /* 本帧请求的舵机角度，单位度。 */
    float out;          /* 控制器输出的舵机角度增量，单位度。 */
    uint8_t drop_push = 0U; /* 本帧是否需要立即释放临时推力。 */

    now = Camera_Vision_GetTimeMs();
    if (Camera_Vision_IsUsable() == 0U)
    {
        /* 单次漏检时保持末角度；持续失效才回水平并清状态。 */
        if (s_lost == 0U)
        {
            s_lost = 1U;
            s_lost_ms = now;
        }
        if ((uint32_t)(now - s_lost_ms) < SV_LOST_HOLD_MS)
        {
            return;
        }

        s_has_pos = 0U;
        s_vel = 0.0f;
        s_push = 0.0f;
        s_still_ms = 0U;
        s_ms = now;
        set_ang(SV_ANG0);
        return;
    }
    s_lost = 0U;

    frame = vision.valid_n;
    if (frame == s_frame)
    {
        /* 同一帧不重复计算和驱动舵机。 */
        return;
    }
    s_frame = frame;

    pos = ((float)vision.x - SV_X0) * SV_CM_PX;
    if (s_has_pos != 0U)
    {
        dt = (uint32_t)(vision.valid_ms - s_ms);
        if ((dt > 0U) && (dt <= SV_VEL_DT_MAX_MS))
        {
            dt_s = (float)dt / 1000.0f;
            vel = (pos - s_pos) / dt_s;
            s_vel = SV_VEL_ALPHA * vel + (1.0f - SV_VEL_ALPHA) * s_vel;
        }
        else
        {
            s_vel = 0.0f;
        }
    }
    else
    {
        s_has_pos = 1U;
        s_vel = 0.0f;
    }

    s_ms = vision.valid_ms;
    s_pos = pos;
    err = s_target - pos;
    ctl = (absf(err) <= SV_ERR_TOL) ? 0.0f : err;

    /* 先确认钢珠持续静止，再建立临时推力；运动或到位后立即释放。 */
    if ((absf(err) <= SV_ERR_TOL) ||
        (absf(s_vel) > SV_PUSH_VEL_MAX))
    {
        drop_push = (uint8_t)(absf(s_push) > 0.0f);
        s_push = 0.0f;
        s_still_ms = 0U;
    }
    else if (dt_s > 0.0f)
    {
        if (s_push * err < 0.0f)
        {
            s_push = 0.0f;
            s_still_ms = 0U;
        }

        if (s_still_ms < SV_PUSH_WAIT_MS)
        {
            s_still_ms += dt;
        }
        else if (err > 0.0f)
        {
            s_push = clampf(s_push + SV_PUSH_RATE * dt_s,
                            0.0f,
                            SV_PUSH_MAX);
        }
        else
        {
            s_push = clampf(s_push - SV_PUSH_RATE * dt_s,
                            -SV_PUSH_MAX,
                            0.0f);
        }
    }

    out = SV_KP * ctl + s_push - SV_KD * s_vel;
    req = SV_ANG0 + SV_DIR * out;
    if (drop_push != 0U)
    {
        set_ang_now(req);
    }
    else
    {
        set_ang(req);
    }

#if SV_DBG
    printf("[SERVO] ref=%.2f x=%.2f pos=%.2f vel=%.2f err=%.2f base=%.2f still=%lu push=%.2f req=%.1f angle=%u\r\n",
        (double)s_target,
        (double)vision.x,
        (double)pos,
        (double)s_vel,
        (double)err,
        (double)SV_ANG0,
        (unsigned long)s_still_ms,
        (double)s_push,
        (double)req,
        (unsigned int)(uint16_t)(s_ang + 0.5f));
#endif
}

/* 停止闭环并让摆杆立即回到平衡基准。 */
void Vision_Servo_Test_Stop(void)
{
    s_frame = vision.valid_n;
    s_ms = Camera_Vision_GetTimeMs();
    s_vel = 0.0f;
    s_push = 0.0f;
    s_still_ms = 0U;
    s_has_pos = 0U;
    s_lost = 0U;
    s_lost_ms = 0U;
    set_ang_now(SV_ANG0);
}

float Vision_Servo_Test_GetTarget(void)
{
    return s_target;
}

float Vision_Servo_Test_GetPosition(void)
{
    return s_pos;
}

float Vision_Servo_Test_GetVelocity(void)
{
    return s_vel;
}

uint8_t Vision_Servo_Test_HasPosition(void)
{
    return s_has_pos;
}
