#include "vision_servo_test.h"
#include "bsp_camera_usart.h"
#include "servo.h"
#include <stdint.h>
#include <stdio.h>

static uint32_t s_frame = 0U;    /* 上次已处理的有效视觉帧计数。 */
static uint32_t s_ms = 0U;       /* 上一有效帧的时间戳，单位 ms。 */
static float s_pos = 0.0f;       /* 上一有效帧的钢珠位置，单位 cm。 */
static float s_vel = 0.0f;       /* 滤波后的钢珠速度，单位 cm/s。 */
static float s_i = 0.0f;         /* 仅用于消除静摩擦的小误差积分量，单位度。 */
static uint16_t s_ang = 0xFFFFU; /* 上次下发的整数舵机角度。 */
static uint8_t s_has_pos = 0U;   /* 是否已有一帧位置可用于计算速度。 */
static uint8_t s_lost = 0U;      /* 当前是否处于视觉短暂失效保持状态。 */
static uint32_t s_lost_ms = 0U;  /* 视觉失效开始时刻，单位 ms。 */

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
    if (v < 0.0f)
    {
        return -v;
    }
    return v;
}

/* 限幅、量化并仅在角度变化时更新 PWM。 */
static void set_ang(float req)
{
    float lim;      /* 限幅后的浮点角度。 */
    uint16_t ang;   /* 下发给舵机驱动的整数角度。 */

    lim = clampf(req, SV_ANG_MIN, SV_ANG_MAX);

    /* 增角限速以免突然加速；减角限速更大，使接近或越过目标时能及时制动。 */
    if (s_ang != 0xFFFFU)
    {
        if (lim > (float)s_ang)
        {
            lim = clampf(lim, (float)s_ang, (float)s_ang + SV_STEP_UP);
        }
        else
        {
            lim = clampf(lim, (float)s_ang - SV_STEP_DN, (float)s_ang);
        }
    }
    ang = (uint16_t)(lim + 0.5f);

    if (ang != s_ang)
    {
        Servo_SetAngle(ang);
        s_ang = ang;
    }
}

/* 初始化 PWM、控制状态和摆杆平衡基准。 */
void Vision_Servo_Test_Init(void)
{
    Servo_Init();
    s_frame = vision.valid_n;
    s_ms = Camera_Vision_GetTimeMs();
    s_pos = 0.0f;
    s_vel = 0.0f;
    s_i = 0.0f;
    s_has_pos = 0U;
    s_lost = 0U;
    s_lost_ms = 0U;
    set_ang(SV_ANG0);
}

/* 主循环调用：使用带抗积分饱和的 PID 闭环让钢珠稳定在固定目标位置。 */
void Vision_Servo_Test_Update(void)
{
    uint32_t now;       /* 当前时间戳，单位 ms。 */
    uint32_t dt;        /* 相邻有效帧的时间间隔，单位 ms。 */
    uint32_t frame;     /* 当前有效视觉帧计数。 */
    float pos;          /* 当前钢珠位置，单位 cm。 */
    float vel;          /* 原始速度，单位 cm/s。 */
    float err;          /* 目标位置与当前位置的误差，单位 cm。 */
    float e_ctl;        /* 进入允许误差范围后的控制误差，单位 cm。 */
    float dt_s;         /* 相邻帧时间间隔，单位 s。 */
    float out;          /* PID 计算得到的角度增量。 */
    float ang;          /* 加水平基准角后的目标舵机角度。 */

    now = Camera_Vision_GetTimeMs();
    if (Camera_Vision_IsUsable() == 0U)
    {
        /* 单次漏检常由残影引起，短时保持原角度，避免突然回水平导致钢珠滚远。 */
        if (s_lost == 0U)
        {
            s_lost = 1U;
            s_lost_ms = now;
        }
        if ((uint32_t)(now - s_lost_ms) < SV_LOST_HOLD_MS)
        {
            return;
        }

        /* 连续失效超时后才回到水平安全角，并清空本轮控制状态。 */
        s_has_pos = 0U;
        s_vel = 0.0f;
        s_i = 0.0f;
        set_ang(SV_ANG0);
        return;
    }
    s_lost = 0U;

    frame = vision.valid_n;
    if (frame == s_frame)
    {
        /* 同一帧不重复计算，舵机保持上一次目标角度。 */
        return;
    }
    s_frame = frame;

    pos = ((float)vision.x - SV_X0) * SV_CM_PX;
    dt_s = 0.0f;

    if (s_has_pos != 0U)
    {
        dt = (uint32_t)(now - s_ms);
        if ((dt > 0U) && (dt <= 500U))
        {
            /* 相邻帧差分后低通滤波，降低视觉坐标抖动对制动的影响。 */
            dt_s = (float)dt / 1000.0f;
            vel = (pos - s_pos) * (1000.0f / (float)dt);
            s_vel = SV_VEL_A * vel + (1.0f - SV_VEL_A) * s_vel;
        }
        else
        {
            s_vel = 0.0f;
        }
    }
    else
    {
        /* 首帧没有速度信息，只建立位置基线。 */
        s_has_pos = 1U;
        s_vel = 0.0f;
    }

    s_ms = now;
    s_pos = pos;

    err = SV_POS_REF - pos;

    /* 进入 ±0.20 cm 后停止位置推动，避免视觉像素抖动反复驱动舵机。 */
    if (absf(err) <= SV_ERR_TOL)
    {
        e_ctl = 0.0f;
    }
    else
    {
        e_ctl = err;
    }

    /* 越过目标时立即清空反方向的积分，避免旧积分抵消舵机的减角制动。 */
    if (((s_i > 0.0f) && (err < -SV_ERR_TOL)) ||
        ((s_i < 0.0f) && (err > SV_ERR_TOL)))
    {
        s_i = 0.0f;
    }

    /* 只在接近目标、速度很低且确实卡住时积分，既克服静摩擦，也避免远距离积分饱和。 */
    if ((dt_s > 0.0f) &&
        (absf(err) > SV_ERR_TOL) &&
        (absf(err) <= SV_I_ERR_MAX) &&
        (absf(s_vel) <= SV_I_VEL_MAX))
    {
        s_i = clampf(s_i + SV_KI * err * dt_s, -SV_I_MAX, SV_I_MAX);
    }

    /* 以水平角为基准：目标在左侧时减角，目标在右侧时增角；速度项负责反向制动。 */
    out = SV_KP * e_ctl + s_i - SV_KD * s_vel;
    ang = SV_ANG0 + SV_DIR * out;
    set_ang(ang);

#if SV_DBG
    printf("[SERVO] ref=%.1f pos=%.2f vel=%.2f err=%.2f i=%.2f req=%.1f angle=%u\r\n",
        (double)SV_POS_REF,
        (double)pos,
        (double)s_vel,
        (double)err,
        (double)s_i,
        (double)ang,
        (unsigned int)s_ang);
#endif
}
