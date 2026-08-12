#include "vision_servo_test.h"
#include "bsp_camera_usart.h"
#include "servo.h"
#include <stdint.h>
#include <stdio.h>

static uint32_t s_frame = 0U;    /* 上次已处理的有效视觉帧计数。 */
static uint32_t s_ms = 0U;       /* 上一有效帧的时间戳，单位 ms。 */
static float s_pos = 0.0f;       /* 上一有效帧的钢珠位置，单位 cm。 */
static float s_vel = 0.0f;       /* 滤波后的钢珠速度，单位 cm/s。 */
static uint16_t s_ang = 0xFFFFU; /* 上次下发的整数舵机角度。 */
static uint8_t s_has_pos = 0U;   /* 是否已有一帧位置可用于计算速度。 */

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

/* 限幅、量化并仅在角度变化时更新 PWM。 */
static void set_ang(float req)
{
    float lim;      /* 限幅后的浮点角度。 */
    uint16_t ang;   /* 下发给舵机驱动的整数角度。 */

    lim = clampf(req, SV_ANG_MIN, SV_ANG_MAX);

    /* 限制每帧的角度变化，避免一次大倾角使钢珠突然加速。 */
    if (s_ang != 0xFFFFU)
    {
        lim = clampf(lim,
                     (float)s_ang - SV_STEP_MAX,
                     (float)s_ang + SV_STEP_MAX);
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
    s_has_pos = 0U;
    set_ang(SV_ANG0);
}

/* 主循环调用：使用带保持角补偿的 PD 闭环让钢珠稳定在固定的 +12 cm 位置。 */
void Vision_Servo_Test_Update(void)
{
    uint32_t now;       /* 当前时间戳，单位 ms。 */
    uint32_t dt;        /* 相邻有效帧的时间间隔，单位 ms。 */
    uint32_t frame;     /* 当前有效视觉帧计数。 */
    float pos;          /* 当前钢珠位置，单位 cm。 */
    float vel;          /* 原始速度，单位 cm/s。 */
    float err;          /* 目标位置与当前位置的误差，单位 cm。 */
    float out;          /* PD 计算得到的角度增量。 */
    float ang;          /* 加保持角后的目标舵机角度。 */

    if (Camera_Vision_IsUsable() == 0U)
    {
        /* 无目标或通信超时时停止使用旧数据，并让摆杆回到平衡角。 */
        s_has_pos = 0U;
        s_vel = 0.0f;
        set_ang(SV_ANG0);
        return;
    }

    frame = vision.valid_n;
    if (frame == s_frame)
    {
        /* 同一帧不重复计算，舵机保持上一次目标角度。 */
        return;
    }
    s_frame = frame;

    now = Camera_Vision_GetTimeMs();
    pos = ((float)vision.x - SV_X0) * SV_CM_PX;

    if (s_has_pos != 0U)
    {
        dt = (uint32_t)(now - s_ms);
        if ((dt > 0U) && (dt <= 500U))
        {
            /* 相邻帧差分后低通滤波，降低视觉坐标抖动对制动的影响。 */
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

    /* 42° 是 +12 cm 的实测保持角，省去从零慢慢积累补偿量的等待。 */
    out = SV_KP * err - SV_KD * s_vel;
    ang = SV_ANG_HOLD + SV_DIR * out;
    ang = clampf(ang, SV_ANG_MIN, SV_ANG_RUN_MAX);
    set_ang(ang);

#if SV_DBG
    printf("[SERVO] ref=%.1f pos=%.2f vel=%.2f err=%.2f angle=%u\r\n",
        (double)SV_POS_REF,
        (double)pos,
        (double)s_vel,
        (double)err,
        (unsigned int)s_ang);
#endif
}
