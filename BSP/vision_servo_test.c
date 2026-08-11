#include "vision_servo_test.h"
#include "bsp_camera_usart.h"
#include "servo.h"
#include <stdint.h>
#include <stdio.h>

/* 以下角度是舵机控制输入角度，不是摆杆实际倾角。 */
#define SV_X0          (160.0f)  /* 摆杆中心的图像横坐标。 */
#define SV_CM_PX       (0.10f)   /* 每个像素对应的摆杆距离，单位 cm。 */
#define SV_POS_REF     (5.0f)    /* 钢珠目标位置，单位 cm。 */

#define SV_ANG0        (15.0f)   /* 摆杆水平时的舵机基准角。 */
#define SV_ANG_MIN     (0.0f)    /* 舵机允许的最小输入角。 */
#define SV_ANG_MAX     (30.0f)   /* 舵机允许的最大输入角。 */
#define SV_DIR         (-1.0f)   /* 正位置误差对应的舵机调节方向。 */

#define SV_KP          (1.00f)   /* 位置误差到角度增量的比例系数。 */
#define SV_KD          (0.20f)   /* 钢珠速度的制动系数。 */
#define SV_VEL_A       (0.35f)   /* 速度低通滤波系数。 */
#define SV_DBG         (0U)      /* 串口调试开关，闭环控制时保持关闭。 */

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

/* 主循环调用：每个新有效视觉帧执行一次闭环控制。 */
void Vision_Servo_Test_Update(void)
{
    uint32_t now;       /* 当前时间戳，单位 ms。 */
    uint32_t dt;        /* 相邻有效帧的时间间隔，单位 ms。 */
    uint32_t frame;     /* 当前有效视觉帧计数。 */
    float pos;          /* 当前钢珠位置，单位 cm。 */
    float vel;          /* 原始速度，单位 cm/s。 */
    float err;          /* 目标位置与当前位置的误差，单位 cm。 */
    float out;          /* PD 计算得到的角度增量。 */
    float ang;          /* 加基准后的目标舵机角度。 */

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
    /* 误差提供驱动力，速度项在接近目标时提前反向制动。 */
    out = SV_KP * err - SV_KD * s_vel;
    ang = SV_ANG0 + SV_DIR * out;
    set_ang(ang);

#if SV_DBG
    printf("[SERVO] pos=%.2f vel=%.2f err=%.2f angle=%u\r\n",
        (double)pos,
        (double)s_vel,
        (double)err,
        (unsigned int)s_ang);
#endif
}
