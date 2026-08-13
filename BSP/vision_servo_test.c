#include "vision_servo_test.h"
#include "bsp_camera_usart.h"
#include "servo.h"
#include <stdint.h>
#include <stdio.h>

typedef struct
{
    float x;       /* 当前滤波位置。 */
    float p;       /* 当前估计协方差。 */
    uint8_t first; /* 是否为首个测量值。 */
} sv_kalman_t;

static uint32_t s_frame = 0U;      /* 上次已处理的有效视觉帧计数。 */
static float s_sum = 0.0f;         /* 位置误差积分累计值。 */
static float s_err_last = 0.0f;    /* 上一有效帧的位置误差。 */
static float s_pulse = SV_PULSE_BASE; /* 上次下发的舵机脉宽，单位 us。 */
static uint16_t s_pulse_out = (uint16_t)SV_PULSE_BASE; /* 实际写入 PWM 的整数脉宽。 */
static sv_kalman_t s_kf;           /* 图像横坐标一维卡尔曼状态。 */

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

/* 返回浮点数绝对值。 */
static float absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

/* 复位位置滤波器和 PID 状态。 */
static void reset_ctl(void)
{
    s_sum = 0.0f;
    s_err_last = 0.0f;
    s_kf.x = 0.0f;
    s_kf.p = 0.01f;
    s_kf.first = 1U;
}

/* 对图像横坐标执行一维卡尔曼滤波。 */
static float filter_x(float x)
{
    float gain; /* 本次卡尔曼增益。 */

    if (s_kf.first != 0U)
    {
        s_kf.x = x;
        s_kf.first = 0U;
        return x;
    }

    s_kf.p += SV_KALMAN_Q;
    gain = s_kf.p / (s_kf.p + SV_KALMAN_R);
    s_kf.x += gain * (x - s_kf.x);
    s_kf.p = (1.0f - gain) * s_kf.p;
    return s_kf.x;
}

/* 按机械范围和单帧变化范围更新舵机脉宽。 */
static void set_pulse(float req)
{
    float pulse = clampf(req, SV_PULSE_MIN, SV_PULSE_MAX); /* 限幅后的舵机脉宽。 */
    uint16_t out; /* 四舍五入后写入 PWM 的整数脉宽。 */

    pulse = clampf(pulse,
                   s_pulse - SV_PULSE_STEP_US,
                   s_pulse + SV_PULSE_STEP_US);
    s_pulse = pulse;
    out = (uint16_t)(pulse + 0.5f);
    if (out != s_pulse_out)
    {
        Servo_SetPulseUs(out);
        s_pulse_out = out;
    }
}

/* 初始化视觉位置式 PID，并设置舵机基准脉宽。 */
void Vision_Servo_Test_Init(void)
{
    Servo_Init();
    s_frame = vision.valid_n;
    s_pulse = SV_PULSE_BASE;
    s_pulse_out = (uint16_t)SV_PULSE_BASE;
    reset_ctl();
    Servo_SetPulseUs(s_pulse_out);
}

/* 主循环调用：对每个新视觉帧执行一次像素域位置式 PID。 */
void Vision_Servo_Test_Update(void)
{
    uint32_t frame; /* 当前有效视觉帧计数。 */
    float x;        /* 卡尔曼滤波后的图像横坐标。 */
    float pos;      /* 当前钢珠位置，单位 cm。 */
    float err;      /* 目标横坐标与当前位置误差，单位 px。 */
    float raw;      /* 本帧 PID 原始输出。 */
    float offset;   /* 本帧 PID 产生的基准脉宽偏移，单位 us。 */
    float req;      /* 本帧请求的舵机脉宽，单位 us。 */

    if (Camera_Vision_IsUsable() == 0U)
    {
        reset_ctl();
        return;
    }

    frame = vision.valid_n;
    if (frame == s_frame)
    {
        return;
    }
    s_frame = frame;

    x = filter_x((float)vision.x);
    pos = (x - SV_X0) * SV_CM_PX;
    err = SV_X_REF - x;

    /* 到位后保持上一脉宽，不继续累计积分。 */
    if (absf(err) < SV_DEADBAND_PX)
    {
#if SV_DBG
        printf("[SERVO] ref=%.1f x=%.2f pos=%.2f err=%.2f sum=%.2f raw=0.00 offset=%.2f req=%.1f pulse=%u\r\n",
            (double)SV_X_REF,
            (double)x,
            (double)pos,
            (double)err,
            (double)s_sum,
            (double)(s_pulse - SV_PULSE_BASE),
            (double)s_pulse,
            (unsigned int)s_pulse_out);
#endif
        return;
    }

    s_sum = clampf(s_sum + err, -SV_I_MAX, SV_I_MAX);
    raw = SV_KP * err + SV_KI * s_sum + SV_KD * (err - s_err_last);
    raw = clampf(raw, -SV_OUT_MAX, SV_OUT_MAX);
    s_err_last = err;

    offset = SV_DIR * raw * SV_OUT_SCALE_US;
    req = SV_PULSE_BASE + offset;
    set_pulse(req);

#if SV_DBG
    printf("[SERVO] ref=%.1f x=%.2f pos=%.2f err=%.2f sum=%.2f raw=%.2f offset=%.2f req=%.1f pulse=%u\r\n",
        (double)SV_X_REF,
        (double)x,
        (double)pos,
        (double)err,
        (double)s_sum,
        (double)raw,
        (double)offset,
        (double)req,
        (unsigned int)s_pulse_out);
#endif
}
