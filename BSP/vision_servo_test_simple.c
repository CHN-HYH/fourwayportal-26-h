#include "vision_servo_test.h"
#include "bsp_camera_usart.h"
#include "servo.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 简化的 PID 状态 */
typedef struct
{
    float err;       /* 当前误差 */
    float last_err;  /* 上次误差 */
    float integral;  /* 积分累计 */
} pid_simple_t;

/* 卡尔曼滤波 */
typedef struct
{
    float prev;
    float p;
    uint8_t first;
} kalman_simple_t;

static uint32_t s_frame;
static uint8_t s_inited;
static pid_simple_t s_pid;
static kalman_simple_t s_kf;
static float s_cc_last = PWM_CC_CENTER;
static uint32_t s_valid_ms;

/* 将视觉坐标转为像素误差 */
static float vision_to_target_px(float vision_x, float target_cm)
{
    /* 左右独立标定 */
    float center_x = INPUT_CENTER_X;
    float cm_px = (target_cm >= 0.0f) ? INPUT_CM_PX_POS : INPUT_CM_PX_NEG;
    float target_x = center_x + target_cm / cm_px;
    return target_x - vision_x;
}

/* 卡尔曼滤波 */
static float kalman(kalman_simple_t *kf, float x)
{
    if (kf->first)
    {
        kf->prev = x;
        kf->first = 0;
        return x;
    }

    kf->p += KALMAN_Q;
    float k = kf->p / (kf->p + KALMAN_R);
    kf->prev += k * (x - kf->prev);
    kf->p = (1.0f - k) * kf->p;
    return kf->prev;
}

/* 核心 PID 计算 - 返回 CC 增量 */
static float pid_calc(pid_simple_t *pid, float err, uint8_t first_frame)
{
    float p, i, d;
    float abs_err = fabsf(err);

    /* 初始化或目标切换时不产生 D 冲击 */
    if (first_frame)
    {
        pid->err = err;
        pid->last_err = err;
        pid->integral = 0.0f;
        return 0.0f;
    }

    /* 误差跨零清积分 */
    if ((err > 0.0f && pid->last_err < 0.0f) ||
        (err < 0.0f && pid->last_err > 0.0f))
    {
        pid->integral = 0.0f;
    }

    /* 计算 PID 三项 */
    p = PID_KP * err;

    /* 积分：只在中等误差时累加 */
    if (abs_err > STOP_ERR_PX && abs_err < DIST_MID_PX)
    {
        pid->integral += err;
        /* 积分限幅 */
        if (pid->integral > PID_I_LIMIT)
            pid->integral = PID_I_LIMIT;
        if (pid->integral < -PID_I_LIMIT)
            pid->integral = -PID_I_LIMIT;
    }
    i = PID_KI * pid->integral;

    /* 微分：使用帧间误差变化 */
    float delta_err = err - pid->last_err;
    d = PID_KD * delta_err;

    /* 接近目标时限制 D 项反向制动，防止反向过冲 */
    if (err * delta_err < 0.0f) /* 正在接近 */
    {
        float max_d = fabsf(p + i) + D_REVERSE_MARGIN;
        if (d > max_d) d = max_d;
        if (d < -max_d) d = -max_d;
    }

    pid->err = err;
    pid->last_err = err;

    return (p + i + d) * PWM_CC_SCALE;
}

/* 简化的舵机输出 */
static void set_servo(float cc_req)
{
    /* 机械限幅 */
    if (cc_req < PWM_CC_MIN) cc_req = PWM_CC_MIN;
    if (cc_req > PWM_CC_MAX) cc_req = PWM_CC_MAX;

    /* 变化率限制 */
    float delta = cc_req - s_cc_last;
    if (delta > RATE_LIMIT) cc_req = s_cc_last + RATE_LIMIT;
    if (delta < -RATE_LIMIT) cc_req = s_cc_last - RATE_LIMIT;

    uint16_t cc = (uint16_t)(cc_req + 0.5f);
    Servo_SetCc(cc);
    s_cc_last = (float)cc;
}

void Vision_Servo_Test_Init(void)
{
    Servo_Init();
    s_frame = vision.valid_n;
    s_inited = 0;
    s_cc_last = PWM_CC_CENTER;
    s_valid_ms = 0;

    memset(&s_pid, 0, sizeof(s_pid));
    memset(&s_kf, 0, sizeof(s_kf));
    s_kf.p = 0.01f;
    s_kf.first = 1;

#if SV_LINK_DEBUG
    printf("[SERVO] Simplified version build=%s %s\r\n", __DATE__, __TIME__);
    printf("        Kp=%.4f Ki=%.4f Kd=%.4f CC=%.0f\r\n",
           (double)PID_KP, (double)PID_KI, (double)PID_KD, (double)PWM_CC_CENTER);
#endif
}

VisionServoResult Vision_Servo_Test_Update(float target_cm, uint8_t hold)
{
    uint32_t frame = vision.valid_n;

    /* 没有新帧 */
    if (frame == s_frame)
        return VISION_SERVO_NO_FRAME;

    s_frame = frame;
    uint32_t valid_ms = vision.valid_ms;

    /* 初始化 */
    uint8_t first = 0;
    if (!s_inited)
    {
        s_kf.first = 1;
        s_pid.integral = 0.0f;
        s_inited = 1;
        first = 1;
    }

    /* 视觉超时重置 */
    if (s_valid_ms != 0 && (valid_ms - s_valid_ms) > HISTORY_TIMEOUT_MS)
    {
        s_kf.first = 1;
        s_pid.integral = 0.0f;
        first = 1;
    }
    s_valid_ms = valid_ms;

    /* 滤波后的像素坐标 */
    float x_flt = kalman(&s_kf, (float)vision.x);

    /* 计算像素误差 */
    float err_px = vision_to_target_px(x_flt, target_cm);
    float abs_err = fabsf(err_px);

    /* 判断是否到位 */
    uint8_t reached = (abs_err < REACH_ERR_PX);

    /* 计算 PID */
    float cc_delta = pid_calc(&s_pid, err_px, first);
    float cc_req = PWM_CC_CENTER + cc_delta;

    /* 输出舵机 */
    set_servo(cc_req);

#if SV_FRAME_DEBUG
    float pos_cm = (x_flt - INPUT_CENTER_X) *
                   ((x_flt >= INPUT_CENTER_X) ? INPUT_CM_PX_POS : INPUT_CM_PX_NEG);
    printf("[BALL] n=%lu x=%.1f pos=%.2f target=%.1f err_px=%.1f "
           "i=%.1f cc_req=%.1f cc=%.1f reached=%u\r\n",
           (unsigned long)frame,
           (double)x_flt,
           (double)pos_cm,
           (double)target_cm,
           (double)err_px,
           (double)s_pid.integral,
           (double)cc_req,
           (double)s_cc_last,
           (unsigned int)reached);
#endif

    return reached ? VISION_SERVO_REACHED : VISION_SERVO_MOVING;
}
