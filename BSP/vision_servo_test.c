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
    float err;       /* 当前误差（像素）*/
    float last_err;  /* 上次误差 */
    float integral;  /* 积分累计 */
    float target_cm; /* 当前目标位置 */
    float last_pos;  /* 上次钢珠位置，用于检测运动方向 */
    uint8_t still_count; /* 静止帧计数 */
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
static float vision_to_error_px(float vision_x, float target_cm)
{
    /* 左右独立标定 */
    float center_x = INPUT_CENTER_X;
    float cm_px = (target_cm >= 0.0f) ? INPUT_CM_PX_POS : INPUT_CM_PX_NEG;
    float target_x = center_x + target_cm / cm_px;
    return target_x - vision_x;
}

/* 卡尔曼滤波 */
static float kalman_filter(kalman_simple_t *kf, float x)
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
static float pid_calc(pid_simple_t *pid, float err, float current_pos, uint8_t reset)
{
    float p, i, d;
    float abs_err = fabsf(err);

    /* 初始化或目标切换时不产生 D 冲击 */
    if (reset)
    {
        pid->err = err;
        pid->last_err = err;
        pid->last_pos = current_pos;
        pid->integral = 0.0f;
        pid->still_count = 0;
        return 0.0f;
    }

    /* 检测钢珠是否静止 */
    float ball_movement = current_pos - pid->last_pos;
    float abs_movement = fabsf(ball_movement);

    if (abs_movement < 0.15f) /* 静止阈值：约0.15cm */
    {
        if (pid->still_count < 255)
            pid->still_count++;
    }
    else
    {
        pid->still_count = 0;
    }

    /* P 项 */
    p = PID_KP * err;

    /* 误差跨零判断：区分"过冲回摆"和"穿越目标继续运动" */
    float delta_err = err - pid->last_err;
    float abs_delta = fabsf(delta_err);

    if ((err > 0.0f && pid->last_err < 0.0f) ||
        (err < 0.0f && pid->last_err > 0.0f))
    {
        /* 情况1：小幅度跨零 + 钢珠反向运动 = 过冲回摆，清积分 */
        if (abs_delta < 15.0f && (err * ball_movement < 0.0f))
        {
            pid->integral = 0.0f;
            pid->still_count = 0;
#if SV_FRAME_DEBUG
            printf("  [INTEGRAL RESET] Overshoot detected\r\n");
#endif
        }
        /* 情况2：大幅度跨零或钢珠同向运动 = 穿越目标，保留积分 */
    }

    /* I 项：只在中等误差时累加 */
    if (abs_err > STOP_ERR_PX && abs_err < DIST_MID_PX)
    {
        /* 基础积分 */
        float i_gain = 1.0f;

        /* 静止时加速积分累积 */
        if (pid->still_count > 3 && abs_err > 15.0f)
        {
            i_gain = 2.5f; /* 静止时积分增益 */
        }

        pid->integral += err * i_gain;

        /* 积分限幅 */
        if (pid->integral > PID_I_LIMIT)
            pid->integral = PID_I_LIMIT;
        if (pid->integral < -PID_I_LIMIT)
            pid->integral = -PID_I_LIMIT;
    }
    else if (abs_err >= DIST_MID_PX)
    {
        /* 远距离直接清零积分 */
        pid->integral = 0.0f;
    }
    i = PID_KI * pid->integral;

    /* D 项：使用帧间误差变化 */
    d = PID_KD * delta_err;

    /* 接近目标时限制 D 项反向制动，防止反向过冲 */
    if (err * delta_err < 0.0f) /* 正在接近 */
    {
        float max_d = fabsf(p + i) + D_REVERSE_MARGIN;
        if (d > max_d) d = max_d;
        if (d < -max_d) d = -max_d;
    }

    /* 静止突破机制：静止超过7帧且误差>18像素，给额外推力 */
    float boost = 0.0f;
    if (pid->still_count > 7 && abs_err > 18.0f)
    {
        /* 根据误差大小动态调整boost */
        float boost_base = (abs_err > 40.0f) ? 3.5f : 2.5f;
        boost = (err > 0.0f) ? boost_base : -boost_base;
#if SV_FRAME_DEBUG
        static uint8_t boost_print = 0;
        if ((boost_print++ % 10) == 0)
            printf("  [BOOST] Still %u frames, err=%.1f boost=%.1f\r\n",
                   pid->still_count, (double)abs_err, (double)boost);
#endif
    }

    pid->err = err;
    pid->last_err = err;
    pid->last_pos = current_pos;

    float output = (p + i + d) * PWM_CC_SCALE + boost;

#if SV_FRAME_DEBUG
    /* 输出调试信息便于调参 */
    static uint32_t s_print_cnt = 0;
    if ((++s_print_cnt % 5) == 0) /* 每5帧打印一次，减少串口负担 */
    {
        printf("  [PID] err=%.1f p=%.2f i=%.2f(%.0f) d=%.2f boost=%.1f out=%.2f still=%u\r\n",
               (double)err, (double)p, (double)i, (double)pid->integral,
               (double)d, (double)boost, (double)output, pid->still_count);
    }
#endif

    return output;
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
    printf("[SERVO] Simplified PID - build=%s %s\r\n", __DATE__, __TIME__);
    printf("        Kp=%.4f Ki=%.4f Kd=%.4f CC_center=%.0f scale=%.2f\r\n",
           (double)PID_KP, (double)PID_KI, (double)PID_KD,
           (double)PWM_CC_CENTER, (double)PWM_CC_SCALE);
    printf("        I_limit=%.0f I_range=%.0f-%.0f px\r\n",
           (double)PID_I_LIMIT, (double)STOP_ERR_PX, (double)DIST_MID_PX);
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

    /* 初始化或视觉超时 */
    uint8_t reset = 0;
    if (!s_inited)
    {
        s_kf.first = 1;
        s_pid.integral = 0.0f;
        s_pid.target_cm = target_cm;
        s_inited = 1;
        reset = 1;
    }
    else if (s_valid_ms != 0 && (valid_ms - s_valid_ms) > HISTORY_TIMEOUT_MS)
    {
        /* 视觉超时，重置滤波器和积分 */
        s_kf.first = 1;
        s_pid.integral = 0.0f;
        reset = 1;
    }
    else if (fabsf(target_cm - s_pid.target_cm) > 0.1f)
    {
        /* 目标改变，清除积分 */
        s_pid.integral = 0.0f;
        s_pid.target_cm = target_cm;
        reset = 1;
    }
    s_valid_ms = valid_ms;

    /* 滤波后的像素坐标 */
    float x_flt = kalman_filter(&s_kf, (float)vision.x);

    /* 计算当前钢珠位置（cm）用于运动方向判断 */
    float pos_cm = (x_flt - INPUT_CENTER_X) *
                   ((x_flt >= INPUT_CENTER_X) ? INPUT_CM_PX_POS : INPUT_CM_PX_NEG);

    /* 计算像素误差 */
    float err_px = vision_to_error_px(x_flt, target_cm);
    float abs_err = fabsf(err_px);

    /* 判断是否到位 */
    uint8_t reached = (abs_err < REACH_ERR_PX);

    /* 计算 PID */
    float cc_delta = pid_calc(&s_pid, err_px, pos_cm, reset);
    float cc_req = PWM_CC_CENTER + cc_delta;

    /* 输出舵机 */
    set_servo(cc_req);

#if SV_FRAME_DEBUG
    printf("[BALL] n=%lu x=%.1f pos=%.2f->%.1f err_px=%.1f "
           "cc_req=%.1f cc=%.1f %s\r\n",
           (unsigned long)frame,
           (double)x_flt,
           (double)pos_cm,
           (double)target_cm,
           (double)err_px,
           (double)cc_req,
           (double)s_cc_last,
           reached ? "REACHED" : "MOVING");
#endif

    return reached ? VISION_SERVO_REACHED : VISION_SERVO_MOVING;
}
