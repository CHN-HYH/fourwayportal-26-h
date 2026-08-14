#include "vision_servo_test.h"
#include "bsp_camera_usart.h"
#include "servo.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 位置式 PID 参数。 */
#define PID_KP             (0.028f)
#define PID_KI             (0.005f)
#define PID_KD             (0.45f)
#define PID_I_LIMIT        (1300.0f)

/* 控制器使用的标准图像坐标标定。 */
#define CTRL_CENTER_X      (360.0f)
#define CTRL_MIN_X         (99.0f)
#define CTRL_MAX_X         (599.0f)
#define PIPE_HALF_CM       (12.5f)

/* 当前串口协议坐标到实际位置的标定。 */
#define INPUT_CENTER_X     (158.0f)
#define INPUT_CM_PER_PX    (0.082f)

/* 舵机 CC 和控制保护参数。 */
#define PWM_CC_CENTER      (60.0f)
#define PWM_CC_MIN         (50.0f)
#define PWM_CC_MAX         (70.0f)
#define PWM_CC_SCALE       (0.80f)
#define DEADBAND_PX        (10.0f)
#define INTEGRAL_DERR_PX   (4.0f)
#define STATIC_DERR_PX     (1.0f)
#define STATIC_CC_POS      (65.0f)
#define STATIC_CC_NEG      (54.0f)
#define RATE_LIMIT         (2.0f)
#define HISTORY_TIMEOUT_MS (300U)
#define KALMAN_Q           (1.0f)
#define KALMAN_R           (0.3f)

typedef struct
{
    float kp;       /* 比例系数。 */
    float ki;       /* 积分系数。 */
    float kd;       /* 微分系数。 */
    float err;      /* 当前误差。 */
    float last_err; /* 上一次误差。 */
    float integral; /* 误差积分。 */
    float target;   /* 目标值。 */
    float p_term;   /* 本帧比例项。 */
    float i_term;   /* 本帧积分项。 */
    float d_term;   /* 本帧差分项。 */
    float delta_err; /* 相邻有效帧的误差变化。 */
    float raw;      /* 本帧 PID 浮点输出。 */
    float out;      /* 控制器浮点输出。 */
} pid_t;

typedef struct
{
    float prev;     /* 上一次滤波结果。 */
    float p;        /* 估计误差协方差。 */
    float q;        /* 过程噪声。 */
    float r;        /* 测量噪声。 */
    float gain;     /* 卡尔曼增益。 */
    uint8_t first;  /* 首次采样标志。 */
} kalman_t;

static uint32_t s_frame;             /* 上次处理的有效视觉帧编号。 */
static uint8_t s_inited;             /* 控制状态是否已经初始化。 */
static pid_t s_pid;                  /* 位置式 PID 状态。 */
static kalman_t s_kf;                /* 横坐标卡尔曼滤波状态。 */
static float s_cc_last = PWM_CC_CENTER; /* 上一次舵机 CC 值。 */
static uint32_t s_valid_ms;          /* 上一次参与控制的有效帧时刻。 */

static void pid_init(pid_t *pid)
{
    memset(pid, 0, sizeof(*pid));
}

/* 位置式 PID：u=Kp*e+Ki*sum(e)+Kd*(e-e_last)。 */
static float pid_calc(pid_t *pid, float val)
{
    float delta_error;
    float out;

    pid->err = pid->target - val;
    delta_error = pid->err - pid->last_err;
    pid->delta_err = delta_error;

    /* 钢珠运动较慢时才累计积分，避免往返运动期间长期积分饱和。 */
    if ((delta_error > -INTEGRAL_DERR_PX) &&
        (delta_error < INTEGRAL_DERR_PX))
    {
        pid->integral += pid->err;
    }
    if (pid->integral > PID_I_LIMIT)
    {
        pid->integral = PID_I_LIMIT;
    }
    if (pid->integral < -PID_I_LIMIT)
    {
        pid->integral = -PID_I_LIMIT;
    }

    pid->p_term = pid->kp * pid->err;
    pid->i_term = pid->ki * pid->integral;
    pid->d_term = pid->kd * delta_error;
    out = pid->p_term + pid->i_term + pid->d_term;
    pid->last_err = pid->err;
    if (out > 500.0f)
    {
        out = 500.0f;
    }
    if (out < -500.0f)
    {
        out = -500.0f;
    }
    pid->raw = out;
    pid->out = out;
    return pid->out;
}

static float pid_to_cc(pid_t *pid, float val)
{
    float out = pid_calc(pid, val); /* 位置式 PID 的浮点输出。 */
    float cc = PWM_CC_CENTER + out * PWM_CC_SCALE;

    /* 低速未到位时直接越过实测静摩擦区，开始运动后恢复 PID 输出。 */
    if ((pid->delta_err > -STATIC_DERR_PX) &&
        (pid->delta_err < STATIC_DERR_PX))
    {
        if ((pid->err >= DEADBAND_PX) && (cc < STATIC_CC_POS))
        {
            cc = STATIC_CC_POS;
        }
        else if ((pid->err <= -DEADBAND_PX) && (cc > STATIC_CC_NEG))
        {
            cc = STATIC_CC_NEG;
        }
    }

    if (cc < PWM_CC_MIN)
    {
        cc = PWM_CC_MIN;
    }
    if (cc > PWM_CC_MAX)
    {
        cc = PWM_CC_MAX;
    }
    return cc;
}

/* 按机械限幅和单帧变化限制写入舵机 CC。 */
static float set_cc(float req)
{
    float cc = req;
    float delta;

    if (cc < PWM_CC_MIN)
    {
        cc = PWM_CC_MIN;
    }
    if (cc > PWM_CC_MAX)
    {
        cc = PWM_CC_MAX;
    }

    delta = cc - s_cc_last;
    if (delta > RATE_LIMIT)
    {
        cc = s_cc_last + RATE_LIMIT;
    }
    if (delta < -RATE_LIMIT)
    {
        cc = s_cc_last - RATE_LIMIT;
    }
    s_cc_last = cc;

    DL_TimerA_setCaptureCompareValue(PWM_Servo_INST,
                                     (uint16_t)(cc + 0.5f),
                                     DL_TIMER_CC_0_INDEX);
    return cc;
}

static void kalman_init(kalman_t *kf, float q, float r)
{
    memset(kf, 0, sizeof(*kf));
    kf->q = q;
    kf->r = r;
    kf->p = 0.01f;
    kf->first = 1U;
}

static float kalman_filter(kalman_t *kf, float in)
{
    if (kf->first != 0U)
    {
        kf->prev = in;
        kf->first = 0U;
        return in;
    }

    kf->p += kf->q;
    kf->gain = kf->p / (kf->p + kf->r);
    kf->prev += kf->gain * (in - kf->prev);
    kf->p = (1.0f - kf->gain) * kf->p;
    return kf->prev;
}

/* 将当前协议坐标换算到控制器原始标定的坐标域。 */
static float input_to_ctrl_x(float input_x)
{
    float pos_cm = (input_x - INPUT_CENTER_X) * INPUT_CM_PER_PX;

    if (pos_cm >= 0.0f)
    {
        return CTRL_CENTER_X +
               pos_cm * (CTRL_MAX_X - CTRL_CENTER_X) / PIPE_HALF_CM;
    }
    return CTRL_CENTER_X +
           pos_cm * (CTRL_CENTER_X - CTRL_MIN_X) / PIPE_HALF_CM;
}

/* 将厘米目标按左右两侧独立标定换算为控制器坐标。 */
static float target_to_ctrl_x(float target_cm)
{
    float px_per_cm;

    if (target_cm >= 0.0f)
    {
        px_per_cm = (CTRL_MAX_X - CTRL_CENTER_X) / PIPE_HALF_CM;
    }
    else
    {
        px_per_cm = (CTRL_CENTER_X - CTRL_MIN_X) / PIPE_HALF_CM;
    }
    return CTRL_CENTER_X + target_cm * px_per_cm;
}

/* 将控制器坐标还原为钢珠物理位置，便于逐帧观察。 */
static float ctrl_x_to_pos_cm(float x)
{
    if (x >= CTRL_CENTER_X)
    {
        return (x - CTRL_CENTER_X) * PIPE_HALF_CM /
               (CTRL_MAX_X - CTRL_CENTER_X);
    }
    return (x - CTRL_CENTER_X) * PIPE_HALF_CM /
           (CTRL_CENTER_X - CTRL_MIN_X);
}

/* 输出一帧识别位置和对应的 PID 控制结果。 */
static void print_frame(float input_x,
                        float x,
                        float target_x,
                        float error,
                        float delta_error,
                        float req,
                        float cc,
                        uint8_t hold)
{
#if SV_FRAME_DEBUG
    float pos_cm = ctrl_x_to_pos_cm(x); /* 滤波后的钢珠位置，单位 cm。 */
    float x_flt = INPUT_CENTER_X + pos_cm / INPUT_CM_PER_PX;

    printf("[BALL] n=%lu raw_x=%.1f flt_x=%.2f pos=%.2f target=%.1f "
           "err=%.2f derr=%.2f p=%.2f i=%.2f d=%.2f out=%.2f req=%.2f cc=%.2f "
           "pwm=%u hold=%u\r\n",
           (unsigned long)vision.valid_n,
           (double)input_x,
           (double)x_flt,
           (double)pos_cm,
           (double)target_x,
           (double)error,
           (double)delta_error,
           (double)s_pid.p_term,
           (double)s_pid.i_term,
           (double)s_pid.d_term,
           (double)s_pid.raw,
           (double)req,
           (double)cc,
           (unsigned int)(cc + 0.5f),
           (unsigned int)hold);
#else
    (void)input_x;
    (void)x;
    (void)target_x;
    (void)error;
    (void)delta_error;
    (void)req;
    (void)cc;
    (void)hold;
#endif
}

void Vision_Servo_Test_Init(void)
{
    Servo_Init();
    s_frame = vision.valid_n;
    s_inited = 0U;
    s_cc_last = PWM_CC_CENTER;
    s_valid_ms = 0U;
#if SV_LINK_DEBUG
    printf("[SERVO] init pin=PB8 timer=TIMA0_CCP0 cc=60\r\n");
#endif
}

VisionServoResult Vision_Servo_Test_Update(float target_cm)
{
    uint32_t frame = vision.valid_n; /* 当前有效视觉帧编号。 */
    float input_x;
    float x;
    float target_x;
    float error;
    float delta_error;
    float cc;
    float req;
    uint32_t valid_ms;
    uint8_t first = 0U; /* 当前是否为控制器初始化后的第一帧。 */
    uint8_t history_reset = 0U; /* 视觉间隔过长时重建位置和差分历史。 */
    uint8_t target_changed = 0U; /* 目标改变时重置旧目标的控制状态。 */
    static uint32_t status_ms = 0U; /* 上次链路状态输出时间。 */

#if SV_LINK_DEBUG
    {
        uint32_t now = Camera_Vision_GetTimeMs(); /* 当前视觉时间基，单位 ms。 */
        if ((uint32_t)(now - status_ms) >= 1000U)
        {
            status_ms = now;
            printf("[SERVO] frame=%lu valid_n=%lu valid=%u x=%u sum_err=%lu overflow=%lu\r\n",
                   (unsigned long)vision.frame_n,
                   (unsigned long)vision.valid_n,
                   (unsigned int)vision.valid,
                   (unsigned int)vision.x,
                   (unsigned long)vision.sum_err,
                   (unsigned long)vision.rx_overflow);
        }
    }
#endif

    /* valid_n 增长表示存在尚未消费的有效坐标，后续无目标帧不能覆盖它。 */
    if (frame == s_frame)
    {
        return VISION_SERVO_NO_FRAME;
    }
    s_frame = frame;
    valid_ms = vision.valid_ms;

    if (s_inited == 0U)
    {
        pid_init(&s_pid);
        kalman_init(&s_kf, KALMAN_Q, KALMAN_R);
        s_inited = 1U;
        first = 1U;
    }

    if ((s_valid_ms != 0U) &&
        ((uint32_t)(valid_ms - s_valid_ms) > HISTORY_TIMEOUT_MS))
    {
        kalman_init(&s_kf, KALMAN_Q, KALMAN_R);
        s_pid.integral = 0.0f;
        history_reset = 1U;
    }
    s_valid_ms = valid_ms;

    input_x = (float)vision.x;
    x = kalman_filter(&s_kf, input_to_ctrl_x(input_x));
    target_x = target_to_ctrl_x(target_cm);
    if ((first == 0U) &&
        ((target_x < (s_pid.target - 0.01f)) ||
         (target_x > (s_pid.target + 0.01f))))
    {
        s_pid.integral = 0.0f;
        target_changed = 1U;
    }
    s_pid.target = target_x;
    error = target_x - x;
    delta_error = error - s_pid.last_err;

    s_pid.kp = PID_KP;
    s_pid.ki = PID_KI;
    s_pid.kd = PID_KD;
    if ((first != 0U) || (history_reset != 0U) ||
        (target_changed != 0U))
    {
        /* 没有连续历史时不产生差分冲击。 */
        s_pid.last_err = error;
        delta_error = 0.0f;
    }

    /* 进入目标误差范围后立即到位，清除已经积累的原方向推力。 */
    if ((error > -DEADBAND_PX) && (error < DEADBAND_PX))
    {
        s_pid.err = error;
        s_pid.last_err = error;
        s_pid.integral = 0.0f;
        s_pid.p_term = PID_KP * error;
        s_pid.i_term = 0.0f;
        s_pid.d_term = 0.0f;
        s_pid.delta_err = 0.0f;
        s_pid.raw = 0.0f;
        s_pid.out = 0.0f;
        req = PWM_CC_CENTER;
        cc = set_cc(req);
        print_frame(input_x, x, target_x, error, delta_error, req, cc, 1U);
        return VISION_SERVO_HOLDING;
    }

    /* 跨过目标后清除旧方向积分，避免残余推力继续扩大过冲。 */
    if (((error > 0.0f) && (s_pid.last_err < 0.0f)) ||
        ((error < 0.0f) && (s_pid.last_err > 0.0f)))
    {
        s_pid.integral = 0.0f;
    }

    req = pid_to_cc(&s_pid, x);
    cc = set_cc(req);
    print_frame(input_x, x, target_x, error, delta_error, req, cc, 0U);
    return VISION_SERVO_MOVING;
}
