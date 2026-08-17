#include "vision_servo_test.h"
#include "bsp_camera_usart.h"
#include "servo.h"
#include "ti_msp_dl_config.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    float err;      /* 当前误差。 */
    float last_err; /* 上一次误差。 */
    float integral; /* 误差积分。 */
    float target;   /* 目标值。 */
    float p_term;   /* 本帧比例项。 */
    float i_term;   /* 本帧积分项。 */
    float d_term;   /* 本帧差分项。 */
    float delta_err; /* 相邻有效帧的误差变化。 */
    float raw;      /* 本帧 PID 浮点输出。 */
} pid_t;

typedef struct
{
    float prev;     /* 上一次滤波结果。 */
    float p;        /* 估计误差协方差。 */
    float q;        /* 过程噪声。 */
    float r;        /* 测量噪声。 */
    uint8_t first;  /* 首次采样标志。 */
} kalman_t;

static uint32_t s_frame;             /* 上次处理的有效视觉帧编号。 */
static uint8_t s_inited;             /* 控制状态是否已经初始化。 */
static pid_t s_pid;                  /* 位置式 PID 状态。 */
static kalman_t s_kf;                /* 横坐标卡尔曼滤波状态。 */
static float s_cc_last = PWM_CC_CENTER; /* 上一次舵机 CC 值。 */
static uint32_t s_valid_ms;          /* 上一次参与控制的有效帧时刻。 */
static uint8_t s_static_pulse_left;  /* 静摩擦启动脉冲剩余帧数。 */
static uint8_t s_static_cooldown;    /* 再次启动补偿前的冷却帧数。 */
static uint8_t s_static_active;      /* 本帧是否使用了静摩擦补偿。 */
static uint8_t s_stop_active;        /* PID 是否已经因连续到位而停止。 */
static uint8_t s_stop_n;             /* 连续满足停止误差的有效帧数。 */

static void pid_init(pid_t *pid)
{
    memset(pid, 0, sizeof(*pid));
}

static void static_comp_reset(void)
{
    s_static_pulse_left = 0U;
    s_static_cooldown = 0U;
    s_static_active = 0U;
}

static void stop_reset(void)
{
    s_stop_active = 0U;
    s_stop_n = 0U;
}

/* 停止 PID 后冻结积分并清除内部输出，保留当前舵机 CC 不变。 */
static void pid_stop(pid_t *pid, float err)
{
    pid->err = err;
    pid->last_err = err;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->delta_err = 0.0f;
    pid->raw = 0.0f;
    static_comp_reset();
}

/* 按统一距离阶段计算积分和微分权重。 */
static void distance_scale(float abs_error, float *i_scale, float *d_scale)
{
    *i_scale = 1.0f;
    *d_scale = 1.0f;

    if (abs_error >= DIST_FAR_PX)
    {
        *i_scale = 0.0f;
        *d_scale = D_MIN_SCALE;
    }
    else if (abs_error > DIST_MID_PX)
    {
        *i_scale = (DIST_FAR_PX - abs_error) /
                   (DIST_FAR_PX - DIST_MID_PX);
        *d_scale = D_MIN_SCALE;
    }
    else if (abs_error > DIST_NEAR_PX)
    {
        *d_scale = D_MIN_SCALE + (1.0f - D_MIN_SCALE) *
                   (DIST_MID_PX - abs_error) /
                   (DIST_MID_PX - DIST_NEAR_PX);
    }
}

/* 位置式 PID：u=Kp*e+Ki*sum(e)+Kd*(e-e_last)。 */
static float pid_calc(pid_t *pid, float val, float gain)
{
    float delta_error;
    float abs_error;
    float abs_delta;
    float i_scale;
    float i_gain;
    float d_scale;
    float out;

    pid->err = pid->target - val;
    delta_error = pid->err - pid->last_err;
    pid->delta_err = delta_error;
    abs_error = (pid->err >= 0.0f) ? pid->err : -pid->err;
    abs_delta = (delta_error >= 0.0f) ? delta_error : -delta_error;
    distance_scale(abs_error, &i_scale, &d_scale);

    /* 远距离清积分，中距离预积分，近距离叠加速度权重。 */
    if (abs_error >= DIST_FAR_PX)
    {
        pid->integral = 0.0f;
    }
    else
    {
        if ((abs_error <= DIST_MID_PX) &&
            (abs_delta >= (INTEGRAL_DERR_PX * 3.0f)))
        {
            i_scale = 0.0f;
        }
        else if ((abs_error <= DIST_MID_PX) &&
                 (abs_delta > INTEGRAL_DERR_PX))
        {
            i_scale *= ((INTEGRAL_DERR_PX * 3.0f) - abs_delta) /
                       (INTEGRAL_DERR_PX * 2.0f);
        }
        pid->integral += pid->err * i_scale;
    }
    if (pid->integral > PID_I_LIMIT)
    {
        pid->integral = PID_I_LIMIT;
    }
    if (pid->integral < -PID_I_LIMIT)
    {
        pid->integral = -PID_I_LIMIT;
    }

    pid->p_term = PID_KP * pid->err;
    pid->i_term = PID_KI * pid->integral;
    /* 钢珠偏离目标时使用完整 D，接近目标时采用距离阶段权重。 */
    if ((pid->err * delta_error) >= 0.0f)
    {
        d_scale = 1.0f;
    }
    pid->d_term = PID_KD * delta_error * d_scale;
    i_gain = (gain < I_GAIN_MIN) ? I_GAIN_MIN : gain;
    /* P、D 按位置缩放，积分保留最低强度以减少对启动脉冲的依赖。 */
    pid->p_term *= gain;
    pid->i_term *= i_gain;
    pid->d_term *= gain;
    out = pid->p_term + pid->i_term + pid->d_term;
    pid->last_err = pid->err;
    pid->raw = out;
    return pid->raw;
}

static float pid_to_cc(pid_t *pid, float val, float gain)
{
    float out = pid_calc(pid, val, gain); /* 位置式 PID 的浮点输出。 */
    float cc = PWM_CC_CENTER + out * PWM_CC_SCALE;
    uint8_t stopped;
    uint8_t need_static;

    stopped = ((pid->delta_err > -STATIC_DERR_PX) &&
               (pid->delta_err < STATIC_DERR_PX));
    need_static = (((pid->err >= DEADBAND_PX) && (cc < STATIC_CC_POS)) ||
                   ((pid->err <= -DEADBAND_PX) && (cc > STATIC_CC_NEG)));
    s_static_active = 0U;

    if (stopped == 0U)
    {
        s_static_pulse_left = 0U;
        s_static_cooldown = STATIC_COOLDOWN_FRAMES;
    }
    else
    {
        if (s_static_cooldown > 0U)
        {
            s_static_cooldown--;
        }
        if ((s_static_pulse_left == 0U) &&
            (s_static_cooldown == 0U) && (need_static != 0U))
        {
            s_static_pulse_left = STATIC_PULSE_FRAMES;
        }

        /* 补偿只作为启动脉冲，防止低速时长期保持大倾角造成反复过冲。 */
        if (s_static_pulse_left > 0U)
        {
            if ((pid->err >= DEADBAND_PX) && (cc < STATIC_CC_POS))
            {
                cc = STATIC_CC_POS;
                s_static_active = 1U;
            }
            else if ((pid->err <= -DEADBAND_PX) && (cc > STATIC_CC_NEG))
            {
                cc = STATIC_CC_NEG;
                s_static_active = 1U;
            }

            s_static_pulse_left--;
            if (s_static_pulse_left == 0U)
            {
                s_static_cooldown = STATIC_COOLDOWN_FRAMES;
            }
        }
    }

    return cc;
}

/* 按机械限幅和单帧变化限制写入舵机 CC。 */
static float set_cc(float req)
{
    float cc = req;
    float delta;
    float limit = RATE_LIMIT;

    if (cc < PWM_CC_MIN)
    {
        cc = PWM_CC_MIN;
    }
    if (cc > PWM_CC_MAX)
    {
        cc = PWM_CC_MAX;
    }

    delta = cc - s_cc_last;
    /* 钢珠离目标较远且尚未运动时，加快舵机向目标方向越过静摩擦区。 */
    if ((((s_pid.err >= DIST_FAR_PX) && (delta > 0.0f)) ||
         ((s_pid.err <= -DIST_FAR_PX) && (delta < 0.0f))) &&
        (s_pid.delta_err > -STATIC_DERR_PX) &&
        (s_pid.delta_err < STATIC_DERR_PX))
    {
        limit = START_RATE_LIMIT;
    }
    /* 舵机输出与误差变化同向时是在抑制钢珠运动，允许更快制动。 */
    if (((s_pid.delta_err > STATIC_DERR_PX) && (delta > 0.0f)) ||
        ((s_pid.delta_err < -STATIC_DERR_PX) && (delta < 0.0f)))
    {
        limit = BRAKE_RATE_LIMIT;
    }
    if (delta > limit)
    {
        cc = s_cc_last + limit;
    }
    if (delta < -limit)
    {
        cc = s_cc_last - limit;
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
    float gain;

    if (kf->first != 0U)
    {
        kf->prev = in;
        kf->first = 0U;
        return in;
    }

    kf->p += kf->q;
    gain = kf->p / (kf->p + kf->r);
    kf->prev += gain * (in - kf->prev);
    kf->p = (1.0f - gain) * kf->p;
    return kf->prev;
}

/* 将当前协议坐标换算到控制器原始标定的坐标域。 */
static float input_to_ctrl_x(float input_x)
{
    float cm_px = (input_x >= INPUT_CENTER_X) ?
                  INPUT_CM_PX_POS : INPUT_CM_PX_NEG;
    float pos_cm = (input_x - INPUT_CENTER_X) * cm_px;

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

static float position_gain(float pos_cm)
{
    float arm_cm = PIVOT_POS_CM - pos_cm;
    float gain;

    if (arm_cm < GAIN_MIN_ARM_CM)
    {
        arm_cm = GAIN_MIN_ARM_CM;
    }
    gain = sqrtf(GAIN_REF_ARM_CM / arm_cm);
    if (gain < GAIN_SCALE_MIN)
    {
        gain = GAIN_SCALE_MIN;
    }
    if (gain > GAIN_SCALE_MAX)
    {
        gain = GAIN_SCALE_MAX;
    }
    return gain;
}

/* 输出一帧识别位置和对应的 PID 控制结果。 */
static void print_frame(float input_x,
                        float x,
                        float target_x,
                        float error,
                        float delta_error,
                        float gain,
                        float req,
                        float cc,
                        uint8_t reached)
{
#if SV_FRAME_DEBUG
    float pos_cm = ctrl_x_to_pos_cm(x); /* 滤波后的钢珠位置，单位 cm。 */
    float cm_px = (pos_cm >= 0.0f) ? INPUT_CM_PX_POS : INPUT_CM_PX_NEG;
    float x_flt = INPUT_CENTER_X + pos_cm / cm_px;

    printf("[BALL] n=%lu raw_x=%.1f flt_x=%.2f pos=%.2f target=%.1f "
           "err=%.2f derr=%.2f gain=%.2f p=%.2f i=%.2f d=%.2f out=%.2f "
           "req=%.2f cc=%.2f pwm=%u kick=%u reached=%u\r\n",
           (unsigned long)vision.valid_n,
           (double)input_x,
           (double)x_flt,
           (double)pos_cm,
           (double)target_x,
           (double)error,
           (double)delta_error,
           (double)gain,
           (double)s_pid.p_term,
           (double)s_pid.i_term,
           (double)s_pid.d_term,
           (double)s_pid.raw,
           (double)req,
           (double)cc,
           (unsigned int)(cc + 0.5f),
           (unsigned int)s_static_active,
           (unsigned int)reached);
#else
    (void)input_x;
    (void)x;
    (void)target_x;
    (void)error;
    (void)delta_error;
    (void)gain;
    (void)req;
    (void)cc;
    (void)reached;
#endif
}

void Vision_Servo_Test_Init(void)
{
    Servo_Init();
    s_frame = vision.valid_n;
    s_inited = 0U;
    s_cc_last = PWM_CC_CENTER;
    s_valid_ms = 0U;
    static_comp_reset();
    stop_reset();
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
    float abs_error;
    float abs_delta;
    float pos_cm;
    float gain;
    float cc;
    float req;
    uint8_t stop_ready;
    uint32_t valid_ms;
    uint8_t first = 0U; /* 当前是否为控制器初始化后的第一帧。 */
    uint8_t history_reset = 0U; /* 视觉间隔过长时重建位置和差分历史。 */
    uint8_t target_changed = 0U; /* 目标改变时重置旧目标的控制状态。 */

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
        static_comp_reset();
        stop_reset();
        s_inited = 1U;
        first = 1U;
    }

    if ((s_valid_ms != 0U) &&
        ((uint32_t)(valid_ms - s_valid_ms) > HISTORY_TIMEOUT_MS))
    {
        kalman_init(&s_kf, KALMAN_Q, KALMAN_R);
        s_pid.integral = 0.0f;
        static_comp_reset();
        stop_reset();
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
        static_comp_reset();
        stop_reset();
        target_changed = 1U;
    }
    s_pid.target = target_x;
    error = target_x - x;
    delta_error = error - s_pid.last_err;
    pos_cm = ctrl_x_to_pos_cm(x);
    gain = position_gain(pos_cm);
    if ((first != 0U) || (history_reset != 0U) ||
        (target_changed != 0U))
    {
        /* 没有连续历史时不产生差分冲击。 */
        s_pid.last_err = error;
        delta_error = 0.0f;
    }
    abs_error = (error >= 0.0f) ? error : -error;
    abs_delta = (delta_error >= 0.0f) ? delta_error : -delta_error;
    stop_ready = ((abs_error < STOP_ERR_PX) &&
                  (abs_delta < STATIC_DERR_PX));

    /* 跨过目标后清除旧方向积分，避免残余推力继续扩大过冲。 */
    if ((s_stop_active == 0U) &&
        (((error > 0.0f) && (s_pid.last_err < 0.0f)) ||
         ((error < 0.0f) && (s_pid.last_err > 0.0f))))
    {
        s_pid.integral = 0.0f;
    }

    if (s_stop_active != 0U)
    {
        if (abs_error < STOP_OUT_PX)
        {
            pid_stop(&s_pid, error);
            req = s_cc_last;
            cc = s_cc_last;
            print_frame(input_x, x, target_x, error, delta_error, gain,
                        req, cc, 1U);
            return VISION_SERVO_REACHED;
        }
        stop_reset();
        static_comp_reset();
        s_static_cooldown = STATIC_COOLDOWN_FRAMES;
        s_pid.last_err = error;
        delta_error = 0.0f;
    }
    else if (stop_ready != 0U)
    {
        if (s_stop_n < STOP_FRAMES)
        {
            s_stop_n++;
        }
        if (s_stop_n >= STOP_FRAMES)
        {
            s_stop_active = 1U;
            pid_stop(&s_pid, error);
            req = s_cc_last;
            cc = s_cc_last;
            print_frame(input_x, x, target_x, error, delta_error, gain,
                        req, cc, 1U);
            return VISION_SERVO_REACHED;
        }
    }
    else
    {
        s_stop_n = 0U;
    }

    req = pid_to_cc(&s_pid, x, gain);
    cc = set_cc(req);
    print_frame(input_x, x, target_x, error, delta_error, gain,
                req, cc, 0U);
    return VISION_SERVO_MOVING;
}
