#include "vision_servo_test.h"
#include "bsp_camera_usart.h"
#include "servo.h"
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
static uint8_t s_static_active;      /* 本帧是否启用最小起步倾角。 */
static uint8_t s_still_n;            /* 目标外连续静止的有效帧数。 */
static uint8_t s_pulse_n;            /* 本次静摩擦补偿还需输出的有效帧数。 */
static uint8_t s_pulse_used;         /* 当前静止阶段是否已经输出过补偿脉冲。 */
static uint8_t s_stop_active;        /* PID 是否已经因连续到位而停止。 */
static uint8_t s_stop_n;             /* 连续满足停止误差的有效帧数。 */

static void pid_init(pid_t *pid)
{
    memset(pid, 0, sizeof(*pid));
}

static void static_comp_reset(void)
{
    s_static_active = 0U;
    s_still_n = 0U;
    s_pulse_n = 0U;
    s_pulse_used = 0U;
}

static void stop_reset(void)
{
    s_stop_active = 0U;
    s_stop_n = 0U;
}

/* 停止 PID 后清除内部项并保留当前舵机 CC。 */
static void pid_stop(pid_t *pid, float err, float delta_error)
{
    pid->err = err;
    pid->last_err = err;
    pid->integral = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->delta_err = delta_error;
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

static float speed_scale(float abs_delta)
{
    float max_delta = INTEGRAL_DERR_PX * 3.0f;

    if (abs_delta <= INTEGRAL_DERR_PX)
    {
        return 1.0f;
    }
    if (abs_delta >= max_delta)
    {
        return 0.0f;
    }
    return (max_delta - abs_delta) /
           (max_delta - INTEGRAL_DERR_PX);
}

static void pid_terms(pid_t *pid,
                      float gain,
                      float d_scale,
                      uint8_t approaching)
{
    float d_limit;

    pid->p_term = PID_KP * pid->err * gain;
    pid->i_term = PID_KI * pid->integral * gain;
    pid->d_term = PID_KD * pid->delta_err * d_scale * gain;
    if (approaching != 0U)
    {
        d_limit = fabsf(pid->p_term + pid->i_term) + D_REVERSE_MARGIN;
        if (pid->d_term > d_limit)
        {
            pid->d_term = d_limit;
        }
        if (pid->d_term < -d_limit)
        {
            pid->d_term = -d_limit;
        }
    }
    pid->raw = pid->p_term + pid->i_term + pid->d_term;
}

/* 静止误差越大，最低起步倾角从近端值平滑增加到远端值。 */
static float static_request(float error, float abs_error)
{
    float offset = STATIC_OFFSET_FAR;

    if (abs_error < STOP_OUT_PX)
    {
        offset = STATIC_OFFSET_NEAR +
                 (STATIC_OFFSET_FAR - STATIC_OFFSET_NEAR) *
                 (abs_error - STOP_ERR_PX) /
                 (STOP_OUT_PX - STOP_ERR_PX);
    }
    return PWM_CC_CENTER + ((error > 0.0f) ? offset : -offset);
}

/* 计算位置式 PID 请求；静摩擦脉冲不阻断积分累计。 */
static float pid_request(pid_t *pid,
                         float error,
                         float delta_error,
                         float gain,
                         uint8_t static_ready)
{
    float abs_error;
    float abs_delta;
    float i_scale;
    float d_scale;
    float old_integral;
    float req;
    float static_cc;
    uint8_t approaching;
    uint8_t integrated = 0U;
    uint8_t static_apply = 0U;

    pid->err = error;
    pid->delta_err = delta_error;
    abs_error = (error >= 0.0f) ? error : -error;
    abs_delta = (delta_error >= 0.0f) ? delta_error : -delta_error;
    distance_scale(abs_error, &i_scale, &d_scale);
    i_scale *= speed_scale(abs_delta);
    approaching = (uint8_t)((error * delta_error) < 0.0f);
    if (abs_error >= DIST_FAR_PX)
    {
        pid->integral = 0.0f;
    }
    old_integral = pid->integral;

    /* 先用当前积分判断 PID 是否已经达到最小起步倾角。 */
    s_static_active = 0U;
    if (static_ready != 0U)
    {
        if (approaching == 0U)
        {
            d_scale = 1.0f;
        }
        pid_terms(pid, gain, d_scale, approaching);
        req = PWM_CC_CENTER + pid->raw * PWM_CC_SCALE;
        static_cc = static_request(error, abs_error);
        if ((error > 0.0f) && (req < static_cc))
        {
            s_static_active = 1U;
            static_apply = 1U;
        }
        if ((error < 0.0f) && (req > static_cc))
        {
            s_static_active = 1U;
            static_apply = 1U;
        }
    }

    if ((abs_error < DIST_FAR_PX) && (i_scale > 0.0f))
    {
        pid->integral += error * i_scale;
        integrated = 1U;
    }
    if (pid->integral > PID_I_LIMIT)
    {
        pid->integral = PID_I_LIMIT;
    }
    if (pid->integral < -PID_I_LIMIT)
    {
        pid->integral = -PID_I_LIMIT;
    }

    /* 钢珠偏离目标时使用完整 D，接近目标时采用距离阶段权重。 */
    if (approaching == 0U)
    {
        d_scale = 1.0f;
    }
    pid_terms(pid, gain, d_scale, approaching);

    /* 机械限幅方向上的积分不再继续累加。 */
    req = PWM_CC_CENTER + pid->raw * PWM_CC_SCALE;
    if ((integrated != 0U) &&
        (((req > PWM_CC_MAX) && (error > 0.0f)) ||
         ((req < PWM_CC_MIN) && (error < 0.0f))))
    {
        pid->integral = old_integral;
        pid_terms(pid, gain, d_scale, approaching);
        req = PWM_CC_CENTER + pid->raw * PWM_CC_SCALE;
    }

    if (static_apply != 0U)
    {
        req = static_cc;
    }
    pid->last_err = pid->err;
    return req;
}

/* 目标外连续静止时准备最小起步倾角。 */
static uint8_t static_comp_update(float abs_error, float abs_delta)
{
    if ((abs_error >= STOP_ERR_PX) &&
        (abs_delta < STILL_DERR_PX))
    {
        if (s_still_n < STATIC_WAIT_FRAMES)
        {
            s_still_n++;
        }
    }
    else
    {
        static_comp_reset();
    }
    if ((s_pulse_used == 0U) && (s_still_n >= STATIC_WAIT_FRAMES))
    {
        s_pulse_used = 1U;
        s_pulse_n = STATIC_PULSE_FRAMES;
    }
    if (s_pulse_n == 0U)
    {
        return 0U;
    }
    s_pulse_n--;
    return 1U;
}

/* 按机械限幅和单帧变化限制写入舵机 CC。 */
static float set_cc(float req)
{
    float cc = req;
    float delta;
    uint16_t pwm;

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

    pwm = (uint16_t)(cc + 0.5f);
    Servo_SetCc(pwm);
    s_cc_last = (float)pwm;
    return s_cc_last;
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
    printf("[SERVO] build=%s %s pin=PB8 timer=TIMA0_CCP0 "
           "cc=%.0f kp=%.4f ki=%.4f kd=%.4f dmargin=%.2f "
           "reach=%.1f/%u static=%.1f-%.1f/%u*%u\r\n",
           __DATE__,
           __TIME__,
           (double)PWM_CC_CENTER,
           (double)PID_KP,
           (double)PID_KI,
           (double)PID_KD,
           (double)D_REVERSE_MARGIN,
           (double)REACH_ERR_PX,
           (unsigned int)STOP_FRAMES,
           (double)STATIC_OFFSET_NEAR,
           (double)STATIC_OFFSET_FAR,
           (unsigned int)STATIC_WAIT_FRAMES,
           (unsigned int)STATIC_PULSE_FRAMES);
#endif
}

VisionServoResult Vision_Servo_Test_Update(float target_cm, uint8_t hold)
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
    uint8_t moving;
    uint8_t static_ready;
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
    /* 途经点只按位置误差快速切换，最终保持还需确认钢珠已减速。 */
    stop_ready = (uint8_t)(abs_error < REACH_ERR_PX);
    moving = (uint8_t)(abs_delta >= STOP_MOVE_DERR_PX);
    if (hold != 0U)
    {
        stop_ready = (uint8_t)(stop_ready && (moving == 0U));
    }

    if ((hold != 0U) && (s_stop_active != 0U))
    {
        /* 回差区内且钢珠没有重新运动时，继续保持当前舵机位置。 */
        if ((abs_error < STOP_OUT_PX) && (moving == 0U))
        {
            pid_stop(&s_pid, error, delta_error);
            req = s_cc_last;
            cc = s_cc_last;
            print_frame(input_x, x, target_x, error, delta_error, gain,
                        req, cc, 1U);
            return VISION_SERVO_REACHED;
        }
        stop_reset();
        static_comp_reset();
    }

    /* 跨过目标后清除旧方向积分，避免残余推力继续扩大过冲。 */
    if (((error > 0.0f) && (s_pid.last_err < 0.0f)) ||
        ((error < 0.0f) && (s_pid.last_err > 0.0f)))
    {
        s_pid.integral = 0.0f;
    }

    if (stop_ready != 0U)
    {
        if (s_stop_n < STOP_FRAMES)
        {
            s_stop_n++;
        }
        if (s_stop_n >= STOP_FRAMES)
        {
            if (hold != 0U)
            {
                s_stop_active = 1U;
                pid_stop(&s_pid, error, delta_error);
                req = s_cc_last;
                cc = s_cc_last;
            }
            else
            {
                req = PWM_CC_CENTER + s_pid.raw * PWM_CC_SCALE;
                cc = s_cc_last;
            }
            print_frame(input_x, x, target_x, error, delta_error, gain,
                        req, cc, 1U);
            return VISION_SERVO_REACHED;
        }
    }
    else
    {
        s_stop_n = 0U;
    }

    static_ready = static_comp_update(abs_error, abs_delta);
    req = pid_request(&s_pid,
                      error,
                      delta_error,
                      gain,
                      static_ready);
    cc = set_cc(req);
    print_frame(input_x, x, target_x, error, delta_error, gain,
                req, cc, 0U);
    return VISION_SERVO_MOVING;
}
