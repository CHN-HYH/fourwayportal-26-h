/* 修复后的 pid_request 函数 - 主要改动：
 * 1. 只计算一次 PID terms
 * 2. 统一 d_scale 处理逻辑
 * 3. 先判断静摩擦，再更新积分
 * 4. 修复积分抗饱和逻辑
 */

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
    float integral_increment;
    uint8_t approaching;
    uint8_t static_apply = 0U;

    pid->err = error;
    pid->delta_err = delta_error;
    abs_error = (error >= 0.0f) ? error : -error;
    abs_delta = (delta_error >= 0.0f) ? delta_error : -delta_error;

    /* 计算距离和速度的缩放因子 */
    distance_scale(abs_error, &i_scale, &d_scale);
    i_scale *= speed_scale(abs_delta);

    /* 判断是否正在接近目标 */
    approaching = (uint8_t)((error * delta_error) < 0.0f);

    /* 偏离目标时使用完整 D 增强响应；接近时用距离权重提供阻尼 */
    if (approaching == 0U)
    {
        d_scale = 1.0f;
    }

    /* 远距离清零积分 */
    if (abs_error >= DIST_FAR_PX)
    {
        pid->integral = 0.0f;
    }

    /* 使用当前积分计算 PID（用于判断是否需要静摩擦补偿）*/
    pid_terms(pid, gain, d_scale, approaching);
    req = PWM_CC_CENTER + pid->raw * PWM_CC_SCALE;

    /* 检查是否需要启用最小起步倾角 */
    s_static_active = 0U;
    if (static_ready != 0U)
    {
        static_cc = static_request(error, abs_error);

        /* error > 0 需要向右，但 PID 输出不足，应用静摩擦补偿 */
        if ((error > 0.0f) && (req < static_cc))
        {
            s_static_active = 1U;
            static_apply = 1U;
        }
        /* error < 0 需要向左，但 PID 输出不足，应用静摩擦补偿 */
        else if ((error < 0.0f) && (req > static_cc))
        {
            s_static_active = 1U;
            static_apply = 1U;
        }
    }

    /* 更新积分：只在中近距离且速度适中时累加 */
    integral_increment = 0.0f;
    if ((abs_error < DIST_FAR_PX) && (i_scale > 0.0f))
    {
        integral_increment = error * i_scale;
        old_integral = pid->integral;
        pid->integral += integral_increment;

        /* 积分限幅 */
        if (pid->integral > PID_I_LIMIT)
        {
            pid->integral = PID_I_LIMIT;
        }
        if (pid->integral < -PID_I_LIMIT)
        {
            pid->integral = -PID_I_LIMIT;
        }

        /* 重新计算 PID（积分改变了）*/
        pid_terms(pid, gain, d_scale, approaching);
        req = PWM_CC_CENTER + pid->raw * PWM_CC_SCALE;

        /* 机械限幅方向上的积分不再继续累加 - 使用积分增量的符号判断 */
        if (((req > PWM_CC_MAX) && (integral_increment > 0.0f)) ||
            ((req < PWM_CC_MIN) && (integral_increment < 0.0f)))
        {
            /* 回退积分并重新计算 */
            pid->integral = old_integral;
            pid_terms(pid, gain, d_scale, approaching);
            req = PWM_CC_CENTER + pid->raw * PWM_CC_SCALE;
        }
    }

    /* 如果需要静摩擦补偿，覆盖 PID 输出 */
    if (static_apply != 0U)
    {
        req = static_cc;
    }

    pid->last_err = pid->err;
    return req;
}
