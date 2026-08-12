#ifndef __VISION_SERVO_TEST_H__
#define __VISION_SERVO_TEST_H__

/* 以下角度是舵机控制输入角度，不是摆杆实际倾角。 */
#define SV_X0          (152.0f)  /* 标定后的摆杆中心图像横坐标。 */
#define SV_CM_PX       (0.082f)  /* 实测位置标定系数，单位 cm/px。 */
#define SV_POS_REF     (5.0f)   /* 钢珠固定目标位置，单位 cm。 */
#define SV_ERR_TOL     (0.20f)   /* 目标允许误差范围，单位 cm。 */

#define SV_ANG0        (30.0f)   /* 实测摆杆水平时的舵机基准角。 */
#define SV_ANG_MIN     (0.0f)    /* 实测机械允许的最小输入角。 */
#define SV_ANG_MAX     (60.0f)   /* 实测机械允许的最大输入角。 */
#define SV_DIR         (1.0f)    /* 角度增大时钢珠向正方向移动。 */

#define SV_KP          (1.00f)   /* 位置误差到角度增量的比例系数。 */
#define SV_KI          (1.20f)   /* 静止且存在小误差时的积分系数。 */
#define SV_KD          (0.65f)   /* 钢珠速度的制动系数。 */
#define SV_I_MAX       (8.0f)    /* 积分项允许的最大角度增量，单位度。 */
#define SV_I_ERR_MAX   (4.00f)   /* 大于此误差不积分，避免远距离运动时积分饱和，单位 cm。 */
#define SV_I_VEL_MAX   (1.20f)   /* 速度低于此值才积分，单位 cm/s。 */

#define SV_VEL_A       (0.35f)   /* 速度低通滤波系数。 */
#define SV_STEP_UP     (2.0f)    /* 相邻视觉帧允许的最大增角，单位度。 */
#define SV_STEP_DN     (4.0f)    /* 相邻视觉帧允许的最大减角，用于超目标快速制动，单位度。 */
#define SV_LOST_HOLD_MS (500U)   /* 视觉短暂失效时保持上一角度的最长时间，单位 ms。 */
#define SV_DBG         (1U)      /* 串口调试开关，闭环控制时保持关闭。 */

void Vision_Servo_Test_Init(void);
void Vision_Servo_Test_Update(void);

#endif
