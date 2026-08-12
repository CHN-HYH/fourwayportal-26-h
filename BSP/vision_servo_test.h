#ifndef __VISION_SERVO_TEST_H__
#define __VISION_SERVO_TEST_H__

/* 以下角度是舵机控制输入角度，不是摆杆实际倾角。 */
#define SV_X0          (152.0f)  /* 标定后的摆杆中心图像横坐标。 */
#define SV_CM_PX       (0.082f)  /* 实测位置标定系数，单位 cm/px。 */
#define SV_POS_REF     (12.0f)   /* 钢珠固定目标位置，单位 cm。 */

#define SV_ANG0        (30.0f)   /* 实测摆杆水平时的舵机基准角。 */
#define SV_ANG_HOLD    (42.0f)   /* 钢珠在 +12 cm 处稳定时实测所需的保持角。 */
#define SV_ANG_MIN     (10.0f)   /* 实测机械允许的最小输入角。 */
#define SV_ANG_MAX     (50.0f)   /* 舵机允许的最大输入角。 */
#define SV_ANG_RUN_MAX (45.0f)   /* 定点运行阶段的最大角度，限制钢珠最高速度。 */
#define SV_DIR         (1.0f)    /* 角度增大时钢珠向正方向移动。 */

#define SV_KP          (1.00f)   /* 位置误差到角度增量的比例系数。 */
#define SV_KD          (0.35f)   /* 钢珠速度的制动系数。 */
#define SV_VEL_A       (0.35f)   /* 速度低通滤波系数。 */
#define SV_STEP_MAX    (2.0f)    /* 相邻视觉帧允许的最大舵机角度变化，单位度。 */
#define SV_DBG         (1U)      /* 串口调试开关，闭环控制时保持关闭。 */

void Vision_Servo_Test_Init(void);
void Vision_Servo_Test_Update(void);

#endif
