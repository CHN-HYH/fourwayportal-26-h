#ifndef __VISION_SERVO_TEST_H__
#define __VISION_SERVO_TEST_H__

/* 图像位置标定。 */
#define SV_X0             (152.0f) /* 摆杆中心对应的图像横坐标。 */
#define SV_CM_PX          (0.082f) /* 图像坐标换算位置的标定系数，单位 cm/px。 */
#define SV_X_REF          (213.0f) /* 钢珠固定目标横坐标，单位 px。 */
#define SV_POS_REF        ((SV_X_REF - SV_X0) * SV_CM_PX) /* 固定目标位置，单位 cm。 */

/* 舵机脉宽标定。 */
#define SV_PULSE_BASE     (933.0f)  /* PID 零输出对应的舵机基准脉宽，单位 us。 */
#define SV_PULSE_MIN      (500.0f)  /* 机械允许的最小高电平脉宽，单位 us。 */
#define SV_PULSE_MAX      (1166.0f) /* 机械允许的最大高电平脉宽，单位 us。 */
#define SV_DIR            (1.0f)    /* 脉宽增大时钢珠向图像正方向移动。 */

/* 像素域位置式 PID 参数，结构与原控球工程一致。 */
#define SV_KP             (0.025f)  /* 比例系数，控制主要响应力度。 */
#define SV_KI             (0.003f)  /* 积分系数，用于克服持续静摩擦。 */
#define SV_KD             (1.500f)  /* 差分系数，加强接近目标时的制动。 */
#define SV_I_MAX          (700.0f) /* 积分累计限幅，提供足够的静摩擦克服能力。 */
#define SV_OUT_MAX        (500.0f) /* 单帧 PID 输出限幅。 */
#define SV_OUT_SCALE_US   (20.0f)  /* PID 输出换算为脉宽偏移的系数，单位 us。 */

/* 位置滤波和输出保护参数。 */
#define SV_DEADBAND_PX    (3.0f)   /* 像素误差死区。 */
#define SV_KALMAN_Q       (1.0f)   /* 卡尔曼过程噪声。 */
#define SV_KALMAN_R       (0.30f)  /* 卡尔曼测量噪声。 */
#define SV_PULSE_STEP_US  (20.0f)  /* 相邻有效帧允许的最大脉宽变化，单位 us。 */
#define SV_DBG            (1U)     /* 串口调试开关。 */

void Vision_Servo_Test_Init(void);
void Vision_Servo_Test_Update(void);

#endif
