#ifndef __VISION_SERVO_TEST_H__
#define __VISION_SERVO_TEST_H__

/* K4 启动的钢珠位置测试参数。 */
#define SV_TEST_ORIGIN_TARGET_CM (0.0f)  /* 上电等待和序列起点位置，单位 cm。 */
#define SV_TEST_FIRST_TARGET_CM  (5.0f)  /* 第一段目标位置，单位 cm。 */
#define SV_TEST_SECOND_TARGET_CM (-5.0f) /* 第二段目标位置，单位 cm。 */
#define SV_STABLE_FRAMES         (6U)    /* 连续到位的有效帧数。 */
#define SV_TEST_TIMEOUT_MS       (5000U) /* K4 启动后整段动作的总时间要求，单位 ms。 */

/* 串口调试输出开关。 */
#define SV_LINK_DEBUG     (1U)   /* 每秒输出一次视觉链路和控制入口状态。 */
#define SV_FRAME_DEBUG    (1U)   /* 每个有效视觉帧输出一次钢珠位置和控制量。 */

/* 位置式 PID 参数。 */
#define PID_KP             (0.028f)  /* 比例系数，决定位置误差对应的基本控制量。 */
#define PID_KI             (0.0047f) /* 积分系数，用于逐步克服静摩擦和稳态误差。 */
#define PID_KD             (0.48f)   /* 差分系数，用于抑制快速接近目标时的过冲。 */
#define PID_I_LIMIT        (1300.0f) /* 积分累计绝对值上限，防止积分持续饱和。 */

/* 控制器内部使用的标准图像坐标标定。 */
#define CTRL_CENTER_X      (360.0f) /* 摆杆物理中心对应的控制器横坐标。 */
#define CTRL_MIN_X         (99.0f)  /* 摆杆负方向端点对应的控制器横坐标。 */
#define CTRL_MAX_X         (599.0f) /* 摆杆正方向端点对应的控制器横坐标。 */
#define PIPE_HALF_CM       (12.5f)  /* 摆杆中心到单侧端点的距离，单位 cm。 */

/* 摄像头输入坐标到实际位置的标定。 */
#define INPUT_CENTER_X     (158.0f) /* 钢珠位于摆杆中心时的摄像头像素坐标。 */
#define INPUT_CM_PER_PX    (0.082f) /* 摄像头横坐标每像素对应的物理距离，单位 cm/px。 */

/* 舵机输出和控制保护参数。 */
#define PWM_CC_CENTER      (60.0f) /* 摆杆水平时的舵机比较值。 */
#define PWM_CC_MIN         (50.0f) /* 舵机允许输出的最小比较值。 */
#define PWM_CC_MAX         (70.0f) /* 舵机允许输出的最大比较值。 */
#define PWM_CC_SCALE       (0.80f) /* PID 输出换算为舵机比较值的比例。 */
#define DEADBAND_PX        (8.0f)  /* 到位判定及静摩擦补偿使用的误差阈值。 */
#define INTEGRAL_DERR_PX   (4.0f)  /* 允许累计积分的单帧误差变化上限。 */
#define STATIC_DERR_PX     (1.0f)  /* 判定钢珠接近静止的单帧误差变化上限。 */
#define STATIC_CC_POS      (65.0f) /* 正方向克服静摩擦所需的舵机比较值。 */
#define STATIC_CC_NEG      (54.0f) /* 负方向克服静摩擦所需的舵机比较值。 */
#define STATIC_PULSE_FRAMES   (5U) /* 单次静摩擦启动脉冲持续的有效帧数。 */
#define STATIC_COOLDOWN_FRAMES (4U) /* 两次静摩擦启动脉冲之间的冷却帧数。 */
#define RATE_LIMIT         (1.0f)  /* 舵机比较值每个有效帧允许的最大变化量。 */
#define HISTORY_TIMEOUT_MS (300U) /* 超过该时间未收到有效帧时重建控制历史。 */

/* 一维卡尔曼滤波参数。 */
#define KALMAN_Q           (1.0f) /* 过程噪声协方差。 */
#define KALMAN_R           (0.3f) /* 测量噪声协方差。 */

/* 根据钢珠到转轴的距离调整动态控制强度。 */
#define PIVOT_POS_CM       (12.0f) /* 摆杆转轴在位置坐标中的位置，单位 cm。 */
#define GAIN_REF_ARM_CM    (7.0f)  /* 计算位置增益时使用的参考力臂长度，单位 cm。 */
#define GAIN_MIN_ARM_CM    (1.0f)  /* 参与位置增益计算的最小力臂长度，单位 cm。 */
#define GAIN_SCALE_MIN     (0.55f) /* 位置增益的下限。 */
#define GAIN_SCALE_MAX     (1.10f) /* 位置增益的上限。 */

typedef enum
{
    VISION_SERVO_NO_FRAME = 0, /* 当前没有新的有效视觉帧。 */
    VISION_SERVO_MOVING,       /* 已处理新帧，但钢珠尚未到位。 */
    VISION_SERVO_REACHED       /* 已处理新帧，钢珠处于到位区。 */
} VisionServoResult;

void Vision_Servo_Test_Init(void);
VisionServoResult Vision_Servo_Test_Update(float target_cm);

#endif
