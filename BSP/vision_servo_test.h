#ifndef __VISION_SERVO_TEST_H__
#define __VISION_SERVO_TEST_H__

#include <stdint.h>

/* K4 启动的钢珠位置测试参数。 */
#define SV_TEST_ORIGIN_TARGET_CM (0.0f)  /* 上电等待和序列起点位置，单位 cm。 */
#define SV_TEST_FIRST_TARGET_CM  (5.0f)  /* 第一段目标位置，单位 cm。 */
#define SV_TEST_SECOND_TARGET_CM (-5.0f) /* 第二段目标位置，单位 cm。 */
#define SV_TEST_TIMEOUT_MS       (5000U) /* K4 启动后整段动作的总时间要求，单位 ms。 */

/* 串口调试输出开关。 */
#define SV_LINK_DEBUG     (1U)   /* 启动时输出编译版本和主要控制参数。 */
#define SV_FRAME_DEBUG    (1U)   /* 每个有效视觉帧输出一次钢珠位置和控制量。 */

/* 位置式 PID 参数。 */
#define PID_KP             (0.035f)  /* 比例系数，决定位置误差对应的基本控制量。 */
#define PID_KI             (0.0040f) /* 积分系数，只补偿低速稳态误差。 */
#define PID_KD             (0.560f)  /* 差分系数，用于抑制运动过冲。 */
#define PID_I_LIMIT        (1300.0f) /* 积分累计绝对值上限。 */
#define DIST_FAR_PX        (185.0f)  /* 远距离阶段边界，积分从零开始介入。 */
#define DIST_MID_PX        (80.0f)   /* 中距离阶段边界，积分完全生效。 */
#define DIST_NEAR_PX       (25.0f)   /* 近距离阶段边界，D 完全生效。 */
#define D_MIN_SCALE        (0.45f)   /* 大误差接近目标时的最小 D 比例。 */
#define D_REVERSE_MARGIN   (0.45f)   /* D 抵消 P/I 后允许的最大反向制动量。 */

/* 控制器内部使用的标准图像坐标标定。 */
#define CTRL_CENTER_X      (360.0f) /* 摆杆物理中心对应的控制器横坐标。 */
#define CTRL_MIN_X         (99.0f)  /* 摆杆负方向端点对应的控制器横坐标。 */
#define CTRL_MAX_X         (599.0f) /* 摆杆正方向端点对应的控制器横坐标。 */
#define PIPE_HALF_CM       (12.5f)  /* 摆杆中心到单侧端点的距离，单位 cm。 */

/* 摄像头输入坐标到实际位置的标定。 */
#define INPUT_CENTER_X     (150.0f)  /* 钢珠位于摆杆中心时的摄像头像素坐标。 */
#define INPUT_CM_PX_POS    (0.0863f) /* 正半轴每个摄像头像素对应的距离，单位 cm/px。 */
#define INPUT_CM_PX_NEG    (0.0837f) /* 负半轴每个摄像头像素对应的距离，单位 cm/px。 */

/* 舵机输出和控制保护参数。 */
#define PWM_CC_CENTER      (60.0f)  /* 摆杆水平时的舵机比较值。 */
#define PWM_CC_MIN         (50.0f)  /* 舵机允许输出的最小比较值。 */
#define PWM_CC_MAX         (70.0f)  /* 舵机允许输出的最大比较值。 */
#define PWM_CC_SCALE       (0.80f)  /* PID 输出换算为舵机比较值的比例。 */
#define STOP_ERR_PX        (5.0f)   /* 静摩擦补偿开始介入的误差阈值。 */
#define REACH_ERR_PX       (10.0f)  /* 连续判定到位的绝对误差阈值。 */
#define STOP_OUT_PX        (10.0f)  /* PID 停止后恢复控制的误差边界。 */
#define STOP_MOVE_DERR_PX  (2.0f)   /* 停止后判定钢珠重新运动的变化阈值。 */
#define STOP_FRAMES        (2U)     /* 连续满足到位条件的有效帧数。 */
#define INTEGRAL_DERR_PX   (4.0f)   /* 全量积分速度上限，至三倍时平滑降为零。 */
#define STILL_DERR_PX      (1.0f)   /* 到位和静摩擦共用的静止变化阈值。 */
#define STATIC_WAIT_FRAMES (3U)     /* 连续静止后启用最小起步倾角的帧数。 */
#define STATIC_PULSE_FRAMES (2U)    /* 静摩擦补偿单次脉冲的有效帧数。 */
#define STATIC_OFFSET_NEAR (3.0f)   /* 刚离开到位区时的最小 CC 偏移。 */
#define STATIC_OFFSET_FAR  (5.0f)   /* 达到回差外侧后的最大 CC 偏移。 */
#define RATE_LIMIT         (2.0f)   /* 舵机比较值每个有效帧允许的最大变化量。 */
#define HISTORY_TIMEOUT_MS (300U)   /* 超过该时间未收到有效帧时重建控制历史。 */

/* 一维卡尔曼滤波参数。 */
#define KALMAN_Q           (1.0f) /* 过程噪声协方差。 */
#define KALMAN_R           (0.3f) /* 测量噪声协方差。 */

/* 根据钢珠到转轴的距离调整动态控制强度。 */
#define PIVOT_POS_CM       (12.0f) /* 摆杆转轴在位置坐标中的位置，单位 cm。 */
#define GAIN_REF_ARM_CM    (5.5f)  /* 计算位置增益时使用的参考力臂长度，单位 cm。 */
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
VisionServoResult Vision_Servo_Test_Update(float target_cm, uint8_t hold);

#endif
