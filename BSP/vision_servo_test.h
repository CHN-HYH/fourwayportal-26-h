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

/* 简化 PID 参数 - 针对 0→5→-5cm 在 5 秒内完成 */
#define PID_KP             (0.060f)  /* 比例系数，提高加快0→5响应 */
#define PID_KI             (0.0080f) /* 积分系数，适中 */
#define PID_KD             (0.800f)  /* 差分系数，强阻尼抑制振荡 */
#define PID_I_LIMIT        (500.0f)  /* 积分限幅进一步降低 */
#define DIST_MID_PX        (70.0f)   /* 积分启用边界 */
#define STOP_ERR_PX        (6.0f)    /* 积分停止边界 */
#define D_REVERSE_MARGIN   (0.60f)   /* D 项反向制动余量增加 */

/* 已移除复杂的控制器坐标系统，直接使用视觉像素 */

/* 摄像头输入坐标到实际位置的标定。 */
#define INPUT_CENTER_X     (150.0f)  /* 钢珠位于摆杆中心时的摄像头像素坐标。 */
#define INPUT_CM_PX_POS    (0.0863f) /* 正半轴每个摄像头像素对应的距离，单位 cm/px。 */
#define INPUT_CM_PX_NEG    (0.0835f) /* 负半轴：0.092/1.1=0.0836，修正 pos 偏大问题。 */

/* 舵机输出参数 */
#define PWM_CC_CENTER      (60.0f)  /* 摆杆水平时的舵机比较值 */
#define PWM_CC_MIN         (50.0f)  /* 舵机最小比较值 */
#define PWM_CC_MAX         (70.0f)  /* 舵机最大比较值 */
#define PWM_CC_SCALE       (1.15f)  /* PID 输出缩放，提高加快响应 */
#define REACH_ERR_PX       (18.0f)  /* 到位判断阈值进一步放宽，约1.5cm */
#define RATE_LIMIT         (4.0f)   /* 每帧最大变化，加快 */
#define HISTORY_TIMEOUT_MS (300U)   /* 视觉超时阈值 */

/* 一维卡尔曼滤波参数。 */
#define KALMAN_Q           (1.0f) /* 过程噪声协方差。 */
#define KALMAN_R           (0.3f) /* 测量噪声协方差。 */

/* 已移除位置增益，简化控制逻辑 */

typedef enum
{
    VISION_SERVO_NO_FRAME = 0, /* 当前没有新的有效视觉帧。 */
    VISION_SERVO_MOVING,       /* 已处理新帧，但钢珠尚未到位。 */
    VISION_SERVO_REACHED       /* 已处理新帧，钢珠处于到位区。 */
} VisionServoResult;

void Vision_Servo_Test_Init(void);
VisionServoResult Vision_Servo_Test_Update(float target_cm, uint8_t hold);

#endif
