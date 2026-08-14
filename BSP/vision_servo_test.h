#ifndef __VISION_SERVO_TEST_H__
#define __VISION_SERVO_TEST_H__

#define SV_TEST_FIRST_TARGET_CM  (5.0f)  /* 第一阶段目标位置，单位 cm。 */
#define SV_TEST_SECOND_TARGET_CM (-5.0f) /* 第二阶段目标位置，单位 cm。 */
#define SV_TEST_STABLE_FRAMES    (1U)    /* 到位一个有效帧后立即切换阶段。 */
#define SV_TEST_TIMEOUT_MS       (5000U) /* 两阶段动作的总时间要求，单位 ms。 */
#define SV_LINK_DEBUG     (1U)   /* 每秒输出一次视觉链路和控制入口状态。 */
#define SV_FRAME_DEBUG    (1U)   /* 每个有效视觉帧输出一次钢珠位置和控制量。 */

typedef enum
{
    VISION_SERVO_NO_FRAME = 0, /* 当前没有新的有效视觉帧。 */
    VISION_SERVO_MOVING,       /* 已处理新帧，但钢珠尚未到位。 */
    VISION_SERVO_HOLDING       /* 已处理新帧，钢珠处于到位区。 */
} VisionServoResult;

void Vision_Servo_Test_Init(void);
VisionServoResult Vision_Servo_Test_Update(float target_cm);

#endif
