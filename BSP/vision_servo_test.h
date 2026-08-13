#ifndef __VISION_SERVO_TEST_H__
#define __VISION_SERVO_TEST_H__

#include <stdint.h>

/* 以下角度是舵机控制输入角度，不是摆杆实际倾角。 */
#define SV_X0             (152.0f) /* 摆杆中心对应图像横坐标。 */
#define SV_CM_PX          (0.082f) /* 图像坐标换算位置的标定系数，单位 cm/px。 */
#define SV_X_REF          (213.0f) /* 钢珠固定目标横坐标，单位 px。 */
#define SV_POS_REF        ((SV_X_REF - SV_X0) * SV_CM_PX) /* 固定目标位置，单位 cm。 */

#define SV_ANG0           (39.0f)  /* 当前机构实测平衡角，单位度。 */
#define SV_ANG_MIN        (0.0f)   /* 实测机械允许的最小输入角。 */
#define SV_ANG_MAX        (60.0f)  /* 实测机械允许的最大输入角。 */
#define SV_DIR            (1.0f)   /* 已实测：角度增大时钢珠向正方向移动。 */

/* 位置和速度控制参数。 */
#define SV_KP             (0.75f)  /* 位置比例系数，单位度/cm。 */
#define SV_KD             (1.10f)  /* 速度制动系数，单位度/(cm/s)。 */

/* 以下为静摩擦补偿、速度和舵机输出保护参数。 */
#define SV_ERR_TOL        (0.30f)  /* 绝对误差小于等于该值时视为到位，单位 cm。 */
#define SV_PUSH_MAX       (14.0f)  /* 静止时允许的最大临时推力角，单位度。 */
#define SV_PUSH_RATE      (10.0f)  /* 静止时临时推力的建立速度，单位度/s。 */
#define SV_PUSH_VEL_MAX   (0.50f)  /* 低于该速度才启用临时推力，单位 cm/s。 */
#define SV_PUSH_WAIT_MS   (500U)   /* 连续低速达到该时间后才建立临时推力。 */
#define SV_VEL_ALPHA      (0.35f)  /* 速度低通滤波系数。 */
#define SV_VEL_DT_MAX_MS  (1000U)  /* 可用于估算平均速度的最大有效帧间隔，单位 ms。 */

#define SV_STEP_UP        (6.0f)   /* 相邻视觉帧允许的最大增角，单位度。 */
#define SV_STEP_DN        (6.0f)   /* 相邻视觉帧允许的最大减角，单位度。 */
#define SV_LOST_HOLD_MS   (500U)   /* 视觉短暂失效时保持上一角度的最长时间，单位 ms。 */
#define SV_DBG            (1U)     /* 串口调试开关。 */

void Vision_Servo_Test_Init(void);
void Vision_Servo_Test_Reset(void);
void Vision_Servo_Test_SetTarget(float target_cm);
void Vision_Servo_Test_Update(void);
void Vision_Servo_Test_Stop(void);
float Vision_Servo_Test_GetTarget(void);
float Vision_Servo_Test_GetPosition(void);
float Vision_Servo_Test_GetVelocity(void);
uint8_t Vision_Servo_Test_HasPosition(void);

#endif
