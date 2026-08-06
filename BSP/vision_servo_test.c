#include "vision_servo_test.h"
#include "bsp_camera_usart.h"
#include "servo.h"
#include <stdint.h>
#include <stdio.h>

#define VISION_SERVO_CENTER_X       (160)
#define VISION_SERVO_CENTER_ANGLE   (90)
#define VISION_SERVO_KP              (0.20f)
#define VISION_SERVO_DIRECTION      (1.0f)
#define VISION_SERVO_UPDATE_MS      (20U)

static uint32_t g_vision_servo_last_update_ms = 0U;
static uint16_t g_vision_servo_last_angle = 0xFFFFU;

void Vision_Servo_Test_Init(void)
{
    Servo_Init();
    Servo_SetAngle((uint16_t)VISION_SERVO_CENTER_ANGLE);
}

void Vision_Servo_Test_Update(void)
{
    uint32_t now_ms;
    int32_t error_pixel = 0;
    int32_t target_angle;
    float correction;
    uint16_t angle;
    uint8_t vision_usable;

    now_ms = Camera_Vision_GetTimeMs();
    if ((uint32_t)(now_ms - g_vision_servo_last_update_ms) < VISION_SERVO_UPDATE_MS)
    {
        return;
    }
    g_vision_servo_last_update_ms = now_ms;

    vision_usable = Camera_Vision_IsUsable();
    if (vision_usable != 0U)
    {
        error_pixel = (int32_t)vision.x - (int32_t)VISION_SERVO_CENTER_X;
        correction = (float)error_pixel * VISION_SERVO_KP * VISION_SERVO_DIRECTION;
        target_angle = (int32_t)VISION_SERVO_CENTER_ANGLE + (int32_t)correction;

        if (target_angle < 0)
        {
            target_angle = 0;
        }
        else if (target_angle > 180)
        {
            target_angle = 180;
        }

        angle = (uint16_t)target_angle;
    }
    else
    {
        angle = (uint16_t)VISION_SERVO_CENTER_ANGLE;
    }

    if (angle != g_vision_servo_last_angle)
    {
        Servo_SetAngle(angle);
        if (vision_usable != 0U)
        {
            printf("[SERVO] x=%u error=%d angle=%u\r\n",
                (unsigned int)vision.x,
                (int)error_pixel,
                (unsigned int)angle);
        }
        else
        {
            printf("[SERVO] vision_invalid angle=%u\r\n",
                (unsigned int)angle);
        }
        g_vision_servo_last_angle = angle;
    }
}
