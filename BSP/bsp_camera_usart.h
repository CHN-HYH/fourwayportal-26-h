#ifndef __BSP_CAMERA_USART_H_
#define __BSP_CAMERA_USART_H_

#include "ti_msp_dl_config.h"

#define CAMERA_UART_RX_BUFFER_LEN        (128U)
#define CAMERA_UART_FRAME_LEN             (8U)
#define CAMERA_VISION_IMAGE_WIDTH         (320U)
#define CAMERA_VISION_TIMEOUT_MS          (250U)

typedef struct
{
    uint8_t valid;
    uint8_t flags;
    uint8_t seq;
    uint16_t x;
    uint8_t width;
    uint32_t last_rx_ms;
    uint32_t last_valid_ms;
    uint32_t frame_count;
    uint32_t valid_frame_count;
    uint32_t checksum_error_count;
    uint32_t seq_lost_count;
    uint32_t seq_repeat_count;
    uint32_t seq_discontinuity_count;
    uint32_t range_error_count;
    uint32_t rx_overflow_count;
} CameraVisionState;

/* 摄像头视觉状态仅在主循环中更新，后续控制可直接读取。 */
extern volatile CameraVisionState vision;

/* 初始化摄像头接收、帧解析状态和 TIMG12 的 1 ms 时间基。 */
void Camera_Vision_Init(void);
/* 在主循环中调用，解析 UART3 接收环形缓冲区中的数据。 */
void Camera_Vision_Process(void);
uint32_t Camera_Vision_GetTimeMs(void);
uint8_t Camera_Vision_IsUsable(void);
uint8_t Camera_Vision_IsLinkAlive(uint32_t timeout_ms);
void Camera_Uart_ClearBuffer(void);

#endif