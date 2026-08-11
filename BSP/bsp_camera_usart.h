#ifndef __BSP_CAMERA_USART_H_
#define __BSP_CAMERA_USART_H_

#include "ti_msp_dl_config.h"

#define CAMERA_UART_RX_BUFFER_LEN        (128U)  /* UART3 环形接收缓冲区字节数。 */
#define CAMERA_UART_FRAME_LEN             (8U)    /* 摄像头协议的完整帧长度。 */
#define CAMERA_VISION_IMAGE_WIDTH         (320U)  /* 图像横向像素数。 */
#define CAMERA_VISION_TIMEOUT_MS          (250U)  /* 有效目标的超时阈值，单位 ms。 */

typedef struct
{
    uint8_t valid;        /* 最新帧是否包含有效目标。 */
    uint8_t flags;        /* 摄像头协议的标志字节。 */
    uint8_t seq;          /* 最新帧序号。 */
    uint16_t x;           /* 目标横向坐标，单位 px。 */
    uint8_t width;        /* 目标宽度，单位 px。 */
    uint32_t rx_ms;       /* 最近正确帧的接收时刻，单位 ms。 */
    uint32_t valid_ms;    /* 最近有效目标帧的接收时刻，单位 ms。 */
    uint32_t frame_n;     /* 校验和正确的帧总数。 */
    uint32_t valid_n;     /* 有效目标帧总数。 */
    uint32_t sum_err;     /* 校验和错误帧数。 */
    uint32_t seq_lost;    /* 推断丢失的序号数量。 */
    uint32_t seq_repeat;  /* 重复序号帧数。 */
    uint32_t seq_break;   /* 倒退或异常跳变的序号次数。 */
    uint32_t range_err;   /* 坐标越界帧数。 */
    uint32_t rx_overflow; /* UART3 环形缓冲区溢出次数。 */
} CameraVisionState;

/* 摄像头视觉状态仅在主循环中更新，后续控制可直接读取。 */
extern volatile CameraVisionState vision;

/* 初始化摄像头接收、帧解析状态和 TIMG12 的 1 ms 时间基。 */
void Camera_Vision_Init(void);
/* 在主循环中调用，解析 UART3 接收环形缓冲区中的数据。 */
void Camera_Vision_Process(void);
/* 返回 TIMG12 提供的系统时间，单位 ms。 */
uint32_t Camera_Vision_GetTimeMs(void);
/* 最近一帧目标是否有效且未超时。 */
uint8_t Camera_Vision_IsUsable(void);
/* 最近正确帧是否在 timeout_ms 时间内到达。 */
uint8_t Camera_Vision_IsLinkAlive(uint32_t timeout_ms);
/* 原子清空 UART3 的接收缓冲区和解析状态。 */
void Camera_Uart_ClearBuffer(void);

#endif
