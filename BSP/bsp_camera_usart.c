#include "bsp_camera_usart.h"
#include <stdio.h>

#define CAMERA_VISION_DEBUG_ENABLE     (1U)

typedef enum
{
    CAMERA_PARSE_WAIT_AA = 0,
    CAMERA_PARSE_WAIT_55,
    CAMERA_PARSE_COLLECT
} CameraParseState;

volatile CameraVisionState vision = {0};

static volatile uint8_t g_camera_uart_rx_buffer[CAMERA_UART_RX_BUFFER_LEN] = {0};
static volatile uint16_t g_camera_uart_rx_head = 0;
static volatile uint16_t g_camera_uart_rx_tail = 0;
static volatile uint32_t g_camera_uart_rx_overflow_count = 0;
static volatile uint32_t g_camera_time_ms = 0;

static uint8_t g_camera_frame[CAMERA_UART_FRAME_LEN] = {0};
static uint8_t g_camera_frame_index = 0;
static uint8_t g_camera_has_seq = 0;
static CameraParseState g_camera_parse_state = CAMERA_PARSE_WAIT_AA;

static const DL_TimerG_ClockConfig g_camera_time_clock_config = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0
};

static const DL_TimerG_TimerConfig g_camera_time_timer_config = {
    .timerMode = DL_TIMER_TIMER_MODE_PERIODIC,
    .period = 31999U,
    .startTimer = DL_TIMER_STOP,
    .genIntermInt = DL_TIMER_INTERM_INT_DISABLED,
    .counterVal = 0
};

static uint16_t Camera_Uart_NextIndex(uint16_t index)
{
    index++;
    if (index >= CAMERA_UART_RX_BUFFER_LEN)
    {
        index = 0;
    }
    return index;
}

static uint8_t Camera_Crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;
    uint8_t i;
    uint8_t bit;

    for (i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8U; bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ 0x07U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

static void Camera_ResetParser(void)
{
    g_camera_parse_state = CAMERA_PARSE_WAIT_AA;
    g_camera_frame_index = 0;
    g_camera_has_seq = 0;
}

static void Camera_UpdateSequence(uint8_t seq)
{
    uint8_t delta;

    if (g_camera_has_seq != 0U)
    {
        delta = (uint8_t)(seq - vision.seq);
        if (delta == 0U)
        {
            vision.seq_repeat_count++;
            return;
        }
        if (delta != 1U)
        {
            if (delta < 128U)
            {
                vision.seq_lost_count += (uint32_t)(delta - 1U);
            }
            else
            {
                vision.seq_discontinuity_count++;
            }
        }
    }

    vision.seq = seq;
    g_camera_has_seq = 1U;
}

static uint8_t Camera_IsNewSequence(uint8_t seq)
{
    if ((g_camera_has_seq != 0U) && (seq == vision.seq))
    {
        vision.seq_repeat_count++;
        return 0U;
    }

    Camera_UpdateSequence(seq);
    return 1U;
}

static void Camera_HandleFrame(void)
{
    uint8_t flags;
    uint8_t seq;
    uint8_t crc;
    uint16_t x;
    uint32_t now_ms;

    crc = Camera_Crc8(g_camera_frame, CAMERA_UART_FRAME_LEN - 1U);
    if (crc != g_camera_frame[CAMERA_UART_FRAME_LEN - 1U])
    {
        vision.crc_error_count++;
#if CAMERA_VISION_DEBUG_ENABLE
        printf("[VIS] crc_err calc=%02X rx=%02X\r\n",
            (unsigned int)crc,
            (unsigned int)g_camera_frame[CAMERA_UART_FRAME_LEN - 1U]);
#endif
        return;
    }

    now_ms = Camera_Vision_GetTimeMs();
    flags = g_camera_frame[2];
    seq = g_camera_frame[3];

    vision.frame_count++;
    vision.last_rx_ms = now_ms;

    if (Camera_IsNewSequence(seq) == 0U)
    {
#if CAMERA_VISION_DEBUG_ENABLE
        printf("[VIS] seq_repeat=%u\r\n", (unsigned int)seq);
#endif
        return;
    }

    vision.flags = flags;
    vision.width = g_camera_frame[6];

    if ((flags & 0x01U) == 0U)
    {
        vision.valid = 0U;
#if CAMERA_VISION_DEBUG_ENABLE
        printf("[VIS] seq=%u valid=0 width=%u\r\n",
            (unsigned int)seq, (unsigned int)vision.width);
#endif
        return;
    }

    x = (uint16_t)g_camera_frame[4] | ((uint16_t)g_camera_frame[5] << 8U);
    if (x >= CAMERA_VISION_IMAGE_WIDTH)
    {
        vision.valid = 0U;
        vision.range_error_count++;
#if CAMERA_VISION_DEBUG_ENABLE
        printf("[VIS] seq=%u range_err x=%u width=%u\r\n",
            (unsigned int)seq, (unsigned int)x, (unsigned int)vision.width);
#endif
        return;
    }

    vision.x = x;
    vision.valid = 1U;
    vision.last_valid_ms = now_ms;
    vision.valid_frame_count++;
#if CAMERA_VISION_DEBUG_ENABLE
    printf("[VIS] seq=%u valid=1 x=%u width=%u\r\n",
        (unsigned int)seq, (unsigned int)vision.x, (unsigned int)vision.width);
#endif
}

static void Camera_ParseByte(uint8_t data)
{
    switch (g_camera_parse_state)
    {
        case CAMERA_PARSE_WAIT_AA:
            if (data == 0xAAU)
            {
                g_camera_frame[0] = data;
                g_camera_parse_state = CAMERA_PARSE_WAIT_55;
            }
            break;

        case CAMERA_PARSE_WAIT_55:
            if (data == 0x55U)
            {
                g_camera_frame[1] = data;
                g_camera_frame_index = 2U;
                g_camera_parse_state = CAMERA_PARSE_COLLECT;
            }
            else if (data != 0xAAU)
            {
                g_camera_parse_state = CAMERA_PARSE_WAIT_AA;
            }
            break;

        case CAMERA_PARSE_COLLECT:
            g_camera_frame[g_camera_frame_index++] = data;
            if (g_camera_frame_index >= CAMERA_UART_FRAME_LEN)
            {
                Camera_HandleFrame();
                g_camera_parse_state = CAMERA_PARSE_WAIT_AA;
                g_camera_frame_index = 0U;
            }
            break;

        default:
            Camera_ResetParser();
            break;
    }
}

static void Camera_Time_Init(void)
{
    DL_TimerG_reset(TIMG12);
    DL_TimerG_enablePower(TIMG12);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_TimerG_setClockConfig(TIMG12,
        (DL_TimerG_ClockConfig *)&g_camera_time_clock_config);
    DL_TimerG_initTimerMode(TIMG12,
        (DL_TimerG_TimerConfig *)&g_camera_time_timer_config);
    DL_TimerG_enableInterrupt(TIMG12, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(TIMG12_INT_IRQn);
    NVIC_EnableIRQ(TIMG12_INT_IRQn);
    DL_TimerG_startCounter(TIMG12);
}

void Camera_Uart_ClearBuffer(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_camera_uart_rx_head = 0U;
    g_camera_uart_rx_tail = 0U;
    Camera_ResetParser();
    __set_PRIMASK(primask);
}

void Camera_Vision_Init(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    vision = (CameraVisionState){0};
    g_camera_uart_rx_head = 0U;
    g_camera_uart_rx_tail = 0U;
    g_camera_uart_rx_overflow_count = 0U;
    g_camera_time_ms = 0U;
    Camera_ResetParser();
    __set_PRIMASK(primask);

    Camera_Time_Init();
}

void Camera_Vision_Process(void)
{
    uint8_t data;

    while (g_camera_uart_rx_tail != g_camera_uart_rx_head)
    {
        data = g_camera_uart_rx_buffer[g_camera_uart_rx_tail];
        g_camera_uart_rx_tail = Camera_Uart_NextIndex(g_camera_uart_rx_tail);
        Camera_ParseByte(data);
    }

    vision.rx_overflow_count = g_camera_uart_rx_overflow_count;
}

uint32_t Camera_Vision_GetTimeMs(void)
{
    return g_camera_time_ms;
}

uint8_t Camera_Vision_IsUsable(void)
{
    return (uint8_t)((vision.valid != 0U) &&
        ((uint32_t)(Camera_Vision_GetTimeMs() - vision.last_valid_ms) <
            CAMERA_VISION_TIMEOUT_MS));
}

uint8_t Camera_Vision_IsLinkAlive(uint32_t timeout_ms)
{
    return (uint8_t)((vision.frame_count != 0U) &&
        ((uint32_t)(Camera_Vision_GetTimeMs() - vision.last_rx_ms) < timeout_ms));
}

void UART3_IRQHandler(void)
{
    uint8_t received_data;
    uint16_t next_head;

    switch (DL_UART_getPendingInterrupt(UART_3_INST))
    {
        case DL_UART_IIDX_RX:
            received_data = DL_UART_Main_receiveData(UART_3_INST);
            next_head = Camera_Uart_NextIndex(g_camera_uart_rx_head);
            if (next_head == g_camera_uart_rx_tail)
            {
                g_camera_uart_rx_overflow_count++;
            }
            else
            {
                g_camera_uart_rx_buffer[g_camera_uart_rx_head] = received_data;
                g_camera_uart_rx_head = next_head;
            }
            break;

        default:
            break;
    }
}

void TIMG12_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMG12))
    {
        case DL_TIMER_IIDX_ZERO:
            g_camera_time_ms++;
            break;

        default:
            break;
    }
}
