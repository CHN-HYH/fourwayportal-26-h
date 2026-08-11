#include "bsp_camera_usart.h"
#include <stdio.h>

/* 逐帧打印会占用串口时间，闭环控制时保持关闭。 */
#define CAMERA_VISION_DEBUG_ENABLE     (0U)

typedef enum
{
    CAMERA_PARSE_WAIT_AA = 0,
    CAMERA_PARSE_WAIT_55,
    CAMERA_PARSE_COLLECT
} CameraParseState;

volatile CameraVisionState vision = {0};

static volatile uint8_t s_rx[CAMERA_UART_RX_BUFFER_LEN] = {0}; /* UART3 ISR 写入、主循环读取的环形缓冲区。 */
static volatile uint16_t s_head = 0U;      /* 环形缓冲区的写入位置。 */
static volatile uint16_t s_tail = 0U;      /* 环形缓冲区的读取位置。 */
static volatile uint32_t s_overflow = 0U;  /* 环形缓冲区已满的次数。 */
static volatile uint32_t s_ms = 0U;        /* TIMG12 维护的毫秒时间基。 */

static uint8_t s_frame[CAMERA_UART_FRAME_LEN] = {0}; /* 正在组装的 8 字节协议帧。 */
static uint8_t s_i = 0U;                   /* 当前已写入的帧字节数。 */
static uint8_t s_has_seq = 0U;              /* 是否已有序号可供连续性判断。 */
static CameraParseState s_parse = CAMERA_PARSE_WAIT_AA; /* 当前协议解析状态。 */

static const DL_TimerG_ClockConfig s_time_clk = { /* TIMG12 的时钟配置。 */
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0
};

static const DL_TimerG_TimerConfig s_time_cfg = { /* TIMG12 的 1 ms 周期配置。 */
    .timerMode = DL_TIMER_TIMER_MODE_PERIODIC,
    .period = 31999U,
    .startTimer = DL_TIMER_STOP,
    .genIntermInt = DL_TIMER_INTERM_INT_DISABLED,
    .counterVal = 0
};

/* 计算 UART3 环形缓冲区的下一个索引，并在末尾回绕。 */
static uint16_t Camera_Uart_NextIndex(uint16_t index)
{
    index++;
    if (index >= CAMERA_UART_RX_BUFFER_LEN)
    {
        index = 0;
    }
    return index;
}

/* 计算协议帧指定长度数据的低 8 位累加和。 */
static uint8_t Camera_Checksum(const uint8_t *data, uint8_t length)
{
    uint8_t sum = 0U; /* 协议帧前 7 字节的累加和。 */
    uint8_t i;        /* 累加索引。 */

    for (i = 0U; i < length; i++)
    {
        sum = (uint8_t)(sum + data[i]);
    }

    return sum;
}

/* 回到等待帧头状态，并丢弃正在组装的半帧。 */
static void Camera_ResetParser(void)
{
    s_parse = CAMERA_PARSE_WAIT_AA;
    s_i = 0U;
    s_has_seq = 0U;
}

/* 更新帧序号并记录丢帧、重复帧与异常跳变。 */
static void Camera_UpdateSequence(uint8_t seq)
{
    uint8_t delta; /* 当前序号相对上一次序号的增量。 */

    if (s_has_seq != 0U)
    {
        delta = (uint8_t)(seq - vision.seq);
        if (delta == 0U)
        {
            vision.seq_repeat++;
            return;
        }
        if (delta != 1U)
        {
            if (delta < 128U)
            {
                vision.seq_lost += (uint32_t)(delta - 1U);
            }
            else
            {
                vision.seq_break++;
            }
        }
    }

    vision.seq = seq;
    s_has_seq = 1U;
}

/* 判断帧序号是否首次出现，重复帧不进入控制链路。 */
static uint8_t Camera_IsNewSequence(uint8_t seq)
{
    if ((s_has_seq != 0U) && (seq == vision.seq))
    {
        vision.seq_repeat++;
        return 0U;
    }

    Camera_UpdateSequence(seq);
    return 1U;
}

/* 校验并解析完整的 8 字节摄像头协议帧。 */
static void Camera_HandleFrame(void)
{
    uint8_t flags;    /* 本帧状态标志。 */
    uint8_t seq;      /* 本帧序号。 */
    uint8_t sum;      /* 计算得到的累加和。 */
    uint16_t x;       /* 本帧目标横向坐标，单位 px。 */
    uint32_t now;     /* 正确帧到达时刻，单位 ms。 */

    sum = Camera_Checksum(s_frame, CAMERA_UART_FRAME_LEN - 1U);
    if (sum != s_frame[CAMERA_UART_FRAME_LEN - 1U])
    {
        vision.sum_err++;
#if CAMERA_VISION_DEBUG_ENABLE
        printf("[VIS] checksum_err calc=%02X rx=%02X\r\n",
            (unsigned int)sum,
            (unsigned int)s_frame[CAMERA_UART_FRAME_LEN - 1U]);
#endif
        return;
    }

    now = Camera_Vision_GetTimeMs();
    flags = s_frame[2];
    seq = s_frame[3];

    vision.frame_n++;
    vision.rx_ms = now;

    if (Camera_IsNewSequence(seq) == 0U)
    {
#if CAMERA_VISION_DEBUG_ENABLE
        printf("[VIS] seq_repeat=%u\r\n", (unsigned int)seq);
#endif
        return;
    }

    vision.flags = flags;
    vision.width = s_frame[6];

    if ((flags & 0x01U) == 0U)
    {
        vision.valid = 0U;
#if CAMERA_VISION_DEBUG_ENABLE
        printf("[VIS] seq=%u valid=0 width=%u\r\n",
            (unsigned int)seq, (unsigned int)vision.width);
#endif
        return;
    }

    x = (uint16_t)s_frame[4] | ((uint16_t)s_frame[5] << 8U);
    if (x >= CAMERA_VISION_IMAGE_WIDTH)
    {
        vision.valid = 0U;
        vision.range_err++;
#if CAMERA_VISION_DEBUG_ENABLE
        printf("[VIS] seq=%u range_err x=%u width=%u\r\n",
            (unsigned int)seq, (unsigned int)x, (unsigned int)vision.width);
#endif
        return;
    }

    vision.x = x;
    vision.valid = 1U;
    vision.valid_ms = now;
    vision.valid_n++;
#if CAMERA_VISION_DEBUG_ENABLE
    printf("[VIS] seq=%u valid=1 x=%u width=%u\r\n",
        (unsigned int)seq, (unsigned int)vision.x, (unsigned int)vision.width);
#endif
}

/* 使用 AA 55 帧头状态机接收单个协议字节。 */
static void Camera_ParseByte(uint8_t data)
{
    switch (s_parse)
    {
        case CAMERA_PARSE_WAIT_AA:
            if (data == 0xAAU)
            {
                s_frame[0] = data;
                s_parse = CAMERA_PARSE_WAIT_55;
            }
            break;

        case CAMERA_PARSE_WAIT_55:
            if (data == 0x55U)
            {
                s_frame[1] = data;
                s_i = 2U;
                s_parse = CAMERA_PARSE_COLLECT;
            }
            else if (data != 0xAAU)
            {
                s_parse = CAMERA_PARSE_WAIT_AA;
            }
            break;

        case CAMERA_PARSE_COLLECT:
            s_frame[s_i++] = data;
            if (s_i >= CAMERA_UART_FRAME_LEN)
            {
                Camera_HandleFrame();
                s_parse = CAMERA_PARSE_WAIT_AA;
                s_i = 0U;
            }
            break;

        default:
            Camera_ResetParser();
            break;
    }
}

/* 初始化 TIMG12，使其产生独立的 1 ms 时间基。 */
static void Camera_Time_Init(void)
{
    DL_TimerG_reset(TIMG12);
    DL_TimerG_enablePower(TIMG12);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_TimerG_setClockConfig(TIMG12,
        (DL_TimerG_ClockConfig *)&s_time_clk);
    DL_TimerG_initTimerMode(TIMG12,
        (DL_TimerG_TimerConfig *)&s_time_cfg);
    DL_TimerG_enableInterrupt(TIMG12, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(TIMG12_INT_IRQn);
    NVIC_EnableIRQ(TIMG12_INT_IRQn);
    DL_TimerG_startCounter(TIMG12);
}

void Camera_Uart_ClearBuffer(void)
{
    uint32_t mask = __get_PRIMASK(); /* 进入临界区前的中断屏蔽状态。 */

    __disable_irq();
    s_head = 0U;
    s_tail = 0U;
    Camera_ResetParser();
    __set_PRIMASK(mask);
}

void Camera_Vision_Init(void)
{
    uint32_t mask = __get_PRIMASK(); /* 进入临界区前的中断屏蔽状态。 */

    __disable_irq();
    vision = (CameraVisionState){0};
    s_head = 0U;
    s_tail = 0U;
    s_overflow = 0U;
    s_ms = 0U;
    Camera_ResetParser();
    __set_PRIMASK(mask);

    Camera_Time_Init();
}

void Camera_Vision_Process(void)
{
    uint8_t data; /* 从环形缓冲区取出的单个字节。 */

    while (s_tail != s_head)
    {
        data = s_rx[s_tail];
        s_tail = Camera_Uart_NextIndex(s_tail);
        Camera_ParseByte(data);
    }

    vision.rx_overflow = s_overflow;
}

uint32_t Camera_Vision_GetTimeMs(void)
{
    return s_ms;
}

uint8_t Camera_Vision_IsUsable(void)
{
    return (uint8_t)((vision.valid != 0U) &&
        ((uint32_t)(Camera_Vision_GetTimeMs() - vision.valid_ms) <
            CAMERA_VISION_TIMEOUT_MS));
}

uint8_t Camera_Vision_IsLinkAlive(uint32_t timeout_ms)
{
    return (uint8_t)((vision.frame_n != 0U) &&
        ((uint32_t)(Camera_Vision_GetTimeMs() - vision.rx_ms) < timeout_ms));
}

void UART3_IRQHandler(void)
{
    uint8_t rx;       /* UART3 硬件刚接收的字节。 */
    uint16_t next;    /* 写入当前字节后的缓冲区位置。 */

    switch (DL_UART_getPendingInterrupt(UART_3_INST))
    {
        case DL_UART_IIDX_RX:
            rx = DL_UART_Main_receiveData(UART_3_INST);
            next = Camera_Uart_NextIndex(s_head);
            if (next == s_tail)
            {
                s_overflow++;
            }
            else
            {
                s_rx[s_head] = rx;
                s_head = next;
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
            s_ms++;
            break;

        default:
            break;
    }
}
