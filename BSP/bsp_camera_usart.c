#include "bsp_camera_usart.h"
#include <stdio.h>

/* 逐帧打印会占用串口时间，闭环控制时保持关闭。 */
#define CAMERA_VISION_DEBUG_ENABLE     (0U)

/*
 * 数据流：UART3 中断每次只收一个字节放入 s_rx；主循环调用
 * Camera_Vision_Process() 后才从 s_rx 取字节、拼成完整帧并更新 vision。
 * 这样中断执行很短，不会被校验、打印等较慢操作占用。
 */

/* 三个状态依次等待 AA、55，再收集后面的 6 个字节。 */
typedef enum
{
    CAMERA_PARSE_WAIT_AA = 0,
    CAMERA_PARSE_WAIT_55,
    CAMERA_PARSE_COLLECT
} CameraParseState;

/* 解析后的最新视觉结果，舵机控制等其他模块只读取这里。 */
volatile CameraVisionState vision = {0};

/* s_head 由中断推进，s_tail 由主循环推进，二者组成一个环形队列。 */
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
        /* uint8_t 相减会自然处理 0xFF 回绕到 0 的正常情况。 */
        delta = (uint8_t)(seq - vision.seq);
        if (delta == 0U)
        {
            vision.seq_repeat++;
            return;
        }
        if (delta != 1U)
        {
            /* 跳号只作通信统计，仍然接受当前最新帧给控制使用。 */
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

    /* 先确认数据没有在串口传输中损坏，校验失败的帧完全不用。 */
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

    /* 这一帧校验正确，先记录链路仍有数据到达。 */
    vision.frame_n++;
    vision.rx_ms = now;

    /* 重复包可能来自串口重发，不能重复驱动舵机。 */
    if (Camera_IsNewSequence(seq) == 0U)
    {
#if CAMERA_VISION_DEBUG_ENABLE
        printf("[VIS] seq_repeat=%u\r\n", (unsigned int)seq);
#endif
        return;
    }

    vision.flags = flags;
    vision.width = s_frame[6];

    /* FLAGS.bit0 为 0 表示摄像头当前没有可靠目标。 */
    if ((flags & 0x01U) == 0U)
    {
        vision.valid = 0U;
#if CAMERA_VISION_DEBUG_ENABLE
        printf("[VIS] seq=%u valid=0 width=%u\r\n",
            (unsigned int)seq, (unsigned int)vision.width);
#endif
        return;
    }

    /* X 用低字节在前的小端格式发送，合成为 16 位坐标。 */
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

    /* 只有有效且坐标没有越界的帧，才允许更新控制所用的 x。 */
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
            /* 未找到帧头时忽略所有杂字节，只等第一个 AA。 */
            if (data == 0xAAU)
            {
                s_frame[0] = data;
                s_parse = CAMERA_PARSE_WAIT_55;
            }
            break;

        case CAMERA_PARSE_WAIT_55:
            /* 已收到 AA，只有紧跟 55 才算完整帧头。 */
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
            /* 帧头已确认，把剩余字节依次放入 8 字节帧缓冲区。 */
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

    /* 清空时暂停 UART3 中断，避免中断同时修改 s_head。 */
    __disable_irq();
    s_head = 0U;
    s_tail = 0U;
    Camera_ResetParser();
    __set_PRIMASK(mask);
}

void Camera_Vision_Init(void)
{
    uint32_t mask = __get_PRIMASK(); /* 进入临界区前的中断屏蔽状态。 */

    /* 初始化共享状态时暂停中断，防止读写到一半的数据。 */
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
    uint32_t frame = vision.frame_n; /* 本次调用前已解析的正确帧数。 */

    /* 每次最多交付一帧，保证控制和日志不会跳过积压的中间坐标。 */
    while (s_tail != s_head)
    {
        data = s_rx[s_tail];
        s_tail = Camera_Uart_NextIndex(s_tail);
        Camera_ParseByte(data);
        if (vision.frame_n != frame)
        {
            break;
        }
    }

    vision.rx_overflow = s_overflow;
}

uint32_t Camera_Vision_GetTimeMs(void)
{
    return s_ms;
}

uint8_t Camera_Vision_IsUsable(void)
{
    /* 控制可用必须同时满足：最后结果有效，且距离有效帧未超时。 */
    return (uint8_t)((vision.valid != 0U) &&
        ((uint32_t)(Camera_Vision_GetTimeMs() - vision.valid_ms) <
            CAMERA_VISION_TIMEOUT_MS));
}

uint8_t Camera_Vision_IsLinkAlive(uint32_t timeout_ms)
{
    /* 这里只判断串口是否仍有正确帧到达，不关心帧内是否检测到钢珠。 */
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
            /* 中断中只做收字节入队，绝不在这里拼帧或控制舵机。 */
            rx = DL_UART_Main_receiveData(UART_3_INST);
            next = Camera_Uart_NextIndex(s_head);
            if (next == s_tail)
            {
                /* 队列满时丢弃新字节，并留下溢出计数供排查。 */
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
            /* TIMG12 每 1 ms 进入一次，用于视觉数据超时判断。 */
            s_ms++;
            break;

        default:
            break;
    }
}
