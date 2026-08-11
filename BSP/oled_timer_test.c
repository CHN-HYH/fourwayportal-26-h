#include "oled_timer_test.h"
#include "oled.h"
#include "bsp_camera_usart.h"
#include <stdint.h>

#define OLED_MAX_SEC  (5999U) /* OLED 能显示的最大计时秒数，即 99:59。 */
#define OLED_X        (24U)   /* 大号计时文本的起始列。 */
#define OLED_PAGE     (2U)    /* 大号计时文本的起始页。 */
#define OLED_CH_N     (5U)    /* MM:SS 文本的字符数。 */

extern const unsigned char F8X16[];

static uint8_t s_run = 0U;          /* 计时器是否正在运行。 */
static uint32_t s_start_ms = 0U;    /* 本次计时的起始时刻，单位 ms。 */
static uint32_t s_sec = 0xFFFFFFFFU; /* 已显示的秒数，用于避免重复刷屏。 */

/* 将一个字模字节横向放大为两个输出字节。 */
static uint8_t Oled_Timer_ExpandByte(uint8_t data, uint8_t half)
{
    uint16_t exp = 0U; /* 将 8 位字模横向放大后的 16 位数据。 */
    uint8_t bit;       /* 原始字模中的位索引。 */

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((data & (uint8_t)(1U << bit)) != 0U)
        {
            exp |= (uint16_t)(3U << (2U * bit));
        }
    }

    return (uint8_t)((half == 0U) ? exp : (exp >> 8U));
}

/* 将总秒数格式化为 MM:SS，并以双倍宽度显示到 OLED。 */
static void Oled_Timer_Show(uint32_t all_sec)
{
    char txt[OLED_CH_N + 1U]; /* 待显示的 MM:SS 文本。 */
    uint32_t min = all_sec / 60U; /* 计时的分钟部分。 */
    uint32_t sec = all_sec % 60U; /* 计时的秒部分。 */
    uint8_t page;                  /* OLED 页索引。 */
    uint8_t ch;                    /* 文本字符索引。 */
    uint8_t col;                   /* 单个字符字模列索引。 */
    uint16_t off;                  /* 当前字符在字库中的起始偏移。 */
    uint8_t dat;                   /* 当前写入 OLED 的字模数据。 */

    min %= 100U;
    txt[0] = (char)('0' + (min / 10U));
    txt[1] = (char)('0' + (min % 10U));
    txt[2] = ':';
    txt[3] = (char)('0' + (sec / 10U));
    txt[4] = (char)('0' + (sec % 10U));
    txt[5] = '\0';

    for (page = 0U; page < 4U; page++)
    {
        OLED_SetPos(OLED_X, (unsigned char)(OLED_PAGE + page));
        for (ch = 0U; ch < OLED_CH_N; ch++)
        {
            off = (uint16_t)(((uint8_t)txt[ch] - 32U) * 16U);
            for (col = 0U; col < 8U; col++)
            {
                dat = F8X16[off + (uint8_t)((page / 2U) * 8U) + col];
                dat = Oled_Timer_ExpandByte(dat, (uint8_t)(page & 1U));
                WriteData(dat);
                WriteData(dat);
            }
        }
    }
}

void Oled_Timer_Init(void)
{
    OLED_Init();
    OLED_Fill(0x00U);
    Oled_Timer_Show(0U);
    s_run = 0U;
    s_sec = 0U;
}

void Oled_Timer_Start(void)
{
    s_run = 1U;
    s_start_ms = Camera_Vision_GetTimeMs();
    s_sec = 0xFFFFFFFFU;
    Oled_Timer_Show(0U);
}

void Oled_Timer_Update(void)
{
    uint32_t now; /* 当前系统时刻，单位 ms。 */
    uint32_t sec; /* 当前计时结果，单位 s。 */

    if (s_run == 0U)
    {
        return;
    }

    now = Camera_Vision_GetTimeMs();
    sec = ((now - s_start_ms) / 1000U) % (OLED_MAX_SEC + 1U);
    if (sec != s_sec)
    {
        s_sec = sec;
        Oled_Timer_Show(sec);
    }
}
