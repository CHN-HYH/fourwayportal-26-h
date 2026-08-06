#include "oled_timer_test.h"
#include "oled.h"
#include "bsp_camera_usart.h"
#include <stdint.h>

#define OLED_TIMER_MAX_SECONDS    (5999U)
#define OLED_TIMER_START_X        (24U)
#define OLED_TIMER_START_PAGE     (2U)
#define OLED_TIMER_CHAR_COUNT     (5U)

extern const unsigned char F8X16[];

static uint8_t g_oled_timer_running = 0U;
static uint32_t g_oled_timer_start_ms = 0U;
static uint32_t g_oled_timer_displayed_seconds = 0xFFFFFFFFU;

static uint8_t Oled_Timer_ExpandByte(uint8_t data, uint8_t half)
{
    uint16_t expanded = 0U;
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((data & (uint8_t)(1U << bit)) != 0U)
        {
            expanded |= (uint16_t)(3U << (2U * bit));
        }
    }

    return (uint8_t)((half == 0U) ? expanded : (expanded >> 8U));
}

static void Oled_Timer_Show(uint32_t elapsed_seconds)
{
    char text[OLED_TIMER_CHAR_COUNT + 1U];
    uint32_t minutes = elapsed_seconds / 60U;
    uint32_t seconds = elapsed_seconds % 60U;
    uint8_t page;
    uint8_t character;
    uint8_t column;
    uint16_t font_offset;
    uint8_t source;

    minutes %= 100U;
    text[0] = (char)('0' + (minutes / 10U));
    text[1] = (char)('0' + (minutes % 10U));
    text[2] = ':';
    text[3] = (char)('0' + (seconds / 10U));
    text[4] = (char)('0' + (seconds % 10U));
    text[5] = '\0';

    for (page = 0U; page < 4U; page++)
    {
        OLED_SetPos(OLED_TIMER_START_X, (unsigned char)(OLED_TIMER_START_PAGE + page));
        for (character = 0U; character < OLED_TIMER_CHAR_COUNT; character++)
        {
            font_offset = (uint16_t)(((uint8_t)text[character] - 32U) * 16U);
            for (column = 0U; column < 8U; column++)
            {
                source = F8X16[font_offset + (uint8_t)((page / 2U) * 8U) + column];
                source = Oled_Timer_ExpandByte(source, (uint8_t)(page & 1U));
                WriteData(source);
                WriteData(source);
            }
        }
    }
}

void Oled_Timer_Init(void)
{
    OLED_Init();
    OLED_Fill(0x00U);
    Oled_Timer_Show(0U);
    g_oled_timer_running = 0U;
    g_oled_timer_displayed_seconds = 0U;
}

void Oled_Timer_Start(void)
{
    g_oled_timer_running = 1U;
    g_oled_timer_start_ms = Camera_Vision_GetTimeMs();
    g_oled_timer_displayed_seconds = 0xFFFFFFFFU;
    Oled_Timer_Show(0U);
}

void Oled_Timer_Update(void)
{
    uint32_t now_ms;
    uint32_t elapsed_seconds;

    if (g_oled_timer_running == 0U)
    {
        return;
    }

    now_ms = Camera_Vision_GetTimeMs();
    elapsed_seconds = ((now_ms - g_oled_timer_start_ms) / 1000U) %
        (OLED_TIMER_MAX_SECONDS + 1U);
    if (elapsed_seconds != g_oled_timer_displayed_seconds)
    {
        g_oled_timer_displayed_seconds = elapsed_seconds;
        Oled_Timer_Show(elapsed_seconds);
    }
}
