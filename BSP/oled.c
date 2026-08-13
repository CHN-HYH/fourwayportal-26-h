#include "oled.h"
#include "codetab.h"
#include "delay.h"
#include "oledfont_steel.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#define OLED_WIDTH       (128U)
#define OLED_HEIGHT       (32U)
#define OLED_PAGE_COUNT    (4U)
#define OLED_FIFO_DATA_N   (7U)

static uint8_t s_gram[OLED_WIDTH][OLED_PAGE_COUNT]; /* Steel 风格屏幕显存。 */

static void OLED_WaitIdle(void)
{
    while ((DL_I2C_getControllerStatus(I2C_0_INST) &
        DL_I2C_CONTROLLER_STATUS_IDLE) == 0U)
    {
    }
}

void I2C_WriteByte(uint8_t addr, uint8_t data)
{
    uint8_t temp[2];

    temp[0] = addr;
    temp[1] = data;
    OLED_WaitIdle();
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, temp, 2U);
    DL_I2C_startControllerTransfer(I2C_0_INST, OLED_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2U);
    while ((DL_I2C_getControllerStatus(I2C_0_INST) &
        DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U)
    {
    }
    OLED_WaitIdle();
}

void WriteCmd(unsigned char command)
{
    I2C_WriteByte(0x00U, command);
}

void WriteData(unsigned char data)
{
    I2C_WriteByte(0x40U, data);
}

void OLED_SetPos(unsigned char x, unsigned char y)
{
    WriteCmd((unsigned char)(0xB0U + y));
    WriteCmd((unsigned char)(x & 0x0FU));
    WriteCmd((unsigned char)(((x & 0xF0U) >> 4U) | 0x10U));
}

void OLED_Refresh(void)
{
    uint8_t page;
    uint8_t col;

    for (page = 0U; page < OLED_PAGE_COUNT; page++)
    {
        OLED_SetPos(0U, page);
        for (col = 0U; col < OLED_WIDTH; col += OLED_FIFO_DATA_N)
        {
            uint8_t chunk[OLED_FIFO_DATA_N + 1U];
            uint8_t batch = (uint8_t)((OLED_WIDTH - col > OLED_FIFO_DATA_N) ?
                OLED_FIFO_DATA_N : (OLED_WIDTH - col));
            uint8_t i;

            chunk[0] = 0x40U;
            for (i = 0U; i < batch; i++)
            {
                chunk[i + 1U] = s_gram[col + i][page];
            }
            OLED_WaitIdle();
            DL_I2C_fillControllerTXFIFO(I2C_0_INST, chunk,
                (uint32_t)(batch + 1U));
            DL_I2C_startControllerTransfer(I2C_0_INST, OLED_ADDRESS,
                DL_I2C_CONTROLLER_DIRECTION_TX, (uint32_t)(batch + 1U));
            while ((DL_I2C_getControllerStatus(I2C_0_INST) &
                DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U)
            {
            }
            OLED_WaitIdle();
        }
    }
}

void OLED_Clear(void)
{
    uint8_t page;
    uint8_t col;

    for (page = 0U; page < OLED_PAGE_COUNT; page++)
    {
        for (col = 0U; col < OLED_WIDTH; col++)
        {
            s_gram[col][page] = 0U;
        }
    }
    OLED_Refresh();
}

void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t mode)
{
    uint8_t mask;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }
    mask = (uint8_t)(1U << (y & 0x07U));
    if (mode != 0U)
    {
        s_gram[x][y / 8U] |= mask;
    }
    else
    {
        s_gram[x][y / 8U] &= (uint8_t)~mask;
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr,
    uint8_t size, uint8_t mode)
{
    uint8_t index;
    uint8_t bytes;
    uint8_t i;
    uint8_t bit;
    uint8_t x0 = x;
    uint8_t y0 = y;

    if ((chr < ' ') || (chr > '~'))
    {
        return;
    }
    index = (uint8_t)(chr - ' ');
    if (size == 8U)
    {
        bytes = 6U;
    }
    else if (size == 12U)
    {
        bytes = 12U;
    }
    else
    {
        return;
    }

    for (i = 0U; i < bytes; i++)
    {
        uint8_t data = (size == 8U) ? F6x8[index][i] :
            g_steel_font_6x12[index][i];

        for (bit = 0U; bit < 8U; bit++)
        {
            OLED_DrawPoint(x, y, (uint8_t)(((data & 0x01U) != 0U) ?
                mode : !mode));
            data >>= 1U;
            y++;
        }
        x++;
        if ((size == 12U) && ((x - x0) == 6U))
        {
            x = x0;
            y0 = (uint8_t)(y0 + 8U);
        }
        y = y0;
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *text,
    uint8_t size, uint8_t mode)
{
    while ((*text >= ' ') && (*text <= '~'))
    {
        OLED_ShowChar(x, y, *text, size, mode);
        x = (uint8_t)(x + 6U);
        text++;
    }
}

void OLED_Init(void)
{
    delay_ms(200U);
    WriteCmd(0xAEU);
    WriteCmd(0x00U);
    WriteCmd(0x10U);
    WriteCmd(0x00U);
    WriteCmd(0xB0U);
    WriteCmd(0x81U);
    WriteCmd(0xFFU);
    WriteCmd(0xA1U);
    WriteCmd(0xA6U);
    WriteCmd(0xA8U);
    WriteCmd(0x1FU);
    WriteCmd(0xC8U);
    WriteCmd(0xD3U);
    WriteCmd(0x00U);
    WriteCmd(0xD5U);
    WriteCmd(0x80U);
    WriteCmd(0xD9U);
    WriteCmd(0x1FU);
    WriteCmd(0xDAU);
    WriteCmd(0x00U);
    WriteCmd(0xDBU);
    WriteCmd(0x40U);
    WriteCmd(0x8DU);
    WriteCmd(0x14U);
    OLED_Clear();
    WriteCmd(0xAFU);
}

void OLED_Fill(unsigned char data)
{
    uint8_t page;
    uint8_t col;

    for (page = 0U; page < OLED_PAGE_COUNT; page++)
    {
        for (col = 0U; col < OLED_WIDTH; col++)
        {
            s_gram[col][page] = data;
        }
    }
    OLED_Refresh();
}

void OLED_CLS(void)
{
    OLED_Clear();
}

void OLED_ON(void)
{
    WriteCmd(0x8DU);
    WriteCmd(0x14U);
    WriteCmd(0xAFU);
}

void OLED_OFF(void)
{
    WriteCmd(0x8DU);
    WriteCmd(0x10U);
    WriteCmd(0xAEU);
}

void OLED_ShowStr(unsigned char x, unsigned char y,
    unsigned char text[], unsigned char size)
{
    OLED_ShowString(x, (uint8_t)(y * 8U), text,
        (uint8_t)((size == 1U) ? 8U : 12U), 1U);
    OLED_Refresh();
}
