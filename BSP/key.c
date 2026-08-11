#include "key.h"
#include "delay.h"
#include "ti_msp_dl_config.h"

/* 读取按键引脚，低电平表示按下。 */
static uint8_t Key_IsPressed(GPIO_Regs *port, uint32_t pin)
{
    return (uint8_t)((DL_GPIO_readPins(port, pin) & pin) == 0U);
}

/* 按 K1、K2、K3、K4 的优先顺序读取当前按下的按键。 */
static KeyEvent Key_ReadPressed(void)
{
    if (Key_IsPressed(KEY_A_PORT, KEY_A_K1_PIN) != 0U)
    {
        return KEY_EVENT_K1;
    }
    if (Key_IsPressed(KEY_B_PORT, KEY_B_K2_PIN) != 0U)
    {
        return KEY_EVENT_K2;
    }
    if (Key_IsPressed(KEY_B_PORT, KEY_B_K3_PIN) != 0U)
    {
        return KEY_EVENT_K3;
    }
    if (Key_IsPressed(KEY_A_PORT, KEY_A_K4_PIN) != 0U)
    {
        return KEY_EVENT_K4;
    }

    return KEY_EVENT_NONE;
}

KeyEvent Key_GetEvent(void)
{
    static KeyEvent last_key = KEY_EVENT_NONE; /* 上次已确认的按键，用于防止长按重复触发。 */
    KeyEvent key = Key_ReadPressed();          /* 本次检测到的按键。 */

    if (key == KEY_EVENT_NONE)
    {
        last_key = KEY_EVENT_NONE;
        return KEY_EVENT_NONE;
    }

    delay_ms(KEY_DEBOUNCE_DELAY_MS);
    if ((Key_ReadPressed() == key) && (last_key == KEY_EVENT_NONE))
    {
        last_key = key;
        return key;
    }

    return KEY_EVENT_NONE;
}
