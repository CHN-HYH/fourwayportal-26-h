#include "key.h"
#include "delay.h"
#include "ti_msp_dl_config.h"

static uint8_t Key_IsPressed(GPIO_Regs *port, uint32_t pin)
{
    return (uint8_t)((DL_GPIO_readPins(port, pin) & pin) == 0U);
}

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
    static KeyEvent last_key = KEY_EVENT_NONE;
    KeyEvent key = Key_ReadPressed();

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