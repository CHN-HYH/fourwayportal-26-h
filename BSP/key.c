#include "key.h"
#include "ti_msp_dl_config.h"

typedef enum
{
    KEY_STATE_RELEASED = 0U,
    KEY_STATE_DEBOUNCE,
    KEY_STATE_PRESSED,
    KEY_STATE_LONG
} KeyState;

static uint8_t Key_IsK1Pressed(void)
{
    return (uint8_t)((DL_GPIO_readPins(KEY_A_PORT, KEY_A_K1_PIN) &
        KEY_A_K1_PIN) == 0U);
}

KeyEvent Key_GetEvent(uint32_t current_ms)
{
    static KeyState state = KEY_STATE_RELEASED; /* K1 当前扫描状态。 */
    static uint32_t debounce_ms = 0U;           /* 本次消抖开始时刻。 */
    static uint32_t press_ms = 0U;              /* 确认按下的时刻。 */
    uint8_t pressed = Key_IsK1Pressed();

    switch (state)
    {
        case KEY_STATE_RELEASED:
            if (pressed != 0U)
            {
                state = KEY_STATE_DEBOUNCE;
                debounce_ms = current_ms;
            }
            break;

        case KEY_STATE_DEBOUNCE:
            if (pressed == 0U)
            {
                state = KEY_STATE_RELEASED;
            }
            else if ((uint32_t)(current_ms - debounce_ms) >=
                KEY_DEBOUNCE_DELAY_MS)
            {
                state = KEY_STATE_PRESSED;
                press_ms = current_ms;
            }
            break;

        case KEY_STATE_PRESSED:
            if (pressed == 0U)
            {
                state = KEY_STATE_RELEASED;
                return KEY_EVENT_SHORT;
            }
            if ((uint32_t)(current_ms - press_ms) >= KEY_LONG_PRESS_MS)
            {
                state = KEY_STATE_LONG;
                return KEY_EVENT_LONG;
            }
            break;

        case KEY_STATE_LONG:
            if (pressed == 0U)
            {
                state = KEY_STATE_RELEASED;
            }
            break;

        default:
            state = KEY_STATE_RELEASED;
            break;
    }

    return KEY_EVENT_NONE;
}
