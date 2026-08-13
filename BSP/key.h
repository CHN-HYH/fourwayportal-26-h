#ifndef __KEY_H__
#define __KEY_H__

#include <stdint.h>

#define KEY_DEBOUNCE_DELAY_MS  (20U)  /* 按键消抖时间，单位 ms。 */
#define KEY_LONG_PRESS_MS      (700U) /* 长按触发阈值，单位 ms。 */

typedef enum
{
    KEY_EVENT_NONE = 0U, /* 当前没有新事件。 */
    KEY_EVENT_SHORT,     /* K1 释放时产生短按事件。 */
    KEY_EVENT_LONG       /* K1 持续按下达到阈值时产生长按事件。 */
} KeyEvent;

/* 按 Steel 工程的交互规则扫描 K1，长按只触发一次。 */
KeyEvent Key_GetEvent(uint32_t current_ms);

#endif
