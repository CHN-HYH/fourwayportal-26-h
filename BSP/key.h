#ifndef __KEY_H__
#define __KEY_H__

#include <stdint.h>

#define KEY_DEBOUNCE_DELAY_MS  (20U) /* 两次采样之间的消抖时间，单位 ms。 */

typedef enum
{
    KEY_EVENT_NONE = 0U, /* 当前没有按键按下。 */
    KEY_EVENT_K1 = 1U,   /* K1 被确认按下。 */
    KEY_EVENT_K2 = 2U,   /* K2 被确认按下。 */
    KEY_EVENT_K3 = 3U,   /* K3 被确认按下。 */
    KEY_EVENT_K4 = 4U    /* K4 被确认按下。 */
} KeyEvent;

/* 返回一次性按下事件，持续按住只返回一次。 */
KeyEvent Key_GetEvent(void);

#endif
