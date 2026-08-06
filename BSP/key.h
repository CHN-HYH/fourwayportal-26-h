#ifndef __KEY_H__
#define __KEY_H__

#include <stdint.h>

#define KEY_DEBOUNCE_DELAY_MS  (20U)

typedef enum
{
    KEY_EVENT_NONE = 0U,
    KEY_EVENT_K1 = 1U,
    KEY_EVENT_K2 = 2U,
    KEY_EVENT_K3 = 3U,
    KEY_EVENT_K4 = 4U
} KeyEvent;

KeyEvent Key_GetEvent(void);

#endif