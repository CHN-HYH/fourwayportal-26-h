#include "competition_ui.h"
#include "bsp_camera_usart.h"
#include "competition_task.h"
#include "key.h"
#include "oled.h"
#include <stdint.h>

#define UI_REFRESH_MS  (200U) /* 与 Steel 调度器相同的 OLED 刷新周期。 */

static uint32_t s_refresh_ms = 0U; /* 上一次 OLED 刷新时刻。 */
static uint8_t s_dirty = 1U;       /* 按键改变状态后的立即刷新标志。 */
static uint8_t s_drawn = 0U;       /* 是否已经绘制过一个完整界面。 */
static CompetitionTask s_last_task; /* 上一次显示的任务。 */
static CompetitionStatus s_last_status; /* 上一次显示的任务状态。 */
static uint32_t s_last_sec = 0U;   /* 上一次显示的计时秒数。 */

static void handle_key(KeyEvent key)
{
    CompetitionStatus status = Competition_Task_GetStatus();

    if (key == KEY_EVENT_NONE)
    {
        return;
    }
    if (status == COMP_STATUS_READY)
    {
        if (key == KEY_EVENT_LONG)
        {
            Competition_Task_SelectNext();
        }
        else
        {
            Competition_Task_Start();
        }
    }
    else if (key == KEY_EVENT_LONG)
    {
        Competition_Task_Stop();
    }
    else if (status == COMP_STATUS_RUNNING)
    {
        Competition_Task_Pause();
    }
    else if (status == COMP_STATUS_PAUSED)
    {
        Competition_Task_Resume();
    }
    else
    {
        return;
    }
    s_dirty = 1U;
}

static const uint8_t *task_text(CompetitionTask task)
{
    switch (task)
    {
        case COMP_TASK_LINE_LAP:          return (const uint8_t *)"T1:Line";
        case COMP_TASK_BALL_ROUND_TRIP:   return (const uint8_t *)"T2:Ball";
        case COMP_TASK_AB_WITH_BALL:      return (const uint8_t *)"T3:A->B";
        case COMP_TASK_LAP_WITH_BALL:     return (const uint8_t *)"T4:Lap";
        case COMP_TASK_LAP_WITH_POSITION: return (const uint8_t *)"T5:Pos";
        default:                          return (const uint8_t *)"";
    }
}

static uint8_t OLED_CenterX(const uint8_t *text)
{
    uint8_t length = 0U;

    while (text[length] != '\0')
    {
        length++;
    }
    return (uint8_t)((128U - (uint16_t)length * 6U) / 2U);
}

static void format_time(uint8_t *text, uint32_t sec)
{
    uint32_t min = (sec / 60U) % 100U;

    sec %= 60U;
    text[0] = (uint8_t)('0' + min / 10U);
    text[1] = (uint8_t)('0' + min % 10U);
    text[2] = ':';
    text[3] = (uint8_t)('0' + sec / 10U);
    text[4] = (uint8_t)('0' + sec % 10U);
    text[5] = '\0';
}

static void refresh(void)
{
    uint8_t text[8];
    CompetitionTask task = Competition_Task_GetSelected();
    CompetitionStatus status = Competition_Task_GetStatus();
    uint32_t sec = Competition_Task_GetElapsedMs() / 1000U;

    if ((s_drawn != 0U) && (task == s_last_task) &&
        (status == s_last_status) &&
        ((status != COMP_STATUS_RUNNING) || (sec == s_last_sec)))
    {
        return;
    }
    s_drawn = 1U;
    s_last_task = task;
    s_last_status = status;
    s_last_sec = sec;

    OLED_Clear();
    if (status == COMP_STATUS_READY)
    {
        text[0] = 'M';
        text[1] = 'o';
        text[2] = 'd';
        text[3] = 'e';
        text[4] = ':';
        text[5] = (uint8_t)('1' + (uint8_t)task);
        text[6] = '\0';
        OLED_ShowString(OLED_CenterX(text), 2U, text, 12U, 1U);
    }
    else if (status == COMP_STATUS_PAUSED)
    {
        OLED_ShowString(OLED_CenterX((const uint8_t *)"PAUSED"), 2U,
            (const uint8_t *)"PAUSED", 12U, 1U);
    }
    else if ((status == COMP_STATUS_DONE) ||
        (status == COMP_STATUS_TIMEOUT))
    {
        OLED_ShowString(OLED_CenterX((const uint8_t *)"FINISH"), 0U,
            (const uint8_t *)"FINISH", 12U, 1U);
    }
    else
    {
        OLED_ShowString(OLED_CenterX(task_text(task)), 0U, task_text(task),
            12U, 1U);
    }

    format_time(text, sec);
    OLED_ShowString(OLED_CenterX(text), 22U, text, 8U, 1U);
    OLED_Refresh();
}

void Competition_UI_Init(void)
{
    OLED_Init();
    s_refresh_ms = Camera_Vision_GetTimeMs();
    s_dirty = 1U;
    s_drawn = 0U;
    refresh();
    s_dirty = 0U;
}

void Competition_UI_Update(void)
{
    uint32_t now = Camera_Vision_GetTimeMs();

    handle_key(Key_GetEvent(now));
    if ((s_dirty == 0U) &&
        ((uint32_t)(now - s_refresh_ms) < UI_REFRESH_MS))
    {
        return;
    }
    s_refresh_ms = now;
    s_dirty = 0U;
    refresh();
}
