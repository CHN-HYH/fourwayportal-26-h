#ifndef __COMPETITION_TASK_H__
#define __COMPETITION_TASK_H__

#include <stdint.h>

typedef enum
{
    COMP_TASK_LINE_LAP = 0,
    COMP_TASK_BALL_ROUND_TRIP,
    COMP_TASK_AB_WITH_BALL,
    COMP_TASK_LAP_WITH_BALL,
    COMP_TASK_LAP_WITH_POSITION,
    COMP_TASK_COUNT
} CompetitionTask;

typedef enum
{
    COMP_STATUS_READY = 0,
    COMP_STATUS_RUNNING,
    COMP_STATUS_PAUSED,
    COMP_STATUS_DONE,
    COMP_STATUS_TIMEOUT
} CompetitionStatus;

void Competition_Task_Init(void);
void Competition_Task_Update(void);
void Competition_Task_Select(CompetitionTask task);
void Competition_Task_SelectPrevious(void);
void Competition_Task_SelectNext(void);
void Competition_Task_Start(void);
void Competition_Task_Pause(void);
void Competition_Task_Resume(void);
void Competition_Task_Stop(void);
void Competition_Task_SetBallPosition(float target_cm);

CompetitionTask Competition_Task_GetSelected(void);
CompetitionStatus Competition_Task_GetStatus(void);
uint32_t Competition_Task_GetElapsedMs(void);
uint8_t Competition_Task_GetWaypointCount(void);
uint8_t Competition_Task_GetBallPhase(void);
float Competition_Task_GetBallTarget(void);

#endif
