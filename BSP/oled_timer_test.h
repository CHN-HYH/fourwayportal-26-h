#ifndef __OLED_TIMER_TEST_H__
#define __OLED_TIMER_TEST_H__

/* 初始化 OLED 计时显示，初始显示 00:00。 */
void Oled_Timer_Init(void);
/* 以当前毫秒时间基重新开始计时。 */
void Oled_Timer_Start(void);
/* 在主循环中调用，每秒刷新一次显示。 */
void Oled_Timer_Update(void);

#endif
