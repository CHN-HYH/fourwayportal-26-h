#include "delay.h"

volatile unsigned int delay_times = 0; /* 预留的 SysTick 毫秒倒计时变量。 */

//搭配滴答定时器实现的精确us延时
//Accurate us delay with tick timer
void delay_us(unsigned long us)
{
    uint32_t need;      /* 本次延时所需的 SysTick 计数。 */
    uint32_t last;      /* 上一次读取到的 SysTick 值。 */
    uint32_t now;       /* 本次循环读取到的 SysTick 值。 */
    uint32_t cnt = 38U; /* 延时函数自身开销的补偿计数。 */

    // 计算需要的时钟数 = 延迟微秒数 * 每微秒的时钟数
	// Calculate the number of clocks required = delay microseconds * number of clocks per microsecond
    need = us * (32000000U / 1000000U);

    // 获取当前的SysTick值
	// Get the current SysTick value
    last = SysTick->VAL;

    while (1)
    {
        // 重复刷新获取当前的SysTick值
		// Repeatedly refresh to get the current SysTick value
        now = SysTick->VAL;

        if (now != last)
        {
            if (now < last)
                cnt += last - now;
            else
                cnt += SysTick->LOAD - now + last;

            last = now;

            // 如果达到了需要的时钟数，就退出循环
			// If the required number of clocks is reached, exit the loop
            if (cnt >= need)
                break;
        }
    }
}
//搭配滴答定时器实现的精确ms延时
//Accurate ms delay with tick timer
void delay_ms(unsigned long ms)
{
	delay_us( ms * 1000 );
}

//void SysTick_Handler(void)
//{
//	if(delay_times != 0)
//	{
//		delay_times--;
//	}
//}
