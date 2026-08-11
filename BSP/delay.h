#ifndef _DELAY_H
#define _DELAY_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* 基于 SysTick 的阻塞微秒延时。 */
void delay_us(unsigned long us);
/* 基于 delay_us() 的阻塞毫秒延时。 */
void delay_ms(unsigned long ms);


#endif
