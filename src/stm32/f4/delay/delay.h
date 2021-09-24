#ifndef _SYSTICK_DELAY_H
#define _SYSTICK_DELAY_H

#include <stdint.h>

#define CYCLES_MS   16000

#define CTRL_ENABLE         (1U<<0)
#define CTRL_CLKSRC         (1U<<2)
#define CTRL_COUNTFLAG      (1U<<16)

void delayMs(uint32_t ms);
void delayMs(uint32_t us);

#endif
