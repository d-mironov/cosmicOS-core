#ifndef _SYSTICK_DELAY_H
#define _SYSTICK_DELAY_H

#include <stdint.h>
#include "../rcc/rcc.h"

//#define CYCLES_MS   (rcc_active_clock.ahb_freq / 1000)

#define CTRL_ENABLE         (1U<<0)
#define CTRL_CLKSRC         (1U<<2)
#define CTRL_COUNTFLAG      (1U<<16)

void delayMs(uint32_t ms);
void delayMs(uint32_t us);

void delay_ms(uint32_t ms);
void delay_us(uint32_t ms);

//uint32_t millis();
//uint32_t micros();

#endif
