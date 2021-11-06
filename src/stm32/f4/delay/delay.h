#ifndef _SYSTICK_DELAY_H
#define _SYSTICK_DELAY_H

#include <stdint.h>
#include "../rcc/rcc.h"
#include "../../../cosmic.h"

//#define CYCLES_MS   (rcc_active_clock.ahb_freq / 1000)

#define CTRL_ENABLE         (1U<<0)
#define CTRL_CLKSRC         (1U<<2)
#define CTRL_COUNTFLAG      (1U<<16)

void delayMs(u32 ms);
void delayMs(u32 us);

void delay_ms(u32 ms);
void delay_us(u32 ms);

//u32 millis();
//u32 micros();

#endif
