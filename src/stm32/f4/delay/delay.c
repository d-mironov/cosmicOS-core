#include "delay.h"
#include <stm32f4xx.h>
#include "../rcc/rcc.h"
#include "../timer/timer.h"

volatile u32 syscount = 0;

void delayMs(u32 ms) {
    SysTick->LOAD = (SYSTEM_CLOCK/1000)-1;
    SysTick->VAL = 0x00;

    SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;
    for(u32 i = 0; i < ms; i++) {
        while((SysTick->CTRL & CTRL_COUNTFLAG) == 0);
    }

    SysTick->CTRL = 0x00;
}


void delayUs(u32 us) {
    SysTick->LOAD = (SYSTEM_CLOCK/1000)/1000;
    SysTick->VAL = 0x00;

    SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;
    for (u32 i = 0; i < us; i++) {
        while((SysTick->CTRL & CTRL_COUNTFLAG) == 0);
    }

    SysTick->CTRL = 0x00;
}


// NEW DELAY FUNCTIONS

