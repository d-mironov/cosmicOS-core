#include "delay.h"
#include <stm32f4xx.h>

void delayMs(uint32_t ms) {
    SysTick->LOAD = CYCLES_MS;
    SysTick->VAL = 0x00;

    SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;
    for(int i = 0; i < ms; i++) {
        while((SysTick->CTRL & CTRL_COUNTFLAG) == 0);
    }

    SysTick->CTRL = 0x00;
}


void delayUs(uint32_t us) {
    SysTick->LOAD = CYCLES_MS/1000;
    SysTick->VAL = 0x00;

    SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;
    for (int i = 0; i < us; i++) {
        while((SysTick->CTRL & CTRL_COUNTFLAG) == 0);
    }

    SysTick->CTRL = 0x00;
}
