#include "stm32.h"
#include "stm32f4xx.h"
#include "f4/delay/delay.h"
#include "f4/rcc/rcc.h"


void cosmicOS_init() {
    RCC_periphclock_enable(RCC_APB1, RCC_APB1_TIM5, RCC_ENABLE);
    TIM5->PSC = (apb1_freq/1000)-1;
    TIM5->ARR = 0xFFFF-1;
    TIM5->CR1 |= TIM_CR1_CEN;
    //delayMs(4); 
    //SysTick->CTRL &= ~ST_CTRL_ENABLE;    
    //SysTick->LOAD |= (apb1_freq/1000000)-1;
    //SysTick->CTRL |= ST_CTRL_INTEN | ST_CTRL_CLKSRC | ST_CTRL_ENABLE;
    //SysTick->VAL = 0x00;

    
}



