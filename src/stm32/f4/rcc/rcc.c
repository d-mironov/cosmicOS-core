#include "rcc.h"
#include <stm32f4xx.h>


void RCC_system_clock_config(clock_t *clock);

void RCC_periphclock_enable(rcc_clock_port_t port, uint32_t periph_enable, uint8_t enable) {
    if (enable == RCC_ENABLE) {
        if (port == RCC_AHB1) {
            RCC->AHB1ENR |= periph_enable;
        } else if (port == RCC_AHB2) {
            RCC->AHB2ENR |= periph_enable;
        } else if (port == RCC_APB1) {
            RCC->APB1ENR |= periph_enable;
        } else if (port == RCC_APB2) {
            RCC->APB2ENR |= periph_enable;
        }
    } else {
        if (port == RCC_AHB1) {
            RCC->AHB1RSTR |= periph_enable;
        } else if (port == RCC_AHB2) {
            RCC->AHB2RSTR |= periph_enable;
        } else if (port == RCC_APB1) {
            RCC->APB1RSTR |= periph_enable;
        } else if (port == RCC_APB2) {
            RCC->APB2RSTR |= periph_enable;
        } 
    } 
}
