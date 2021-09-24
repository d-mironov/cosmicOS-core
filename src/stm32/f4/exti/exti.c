#include "exti.h"
#include "../gpio/gpio.h"
#include "../uart/uart.h"
#include <stm32f4xx.h>



exti_err_t EXTI_select_trigger(uint32_t lines, exti_trigger_t trigger) {
    if (lines & EXTI_RESERVED) {
        return EXTI_LINES_RESERVED;
    }

    switch (trigger) {
        case EXTI_RISING_EDGE:
            EXTI->RTSR &= ~(lines);
            EXTI->RTSR |= lines;
            break;
        case EXTI_FALLING_EDGE:
            EXTI->FTSR &= ~(lines);
            EXTI->FTSR |= lines;
            break;
        default:
            EXTI->RTSR &= ~(lines);
            EXTI->FTSR &= ~(lines);
            EXTI->RTSR |= lines;
            EXTI->FTSR |= lines;
            break;
    }

    return EXTI_OK;
}



exti_err_t EXTI_unmask(uint32_t lines) {
    if (lines & EXTI_RESERVED) {
        return EXTI_LINES_RESERVED;
    }

    EXTI->IMR |= lines;

    return EXTI_OK;
}


exti_err_t EXTI_nvic_enable_irq(uint8_t pin) {
    if (pin == 0) {
        NVIC_EnableIRQ(EXTI0_IRQn);
    } else if (pin == 1) {
        NVIC_EnableIRQ(EXTI1_IRQn);
    } else if (pin == 2) {
        NVIC_EnableIRQ(EXTI2_IRQn);
    } else if (pin == 3) {
        NVIC_EnableIRQ(EXTI3_IRQn);
    } else if (pin == 4) {
        NVIC_EnableIRQ(EXTI4_IRQn);
    } else if (pin >= 5 && pin <= 9) {
        NVIC_EnableIRQ(EXTI9_5_IRQn);
    } else if (pin >= 10 && pin <= 15) {
        NVIC_EnableIRQ(EXTI15_10_IRQn);
    } else {
        return EXTI_PIN_TOO_HIGH;
    }
    return EXTI_OK;
}


exti_err_t EXTI_attach_gpio(const gpio_pin_t pin, exti_trigger_t trigger) {
    if (pin > 15) {
        return EXTI_PIN_TOO_HIGH;
    }

    __disable_irq();
    
    GPIO_enable(pin, GPIO_INPUT);
    GPIO_TypeDef *port = _GPIO_fetch_port(pin);
    if (port == NULL) {
        return EXTI_PIN_TOO_HIGH;
    }
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // uint8_t cr = (pin / 4) % 4;
    uint8_t exti_port = ((uint64_t) port - AHB1PERIPH_BASE) / 0x00400UL;
    // Calculations for port and pin on the EXTI line selection
    // Don't try to understand this, if you don't want you mind to blow up lol
    SYSCFG->EXTICR[(pin/4) % 4] |= (exti_port << ((pin % SYSCFG_EXTI_PORTS_PER_REG) * SYSCFG_EXTI_BITNUM)); 
    
    if ( EXTI_unmask( (1 << pin) ) != EXTI_OK) {
        __enable_irq();
        return EXTI_LINES_RESERVED;
    }

    if ( EXTI_select_trigger((1<<pin), trigger) != EXTI_OK ) {
        __enable_irq();
        return EXTI_LINES_RESERVED;
    }

    EXTI_nvic_enable_irq(pin);

    __enable_irq();
    return EXTI_OK;
}


