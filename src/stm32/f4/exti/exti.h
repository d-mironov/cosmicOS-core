#ifndef _STM_EXTI_H
#define _STM_EXTI_H

#include <stm32f4xx.h>
#include "../gpio/gpio.h"

#define EXTI00      (1U << 0)
#define EXTI01      (1U << 1)
#define EXTI02      (1U << 2)
#define EXTI03      (1U << 3)
#define EXTI04      (1U << 4)
#define EXTI05      (1U << 5)
#define EXTI06      (1U << 6)
#define EXTI07      (1U << 7)
#define EXTI08      (1U << 8)
#define EXTI09      (1U << 9)
#define EXTI10      (1U << 10)
#define EXTI11      (1U << 11)
#define EXTI12      (1U << 12)
#define EXTI13      (1U << 13)
#define EXTI14      (1U << 14)
#define EXTI15      (1U << 15)
#define EXTI16      (1U << 16)
#define EXTI17      (1U << 17)
#define EXTI18      (1U << 18)
#define EXTI19      (1U << 19)
#define EXTI20      (1U << 20)
#define EXTI21      (1U << 21)
#define EXTI22      (1U << 22)

#define SYSCFG_EXTI_BITNUM          4
#define SYSCFG_EXTI_PORTS_PER_REG   4

#define EXTI_RESERVED   0xFF980000

typedef enum exti_trigger {
    EXTI_RISING_EDGE,
    EXTI_FALLING_EDGE,
    EXTI_RISING_FALLING_EDGE
} exti_trigger_t;

typedef enum exti_err {
    EXTI_OK,
    EXTI_LINES_RESERVED,
    EXTI_PIN_TOO_HIGH
} exti_err_t;


exti_err_t EXTI_select_trigger(uint32_t lines, exti_trigger_t trigger);
exti_err_t EXTI_unmask(uint32_t lines);
exti_err_t EXTI_attach_gpio(const gpio_pin_t pin, exti_trigger_t trigger);
exti_err_t EXTI_nvic_enable_irq(uint8_t pin);


#endif
