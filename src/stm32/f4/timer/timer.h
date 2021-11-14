#ifndef _COSMIC_TIMER_H
#define _COSMIC_TIMER_H
#include "stm32f4xx.h"
#include <stdbool.h>
#include "../../../cosmic.h"

#define NULL (void*)0

#define TIM_MAX_PSC     (0xFFFF)
#define TIM_MAX_ARR     (0xFFFF)

#define TIM_DIR_OFFSET  (0x04)

#define TIM_UIE  (1U << 0x00)
#define TIM_TIE  (1U << 0x06)

#define TIM_UIE_FLAG    (1U << 0x00)

void (*tim2_it_func)();
void (*tim3_it_func)();
void (*tim4_it_func)();
void (*tim5_it_func)();
void (*tim1_it_func)();
void (*tim9_it_func)();
void (*tim10_it_func)();
void (*tim11_it_func)();

typedef enum {
    TIM_DIR_UP   = (0 << TIM_DIR_OFFSET),
    TIM_DIR_DOWN = (1 << TIM_DIR_OFFSET),
} tim_dir_t;

/**
 * Timer object structure
 * Use in combination with 
 * all the other functions
 */
typedef struct _timer {
    TIM_TypeDef *timer;         /*! `TIMx` structure which timer to use  */
    u32 prescaler;              /*! Prescaler value - normaly stuff like: `AHB_FREQ/1000` for 1ms*/
    u32 autoreload;             /*! autoreload value - value the timer will count up to */
    tim_dir_t dir;              /*! counting direction (`RCC_DIR_UP`, `RCC_DIR_DOWN`)*/
    bool interrup_en;           /*! enable interrupt on update event */
    void (*func)(void);         /*! function to execute on interrupt */
} timer;

typedef enum timer_err {
    TIM_OK,
    TIM_ERR_CONFIG_NO_TIMER,
    TIM_ERR_CONFIG_PRESCALER_OVERFLOW,
    TIM_ERR_CONFIG_PRESCALER_ZERO,
    TIM_ERR_CONFIG_AUTORELOAD
} tim_err_t;



tim_err_t TIM_init(const struct _timer *port);

tim_err_t TIM_set_prescaler(const struct _timer *port);
tim_err_t TIM_set_autoreload(const struct _timer *port);
tim_err_t TIM_set_dir(const struct _timer *port);

tim_err_t TIM_reset_count(const struct _timer *port);

bool TIM_is_TIM2_5(const struct _timer *port);
void TIM_rcc_enable(const struct _timer *port);
void TIM_rcc_disable(const struct _timer *port);

str TIM_err_str(const tim_err_t err);

void _TIM_NVIC_enable(const struct _timer *port);

#endif
