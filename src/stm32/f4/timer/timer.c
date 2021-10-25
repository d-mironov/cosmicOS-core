
#include "timer.h"
#include "stm32f4xx.h"
#include "../rcc/rcc.h"


tim_err_t TIM_init(const struct timer_port *port) {
    // check if Timer port is NULL
    if (port->timer == NULL) {
        return TIM_ERR_CONFIG_NO_TIMER;
    }
    // check if prescaler or autoreload value is = 0
    if (port->prescaler == 0 || port->prescaler >= TIM_MAX_PSC) {
        return TIM_ERR_CONFIG_PRESCALER;
    }
    // check if timer is not 32-bit and value is >16-bit
    if (!(port->timer == TIM2 || port->timer == TIM5) && 
            (port->autoreload == 0 || port->autoreload >= TIM_MAX_ARR)) {
        return TIM_ERR_CONFIG_AUTORELOAD;
    }
    // Enable clock to timer
    TIM_rcc_enable(port);
    // set timer prescaler
    TIM_set_prescaler(port);
    // set autoreload value
    TIM_set_autoreload(port);
    // reset counter
    TIM_reset_count(port);
    // set counting direction
    TIM_set_dir(port);
    if (port->interrup_en) {
        port->timer->DIER |= TIM_UIE;
        _TIM_NVIC_enable(port);
    }

    return TIM_OK;
}


bool TIM_is_TIM2_5(const struct timer_port *port) {
    return (port->timer==TIM2)||(port->timer==TIM3)||(port->timer==TIM4)||(port->timer==TIM5);
}

void TIM_rcc_enable(const struct timer_port *port) {
    if (port->timer == TIM2) {
        RCC_periphclock_enable(RCC_APB1, RCC_APB1_TIM2, RCC_ENABLE);
        tim2_it_func = port->func;
    } else if (port->timer == TIM3) {
        RCC_periphclock_enable(RCC_APB1, RCC_APB1_TIM3, RCC_ENABLE);
        tim3_it_func = port->func;
    } else if (port->timer == TIM4) {
        RCC_periphclock_enable(RCC_APB1, RCC_APB1_TIM4, RCC_ENABLE);
        tim4_it_func = port->func;
    } else if (port->timer == TIM5) {
        RCC_periphclock_enable(RCC_APB1, RCC_APB1_TIM5, RCC_ENABLE);
        tim5_it_func = port->func;
    } else if (port->timer == TIM9) {
        RCC_periphclock_enable(RCC_APB2, RCC_APB2_TIM9, RCC_ENABLE);
        tim9_it_func = port->func;
    } else if (port->timer == TIM1) {
        RCC_periphclock_enable(RCC_APB2, RCC_APB2_TIM1, RCC_ENABLE);
        tim1_it_func = port->func;
    } else if (port->timer == TIM10) {
        RCC_periphclock_enable(RCC_APB2, RCC_APB2_TIM10, RCC_ENABLE);
        tim10_it_func = port->func;
    } else if (port->timer == TIM11) {
        RCC_periphclock_enable(RCC_APB2, RCC_APB2_TIM11, RCC_ENABLE);
        tim11_it_func = port->func;
    }
}

void TIM_rcc_disable(const struct timer_port *port) {
    if (port->timer == TIM2) {
        RCC_periphclock_enable(RCC_APB1, RCC_APB1_TIM2, RCC_DISABLE);
    } else if (port->timer == TIM3) {
        RCC_periphclock_enable(RCC_APB1, RCC_APB1_TIM3, RCC_DISABLE);
    } else if (port->timer == TIM4) {
        RCC_periphclock_enable(RCC_APB1, RCC_APB1_TIM4, RCC_DISABLE);
    } else if (port->timer == TIM5) {
        RCC_periphclock_enable(RCC_APB1, RCC_APB1_TIM5, RCC_DISABLE);
    } else if (port->timer == TIM9) {
        RCC_periphclock_enable(RCC_APB2, RCC_APB2_TIM9, RCC_DISABLE);
    } else if (port->timer == TIM1) {
        RCC_periphclock_enable(RCC_APB2, RCC_APB2_TIM1, RCC_DISABLE);
    } else if (port->timer == TIM10) {
        RCC_periphclock_enable(RCC_APB2, RCC_APB2_TIM10, RCC_DISABLE);
    } else if (port->timer == TIM11) {
        RCC_periphclock_enable(RCC_APB2, RCC_APB2_TIM11, RCC_DISABLE);
    }
}

tim_err_t TIM_set_prescaler(const struct timer_port *port) {
    if (port->timer == NULL) {
        return TIM_ERR_CONFIG_NO_TIMER;
    }
    port->timer->PSC = (port->prescaler)-1;
    return TIM_OK;
}

tim_err_t TIM_set_autoreload(const struct timer_port *port) {
    if (port->timer == NULL) {
        return TIM_ERR_CONFIG_NO_TIMER;
    }
    port->timer->ARR = (port->autoreload);
    return TIM_OK;
}

tim_err_t TIM_reset_count(const struct timer_port *port) {
    port->timer->CNT = 0;
    return TIM_OK;
}

tim_err_t TIM_set_dir(const struct timer_port *port) {
    port->timer->CR1 &= ~TIM_DIR_DOWN;
    port->timer->CR1 |= port->dir;
    return TIM_OK;
}

uint32_t millis() {
    return TIM5->CNT;
}

uint32_t micros() {
    return TIM5->CNT;
}

void _TIM_NVIC_enable(const struct timer_port *port) {
    if (port->timer == TIM2) {
        NVIC_EnableIRQ(TIM2_IRQn);
    } else if (port->timer == TIM3) {
        NVIC_EnableIRQ(TIM3_IRQn);
    } else if (port->timer == TIM4) {
        NVIC_EnableIRQ(TIM4_IRQn);
    } else if (port->timer == TIM5) {
        NVIC_EnableIRQ(TIM5_IRQn);
    }
}

void TIM1_IRQHandler(void) {
    if (tim1_it_func != NULL && TIM1->SR & TIM_UIE_FLAG) {
        tim1_it_func();
        TIM1->SR &= ~(TIM_UIE_FLAG);
    }
}
void TIM2_IRQHandler(void) {
    if (tim2_it_func != NULL && TIM2->SR & TIM_UIE_FLAG) {
        tim2_it_func();
        TIM2->SR &= ~(TIM_UIE_FLAG);
    }
}
void TIM3_IRQHandler(void) {
    if (tim3_it_func != NULL && TIM3->SR & TIM_UIE_FLAG) {
        tim3_it_func();
        TIM3->SR &= ~(TIM_UIE_FLAG);
    }
}
void TIM4_IRQHandler(void) {
    if (tim4_it_func != NULL && TIM4->SR & TIM_UIE_FLAG) {
        tim4_it_func();
        TIM4->SR &= ~(TIM_UIE_FLAG);
    }
}
void TIM5_IRQHandler(void) {
    tim5_it_func();
    if (tim5_it_func != NULL && TIM5->SR & TIM_UIE_FLAG) {
        TIM5->SR &= ~(TIM_UIE_FLAG);
    }
}
void TIM9_IRQHandler(void) {
    if (tim9_it_func != NULL && TIM9->SR & TIM_UIE_FLAG) {
        tim9_it_func();
        TIM9->SR &= ~(TIM_UIE_FLAG);
    }
}
void TIM10_IRQHandler(void) {
    if (tim10_it_func != NULL && TIM10->SR & TIM_UIE_FLAG) {
        tim10_it_func();
        TIM10->SR &= ~(TIM_UIE_FLAG);
    }
}
void TIM11_IRQHandler(void) {
    if (tim11_it_func != NULL && TIM11->SR & TIM_UIE_FLAG) {
        tim11_it_func();
        TIM11->SR &= ~(TIM_UIE_FLAG);
    }
}
