/*
 * TODO: 
 *   - [TESTING] Clock enabling on AHB1, AHB2, APB1, APB2
 *   - [TESTING] resetting Peripheral clocks
 *   - [ ] Clock configuration for STM32F4
 *   - [ ] Clock Control register selection
 *   - [ ] PLL configuration 
 *   - [ ] RCC_CFGR configuration
 */
#ifndef _STM_RCC_H
#define _STM_RCC_H

#include <stm32f4xx.h>

#define RCC_PLL_P_2     0x00
#define RCC_PLL_P_4     0x01
#define RCC_PLL_P_6     0x02
#define RCC_PLL_P_8     0x08

#define RCC_PLL_Q_2     0x02
#define RCC_PLL_Q_3     0x03
#define RCC_PLL_Q_4     0x04
#define RCC_PLL_Q_5     0x05
#define RCC_PLL_Q_6     0x06
#define RCC_PLL_Q_7     0x07
#define RCC_PLL_Q_8     0x08
#define RCC_PLL_Q_9     0x09
#define RCC_PLL_Q_10    0x0A
#define RCC_PLL_Q_11    0x0B
#define RCC_PLL_Q_12    0x0C
#define RCC_PLL_Q_13    0x0D
#define RCC_PLL_Q_14    0x0E
#define RCC_PLL_Q_15    0x0F

#define RCC_APB1_NODIV      0x01
#define RCC_APB1_DIV_2      0x02
#define RCC_APB1_DIV_4      0x03
#define RCC_APB1_DIV_8      0x04
#define RCC_APB1_DIV_16     0x05

#define RCC_APB2_NODIV      0x00
#define RCC_APB2_DIV_2      0x01
#define RCC_APB2_DIV_4      0x02
#define RCC_APB2_DIV_8      0x03
#define RCC_APB2_DIV_16     0x04

#define RCC_AHB_NODIV       0x00
#define RCC_AHB_DIV_2       0x01
#define RCC_AHB_DIV_4       0x02
#define RCC_AHB_DIV_8       0x03
#define RCC_AHB_DIV_16      0x04
#define RCC_AHB_DIV_32      0x05
#define RCC_AHB_DIV_64      0x06
#define RCC_AHB_DIV_128     0x07
#define RCC_AHB_DIV_256     0x08
#define RCC_AHB_DIV_512     0x09


#define RCC_AHB1_DMA2     (1U << 22)
#define RCC_AHB1_DMA1     (1U << 21)
#define RCC_AHB1_CRC      (1U << 12)
#define RCC_AHB1_GPIOH    (1U << 7)
#define RCC_AHB1_GPIOE    (1U << 4)
#define RCC_AHB1_GPIOD    (1U << 3)
#define RCC_AHB1_GPIOC    (1U << 2)
#define RCC_AHB1_GPIOB    (1U << 1)
#define RCC_AHB1_GPIOA    (1U << 0)

#define RCC_APB1_PWR      (1U << 28)
#define RCC_APB1_I2C3     (1U << 23)
#define RCC_APB1_I2C2     (1U << 22)
#define RCC_APB1_I2C1     (1U << 21)
#define RCC_APB1_USART2   (1U << 17)
#define RCC_APB1_SPI3     (1U << 15)
#define RCC_APB1_SPI2     (1U << 14)
#define RCC_APB1_WWDG     (1U << 11)
#define RCC_APB1_TIM5     (1U << 3)
#define RCC_APB1_TIM4     (1U << 2)
#define RCC_APB1_TIM3     (1U << 1)
#define RCC_APB1_TIM2     (1U << 0)

#define RCC_APB2_SPI5     (1U << 20)
#define RCC_APB2_TIM11    (1U << 18)
#define RCC_APB2_TIM10    (1U << 17)
#define RCC_APB2_TIM9     (1U << 16)
#define RCC_APB2_SYSCF    (1U << 14)
#define RCC_APB2_SPI4     (1U << 13)
#define RCC_APB2_SPI1     (1U << 12)
#define RCC_APB2_SDIO     (1U << 11)
#define RCC_APB2_ADC1     (1U << 8)
#define RCC_APB2_USART6   (1U << 5)
#define RCC_APB2_USART1   (1U << 4)
#define RCC_APB2_TIM1     (1U << 0)

#define RCC_ENABLE          1U
#define RCC_DISABLE         0U


#define RCC_25MHZ_TO_84MHZ  rcc_hse_25_mhz_to_84_mhz

typedef enum {
    PLL_SRC_HSI,
    PLL_SRC_HSE
} rcc_pllsrc_t;

typedef enum {
    SYSCLK_HSI,
    SYSCLK_HSE,
    SYSCLK_PLL
} sysclk_src_t;



/**
 * Clock configuration structure
 */
typedef struct clock_conf {
    uint32_t hse_clock;

    rcc_pllsrc_t pll_source;
    uint16_t pll_m;
    uint16_t pll_n;
    uint16_t pll_p;
    uint16_t pll_q;

    uint8_t ahb_pre;
    uint8_t apb1_pre;
    uint8_t apb2_pre;

    sysclk_src_t sysclk_source; 

    uint32_t ahb_freq;
    uint32_t apb1_freq;
    uint32_t apb2_freq;
    uint32_t usb_swio_freq;
} clock_t;

typedef enum rcc_clock_port {
    RCC_AHB1,
    RCC_AHB2,
    RCC_APB1,
    RCC_APB2,
} rcc_clock_port_t;

static const clock_t rcc_hse_25_mhz_to_84_mhz = {
    .hse_clock = 25000000,
    .pll_source = PLL_SRC_HSE,
    .pll_m = 25,
    .pll_n = 336,
    .pll_p = RCC_PLL_P_4,
    .pll_q = RCC_PLL_Q_7,
    .ahb_pre = RCC_AHB_NODIV,
    .apb1_pre = RCC_APB1_DIV_2,
    .apb2_pre = RCC_APB2_NODIV,
    .sysclk_source = SYSCLK_PLL,
    .ahb_freq = 84000000,
    .apb1_freq = 42000000,
    .apb2_freq = 84000000,
    .usb_swio_freq = 48000000
};


void RCC_system_clock_config(clock_t *clock);

void RCC_periphclock_select(rcc_clock_port_t port, uint32_t periph, uint8_t enable);


#endif

