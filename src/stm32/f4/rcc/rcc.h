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
#include <stdbool.h>

#define RCC_PLL_Q_OFFSET    (24)
#define RCC_PLLSRC_OFFSET   (22)
#define RCC_PLL_P_OFFSET    (16)
#define RCC_PLL_N_OFFSET    (6)
#define RCC_PLL_M_OFFSET    (0)
#define RCC_PPRE1_OFFSET    (10)
#define RCC_PPRE2_OFFSET    (13)
#define RCC_HPRE_OFFSET     (4)

#define RCC_PLL_P_2     (0x00 << RCC_PLL_P_OFFSET)
#define RCC_PLL_P_4     (0x01 << RCC_PLL_P_OFFSET)
#define RCC_PLL_P_6     (0x02 << RCC_PLL_P_OFFSET)
#define RCC_PLL_P_8     (0x08 << RCC_PLL_P_OFFSET)

#define RCC_PLL_Q_2     (0x02 << RCC_PLL_Q_OFFSET)
#define RCC_PLL_Q_3     (0x03 << RCC_PLL_Q_OFFSET)
#define RCC_PLL_Q_4     (0x04 << RCC_PLL_Q_OFFSET)
#define RCC_PLL_Q_5     (0x05 << RCC_PLL_Q_OFFSET)
#define RCC_PLL_Q_6     (0x06 << RCC_PLL_Q_OFFSET)
#define RCC_PLL_Q_7     (0x07 << RCC_PLL_Q_OFFSET)
#define RCC_PLL_Q_8     (0x08 << RCC_PLL_Q_OFFSET)
#define RCC_PLL_Q_9     (0x09 << RCC_PLL_Q_OFFSET)
#define RCC_PLL_Q_10    (0x0A << RCC_PLL_Q_OFFSET)
#define RCC_PLL_Q_11    (0x0B << RCC_PLL_Q_OFFSET)
#define RCC_PLL_Q_12    (0x0C << RCC_PLL_Q_OFFSET)
#define RCC_PLL_Q_13    (0x0D << RCC_PLL_Q_OFFSET)
#define RCC_PLL_Q_14    (0x0E << RCC_PLL_Q_OFFSET)
#define RCC_PLL_Q_15    (0x0F << RCC_PLL_Q_OFFSET)


#define RCC_PLLRDY      (1 << 25)
#define RCC_HSERDY      (1 << 17)
#define RCC_HSIRDY      (1 << 1)


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


#define RCC_25MHZ_TO_100MHZ  rcc_hse_25_mhz_to_100_mhz

typedef enum {
    RCC_PLLSRC_HSI,
    RCC_PLLSRC_HSE
} rcc_pllsrc_t;

typedef enum {
    RCC_SYSCLK_HSI,
    RCC_SYSCLK_HSE,
    RCC_SYSCLK_PLL
} rcc_sysclksrc_t;

typedef enum {
    RCC_OSC_HSE     =  (1 << 16),
    RCC_OSC_HSI     =  (1 << 0),
    RCC_OSC_PLL     =  (1 << 24),
    RCC_OSC_CSS     =  (1 << 19),
    RCC_OSC_PLLI2S  =  (1 << 26),
} rcc_osc_t;

typedef enum {
    RCC_APB1_NODIV  = (0x00),     
    RCC_APB1_DIV_2  = (0x04 << RCC_PPRE1_OFFSET),     
    RCC_APB1_DIV_4  = (0x05 << RCC_PPRE1_OFFSET),     
    RCC_APB1_DIV_8  = (0x06 << RCC_PPRE1_OFFSET),     
    RCC_APB1_DIV_16 = (0x07 << RCC_PPRE1_OFFSET),     
} rcc_apb1_pre_t;

typedef enum {
    RCC_APB2_NODIV  = (0x00),     
    RCC_APB2_DIV_2  = (0x04 << RCC_PPRE2_OFFSET),     
    RCC_APB2_DIV_4  = (0x05 << RCC_PPRE2_OFFSET),     
    RCC_APB2_DIV_8  = (0x06 << RCC_PPRE2_OFFSET),     
    RCC_APB2_DIV_16 = (0x07 << RCC_PPRE2_OFFSET),     
} rcc_apb2_pre_t;

typedef enum {
    RCC_AHB_NODIV   = (0x00 << RCC_HPRE_OFFSET),   
    RCC_AHB_DIV_2   = (0x08 << RCC_HPRE_OFFSET),
    RCC_AHB_DIV_4   = (0x09 << RCC_HPRE_OFFSET),
    RCC_AHB_DIV_8   = (0x0A << RCC_HPRE_OFFSET),
    RCC_AHB_DIV_16  = (0x0B << RCC_HPRE_OFFSET),
    RCC_AHB_DIV_64  = (0x0C << RCC_HPRE_OFFSET),
    RCC_AHB_DIV_128 = (0x0D << RCC_HPRE_OFFSET),
    RCC_AHB_DIV_256 = (0x0E << RCC_HPRE_OFFSET),
    RCC_AHB_DIV_512 = (0x0F << RCC_HPRE_OFFSET),
} rcc_ahb_pre_t;


/**
 * Clock configuration structure
 */
typedef struct clock_conf {
    uint32_t hse_clock;

    uint32_t pll_m;
    uint32_t pll_n;
    uint32_t pll_p;
    uint32_t pll_q;

    rcc_ahb_pre_t ahb_pre;
    rcc_apb1_pre_t apb1_pre;
    rcc_apb2_pre_t apb2_pre;

    rcc_pllsrc_t pll_src;
    rcc_sysclksrc_t sysclk_src; 

    uint32_t ahb_freq;
    uint32_t apb1_freq;
    uint32_t apb2_freq;
    uint32_t usb_swio_freq;
} clock_t;


uint32_t ahb_freq;
uint32_t apb1_freq;
uint32_t apb2_freq;

typedef enum rcc_clock_port {
    RCC_AHB1,
    RCC_AHB2,
    RCC_APB1,
    RCC_APB2,
} rcc_clock_port_t;



volatile static clock_t rcc_hse_25_mhz_to_100_mhz = {
    .hse_clock = 25000000,
    .pll_src = RCC_PLLSRC_HSE,
    .pll_m = 16,
    .pll_n = 256,
    .pll_p = RCC_PLL_P_4,
    .pll_q = RCC_PLL_Q_9,
    .ahb_pre = RCC_AHB_NODIV,
    .apb1_pre = RCC_APB1_DIV_2,
    .apb2_pre = RCC_APB2_NODIV,
    .sysclk_src = RCC_SYSCLK_PLL,
    .ahb_freq = 100000000,
    .apb1_freq = 50000000,
    .apb2_freq = 100000000,
    .usb_swio_freq = 44444444,
};


volatile static clock_t rcc_hse_25_mhz_to_96_mhz = {
    .hse_clock = 25000000,
    .pll_src = RCC_PLLSRC_HSE,
    .pll_m = 25,
    .pll_n = 192,
    .pll_p = RCC_PLL_P_2,
    .pll_q = RCC_PLL_Q_4,
    .ahb_pre = RCC_AHB_NODIV,
    .apb1_pre = RCC_APB1_DIV_2,
    .apb2_pre = RCC_APB2_NODIV,
    .sysclk_src = RCC_SYSCLK_PLL,
    .ahb_freq = 96000000,
    .apb1_freq = 48000000,
    .apb2_freq = 960000000,
    .usb_swio_freq = 48000000,
};

volatile static clock_t *rcc_active_clock;

void RCC_system_clock_config(clock_t clock);

void RCC_periphclock_enable(rcc_clock_port_t port, uint32_t periph, uint8_t enable);
void RCC_set_pllm_pre(uint32_t pll_m);
void RCC_set_plln_pre(uint32_t pll_n);
void RCC_set_pllp_pre(uint32_t pll_p);
void RCC_set_pllq_pre(uint32_t pll_q);
void RCC_set_ahb_pre(rcc_ahb_pre_t ahb_pre);
void RCC_set_apb1_pre(rcc_apb1_pre_t apb1_pre);
void RCC_set_apb2_pre(rcc_apb2_pre_t apb2_pre);

void RCC_set_osc(rcc_osc_t osc);
void RCC_set_sysclk_src(rcc_sysclksrc_t src);
void RCC_set_pll_src(rcc_pllsrc_t src);

void RCC_reset_osc(rcc_osc_t osc);
void RCC_reset_ahb_pre(rcc_ahb_pre_t ahb_pre);
void RCC_reset_apb1_pre(rcc_apb1_pre_t apb1_pre);
void RCC_reset_apb2_pre(rcc_apb2_pre_t apb2_pre);

bool RCC_osc_rdy(rcc_osc_t osc);
void RCC_wait_osc_rdy(rcc_osc_t osc);
bool RCC_sysclk_rdy(rcc_sysclksrc_t src);
void RCC_wait_sysclk_rdy(rcc_sysclksrc_t src);



#endif

