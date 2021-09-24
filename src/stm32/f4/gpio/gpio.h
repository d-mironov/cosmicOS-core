#ifndef _DRV_GPIO_H
#define _DRV_GPIO_H

#include <stm32f4xx.h>

#define NULL (void*) 0x00

/* Values to write to GPIO */
#define GPIO_ON     0x01
#define GPIO_OFF    0x00

// How many pins there are per port
#define PINS_PER_PORT   0x10

/* Speeds of GPIO */
#define GPIO_LOW_SPEED      0x00
#define GPIO_MEDIUM_SPEED   0x01
#define GPIO_FAST_SPEED     0x02
#define GPIO_HIGH_SPEED     0x03

/* Pull up/down of GPIO */
#define GPIO_NO_PULL_UP_DOWN    0x00
#define GPIO_PULL_UP            0x01
#define GPIO_PULL_DOWN          0x02

// Push pull/Open drain
#define GPIO_OUT_PP     0x00
#define GPIO_OUT_OD     0x01

// Alternate function selection of GPIO
#define GPIO_AF00       0x00
#define GPIO_AF01       0x01
#define GPIO_AF02       0x02
#define GPIO_AF03       0x03
#define GPIO_AF04       0x04
#define GPIO_AF05       0x05
#define GPIO_AF06       0x06
#define GPIO_AF07       0x07
#define GPIO_AF08       0x08
#define GPIO_AF09       0x09
#define GPIO_AF10       0x0A
#define GPIO_AF11       0x0B
#define GPIO_AF12       0x0C
#define GPIO_AF13       0x0D
#define GPIO_AF14       0x0E
#define GPIO_AF15       0x0F

#define ADC_PA0     0x00
#define ADC_PA1     0x01
#define ADC_PA2     0x02
#define ADC_PA3     0x03
#define ADC_PA4     0x04
#define ADC_PA5     0x05
#define ADC_PA6     0x06
#define ADC_PA7     0x07
#define ADC_PB0     0x08
#define ADC_PB1     0x09
#define ADC_PC0     0x0A
#define ADC_PC1     0x0B
#define ADC_PC2     0x0C
#define ADC_PC3     0x0D
#define ADC_PC4     0x0E
#define ADC_PC5     0x0F






typedef enum gpio_mode {
    GPIO_INPUT,
    GPIO_OUTPUT,
    GPIO_ALTERNATE,
    GPIO_ANALOG,
    GPIO_OUTPUT_PULLUP,
    GPIO_OUTPUT_PULLDOWN,
} gpio_mode_t;

typedef enum gpio_err {
    GPIO_OK,
    GPIO_PIN_TOO_HIGH,
    GPIO_ALTERNATE_FUNC_TOO_HIGH,
    GPIO_ALTERNATE_NOT_SELECTED,
    GPIO_INVALID_SETTING
} gpio_err_t;

/**
 * GPIO pin definitions
 */
typedef enum _gpio_pin_t {
    // GPIOA pins
    PA0,PA1,PA2,PA3,PA4,PA5,PA6,PA7,
    PA8,PA9,PA10,PA11,PA12,PA13,PA14,PA15,
    // GPIOB pins
    PB0,PB1,PB2,PB3,PB4,PB5,PB6,PB7,
    PB8,PB9,PB10,PB11,PB12,PB13,PB14,PB15,
    // GPIOC pins
    PC0,PC1,PC2,PC3,PC4,PC5,PC6,PC7,
    PC8,PC9,PC10,PC11,PC12,PC13,PC14,PC15,
    // GPIOD pins
    PD0,PD1,PD2,PD3,PD4,PD5,PD6,PD7,
    PD8,PD9,PD10,PD11,PD12,PD13,PD14,PD15,
    // GPIOE pins
    PE0,PE1,PE2,PE3,PE4,PE5,PE6,PE7,
    PE8,PE9,PE10,PE11,PE12,PE13,PE14,PE15, 
    // GPIOH pins
    PH0,PH1,PH2,PH3,PH4,PH5,PH6,PH7,
    PH8,PH9,PH10,PH11,PH12,PH13,PH14,PH15,
} gpio_pin_t;

typedef struct _gpio_init_t { 
    uint8_t pin;
    uint8_t mode;
    uint8_t speed;
    uint8_t pull_up_down;
    uint8_t push_pull_open_drain;
} gpio_init_t;

gpio_err_t GPIO_init(gpio_init_t *gpio);
gpio_err_t GPIO_enable(const gpio_pin_t, gpio_mode_t mode);
gpio_err_t GPIO_settings(const gpio_pin_t, const uint8_t speed, const uint8_t pull_up_down, const uint8_t push_pull_open_drain);
gpio_err_t GPIO_set_speed(const gpio_pin_t, const uint8_t speed);
gpio_err_t GPIO_set_pull_up_down(const gpio_pin_t pin, const uint8_t pull_up_down);

gpio_err_t GPIO_toggle(const gpio_pin_t pin);
gpio_err_t GPIO_write(const gpio_pin_t pin, const uint8_t on_off);

uint8_t GPIO_read_digital(const gpio_pin_t pin);
uint16_t GPIO_read_analog(const gpio_pin_t pin);
gpio_err_t GPIO_select_alternate(const gpio_pin_t pin, const uint8_t af);

gpio_err_t GPIO_lock(const gpio_pin_t pin);

GPIO_TypeDef *_GPIO_fetch_port(const gpio_pin_t pin);



#endif
