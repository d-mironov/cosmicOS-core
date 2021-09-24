#ifndef __STM32_ADC
#define __STM32_ADC

/*
 * ├ ╰ ─ │
 * STM32 ADC driver changelog
 * v0.1
 *   ├── added #defines for different settings
 *   ├── added initial error codes
 *   ├── added initial driver struct
 *   ├── added driver mode
 *   ├── added driver resolution
 *   ├── added ADC init function
 *   ├── added ADC deinit function
 *   ╰── added private functions
 * v0.2
 *   ├──
 *   ╰──
 * TODO: ADC - figure out multi-channel ADC read
 * TODO: ADC - figure out multi-channel ADC write
 * TODO: ADC - figure out single-channel ADC read
 * TODO: ADC - figure out single channel ADC write
 * TODO: ADC - write ADC read function
 * TODO: ADC - write ADC write function
 */

#include <stm32f4xx.h>
#include <stdbool.h>
#include "../gpio/gpio.h"

#define ADC1_1 		PA1
#define ADC1_2 		PA2
#define ADC1_3 		PA3
#define ADC1_4  	PA4
#define ADC1_5  	PA5
#define ADC1_6 		PA6
#define ADC1_7 		PA7
#define ADC1_8  	PB0
#define ADC1_9  	PB1
#define ADC1_10		PC0
#define ADC1_11		PC1
#define ADC1_12		PC2
#define ADC1_13 	PC3
#define ADC1_14  	PC4
#define ADC1_15		PC5

#define ADC_

#define ADC_NUM_CHANNELS 			(0x0F)
#define ADC_NUM_CHANNELS_SQR3 		(0x06)
#define ADC_NUM_CHANNELS_SQR2		(0x06)
#define ADC_NUM_CHANNELS_SQR1 		(0x04)

#define ADC_RESOLUTION_OFFSET 		(0x18)

#define ADC_CONT_OFFSET 			(0x01)
#define ADC_SINGLE_CONT 			(0x00 << ADC_CONT_OFFSET)
#define ADC_CONTINUOUS_CONT 		(0x01 << ADC_CONT_OFFSET)

#define ADC_CR2_SWSTART_OFFSET		(0x1E)

#define ADC_SQR1_LBIT_OFFSET 		(0x14)
#define ADC_CR2_ADON_OFFSET 		(0x00)

typedef enum _ADC_mode {
	ADC_MODE_SINGLE,
	ADC_MODE_CONTINUOUS
} ADC_mode;

typedef enum _ADC_resolution {
	ADC_RESOLUTION_12BIT,
	ADC_RESOLUTION_10BIT,
	ADC_RESOLUTION_8BIT,
	ADC_RESOLUTION_6BIT
} ADC_resolution;

typedef enum adc_err {
	ADC_OK,
	ADC_ERR_PIN_NOT_ADC,
	ADC_ERR_WRONG_MODE,
	ADC_ERR_WRONG_RESOLUTION,
	ADC_ERR_PORT_ALREADY_INIT,
	ADC_ERR_NO_FREE_CHANNEL,
} adc_err_t;

typedef struct _ADC_port {
	ADC_TypeDef *port;
	gpio_pin_t pin;
	ADC_mode mode;
	ADC_resolution resolution;
	uint8_t _order;
} ADC_port;

adc_err_t ADC_init(ADC_port *self);
adc_err_t ADC_deinit(ADC_port *self);

adc_err_t ADC_read(ADC_port *self);
adc_err_t ADC_write(ADC_port *self);

adc_err_t _ADC_is_adc(gpio_pin_t pin);
int8_t _ADC_fetch_channel(ADC_port *self);

#endif
