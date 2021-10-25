#include <stm32f4xx.h>
#include <stdbool.h>
#include "stm32/f4/gpio/gpio.h"
#include "stm32/f4/delay/delay.h"
#include "stm32/f4/uart/uart.h"
#include "stm32/f4/i2c/i2c.h" 
#include "drivers/mpu6050/mpu.h"
#include "bitutils.h"
#include "stm32/f4/rcc/rcc.h"
#include "stm32/f4/timer/timer.h"
#include "stm32/stm32.h"
#include "stm32/f4/timer/timer.h"
//#include "stm32/f4/exti/exti.h"

#define MPU_ADDR    0x68
#define DEBUG_LED   PB8
#define TEST_LED    PA8



uint16_t read_x(I2C_port port, USART_port usart) {
    uint8_t data1, data2;
    i2c_err_t err;
    err = I2C_read(port, MPU_ADDR, 0x3B, &data1);
    if (err != I2C_OK) {
        USART_printf(usart, "%s\n", I2C_get_err_str(err));
    }
    err = I2C_read(port, MPU_ADDR, 0x3C, &data2);
    if (err != I2C_OK) {
        USART_printf(usart, "%s\n", I2C_get_err_str(err));
    }
    return (data1 << 8) | data2;
}

void toggle_test_led(void) {
    GPIO_toggle(DEBUG_LED);        
}



int main(void) {

    RCC_system_clock_config(rcc_hse_25_mhz_to_96_mhz);   
    cosmicOS_init();
    I2C_port i2c1 = {
        .i2c = I2C1,
        .frequency = 16,
        .mode = I2C_STD_MODE,
        .duty = 0,
    };
    USART_port port = {
        .usart = USART2,
        .baud = 115200,
        .mode = USART_RX_TX_MODE,
        .stop_bits = 0,
        .parity_enable = 0,
        .parity_even_odd = 0,
        .interrupt_driven = true,
    };

    mpu_t mpu = {
        .accel_range = ACCEL_4G,
        .gyro_range = GYRO_250_DEG_S,
        .port = i2c1,
    };
    timer_port_t tim5 = {
        .timer = TIM5,
        .prescaler = apb1_freq/1000,
        .autoreload = 200,
        .func = toggle_test_led,
        .interrup_en = true,
    };
    GPIO_enable(DEBUG_LED, GPIO_OUTPUT);
    GPIO_enable(PA8, GPIO_OUTPUT);
    
    USART_init(&port);   
    //USART_init(&gps);
    mpu_err_t mpu_err;
    mpu_err = MPU_init(&mpu);
    USART_printf(port, "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    if (mpu_err != MPU_OK) {
        USART_printf(port, "[x] Error initializing MPU\n");
    } else {
        USART_printf(port, "[+] MPU init success\n");
    }
    delayMs(2000);
    I2C_init(&i2c1);
    i2c_err_t i2c_err;
    uint8_t init[2] = {0x00, 0x00};
    uint8_t i2c_data[2]; 
    //i2c_err = I2C_write(i2c1, MPU_ADDR, 0x6B, 0x00);
    USART_printf(port, "%s\n", I2C_get_err_str(i2c_err));
    delayMs(2000);
    
    //const clock_t *test = &RCC_25MHZ_TO_84MHZ;
    char usart_test[512];
    unsigned long int cycle = 0; 
    uint8_t bit_test = 0, data1;
    usart_err_t usart_err;
    USART_printf(port, "APB2 clock: %d\n", ahb_freq);
    uint32_t last_time = millis();
    TIM_init(&tim5);
    float accel[3];
    while (1) {
        USART_printf(port, "cycle: %d\n", cycle++);
        //GPIO_toggle(DEBUG_LED);        
        //while(!(TIM5->SR & 1));
        //TIM5->SR &= ~(1);
        //float gyro[3], accel[3];
        //mpu_err = MPU_gyro(mpu, gyro, 3);
        //if (mpu_err != MPU_OK) {
        //    USART_printf(port, "[x] Gyroscope reading failed\n");
        //}
        ////delayMs(2);
        i2c_err = I2C_read(i2c1, MPU_ADDR, 0x3B, &data1);
        if (i2c_err != I2C_OK) {
            USART_printf(port, "%s\n", I2C_get_err_str(i2c_err));
        }
        //mpu_err = MPU_accel(mpu, accel, 3);
        //if (mpu_err != MPU_OK) {
        //    USART_printf(port, "[x] Accelerometer reading failed\n");
        //}
        ////MPU_calibrate(mpu);
        //if (mpu_err == MPU_OK) {
        //    USART_printf(port, "Gyro ->  X: % -9.3f  Y: % -9.3f  Z: % -9.3f    Accel ->  X: % -9.3f  Y: % -9.3f  Z: % -9.3f\r", gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
        //}
        delayMs(500);
    }
}




//static void exti_callback(void) {
//    USART_printf(USART2, "Hello from EXIT07\n");
//}
//
//void EXTI9_5_IRQHandler(void) {
//    if (EXTI->PR & (1<<7)) {
//        // Clear PR flag
//        EXTI->PR |= (1<<7);
//        // Do something...
//        exti_callback();
//    }
//}
