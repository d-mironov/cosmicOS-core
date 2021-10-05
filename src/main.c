#include <stm32f4xx.h>
#include <stdbool.h>
#include "stm32/f4/gpio/gpio.h"
#include "stm32/f4/delay/delay.h"
#include "stm32/f4/uart/uart.h"
#include "stm32/f4/i2c/i2c.h" 
#include "drivers/mpu6050/mpu.h"
#include "bitutils.h"
//#include "stm32/f4/rcc/rcc.h"
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



int main(void) {
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
        .accel_range = ACCEL_2G,
        .gyro_range = GYRO_500_DEG_S,
        .port = i2c1,
    };
    GPIO_enable(DEBUG_LED, GPIO_OUTPUT);
    GPIO_enable(PA8, GPIO_OUTPUT);
    
    USART_init(&port);   
    mpu_err_t mpu_err;
    mpu_err = MPU_init(&mpu);
    if (mpu_err != MPU_OK) {
        USART_printf(port, "[x] Error initializing MPU\n");
    } else {
        USART_printf(port, "[+] MPU init success\n");
    }
    delayMs(2000);
    I2C_init(&i2c1);
    i2c_err_t i2c_err;
    uint8_t init[2] = {0x00, 0x00};
    //i2c_err = I2C_write(i2c1, MPU_ADDR, 0x6B, 0x00);
    USART_printf(port, "%s\n", I2C_get_err_str(i2c_err));
    delayMs(2000);
    
    //const clock_t *test = &RCC_25MHZ_TO_84MHZ;
    char usart_test[255] = {0};
    unsigned long int cycle = 0; 
    uint8_t bit_test = 0;
    uint8_t i2c_data[2]; 
    while (1) {
        GPIO_toggle(DEBUG_LED);
        
        //if (USART_available(port)) {
        //    USART_scan(port, usart_test, 255);
        //    USART_printf(port, "%s\n", usart_test);
        //}
        //if (USART_printf(port, "Cycle: %d\n", cycle++) != USART_OK) {
        //    USART_printf(port, "[x] printing did not work\n");
        //}

        //USART_printf(USART2, "This is a value: %.3f\n", 2.123);
        //USART_printf(&port, "x: %6d\n", read_x(&i2c1));
        //i2c_err = I2C_read(i2c1, MPU_ADDR, 0x3C, &i2c_data);
        //USART_printf(port, "%s\n", I2C_get_err_str(i2c_err)); 
        //uint16_t mpu_test = read_x(i2c1, port); 
        USART_printf(port, "Gyro -> X: %5d    Y: %5d    Z: %5d        Accel -> X: %5d    Y: %5d    Z: %5d\r", 
                MPU_gyro_x_raw(&mpu), 
                MPU_gyro_y_raw(&mpu), 
                MPU_gyro_z_raw(&mpu),
                MPU_accel_x_raw(&mpu),
                MPU_accel_y_raw(&mpu),
                MPU_accel_z_raw(&mpu)
        );
        //i2c_err = I2C_read_burst(i2c1, MPU_ADDR, 0x43, 2, i2c_data);
        //USART_printf(port, "%s\n", I2C_get_err_str(i2c_err));
        //USART_printf(port, "%d\n", (int16_t)(i2c_data[0] << 8) | i2c_data[1]);
                


        delayMs(100);
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
