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

    USART_port gps = {
        .usart = USART1,
        .baud = 9600,
        .mode = USART_RX_MODE,
        .interrupt_driven = true,
        .stop_bits = 0,
        .parity_enable = 0,
        .parity_even_odd = 0,
    };

    mpu_t mpu = {
        .accel_range = ACCEL_4G,
        .gyro_range = GYRO_250_DEG_S,
        .port = i2c1,
    };
    GPIO_enable(DEBUG_LED, GPIO_OUTPUT);
    GPIO_enable(PA8, GPIO_OUTPUT);
    
    USART_init(&port);   
    USART_init(&gps);
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
    uint8_t bit_test = 0;
    usart_err_t usart_err;

    while (1) {
        GPIO_toggle(DEBUG_LED);
                
        //if (USART_available(gps)) {
        //    usart_err = USART_scan(gps, usart_test, 512);
        //    if (usart_err != USART_OK) {
        //        USART_printf(port, "[x] cannot scan GPS\n");
        //    }
        //    USART_printf(port, "%s\n", usart_test);
        //}
        //USART_printf(gps, "Hello USART\n");
        USART_scan(gps, usart_test, 512);
        USART_printf(port, "%s\n", usart_test);

        //if (USART_printf(port, "Cycle: %d\n", cycle++) != USART_OK) {
        //    USART_printf(port, "[x] printing did not work\n");
        //}

        //USART_printf(USART2, "This is a value: %.3f\n", 2.123);
        //USART_printf(&port, "x: %6d\n", read_x(&i2c1));
        //i2c_err = I2C_read(i2c1, MPU_ADDR, 0x3C, &i2c_data);
        //USART_printf(port, "%s\n", I2C_get_err_str(i2c_err)); 
        //uint16_t mpu_test = read_x(i2c1, port); 
        //USART_printf(port, "Gyro -> X: %5d    Y: %5d    Z: %5d        Accel -> X: %5d    Y: %5d    Z: %5d\r", 
        //        MPU_gyro_x_raw(mpu), 
        //        MPU_gyro_y_raw(mpu), 
        //        MPU_gyro_z_raw(mpu),
        //        MPU_accel_x_raw(mpu),
        //        MPU_accel_y_raw(mpu),
        //        MPU_accel_z_raw(mpu)
        //);



        float gyro[3], accel[3];
        mpu_err = MPU_gyro(mpu, gyro, 3);
        if (mpu_err != MPU_OK) {
            USART_printf(port, "[x] Gyroscope reading failed\n");
        }
        mpu_err = MPU_accel(mpu, accel, 3);
        if (mpu_err != MPU_OK) {
            USART_printf(port, "[x] Accelerometer reading failed\n");
        }
        //USART_printf(port, "Gyro ->  X: % -9.3f  Y: % -9.3f  Z: % -9.3f    Accel ->  X: % -9.3f  Y: % -9.3f  Z: % -9.3f\r", gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);

        //int32_t accel_raw[3];
        //mpu_err = MPU_accel_raw(mpu, accel_raw);
        //if (mpu_err != MPU_OK) {
        //    USART_printf(port, "[x] MPU error\n");
        //}
        //USART_printf(port, "accel ->  X: %4d  Y: %4d  Z: %4d\n", accel_raw[0], accel_raw[1], accel_raw[2]);

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
