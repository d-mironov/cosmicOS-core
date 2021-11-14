#include <stm32f4xx.h>
#include <stdbool.h>
#include "stm32/f4/gpio/gpio.h"
#include "stm32/f4/delay/delay.h"
#include "stm32/f4/uart/uart.h"
#include "stm32/f4/i2c/i2c.h" 
#include "stm32/f4/timer/timer.h"
#include "stm32/f4/rcc/rcc.h"
#include "stm32/f4/timer/timer.h"
#include "drivers/mpu6050/mpu.h"
#include "bno/bno.h"
#include "stm32/stm32.h"
#include "bitutils.h"
#include "cosmic.h"
//#include "stm32/f4/exti/exti.h"

#define DEBUG_LED   PB8
#define TEST_LED    PA8

bno055 bno;

usart_err_t usart_err;
tim_err_t err_tim;
i2c_err_t i2c_err;
bno_err_t bno_err;

vec3 gyro;

/**
 * Test function for timer interrupt
 */
void toggle_test_led(void) {
    GPIO_toggle(DEBUG_LED);        
}



int main(void) {

    // Set system clock to 96MHz
    RCC_system_clock_config(rcc_hse_25_mhz_to_96_mhz);   
    //cosmicOS_init();
    
    // I2C1 init object
    I2C i2c1 = {
        .i2c = I2C1,
        .frequency = 16,
        .mode = I2C_STD_MODE,
        .duty = 0,
    };
    // USART2 init object
    USART port = {
        .usart = USART2,
        .baud = 115200,
        .mode = USART_RX_TX_MODE,
        .stop_bits = 0,
        .parity_enable = 0,
        .parity_even_odd = 0,
        .interrupt_driven = true,
    };
    //Timer 5 init object
    timer tim5 = {
        .timer = TIM5,
        .prescaler = (apb1_freq*2)/10000,
        .autoreload = 10000,
        .func = toggle_test_led,
        .interrup_en = true,
    };
    // enable GPIO output
    GPIO_enable(DEBUG_LED, GPIO_OUTPUT);
    GPIO_enable(PA8, GPIO_OUTPUT);
    
    //========================| Initialization |========================
    USART_init(&port);   
    //USART_init(&gps);
    USART_printf(port, "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    delayMs(1000);
    I2C_init(&i2c1);
    u8 i2c_data[2]; 

    // BNO 
    bno.i2c = i2c1;
    bno.mode = BNO_MODE_IMU;

    //i2c_err = I2C_write(i2c1, MPU_ADDR, 0x6B, 0x00);
    USART_printf(port, "%s\n\n", I2C_get_err_str(i2c_err));
    delayMs(1000);
    if (bno055_init(&bno)) {
        USART_printf(port, "[BNO] init success!\n");
    } else {
        USART_printf(port, "[BNO] init failed\n");
    }
    delayMs(1000);
    
    //BNO_set_temp_src(&bno, BNO_TEMP_SRC_ACC);
    bno.err = bno055_set_unit(
            &bno,
            BNO_TEMP_UNIT_C,
            BNO_GYR_UNIT_DPS,
            BNO_ACC_UNITSEL_M_S2,
            BNO_EUL_UNIT_DEG
    );
    if (bno.err != BNO_OK) {
        USART_printf(port, "[BNO] error: %s\n", bno055_err_str(bno.err));
    } else {
        USART_printf(port, "[BNO] units set!\n");
    }
    delayMs(1000);
    bno.err = bno055_set_pwr_mode(&bno, BNO_PWR_NORMAL); 
    if (bno.err != BNO_OK) {
        USART_printf(port, "[BNO] error: %s\n", bno055_err_str(bno.err));
    } else {
        USART_printf(port, "[BNO] power mode set!\n"); 
    }
    USART_printf(port, "\n");
    
    
    //const clock_t *test = &RCC_25MHZ_TO_84MHZ;
    char usart_test[512];
    unsigned long int cycle = 0; 
    u8 bit_test = 0;
    //USART_printf(port, "APB1 clock: %d\n", apb1_freq);
    //uint32_t last_time = millis();
    if ((err_tim = TIM_init(&tim5)) != TIM_OK) {
        USART_printf(port, "[TIM5] error: %s\n", TIM_err_str(err_tim));
        //USART_printf(port, "SystemCoreClock: %d\n", SystemCoreClock);
    } else {
        USART_printf(port, "[TIM5] ok!\n");
    }
    USART_printf(port, "\n");

    i8 temperature;
    u8 addr_data;
    u32 min = 0, hour = 0;
    USART_printf(port, "[System] Starting main loop...\n\n");
    delayMs(1000);
    I2C_write(i2c1, BNO_ADDR, BNO_TEMP_SOURCE, 0x00);
    i16 roll;
    //BNO_set_opmode(&bno, BNO_MODE_CONFIG);
    //BNO_set_page(&bno, 0x00);
    //i2c_err = I2C_write(i2c1, BNO_ADDR, BNO_UNIT_SEL, (1 << 0x04));
    //BNO_set_opmode(&bno, bno.mode);
    while (1) {
        //bno.err = BNO_temperature(&bno, &temperature);
        //
        //if (bno.err != BNO_OK) {
        //    USART_printf(port, "[BNO] error reading temperature\n");
        //}
        //I2C_write(i2c1, BNO_ADDR, BNO_OPR_MODE, (1<<3));
        //



        i2c_err = I2C_read(i2c1, BNO_ADDR, BNO_OPR_MODE, &addr_data);
        if ( i2c_err != I2C_OK) {
            USART_printf(port, "[I2C] error: %s\n", I2C_get_err_str(i2c_err));
        }
        bno_err = bno055_euler_roll(&bno, &roll);
        if (bno_err != BNO_OK) {
            USART_printf(port, "[BNO] error: %s\n", bno055_err_str(bno_err));
        }
        bno_err = bno055_temperature(&bno, &temperature);
        if (bno_err != BNO_OK) {
            USART_printf(port, "[BNO] error: %s\n", bno055_err_str(bno_err));
        }
        if ((bno_err = bno055_gyro_x(&bno, (i16*)&gyro.x)) != BNO_OK) {
            USART_printf(port, "[BNO] error: %s\n", bno055_err_str(bno_err));
        }
        bno055_gyro_y(&bno, (i16*)&gyro.y); 
        bno055_gyro_z(&bno, (i16*)&gyro.z); 

        str str_test = "This is a string test";
        //USART_printf(port, "time: %02ldh%02dm%02ds -> temperature: %02d*C -> roll:%2.1f -> Read Data: %d\r", hour, min, cycle++, temperature, (float)roll/16.0, addr_data);
        USART_printf(port, "Gyro -> X: %6.2f  Y: %6.2f  Z: %6.2f    %s\r", gyro.x, gyro.y, gyro.z, str_test);
        if (cycle == 60) {
            min++;
            cycle=0;
        }
        if (min == 60) {
            hour++;
            min = 0;
        }
        delayMs(1000);
    }
}



