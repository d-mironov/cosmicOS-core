#include "twowire.h"
#include "../gpio/gpio.h"
#include <stm32f4xx.h>
#include <stdbool.h>


/**
 * I2C initialization:
 * Resets the peripheral and sets the new settings
 * 
 * @param port I2C port configuration struct
 *
 * @return twowire_err_t error code
 */
twowire_err_t I2C_init(I2C_port *port) {
    port->set_up = false;
    if (port->i2c == I2C1) {
        // enabling GPIO 6,7, selecting alternate function, setting speed
        GPIO_enable(PB6, GPIO_ALTERNATE);
        GPIO_enable(PB7, GPIO_ALTERNATE);
        GPIO_settings(PB6, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        GPIO_settings(PB7, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        GPIO_select_alternate(PB6, GPIO_AF04);
        GPIO_select_alternate(PB7, GPIO_AF04);  
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    } else if (port->i2c == I2C2) {
        GPIO_enable(PB10, GPIO_ALTERNATE);
        GPIO_enable(PB11, GPIO_ALTERNATE);
        GPIO_settings(PB10, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        GPIO_settings(PB11, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        GPIO_select_alternate(PB10, GPIO_AF04);
        GPIO_select_alternate(PB11, GPIO_AF04);  
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    } else if (port->i2c == I2C3) {
        GPIO_enable(PB8, GPIO_ALTERNATE);
        GPIO_enable(PC9, GPIO_ALTERNATE);
        GPIO_settings(PA8, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        GPIO_settings(PC9, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        GPIO_select_alternate(PA8, GPIO_AF04);
        GPIO_select_alternate(PC9, GPIO_AF04);  
        RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
    } else {
        return I2C_PORT_NOT_AVAILABLE;
    }

    if (port->mode == I2C_FAST_MODE && port->frequency < I2C_FREQ_MIN_FM ) {
        return I2C_FREQ_TOO_LOW;
    } else if (port->frequency < I2C_FREQ_MIN_SM) {
        return I2C_FREQ_TOO_LOW; 
    } else if (port->frequency > I2C_FREQ_MAX) {
        return I2C_FREQ_TOO_HIGH;
    }

    // Reset CRx, CCR and TRISE
    (port->i2c)->CR1 = 0x0000;
    (port->i2c)->CR2 = 0x0000;
    (port->i2c)->CCR = 0x0000;
    (port->i2c)->TRISE = 0x0002;

    (port->i2c)->CR1 |= I2C_CR1_SWRST;
    (port->i2c)->CR1 &= ~I2C_CR1_SWRST;

    (port->i2c)->CR2 |= port->frequency;
    (port->i2c)->CCR = (uint32_t) _I2C_ccr_calc(port);
    (port->i2c)->CCR |= port->duty << 14;
    (port->i2c)->CCR |= port->mode << 15;
    (port->i2c)->TRISE |= (uint32_t) _I2C_trise_calc(port); 

    (port->i2c)->CR1 |= I2C_CR1_PE;
    port->set_up = true;
    return I2C_OK;
}

/**
 * I2C CCR register calculation
 * @param port I2C init struct
 * @return ccr register value
 */
float _I2C_ccr_calc(I2C_port *port) {
    float t_high = 0;
    int freq = port->frequency * 1000000;
    if (port->mode == I2C_STD_MODE) {
        t_high = I2C_SM_SCL_RISE_MAX + I2C_SM_SCLH;
    } else if (port->mode == I2C_FAST_MODE) {
        t_high = I2C_SM_SCL_RISE_MAX + I2C_FM_SCLH;
    }

    float tmp = 1.0 / freq;
    if (port->mode == I2C_STD_MODE) {
        return t_high / tmp;
    } else if (port->mode == I2C_FAST_MODE) {
        if (port->duty == I2C_DUTY_2) {
            return t_high / tmp;
        } else if (port->duty == I2C_DUTY_16_9) {
            return t_high / (tmp * 9);
        } else {
            return 0.0;
        }
    }
    return 0;
}

/**
 * I2C TRISE register calculation
 * @param port I2C init struct
 * @return ccr register value
 */
float _I2C_trise_calc(I2C_port *port) {
    float freq = 1.0 / (port->frequency * 1000000.0);
    if (port->mode == I2C_STD_MODE) {
        return (I2C_SM_SCL_RISE_MAX / freq) + 1.0;
    } else if (port->mode == I2C_FAST_MODE) {
        return (I2C_FM_SCL_RISE_MAX / freq) + 1.0;
    } else {
        return 0.0;
    }
}


/**
 * I2C byte read
 * Read 1 byte from a slave device with `slave` address on Memory 
 * address `memaddr`
 *
 * @param port - I2C object
 * @param slave - 7/10-bit slave address of slave
 * @param memaddr - Memory address to read from
 *
 * @return byte read from the I2C device (uint8_t)
 */
uint8_t I2C_read(I2C_port *port, uint8_t slave, uint8_t memaddr) {
    volatile int tmp;
    uint8_t out;
    while(I2C1->SR2 & I2C_SR2_BUSY);     
    // Generate start condition
    (port->i2c)->CR1 |= I2C_CR1_ACK;
    (port->i2c)->CR1 |= I2C_CR1_START;
    // wait for SB=1
    while(!((port->i2c)->SR1 & I2C_SR1_SB));
    // put address in the data register
    (port->i2c)->DR = slave << 1;
    // set ACK high
    //(port->i2c)->CR1 |= I2C_CR1_ACK;
    // wait for ADDR=1
    while(!((port->i2c)->SR1 & I2C_SR1_ADDR));
    tmp = (port->i2c)->SR2;
    //send memory address
    (port->i2c)->DR = memaddr;
    // check TXE flag
    while(!((port->i2c)->SR1 & I2C_SR1_TXE));
    //USART_printf(USART2, "Fail 3\n");
    // generate restart condition
    (port->i2c)->CR1 |= I2C_CR1_START;
    // wait for SB=1
    while(!((port->i2c)->SR1 & I2C_SR1_SB));
    // put address in the data register (read)
    (port->i2c)->DR = (slave << 1) | 1;
    // wait for ADDR=1
    while(!((port->i2c)->SR1 & I2C_SR1_ADDR));
    //disable ACK
    (port->i2c)->CR1 &= ~I2C_CR1_ACK;
    tmp = (port->i2c)->SR2;
    //generate stop
    (port->i2c)->CR1 |= I2C_CR1_STOP;
    // check for RXNE flag
    while(!((port->i2c)->SR1 & I2C_SR1_RXNE));
    out = (port->i2c)->DR;
    return out;
}

/**
 * I2C Burst read
 * Read `n` number of bytes from slave starting
 * on memory address `memaddr` and store them in 
 * and array
 *
 * @param port - I2C struct
 * @param slave - Slave address
 * @param memaddr - start address of burst read
 * @param n - number of bytes to read
 *
 * @return error-code - error code
 */
twowire_err_t I2C_read_burst(I2C_port *port, uint8_t slave, uint8_t memaddr, uint8_t n, uint8_t *data) {
    volatile int tmp;
    while((port->i2c)->SR2 & I2C_SR2_BUSY);     
    // Generate start condition
    (port->i2c)->CR1 |= I2C_CR1_START;
    // wait for SB=1
    while(!((port->i2c)->SR1 & I2C_SR1_SB));
    // put address in the data register
    (port->i2c)->DR = slave << 1;
    // set ACK high
    //(port->i2c)->CR1 |= I2C_CR1_ACK;
    // wait for ADDR=1
    while(!((port->i2c)->SR1 & I2C_SR1_ADDR));
    tmp = (port->i2c)->SR1;
    while(!((port->i2c)->SR1 & I2C_SR1_TXE));
    //send memory address
    (port->i2c)->DR = memaddr;
    // check TXE flag
    while(!((port->i2c)->SR1 & I2C_SR1_TXE));
    // generate restart condition
    (port->i2c)->CR1 |= I2C_CR1_START;
    // wait for SB=1
    while(!((port->i2c)->SR1 & I2C_SR1_SB));
    // put address in the data register (read)
    (port->i2c)->DR = (slave << 1) | 1;
    // wait for ADDR=1
    while(!((port->i2c)->SR1 & I2C_SR1_ADDR));
    tmp = (port->i2c)->SR1;
    //disable ACK
    (port->i2c)->CR1 |= I2C_CR1_ACK;

    for (int i = 0; i < n; i++) {
        if (i == n-1) {
            (port->i2c)->CR1 &= ~I2C_CR1_ACK;
            (port->i2c)->CR1 |= I2C_CR1_STOP;
            while(!((port->i2c)->SR1 & I2C_SR1_RXNE));
            data[i] = (port->i2c)->DR;
            break;
        } else {
            while(!((port->i2c)->SR1 & I2C_SR1_RXNE));
            data[i] = (port->i2c)->DR;
        }
    }
    //while (n > 0U) {
    //    // if one byte
    //    if (n == 1) {
    //        (port->i2c)->CR1 &= ~I2C_CR1_ACK;
    //        (port->i2c)->CR1 |= I2C_CR1_STOP;
    //        while(!((port->i2c)->SR1 & I2C_SR1_RXNE));
    //        *data++ = (port->i2c)->DR;
    //        break;
    //    } else {
    //        while(!((port->i2c)->SR1 & I2C_SR1_RXNE));
    //        *data++ = (port->i2c)->DR;
    //        n--; 
    //    }
    //}
    // check for RXNE flag
    while(!((port->i2c)->SR1 & I2C_SR1_RXNE));


    return I2C_OK;
}



twowire_err_t I2C_write_burst(I2C_port *port, uint8_t slave, uint8_t memaddr, uint8_t n, uint8_t *data) {
    volatile int tmp;
    while((port->i2c)->SR2 & I2C_SR2_BUSY);     
    // Generate start condition
    (port->i2c)->CR1 |= I2C_CR1_START;
    // wait for SB=1
    while(!((port->i2c)->SR1 & I2C_SR1_SB));
    // put address in the data register
    (port->i2c)->DR = slave << 1;
    // set ACK high
    //(port->i2c)->CR1 |= I2C_CR1_ACK;
    // wait for ADDR=1
    while(!((port->i2c)->SR1 & I2C_SR1_ADDR));
    tmp = (port->i2c)->SR2;
    // check TXE flag
    while(!((port->i2c)->SR1 & I2C_SR1_TXE));
    //send memory address
    (port->i2c)->DR = memaddr;
    for (int i = 0; i < n; i++) { 
        while(!((port->i2c)->SR1 & I2C_SR1_TXE));
        (port->i2c)->DR = data[i];
    }
    while(!((port->i2c)->SR1 & I2C_SR1_BTF));
    (port->i2c)->CR1 |= I2C_CR1_STOP; 
    
    return I2C_OK;
}
