#include <stm32f4xx.h>
#include "mpu.h"
#include "../../stm32/f4/twowire/twowire.h"
#include "../../stm32/f4/delay/delay.h"


mpu_err_t MPU_init(mpu_t *mpu) {
    if (I2C_init(mpu->port) != I2C_OK) {
        return MPU_I2C_FAILED;
    }
    uint8_t init[1] = {0x00};
    I2C_write_burst(mpu->port, (mpu->alt_addr ? MPU_ADDR_ALT : MPU_ADDR), PWR_MGMT_1, 1, init);
    MPU_set_gyro_range(mpu);
    MPU_set_accel_range(mpu);
    delayMs(4);
    return MPU_OK;
}

mpu_err_t MPU_set_gyro_range(mpu_t *mpu) {
    uint8_t data[1] = {(mpu->gyro_range << GYRO_CONF_OFFSET)};
    I2C_write_burst(mpu->port,(mpu->alt_addr ? MPU_ADDR_ALT : MPU_ADDR), GYRO_CONFIG, 1, data); 
    delayMs(2);
    return MPU_OK;
}

mpu_err_t MPU_set_accel_range(mpu_t *mpu) {
    uint8_t data[1] = {(mpu->accel_range << ACCEL_CONF_OFFSET)};
    I2C_write_burst(mpu->port,(mpu->alt_addr ? MPU_ADDR_ALT : MPU_ADDR), ACCEL_CONFIG, 1, data); 
    delayMs(2);
    return MPU_OK;
}


uint16_t MPU_gyro_x_raw(mpu_t *mpu) {
    uint8_t out[2] = {0, 0}; 
    I2C_read_burst(mpu->port, (mpu->alt_addr ? MPU_ADDR_ALT : MPU_ADDR), GYRO_XOUT_H, 2, out);
    return (out[0] << 8) | out[1];
}


uint16_t MPU_gyro_y_raw(mpu_t *mpu) {
    uint8_t out[2] = {0, 0}; 
    I2C_read_burst(mpu->port, (mpu->alt_addr ? MPU_ADDR_ALT : MPU_ADDR), GYRO_YOUT_H, 2, out);
    return (out[0] << 8) | out[1];
}


uint16_t MPU_gyro_z_raw(mpu_t *mpu) {
    uint8_t out[2] = {0, 0}; 
    I2C_read_burst(mpu->port, (mpu->alt_addr ? MPU_ADDR_ALT : MPU_ADDR), GYRO_ZOUT_H, 2, out);
    return (out[0] << 8) | out[1];
}
