#include <stm32f4xx.h>
#include "mpu.h"
#include "../../stm32/f4/i2c/i2c.h"
#include "../../stm32/f4/delay/delay.h"
#include "../../stm32/f4/uart/uart.h"
#include "../../stm32/f4/gpio/gpio.h"

volatile int32_t mpu_gyro_calib[3];
volatile int32_t mpu_accel_calib[3];

mpu_err_t MPU_init(mpu_t *mpu) {
    if (I2C_init(&mpu->port) != I2C_OK) {
        return MPU_ERR_I2C_FAILED;
    }
    i2c_err_t i2c_err;
    mpu_err_t mpu_err;
    i2c_err = I2C_write(mpu->port, (mpu->alt_addr ? MPU_ADDR_ALT : MPU_ADDR), PWR_MGMT_1, 0x00);
    if (i2c_err != I2C_OK) {
        _I2C_send_stop(mpu->port);
        return MPU_ERR_I2C_FAILED;
    }
    if ((mpu_err = MPU_set_gyro_range(mpu)) != MPU_OK) {
        return mpu_err;
    }
    if ((mpu_err = MPU_set_accel_range(mpu)) != MPU_OK) {
        return mpu_err;
    }
    delayMs(4);
    return MPU_OK;
}

mpu_err_t MPU_set_gyro_range(mpu_t *mpu) {
    uint8_t data = (mpu->gyro_range << GYRO_CONF_OFFSET);
    i2c_err_t err;
    err = I2C_write(mpu->port,(mpu->alt_addr ? MPU_ADDR_ALT : MPU_ADDR), GYRO_CONFIG, data); 
    if (err != I2C_OK) {
        return MPU_ERR_I2C_FAILED;
    }
    delayMs(2);
    return MPU_OK;
}

mpu_err_t MPU_set_accel_range(mpu_t *mpu) {
    uint8_t data = (mpu->accel_range << ACCEL_CONF_OFFSET);
    i2c_err_t err;
    err = I2C_write(mpu->port,(mpu->alt_addr ? MPU_ADDR_ALT : MPU_ADDR), ACCEL_CONFIG, data); 
    if (err != I2C_OK) {
        return MPU_ERR_I2C_FAILED;
    }
    delayMs(2);
    return MPU_OK;
}


mpu_err_t MPU_gyro_x_raw(mpu_t mpu, int32_t *data) {
    uint8_t out[2] = {0, 0}; 
    i2c_err_t err;
    err = I2C_read_burst(mpu.port, (mpu.alt_addr ? MPU_ADDR_ALT : MPU_ADDR), GYRO_XOUT_H, 2, out);
    if (err != I2C_OK) {
        return MPU_ERR_I2C_FAILED;
    }
    *data = (int16_t)(((out[0] << 8) | out[1]));
    //if (mpu._calibrated) {
    //    *data -= mpu_gyro_calib[0];
    //}
    delayMs(2);
    return MPU_OK; 
}


mpu_err_t MPU_gyro_y_raw(mpu_t mpu, int32_t *data) {
    uint8_t out[2] = {0, 0}; 
    i2c_err_t err;
    err = I2C_read_burst(mpu.port, (mpu.alt_addr ? MPU_ADDR_ALT : MPU_ADDR), GYRO_YOUT_H, 2, out);
    if (err != I2C_OK) {
        return MPU_ERR_I2C_FAILED;
    }
    *data = (int16_t)((out[0] << 8) | out[1]);
    delayMs(2);
    return MPU_OK; 
}


mpu_err_t MPU_gyro_z_raw(mpu_t mpu, int32_t *data) {
    uint8_t out[2] = {0, 0}; 
    i2c_err_t err;
    err = I2C_read_burst(mpu.port, (mpu.alt_addr ? MPU_ADDR_ALT : MPU_ADDR), GYRO_ZOUT_H, 2, out);
    if (err != I2C_OK) {
        return MPU_ERR_I2C_FAILED;
    }
    *data = (int16_t)((out[0] << 8) | out[1]);
    delayMs(2);
    return MPU_OK; 
}

mpu_err_t MPU_gyro_raw(mpu_t mpu, int32_t *data) {
    mpu_err_t err;
    err = MPU_gyro_x_raw(mpu, &(data[0])); 
    if (err != MPU_OK) {
        return err;
    }
    err = MPU_gyro_y_raw(mpu, &data[1]); 
    if (err != MPU_OK) {
        return err;
    }
    err = MPU_gyro_z_raw(mpu, &data[2]); 
    if (err != MPU_OK) {
        return err;
    }
    delayMs(2);
    return MPU_OK;
}



mpu_err_t MPU_accel_x_raw(mpu_t mpu, int32_t *data) {
    uint8_t out[2] = {0, 0}; 
    i2c_err_t err;
    
    err = I2C_read_burst(mpu.port, (mpu.alt_addr ? MPU_ADDR_ALT : MPU_ADDR), ACCEL_XOUT_H, 2, out);
    if (err != I2C_OK) {
        return MPU_ERR_I2C_FAILED;
    }
    *data = (int32_t)((out[0] << 8) | out[1]);
    delayMs(2);
    return MPU_OK;
}


mpu_err_t MPU_accel_y_raw(mpu_t mpu, int32_t *data) {
    uint8_t out[2] = {0, 0}; 
    i2c_err_t err;
    err = I2C_read_burst(mpu.port, (mpu.alt_addr ? MPU_ADDR_ALT : MPU_ADDR), ACCEL_YOUT_H, 2, out);
    if (err != I2C_OK) {
        return MPU_ERR_I2C_FAILED;
    }
    *data = (int16_t)((out[0] << 8) | out[1]);
    delayMs(2);
    return MPU_OK;
}


mpu_err_t MPU_accel_z_raw(mpu_t mpu, int32_t *data) {
    uint8_t out[2] = {0, 0}; 
    i2c_err_t err;
    err = I2C_read_burst(mpu.port, (mpu.alt_addr ? MPU_ADDR_ALT : MPU_ADDR), ACCEL_ZOUT_H, 2, out);
    if (err != I2C_OK) {
        return MPU_ERR_I2C_FAILED;
    }
    *data = (int16_t)((out[0] << 8) | out[1]);
    delayMs(2);
    return MPU_OK;
}

mpu_err_t MPU_accel_raw(mpu_t mpu, int32_t *data) {
    mpu_err_t err;
    err = MPU_accel_x_raw(mpu, &data[0]); 
    if (err != MPU_OK) {
        return err;
    }
    err = MPU_accel_y_raw(mpu, &data[1]); 
    if (err != MPU_OK) {
        return err;
    }
    err = MPU_accel_z_raw(mpu, &data[2]); 
    if (err != MPU_OK) {
        return err;
    }
    delayMs(2);
    return MPU_OK;
}



mpu_err_t MPU_calibrate(mpu_t *mpu) {
    if (mpu->_calibrated) {
        return MPU_ERR_ALREADY_CALIBRATED;
    }
    volatile uint32_t errors = 0;
    int32_t gyro[3];
    int32_t accel[3];
    volatile mpu_err_t err;
    for (volatile uint32_t i = 0; i < MPU_NUM_CALIBRATIONS; i++) {
        //int32_t tmp = 0;
        //if ((err = MPU_gyro_x_raw(mpu, &tmp)) != MPU_OK) {
        //    errors++; 
        //}
        //gyro[0] += tmp;
        //if ((err = MPU_gyro_y_raw(mpu, &tmp)) != MPU_OK) {
        //    errors++;
        //}
        //gyro[1] += tmp;
        //if ((err = MPU_gyro_z_raw(mpu, &tmp)) != MPU_OK) {
        //    errors++;
        //}
        //gyro[2] += tmp;
        //if ((err = MPU_accel_x_raw(mpu, &tmp)) != MPU_OK) {
        //    errors++;
        //}
        //accel[0] += tmp;
        //if ((err = MPU_accel_y_raw(mpu, &tmp)) != MPU_OK) {
        //    errors++;
        //}
        //accel[1] += tmp;
        //if ((err = MPU_accel_z_raw(mpu, &tmp)) != MPU_OK) {
        //    errors++;
        //}
        //accel[2] += tmp;
        if ((err = MPU_gyro_raw(*mpu, gyro)) != MPU_OK) {
            errors++;
        }
        if ((err = MPU_accel_raw(*mpu, accel)) != MPU_OK) {
            errors++;
        }
        for (int j = 0; j < 3; j++) {
            mpu_gyro_calib[j] += gyro[j];
            mpu_accel_calib[j] += accel[j];
        }
        delayMs(20);
        if (errors > MPU_MAX_ERRORS) {
            return MPU_ERR_I2C_FAILED;
        }
    }
    for (volatile uint32_t i = 0; i < 3; i++) {
        mpu_gyro_calib[i] /= MPU_NUM_CALIBRATIONS;
        mpu_accel_calib[i] /= MPU_NUM_CALIBRATIONS;
    }
    mpu->_calibrated = true;
    return MPU_OK;
}

float MPU_get_gyro_range(mpu_t mpu) {
    switch (mpu.gyro_range) {
        case GYRO_500_DEG_S:
            return 65.5;
        case GYRO_1000_DEG_S:
            return 32.8;
        case GYRO_2000_DEG_S:
            return 16.4;
        default:
            return 131;
    }
}


int32_t  MPU_get_accel_range(mpu_t mpu) {
    switch (mpu.accel_range) {
        case ACCEL_4G:
            return 8192;
        case ACCEL_8G:
            return 4096;
        case ACCEL_16G:
            return 2048;
        default:
            return 16384;
    }
}


mpu_err_t MPU_gyro_x(mpu_t mpu, float *data) {
    int32_t raw;
    mpu_err_t err;
    err = MPU_gyro_x_raw(mpu, &raw); 
    if (err != MPU_OK) {
        return err;
    }
    if (mpu._calibrated) {
        raw -= mpu_gyro_calib[0];
    }
    *data = (raw / MPU_get_gyro_range(mpu));
    return MPU_OK;
}

mpu_err_t MPU_gyro_y(mpu_t mpu, float *data) {
    int32_t raw;
    mpu_err_t err;
    err = MPU_gyro_y_raw(mpu, &raw); 
    if (err != MPU_OK) {
        return err;
    }
    if (mpu._calibrated) {
        raw -= mpu_gyro_calib[1];
    }
    *data = raw / MPU_get_gyro_range(mpu);
    return MPU_OK;
}

mpu_err_t MPU_gyro_z(mpu_t mpu, float *data) {
    int32_t raw;
    mpu_err_t err;
    err = MPU_gyro_z_raw(mpu, &raw); 
    if (err != MPU_OK) {
        return err;
    }
    if (mpu._calibrated) {
        raw -= mpu_gyro_calib[2];
    }
    *data = raw / MPU_get_gyro_range(mpu);
    return MPU_OK;
}

mpu_err_t MPU_gyro(mpu_t mpu, float *data, int32_t n) {
    if (n != 3) {
        return MPU_ERR_ARR_SIZE;
    }
    mpu_err_t err;
    err = MPU_gyro_x(mpu, &data[0]);
    if (err != MPU_OK) {
        return err;
    }
    err = MPU_gyro_y(mpu, &data[1]);
    if (err != MPU_OK) {
        return err;
    }
    err = MPU_gyro_z(mpu, &data[2]);
    if (err != MPU_OK) {
        return err;
    }
    return MPU_OK;
}

mpu_err_t MPU_accel_x(mpu_t mpu, float *data) {
    int32_t raw;
    mpu_err_t err;
    err = MPU_accel_x_raw(mpu, &raw); 
    if (err != MPU_OK) {
        return err;
    }
    if (mpu._calibrated) {
        raw -= mpu_accel_calib[0];
    }
    *data = raw / (float) MPU_get_accel_range(mpu);
    return MPU_OK;
}

mpu_err_t MPU_accel_y(mpu_t mpu, float *data) {
    int32_t raw;
    mpu_err_t err;
    err = MPU_accel_y_raw(mpu, &raw); 
    if (err != MPU_OK) {
        return err;
    }
    if (mpu._calibrated) {
        raw -= mpu_accel_calib[0];
    }
    *data = raw / (float) MPU_get_accel_range(mpu);
    return MPU_OK;
}

mpu_err_t MPU_accel_z(mpu_t mpu, float *data) {
    int32_t raw;
    mpu_err_t err;
    err = MPU_accel_z_raw(mpu, &raw); 
    if (err != MPU_OK) {
        return err;
    }
    if (mpu._calibrated) {
        raw -= mpu_accel_calib[0];
    }
    *data = raw / (float) MPU_get_accel_range(mpu);
    return MPU_OK;
}

mpu_err_t MPU_accel(mpu_t mpu, float *data, int32_t n) {
    if (n != 3) {
        return MPU_ERR_ARR_SIZE;
    }
    mpu_err_t err;
    err = MPU_accel_x(mpu, &data[0]);
    if (err != MPU_OK) {
        return err;
    }
    err = MPU_accel_y(mpu, &data[1]);
    if (err != MPU_OK) {
        return err;
    }
    err = MPU_accel_z(mpu, &data[2]);
    if (err != MPU_OK) {
        return err;
    }
    return MPU_OK;
}

char *MPU_err_str(mpu_err_t err) {
    switch (err) {
        case MPU_ERR_I2C_FAILED:
            return "[-] I2C bus error";
        case MPU_ERR_ARR_SIZE:
            return "[-] Wrong input array size";
        default:
            return "[+] MPU Ok!";
    }
}


mpu_err_t MPU_set_offsets(mpu_t *mpu, int32_t gx, int32_t gy, int32_t gz, int32_t ax, int32_t ay, int32_t az) {
    

    return MPU_OK;
}
