#include "bno.h"
#include "../stm32/f4/i2c/i2c.h"
#include "../stm32/f4/delay/delay.h"
#include "../stm32/f4/gpio/gpio.h"
#include "../cosmic.h"


bool bno055_init(bno055 *bno) {
    u8 id;
    i2c_err_t err;
    bno_err_t err_bno;
    err = I2C_read(bno->i2c, BNO_ADDR, BNO_CHIP_ID, &id);
    if (err != I2C_OK) {
        _I2C_send_stop(bno->i2c);
        return false;
    }
    if (id != BNO_DEF_CHIP_ID) {
        _I2C_send_stop(bno->i2c);
        return false;
    }
    // set operation mode to config mode
    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return false;
    }
    delayMs(2);
    //BNO_reset(bno);
    //delayMs(100);
    if (bno055_set_pwr_mode(bno, BNO_PWR_NORMAL) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return false;
    }
    delayMs(10);

    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return false;
    }
    delayMs(BNO_CONFIG_TIME_DELAY+5);

    //BNO_on(bno);
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        GPIO_write(PA8, GPIO_ON); 
        return false;
    }
    delayMs(BNO_ANY_TIME_DELAY+5); 
    return true;
}


str bno055_err_str(const bno_err_t err) {
    switch (err) {
        case BNO_ERR_I2C:
            return "I2C error";
        case BNO_ERR_PAGE_TOO_HIGH:
            return "BNO wrong page -> must be 0 or 1";
        default:
            return "BNO ok";
    }
}


bno_err_t bno055_set_pwr_mode(bno055 *bno, const bno_pwr_t pwr) {

    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }

if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        return BNO_ERR_I2C;
    }
    if (I2C_write(bno->i2c, BNO_ADDR, BNO_PWR_MODE, pwr) != I2C_OK) {
        return BNO_ERR_I2C;
    }
    bno->_pwr_mode = pwr;
    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        return BNO_ERR_SETTING_PAGE;
    }

    // Set Operation mode to selected mode
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }


    delayMs(2);
    return BNO_OK;
}

bno_err_t bno055_set_page(bno055 *bno, const bno_page_t page) {
    if (page > 0x01) {
        return BNO_ERR_PAGE_TOO_HIGH;
    }
    if (I2C_write(bno->i2c, BNO_ADDR, BNO_PAGE_ID, page) != I2C_OK) {
        return BNO_ERR_I2C;
    }
    bno->_page = page;
    //BNO_set_page(bno, BNO_PAGE_0);
    delayMs(2);
    return BNO_OK;
}

bno_err_t bno055_set_opmode(bno055 *bno, const bno_opmode_t mode) {
    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        return BNO_ERR_I2C;
    }
    if (I2C_write(bno->i2c, BNO_ADDR, BNO_OPR_MODE, mode) != I2C_OK) {
        return BNO_ERR_I2C;
    }
    delayMs(BNO_ANY_TIME_DELAY+5);
    return BNO_OK;
}

bno_err_t bno055_set_acc_conf(bno055 *bno,
                           const bno_acc_range_t range,
                           const bno_acc_band_t bandwidth,
                           const bno_acc_mode_t mode) {
    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
    if (bno055_set_page(bno, BNO_PAGE_1) != BNO_OK) {
        return BNO_ERR_I2C;
    }
    if (I2C_write(bno->i2c, BNO_ADDR, BNO_ACC_CONFIG, range|bandwidth|mode) != I2C_OK) {
        return BNO_ERR_I2C;
    }
    bno->_acc_mode = mode;
    bno->_acc_bandwidth = bandwidth;
    bno->_acc_range = range;
    bno055_set_page(bno, BNO_PAGE_0);
    
    // Set Operation mode to selected mode
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
    delayMs(2);
    return BNO_OK;
}

bno_err_t bno055_set_mag_conf(bno055 *bno,
                           const bno_mag_rate_t out_rate,
                           const bno_mag_pwr_t pwr_mode,
                           const bno_mag_mode_t mode) {
    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
    if (bno055_set_page(bno, BNO_PAGE_1) != BNO_OK) {
        return BNO_ERR_I2C;
    } if (I2C_write(bno->i2c, BNO_ADDR, BNO_MAG_CONFIG, out_rate|pwr_mode|mode) != I2C_OK) {
        return BNO_ERR_I2C;
    }
    bno->_mag_mode = mode;
    bno->_mag_out_rate = out_rate;
    bno->_mag_pwr_mode = pwr_mode;

    // Set Operation mode to selected mode
    bno055_set_page(bno, BNO_PAGE_0);
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        return BNO_ERR_SETTING_PAGE;
    }
    delayMs(2);
    return BNO_OK;
}

bno_err_t bno055_set_gyr_conf(bno055 *bno,
                           const bno_gyr_range_t range,
                           const bno_gyr_band_t bandwidth,
                           const bno_gyr_mode_t mode) {
    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
    if (bno055_set_page(bno, BNO_PAGE_1) != BNO_OK) {
        return BNO_ERR_SETTING_PAGE;
    }
    if (I2C_write(bno->i2c, BNO_ADDR, BNO_GYR_CONFIG_0, range|bandwidth) != I2C_OK) {
        return BNO_ERR_I2C;
    }
    if (I2C_write(bno->i2c, BNO_ADDR, BNO_GYR_CONFIG_1, mode) != I2C_OK) {
        return BNO_ERR_I2C;
    }
    bno->_gyr_mode = mode;
    bno->_gyr_bandwidth = bandwidth;
    bno->_gyr_range = range;
    bno055_set_page(bno, BNO_PAGE_0);
    
    // Set Operation mode to selected mode
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }

    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        return BNO_ERR_SETTING_PAGE;
    }
    delayMs(2);
    return BNO_OK;
}


/**
 * Set units for BNO sensor
 * @param bno BNO structure
 * @param [t_unit](BNO_TEMP_UNIT_C,BNO_TEMP_UNIT_F) Temperature unit
 * @param [g_unit](BNO_GYR_UNIT_DPS,BNO_GYR_UNIT_RPS) Gyroscope unit
 * @param [a_unit](BNO_ACC_UNITSEL_M_S2,BNO_ACC_UNITSEL_M_S2) Accelerometer unit
 * @param [e_unit](BNO_EUL_UNIT_DEG,BNO_EUL_UNIT_RAD) Euler angles unit
 *
 * @return bno_err_t error code
 */
bno_err_t bno055_set_unit(bno055 *bno,
                       const bno_temp_unitsel_t t_unit,
                       const bno_gyr_unitsel_t g_unit,
                       const bno_acc_unitsel_t a_unit,
                       const bno_eul_unitsel_t e_unit) {
    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        return BNO_ERR_I2C;
    }
    if (I2C_write(bno->i2c, BNO_ADDR, BNO_UNIT_SEL, t_unit|g_unit|a_unit|e_unit) != I2C_OK) {
        return BNO_ERR_I2C;
    }
    bno->_gyr_unit = g_unit;
    bno->_acc_unit = a_unit;
    bno->_eul_unit = e_unit;
    bno->_temp_unit = t_unit;

    // Set Operation mode to selected mode
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
    delayMs(2);
    return BNO_OK;
}

bno_err_t bno055_set_temp_src(bno055 *bno, const enum bno_temp_src src) {
    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        return BNO_ERR_I2C;
    }
    if (I2C_write(bno->i2c, BNO_ADDR, BNO_TEMP_SOURCE, src) != I2C_OK) {
        return BNO_ERR_I2C;
    }

    // Set Operation mode to selected mode
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
    delayMs(2);
    return BNO_OK;
}

bno_err_t bno055_reset(bno055 *bno) {
    if (I2C_write(bno->i2c, BNO_ADDR, BNO_SYS_TRIGGER, (1<<5)) != I2C_OK) {
        return BNO_ERR_I2C;
    }
    return BNO_OK;
}

bno_err_t bno055_on(bno055 *bno) {
    if (I2C_write(bno->i2c, BNO_ADDR, BNO_SYS_TRIGGER, (0<<5)) != I2C_OK) {
        return BNO_ERR_I2C;
    }
    return BNO_OK;
}

bno_err_t bno055_temperature(bno055 *bno, i8 *temp) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
#endif
    bno_err_t err_bno;
    i2c_err_t err_i2c; 
    u8 data;
    if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
        return err_bno;
    }
    err_i2c = I2C_read(bno->i2c, BNO_ADDR, BNO_TEMP, &data);
    if (err_i2c != I2C_OK) {
        return BNO_ERR_I2C;
    }
    *temp = (bno->_temp_unit) ? data*2 : data;
    return BNO_OK;
}


bno_err_t bno055_euler_roll(bno055 *bno, i16 *roll) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
#endif
    bno_err_t err_bno;
    i2c_err_t err_i2c;
    u8 data[2];
    if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
        return err_bno;
    }
    //err_i2c = I2C_read(bno->i2c, BNO_ADDR, BNO_EUL_ROLL_LSB, &data[0]);
    //if (err_i2c != I2C_OK) {
    //    return BNO_ERR_I2C;
    //}
    //err_i2c = I2C_read(bno->i2c, BNO_ADDR, BNO_EUL_ROLL_MSB, &data[1]);
    err_i2c = I2C_read_burst(bno->i2c, BNO_ADDR, BNO_EUL_ROLL_LSB, 2, data);
    if (err_i2c != I2C_OK) {
        return BNO_ERR_I2C;
    }
    *data = (i16)(data[0] | (data[1] << 8)); 
    return BNO_OK;
}

bno_err_t bno055_euler_pitch(bno055 *bno, i16 *pitch) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
#endif
    bno_err_t err_bno;
    i2c_err_t err_i2c;
    u8 data[2];
    if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
        return err_bno;
    }
    //err_i2c = I2C_read(bno->i2c, BNO_ADDR, BNO_EUL_ROLL_LSB, &data[0]);
    //if (err_i2c != I2C_OK) {
    //    return BNO_ERR_I2C;
    //}
    //err_i2c = I2C_read(bno->i2c, BNO_ADDR, BNO_EUL_ROLL_MSB, &data[1]);
    err_i2c = I2C_read_burst(bno->i2c, BNO_ADDR, BNO_EUL_PITCH_LSB, 2, data);
    if (err_i2c != I2C_OK) {
        return BNO_ERR_I2C;
    }
    *data = (i16)(data[0] | (data[1] << 8)); 
    return BNO_OK;
}

bno_err_t bno055_euler_yaw(bno055 *bno, i16 *yaw) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
#endif
    bno_err_t err_bno;
    i2c_err_t err_i2c;
    u8 data[2];
    if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
        return err_bno;
    }
    //err_i2c = I2C_read(bno->i2c, BNO_ADDR, BNO_EUL_ROLL_LSB, &data[0]);
    //if (err_i2c != I2C_OK) {
    //    return BNO_ERR_I2C;
    //}
    //err_i2c = I2C_read(bno->i2c, BNO_ADDR, BNO_EUL_ROLL_MSB, &data[1]);
    err_i2c = I2C_read_burst(bno->i2c, BNO_ADDR, BNO_EUL_HEADING_LSB, 2, data);
    if (err_i2c != I2C_OK) {
        return BNO_ERR_I2C;
    }
    *data = (i16)(data[0] | (data[1] << 8)); 
    return BNO_OK;
}

bno_err_t bno055_gyro_x(bno055 *bno, i16 *_x) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
#endif
    bno_err_t err_bno;
    i2c_err_t err_i2c;
    u8 data[2];
    if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
        return err_bno;
    }
    err_i2c = I2C_read_burst(bno->i2c, BNO_ADDR, BNO_GYR_DATA_X_LSB, 2, data);
    if (err_i2c != I2C_OK) {
        return BNO_ERR_I2C;
    }
    *data = (i16) (data[0] | (data[1] << 8)); 
    return BNO_OK;
}

bno_err_t bno055_gyro_y(bno055 *bno, i16 *_y) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
#endif
    bno_err_t err_bno;
    i2c_err_t err_i2c;
    u8 data[2];
    if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
        return err_bno;
    }
    err_i2c = I2C_read_burst(bno->i2c, BNO_ADDR, BNO_GYR_DATA_Y_LSB, 2, data);
    if (err_i2c != I2C_OK) {
        return BNO_ERR_I2C;
    }
    *data = (i16) (data[0] | (data[1] << 8)); 
    return BNO_OK;
}

bno_err_t bno055_gyro_z(bno055 *bno, i16 *_z) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _I2C_send_stop(bno->i2c);
        return BNO_ERR_I2C;
    }
#endif
    bno_err_t err_bno;
    i2c_err_t err_i2c;
    u8 data[2];
    if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
        return err_bno;
    }
    err_i2c = I2C_read_burst(bno->i2c, BNO_ADDR, BNO_GYR_DATA_Z_LSB, 2, data);
    if (err_i2c != I2C_OK) {
        return BNO_ERR_I2C;
    }
    *data = (i16) (data[0] | (data[1] << 8)); 
    return BNO_OK;
}
