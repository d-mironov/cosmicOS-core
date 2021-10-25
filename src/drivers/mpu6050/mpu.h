#ifndef _MPU_H
#define _MPU_H

#include <stm32f4xx.h>
#include "../../stm32/f4/i2c/i2c.h"
#include <stdbool.h>

#define MPU_ADDR        0x68
#define MPU_ADDR_ALT    0x69

#define SELF_TEST_X     0x00
#define SELF_TEST_Y     0x0E
#define SELF_TEST_Z     0x0F
#define SELF_TEST_A     0x10
#define SMPLRT_DIV      0x19
#define CONFIG          0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define FIFO_EN         0x23
#define I2C_MST_CTRL    0x24
#define I2C_SLV0_ADDR   0x25
#define I2C_SLV0_REG    0x26
#define I2C_SLV0_CTRL   0x27
#define I2C_SLV1_ADDR   0x28
#define I2C_SLV1_REG    0x29
#define I2C_SLV1_CTRL   0x2A
#define I2C_SLV2_ADDR   0x2B
#define I2C_SLV2_REG    0x2C
#define I2C_SLV2_CTRL   0x2D
#define I2C_SLV3_ADDR   0x2E
#define I2C_SLV3_REG    0x2F
#define I2C_SLV3_CTRL   0x30
#define I2C_SLV4_ADDR   0x31
#define I2C_SLV4_REG    0x32
#define I2C_SLV4_DO     0x33
#define I2C_SLV4_CTRL   0x34
#define I2C_SLV4_DI     0x35
#define I2C_MST_STATUS  0x36
#define INT_PIN_CFG     0x37
#define INT_ENABLE      0x38
#define INT_STATUS      0x3A

#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48

#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60

#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define USER_CTRL           0x6A
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I            0x75


#define GYRO_CONF_OFFSET    0x03
#define ACCEL_CONF_OFFSET   0x03

#define MPU_NUM_CALIBRATIONS    (1000)
#define MPU_MAX_ERRORS          (0x0A)


/**
 * Gyroscope range in deg/s
 */
typedef enum _mpu_gyro_range_t {
    GYRO_250_DEG_S,
    GYRO_500_DEG_S,
    GYRO_1000_DEG_S,
    GYRO_2000_DEG_S,
} mpu_gyro_range_t;

/**
 * Accelerometer range in g
 */
typedef enum _mpu_accel_range_t {
    ACCEL_2G,
    ACCEL_4G,
    ACCEL_8G,
    ACCEL_16G,
} mpu_accel_range_t;

typedef enum _mpu_err_t {
    MPU_OK,
    MPU_ERR_I2C_FAILED,
    MPU_ERR_ARR_SIZE,
    MPU_ERR_ALREADY_CALIBRATED,
} mpu_err_t;


/**
 * MPU sensor structure
 */
typedef struct _mpu_t {
    I2C_port port;
    bool alt_addr;
    mpu_gyro_range_t gyro_range; 
    mpu_accel_range_t accel_range;
    bool _calibrated;
} mpu_t;

mpu_err_t MPU_init(mpu_t *mpu);
mpu_err_t MPU_set_gyro_range(mpu_t *mpu);
mpu_err_t MPU_set_accel_range(mpu_t *mpu);
mpu_err_t MPU_set_offsets(mpu_t *mpu, int32_t gx, int32_t gy, int32_t gz, int32_t ax, int32_t ay, int32_t az);
mpu_err_t MPU_calibrate(mpu_t *mpu);
float     MPU_get_gyro_range(mpu_t mpu);
int32_t  MPU_get_accel_range(mpu_t mpu);

// RAW Gyroscope measurements
mpu_err_t MPU_gyro_x_raw(mpu_t mpu, int32_t *data);
mpu_err_t MPU_gyro_y_raw(mpu_t mpu, int32_t *data);
mpu_err_t MPU_gyro_z_raw(mpu_t mpu, int32_t *data);
mpu_err_t MPU_gyro_raw(mpu_t mpu, int32_t *data);

// RAW Accelerometer measurements
mpu_err_t MPU_accel_x_raw(mpu_t mpu, int32_t *data);
mpu_err_t MPU_accel_y_raw(mpu_t mpu, int32_t *data);
mpu_err_t MPU_accel_z_raw(mpu_t mpu, int32_t *data);
mpu_err_t MPU_accel_raw(mpu_t mpu, int32_t *data);

mpu_err_t MPU_gyro_x(mpu_t mpu, float *data);
mpu_err_t MPU_gyro_y(mpu_t mpu, float *data);
mpu_err_t MPU_gyro_z(mpu_t mpu, float *data);
mpu_err_t MPU_gyro(mpu_t mpu, float *data, int32_t n);

mpu_err_t MPU_accel_x(mpu_t mpu, float *data);
mpu_err_t MPU_accel_y(mpu_t mpu, float *data);
mpu_err_t MPU_accel_z(mpu_t mpu, float *data);
mpu_err_t MPU_accel(mpu_t mpu, float *data, int32_t n);
// 
char *MPU_err_str(mpu_err_t err);

#endif
