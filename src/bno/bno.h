#ifndef _BNO_H
#define _BNO_H

#include "../stm32/f4/i2c/i2c.h"
#include <stm32f4xx.h>
#include <stdbool.h>

#define BNO_ADDR                (0x29)
#define BNO_ADDR_ALT            (0x28)
#define BNO_DEF_CHIP_ID         (0xA0)

// Page 1 
#define BNO_MAG_RADIUS_MSB      (0x6A)
#define BNO_MAG_RADIUS_LSB      (0x69)
#define BNO_ACC_RADIUS_MSB      (0x68)
#define BNO_ACC_RADIUS_LSB      (0x67)

// Gyroscope offset data
#define BNO_GYR_OFFSET_Z_MSB    (0x66)
#define BNO_GYR_OFFSET_Z_LSB    (0x65)
#define BNO_GYR_OFFSET_Y_MSB    (0x64)
#define BNO_GYR_OFFSET_Y_LSB    (0x63)
#define BNO_GYR_OFFSET_X_MSB    (0x62)
#define BNO_GYR_OFFSET_X_LSB    (0x61)

// magnetometer offset data
#define BNO_MAG_OFFSET_Z_MSB    (0x60)
#define BNO_MAG_OFFSET_Z_LSB    (0x5F)
#define BNO_MAG_OFFSET_Y_MSB    (0x5E)
#define BNO_MAG_OFFSET_Y_LSB    (0x5D)
#define BNO_MAG_OFFSET_X_MSB    (0x5C)
#define BNO_MAG_OFFSET_X_LSB    (0x5B)

// Accelerometer offset data
#define BNO_ACC_OFFSET_Z_MSB    (0x5A)
#define BNO_ACC_OFFSET_Z_LSB    (0x59)
#define BNO_ACC_OFFSET_Y_MSB    (0x58)
#define BNO_ACC_OFFSET_Y_LSB    (0x57)
#define BNO_ACC_OFFSET_X_MSB    (0x56)
#define BNO_ACC_OFFSET_X_LSB    (0x55)

// Axis remap
#define BNO_AXIS_MAP_SIGN       (0x42)
#define BNO_AXIS_MAP_CONFIG     (0x41)


#define BNO_TEMP_SOURCE         (0x40)
#define BNO_SYS_TRIGGER         (0x3F)

// Power mode registers
#define BNO_PWR_MODE            (0x3E)
#define BNO_OPR_MODE            (0x3D)
#define BNO_UNIT_SEL            (0x3B)

// status registers
#define BNO_SYS_ERR             (0x3A)
#define BNO_SYS_STATUS          (0x39)
#define BNO_SYS_CLK_STATUS      (0x38)

#define BNO_INT_STA             (0x37)
#define BNO_ST_RESULT           (0x36)
#define BNO_CALIB_STAT          (0x35)
#define BNO_TEMP                (0x34)

// Gravity Data
#define BNO_GRV_DATA_Z_MSB      (0x33)
#define BNO_GRV_DATA_Z_LSB      (0x32)
#define BNO_GRV_DATA_Y_MSB      (0x31)
#define BNO_GRV_DATA_Y_LSB      (0x30)
#define BNO_GRV_DATA_X_MSB      (0x2F)
#define BNO_GRV_DATA_X_LSB      (0x2E)

// Linear acceleration data
#define BNO_LIA_DATA_Z_MSB      (0x2D)
#define BNO_LIA_DATA_Z_LSB      (0x2C)
#define BNO_LIA_DATA_Y_MSB      (0x2B)
#define BNO_LIA_DATA_Y_LSB      (0x2A)
#define BNO_LIA_DATA_X_MSB      (0x29)
#define BNO_LIA_DATA_X_LSB      (0x28)

// Quaternion data
#define BNO_QUA_DATA_Z_MSB      (0x27)
#define BNO_QUA_DATA_Z_LSB      (0x26)
#define BNO_QUA_DATA_Y_MSB      (0x25)
#define BNO_QUA_DATA_Y_LSB      (0x24)
#define BNO_QUA_DATA_X_MSB      (0x23)
#define BNO_QUA_DATA_X_LSB      (0x22)
#define BNO_QUA_DATA_W_MSB      (0x21)
#define BNO_QUA_DATA_W_LSB      (0x20)

// Euler Angle data
#define BNO_EUL_PITCH_MSB       (0x1F)
#define BNO_EUL_PITCH_LSB       (0x1E)
#define BNO_EUL_ROLL_MSB        (0x1D)
#define BNO_EUL_ROLL_LSB        (0x1C)
#define BNO_EUL_HEADING_MSB     (0x1B)
#define BNO_EUL_HEADING_LSB     (0x1A)

// Gyroscope data
#define BNO_GYR_DATA_Z_MSB      (0x19)
#define BNO_GYR_DATA_Z_LSB      (0x18)
#define BNO_GYR_DATA_Y_MSB      (0x17)
#define BNO_GYR_DATA_Y_LSB      (0x16)
#define BNO_GYR_DATA_X_MSB      (0x15)
#define BNO_GYR_DATA_X_LSB      (0x14)

// Magnetometer data
#define BNO_MAG_DATA_Z_MSB      (0x13)
#define BNO_MAG_DATA_Z_LSB      (0x12)
#define BNO_MAG_DATA_Y_MSB      (0x11)
#define BNO_MAG_DATA_Y_LSB      (0x10)
#define BNO_MAG_DATA_X_MSB      (0x0F)
#define BNO_MAG_DATA_X_LSB      (0x0E)

// Accelerometer data
#define BNO_ACC_DATA_Z_MSB      (0x0D)
#define BNO_ACC_DATA_Z_LSB      (0x0C)
#define BNO_ACC_DATA_Y_MSB      (0x0B)
#define BNO_ACC_DATA_Y_LSB      (0x0A)
#define BNO_ACC_DATA_X_MSB      (0x09)
#define BNO_ACC_DATA_X_LSB      (0x08)

// Config Page ID
#define BNO_PAGE_ID             (0x07)
#define BNO_BL_REV_ID           (0x06)
#define BNO_SW_REV_ID_MSB       (0x05)
#define BNO_SW_REV_ID_LSB       (0x04)
#define BNO_GYR_ID              (0x03)
#define BNO_MAG_ID              (0x02)
#define BNO_ACC_ID              (0x01)
// Sensor ID
#define BNO_CHIP_ID             (0x00)


// PAGE 1
#define BNO_ACC_CONFIG          (0x08)
#define BNO_MAG_CONFIG          (0x09)
#define BNO_GYR_CONFIG_0        (0x0A)
#define BNO_GYR_CONFIG_1        (0x0B)
#define BNO_ACC_SLEEP_CONFIG    (0x0C)
#define BNO_GYR_SLEEP_CONFIG    (0x0D)

#define BNO_CONFIG_TIME_DELAY   7//ms
#define BNO_ANY_TIME_DELAY      19//ms

// mag, acc, gyr, config offsets
#define BNO_ACC_BAND_OFFSET     (0x02)
#define BNO_ACC_MODE_OFFSET     (0x05)
#define BNO_GYR_BAND_OFFSET     (0x03)
#define BNO_GYR_MODE_OFFSET     (0x00)
#define BNO_MAG_MODE_OFFSET     (0x03)
#define BNO_MAG_PWRMODE_OFFSET  (0x05)
// unit selection
#define BNO_GYR_UNITSEL_OFFSET  (0x01)
#define BNO_EUL_UNITSEL_OFFSET  (0x02)
#define BNO_TEMP_UNITSEL_OFFSET (0x04)


typedef enum bno_page {
    BNO_PAGE_0,
    BNO_PAGE_1
} bno_page_t;


typedef enum bno_pwr {
    BNO_PWR_NORMAL,
    BNO_PWR_LOW,
    BNO_PWR_SUSPEND,
} bno_pwr_t;

typedef enum bno_acc_range {
    BNO_ACC_RANGE_2G,
    BNO_ACC_RANGE_4G,
    BNO_ACC_RANGE_8G,
    BNO_ACC_RANGE_16G,
} bno_acc_range_t;

typedef enum bno_acc_band {
    BNO_ACC_BAND_7_81  = (0 << BNO_ACC_BAND_OFFSET),
    BNO_ACC_BAND_15_63 = (1 << BNO_ACC_BAND_OFFSET),
    BNO_ACC_BAND_31_25 = (2 << BNO_ACC_BAND_OFFSET),
    BNO_ACC_BAND_62_5  = (3 << BNO_ACC_BAND_OFFSET),
    BNO_ACC_BAND_125   = (4 << BNO_ACC_BAND_OFFSET),
    BNO_ACC_BAND_250   = (5 << BNO_ACC_BAND_OFFSET),
    BNO_ACC_BAND_500   = (6 << BNO_ACC_BAND_OFFSET),
    BNO_ACC_BAND_1000  = (7 << BNO_ACC_BAND_OFFSET),
} bno_acc_band_t;

typedef enum bno_acc_mode {
    BNO_ACC_MODE_NORMAL        = (0 << BNO_ACC_MODE_OFFSET),
    BNO_ACC_MODE_SUSPEND       = (1 << BNO_ACC_MODE_OFFSET),
    BNO_ACC_MODE_LOW1          = (2 << BNO_ACC_MODE_OFFSET),
    BNO_ACC_MODE_STANDY        = (3 << BNO_ACC_MODE_OFFSET),
    BNO_ACC_MODE_LOW2          = (4 << BNO_ACC_MODE_OFFSET),
    BNO_ACC_MODE_DEEP_SUSPEND  = (5 << BNO_ACC_MODE_OFFSET),
} bno_acc_mode_t;

typedef enum bno_gyr_range {
    BNO_GYR_RANGE_2000_DPS,
    BNO_GYR_RANGE_1000_DPS,
    BNO_GYR_RANGE_500_DPS,
    BNO_GYR_RANGE_250_DPS,
    BNO_GYR_RANGE_125_DPS,
} bno_gyr_range_t;

typedef enum bno_gyr_band {
    BNO_GYR_BAND_523 = (0 << BNO_GYR_BAND_OFFSET),
    BNO_GYR_BAND_230 = (1 << BNO_GYR_BAND_OFFSET),
    BNO_GYR_BAND_116,
    BNO_GYR_BAND_47,
    BNO_GYR_BAND_23,
    BNO_GYR_BAND_12,
    BNO_GYR_BAND_64,
    BNO_GYR_BAND_32,
} bno_gyr_band_t;

typedef enum bno_gyr_mode {
    BNO_GYR_MODE_NORMAL,        /*!< Gyro normal mode */
    BNO_GYR_MODE_FPU,           /*!< Gyro fast powerup mode */
    BNO_GYR_MODE_DEEP_SUSPEND,  /*!< Gyro deep suspend mode*/
    BNO_GYR_MODE_SUSPEND,       /*!< Gyro suspend mode */
    BNO_GYR_MODE_APS,           /*!< Gyro advanced powersave mode*/
} bno_gyr_mode_t;

typedef enum bno_mag_rate {
    BNO_MAG_RATE_2HZ,
    BNO_MAG_RATE_6HZ,
    BNO_MAG_RATE_8HZ,
    BNO_MAG_RATE_10HZ,
    BNO_MAG_RATE_15HZ,
    BNO_MAG_RATE_20HZ,
    BNO_MAG_RATE_25HZ,
    BNO_MAG_RATE_30HZ,
} bno_mag_rate_t;

typedef enum bno_mag_mode {
    BNO_MAG_MODE_LOW_PWR = (0 << BNO_MAG_MODE_OFFSET),
    BNO_MAG_MODE_REGULAR = (1 << BNO_MAG_MODE_OFFSET),
    BNO_MAG_MODE_ENH_REG,
    BNO_MAG_MODE_HIGH_ACC,
} bno_mag_mode_t;

typedef enum bno_mag_pwr {
    BNO_MAG_PWRMODE_NORMAL = (0 << BNO_MAG_PWRMODE_OFFSET),
    BNO_MAG_PWRMODE_SLEEP  = (1 << BNO_MAG_PWRMODE_OFFSET),
    BNO_MAG_PWRMODE_SUSPEND,
    BNO_MAG_PWRMODE_FORCE
} bno_mag_pwr_t;

typedef enum bno_acc_unitsel {
    BNO_ACC_UNITSEL_M_S2,
    BNO_ACC_UNITSEL_MG,
} bno_acc_unitsel_t;

typedef enum bno_gyr_unitsel {
    BNO_GYR_UNIT_DPS = (0 << BNO_GYR_UNITSEL_OFFSET),
    BNO_GYR_UNIT_RPS = (1 << BNO_GYR_UNITSEL_OFFSET),
} bno_gyr_unitsel_t;

typedef enum bno_eul_unitsel {
    BNO_EUL_UNIT_DEG = (0 << BNO_EUL_UNITSEL_OFFSET),
    BNO_EUL_UNIT_RAD = (1 << BNO_EUL_UNITSEL_OFFSET),
} bno_eul_unitsel_t;

typedef enum bno_temp_unitsel {
    BNO_TEMP_UNIT_C = (0 << BNO_TEMP_UNITSEL_OFFSET),
    BNO_TEMP_UNIT_F = (1 << BNO_TEMP_UNITSEL_OFFSET),
} bno_temp_unitsel_t;

typedef enum bno_opmode {
    BNO_MODE_CONFIG,
    BNO_MODE_AO,
    BNO_MODE_MO,
    BNO_MODE_GO,
    BNO_MODE_AM,
    BNO_MODE_AG,
    BNO_MODE_MG,
    BNO_MODE_AMG,
    BNO_MODE_IMU,
    BNO_MODE_COMPASS,
    BNO_MODE_M4G,
    BNO_MODE_NDOF_FMC_OFF,
    BNO_MODE_NDOF,
} bno_opmode_t;

enum bno_temp_src {
    BNO_TEMP_SRC_ACC,
    BNO_TEMP_SRC_GYR,
};

/**
 * BNO055 error codes
 */
typedef enum bno_err {
    BNO_OK,                             /*!<No error*/
    BNO_ERR_I2C,                        /*!<Error on the I2C bus*/
    BNO_ERR_PAGE_TOO_HIGH,              /*!<Page set too high [page](`BNO_PAGE_0`,`BNO_PAGE_1`)*/
} bno_err_t;


/**
 * BNO055 object struct
 */
typedef struct _BNO {

    //======| PRIVATE |======
    bno_pwr_t           _pwr_mode;                /*!<BNO power mode */
    bno_page_t          _page;
    // unit selection
    bno_acc_unitsel_t   _acc_unit;        /*!<Accelerometer unit [m/s^2, mg](`BNO_ACC_UNITSEL_*`)*/
    bno_temp_unitsel_t  _temp_unit;      /*!<Temperature unit [C, F](`BNO_TEMP_UNITSEL_*`) */
    bno_gyr_unitsel_t   _gyr_unit;        /*!<Gyroscope unit select [dps, rps]*/
    bno_eul_unitsel_t   _eul_unit;        /*!<Euler angle unit select [deg, rad] */

    // accelerometer
    bno_acc_range_t     _acc_range;         /*!<Accelerometer range (`BNO_ACC_RANGE_*`)*/
    bno_acc_band_t      _acc_bandwidth;      /*!<Accelerometer bandwidth (`BNO_ACC_BAND_*`)*/
    bno_acc_mode_t      _acc_mode;           /*!<Accelerometer operation mode (`BNO_ACC_MODE_*`)*/
    // gyroscope
    bno_gyr_range_t     _gyr_range;         /*!<Gyroscope range (`BNO_GYR_RANGE_*`)*/
    bno_gyr_band_t      _gyr_bandwidth;      /*!<Gyroscope bandwidth (`BNO_GYR_BAND_*`)*/
    bno_gyr_mode_t      _gyr_mode;           /*!<Gyroscope mode select (`BNO_GYR_MODE_*`)*/
    // magnetometer
    bno_mag_rate_t      _mag_out_rate;       /*!<Magnetometer output rate (`BNO_MAG_RATE_*`)*/
    bno_mag_mode_t      _mag_mode;           /*!<Magnetometer mode select (`BNO_MAG_MODE_)*/
    bno_mag_pwr_t       _mag_pwr_mode;        /*!<Magnetometer power mode (`BNO_MAG_PWRMODE_*`)*/

    //======| PUBLIC |======
    bno_opmode_t        mode;                  /*!<BNO operation mode like `CONFIGMODE`,`IMU`, etc.*/
    I2C                 i2c;                            /*!<I2C port */
    bno_err_t           err;                      /*!<current error code*/

} BNO;

BNO _bno;


BNO         BNO055_new          ();
bool        BNO_init            (BNO *bno);
bno_err_t   BNO_reset           (BNO *bno);
bno_err_t   BNO_on              (BNO *bno);

// ======|   BNO sensor data |======

bno_err_t   BNO_temperature     (BNO *bno, i8 *temp);
bno_err_t   BNO_euler_roll      (BNO *bno, i16 *roll);



// setter
bno_err_t   BNO_set_page        (BNO *bno, const bno_page_t page);
bno_err_t   BNO_set_opmode      (BNO *bno, const bno_opmode_t mode);

bno_err_t   BNO_set_unit        (BNO *bno,
                                 const bno_temp_unitsel_t t_unit,
                                 const bno_gyr_unitsel_t g_unit,
                                 const bno_acc_unitsel_t a_unit,
                                 const bno_eul_unitsel_t e_unit);

//bno_err_t BNO_set_acc_range(const bno_t bno, const bno_acc_range_t range);
//bno_err_t BNO_set_acc_bandwidth(const bno_t bno, const bno_acc_band_t bandwidth);
//bno_err_t BNO_set_acc_mode(const bno_t bno, const bno_acc_mode_t mode);
bno_err_t   BNO_set_acc_conf    (BNO *bno,
                                 const bno_acc_range_t range,
                                 const bno_acc_band_t bandwidth,
                                 const bno_acc_mode_t mode);
//bno_err_t BNO_set_gyr_range(const bno_t bno, const bno_gyr_range_t range);
//bno_err_t BNO_set_gyr_bandwidth(const bno_t bno, const bno_gyr_band_t bandwidth);
//bno_err_t BNO_set_gyr_mode(const bno_t bno, const bno_gyr_mode_t mode);
bno_err_t   BNO_set_gyr_conf    (BNO *bno,
                                 const bno_gyr_range_t range,
                                 const bno_gyr_band_t bandwidth,
                                 const bno_gyr_mode_t mode);
//bno_err_t BNO_set_mag_out_rate(const bno_t bno, const bno_mag_rate_t rate);
//bno_err_t BNO_set_mag_pwr_mode(const bno_t bno, const bno_mag_pwr_t pwr_mode);
//bno_err_t BNO_set_mag_mode(const bno_t bno, const bno_mag_mode_t mode);
bno_err_t   BNO_set_mag_conf    (BNO *bno,
                                 const bno_mag_rate_t out_rate,
                                 const bno_mag_pwr_t pwr_mode,
                                 const bno_mag_mode_t mode);

bno_err_t   BNO_set_pwr_mode    (BNO *bno, const bno_pwr_t pwr);
bno_err_t   BNO_set_temp_src    (BNO *bno, const enum bno_temp_src src);


// getter
u8          BNO_get_chip_id     (const BNO bno);


string      BNO_err_str         (const bno_err_t err);



#endif
