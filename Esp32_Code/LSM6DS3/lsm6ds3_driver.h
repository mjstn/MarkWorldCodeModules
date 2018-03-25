//
//  LSM6DS3 Accelerometer/Gyro I2C based Device support
//
//
// This code put together to be used for Mark-Toys.com projects.  Use at your own risk.
// The code is considered to be a starting point and not a finished and polished codebase
//
// Environment Requirements:
// Requires the Mark-Toys.com i2c wrapper driver that uses the Esp32 SDK I2C calls
//

#ifndef LSM6DS3_DEFS_H_
#define LSM6DS3_DEFS_H_

// lsm6d3s.h suitable values should we use such a file   ---------------------
#define LSM6DS3_I2C_ADDRESS  	0x6A  		// LSM6DS3 7-bit address on I2C when SD0 is low. If 0 we use SPI
#define LSM6DS3_SEC_ADDRESS  	0x6B  		// LSM6DS3 7-bit address secondary I2C accelerometer
#define LSM6DS3_CHIP_ID_REG		0x0f        // Readback CHIP ID to verify device
#define LSM6DS3_CHIP_ID         0x69        // Proper value expected from readback of chip id register
#define LSM6DS3_TEMP_L          0x20        // Temperature sensor low byte
#define LSM6DS3_TEMP_H          0x21        // Temperature sensor high byte
#define LSM6DS3_GYRO_BASE		0x22		// First address for X,Y,Z Gyro  readings with 2 bytes signed int LSB first for each
#define LSM6DS3_ACCEL_BASE		0x28		// First address for X,Y,Z Accel readings with 2 bytes signed int LSB first for each
#define ACCEL_PER_BIT_IN_G      ((double)(0.000061035))	// Gs per bit for Accel when in 2G range
#define GYRO_DEG_PER_SEC_FULL   ((double)(245.0))         // Degrees per second for signed 16 bit or 32768 count
#define GYRO_DEG_PER_SEC_PER_BIT  (GYRO_DEG_PER_SEC_FULL/(double)(32768))

// Structure for holding accelerometer readings in integer form
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} accelLevel_t;

// Structure for holding accelerometer readings in double form (assume in units of G)
typedef struct {
    double x;
    double y;
    double z;
} accelVectorInGs_t;

// Structure for holding accelerometer gyro in double form
typedef struct {
    double x;
    double y;
    double z;
} gyroVector_t;

extern int lsm6ds3_i2c_init(int i2c_num, uint8_t i2cAddr);
extern int lsm6ds3_i2c_read_chipid(int i2c_num, uint8_t i2c_addr, uint8_t *data);
extern int lsm6ds3_i2c_powerdown(int i2c_num, uint8_t i2cAddr);
extern int lsm6ds3_i2c_read_accel(int i2c_num, uint8_t i2c_addr, uint8_t *data);
extern int lsm6ds3_i2c_read_gyro(int i2c_num, uint8_t i2c_addr, uint8_t *data);
extern int lsm6ds3_i2c_read_temperature(int i2c_num, uint8_t i2c_addr, float *tempC);
extern accelVectorInGs_t lsm6ds3_i2c_readAccelVectorInGs(int i2c_num, uint8_t i2c_addr);
extern gyroVector_t lsm6ds3_i2c_readGyroVectorInDegPerSec(int i2c_num, uint8_t i2c_addr);
extern double lsm6ds3_i2c_readForwardAccelInGs(int i2c_num, uint8_t i2c_addr);
extern double lsm6ds3_i2c_readForwardGyroDegPerSec(int i2c_num, uint8_t i2c_addr);

#endif // LSM6DS3_DEFS_H_
