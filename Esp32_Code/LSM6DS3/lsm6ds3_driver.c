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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

// defines specific to Esp32 data types or api calls
#include "esp_system.h"
#include "esp_attr.h"

// Includes to support specific hardware APIs
#include "include/i2c_driver.h"

// Include the driver specific defines
#include <lsm6ds3_driver.h>

//  These commands are sent to initialize LSM6DS3
DRAM_ATTR static const i2c_cmd_table_t lsm6ds3_init_cmds[]={
	{0x18, {0x38}, 1},		// LSM6DS3_REG_CTRL9_XL  Accel enable X,Y,Z (this is the reset default so is optional)
    {0x10, {0x43}, 1},		// LSM6DS3_REG_CTRL1_XL  Accel 100hz sampling (0x40), 2G range (0x00), 50hz anti alias filter (0x03)
	{0x13, {0x00}, 1},		// LSM6DS3_REG_CTRL4_C   Not sleeping, no Int enable, no temp data in fifo, I2C enabled
	{0x16, {0x50}, 1},		// LSM6DS3_REG_CTRL7_G   Gyro settings.  High perf mode, High pass filter 0.0324hz cutoff
    {0x19, {0x38}, 1},		// LSM6DS3_REG_CTRL10_C  Enable X,Y,Z Gyro (0x38)
	{0x11, {0x40}, 1},		// LSM6DS3_REG_CTRL2_G   Gyro  100hz sampling (0x40), 245 degPerSec (0x00)
    {0x12, {0x54}, 1},		// LSM6DS3_REG_CTRL3_C   BDU (block data update used to freeze readings on 2-byte access)
	                        //   Int ouputs Open Drain, IF_INC bit for Auto-inc reg addr on multi byte access, BLE 0 for LSB then MSB
    {0, {0}, 0xff},
};

//  These commands are sent to set the LSM6DS3 into a sleeping or standby mode for low power consumption
//  This is also of use to let the chip cool if temperature is being used and it is desired to get near ambient reading
DRAM_ATTR static const i2c_cmd_table_t lsm6ds3_powerdown_cmds[]={
    {0x10, {0x03}, 1},		// LSM6DS3_REG_CTRL1_XL  Power-down setting, 2G range (0x00), 50hz anti alias filter (0x03)
	{0x13, {0x40}, 1},		// LSM6DS3_REG_CTRL4_C   GYRO sleeping, no Int enable, no temp data in fifo, I2C enabled
    {0, {0}, 0xff},
};

/*
 * @brief 		Initialize the lsm6ds3 sensor on the SPI bus
 *
 * @param		i2c_num		I2C Bus interface number
 * @param		i2c_addr	7-bit I2C bus address
 *
 * @return		Returns 0 for ok or -1 for IO error
 *
 * @note        Caller is responsible for use of i2c_lock() and i2c_unlock() outside of this call
 */
int lsm6ds3_i2c_init(int i2c_num, uint8_t i2cAddr)
{
    int cmd=0;
    uint8_t buf[8];
    int retCode = 0;

    //Send all the commands to fully initialize the device
	while (lsm6ds3_init_cmds[cmd].databytes!=0xff) {
		buf[0] = lsm6ds3_init_cmds[cmd].data[0];
		retCode = i2c_writeBytes(i2c_num, i2cAddr, lsm6ds3_init_cmds[cmd].cmd, &buf[0], 1);
		if (retCode) break;
		cmd++;
	}

	return retCode;
}

/*
 * @brief 		Readback LSM6DS3 device ID
 *
 * @param		i2c_num		I2C Bus interface number
 * @param		i2c_addr	7-bit I2C bus address
 *
 * @return		Returns 0 for ok chip id and -1 for IO error or -2 for bad chip id
 * @retparam	*data		Single byte with the Device ID which should be 0x69
 */
int lsm6ds3_i2c_read_chipid(int i2c_num, uint8_t i2c_addr, uint8_t *data)
{
	uint8_t regAddr = LSM6DS3_CHIP_ID_REG;    // Read address for chip id register
	int	retCode = 0;

	retCode = i2c_readBytes(i2c_num, i2c_addr, regAddr, data, 1);

	if ((retCode == 0) && (*data != LSM6DS3_CHIP_ID)) {
		retCode = -2;
	}
	return retCode;
}

/*
 * @brief 		Set the lsm6ds3 sensor into low power suspend mode
 *
 * @param		i2c_num		I2C Bus interface number
 * @param		i2c_addr	7-bit I2C bus address
 *
 * @return		Returns 0 for ok or -1 for IO error
 * @note        Caller is responsible for use of i2c_lock() and i2c_unlock() outside of this call
 */
int lsm6ds3_i2c_powerdown(int i2c_num, uint8_t i2cAddr)
{
    int cmd=0;
    uint8_t buf[8];
    int retCode = 0;

    //Send all the commands to fully initialize the device
	while (lsm6ds3_powerdown_cmds[cmd].databytes!=0xff) {
		buf[0] = lsm6ds3_powerdown_cmds[cmd].data[0];
		retCode = i2c_writeBytes(i2c_num, i2cAddr, lsm6ds3_init_cmds[cmd].cmd, &buf[0], 1);
		if (retCode) break;
		cmd++;
	}

	return retCode;
}

/*
 * @brief 		Readback LSM6DS3 3D accel as 6 raw bytes in internal format from the chip
 *
 * @param		i2c_num		I2C Bus interface number
 * @param		i2c_addr	7-bit I2C bus address
 *
 * @retparam	*data		Array of bytes to receive 6 raw bytes as signed X,Y,Z
 * @return		Returns 0 for ok or -1 for IO error
 *
 * @note     Readback accel registers.  1st byte read freezes for other regs on same sample
 *          Assumes the IMU is running in continuous mode and not stopped or sleeping
 *          X, Y, then Z signed values each 2 bytes with LSB first.
 *          X is for accel along a line from pin 3 to pin 11
 *          Y is for accel along a line from pin 15 to pin 7
 *          Z is for accel along a line normal to the chip pointing out the top of the chip
 */
int lsm6ds3_i2c_read_accel(int i2c_num, uint8_t i2c_addr, uint8_t *data)
{
	uint8_t regAddr = LSM6DS3_ACCEL_BASE;    // Read address for 1st register of 6 bytes to read

	int retCode = i2c_readBytes(i2c_num, i2c_addr, regAddr, data, 6);

	return retCode;
}

/*
 * @brief 		Readback LSM6DS3 gyro as 6 raw bytes in internal format from the chip
 *
 * @param		i2c_num		I2C Bus interface number
 * @param		i2c_addr	7-bit I2C bus address
 *
 * @retparam	*data		Array of bytes to receive 6 raw bytes as signed X,Y,Z
 * @return		Returns 0 for ok or -1 for IO error
 *
 * @note    Readback byro registers. 1st byte read freezes for other regs on same sample
 *          Assumes the IMU is running in continuous mode and not stopped or sleeping
 *          X, Y, then Z signed values each 2 bytes with LSB first.
 *          X is for rotation about a line from pin 3 to pin 11
 *          Y is for rotation about a line from pins 7 to pin 15
 *          Z is for rotation of the chip about it's plane
 */
int lsm6ds3_i2c_read_gyro(int i2c_num, uint8_t i2c_addr, uint8_t *data)
{
	uint8_t regAddr = LSM6DS3_GYRO_BASE;    // Read address for 1st register of 6 bytes to read

	int retCode = i2c_readBytes(i2c_num, i2c_addr, regAddr, data, 6);

	return retCode;
}

/*
 * @brief 		Readback LSM6DS3 internal temperature sensor
 *
 * @param		i2c_num		I2C Bus interface number
 * @param		i2c_addr	7-bit I2C bus address
 *
 * @retparam	*tempC		Floating point value in degrees C
 * @return		Returns 0 for ok or -1 for IO error or -9 for bad pointer on input
 *
 * @note        This routine will try to acquire the I2C lock so do not lock before calling this routine
 */
int lsm6ds3_i2c_read_temperature(int i2c_num, uint8_t i2c_addr, float *tempC)
{
	uint8_t buf[4];
	uint8_t regAddr = LSM6DS3_TEMP_L;    // Read address for temperature lower order byte
	int16_t deciC = 0;
	float   tempDegC = 0.0;
	int retCode = 0;

	if (tempC == 0)
		return -8;

	i2c_lock();
	retCode = i2c_readBytes(i2c_num, i2c_addr, regAddr, &buf[0], 2);
	i2c_unlock();

	deciC = ((buf[1] << 8) & 0xFF00) | buf[0];    // This is in raw units which are 16 bits per deg C

	tempDegC = ((float)(deciC) / (float)(16.0)) + (float)(25.0);   // Readout is 0 at 25C
	// printf("lsm6ds3_i2c_read_temperature   hex value 0x%04x tempDegC %4.2f\n", deciC, tempDegC);

	if (retCode == 0) {
		*tempC = tempDegC;
	}

	return retCode;
}


/*
 * @name        lsm6ds3_i2c_readAccelVectorInGs
 * @brief 		Readback LSM6DS3 acceleration vector
 *
 * @param		i2c_num		I2C Bus interface number
 * @param		i2c_addr	7-bit I2C bus address
 *
 * @return		Returns struct of type accelVectorInGs_t for the acceleration vector in units of g
 *
 * @note    The X,Y,Z values in units of G are returned as a structure
 *
 * @note    This routine will try to acquire the I2C lock so do not lock before calling this routine
 */
accelVectorInGs_t lsm6ds3_i2c_readAccelVectorInGs(int i2c_num, uint8_t i2c_addr)
{
  double accelValue = 0.0;
  accelVectorInGs_t accelVector = { 0.0, 0.0, 0.0 };
  int16_t accelReading;
  uint8_t buf[8];

  i2c_lock();
  i2c_readBytes(i2c_num, i2c_addr, LSM6DS3_ACCEL_BASE, &buf[0], 6);  // Optimized to read just the 1 Z word
  i2c_unlock();

  accelReading = ((buf[1] << 8) & 0xff00) | buf[0];
  accelVector.x = (double)(accelReading) * ACCEL_PER_BIT_IN_G;

  accelReading = ((buf[3] << 8) & 0xff00) | buf[2];
  accelVector.y = (double)(accelReading) * ACCEL_PER_BIT_IN_G;

  accelReading = ((buf[5] << 8) & 0xff00) | buf[4];
  accelVector.z = (double)(accelReading) * ACCEL_PER_BIT_IN_G;

  return accelVector;
}

/*
 * @name        lsm6ds3_i2c_readGyroVectorInDegPerSec
 * @brief 		Readback LSM6DS3 gyro vector in degrees per second
 *
 * @param		i2c_num		I2C Bus interface number
 * @param		i2c_addr	7-bit I2C bus address
 *
 * @return		Returns struct of type gyroVector_t for the acceleration vector in units of g
 *
 * @note    The X,Y,Z values in units of G are returned as a structure
 *
 * @note    This routine will try to acquire the I2C lock so do not lock before calling this routine
 */
gyroVector_t lsm6ds3_i2c_readGyroVectorInDegPerSec(int i2c_num, uint8_t i2c_addr)
{
  int16_t gyroReading;
  gyroVector_t gyroVector = { 0.0, 0.0, 0.0 };
  double  rotationDegPerSec;
  uint8_t buf[8];

  i2c_lock();
  i2c_readBytes(i2c_num, i2c_addr, LSM6DS3_ACCEL_BASE, &buf[0], 6);  // Optimized to read just the 1 Z word
  i2c_unlock();

  // Get a rotational rate in degree per second that is in same direction as forward accel shows of tilt.
  // For a scale of 245 deg/sec max range we multiply bits by  245/32768
  gyroReading = ((buf[1] << 8) & 0xff00) | buf[0];
  rotationDegPerSec = (double)(gyroReading) * GYRO_DEG_PER_SEC_PER_BIT;
  gyroVector.x = (double)(gyroReading) * ACCEL_PER_BIT_IN_G;

  gyroReading = ((buf[3] << 8) & 0xff00) | buf[2];
  rotationDegPerSec = (double)(gyroReading) * GYRO_DEG_PER_SEC_PER_BIT;
  gyroVector.y = (double)(gyroReading) * ACCEL_PER_BIT_IN_G;

  gyroReading = ((buf[5] << 8) & 0xff00) | buf[4];
  rotationDegPerSec = (double)(gyroReading) * GYRO_DEG_PER_SEC_PER_BIT;
  gyroVector.z = (double)(gyroReading) * ACCEL_PER_BIT_IN_G;

  return gyroVector;
}


/*
 * @name		lsm6ds3_i2c_readForwardAccelInGs
 * @brief 		Readback LSM6DS3 forward acceleration (hard coded for Z azis)
 *
 * @param		i2c_num		I2C Bus interface number
 * @param		i2c_addr	7-bit I2C bus address
 *
 * @return		Returns double for the acceleration in units of g
 *
 * @note    Forward accel points out of the chip or forward and is near zero for vertical angle
 *          An angle leaning forward generates a positive tilt angle from this routine
 *
 *          This will deviate from 0 to indicate how far off of level and is the Z accel adjusted for offset
 *          Assumes the IMU is running in continuous mode and not stopped or sleeping
 *          We can read 1 LSB/MSB because with BDU enabled the 1st byte freezes reading for following byte
 *          This routine is specific for a given hardware orientation of the sensor!
 *
 * @note    This routine will try to acquire the I2C lock so do not lock before calling this routine
 */
double lsm6ds3_i2c_readForwardAccelInGs(int i2c_num, uint8_t i2c_addr)
{
  double accelValue = 0.0;
  int16_t accelReading;
  uint8_t buf[8];

  i2c_lock();
  i2c_readBytes(i2c_num, i2c_addr, (LSM6DS3_ACCEL_BASE+4), &buf[0], 2);  // Optimized to read just the 1 Z word
  i2c_unlock();

  accelReading = ((buf[1] << 8) & 0xff00) | buf[0];

  accelValue = (double)(accelReading) * ACCEL_PER_BIT_IN_G * (double)(-1.0);

  return accelValue;
}

// Forward rotation about the wheel axis that agrees in direction with accel tilt
// Rotation forward generates a positive tilt degrees per second from this routine
//
// This will deviate from zero for increase in angle about the wheel axis
// Assumes the IMU is running in continuous mode and not stopped or sleeping
// We can read 1 LSB/MSB because with BDU enabled the 1st byte freezes reading for following byte
// This routine is specific for a given hardware orientation of the sensor!
double lsm6ds3_i2c_readForwardGyroDegPerSec(int i2c_num, uint8_t i2c_addr)
{
  double rotationDegPerSec = 0.0;
  int16_t gyroReading;
  uint8_t buf[8];

  i2c_lock();
  i2c_readBytes(i2c_num, i2c_addr, (LSM6DS3_GYRO_BASE+2), &buf[0], 2);  // Optimized to read just the 1 Y Gyro
  i2c_unlock();

  gyroReading = ((buf[1] << 8) & 0xff00) | buf[0];

  // Get a rotational rate in degree per second that is in same direction as forward accel shows of tilt.
  // For a scale of 245 deg/sec max range we multiply bits by  245/32768
  rotationDegPerSec = (double)(gyroReading) * GYRO_DEG_PER_SEC_PER_BIT;

  return rotationDegPerSec;
}
