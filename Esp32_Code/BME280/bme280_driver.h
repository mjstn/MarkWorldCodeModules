/**
 * @file 	bme280_driver.h
 * @brief	Bosch BME280 environmental MEMS sensor drivers
 *
 */

#ifndef BME280_DRIVER_H_
#define BME280_DRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>


// BME280  accelerometer defines
#define BME280_I2C_BUS      I2C_NUM_1  	// Bind to system bus required.    THIS IS UNIQUE TO A GIVEN SYSTEM HARDWARE

#define BME280_I2C_ADDRESS      0x76    // Chip address over I2C bus when SD0 is tied to ground
#define BME280_CHIPID			0x60	// Chip ID for check we can access chip ok
#define BMP280_CHIPID			0x58	// Chip ID for Bmp280
#define BMX_ANY_CHIPID          0       // Used to read chip id from any BMx series be it Bme280, Bmp280, Bme680
#define BME280_SOFTRESET_VAL    0xB6    // Value used to soft reset register

#define BME680_CHIPID			0x61	// Chip ID for check we can access chip ok
#define BME680_SOFTRESET_VAL    0xB6    // Value used to soft reset register

#define ATM_PRES_PASCAL_TO_MBAR ((float)(0.01))


/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    enum
    {
      BME280_REGISTER_DIG_T1              = 0x88,
      BME280_REGISTER_DIG_T2              = 0x8A,
      BME280_REGISTER_DIG_T3              = 0x8C,

      BME280_REGISTER_DIG_P1              = 0x8E,
      BME280_REGISTER_DIG_P2              = 0x90,
      BME280_REGISTER_DIG_P3              = 0x92,
      BME280_REGISTER_DIG_P4              = 0x94,
      BME280_REGISTER_DIG_P5              = 0x96,
      BME280_REGISTER_DIG_P6              = 0x98,
      BME280_REGISTER_DIG_P7              = 0x9A,
      BME280_REGISTER_DIG_P8              = 0x9C,
      BME280_REGISTER_DIG_P9              = 0x9E,

      BME280_REGISTER_DIG_H1              = 0xA1,
      BME280_REGISTER_DIG_H2              = 0xE1,
      BME280_REGISTER_DIG_H3              = 0xE3,
      BME280_REGISTER_DIG_H4              = 0xE4,
      BME280_REGISTER_DIG_H5              = 0xE5,
      BME280_REGISTER_DIG_H6              = 0xE7,

	  BME280_REGISTER_CHIPID             = 0xD0,
      BME280_REGISTER_VERSION            = 0xD1,
      BME280_REGISTER_SOFTRESET          = 0xE0,

      BME280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

      BME280_REGISTER_CONTROLHUMID       = 0xF2,
      BME280_REGISTER_CONTROL            = 0xF4,
      BME280_REGISTER_CONFIG             = 0xF5,
      BME280_REGISTER_PRESSUREDATA       = 0xF7,
      BME280_REGISTER_TEMPDATA           = 0xFA,
      BME280_REGISTER_HUMIDDATA          = 0xFD,
    };

/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    typedef struct
    {
      uint16_t dig_T1;
      int16_t  dig_T2;
      int16_t  dig_T3;

      uint16_t dig_P1;
      int16_t  dig_P2;
      int16_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int16_t  dig_P6;
      int16_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;

      uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;

      int32_t  t_fine;		// This is not really cal but is context used across interface
    } bme280_calib_data_t;

// Interfaces to the BME280 driver calls.  See source for explanations.
// One key point is we use I2C or SPI bus based on chipAddr
extern int bme280_read_chipid(int i2c_num, uint8_t i2c_addr, uint8_t *data);
extern int bmenv_read_chipid(int i2c_num, uint8_t i2c_addr, uint8_t expectedId, uint8_t *data);
extern int bme280_Init(int i2cBus, uint8_t chipAddr, uint8_t chipId, bme280_calib_data_t *calData);
extern int bme280_readBytes(int i2cBus, uint8_t chipAddr, int8_t regAddr, uint8_t *bufr, uint16_t numBytes);
extern int bme280_writeBytes(int i2cBus, uint8_t chipAddr, int16_t regAddr, uint8_t *bufr, uint16_t numBytes);

extern int bme280_SoftReset(int i2cBus, uint8_t chipAddr);
extern int bme280_ReadTemperatureDeciC(int i2cBus, uint8_t chipI2cAddr, bme280_calib_data_t *calData, double *degreesDeciC);
extern int bme280_ReadPressurePa(int i2cBus, uint8_t chipI2cAddr, bme280_calib_data_t *calData, double *pressurePascal);
extern int bme280_ReadHumidityPct(int i2cBus, uint8_t chipI2cAddr, bme280_calib_data_t *calData, double *humidityPct);
extern int bme280_ReadAltitudeM(int i2cBus, uint8_t chipI2cAddr, bme280_calib_data_t *calData, double seaLevel, double *altitudeMeters);

#endif  //   BME280_DRIVER_H_

