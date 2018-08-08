/**
 * @file 	bme280_driver.c
 * @brief	Bosch BME280 MEMS environmental sensor driver file
 *
 * @note	Each sensor has factory cal data in registers internal to the chip
 * 			The sensor must use this data to get any sort of reliable readings.
 *
 * @note    This driver supports the use of I2C or SPI communications as long as
 *          the Arm_Drivers code for SPI master or I2C master ports are same as
 *          defined by lower level spimaster_arm_drv and i2c_arm_drv low level drivers.
 */
 
#include "include/i2c_driver.h"
#include "bme280_driver.h"
#include <math.h>
#include "driver_errors.h"

/**
 * @name				bme280_readBytes
 * @brief				Read one or more bytes from the sensor using either I2C or SPI per address
 *
 * @param i2cBus		I2C peripheral e.g. I2C_NUM_1
 * @param chipAddr	    7-bit format I2C address or 3-bit address to use SPI when at or below spiMaster_ChipAddr_Maximum
 * @param regAddr		Internal chip register to be read.  We set address then read bytes
 * @param bufr			Raw byte buffer pointer for 1st byte read.
 * @param numBytes		Number of bytes to be read.  USER MUST BE RESPONSIBLE for buffer overrun
 *
 * @return				Zero indicates no error.  Errors defined as type DriverErrorCode_t
 *
 * @note	This call uses I2C hardware drivers and caller must lock I2C bus with system lock
 */
int bme280_readBytes(int i2cBus, uint8_t chipAddr, int8_t regAddr, uint8_t *bufr, uint16_t numBytes)
{
	int retCode = DRIVER_ERR_NO_ERROR;

	if (bufr == 0) {
	   return DRIVER_ERR_INVALID_INPUT_POINTER;
	}

	// Call the I2C routine to do N reads with optional register addressing
	retCode = i2c_readBytes(i2cBus, chipAddr, regAddr, bufr, numBytes);
	if (retCode != DRIVER_ERR_NO_ERROR) {
		return retCode;
	}

	return retCode;
}

/**
 * @name				bme280_writeBytes
 * @brief				Write one or more bytes to the sensor using either I2C or SPI per address
 *
 * @param i2cBus		I2C peripheral e.g. I2C_NUM_1
 * @param chipAddr	    7-bit format I2C address or 3-bit address to use SPI when at or below spiMaster_ChipAddr_Maximum
 * @param regAddr		Internal chip register to be read or written.  We set address then read bytes
 * @param bufr			Raw byte buffer pointer for 1st byte read.
 * @param numBytes		Number of bytes to be read.  USER MUST BE RESPONSIBLE for buffer overrun
 *
 * @retval			    Zero indicates no error.  Errors defined as type DriverErrorCode_t
 *
 * @note	This call uses I2C hardware drivers and caller must lock I2C bus with system lock
 */
int bme280_writeBytes(int i2cBus, uint8_t chipAddr, int16_t regAddr, uint8_t *bufr, uint16_t numBytes)
{
	int retCode = DRIVER_ERR_NO_ERROR;

	if (bufr == 0) {
	   return DRIVER_ERR_INVALID_INPUT_POINTER;
	}

	// Call the I2C routine to do N writes with optional register addressing
	retCode = i2c_writeBytes(i2cBus, chipAddr, regAddr, bufr, numBytes);
	if (retCode != DRIVER_ERR_NO_ERROR) {
		return retCode;
	}

	return retCode;
}


/**
 * @name	  bme280_InitCalData
 * @brief	  Local use Helper to read in BME280 cal data for our usage in this driver
 *
 * @param     i2cBus		The I2C bus number
 *
 * @return    Returns 0 for all is well or other driver error for problems
 * @retparam  calData       Calibration data for the BME 280 environmental sensor
 *
 * @note      Caller MUST lock the I2C bus or assure no other calls access I2C as this call progresses
 * @note	  Be aware this routine combines unsigned 8-bit values into
 *            same or longer unsigned values and even multi-byte signed values.
 * @note	  This call uses I2C hardware drivers and caller must lock I2C bus with system lock
 */
static int	bme280_InitCalData(int i2cBus, uint8_t chipI2cAddr, bme280_calib_data_t *calData) {
	int retCode = DRIVER_ERR_NO_ERROR;
	uint8_t   buf[20];
	uint16_t  uint16;
	int16_t   int16;

	if (calData == 0) {
	  return DRIVER_ERR_INVALID_INPUT_POINTER;
	}

	// We will read in the data in blasts then place it in our CAL data

	// The Temp cal factors come at us in LSB,MSB order
	retCode = bme280_readBytes(i2cBus, chipI2cAddr, BME280_REGISTER_DIG_T1, &buf[0], 6);
	calData->dig_T1 = (buf[1] << 8) | buf[0];
	calData->dig_T2 = (buf[3] << 8) | buf[2];
	calData->dig_T3 = (buf[5] << 8) | buf[4];

	// All the 'P' values are 16-bit signed values except P1 is unsigned. Bytes are LSB then MSB
	retCode |= bme280_readBytes(i2cBus, chipI2cAddr, BME280_REGISTER_DIG_P1, &buf[0], 18);
	calData->dig_P1 = (buf[1] << 8)  | buf[0];
	calData->dig_P2 = (buf[3] << 8)  | buf[2];
	calData->dig_P3 = (buf[5] << 8)  | buf[4];
	calData->dig_P4 = (buf[7] << 8)  | buf[6];
	calData->dig_P5 = (buf[9] << 8)  | buf[8];
	calData->dig_P6 = (buf[11] << 8) | buf[10];
	calData->dig_P7 = (buf[13] << 8) | buf[12];
	calData->dig_P8 = (buf[15] << 8) | buf[14];
	calData->dig_P9 = (buf[17] << 8) | buf[16];

	// For humidity we have some single 8-bit values and even swapped nibbles in some cases
	retCode |= bme280_readBytes(i2cBus, chipI2cAddr, BME280_REGISTER_DIG_H1, &buf[0], 1);
	calData->dig_H1 = buf[0];

	retCode |= bme280_readBytes(i2cBus, chipI2cAddr, BME280_REGISTER_DIG_H2, &buf[0], 2);
	calData->dig_H2 = (buf[1] << 8)  | buf[0];	// LSB then MSB

	retCode |= bme280_readBytes(i2cBus, chipI2cAddr, BME280_REGISTER_DIG_H3, &buf[0], 1);
	calData->dig_H3 = buf[0];

	retCode |= bme280_readBytes(i2cBus, chipI2cAddr, BME280_REGISTER_DIG_H4, &buf[0], 1);
	retCode |= bme280_readBytes(i2cBus, chipI2cAddr, BME280_REGISTER_DIG_H4+1, &buf[4], 1);
	calData->dig_H4 = (buf[0] << 4) | (buf[4] & 0xf);

	retCode |= bme280_readBytes(i2cBus, chipI2cAddr, BME280_REGISTER_DIG_H5, &buf[0], 1);
	retCode |= bme280_readBytes(i2cBus, chipI2cAddr, BME280_REGISTER_DIG_H5+1, &buf[4], 1);
	calData->dig_H5 = (buf[4] << 4) | (buf[0] >> 4);

	retCode |= bme280_readBytes(i2cBus, chipI2cAddr, BME280_REGISTER_DIG_H6, &buf[0], 1);
	calData->dig_H6 = buf[0];

	return retCode;
}

/**
 * @name				bme280_read_chipid
 * @brief				Readback chip ID which should ALWAYS be 0x60
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param i2c_addr		The 7-bit I2C chip address for the slave I2C device
 * @param data			User supplied buffer that the chip ID is read back into if non-zero
 *
 * @retval				Returns 0 if chip ID is ok else -1
 */
int bme280_read_chipid(int i2c_num, uint8_t i2c_addr, uint8_t *data)
{
	int retCode = 0;
	uint8_t regVal = 0;

	uint8_t regAddr = BME280_REGISTER_CHIPID;    // Read address for chip id register

	i2c_readBytes(i2c_num, i2c_addr, regAddr, &regVal, 1);

	if (data != NULL) {
		*data = regVal;
	}

	if (regVal != BME280_CHIPID ) {
		retCode = -1;
	}

	return retCode;
}


/**
 * @name	bme280_Init
 * @brief	Setup chip and Read in calibration data for BME280 so we can correct data later
 *
 * @param i2cBus		The I2C bus number
 * @param chipAddr	    7-bit format I2C address
 * @param chipId        This identifies if we are using BME280 or BME680
 *
 * @return calData      Calibration data is filled in and is needed for measurements in later calls
 * @retval				Zero indicates no error.  Errors defined as type DriverErrorCode_t
 *
 */
int	bme280_Init(int i2cBus, uint8_t chipAddr, uint8_t chipId, bme280_calib_data_t *calData)
{

	int retCode = DRIVER_ERR_NO_ERROR;
	uint8_t write_data[16];
	uint8_t read_data[8];

	if (calData == 0) {
	   return DRIVER_ERR_INVALID_INPUT_POINTER;
	}

	i2c_lock();		// Get lock for I2C bus

	// Do soft reset.   When this is used we get really way off errors.  Perhaps needs delay after it?
	// bme280_SoftReset(chipAddr);

	while (1) {
		// Verify chip ID is correct or fail right away
		read_data[0] = !chipId;
		retCode = bme280_read_chipid(i2cBus, chipAddr, &read_data[0]);
		if (retCode != DRIVER_ERR_NO_ERROR) {
			break;
		}

		// Here we allow chipid of BME280 or BME680 as both will work for BME280 features
		if (read_data[0] != chipId) {
			retCode = DRIVER_ERR_INVALID_CHIP_ID;
			break;
		}

		// Read in the factory cal data for usage as we do samples later
		retCode = bme280_InitCalData(i2cBus, chipAddr, calData);
		if (retCode != DRIVER_ERR_NO_ERROR) {
			break;
		}

		// Config data is only written in sleep mode so assure instant activation
		// Some notes imply data values are held off when measuring and we are writing values fast so go to sleep
		write_data[0] = 0;
		retCode = bme280_writeBytes(i2cBus, chipAddr, BME280_REGISTER_CONTROL, &write_data[0], 1);
		if (retCode != DRIVER_ERR_NO_ERROR) {
			break;
		}

		// Set oversampling values but more importantly take the mode into  Normal run mode
		// 1x temperature oversampling in bits [7:5], 1x pressure oversampling in bits [4:2] and 'normal' mode
		write_data[0] = (0 << 5)  |  // measure fastest rate
						(0 << 2)  |  // filter off
						0;           // Keep in SPI 4-wire mode
		retCode = bme280_writeBytes(i2cBus, chipAddr, BME280_REGISTER_CONFIG, &write_data[0], 1);
		if (retCode != DRIVER_ERR_NO_ERROR) {
			break;
		}

		// Used to control the measurement of humidity. Only goes active after write to control reg next
		write_data[0] = 5;		// humidity oversampling
		retCode = bme280_writeBytes(i2cBus, chipAddr, BME280_REGISTER_CONTROLHUMID, &write_data[0], 1);
		if (retCode != DRIVER_ERR_NO_ERROR) {
			break;
		}

		// Set oversampling values but more importantly take the mode into  Normal run mode
		// 1x temperature oversampling in bits [7:5], 1x pressure oversampling in bits [4:2] and 'normal' mode
		write_data[0] = (5 << 5)  |  // some temperature oversampling since we are in no rush and less noise is good.
						(5 << 2)  |  // some pressure oversampling to avoid vibration throwing off pressure
						3;           // 1=forced mode, 3 = Normal mode.  00 is sleep, 1 or 2 is forced mode
		retCode = bme280_writeBytes(i2cBus, chipAddr, BME280_REGISTER_CONTROL, &write_data[0], 1);
		if (retCode != DRIVER_ERR_NO_ERROR) {
			break;
		}

		break;	// Always just exit this while loop
	}

	i2c_unlock();		// Get lock for I2C bus

	return retCode;
}

/**
 * @name	bme280_SoftReset
 * @brief	Do a soft reset.  Must use bme280_Init() after a short delay from this call.
 *
 * @param i2cBus		The I2C bus number
 * @param chipAddr	    7-bit format I2C address
 *
 * @retval				Zero indicates no error.  Errors defined as type DriverErrorCode_t
 *
 */
int	bme280_SoftReset(int i2cBus, uint8_t chipI2cAddr)
{

	int retCode = DRIVER_ERR_NO_ERROR;
	uint8_t write_data[4];

	// Used to control the measurement of humidity
	write_data[0] = BME280_SOFTRESET_VAL;
	retCode = bme280_writeBytes(i2cBus, chipI2cAddr, BME280_REGISTER_SOFTRESET, &write_data[0], 1);
	if (retCode != DRIVER_ERR_NO_ERROR) {
		return retCode;
	}

	return retCode;
}


/**
 * @name 	bme280_ReadTemperatureDeciC
 * @brief	Read temperature in 10th of degree C and set a temp fine factor Bosch Bme280 sensor
 *
 * @param i2cBus		The I2C bus number
 * @param chipAddr	    7-bit format I2C address
 * @param calData		The calibration data for this device
 *
 * @return	degreesDeciC:  Temperature is in 10th of degree Centigrade
 * @retval	Zero indicates no error.  Errors defined as type DriverErrorCode_t
 */
int bme280_ReadTemperatureDeciC(int i2cBus, uint8_t chipI2cAddr, bme280_calib_data_t *calData, double *degreesDeciC)
{
  int retCode = DRIVER_ERR_NO_ERROR;
  int32_t var1, var2;
  float temp;
  int32_t adc_T;
  uint32_t buf24;
  uint8_t buf[8];

  if ((calData == 0) || (degreesDeciC == 0)) {
     return DRIVER_ERR_INVALID_INPUT_POINTER;
  }

  //i2c_lock();		// Get lock for I2C bus

  retCode = bme280_readBytes(i2cBus, chipI2cAddr, BME280_REGISTER_TEMPDATA, &buf[0], 3);
  if (retCode != DRIVER_ERR_NO_ERROR) {
	//i2c_unlock();
    return retCode;
  }
  //i2c_unlock();

  buf24 = (buf[0] << 16) | (buf[1] << 8) | buf[2];
  adc_T = buf24 >> 4;

  var1  = ((((adc_T>>3) - ((int32_t)calData->dig_T1 <<1))) *
	   ((int32_t)calData->dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)calData->dig_T1)) *
	     ((adc_T>>4) - ((int32_t)calData->dig_T1))) >> 12) *
	   ((int32_t)calData->dig_T3)) >> 14;

  calData->t_fine = var1 + var2;

  float T  = (calData->t_fine * 5 + 128) >> 8;
  temp = T/(float)(10.0);

  *degreesDeciC = temp;

  return DRIVER_ERR_NO_ERROR;
}

/**
 * @name 	bme280_ReadPressurePa
 * @brief	Read barametric pressure for Bosch Bme280 sensor in units of Pascal (Pa)
 *
 * @param i2cBus		The I2C bus number
 * @param chipAddr	    7-bit format I2C address
 * @param calData		The calibration data for this device
 *
 * @return	pressurePascal:  Read in corrected units of Pascals
 * @retval	Zero indicates no error.  Errors defined as type DriverErrorCode_t
 *
 * @note    101.325 kPa is 101325 from this routine and is normal sea level pressure
 * @note    Pa * 0.00750 -> mmHg,   Pa * 0.00402 -> inH2O,  Pa * 0.01 -> milliBar
 */
int bme280_ReadPressurePa(int i2cBus, uint8_t chipI2cAddr, bme280_calib_data_t *calData, double *pressurePascal)
{
  int retCode = DRIVER_ERR_NO_ERROR;
  int64_t var1, var2, p;
  double temp;
  double pres;
  int32_t adc_P;
  uint32_t buf24;
  uint8_t buf[8];

  if ((calData == 0) || (pressurePascal == 0)) {
     return DRIVER_ERR_INVALID_INPUT_POINTER;
  }

  //i2c_lock();		// Get lock for I2C bus
  retCode = bme280_ReadTemperatureDeciC(i2cBus, chipI2cAddr, calData, &temp); // must be done first to get t_fine
  if (retCode != DRIVER_ERR_NO_ERROR) {
	//i2c_unlock();
    return retCode;
  }

  retCode = bme280_readBytes(i2cBus, chipI2cAddr, BME280_REGISTER_PRESSUREDATA, &buf[0], 3);
  //i2c_unlock();
  if (retCode != DRIVER_ERR_NO_ERROR) {
    return retCode;
  }
  buf24 = (buf[0] << 16) | (buf[1] << 8) | buf[2];
  adc_P = buf24 >> 4;

  var1 = ((int64_t)calData->t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)calData->dig_P6;
  var2 = var2 + ((var1*(int64_t)calData->dig_P5)<<17);
  var2 = var2 + (((int64_t)calData->dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)calData->dig_P3)>>8) +
    ((var1 * (int64_t)calData->dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calData->dig_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)calData->dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)calData->dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)calData->dig_P7)<<4);

  pres = (double)(p)/(double)(256.0);
  *pressurePascal = pres;

  return DRIVER_ERR_NO_ERROR;
}


/**
 * @name 	bme280_ReadHumidityPct
 * @brief	Read air relative humidity read from the Bosch Bme280 sensor
 *
 * @param i2cBus		The I2C bus number
 * @param chipAddr	    7-bit format I2C address
 * @param calData		The calibration data for this device
 *
 * @return	humidityPct:  Relative humidity in percent
 * @retval	Zero indicates no error.  Errors defined as type DriverErrorCode_t
 */
int bme280_ReadHumidityPct(int i2cBus, uint8_t chipI2cAddr, bme280_calib_data_t *calData, double *humidityPct)
{
  int retCode = DRIVER_ERR_NO_ERROR;
  double hum;
  double temp;
  int32_t adc_H;
  uint32_t buf16;
  uint8_t buf[8];

  if ((calData == 0) || (humidityPct == 0)) {
    return DRIVER_ERR_INVALID_INPUT_POINTER;
  }

  //i2c_lock();		// Get lock for I2C bus
  retCode = bme280_ReadTemperatureDeciC(i2cBus, chipI2cAddr, calData, &temp); // must be done first to get t_fine
  if (retCode != DRIVER_ERR_NO_ERROR) {
	//i2c_unlock();
    return retCode;
  }

  retCode = bme280_readBytes(i2cBus, chipI2cAddr, BME280_REGISTER_HUMIDDATA, &buf[0], 2);
  //i2c_unlock();
  if (retCode != DRIVER_ERR_NO_ERROR) {
    return retCode;
  }
  buf16 = (buf[0] << 8) | buf[1];		// Order here is MSB then LSB
  adc_H = buf16;

  int32_t v_x1_u32r;

  v_x1_u32r = (calData->t_fine - ((int32_t)76800));

  v_x1_u32r = (((((adc_H << 14) - (((int32_t)calData->dig_H4) << 20) -
		  (((int32_t)calData->dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
	       (((((((v_x1_u32r * ((int32_t)calData->dig_H6)) >> 10) *
		    (((v_x1_u32r * ((int32_t)calData->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
		  ((int32_t)2097152)) * ((int32_t)calData->dig_H2) + 8192) >> 14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
			     ((int32_t)calData->dig_H1)) >> 4));

  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  double h = (v_x1_u32r>>12);
  hum =  h / (double)(1024.0);

  *humidityPct = hum;

  return DRIVER_ERR_NO_ERROR;
}

/**
 * @name 	bme280_ReadAltitudeM
 * @brief	Read altitude in meters from the Bosch Bme280 sensor
 *
 * @param   chipAddr	  7-bit format I2C address
 * @param   calData		  The calibration data for this device
 * @param   seaLevel      Sea-level pressure in hPa.  Hint: inHg * 3.386 gives hPa with 103 typical
 * @param   atmospheric   Atmospheric pressure in hPa
 *
 * @note    Calculates the altitude (in meters) from the specified atmospheric
 *          pressure (in hPa), and sea-level pressure (in hPa).
 *
 * @return	humidity  is the air humidity corrected with built in calibration data
 * @retval	Zero indicates no error.  Errors defined as type DriverErrorCode_t
 */
int bme280_ReadAltitudeM(int i2cBus, uint8_t chipI2cAddr, bme280_calib_data_t *calData, double seaLevel, double *altitudeMeters)
{
  double alt;
  double pres;

  if ((calData == 0) || (altitudeMeters == 0)) {
     return DRIVER_ERR_INVALID_INPUT_POINTER;
  }

  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  //i2c_lock();		// Get lock for I2C bus
  bme280_ReadPressurePa(i2cBus, chipI2cAddr, calData, &pres);
  //i2c_unlock();
  double atmospheric = pres / 100.0F;
  alt =  44330.0 * (1.0 - pow((atmospheric / seaLevel), (double)(0.1903)));

  *altitudeMeters = alt;

  return DRIVER_ERR_NO_ERROR;
}
