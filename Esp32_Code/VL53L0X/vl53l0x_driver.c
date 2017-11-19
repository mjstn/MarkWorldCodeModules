/**
 * ST Micro VL53L0X Simplified Driver for C Esp32 Environment in FreeRTOS environment
 *
 * This is a greatly simplified driver that appears to work 'good enough' for rough distances
 * This driver is in simple C so it can be used in simpler compiler environments.
 *
 * This code requires the i2c_driver.c and .h to be part of the make
 *
 * Disclaimers is that this code does NOT do the rather complex calibrations and so on
 * and as such it would only be as accurate as the factory default setup.
 * I have found it to be good enough for basic robotics object detects and in fact is fairly good
 *
 * Formed by mjstn2011@gmail.com for Mark-Toys.com usage
 */

#include "include/i2c_driver.h"			// Required for I2C support. This is mark-toys.com implementation
#include "include/vl53l0x_driver.h"

// User must supply these system resource APIs to this code
extern void system_delayMilliSeconds(int milliSeconds);


// This value is used in issue of new commands.
static uint8_t g_vl53l0x_stopVar = 0;		

// Here we have api hooks that may be system specific
static void delay_ms(int milliSec)
{
	system_delayMilliSeconds(milliSec);
}

//  These commands are sent to initialize VL53L0x chip in standard mode
//  WARNING:   Stoped work as it is very complex so will try for this later
DRAM_ATTR static const i2c_cmd_table_t vl53l0x_cmds_1[]={
    {0x80, {0x01}, 1},
    {0xff, {0x01}, 1},
    {0x00, {0x00}, 1},
    {0, {0}, 0xff},
};

DRAM_ATTR static const i2c_cmd_table_t vl53l0x_cmds_2[]={
	{0x00, {0x01}, 1},
	{0xff, {0x00}, 1},
	{0x80, {0x00}, 1},
    {0, {0}, 0xff},
};

DRAM_ATTR static const i2c_cmd_table_t vl53l0x_stopContCmds[]={
	{0xff, {0x01}, 1},
	{0x00, {0x00}, 1},
	{0x91, {0x00}, 1},
	{0x00, {0x01}, 1},
	{0xff, {0x00}, 1},
    {0, {0}, 0xff},
};


/**
 * @name				vl53l0x_writeReg
 * @brief				Write a single byte to an I2C device internal register
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param i2c_addr		The 7-bit I2C chip address for the slave I2C device
 * @param regAddr		Internal register to set for the access.
 * @param value			Byte to be written to the register
 *
 * @return				Returns 0 for all went well else there was an error
 */
int vl53l0x_writeReg(uint8_t i2c_num, uint8_t i2c_addr, uint8_t regAddr, uint8_t value)
{
	uint8_t regVal = value;

	if (i2c_writeBytes(i2c_num, i2c_addr, regAddr, &regVal, 1)  != 0) {
		i2c_unlock();
		return -2;
	}

	return 0;
}

/**
 * @name				vl53l0x_read_chipid
 * @brief				Readback chip ID which should ALWAYS be 0xC0
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param i2c_addr		The 7-bit I2C chip address for the slave I2C device
 * @param data			User supplied buffer that the chip ID is read back into if non-zero
 *
 * @retval				Returns 0 if chip ID is ok else -1
 */
int vl53l0x_read_chipid(int i2c_num, uint8_t i2c_addr, uint8_t *data)
{
	int retCode = 0;
	uint8_t regVal = 0;

	uint8_t regAddr = VL53L0X_WHO_AM_I;    // Read address for chip id register

	i2c_readBytes(i2c_num, i2c_addr, regAddr, &regVal, 1);

	if (data != NULL) {
		*data = regVal;
	}

	if (regVal != VL53L0X_REF_REG_C0_VAL ) {
		retCode = -1;
	}

	return retCode;
}

/**
 * @name				vl53l0x_initI2cAddresses
 * @brief				Sets up unique I2C address for each of N chips that start with default I2C addr
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param firstI2cAddr	First value of I2C chip address for the 1st one to be set
 * @param xshutGpioPins	A list of the GPIO lines that control each of the chips to have its addr set
 *                      If first value is negative THEN it is the NEGATIVE of the PCF8574 that will be XSHUT pins
 *                      For PCF8574 1st sensor must be on P0 and so on with second sensor xshut on P1 and so on.
 * @param chipCount		Number of chips we will be initializing with unique, sequential addresses
 *
 * @return              Returns 0 for all seems ok and negative for input value is bad.  Positive for other failure
 *
 * @note  The VL53L0X comes up with fixed address so for more than one we must set all different.
 *        For each chip we release a GPIO line then set a new address which is sequentially from firstI2cAddr
 */
int vl53l0x_initI2cAddresses(int i2c_num, uint8_t firstI2cAddr, int *xshutGpioPins, int chipCount) {
	int idx;
	int i2c_addr;
	int i2c_pcf8574_addr = 0;
	uint8_t buf[4];

	if (chipCount == 0) {
		return 0;		// Not really an error to ask for no devices to be setup, just 'stupid'
	}

	if (xshutGpioPins == 0) {
		return -1;		// avoid a crash and indicate input error
	}

	i2c_lock();		// Get lock for I2C bus

    // First setup GPIO lines as low outputs so all chips are disabled
	if (xshutGpioPins[0] >= 0) {
		for (idx=0; idx<chipCount ; idx++) {
			gpio_set_direction(xshutGpioPins[idx], GPIO_MODE_OUTPUT);
			gpio_set_level(xshutGpioPins[idx], 0);
		}
	} else {
		// Using PCF8574 for all xshut pins so set all low now
		i2c_pcf8574_addr = xshutGpioPins[0] * -1;		// Negative of input value will be PCF8574 I2C 7-bit addr
		buf[0] = 0;		// Set all lines low on PCF8574
		if (i2c_writeBytes(I2C_MASTER_NUM, i2c_pcf8574_addr, 0xFF, &buf[0], 1) != 0) {
			i2c_unlock();
			return -2;
		}
	}

	// Now one by one enable the chip by setting XShut as input to emulate open collector and set it's address
	buf[0] = 0x01;  // setup for very 1st xshut pin to be released to high which is weak pullup or like open collector
	for (idx=0; idx<chipCount ; idx++) {
		i2c_addr = firstI2cAddr + idx;			// This will be the address we will set this sensor to now

		// First release the sensor's XSHUT pin from low (shutdown) to high and it will use default I2C addr
		if (i2c_pcf8574_addr > 0) {
			if (i2c_writeBytes(I2C_MASTER_NUM, i2c_pcf8574_addr, 0xFF, &buf[0], 1)  != 0)  {
				i2c_unlock();
				return -3;
			}
			printf("sensor %d will be set to I2C addr 0x%x with new pcf8574 value 0x%x\n",idx, i2c_addr, buf[0]);
			buf[0] = (buf[0] << 1) | 0x01;     // get ready for one more bit to be set high to release one more chip
		} else {
			gpio_set_direction(xshutGpioPins[idx], GPIO_MODE_INPUT);
		}

		// Now the xshut line is high on one more sensor so set it's address to next highest address
		delay_ms(2);		// Grace period to come out of reset
		if (vl53l0x_writeReg(i2c_num, VL53L0X_POWER_ON_ADDRESS, I2C_SLAVE_DEVICE_ADDRESS, i2c_addr) != 0)  {
			i2c_unlock();
			return -4;
		}
	}

	i2c_unlock();
	return 0;
}

/**
 * @name				vl53l0x_init
 * @brief				Initialize the lsm6ds3 sensor on the SPI bus in a very minimal way with non-corrected values
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param i2c_addr		The 7-bit I2C chip address for the slave I2C device
 *
 * @return				Returns 0 for ok or negative for some sort of error
 *
 * @note  Only a poor-man's port of a full driver is present as full initialization is far more complex.
 *        We need to understand what the errors but results so far indicate 'close enough' for rough distances
 */
int vl53l0x_init(int i2c_num, uint8_t i2c_addr)
{
    int cmd=0;
    uint8_t buf[8];
    uint8_t regVal;

    i2c_lock();		// Get lock for I2C bus

    // On initial init we write one value out before the common use table can be written
	if (vl53l0x_writeReg(i2c_num, i2c_addr, 0x88, 0) != 0)  {
		i2c_unlock();
		return -21;
	}

    //Send several commands to start to initialize the device
    i2c_writeCmdTable(i2c_num, i2c_addr, vl53l0x_cmds_1);
	i2c_readBytes(i2c_num, i2c_addr, 0x91, &g_vl53l0x_stopVar, 1);	// Read stop_variable
	i2c_writeCmdTable(i2c_num, i2c_addr, vl53l0x_cmds_2);

	i2c_readBytes(i2c_num, i2c_addr, MSRC_CONFIG_CONTROL, &regVal, 1);
	vl53l0x_writeReg(i2c_num, i2c_addr, MSRC_CONFIG_CONTROL, 0x12);

	//PROBLEM: This call locks up as of 20170805 and the cause has not been debugged yet
	//printf("vl53l0x_init: Set signal rate limit\n");
	//vl53l0x_setSignalRateLimit(i2c_num, i2c_addr, (float)(0.25));

	vl53l0x_writeReg(i2c_num, i2c_addr, SYSTEM_SEQUENCE_CONFIG, 0xff);

	// SKIPPING SPAD Initialization in 'hope' we can avoid all that BS
	// SKIPPING load tunnel Initialization in 'hope' we can avoid all that BS

	// SKIPPING a LOT of odds and ends here including several cal steps like 'ref', 'vhv' and 'phase'

	// GOD this chip is so totally DIFFICULT to setup!  I really hope I can get 'close' with basic init

	// Setup interrupts so we can see meas done and so on.
	// We need to set this even if hardware interrupt is not used as we look at bits for measure done
	vl53l0x_writeReg(i2c_num, i2c_addr, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
	printf("vl53l0x_init: Readback some bytes \n");
	if (i2c_readBytes(i2c_num, i2c_addr, GPIO_HV_MUX_ACTIVE_HIGH, &regVal, 1)  != 0)  {
		i2c_unlock();
		return -28;
	}
	regVal = regVal & ~0x10;
	vl53l0x_writeReg(i2c_num, i2c_addr, GPIO_HV_MUX_ACTIVE_HIGH, regVal);
	vl53l0x_writeReg(i2c_num, i2c_addr, SYSTEM_INTERRUPT_CLEAR, 0x01);

	i2c_unlock();

	return 0;
}

/**
 * @name				vl53l0x_setSignalRateLimit
 * @brief				Sets signal rate limit
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param i2c_addr		The 7-bit I2C chip address for the slave I2C device
 * @param limit_Mcps	A limit value to be set
 *
 * @note  WARNING!  THIS CALL HAS NOT BEEN DEBUGGED AND IS DIRECT FROM A CODE PORT!
 */
bool vl53l0x_setSignalRateLimit(int i2c_num, uint8_t i2c_addr, float limit_Mcps)
{
  int regVal;
  uint8_t buf[8];

  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  regVal = limit_Mcps * (1 << 7);

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  buf[0] = regVal >> 8;
  buf[1] = regVal & 0xff;
  i2c_lock();		// Get lock for I2C bus
  i2c_writeBytes(i2c_num, i2c_addr, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, &buf[0], 2);
  i2c_unlock();

  return true;
}

/**
 * @name				vl53l0x_performSingleRefCalibration
 * @brief				Do a single ref calibration
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param i2c_addr		The 7-bit I2C chip address for the slave I2C device
 * @param vhv_init_byte The vhv value to be setup
 *
 * @retval				Zero indicates no error.  -1 is timeout
 *
 * @note  WARNING!  THIS CALL HAS NOT BEEN DEBUGGED AND IS DIRECT FROM A CODE PORT!
 */
int vl53l0x_performSingleRefCalibration(int i2c_num, uint8_t i2c_addr, uint8_t vhv_init_byte)
{
	int regVal;

	i2c_lock();		// Get lock for I2C bus

	vl53l0x_writeReg(i2c_num, i2c_addr, SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

	if (i2c_waitOnRegStatus(i2c_num, i2c_addr, RESULT_INTERRUPT_STATUS, 7, WAIT_FOR_NONZERO, VL53L0X_DEFAULT_INT_TIMEOUT_TICS) != 0) {
		i2c_unlock();
		return -1;
	}

	vl53l0x_writeReg(i2c_num, i2c_addr, SYSTEM_INTERRUPT_CLEAR, 0x01);

	vl53l0x_writeReg(i2c_num, i2c_addr, SYSRANGE_START, 0x00);

	i2c_unlock();

	return 0;
}


/**
 * @name				vl53l0x_startContinuous
 * @brief				Start the simplest of the continuous measurement modes
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param i2c_addr		The 7-bit I2C chip address for the slave I2C device
 *
 * @note  Saves a GLOBAL called g_vl53l0x_stopVar that is used in future commands to the chip
 */
void vl53l0x_startContinuous(int i2c_num, uint8_t i2c_addr)
{
	uint8_t regVal;

	i2c_lock();		// Get lock for I2C bus

	//Send several commands to start to initialize the device
	i2c_writeCmdTable(i2c_num, i2c_addr, vl53l0x_cmds_1);
	vl53l0x_writeReg(i2c_num, i2c_addr, 0x91, g_vl53l0x_stopVar);	// write stop_variable
	i2c_writeCmdTable(i2c_num, i2c_addr, vl53l0x_cmds_2);

	vl53l0x_writeReg(i2c_num, i2c_addr, SYSRANGE_START, 0x02);
	i2c_unlock();

	return;
}

/**
 * @name				vl53l0x_startContinuous
 * @brief				Stop continuous measurement mode
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param i2c_addr		The 7-bit I2C chip address for the slave I2C device
 */
void vl53l0x_stopContinuous(int i2c_num, uint8_t i2c_addr)
{
	vl53l0x_writeReg(i2c_num, i2c_addr, SYSRANGE_START, 0x01);

	i2c_writeCmdTable(i2c_num, i2c_addr, vl53l0x_stopContCmds);
}


/**
 * @name				vl53l0x_readRangeContinuousMm
 * @brief				Waits for reading then reads range value assuming the chip had been triggered
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param i2c_addr		The 7-bit I2C chip address for the slave I2C device
 *
 * @retval				A 14-bit distance in mm where bits D14 and D15 if set indicate an error
 * @return	status		8-bit range status value that shows measurement errors. Use NULL to not use this.
 *
 * @note                Requires interrupts to be enabled so status of meas done can be read
 */
uint16_t vl53l0x_readRangeContinuousMm(int i2c_num, uint8_t i2c_addr, uint8_t *status)
{
	uint16_t  retValue = 0;
	uint8_t   rangeStatus = 0;

	uint16_t rangeValue;
	uint8_t buf[8];

	i2c_lock();		// Get lock for I2C bus

	if (i2c_waitOnRegStatus(i2c_num, i2c_addr, RESULT_INTERRUPT_STATUS, 7, WAIT_FOR_NONZERO, VL53L0X_DEFAULT_INT_TIMEOUT_TICS)) {
		retValue |= 0x8000;		// Indicates no interrupt bits error
	}

	// assumptions: Linearity Corrective Gain is 1000 (default);
	// fractional ranging is not enabled
	i2c_readBytes(i2c_num, i2c_addr, (RESULT_RANGE_STATUS+10), &buf[0], 2);
	rangeValue = (buf[0] << 8) | buf[1];
	retValue = rangeValue | retValue;	// OR in any error bits

	i2c_readBytes(i2c_num, i2c_addr, RESULT_RANGE_STATUS, &rangeStatus, 1);

	vl53l0x_writeReg(i2c_num, i2c_addr, SYSTEM_INTERRUPT_CLEAR, 0x01);

	i2c_unlock();

	if (status != 0) {
		*status = rangeStatus;
	}
	return retValue;
}

/**
 * @name				vl53l0x_readRangeSingleMm
 * @brief				Starts and then gets a distance measurement in mm
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param i2c_addr		The 7-bit I2C chip address for the slave I2C device
 *
 * @retval				A 12-bit distance in mm where bits D14 and D15 if set indicate an error
 * @return	status		8-bit range status value that shows measurement errors. Use NULL to not use this.
 *
 * @note                Requires interrupts to be enabled so status of meas done can be read
 */
uint16_t vl53l0x_readRangeSingleMm(int i2c_num, uint8_t i2c_addr, uint8_t *status)
{
	uint16_t  retValue = 0;
	uint8_t   rangeStatus = 0;
	uint16_t  rangeValue = 0;
	i2c_lock();		// Get lock for I2C bus

	i2c_writeCmdTable(i2c_num, i2c_addr, vl53l0x_cmds_1);
	vl53l0x_writeReg(i2c_num, i2c_addr, 0x91, g_vl53l0x_stopVar);	// write stop_variable
	i2c_writeCmdTable(i2c_num, i2c_addr, vl53l0x_cmds_2);

	vl53l0x_writeReg(i2c_num, i2c_addr, SYSRANGE_START, 0x01);

	// "Wait until start bit has been cleared"
	if (i2c_waitOnRegStatus(i2c_num, i2c_addr, SYSRANGE_START, 1, WAIT_FOR_ZERO, VL53L0X_DEFAULT_INT_TIMEOUT_TICS) != 0) {
  		retValue |= 0x4000;		// Indicates meas did not start
	}
	i2c_unlock();

	rangeValue = vl53l0x_readRangeContinuousMm(i2c_num, i2c_addr, &rangeStatus);

	retValue = (rangeValue & 0xfff) | retValue;	// OR in any error bits in upper bits

	if (status != 0) {
		*status = rangeStatus;
	}

	return retValue;
}

