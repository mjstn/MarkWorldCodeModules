/**
 * I2C Drivers For Esp32 IDF Library wrappers in FreeRTOS environment
 * 
 * These are used for many mark-toys.com projects on the Mark-Toys ESP32 Dev Board
*/


#include "esp_log.h"

#include "include/i2c_driver.h"

/**
 * @name				i2c_lock
 * @brief				Locks the I2C bus to allow multi-thread I2C access
 *
 * @note				This can be a stub if I2C is just in one thread or a very system specific lock otherwise
 */
void i2c_lock(void)
{
	xSemaphoreTake(g_i2c_lock, portMAX_DELAY);		// Get lock for I2C bus
}

/**
 * @name				i2c_unlock
 * @brief				Unlocks the I2C bus to allow multi-thread I2C access
 *
 * @note				This can be a stub if I2C is just in one thread or a very system specific lock otherwise
 */
void i2c_unlock(void)
{
	xSemaphoreGive(g_i2c_lock);
}

/**
 * @name				i2c_writeBytes
 * @brief				Write one or more bytes to the slave device starting at a given chip register
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param i2c_addr		The 7-bit I2C chip address for the slave I2C device
 * @param regAddr		Internal register to set for the access.  Use 0xFF for no register to access (single byte device)
 * @param *bufr			Pointer for the 1st data byte to be written.
 * @param numBytes		Number of bytes to write.
 *
 * @retval				Zero indicates no error.
 */
int i2c_writeBytes(i2c_port_t i2c_num, uint8_t chipI2cAddr, uint8_t regAddr, uint8_t *bufr, uint16_t numBytes)
{
    int retCode = 0;
    int i;
    i2c_cmd_handle_t cmd;

    if (bufr == 0) {
		return -1;
	}

    // Setup the target device for a write to a register address by queuing up addr and the bytes
    cmd= i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chipI2cAddr << 1 | WRITE_BIT, ACK_CHECK_EN);

    if (regAddr != 0xff) { 		// Do address for internal register to be used
    	i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
    }

    // Now write as many bytes as was requested
    for (i=0 ; i < numBytes; i++) {
    	i2c_master_write_byte(cmd, bufr[i], ACK_CHECK_EN);
    }
    i2c_master_stop(cmd);

    int ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return ret;
    }

    return retCode;
}

/**
 * @name				i2c_readBytes
 * @brief				Addresses chip and internal register then Read one or more bytes from the slave device
 * 						After all bytes read the bus is released.
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param i2c_addr		The 7-bit I2C chip address for the slave I2C device
 * @param regAddr		Internal register to set for the access.
 * @param *bufr			Pointer for the 1st data byte to be read.
 * @param numBytes		Number of bytes to be read.  USER IS RESPONSIBLE FOR SUFFICIENT BUFFER!
 *
 * @retval				Zero indicates no error.
 */
int i2c_readBytes(i2c_port_t i2c_num, uint8_t chipI2cAddr, uint8_t regAddr, uint8_t *bufr, uint16_t numBytes)
{
    int retCode = ESP_OK;
    i2c_cmd_handle_t cmd;

    if (numBytes == 0) {
    	return retCode;    // Hay, if the guy asks for 0 bytes, we technically have no error!  DOOH!
    }

    if (bufr == 0) {
		return -1;
	}

    // Setup the target device for an access to a register address by queuing up addr and the bytes
	cmd= i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chipI2cAddr << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);  // Do the address reg selection
	i2c_cmd_link_delete(cmd);
	if (retCode == ESP_FAIL) {
		return retCode;
	}
	vTaskDelay(2 / portTICK_RATE_MS);

	// Now setup for the read address mode
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, chipI2cAddr << 1 | READ_BIT, ACK_CHECK_EN);

	// Now read as many bytes as was requested
	int  ackVal = ACK_VAL;
	int  i;
	for (i=0 ; i < numBytes; i++) {
		if (i == (numBytes - 1)) {
			ackVal = NACK_VAL;
		}
		i2c_master_read_byte(cmd, &bufr[i], ackVal);
	}
	i2c_master_stop(cmd);

	retCode = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (retCode == ESP_FAIL) {
		return ESP_FAIL;
	}
	return ESP_OK;
}

/**
 * @name				i2c_waitOnRegStatus
 * @brief				Read a register until the masked reg value matches the condition
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param i2c_addr		The 7-bit I2C chip address for the slave I2C device
 * @param regAddr		Internal register to set for the access.
 * @param bitMask		Value to mask bits read from the register if mask bits are 1
 * @param waitCondition	Condition for wait completion.  WAIT_FOR_NONZERO or WAIT_FOR_ZERO
 * @param timeoutTics   FreeRTOS tics to pass before we give up on the wait
 *
 * @retval				Zero indicates no error.  -1 is timeout
 */
int i2c_waitOnRegStatus(i2c_port_t i2c_num, uint8_t chipI2cAddr,
		uint8_t regAddr, uint8_t bitMask, int waitCondition, int timeoutTics)
{
	int retCode = 0;
	TickType_t startTics;
	TickType_t nowTics;
	uint8_t regVal;

	startTics = xTaskGetTickCount();
	while (1) {
		i2c_readBytes(i2c_num, chipI2cAddr, regAddr, &regVal, 1);
		if (waitCondition == WAIT_FOR_NONZERO) {
			if ((regVal & bitMask) != 0) {
				break;	// Go the bit(s) to be ready
			}
		} else if ((regVal & bitMask) == 0) {
			break;	// Go the bit(s) to be ready
		}

		vTaskDelay(1);		// Introduce 1 tic delay so we don't hog cpu

		nowTics = xTaskGetTickCount();
		if ((nowTics - startTics) > timeoutTics) {
			return -1;
		}
	}
	return retCode;
}


/**
 * @name				i2c_writeCmdTable
 * @brief				Write out a table of commands in the form of type i2c_cmd_table_t
 *
 * @param i2c_num		I2C peripheral e.g. I2C_NUM_1
 * @param i2c_addr		The 7-bit I2C chip address for the slave I2C device
 * @param cmdTable   	Pointer to table of commands in format of i2c_cmd_table_t
 */
void i2c_writeCmdTable(int i2c_num, uint8_t i2c_addr, i2c_cmd_table_t *cmdTable)
{
	int cmd = 0;
	uint8_t buf[8];

	while (cmdTable[cmd].databytes!=0xff) {
		buf[0] = cmdTable[cmd].data[0];
		// GET_PRT; printf("write cmd 0x%x to I2C addr 0x%x with data 0x%x\n", cmdTable[cmd].cmd, i2c_addr, buf[0]); EXIT_PRT;
		i2c_writeBytes(i2c_num, i2c_addr, cmdTable[cmd].cmd, &buf[0], 1);
		cmd++;
	}
}
