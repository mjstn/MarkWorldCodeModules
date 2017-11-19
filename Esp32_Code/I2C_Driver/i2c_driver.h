/*
 * I2C driver wrapper for Esp32 esp-idf dev environment
 *
 * These are used for many mark-toys.com projects on the Mark-Toys ESP32 Dev Board
 *
 */
#ifndef I2C_DRIVERS_H
#define I2C_DRIVERS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// Includes to support specific hardware APIs
#include "driver/gpio.h"
#include "driver/i2c.h"


/**
 * I2C Drivers For Esp32 IDF Library wrappers
 * 
*/

// I2C releated defines for the port itself and for devices we can operate
#define I2C_MASTER_NUM I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0   	/*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   	/*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ   	 40000     	/*!< I2C master clock frequency */

#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

// Some defines for a type of condition to satisfy some state of ready
#define WAIT_FOR_ZERO       0
#define WAIT_FOR_NONZERO    1


// This table can support the sending of 1 or more bytes starting at a base address.
// We use this to simplfy initialization which often takes many values to several registers
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} i2c_cmd_table_t;

extern void i2c_lock(void);
extern void i2c_unlock(void);
extern xSemaphoreHandle  g_i2c_lock;        // Lock for I2C bus as it runs in two threads

extern void i2c_master_init(int dataGpioPin, int clockGpioPin, int i2cFrequency);
extern void i2c_slave_init(int dataGpioPin, int clockGpioPin);
extern int i2c_writeBytes(i2c_port_t i2c_num, uint8_t chipI2cAddr, uint8_t regAddr, uint8_t *bufr, uint16_t numBytes);
extern int i2c_readBytes(i2c_port_t i2c_num, uint8_t chipI2cAddr, uint8_t regAddr, uint8_t *bufr, uint16_t numBytes);
extern int i2c_waitOnRegStatus(i2c_port_t i2c_num, uint8_t chipI2cAddr, uint8_t regAddr, uint8_t bitMask, int waitCondition, int timeoutTics);
extern void i2c_writeCmdTable(int i2c_num, uint8_t i2c_addr, i2c_cmd_table_t *cmdTable);

#endif // I2C_DRIVERS_H
