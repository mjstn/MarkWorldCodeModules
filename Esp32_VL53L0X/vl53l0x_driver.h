// ST Micro VL53L0X Simplified Driver for C Esp32 Environment Includes
//
// Included besides basic measurement is a call to configure addresses for the VL53L0X which defaults to 0x29
// so please use vl53l0x_initSensorI2cAddresses() when using multiple sensors.
//
// This is a minimal driver for polled usage on top of freeRTOS and uses my I2C drivers on top of ESP32 library
//
// WARNING: This driver code is an attempt for a C only driver of minimal complexity.
//          Based on the overly complex class in:  https://github.com/kriswiner/VL53L0X/blob/master/VL53L0X_t3.ino

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

#ifndef VL53L0X_DRIVER_H
#define VL53L0X_DRIVER_H

#define VL53L0X_WHO_AM_I          0x00C0   // should be 0xEE after a reset
#define VL53L0X_REF_REG_C0_VAL      0xEE   //  Chip ID readback to verify I2C is operational
#define VL53L0X_POWER_ON_ADDRESS  	0x29   //  Default after power-up Device address of VL53L0X
#define VL53L0X_XSHUT_I2C_PCF8574   0x20   //  If non-zero this is 7-bit PCF8574 addr over I2C for the XSHUT pin lines


#define VL53L0X_DEFAULT_INT_TIMEOUT_TICS   20

// Subset of registers to make some of this a bit clearer (this is one frigin complex init for a chip frankly)
#define SYSRANGE_START                                  0x00
#define SYSTEM_SEQUENCE_CONFIG     					 	0x01
#define SYSTEM_INTERRUPT_CONFIG_GPIO					0x0A
#define SYSTEM_INTERRUPT_CLEAR							0x0B
#define RESULT_INTERRUPT_STATUS                         0x13
#define RESULT_RANGE_STATUS                             0x14
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 	0x44
#define MSRC_CONFIG_CONTROL         					0x60
#define GPIO_HV_MUX_ACTIVE_HIGH							0x84
#define I2C_SLAVE_DEVICE_ADDRESS						0x8A

// Externs for VL53L0X driver for header file we should make for this driver
extern int  vl53l0x_writeReg(uint8_t i2c_num, uint8_t i2c_addr, uint8_t reg, uint8_t value);
extern int  vl53l0x_read_chipid(int i2c_num, uint8_t i2c_addr, uint8_t *data);
extern int  vl53l0x_initI2cAddresses(int i2c_num, uint8_t firstI2cAddress, int *xshutGpioPins, int sensor_count);
extern bool vl53l0x_setSignalRateLimit(int i2c_num, uint8_t i2c_addr, float limit_Mcps);
extern int  vl53l0x_init(int i2c_num, uint8_t i2c_addr);
extern void vl53l0x_startContinuous(int i2c_num, uint8_t i2c_addr);
extern void vl53l0x_stopContinuous(int i2c_num, uint8_t i2c_addr);
extern uint16_t vl53l0x_readRangeContinuousMm(int i2c_num, uint8_t i2c_addr);
extern uint16_t vl53l0x_readRangeSingleMm(int i2c_num, uint8_t i2c_addr);
extern int vl53l0x_performSingleRefCalibration(int i2c_num, uint8_t i2c_addr, uint8_t vhv_init_byte);

#endif  // VL53L0X_DRIVER_H
