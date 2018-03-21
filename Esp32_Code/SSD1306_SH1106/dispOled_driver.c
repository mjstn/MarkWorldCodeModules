// Tiny I2C based tiny 128x64 OLED display from www.heltec.cn and others
//
// The SH1106 has a 132x64 pixel area.  The SSD1306 128x64 pixels can be thought of as
// being horizontally centered.   So column 0 on the SSD1306 needs to be column 2 on the SH1106
//
// Great data sheet notes:  http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html
// Basics for code below:   https://github.com/yanbe/ssd1306-esp-idf-i2c/blob/master/main/main.c
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

// Includes to support specific hardware APIs
#include "include/i2c_driver.h"

// Includes specific to the display driver and font
#include <dispOled_driver.h>
#include "include/font8x8_basic.h"

//  These commands are sent to initialize SSD1306.
//  There is not 'internal chip register' for this device and instead it uses a funky protocol.
//  A control byte is followed by a single byte or multiple bytes depending on the control byte.
//  The chip interprits these 'packets' as command(s) or data byte(s) based on the leading control byte
#define SSD1306_INIT_BYTE_COUNT 	5
static uint8_t ssd1306_init_bytes[SSD1306_INIT_BYTE_COUNT] = {
		OLED_CMD_SET_CHARGE_PUMP,	0x14,
		OLED_CMD_SET_SEGMENT_REMAP,
		OLED_CMD_SET_COM_SCAN_MODE,
		OLED_CMD_DISPLAY_ON
};

#define SH1106_INIT_BYTE_COUNT 	7
static uint8_t sh1106_init_bytes[SH1106_INIT_BYTE_COUNT] = {
		0x30,				// Charge pump default
		0x40,				// RAM display line of 0
		OLED_CMD_SET_SEGMENT_REMAP,
		OLED_CMD_SET_COM_SCAN_MODE,
		0x81, 0x80,         // Display contrast set to second byte
		OLED_CMD_DISPLAY_ON
};


/*
 * @name		display_setCursor
 * @brief 		Move cursor to a given horizontal column and line
 *
 * @param		dispCtx		Context for display that holds type and hardware interface info
 * @param		column		The pixel resolution column from 0 to 127
 * @param		line		The line number where 0 is top line
 *
 * @return		Returns 0 for ok or -1 for IO error
 *
 * @note        Caller is responsible for use of i2c_lock() and i2c_unlock() outside of this call
 */
int display_setCursor(dispCtx_t *dispCtx, int column, int line) {
	int retCode = 0;
	uint8_t curserSetup[6];

	switch (dispCtx->dispType) {
	case DISPLAY_TYPE_SSD1306:
		curserSetup[0] = OLED_CMD_SET_COLUMN_RANGE;
		curserSetup[1] = column;		// Start of printing from left seg as 0
		curserSetup[2] = DISPLAY_MAX_SEGMENT;		// last index of printing segments
		curserSetup[3] = OLED_CMD_SET_PAGE_RANGE;
		curserSetup[4] = line;									// We assume only one line written to at a time
		curserSetup[5] = line;

		// We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
		retCode = i2c_writeBytes(dispCtx->i2cBus, dispCtx->i2cAddr, OLED_CONTROL_BYTE_CMD_STREAM, &curserSetup[0], 6);
		break;

	case DISPLAY_TYPE_SH1106:
		// SH1106 has different addressing than SSD1306
		curserSetup[0] = 0xB0 | (line & 0xf);
		curserSetup[1] = 0x00 | ((column + SH1106_HORZ_OFFSET) & 0xf);	// Lower column address
		curserSetup[2] = 0x10 | ((column + SH1106_HORZ_OFFSET) >> 4);	// Upper column address

		// We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
		retCode = i2c_writeBytes(dispCtx->i2cBus, dispCtx->i2cAddr, OLED_CONTROL_BYTE_CMD_STREAM, &curserSetup[0], 3);
		break;
	default:
		break;
	}

	return retCode;
}

/*
 * @name		display_init
 * @brief 		Initialize the display
 *
 * @param		dispCtx		Context for display that holds type and hardware interface info
 * @param		dispType	Type of display. DISPLAY_TYPE_SSD1306 or DISPLAY_TYPE_SH1106
 * @param		i2cBus		I2C Bus interface number
 * @param		i2cAddr		7-bit I2C bus address
 *
 * @return		Returns 0 for ok or -1 for IO error
 *
 * @note        Caller is responsible for use of i2c_lock() and i2c_unlock() outside of this call
 */
int display_init(dispCtx_t *dispCtx, int dispType, int i2cBus, uint8_t i2cAddr)
{
    int retCode = 0;

    dispCtx->dispType = dispType;
    dispCtx->i2cBus = i2cBus;
    dispCtx->i2cAddr = i2cAddr;

    //Send all the commands to fully initialize the device.
    switch (dispCtx->dispType) {
    case DISPLAY_TYPE_SSD1306:
    	// We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
    	retCode = i2c_writeBytes(i2cBus, i2cAddr, OLED_CONTROL_BYTE_CMD_STREAM,
    			&ssd1306_init_bytes[0], SSD1306_INIT_BYTE_COUNT);
    	break;
    case DISPLAY_TYPE_SH1106:
    	// We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
    	retCode = i2c_writeBytes(i2cBus, i2cAddr, OLED_CONTROL_BYTE_CMD_STREAM,
    	    	&sh1106_init_bytes[0], SH1106_INIT_BYTE_COUNT);
    	break;
    default:
    	retCode = -9;
    	break;
    }

	return retCode;
}

/*
 * @name		display_clearDisplay
 * @brief 		Clear the display
 *
 * @param		dispCtx		Context for display that holds type and hardware interface info
 *
 * @return		Returns 0 for ok or -1 for IO error
 *
 * @note        Caller is responsible for use of i2c_lock() and i2c_unlock() outside of this call
 */
int display_clearDisplay(dispCtx_t *dispCtx) {
	int retCode = 0;
	i2c_cmd_handle_t cmd;
	uint8_t curserSetup[6];

	uint8_t zero[128];
	for (uint8_t idx = 0; idx < 128; idx++) {
		zero[idx] = 0;      // All 0 is blank vertical segments of 8 bits all across the row
	}
	for (uint8_t line = 0; line <= DISPLAY_MAX_LINE; line++) {

		retCode = display_setCursor(dispCtx, 0, line);
		if (retCode != 0) {
			return retCode;
		}

		// Clear one line
		retCode = i2c_writeBytes(dispCtx->i2cBus, dispCtx->i2cAddr, OLED_CONTROL_BYTE_DATA_STREAM, &zero[0], (DISPLAY_MAX_SEGMENT+1));
		if (retCode != 0) {
			return retCode;
		}
	}

	return retCode;
}

/*
 * @name		display_writeText
 * @brief 		Send an ASCII text string to the display on a single line. No line feed support.
 *
 * @param		dispCtx		Context for display that holds type and hardware interface info
 * @param		line		Line number for start of the write. 0 is top line of display, 7 bottom
 * @param		segment		Where the text starts in terms of pixel count from the left being 0
 * @param		center      Set to non-zero to center the text on the given line
 * @param		textStr     The text string to be printed
 *
 * @return		Returns 0 for ok or -1 for IO error
 *
 * @note        Caller is responsible for use of i2c_lock() and i2c_unlock() outside of this call
 */
int display_writeText(dispCtx_t *dispCtx, uint8_t line, uint8_t segment, uint8_t center, char *textStr) {
	int retCode = 0;
	char *text = textStr;
	uint8_t text_len = strlen(text);
	uint8_t curserSetup[6];
	uint8_t dispData[(DISPLAY_MAX_SEGMENT+20)];
	int dispDataIdx = 0;

	if (line > 7) {
		return -1;		// out of range line
	}
	if ((segment + (text_len * 8)) > DISPLAY_MAX_SEGMENT) {
		return -2;		// out of range starting segment to end of the print with 8x8 font
	}
	int startSegment = segment;          // MOD: When MAX_SEGMENT was 0x7F this was just    segment & MAX_SEGMENT
	if (segment > (DISPLAY_MAX_SEGMENT-1)) {
		segment = DISPLAY_MAX_SEGMENT-1; // Cap this to max end of line segment
	}
	if (center != 0) {
		startSegment = (DISPLAY_MAX_SEGMENT - (text_len * DISPLAY_CHAR_WIDTH)) / 2;
	}

	retCode = display_setCursor(dispCtx, startSegment, line);
	if (retCode != 0) {
		return retCode;
	}

	// Form pixels to send as data by lookup in font table
	for (uint8_t i = 0; i < text_len; i++) {
		// For each column of pixels for this char send a data byte which is one vertical column of pixels
		for (uint8_t charCol = 0; charCol < DISPLAY_CHAR_WIDTH; charCol++) {
			dispData[dispDataIdx++] = font8x8_basic_tr[(int)(text[i])][charCol];
		}
	}

	// Write the data to display
	retCode = i2c_writeBytes(dispCtx->i2cBus, dispCtx->i2cAddr, OLED_CONTROL_BYTE_DATA_STREAM, &dispData[0], dispDataIdx);

	return retCode;
}

