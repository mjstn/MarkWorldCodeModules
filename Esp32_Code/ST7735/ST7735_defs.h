/*
 * ST7735_defs.h
 *
 * Defines for usage of the ST7735 TFT display on the Mark-Toys Esp-32 based bot called EspressoBot.
 * The bot uses the Mark-Toys Esp32 Dev Board so pin numbers in this file are setup for that hardware
 *
 * The intent of these defines is to specify Esp32 resources and resources on the I2C bus
 *
 * Pins used for the hardware GPIO lines and hooks to use GPIO and graphics APIs are included
 *
 * Created Oct 2017 by Mark Johnston for Mark-Toys.com and is modified code from the site below
 * http://ccspicc.blogspot.com/2016/10/st7735-spi-tft-pic16f877a-pic-c.html
 */

#include <stdlib.h>
#include <stdint.h>

#ifndef ST7735_DEFS_H_
#define ST7735_H_

// ============================================================================================
// GPIO Pin Defines for Mark-Toys Esp32 DevBoard.

// First a few Esp32 lines that are fixed to specific pins
#define DAC_1_OUT_GPIO       25		  	// P3  pin 6.  This is fixed and if enabled is on GPIO25
#define DAC_2_OUT_GPIO       26		  	// P3  pin 7.  This is fixed and if enabled is on GPIO26

// GPIO lines for SPI driven devices such as for displays
#define SPI1_CSO_GPIO        18			// P12 pin 2.  SPI_CS0  line
#define SPI1_CLK_GPIO         5			// P12 pin 3.  SPI_CLK  line
#define SPI1_MOSI_GPIO       17			// P12 pin 4.  SPI_MOSI line
#define SPI1_MISO_GPIO       16			// P12 pin 5.  SPI_MISO line

// GPIO line defines for ST7735 TFT display control.  Must define TFT_DISPLAY_ENABLED to have these used
// THIS IS NOT YET PROVEN SO WE PREFER USE OF DISP_TASK as of 10/2017
#define TFT_GPIO_CS		SPI1_CSO_GPIO	// P12 pin 2.  SPI_CS0  line
#define TFT_GPIO_CLK	SPI1_CLK_GPIO	// P12 pin 3.  SPI_CLK  line
#define TFT_GPIO_DATA	SPI1_MOSI_GPIO	// P12 pin 4.  SPI_MOSI line
#define TFT_GPIO_DC		SPI1_MISO_GPIO	// P12 pin 5.  SPI_MISO line

#define TFT_GPIO_RST	DAC_2_OUT_GPIO	// We use DAC 2 line as digital ouput if TFT is used

// Set following enable to do display drive over SPI
#define DISP_DC_GPIO   DAC_1_OUT_GPIO	// P3  pin 6.  This is fixed and if enabled is on GPIO25
#define DISP_RST_GPIO  DAC_2_OUT_GPIO	// P3  pin 7.  This is fixed and if enabled is on GPIO26
#define DISP_BCKL_GPIO       15

// Some APIs where other modules can call these to get hardware set and readback
// USER SOURCE CODE MUST SUPPLY THESE HOOKS FOR USAGE OF THIS CODE
extern void system_gpioLineDirection(int gpioPin, int direction);
extern void system_gpioLineSet(int gpioPin, uint32_t state);
extern int  system_gpioLineRead(int gpioPin);
extern void system_delayMilliSeconds(int milliSeconds);

// ---------------  Start of include file perhaps called ST7735B_disp.h  -----------------------

// Display sizes in pixels.  These are used by lowest level and also graphics driver level
#define DISP_WIDTH				   128
#define DISP_HEIGHT				   160

// The poor mans PIC ST7735 driver did not have include file so here are the calls I use
extern void TFT_ST7735B_Initialize();
extern void TFT_BlackTab_Initialize();
extern void tft_disp_reset(int resetActive);
extern void fillScreen(uint16_t color);
extern void drawPixel(uint8_t x, uint8_t y, uint16_t color);
extern void drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
extern void drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
extern void fillRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
extern void drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
extern void fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
extern void drawChar(uint8_t x, uint8_t y, uint8_t c, uint16_t color, uint16_t bg,  uint8_t size);
extern void drawtext(uint8_t x, uint8_t y, char *_text, uint16_t color, uint16_t bg, uint8_t size);

// Color definitions
#define   ST7735_BLACK   0x0000
#define   ST7735_BLUE    0x001F
#define   ST7735_RED     0xF800
#define   ST7735_GREEN   0x07E0
#define   ST7735_CYAN    0x07FF
#define   ST7735_MAGENTA 0xF81F
#define   ST7735_YELLOW  0xFFE0
#define   ST7735_WHITE   0xFFFF


#endif /* ST7735_DEFS_H_ */
