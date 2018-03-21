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


#ifndef OLED_DISPLAY_H_
#define OLED_DISPLAY_H_


#define DISPLAY_TYPE_NONE       0
#define DISPLAY_TYPE_SSD1306	1
#define DISPLAY_TYPE_SH1106		2

#define DISPLAY_MAX_LINE		7

#define DISPLAY_CHAR_WIDTH  8
#define DISPLAY_CHAR_HEIGHT 8

// max segment index for (pixel) on right of display (seg 0 is 1st on left)
// This must be a power of 2 minus 1 for code to work properly
#define DISPLAY_MAX_SEGMENT  0x7f

#define SH1106_HORZ_OFFSET   2          // Set to 0 for use of SSD1106

// Display Context used to support different OLED display types
typedef struct {
    int dispType;
    int i2cBus;
    int i2cAddr;
} dispCtx_t;

// Following definitions are bollowed from 
// http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html

// SLA (0x3C) + WRITE_MODE (0x00) =  0x78 (0b01111000)
#define SSD1306_OLED_I2C_ADDRESS   0x3C

// Control byte
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST           0x81    // follow with 0x7F
#define OLED_CMD_DISPLAY_RAM            0xA4
#define OLED_CMD_DISPLAY_ALLON          0xA5
#define OLED_CMD_DISPLAY_NORMAL         0xA6
#define OLED_CMD_DISPLAY_INVERTED       0xA7
#define OLED_CMD_DISPLAY_OFF            0xAE
#define OLED_CMD_DISPLAY_ON             0xAF

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP      0xA1    
#define OLED_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE      0xC8    
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define OLED_CMD_NOP                    0xE3    // NOP

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14


// External Defs for api calls
extern  int display_setCursor(dispCtx_t *dispCtx, int column, int line);
extern  int display_init(dispCtx_t *dispCtx, int dispType, int i2cBus, uint8_t i2cAddr);
extern  int display_clearDisplay(dispCtx_t *dispCtx);
extern  int display_writeText(dispCtx_t *dispCtx, uint8_t line, uint8_t segment, uint8_t center, char *textStr);

#endif /* OLED_DISPLAY_H_ */

