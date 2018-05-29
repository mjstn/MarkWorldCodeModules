// @file   mpu6050_driver.h
// @brief  Drivers for MPU6050 Accellerometer/Gyro over I2C in the esp-sdk environment
//
// This is a port of Jeff Rowbergs https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
//
// This code put together to be used for Mark-Toys.com projects.  Use at your own risk.
// The code is considered to be a starting point and not a finished and polished codebase
//
// Environment Requirements:
// Requires the Mark-Toys.com i2c wrapper driver that uses the Esp32 SDK I2C calls

 
#ifndef MPU6050_DEFS_H_
#define MPU6050_DEFS_H_

// Print statements for debug
//#define MPU6050_DEBUG
#ifdef MPU6050_DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif

// Error returns
#define ERR_NO_ERROR				0
#define ERR_INVALID_INPUT_POINTER	1
#define ERR_INVALID_PARAMETER		2
#define ERR_I2C_WRITE_FAILED		3
#define ERR_I2C_READ_FAILED			4
#define ERR_INVALID_CHIP_ID			5

// MPU6040 Accellerometer and Gyro
#define MPU6050_I2C_BUS       		I2C1  // Bind to system bus required.   THIS IS UNIQUE TO A GIVEN SYSTEM HARDWARE

#define MPU6050_I2C_ADDRESS   		0x68  // Default address if ADDR pin is low

#define pgm_read_byte(x) (*(x))
#define pgm_read_word(x) (*(x))
#define pgm_read_float(x) (*(x))


typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} VectorInt16;

typedef struct {
	float x;
	float y;
	float z;
} VectorFloat;

typedef struct {
	float w;
	float x;
	float y;
	float z;
} Quaternion;

// MPU-6050 Registers and bitfields

// These values used in MPU6050_RA_CONFIG
#define MPU6050_DLPF_BITS           0x07  // bitfield mask for lowpass filter
#define MPU6050_DLPF_260HZ             0
#define MPU6050_DLPF_184HZ             1
#define MPU6050_DLPF_94HZ              2
#define MPU6050_DLPF_44HZ              3
#define MPU6050_DLPF_21HZ              4
#define MPU6050_DLPF_10HZ              5
#define MPU6050_DLPF_5HZ               6

// Values for MPU6050_RA_ACCEL_CONFIG
#define MPU6050_ACCEL_RANGE_2G         0
#define MPU6050_ACCEL_RANGE_4G         1
#define MPU6050_ACCEL_RANGE_8G         2
#define MPU6050_ACCEL_RANGE_16G        3
#define MPU6050_DATA_START          0x3B  // Start reg addr for sample readback

#define MPU6050_RA_INT_MST_STATUS  0x36  // Interrupt master status bits. Most cleared when this is read
#define MPU6050_RA_INT_PIN_CONFIG  0x37  // Configures int out pins and also can enable ref clock output pin
#define MPU6050_RA_INT_ENABLE      0x38  // Enables assorted interrupts
#define MPU6050_INT_EN_RAW_RDY      0x01  // Set to 1 on data ready interrupt
#define MPU6050_INT_EN_MASTER_INT   0x08  // Enables any configured int that feeds the master int source

#define MPU6050_RA_INT_STATUS      0x3A  // Interrupt status register.  Most status bits cleared once read.



// Register names according to the datasheet.
// According to the InvenSense document
// "MPU-6000 and MPU-6050 Register Map
// and Descriptions Revision 3.2", there are no registers
// at 0x02 ... 0x18, but according other information
// the registers in that unknown area are for gain
// and offsets.
//

#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18

#define MPU6050_RA_SMPLRT_DIV         0x19   // R/W
#define MPU6050_RA_CONFIG             0x1A   // R/W
#define MPU6050_RA_GYRO_CONFIG        0x1B   // R/W
#define MPU6050_RA_ACCEL_CONFIG       0x1C   // R/W
#define MPU6050_RA_FF_THR             0x1D   // R/W
#define MPU6050_RA_FF_DUR             0x1E   // R/W
#define MPU6050_RA_MOT_THR            0x1F   // R/W
#define MPU6050_RA_MOT_DUR            0x20   // R/W
#define MPU6050_RA_ZRMOT_THR          0x21   // R/W
#define MPU6050_RA_ZRMOT_DUR          0x22   // R/W
#define MPU6050_RA_FIFO_EN            0x23   // R/W
#define MPU6050_RA_I2C_MST_CTRL       0x24   // R/W
#define MPU6050_RA_I2C_SLV0_ADDR      0x25   // R/W
#define MPU6050_RA_I2C_SLV0_REG       0x26   // R/W
#define MPU6050_RA_I2C_SLV0_CTRL      0x27   // R/W
#define MPU6050_RA_I2C_SLV1_ADDR      0x28   // R/W
#define MPU6050_RA_I2C_SLV1_REG       0x29   // R/W
#define MPU6050_RA_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU6050_RA_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU6050_RA_I2C_SLV2_REG       0x2C   // R/W
#define MPU6050_RA_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU6050_RA_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU6050_RA_I2C_SLV3_REG       0x2F   // R/W
#define MPU6050_RA_I2C_SLV3_CTRL      0x30   // R/W
#define MPU6050_RA_I2C_SLV4_ADDR      0x31   // R/W
#define MPU6050_RA_I2C_SLV4_REG       0x32   // R/W
#define MPU6050_RA_I2C_SLV4_DO        0x33   // R/W
#define MPU6050_RA_I2C_SLV4_CTRL      0x34   // R/W
#define MPU6050_RA_I2C_SLV4_DI        0x35   // R
#define MPU6050_RA_I2C_MST_STATUS     0x36   // R
#define MPU6050_RA_INT_PIN_CFG        0x37   // R/W
#define MPU6050_RA_INT_ENABLE         0x38   // R/W
#define MPU6050_RA_INT_STATUS         0x3A   // R
#define MPU6050_RA_ACCEL_XOUT_H       0x3B   // R
#define MPU6050_RA_ACCEL_XOUT_L       0x3C   // R
#define MPU6050_RA_ACCEL_YOUT_H       0x3D   // R
#define MPU6050_RA_ACCEL_YOUT_L       0x3E   // R
#define MPU6050_RA_ACCEL_ZOUT_H       0x3F   // R
#define MPU6050_RA_ACCEL_ZOUT_L       0x40   // R
#define MPU6050_RA_TEMP_OUT_H         0x41   // R
#define MPU6050_RA_TEMP_OUT_L         0x42   // R
#define MPU6050_RA_GYRO_XOUT_H        0x43   // R
#define MPU6050_RA_GYRO_XOUT_L        0x44   // R
#define MPU6050_RA_GYRO_YOUT_H        0x45   // R
#define MPU6050_RA_GYRO_YOUT_L        0x46   // R
#define MPU6050_RA_GYRO_ZOUT_H        0x47   // R
#define MPU6050_RA_GYRO_ZOUT_L        0x48   // R
#define MPU6050_RA_EXT_SENS_DATA_00   0x49   // R
#define MPU6050_RA_EXT_SENS_DATA_01   0x4A   // R
#define MPU6050_RA_EXT_SENS_DATA_02   0x4B   // R
#define MPU6050_RA_EXT_SENS_DATA_03   0x4C   // R
#define MPU6050_RA_EXT_SENS_DATA_04   0x4D   // R
#define MPU6050_RA_EXT_SENS_DATA_05   0x4E   // R
#define MPU6050_RA_EXT_SENS_DATA_06   0x4F   // R
#define MPU6050_RA_EXT_SENS_DATA_07   0x50   // R
#define MPU6050_RA_EXT_SENS_DATA_08   0x51   // R
#define MPU6050_RA_EXT_SENS_DATA_09   0x52   // R
#define MPU6050_RA_EXT_SENS_DATA_10   0x53   // R
#define MPU6050_RA_EXT_SENS_DATA_11   0x54   // R
#define MPU6050_RA_EXT_SENS_DATA_12   0x55   // R
#define MPU6050_RA_EXT_SENS_DATA_13   0x56   // R
#define MPU6050_RA_EXT_SENS_DATA_14   0x57   // R
#define MPU6050_RA_EXT_SENS_DATA_15   0x58   // R
#define MPU6050_RA_EXT_SENS_DATA_16   0x59   // R
#define MPU6050_RA_EXT_SENS_DATA_17   0x5A   // R
#define MPU6050_RA_EXT_SENS_DATA_18   0x5B   // R
#define MPU6050_RA_EXT_SENS_DATA_19   0x5C   // R
#define MPU6050_RA_EXT_SENS_DATA_20   0x5D   // R
#define MPU6050_RA_EXT_SENS_DATA_21   0x5E   // R
#define MPU6050_RA_EXT_SENS_DATA_22   0x5F   // R
#define MPU6050_RA_EXT_SENS_DATA_23   0x60   // R
#define MPU6050_RA_MOT_DETECT_STATUS  0x61   // R
#define MPU6050_RA_I2C_SLV0_DO        0x63   // R/W
#define MPU6050_RA_I2C_SLV1_DO        0x64   // R/W
#define MPU6050_RA_I2C_SLV2_DO        0x65   // R/W
#define MPU6050_RA_I2C_SLV3_DO        0x66   // R/W
#define MPU6050_RA_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU6050_RA_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU6050_RA_MOT_DETECT_CTRL    0x69   // R/W
#define MPU6050_RA_USER_CTRL          0x6A   // R/W
#define MPU6050_RA_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_RA_PWR_MGMT_2         0x6C   // R/W
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH        0x72   // R/W
#define MPU6050_RA_FIFO_COUNTL        0x73   // R/W
#define MPU6050_RA_FIFO_R_W           0x74   // R/W
#define MPU6050_RA_WHO_AM_I       	   0x75   // R

#define MPU6050_WHO_AM_I_VALUE      0x68  // Value that comes back from who am i register

// Bit names and values used in bitfields
#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous

#define MPU6050_TC_PWR_MODE_BIT     7
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_BIT  0

#define MPU6050_VDDIO_LEVEL_VLOGIC  0
#define MPU6050_VDDIO_LEVEL_VDD     1

#define MPU6050_CFG_EXT_SYNC_SET_BIT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACONFIG_XA_ST_BIT           7
#define MPU6050_ACONFIG_YA_ST_BIT           6
#define MPU6050_ACONFIG_ZA_ST_BIT           5
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DHPF_RESET          0x00
#define MPU6050_DHPF_5              0x01
#define MPU6050_DHPF_2P5            0x02
#define MPU6050_DHPF_1P25           0x03
#define MPU6050_DHPF_0P63           0x04
#define MPU6050_DHPF_HOLD           0x07

#define MPU6050_TEMP_FIFO_EN_BIT    7
#define MPU6050_XG_FIFO_EN_BIT      6
#define MPU6050_YG_FIFO_EN_BIT      5
#define MPU6050_ZG_FIFO_EN_BIT      4
#define MPU6050_ACCEL_FIFO_EN_BIT   3
#define MPU6050_SLV2_FIFO_EN_BIT    2
#define MPU6050_SLV1_FIFO_EN_BIT    1
#define MPU6050_SLV0_FIFO_EN_BIT    0

#define MPU6050_MULT_MST_EN_BIT     7
#define MPU6050_WAIT_FOR_ES_BIT     6
#define MPU6050_SLV_3_FIFO_EN_BIT   5
#define MPU6050_I2C_MST_P_NSR_BIT   4
#define MPU6050_I2C_MST_CLK_BIT     3
#define MPU6050_I2C_MST_CLK_LENGTH  4

#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF

#define MPU6050_I2C_SLV_RW_BIT      7
#define MPU6050_I2C_SLV_ADDR_BIT    6
#define MPU6050_I2C_SLV_ADDR_LENGTH 7
#define MPU6050_I2C_SLV_EN_BIT      7
#define MPU6050_I2C_SLV_BYTE_SW_BIT 6
#define MPU6050_I2C_SLV_REG_DIS_BIT 5
#define MPU6050_I2C_SLV_GRP_BIT     4
#define MPU6050_I2C_SLV_LEN_BIT     3
#define MPU6050_I2C_SLV_LEN_LENGTH  4

#define MPU6050_I2C_SLV4_RW_BIT         7
#define MPU6050_I2C_SLV4_ADDR_BIT       6
#define MPU6050_I2C_SLV4_ADDR_LENGTH    7
#define MPU6050_I2C_SLV4_EN_BIT         7
#define MPU6050_I2C_SLV4_INT_EN_BIT     6
#define MPU6050_I2C_SLV4_REG_DIS_BIT    5
#define MPU6050_I2C_SLV4_MST_DLY_BIT    4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6050_MST_PASS_THROUGH_BIT    7
#define MPU6050_MST_I2C_SLV4_DONE_BIT   6
#define MPU6050_MST_I2C_LOST_ARB_BIT    5
#define MPU6050_MST_I2C_SLV4_NACK_BIT   4
#define MPU6050_MST_I2C_SLV3_NACK_BIT   3
#define MPU6050_MST_I2C_SLV2_NACK_BIT   2
#define MPU6050_MST_I2C_SLV1_NACK_BIT   1
#define MPU6050_MST_I2C_SLV0_NACK_BIT   0

#define MPU6050_INTCFG_INT_LEVEL_BIT        7
#define MPU6050_INTCFG_INT_OPEN_BIT         6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6050_INTCFG_CLKOUT_EN_BIT        0

#define MPU6050_INTMODE_ACTIVEHIGH  0x00
#define MPU6050_INTMODE_ACTIVELOW   0x01

#define MPU6050_INTDRV_PUSHPULL     0x00
#define MPU6050_INTDRV_OPENDRAIN    0x01

#define MPU6050_INTLATCH_50USPULSE  0x00
#define MPU6050_INTLATCH_WAITCLEAR  0x01

#define MPU6050_INTCLEAR_STATUSREAD 0x00
#define MPU6050_INTCLEAR_ANYREAD    0x01

#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_MOTION_MOT_XNEG_BIT     7
#define MPU6050_MOTION_MOT_XPOS_BIT     6
#define MPU6050_MOTION_MOT_YNEG_BIT     5
#define MPU6050_MOTION_MOT_YPOS_BIT     4
#define MPU6050_MOTION_MOT_ZNEG_BIT     3
#define MPU6050_MOTION_MOT_ZPOS_BIT     2
#define MPU6050_MOTION_MOT_ZRMOT_BIT    0

#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6050_PATHRESET_GYRO_RESET_BIT    2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6050_PATHRESET_TEMP_RESET_BIT    0

#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6050_DETECT_FF_COUNT_BIT             3
#define MPU6050_DETECT_FF_COUNT_LENGTH          2
#define MPU6050_DETECT_MOT_COUNT_BIT            1
#define MPU6050_DETECT_MOT_COUNT_LENGTH         2

#define MPU6050_DETECT_DECREMENT_RESET  0x0
#define MPU6050_DETECT_DECREMENT_1      0x1
#define MPU6050_DETECT_DECREMENT_2      0x2
#define MPU6050_DETECT_DECREMENT_4      0x3

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_BIT            5
#define MPU6050_PWR2_STBY_YA_BIT            4
#define MPU6050_PWR2_STBY_ZA_BIT            3
#define MPU6050_PWR2_STBY_XG_BIT            2
#define MPU6050_PWR2_STBY_YG_BIT            1
#define MPU6050_PWR2_STBY_ZG_BIT            0

#define MPU6050_WAKE_FREQ_1P25      0x0
#define MPU6050_WAKE_FREQ_2P5       0x1
#define MPU6050_WAKE_FREQ_5         0x2
#define MPU6050_WAKE_FREQ_10        0x3

#define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6050_BANKSEL_MEM_SEL_BIT         4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6050_WHO_AM_I_BIT        6
#define MPU6050_WHO_AM_I_LENGTH     6

#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16



// Setup function APIs.  Please see source code for usage in header by each api call

extern void MPU6050_reset(int i2c_num, uint8_t chipAddr);
extern void MPU6050_initialize(int i2c_num, uint8_t chipAddr);

extern  int MPU60x0_checkChipId(int i2c_num, uint8_t chipAddr);
#define MPU6050_checkChipId	MPU60x0_checkChipId
#define MPU6000_checkChipId	MPU60x0_checkChipId

extern  int MPU60x0_readBytes(int i2c_num, uint8_t chipAddr, int8_t regAddr, uint8_t *bufr, uint16_t numBytes);
#define MPU6050_readBytes	MPU60x0_readBytes
#define MPU6000_readBytes	MPU60x0_readBytes

extern  int MPU60x0_writeBytes(int i2c_num, uint8_t chipAddr, int8_t regAddr, uint8_t *bufr, uint16_t numBytes);
#define MPU6050_writeBytes	MPU60x0_readBytes
#define MPU6000_writeBytes	MPU60x0_readBytes

#define MPU6050_DMP_PACKET_SIZE  42		// Sets packet size used to talk to onboard 6050 DMP processor

extern uint8_t MPU6050_getFullScaleAccelRange(int i2c_num, uint8_t chipAddr);
extern int MPU6050_getDMPEnabled(int i2c_num, uint8_t chipAddr);
extern void MPU6050_setDMPEnabled(int i2c_num, uint8_t chipAddr, int enabled);
extern uint16_t MPU6050_dmpGetFIFOPacketSize(void);
extern void MPU6050_resetDMP(int i2c_num, uint8_t chipAddr);
extern int MPU6050_getSleepEnabled(int i2c_num, uint8_t chipAddr);
extern void MPU6050_setSleepEnabled(int i2c_num, uint8_t chipAddr, int enabled);

extern void MPU6050_resetFIFO(int i2c_num, uint8_t chipAddr);
extern uint16_t MPU6050_getFIFOCount(int i2c_num, uint8_t chipAddr);
extern uint8_t MPU6050_getFIFOByte(int i2c_num, uint8_t chipAddr);
extern void MPU6050_getFIFOBytes(int i2c_num, uint8_t chipAddr, uint8_t *data, uint8_t length);

extern void MPU6050_setClockSource(int i2c_num, uint8_t chipAddr, uint8_t source);
extern uint8_t MPU6050_getFullScaleAccelRange(int i2c_num, uint8_t chipAddr);
extern void MPU6050_setFullScaleAccelRange(int i2c_num, uint8_t chipAddr, uint8_t range);
extern uint8_t MPU6050_getFullScaleGyroRange(int i2c_num, uint8_t chipAddr);
extern void MPU6050_setFullScaleGyroRange(int i2c_num, uint8_t chipAddr, uint8_t range);

extern uint8_t MPU6050_dmpInitialize(int i2c_num, uint8_t chipAddr);

extern uint8_t MPU6050_getOTPBankValid(int i2c_num, uint8_t chipAddr);
extern void MPU6050_setOTPBankValid(int i2c_num, uint8_t chipAddr, int enabled);

extern void    MPU6050_getAcceleration(int i2c_num, uint8_t chipAddr, int16_t *x, int16_t *y, int16_t *z);
extern uint8_t MPU6050_dmpGetGyroInt16(int16_t *data, const uint8_t* packet);
extern uint8_t MPU6050_dmpGetGyroInt32(int32_t *data, const uint8_t* packet);

extern uint8_t MPU6050_dmpGetQuaternion(Quaternion *q, const uint8_t* packet);
extern uint8_t MPU6050_dmpGetGravity(VectorFloat *v, Quaternion *q);
extern uint8_t MPU6050_dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
extern uint8_t MPU6050_dmpGetEuler(float *data, Quaternion *q);

extern int16_t MPU6050_getXAccelOffset(int i2c_num, uint8_t chipAddr);
extern void MPU6050_setXAccelOffset(int i2c_num, uint8_t chipAddr, int16_t offset);
extern int16_t MPU6050_getYAccelOffset(int i2c_num, uint8_t chipAddr);
extern void MPU6050_setYAccelOffset(int i2c_num, uint8_t chipAddr, int16_t offset);
extern int16_t MPU6050_getZAccelOffset(int i2c_num, uint8_t chipAddr);
extern void MPU6050_setZAccelOffset(int i2c_num, uint8_t chipAddr, int16_t offset);

extern int8_t MPU6050_getXGyroOffsetTC(int i2c_num, uint8_t chipAddr);
extern void MPU6050_setXGyroOffsetTC(int i2c_num, uint8_t chipAddr, int8_t offset);
extern int8_t MPU6050_getYGyroOffsetTC(int i2c_num, uint8_t chipAddr);
extern void MPU6050_setYGyroOffsetTC(int i2c_num, uint8_t chipAddr, int8_t offset);
extern int8_t MPU6050_getZGyroOffsetTC(int i2c_num, uint8_t chipAddr);
extern void MPU6050_setZGyroOffsetTC(int i2c_num, uint8_t chipAddr, int8_t offset);


extern uint8_t MPU6050_getIntStatus(int i2c_num, uint8_t chipAddr);
extern void MPU6050_setMemoryBankWithOptions(int i2c_num, uint8_t chipAddr, uint8_t bank, int prefetchEnabled, int userBank);
extern void MPU6050_setMemoryBank(int i2c_num, uint8_t chipAddr, uint8_t bank);
extern void MPU6050_setMemoryStartAddress(int i2c_num, uint8_t chipAddr, uint8_t address);
extern void MPU6050_readMemoryBlock(int i2c_num, uint8_t chipAddr, uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
extern int MPU6050_writeMemoryBlock(int i2c_num, uint8_t chipAddr, const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, int verify, int useProgMem);
extern int MPU6050_writeProgMemoryBlock(int i2c_num, uint8_t chipAddr, const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, int verify);
extern int MPU6050_writeDMPConfigurationSet(int i2c_num, uint8_t chipAddr, const uint8_t *data, uint16_t dataSize, int useProgMem);
extern int MPU6050_writeProgDMPConfigurationSet(int i2c_num, uint8_t chipAddr, const uint8_t *data, uint16_t dataSize);

#endif // MPU6050_DEFS_H_
