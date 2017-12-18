#ifndef _T3_SOFTI2CAddress_H_
#define _T3_SOFTI2CAddress_H_

#define kI2C_DAC_SLAVE_ADDR      0x64 //DAC
#define kI2C_REVERSE0_SLAVE_ADDR 0x30 //Reverse 0
#define kI2C_REVERSE1_SLAVE_ADDR 0x40 //Reverse 1
#define kI2C_DISTANCE_SLAVE_ADDR 0x70 //Infrared
#define kI2C_ENCODER_SLAVE_ADDR  0x50 //Encoder
#define kI2C_REMOTE_SLAVE_ADDR   0x60 //Remote
#define kI2C_DAC_WRITE_ADDR      0x40 //DAC Register
//#define kI2C_TEST_DFRDUINO_ADDR  0x08 //DFRduino UNO Test

#define kI2C_REVERSE0_DATA_LENGTH		8
#define kI2C_REVERSE1_DATA_LENGTH		4
#define kI2C_DISTANCE_DATA_LENGTH		8
#define kI2C_ENCODER_DATA_LENGTH		9
#define kI2C_ENCODER_FACT_LENGTH		4 // leftEncoder, rightEncoder, leftRPM, rightRPM
#endif
