#ifndef _T3_I2CDEV_H_
#define _T3_I2CDEV_H_

// -----------------------------------------------------------------------------
// I2C interface implementation setting
// -----------------------------------------------------------------------------
#ifndef I2CDEV_IMPLEMENTATION
//#define I2CDEV_IMPLEMENTATION       I2CDEV_ARDUINO_WIRE
#define I2CDEV_IMPLEMENTATION       I2CDEV_BUILTIN_FASTWIRE
#endif

// comment this out if you are using a non-optimal IDE/implementation setting
// but want the compiler to shut up about it
#define I2CDEV_IMPLEMENTATION_WARNINGS

// -----------------------------------------------------------------------------
// I2C interface implementation options
// -----------------------------------------------------------------------------
#define I2CDEV_ARDUINO_WIRE         1 // Wire object from Arduino
#define I2CDEV_BUILTIN_NBWIRE       2 // Tweaked Wire object from Gene Knight's NBWire project
                                      // ^^^ NBWire implementation is still buggy w/some interrupts!
#define I2CDEV_BUILTIN_FASTWIRE     3 // FastWire object from Francesco Ferrara's project
#define I2CDEV_I2CMASTER_LIBRARY    4 // I2C object from DSSCircuits I2C-Master Library at https://github.com/DSSCircuits/I2C-Master-Library

// -----------------------------------------------------------------------------
// Arduino-style "Serial.print" debug constant (uncomment to enable)
// -----------------------------------------------------------------------------
//#define I2CDEV_SERIAL_DEBUG

#ifdef ARDUINO
    #include "Arduino.h"
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
    #endif
#endif



// 1000ms default read timeout (modify with "I2Cdev::readTimeout = [ms];")
#define I2CDEV_DEFAULT_READ_TIMEOUT     1000

class myI2Cdev {
    public:
        myI2Cdev();
        static void begin( uint32_t freq );

        static int8_t masterRead(uint16_t devAddr, uint8_t length, uint8_t *data, uint16_t timeout=myI2Cdev::readTimeout);
        //static int8_t readBit(uint8_t devAddr,uint8_t bitNum, uint8_t *data, uint16_t timeout=myI2Cdev::readTimeout);
        static int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout=myI2Cdev::readTimeout);
        static int8_t readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout=myI2Cdev::readTimeout);
        static int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout=myI2Cdev::readTimeout);
        static int8_t readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout=myI2Cdev::readTimeout);
        //static int8_t readByte(uint8_t devAddr, uint8_t *data, uint16_t timeout=myI2Cdev::readTimeout);
        static int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout=myI2Cdev::readTimeout);
        static int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout=myI2Cdev::readTimeout);
       // static int8_t readBytes(uint8_t devAddr, uint8_t length, uint8_t *data, uint16_t timeout=myI2Cdev::readTimeout);
        static int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout=myI2Cdev::readTimeout);
        static int8_t readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout=myI2Cdev::readTimeout);

        static bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
        static bool writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
        static bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
        static bool writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
        static bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
        static bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
        static bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
        static bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

        static uint16_t readTimeout;
};

#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE

#endif

#endif /* _I2CDEV_H_ */
