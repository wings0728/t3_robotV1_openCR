#ifndef _T3_SOFTI2C_H_
#define _T3_SOFTI2C_H_

#include "stm32f746xx.h"
#include "T3_softI2CAddress.h"

#define kI2C_DAC_LEFT   1
#define kI2C_DAC_RIGHT  2
#define kI2C_SENSOR     3

class SoftI2C
{
	public:
		SoftI2C();
		~SoftI2C();

    bool writeBuffer(uint8_t I2C_Type, uint8_t addr,
                     uint8_t reg, uint8_t len, uint8_t *data);
    bool readBuffer(uint8_t I2C_Type, uint8_t addr,
                    uint8_t reg, uint8_t len, uint8_t *data);
		bool start(uint8_t I2C_Type);
		void stop(uint8_t I2C_Type);
		void sendByte(uint8_t I2C_Type, uint8_t byte);
		uint8_t readByte(uint8_t I2C_Type);
		bool waitAck(uint8_t I2C_Type);
		void Ack(uint8_t I2C_Type);
		void NAck(uint8_t I2C_Type);
	private:
};
#endif
