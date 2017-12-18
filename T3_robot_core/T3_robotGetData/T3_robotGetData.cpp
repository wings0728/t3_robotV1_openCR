#include "T3_robotGetData.h"
#include "T3_I2Cdev.h"


myI2Cdev  T3HardwareI2Cdev;

GetData::GetData()
{

}

GetData::~GetData()
\{

}

void GetData::init(void)
{
	  T3HardwareI2Cdev.begin(100);
}

bool GetData::readEncoder(int32_t *encoder)
{
        bool getEncoderDataResult_ = false;

	uint8_t getEncoderDataRet_ = 0;
	uint8_t wheelDirection_ = 0;
	uint16_t leftEncoder_ = 0;
	uint16_t rightEncoder_ = 0;
	uint16_t leftRPM_ = 0;
	uint16_t rightRPM_ = 0;
	uint8_t encoderData_[kI2C_ENCODER_DATA_LENGTH] = {0};

	getEncoderDataRet_ = T3HardwareI2Cdev.masterRead(kI2C_ENCODER_SLAVE_ADDR, kI2C_ENCODER_DATA_LENGTH, encoderData_); // get the response length.
	if(kI2C_ENCODER_DATA_LENGTH == getEncoderDataRet_)
	{
		leftEncoder_ = ((encoderData_[0] << 8) | encoderData_[1] );
		rightEncoder_ = ((encoderData_[2] << 8) | encoderData_[3] );
		leftRPM_ = ((encoderData_[5] << 8) | encoderData_[6] ) / 4;
		rightRPM_ = ((encoderData_[7] << 8) | encoderData_[8] ) / 4;
		wheelDirection_ = encoderData_[4];
		if(0x0F == (wheelDirection_ >> 4))
		{
			encoder[0] = (int32_t)(leftEncoder_);
			encoder[2] = (int32_t)(leftRPM_);
		}
		else
		{
			encoder[0] = - (int32_t)(leftEncoder_);
			encoder[2] = - (int32_t)(leftRPM_);
		}
		if(0x0F == (wheelDirection_ & 0x0F)
		{
			encoder[1] = - (int32_t)(rightEncoder_);
			encoder[3] = - (int32_t)(rightRPM_);
		}
		else
		{
			encoder[1] = (int32_t)(rightEncoder_);
			encoder[3] = (int32_t)(rightRPM_);
		}
		getEncoderDataResult_ = true;
		return true;
	}
	else
	{
		return false;
	}
}
bool GetData::readInfrared(uint8_t *infraredValue)
{
	bool getInfraredDataResult_ = false;
	uint8_t idx_ = 0;
	uint8_t getInfraredDataRet_ = 0;
	uint8_t infraredData_[kI2C_DISTANCE_DATA_LENGTH] = {0};
	getInfraredDataRet_ = T3HardwareI2Cdev.masterRead(kI2C_DISTANCE_SLAVE_ADDR, kI2C_DISTANCE_DATA_LENGTH, infraredData_);
	if(kI2C_DISTANCE_DATA_LENGTH == getInfraredDataRet_)
	{
		for(idx_ = 0; idx_ < kI2C_DISTANCE_DATA_LENGTH; idx_ ++)
		{
			infraredValue[idx_] = infraredData_[idx_];
		}
		getInfraredDataResult_ = true;
		return true;
	}
	else
	{
		return false;
	}
}
/*
bool GetData::readSonar(uint8_t *reverse0Value, uint8_t *reverse1Value)
{
	bool getReverse0DataResult_ = false;
	bool getReverse1DataResult_ = false;
	uint8_t getReverse0DataRet_ = 0;
	uint8_t getReverse1DataRet_ = 0;
	getReverse0DataRet_ = T3HardwareI2Cdev.masterRead(kI2C_REVERSE0_SLAVE_ADDR, kI2C_REVERSE0_DATA_LENGTH, encoderData);
	getReverse1DataRet_ = T3HardwareI2Cdev.masterRead(kI2C_REVERSE01SLAVE_ADDR, kI2C_REVERSE1_DATA_LENGTH, encoderData);
	if(kI2C_REVERSE0_DATA_LENGTH == getReverse0DataRet_)
	{
		getReverse0DataResult_ = true;
	}

	if(kI2C_REVERSE1_DATA_LENGTH == getReverse1DataRet_)
	{
		getReverse1DataResult_ = true;
	}

	if((kI2C_REVERSE0_DATA_LENGTH == getReverse0DataRet_) && (kI2C_REVERSE1_DATA_LENGTH == getReverse1DataRet_))
	{
		return true;
	}
	else
	{
		return false;
	}
}
*/
