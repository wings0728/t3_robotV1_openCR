#include "T3_softI2C.h"
//#include <chip.h>
#include "Arduino.h"
//#include "delay.h"
#include "stm32f746xx.h"
//#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_gpio.h"
#include "variant.h"
#define kHIGH   1
#define kLOW    0

//0表示写
#define kI2C_directionTransmitter   ((uint8_t)0x00)
//１表示读
#define kI2C_directionReceiver      ((uint8_t)0x01)

/**DAC_LEFT**/
/*SCL - PD2, SDA - PE3*/
#define kI2C_DAC_LEFT_SCL_H    digitalWrite(BDPIN_GPIO_4, kHIGH)//GPIOB->BSRR = GPIO_PIN_2//digitalWrite(BDPIN_GPIO_4, kHIGH)
#define kI2C_DAC_LEFT_SCL_L    digitalWrite(BDPIN_GPIO_4, kLOW)//GPIOB->BRR = GPIO_PIN_2//digitalWrite(BDPIN_GPIO_4, kLOW)

#define kI2C_DAC_LEFT_SDA_H    digitalWrite(BDPIN_GPIO_5, kHIGH)//GPIOE->BSRR = GPIO_PIN_3//digitalWrite(BDPIN_GPIO_5, kHIGH)
#define kI2C_DAC_LEFT_SDA_L    digitalWrite(BDPIN_GPIO_5, kLOW)//GPIOE->BRR = GPIO_PIN_3//digitalWrite(BDPIN_GPIO_5, kLOW)

#define kI2C_DAC_LEFT_SCL_READ //GPIOD->BSRR = GPIO_PIN_2//digitalRead(BDPIN_GPIO_4)
#define kI2C_DAC_LEFT_SDA_READ digitalRead(BDPIN_GPIO_5)//GPIOE->IDR  & GPIO_PIN_3 //digitalRead(BDPIN_GPIO_5)

#define kI2C_DAC_LEFT_SDA_IN   pinMode(BDPIN_GPIO_5, INPUT);

#define kI2C_DAC_LEFT_SDA_OUT  pinMode(BDPIN_GPIO_5, OUTPUT_OPEN);
/*************/
/**DAC_RIGHT**/
/*SCL - PG2, SDA - PE10*/
#define kI2C_DAC_RIGHT_SCL_H   digitalWrite(BDPIN_GPIO_6, kHIGH)
#define kI2C_DAC_RIGHT_SCL_L   digitalWrite(BDPIN_GPIO_6, kLOW)

#define kI2C_DAC_RIGHT_SDA_H   digitalWrite(BDPIN_GPIO_7, kHIGH)
#define kI2C_DAC_RIGHT_SDA_L   digitalWrite(BDPIN_GPIO_7, kLOW)

#define kI2C_DAC_RIGHT_SCL_READ   digitalRead(BDPIN_GPIO_6)
#define kI2C_DAC_RIGHT_SDA_READ   digitalRead(BDPIN_GPIO_7)
#define kI2C_DAC_RIGHT_SDA_IN   pinMode(BDPIN_GPIO_7, OUTPUT_OPEN);
#define kI2C_DAC_RIGHT_SDA_OUT  pinMode(BDPIN_GPIO_7, OUTPUT);
/*************/
/**SENSOR**/
/*SCL - PB10, SDA - PB11*/
#define kI2C_SENSOR_SCL_H      digitalWrite(BDPIN_GPIO_1, kHIGH)
#define kI2C_SENSOR_SCL_L      digitalWrite(BDPIN_GPIO_1, kLOW)

#define kI2C_SENSOR_SDA_H      digitalWrite(BDPIN_GPIO_2, kHIGH)
#define kI2C_SENSOR_SDA_L      digitalWrite(BDPIN_GPIO_2, kLOW)

#define kI2C_SENSOR_SCL_READ   digitalRead(BDPIN_GPIO_1)
#define kI2C_SENSOR_SDA_READ   digitalRead(BDPIN_GPIO_2)

#define kI2C_SENSOR_SDA_IN   pinMode(BDPIN_GPIO_2, OUTPUT_OPEN);
#define kI2C_SENSOR_SDA_OUT  pinMode(BDPIN_GPIO_2, OUTPUT);
/*************/
void SoftI2CDelay()
{
  uint32_t I2CStep;
  while(I2CStep--);
}

SoftI2C::SoftI2C()
{
}
SoftI2C::~SoftI2C()
{
}
bool SoftI2C::start(uint8_t I2C_Type)
{
	if(kI2C_DAC_LEFT == I2C_Type)
	{
	 kI2C_DAC_LEFT_SDA_OUT;
	  kI2C_DAC_LEFT_SDA_H;
	  kI2C_DAC_LEFT_SCL_H;
	  delay_us(2);
//    Serial.println(kI2C_DAC_LEFT_SDA_READ);
	  if(!kI2C_DAC_LEFT_SDA_READ)
	  {
      #ifdef DEBUG
      Serial.println("start false!");
	  	#endif
	  	return false;
	  }
	  kI2C_DAC_LEFT_SDA_L;
	  delay_us(2);
	  if(kI2C_DAC_LEFT_SDA_READ)
	  {
      #ifdef DEBUG
      Serial.println("start false!");
	  	#endif
	  	return false;
	  }
	  kI2C_DAC_LEFT_SCL_L;
//	  delay_us(2);
	  return true;
	}
	else if(kI2C_DAC_RIGHT == I2C_Type)
	{
    kI2C_DAC_RIGHT_SDA_OUT;
		kI2C_DAC_RIGHT_SDA_H;
		kI2C_DAC_RIGHT_SCL_H;
		delay_us(2);
		if(!kI2C_DAC_RIGHT_SDA_READ)
		{
		  return false;
		}
		kI2C_DAC_RIGHT_SDA_L;
		delay_us(2);
		if(kI2C_DAC_RIGHT_SDA_READ)
		{
		  return false;
		}
		kI2C_DAC_RIGHT_SCL_L;
		delay_us(2);
		return true;
	}
	else if(kI2C_SENSOR == I2C_Type)
	{
    kI2C_SENSOR_SDA_OUT;
		kI2C_SENSOR_SDA_H;
		kI2C_SENSOR_SCL_H;
		delay_us(2);
		if(!kI2C_SENSOR_SDA_READ)
		{
		  return false;
		}
		kI2C_SENSOR_SDA_L;
		delay_us(2);
		if(kI2C_SENSOR_SDA_READ)
		{
		  return false;
		}
		kI2C_SENSOR_SCL_L;
		delay_us(2);
		return true;
	}
}
void SoftI2C::stop(uint8_t I2C_Type)
{
	if(kI2C_DAC_LEFT == I2C_Type)
	{
    kI2C_DAC_LEFT_SDA_OUT;
		kI2C_DAC_LEFT_SCL_L;
//		delay_us(2);
		kI2C_DAC_LEFT_SDA_L;
//		delay_us(2);
		kI2C_DAC_LEFT_SCL_H;
		delay_us(2);
		kI2C_DAC_LEFT_SDA_H;
		delay_us(2);
	 }
	 else if(kI2C_DAC_RIGHT == I2C_Type)
	 {
    kI2C_DAC_RIGHT_SDA_OUT;
		kI2C_DAC_RIGHT_SCL_L;
		delay_us(2);
		kI2C_DAC_RIGHT_SDA_L;
		delay_us(2);
		kI2C_DAC_RIGHT_SCL_H;
		delay_us(2);
		kI2C_DAC_RIGHT_SDA_H;
		delay_us(2);
	 }
	 else if(kI2C_SENSOR == I2C_Type)
	 {
    kI2C_SENSOR_SDA_OUT;
		kI2C_SENSOR_SCL_L;
		delay_us(2);
		kI2C_SENSOR_SDA_L;
		delay_us(2);
		kI2C_SENSOR_SCL_H;
		delay_us(2);
		kI2C_SENSOR_SDA_H;
		delay_us(2);
	  }
}
void SoftI2C::sendByte(uint8_t I2C_Type, uint8_t byte)
{
	uint8_t idx = 8;
	if(kI2C_DAC_LEFT == I2C_Type)
	{
    kI2C_DAC_LEFT_SDA_OUT;
    kI2C_DAC_LEFT_SCL_L;
 //   delay_us(2);
		while(idx--)
		{
			if(byte & 0x80)
		  {
				kI2C_DAC_LEFT_SDA_H;
		  }
		  else
		  {
				kI2C_DAC_LEFT_SDA_L;
		  }
		   	byte <<= 1;
		   	delay_us(4);
		   	kI2C_DAC_LEFT_SCL_H;
		   	delay_us(2);
        kI2C_DAC_LEFT_SCL_L;
        delay_us(2);
		 }
		 kI2C_DAC_LEFT_SCL_L;
	}
	else if(kI2C_DAC_RIGHT == I2C_Type)
	{
    kI2C_DAC_RIGHT_SDA_OUT;
		while(idx--)
		{
			kI2C_DAC_RIGHT_SCL_L;
		    delay_us(2);
			if(byte & 0x80)
		   	{
				kI2C_DAC_RIGHT_SDA_H;
		   	}
		   	else
		   	{
				kI2C_DAC_RIGHT_SDA_L;
		   	}
		   	byte <<= 1;
		   	delay_us(2);
		   	kI2C_DAC_RIGHT_SCL_H;
		   	delay_us(2);
		 }
		 kI2C_DAC_RIGHT_SCL_L;
	}
	else if(kI2C_SENSOR == I2C_Type)
	{
    kI2C_SENSOR_SDA_OUT;
		while(idx--)
		{
			kI2C_SENSOR_SCL_L;
		    delay_us(2);
			if(byte & 0x80)
		   	{
				kI2C_SENSOR_SDA_H;
		   	}
		   	else
		   	{
				kI2C_SENSOR_SDA_L;
		   	}
		   	byte <<= 1;
		   	delay_us(2);
		   	kI2C_SENSOR_SCL_H;
		   	delay_us(2);
		 }
		 kI2C_SENSOR_SCL_L;
	}
}
uint8_t SoftI2C::readByte(uint8_t I2C_Type)
{
    uint8_t idx = 8;
    uint8_t byte = 0;
	if(kI2C_DAC_LEFT == I2C_Type)
	{
    kI2C_DAC_LEFT_SDA_IN;
//		kI2C_DAC_LEFT_SDA_H;
		while (idx--)
		{
			byte <<= 1;
		  kI2C_DAC_LEFT_SCL_L;
	    delay_us(2);
			kI2C_DAC_LEFT_SCL_H;
	    delay_us(2);
	    if(kI2C_DAC_LEFT_SDA_READ)
			{
				byte |= 0x01;
			}
		}
		kI2C_DAC_LEFT_SCL_L;
	}
	else if(kI2C_DAC_RIGHT == I2C_Type)
	{
		kI2C_DAC_RIGHT_SDA_H;
		while (idx--)
		{
			byte <<= 1;
			kI2C_DAC_RIGHT_SCL_L;
			delay_us(2);
			kI2C_DAC_RIGHT_SCL_H;
			delay_us(2);
			if(kI2C_DAC_RIGHT_SDA_READ)
			{
				byte |= 0x01;
			}
		}
		kI2C_DAC_RIGHT_SCL_L;
	}
	else if(kI2C_SENSOR == I2C_Type)
	{
		kI2C_SENSOR_SDA_H;
		while (idx--)
		{
			byte <<= 1;
			kI2C_SENSOR_SCL_L;
			delay_us(2);
			kI2C_SENSOR_SCL_H;
			delay_us(2);
			if(kI2C_SENSOR_SDA_READ)
			{
				byte |= 0x01;
			}
		}
		kI2C_SENSOR_SCL_L;
	}
	return byte;
}
bool SoftI2C::waitAck(uint8_t I2C_Type)
{
	if(kI2C_DAC_LEFT == I2C_Type)
	{
    //kI2C_DAC_LEFT_SDA_IN;
    kI2C_DAC_LEFT_SCL_H;
    //kI2C_DAC_LEFT_SDA_OUT;
//		kI2C_DAC_LEFT_SCL_L;
//		delay_us(2);
		//kI2C_DAC_LEFT_SDA_H;
//    Serial.println("SDA first");
//    Serial.println(kI2C_DAC_LEFT_SDA_READ);
//		delay_us(2);
		//kI2C_DAC_LEFT_SCL_H;
//		delay_us(2);

    delay_us(2);
//	Serial.println(kI2C_DAC_LEFT_SDA_READ);
		if(kI2C_DAC_LEFT_SDA_READ)
		{
      #ifdef DEBUG
      Serial.println("sda read false");
			#endif
			kI2C_DAC_LEFT_SCL_L;
      delay_us(2);
	  	return false;
		}
		kI2C_DAC_LEFT_SCL_L;
    delay_us(2);
		return true;
  }
	else if(kI2C_DAC_RIGHT == I2C_Type)
	{
      kI2C_DAC_RIGHT_SDA_IN;
		  kI2C_DAC_RIGHT_SCL_L;
		  delay_us(2);
		  kI2C_DAC_RIGHT_SDA_H;
		  delay_us(2);
		  kI2C_DAC_RIGHT_SCL_H;
		  delay_us(2);
		  if(kI2C_DAC_RIGHT_SDA_READ)
		  {
			  kI2C_DAC_RIGHT_SCL_L;
			  return false;
		  }
		  kI2C_DAC_RIGHT_SCL_L;
		  return true;
	 }
	 else if(kI2C_SENSOR == I2C_Type)
	 {
//
		  kI2C_SENSOR_SCL_L;
		  kI2C_SENSOR_SDA_H;
      kI2C_SENSOR_SDA_IN;
		  delay_us(2);
		  kI2C_SENSOR_SCL_H;
		  delay_us(2);
		  if(kI2C_SENSOR_SDA_READ)
		  {
			  kI2C_SENSOR_SCL_L;
			  return false;
		  }
		  kI2C_SENSOR_SCL_L;
		  return true;
	 }
}
void SoftI2C::Ack(uint8_t I2C_Type)
{
	if(kI2C_DAC_LEFT == I2C_Type)
	 {
	   kI2C_DAC_LEFT_SCL_L;
	 //  delay_us(2);
     kI2C_DAC_LEFT_SDA_OUT;
	   kI2C_DAC_LEFT_SDA_L;
	   delay_us(2);
	   kI2C_DAC_LEFT_SCL_H;
	   delay_us(2);
	   kI2C_DAC_LEFT_SCL_L;
	 //  delay_us(2);
	 }
	 else if(kI2C_DAC_RIGHT == I2C_Type)
	 {
	   kI2C_DAC_RIGHT_SCL_L;
	   delay_us(2);
     kI2C_DAC_RIGHT_SDA_OUT;
	   kI2C_DAC_RIGHT_SDA_L;
	   delay_us(2);
	   kI2C_DAC_RIGHT_SCL_H;
	   delay_us(2);
	   kI2C_DAC_RIGHT_SCL_L;
	   delay_us(2);
	 }
	 else if(kI2C_SENSOR == I2C_Type)
	 {
	   kI2C_SENSOR_SCL_L;
	   delay_us(2);
     kI2C_SENSOR_SDA_OUT;
	   kI2C_SENSOR_SDA_L;
	   delay_us(2);
	   kI2C_SENSOR_SCL_H;
	   delay_us(2);
	   kI2C_SENSOR_SCL_L;
	   delay_us(2);
	 }
}
void SoftI2C::NAck(uint8_t I2C_Type)
{
	if(kI2C_DAC_LEFT == I2C_Type)
	 {
	   kI2C_DAC_LEFT_SCL_L;
	//   delay_us(2);
     kI2C_DAC_LEFT_SDA_OUT;
	   kI2C_DAC_LEFT_SDA_H;
	   delay_us(2);
	   kI2C_DAC_LEFT_SCL_H;
	   delay_us(2);
	   kI2C_DAC_LEFT_SCL_L;
	//   delay_us(2);
	 }
	 else if(kI2C_DAC_RIGHT == I2C_Type)
	 {
	   kI2C_DAC_RIGHT_SCL_L;
	   delay_us(2);
     kI2C_DAC_RIGHT_SDA_OUT;
	   kI2C_DAC_RIGHT_SDA_H;
	   delay_us(2);
	   kI2C_DAC_RIGHT_SCL_H;
	   delay_us(2);
	   kI2C_DAC_RIGHT_SCL_L;
	   delay_us(2);
	 }
	 else if(kI2C_SENSOR == I2C_Type)
	 {
	   kI2C_SENSOR_SCL_L;
	   delay_us(2);
     kI2C_SENSOR_SDA_OUT;
	   kI2C_SENSOR_SDA_H;
	   delay_us(2);
	   kI2C_SENSOR_SCL_H;
	   delay_us(2);
	   kI2C_SENSOR_SCL_L;
	   delay_us(2);
	 }
}

bool SoftI2C::writeBuffer(uint8_t I2C_Type,
                          uint8_t addr,
                          uint8_t reg,
                          uint8_t len,
                          uint8_t *data)
{
 //   Serial.println("writeBuffer Start !");
    int idx;
    if(!start(I2C_Type))
    {
      #ifdef DEBUG
      Serial.println("START false !");
      #endif
      return false;
    }

   sendByte(I2C_Type, addr << 1 | kI2C_directionTransmitter);
//   Serial.println("sendByte Addr !");
//    delay_us(1);

    if (!waitAck(I2C_Type))
    {
      #ifdef DEBUG
      Serial.println("waitACK");
      #endif
      stop(I2C_Type);
      return false;
    }

    sendByte(I2C_Type, reg);
//    Serial.println("sendByte register");
    waitAck(I2C_Type);
//    delay_us(2);
   for (idx = 0; idx < len; idx++)
    {
      //Serial.println("sending!");
      sendByte(I2C_Type, data[idx]);
      if (!waitAck(I2C_Type))
      {
       #ifdef DEBUG
       Serial.println("waitAckData!");
       #endif
       stop(I2C_Type);
       return false;
     }
//      delay_us(2);
    }
    stop(I2C_Type);
 //   Serial.println("true!");
    return true;

}

bool SoftI2C::readBuffer(uint8_t I2C_Type,
                         uint8_t addr,
                         uint8_t reg,
                         uint8_t len,
                         uint8_t *data)
{
  int idx;
  if(!start(I2C_Type))
  {
    #ifdef DEBUG
    Serial.println("START false !");
    #endif
    return false;
  }
  //sendByte(I2C_Type, addr << 1 | kI2C_directionTransmitter);
  sendByte(I2C_Type, addr << 1 | kI2C_directionReceiver);
//  delay_us(5);

  waitAck(I2C_Type);
//  if (!waitAck(I2C_Type))
//  {
//    #ifdef DEBUG
//    Serial.println("waitACK");
//    #endif
//    stop(I2C_Type);
//    return false;
//  }
//  if(reg != 0)
//  {
//    sendByte(I2C_Type, reg);
//    waitAck(I2C_Type);
//  }
//
////  start(I2C_Type);
////  sendByte(I2C_Type, addr << 1 | kI2C_directionReceiver);
//  waitAck(I2C_Type);
//
//  for (idx = 0; idx < len; idx++)
//  {
//     data[idx] = readByte(I2C_Type);
//     if (!waitAck(I2C_Type))
//     {
//        Serial.println("waitAckData!");
//        stop(I2C_Type);
//        return false;
//     }
//  }
//  delay_us(4);
  stop(I2C_Type);
//  return true;
}

