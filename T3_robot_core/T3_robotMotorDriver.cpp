#include "T3_robotMotorDriver.h"
#include "T3_I2Cdev.h"
#include <math.h>


#define kLEFT_BRAKE_PIN      BDPIN_GPIO_10 //PE13
#define kLEFT_DIRECTION_PIN  BDPIN_GPIO_8  //PE11
#define kRIGHT_BRAKE_PIN     BDPIN_GPIO_11 //PE14
#define kRIGHT_DIRECTION_PIN BDPIN_GPIO_9  //PE12
#define kLEFT_SCL			BDPIN_GPIO_4
#define kLEFT_SDA			BDPIN_GPIO_5
#define kRIGHT_SCL			BDPIN_GPIO_6
#define kRIGHT_SDA		BDPIN_GPIO_7

#define kLEFT_FORWARD        digitalWrite(kLEFT_DIRECTION_PIN, LOW)
#define kLEFT_BACK           digitalWrite(kLEFT_DIRECTION_PIN, HIGH)
#define kLEFT_RELEASE_BRAKE  digitalWrite(kLEFT_BRAKE_PIN, LOW)
#define kLEFT_BRAKE          digitalWrite(kLEFT_BRAKE_PIN, HIGH)

#define kRIGHT_FORWARD        digitalWrite(kRIGHT_DIRECTION_PIN, HIGH)
#define kRIGHT_BACK           digitalWrite(kRIGHT_DIRECTION_PIN, LOW)
#define kRIGHT_RELEASE_BRAKE  digitalWrite(kRIGHT_BRAKE_PIN, LOW)
#define kRIGHT_BRAKE          digitalWrite(kRIGHT_BRAKE_PIN, HIGH)

#define kSTEP_SCALE		100  // change 100 times
#define kMAXRPM		   3000
enum kGLOBLE_MOVE_STA
{
	kSTOP_MOVE = 0,
	kMOVE_FOWARD,
	kMOVE_BACK,
	KTURN_LEFT,
	kTURN_RIGHT
};

int32_t currentLeftRPM_ = 0;
int32_t currentRightRPM_ = 0;

T3RobotMotorDriver::T3RobotMotorDriver()
{

}

T3RobotMotorDriver::~T3RobotMotorDriver()
{
	closeMotor();
}

void T3RobotMotorDriver::init(void)
{
	setTorque(kMOTOR_LEFT_ID, true);
	setTorque(kMOTOR_RIGHT_ID, true);
//creat  new object dynamixel::GroupSyncWrite and dynamixel::GroupSyncRead
}

bool T3RobotMotorDriver::setTorque(uint8_t id, bool onoff)
{
	if(kMOTOR_LEFT_ID == id)
	{
		if(onoff)
		{
			/*driver*/
			pinMode(kLEFT_DIRECTION_PIN, OUTPUT);
			pinMode(kLEFT_BRAKE_PIN, OUTPUT);
			kLEFT_BRAKE;
			pinMode(kLEFT_SCL, OUTPUT);
			digitalWrite(kLEFT_SCL, HIGH);
			pinMode(kLEFT_SDA, OUTPUT);
			digitalWrite(kLEFT_SDA, HIGH);
			/*encoder*/
		}
		else
		{
			//myHardwareI2Cdev.masterRead(0x71, 4, data);
		}
	}
	else if(kMOTOR_RIGHT_ID == id)
	{
		if(onoff)
		{
			pinMode(kRIGHT_DIRECTION_PIN, OUTPUT);
			pinMode(kRIGHT_BRAKE_PIN, OUTPUT);
			kRIGHT_BRAKE;
			pinMode(kRIGHT_SCL, OUTPUT);
			digitalWrite(kRIGHT_SCL, HIGH);
			pinMode(kRIGHT_SDA, OUTPUT);
			digitalWrite(kRIGHT_SDA, HIGH);
		}
		else
		{

		}
	}
	return true;
}

void T3RobotMotorDriver::closeMotor(void)
{
	setTorque(kMOTOR_LEFT_ID, false);
	setTorque(kMOTOR_RIGHT_ID, false);
}

int tempVoltageLeft_ = 0, tempVoltageRight_ = 0;
uint8_t minesLeftFlag = 0;
uint8_t minesRightFlag = 0;
uint16_t deltaLeft = 0;
uint16_t deltaRight = 0;
int64_t left_last_RPM = 0;
int64_t right_last_RPM = 0;
#define kStep_Num 0.1
#define kDelta    20
bool T3RobotMotorDriver::speedControl(int64_t left_wheel_val, int64_t right_wheel_val, int64_t left_present_RPM, int64_t right_present_RPM) //give in RPMencoder
{
	uint64_t idx_ = 0;
  uint8_t ret_l = 0, ret_r = 0;
	int32_t stepLeftRPM_ = 0, stepRightRPM_ = 0;

	uint8_t voltageLeftData_[2] = {0};
	uint8_t voltageRightData_[2] = {0};
  int32_t absLeftRPM_ = 0;
  int32_t absRightRPM_ = 0;
  int16_t vol_num_left = 0;
  int16_t vol_num_right = 0;


//  if((left_wheel_val != 0 || right_wheel_val != 0) && (0==left_present_RPM || 0==right_present_RPM))
//  {
//    delta += 100;
//  }
  

  if((left_wheel_val > 0)&&(0 == left_present_RPM)) 
  {
    deltaLeft += kDelta;
    tempVoltageLeft_ = 1100 + deltaLeft;
  }
  if((left_wheel_val < 0)&&(0 == left_present_RPM)) 
  {
    deltaLeft += kDelta;
    tempVoltageLeft_ = -1100 - deltaLeft;
  }
  if((right_wheel_val > 0)&&(0 == right_present_RPM)) 
  {
    deltaRight += kDelta;
    tempVoltageRight_ = 1100 + deltaRight;
  }
  if((right_wheel_val < 0)&&(0 == right_present_RPM)) 
  {
    deltaRight += kDelta;
    tempVoltageRight_ = -1100 - deltaRight;
  }

  if(left_present_RPM != 0) deltaLeft = 0;
  if(right_present_RPM != 0) deltaRight = 0;
  
  if((0 == left_wheel_val)&&(0 == left_present_RPM)) tempVoltageLeft_ = 0;
  if((0 == right_wheel_val)&&(0 == right_present_RPM)) tempVoltageRight_ = 0;
//  Serial.println(deltaLeft);
//  Serial.print("left_wheel_val : ");
//  Serial.println((int32_t)left_wheel_val);
//  Serial.print("right_wheel_val : ");
//  Serial.println((int32_t)right_wheel_val);
//  Serial.print("left_present_RPM : ");
//  Serial.println((int32_t)left_present_RPM);
//  Serial.print("right_present_RPM : ");
//  Serial.println((int32_t)right_present_RPM);
//  Serial.print("tempVoltageLeft_ : ");
//  Serial.println((int32_t)tempVoltageLeft_);
//  Serial.print("tempVoltageRight_ : ");
//  Serial.println((int32_t)tempVoltageRight_);
// 
  vol_num_left = abs(left_present_RPM - left_wheel_val)*kStep_Num;
  vol_num_right = abs(right_present_RPM - right_wheel_val)*kStep_Num;
  if(vol_num_left > 500) vol_num_left = 500;
  if(vol_num_right > 500) vol_num_right = 500;
  
  if(abs(left_present_RPM) > abs(left_wheel_val)) 
  {
    if(left_present_RPM > left_wheel_val) tempVoltageLeft_ = tempVoltageLeft_ - vol_num_left;//
    if(left_present_RPM < left_wheel_val) tempVoltageLeft_ = tempVoltageLeft_ + vol_num_left; 
  }
  else if(abs(left_present_RPM) < abs(left_wheel_val))
  {
    if(left_present_RPM > left_wheel_val) tempVoltageLeft_ = tempVoltageLeft_ - vol_num_left;
    if(left_present_RPM < left_wheel_val) tempVoltageLeft_ = tempVoltageLeft_ + vol_num_left; //
//    Serial.print("tempVoltageLeft_after : ");
//    Serial.println((int32_t)tempVoltageLeft_);
  }
  if(abs(right_present_RPM) > abs(right_wheel_val)) 
  {
    if(right_present_RPM > right_wheel_val) tempVoltageRight_ = tempVoltageRight_ - vol_num_right;//
    if(right_present_RPM < right_wheel_val) tempVoltageRight_ = tempVoltageRight_ + vol_num_right; 
  }
  else if(abs(right_present_RPM) < abs(right_wheel_val))
  {
    if(right_present_RPM > right_wheel_val) tempVoltageRight_ = tempVoltageRight_ - vol_num_right;
    if(right_present_RPM < right_wheel_val) tempVoltageRight_ = tempVoltageRight_ + vol_num_right; //
//    Serial.print("tempVoltageRight_after : ");
//    Serial.println((int32_t)tempVoltageRight_);
  }
    if(tempVoltageLeft_ > 0)
    {
      kLEFT_FORWARD;
      kLEFT_RELEASE_BRAKE;      //left foward
    }
    else if(0 == tempVoltageLeft_)
    {
      kLEFT_BRAKE;      //stop
    }
    else if(tempVoltageLeft_ < 0)
    {
      kLEFT_BACK;
      kLEFT_RELEASE_BRAKE;      //left back
//      Serial.println("3 - 3 ");
//      Serial.println("left back");
    }
    
    if(tempVoltageRight_ > 0)
    {
      kRIGHT_FORWARD;
      kRIGHT_RELEASE_BRAKE;     //right foward
    }
    else if(0 == tempVoltageRight_)
    {
      kRIGHT_BRAKE;     //stop
//       Serial.println("4 - 2");
    }
    else if(tempVoltageRight_ < 0)
    {
      kRIGHT_BACK;
      kRIGHT_RELEASE_BRAKE;     //right back
//      Serial.println("4 - 3");
//      Serial.println("right back");
    }
//    setWheelDirection(tempVoltageLeft_, tempVoltageRight_);
  
//    Serial.print("tempVoltageRight_after : ");
//    Serial.println((int32_t)tempVoltageRight_);

    if(tempVoltageLeft_ > 4095) tempVoltageLeft_ = 4095;
    if(tempVoltageRight_ > 4095) tempVoltageRight_ = 4095;
    if(tempVoltageLeft_ < -4095) tempVoltageLeft_ = -4095;
    if(tempVoltageRight_ < -4095) tempVoltageRight_ = -4095;
    
    if(tempVoltageLeft_ < 0) 
    {
      minesLeftFlag = 1;
      tempVoltageLeft_ = abs(tempVoltageLeft_);
    }
    if(tempVoltageRight_ < 0) 
    {
      minesRightFlag = 1;
      tempVoltageRight_ = abs(tempVoltageRight_);
    }
    voltageLeftData_[0] = tempVoltageLeft_ / 16;
    voltageLeftData_[1] = (tempVoltageLeft_ % 16) << 4;
    voltageRightData_[0] = tempVoltageRight_ / 16;
    voltageRightData_[1] = (tempVoltageRight_ % 16) << 4;
    //jason begin
    SoftI2C  T3SoftI2C;
    //jason end
    ret_l = T3SoftI2C.writeBuffer(kI2C_DAC_LEFT, kI2C_DAC_SLAVE_ADDR, kI2C_DAC_WRITE_ADDR, 2, voltageLeftData_);
    delay_us(20);
    ret_r = T3SoftI2C.writeBuffer(kI2C_DAC_RIGHT, kI2C_DAC_SLAVE_ADDR, kI2C_DAC_WRITE_ADDR, 2, voltageRightData_);
    if(1 == minesLeftFlag)
    {
      tempVoltageLeft_ = -tempVoltageLeft_;
      minesLeftFlag = 0;
//      Serial.print("minesLeftFlag :");
//      Serial.println(minesLeftFlag);
    }
     if(1 == minesRightFlag)
    {
      tempVoltageRight_ = -tempVoltageRight_;
      minesRightFlag = 0;
//      Serial.print("minesRightFlag :");
//      Serial.println(minesRightFlag);
    }
  
  left_last_RPM = left_present_RPM;
  right_last_RPM = right_present_RPM;
// Serial.println("***********************************");
// Serial.println("end of Speed Control!!!!!");
// Serial.println("***********************************");
// delay_ms(10);
}

void T3RobotMotorDriver::setWheelDirection(int leftWheel, int rightWheel)
{
    if(leftWheel > 0)
    {
      kLEFT_FORWARD;
      kLEFT_RELEASE_BRAKE;      //left foward
    }
    else if(0 == leftWheel)
    {
      kLEFT_BRAKE;      //stop
    }
    else if(leftWheel < 0)
    {
      kLEFT_BACK;
      kLEFT_RELEASE_BRAKE;      //left back
//      Serial.println("3 - 3 ");
//      Serial.println("left back");
    }
    
    if(rightWheel > 0)
    {
      kRIGHT_FORWARD;
      kRIGHT_RELEASE_BRAKE;     //right foward
    }
    else if(0 == rightWheel)
    {
      kRIGHT_BRAKE;     //stop
//       Serial.println("4 - 2");
    }
    else if(rightWheel < 0)
    {
      kRIGHT_BACK;
      kRIGHT_RELEASE_BRAKE;     //right back
//      Serial.println("4 - 3");
//      Serial.println("right back");
    }
}
