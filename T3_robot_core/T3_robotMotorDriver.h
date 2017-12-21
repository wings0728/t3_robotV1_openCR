#ifndef _T3_ROBOT_MOTOR_DRIVER_H_
#define _T3_ROBOT_MOTOR_DRIVER_H_
#include "T3_softI2C.h"
#include <stdint.h>
#define kLIMIT_X_MAX_VELOCITY	3000  //ours?
#define kMOTOR_LEFT_ID                      1       // ID of left motor
#define kMOTOR_RIGHT_ID                    2       // ID of right motor

#define kTORQUE_Enable                 1       // Value for enabling the torque
#define KTORQUE_DISABLE              0

#define kVOLTAGE_ENCODER_RPM_CONSTANT_VALUE    3.07125
//(Encoder * 6) / 4 = RPM_real
//MaxEncoder = MaxRPM / 1.5 = 2000 //In fact, the real MaxRPM = 3150, the real MaxEncoder = 2100
//Encoder / MaxEncoder = VoltageADC / 4095
//VotageADC = 4095 * Encoder / MaxEncoder
//          = 4095 * Encoder * 1.5 / MaxEncoder
//          = 3.07125 * Encoder  //MaxEncoder = 2000

struct PID
{
  float Kp;
  float Ki;
  float Kd;
  float error_0;//基波分量
  float error_1;//一次谐波分量
  float error_2;//二次谐波分量
  long  Sum_error;
  float OutputValue;//实际输出量
  float OwenValue;//零误差时的标准输出量  
};

class T3RobotMotorDriver
{
	public:
		T3RobotMotorDriver();
		~T3RobotMotorDriver();
		void init(void);
		void closeMotor(void);
		bool setTorque(uint8_t id, bool onoff);
//		bool readEncoder(int32_t &left_value, int32_t &right_value);
		bool speedControl(int64_t left_wheel_val, int64_t right_wheel_val, int64_t left_present_RPM, int64_t right_present_RPM);
    float PID_calculate(struct PID *Control, float CurrentValue_left);
    
	private:
		uint8_t left_wheel_id_;
		uint8_t right_wheel_id;
    void setWheelDirection(int leftWheel, int rightWheel);
//		void setSpeedSlowChange(uint16_t leftSpeedDAC, uint16_t rightSpeedDAC);

};

    

    

#endif
