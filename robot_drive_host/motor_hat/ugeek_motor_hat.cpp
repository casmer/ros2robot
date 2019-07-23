
#include "ugeek_motor_hat.hpp"

#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"

  //# Registers/etc.
#define __MODE1              0x00
#define __MODE2              0x01
#define __SUBADR1            0x02
#define __SUBADR2            0x03
#define __SUBADR3            0x04
#define __PRESCALE           0xFE
#define __LED0_ON_L          0x06
#define __LED0_ON_H          0x07
#define __LED0_OFF_L         0x08
#define __LED0_OFF_H         0x09
#define __ALL_LED_ON_L       0xFA
#define __ALL_LED_ON_H       0xFB
#define __ALL_LED_OFF_L      0xFC
#define __ALL_LED_OFF_H      0xFD

//  # Bits
#define __RESTART            0x80
#define __SLEEP              0x10
#define __ALLCALL            0x01
#define __INVRT              0x10
#define __OUTDRV             0x04




UGeek_Motor_Hat::UGeek_Motor_Hat(unsigned int addr)
: _i2cAddr(addr),
  _i2cDeviceHandle(0)
{


}

UGeek_Motor_Hat::~UGeek_Motor_Hat()

{
  if (_i2cDeviceHandle)
  {
	  close(_i2cDeviceHandle);
  }
}

bool UGeek_Motor_Hat::initialize()
{
	bool result = true;
	_i2cDeviceHandle = wiringPiI2CSetup (_i2cAddr);
	if (_i2cDeviceHandle <0)
		result = false;

	return result;
}
bool UGeek_Motor_Hat::setPin(int pinNumber, int value)
{
	bool success=true;
	if (pinNumber<0 || pinNumber>15)
	{
		return false;
	}
	else if (value==0)
	{
		setPwm(pinNumber, 0, 4096);
	}
	else if (value==1)
	{
		setPwm(pinNumber, 4096, 0);
	}
	else
	{
		success=false;
	}
	return success;
}
bool UGeek_Motor_Hat::setPwm(int pinNumber, int onValue, int offValue)
{
	bool success=false;
	if (pinNumber>0 && pinNumber<=15)
	{
	success =
		(
		(0==wiringPiI2CWriteReg8(_i2cDeviceHandle,__LED0_ON_L+4*pinNumber, onValue & 0xFF))
		&& (0==wiringPiI2CWriteReg8(_i2cDeviceHandle,__LED0_ON_H+4*pinNumber, onValue >> 8))
		&& (0==wiringPiI2CWriteReg8(_i2cDeviceHandle,__LED0_OFF_L+4*pinNumber, offValue & 0xFF))
		&& (0==wiringPiI2CWriteReg8(_i2cDeviceHandle,__LED0_OFF_H+4*pinNumber, offValue >> 8))
		);
	}
	return success;
}
bool UGeek_Motor_Hat::runMotor(int motorNum, MOTOR_DRIVE_t  command)
{
	bool success = true;
	if (motorNum>MAX_MOTOR || motorNum<MIN_MOTOR)
	{
		success = false;
	}
	else
	{
		switch (command)
		{
		case MOTOR_DRIVE::FORWARD:
			success =
					setPin(_in1Pin[motorNum], 1)
					&& setPin(_in2Pin[motorNum], 0);
			break;
		case MOTOR_DRIVE::BACKWARD:
			success =
					setPin(_in1Pin[motorNum], 0)
					&& setPin(_in2Pin[motorNum], 1);
			break;
		case MOTOR_DRIVE::RELEASE:
			success =
					setPin(_in1Pin[motorNum], 0)
					&& setPin(_in2Pin[motorNum], 0);
			break;
		default:
			success=false;
			break;
		}
	}
	return success;

}

bool UGeek_Motor_Hat::setMotorSpeed(int motorNum, int speed)
{
	bool success=true;
	if (motorNum>MAX_MOTOR || motorNum<MIN_MOTOR)
	{
		success = false;
	}
	else
	{
		if (speed<0)
		{
			speed=0;
		}
		if (speed>255)
		{
			speed=255;
		}
		success = setPwm(_pwmPin[motorNum], 0, speed*16);
	}
	return success;
}

