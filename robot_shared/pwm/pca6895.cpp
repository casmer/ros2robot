
#include <pca6895.hpp>
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

#define PWM_FREQUENCY   60.0
#define PWM_MIN_PULSE_WIDTH 600
#define PWM_MAX_PULSE_WIDTH 2400
#define PWM_DEFAULT_PULSE_WIDTH 1500

#define PWM_PRESCALE    0xFE


#define PWM_MODE1       0x00
#define PWM_MODE2       0x01
#define PWM_LED0_ON_L   0x06
#define PWM_LED0_ON_H   0x07
#define PWM_LED0_OFF_L  0x08
#define PWM_LED0_OFF_H  0x09

#define PWM_RESTART     0x80
#define PWM_SLEEP       0x10
#define PWM_ALLCALL     0x01
#define PWM_INVRT       0x10
#define PWM_OUTDRV      0x04

PCA6895::PCA6895(unsigned int addr)
: _i2cAddr(addr),
  _i2cDeviceHandle(0)
{


}

PCA6895::~PCA6895()

{
  if (_i2cDeviceHandle)
  {
	  close(_i2cDeviceHandle);
  }
}

bool PCA6895::initialize()
{

	bool result = true;

	_i2cDeviceHandle = wiringPiI2CSetupInterface("/dev/i2c-1", _i2cAddr);
#if TRACE_PCA6895
	printf("device handle = %d\n",_i2cDeviceHandle);
#endif

	if (_i2cDeviceHandle <0)
	{
		result = false;
	} else
	{
		 //Setup PWM
		setAllPwm(_i2cDeviceHandle, 0, 0);
		wiringPiI2CWriteReg8(_i2cDeviceHandle, PWM_MODE2, PWM_OUTDRV);
		wiringPiI2CWriteReg8(_i2cDeviceHandle, PWM_MODE1, PWM_ALLCALL);
		usleep(5);
		unsigned short mode1 = wiringPiI2CReadReg8(_i2cDeviceHandle, PWM_MODE1) & ~PWM_SLEEP;
		wiringPiI2CWriteReg8(_i2cDeviceHandle, PWM_MODE1, mode1);
		usleep(5);

		//Set PWM frequency
		unsigned short prescale = (int)(25000000.0 / 4096.0 / PWM_FREQUENCY - 1.0);
		unsigned short oldmode = wiringPiI2CReadReg8(_i2cDeviceHandle, PWM_MODE1);
		unsigned short newmode = (oldmode & 0x7F) | 0x10;
		wiringPiI2CWriteReg8(_i2cDeviceHandle, PWM_MODE1, newmode);
		wiringPiI2CWriteReg8(_i2cDeviceHandle, PWM_PRESCALE, prescale);
		wiringPiI2CWriteReg8(_i2cDeviceHandle, PWM_MODE1, oldmode);
		usleep(5);
		wiringPiI2CWriteReg8(_i2cDeviceHandle, PWM_MODE1, oldmode | 0x80);
	}

	return result;
}

void PCA6895::setAllPwm(unsigned short i2c, unsigned short on, unsigned short off)
{
        wiringPiI2CWriteReg8(i2c, __LED0_ON_L, on & 0xFF);
        wiringPiI2CWriteReg8(i2c, __LED0_ON_H, on >> 8);
        wiringPiI2CWriteReg8(i2c, __LED0_OFF_L, off & 0xFF);
        wiringPiI2CWriteReg8(i2c, __LED0_OFF_H, off >> 8);
}
bool PCA6895::setPwm(unsigned short pinNumber, unsigned short onValue, unsigned short offValue)
{
	bool success=false;
	if ( pinNumber<=15)
	{
#if TRACE_PCA6895
		printf("writing values:fd:%d, %d, on:%d, off: %d\n",_i2cDeviceHandle, pinNumber, onValue, offValue);
#endif
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
short PCA6895::range_map(short  x,short  in_min,short  in_max,
			short  out_min,short  out_max)
{
       return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

short PCA6895::angle_to_analog(short angle)
{
	unsigned int pulse_wide =0;
	short	analog_value =0;

	pulse_wide =	range_map(angle, 0, 180, PWM_MIN_PULSE_WIDTH, PWM_MAX_PULSE_WIDTH);
	analog_value = ((unsigned int)pulse_wide) * PWM_FREQUENCY * 4096 / 1000000 ;
#if TRACE_PCA6895
	printf("writing values:pulse_wide: %d, analog_value:%d\n", pulse_wide, analog_value);
#endif
	return analog_value;
}

short PCA6895::speed_to_analog(unsigned int angle)
{
	unsigned int pulse_wide =0;
	short	analog_value =0;

	pulse_wide =	range_map(angle, PWM_MIN_PULSE_WIDTH, PWM_MAX_PULSE_WIDTH, PWM_MIN_PULSE_WIDTH, PWM_MAX_PULSE_WIDTH);
	analog_value = ((unsigned int)pulse_wide) * PWM_FREQUENCY * 4096 / 1000000 ;
#if TRACE_PCA6895
	printf("writing values:pulse_wide: %d, analog_value:%d\n", pulse_wide, analog_value);
#endif
	return analog_value;
}


bool PCA6895::setPwmAsSpeed(unsigned short pinNumber, unsigned int speed)
{
	bool success=false;
	if ( pinNumber<=15)
	{
		unsigned short analog_angle = speed_to_analog(speed);
#if TRACE_PCA6895
		printf("writing values:pin: %d, angle:%d, analog_angle: %d\n", pinNumber, angle, analog_angle);
#endif
		setPwm(pinNumber,0, analog_angle);
	}
	return success;
}

bool PCA6895::setPwmAsAngle(unsigned short pinNumber, unsigned short angle)
{
	bool success=false;
	if ( pinNumber<=15)
	{
		unsigned short analog_angle = angle_to_analog(angle);
#if TRACE_PCA6895
		printf("writing values:pin: %d, angle:%d, analog_angle: %d\n", pinNumber, angle, analog_angle);
#endif
		setPwm(pinNumber, 0, analog_angle);
	}
	return success;
}
