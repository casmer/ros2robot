
#include "mdds30.hpp"

#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"


MDDS30::MDDS30(const char* dev,unsigned int addr, int baud)
:
  _serialDeviceFd(0),
  _address(addr),
  _baud(baud)
{
	int length = strlen(dev);

	strncpy(_dev, dev, length);

}

MDDS30::~MDDS30()

{
	if (_serialDeviceFd)
	{
		close(_serialDeviceFd);
	}
}

bool MDDS30::initialize()
{

	bool result = true;
#if TRACE_MDDS30
	printf("serial device = %s\n",_dev);
#endif
	_serialDeviceFd = serialOpen(_dev, _baud);
#if TRACE_MDDS30
	printf("serial device handle = %d\n",_serialDeviceFd);
#endif

	if (_serialDeviceFd <0)
	{
		result = false;
	} else
	{

#if TRACE_MDDS30
	printf("Writing setup bit\n");
#endif
		//Initialize Driver
		serialPutchar(_serialDeviceFd, 0x80);
		serialFlush(_serialDeviceFd);
	}

	return result;
}

bool MDDS30::setMotor(enum MOTORSELECT motorNumber, unsigned char speed)
{
	bool success=false;
	unsigned char checksum;
	unsigned char channel_and_address;
#if TRACE_MDDS30
	printf("writing values:fd:%d, motor: %d, speed:%d\n",_serialDeviceFd, motorNumber, speed);
#endif

	checksum = DRIVE_COMMAND+_address+speed;
	serialPutchar(_serialDeviceFd, DRIVE_COMMAND);
	channel_and_address=_address + (motorNumber << 3);
	serialPutchar(_serialDeviceFd, channel_and_address);
	serialPutchar(_serialDeviceFd, speed);
	serialPutchar(_serialDeviceFd, checksum);


	return success;
}
short MDDS30::range_map(short  x,short  in_min,short  in_max,
		short  out_min,short  out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

