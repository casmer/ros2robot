
#include <stdint.h>
#include <wiringSerial.h>
#define TRACE_MDDS30 1
class MDDS30
{

public:

	enum MOTORSELECT
	{
		LEFT=0,
		RIGHT=1
	};
	enum MOTORSPEEDS
	{
		FULL_REVERSE=0,
		STOP=127,
		FULL_FORWARD=255
	};

	unsigned char DRIVE_COMMAND = 85;
	MDDS30(char* dev,unsigned int addr = 0, int baud=115200);
	~MDDS30();


	bool initialize();
	bool setMotor(enum MOTORSELECT motorNumber, unsigned char speed);
private:


	short range_map(short  x,short  in_min,short  in_max,short  out_min,short  out_max);
	const char _dev[256];
	int _serialDeviceFd;
	int _address;
	int _baud;
};


