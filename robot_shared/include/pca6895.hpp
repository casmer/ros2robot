
#include <stdint.h>
#include <wiringPiI2C.h>
#include <string>
#define TRACE_PCA6895 0
class PCA6895
{

public:

	PCA6895();
	~PCA6895();


	bool initialize(const char* i2c_dev, unsigned int  addr = 0x70, std::string node_name = "PCA6895");
	bool setPwmAsAngle(unsigned short pinNumber, unsigned short angle);
	bool setPwmAsSpeed(unsigned short pinNumber, unsigned int speed);


private:

	void setAllPwm(unsigned short i2c, unsigned short on, unsigned short off);
	bool setPwm(unsigned short pinNumber, unsigned short onValue, unsigned short offValue);

	short range_map(short  x,short  in_min,short  in_max,short  out_min,short  out_max);

	short speed_to_analog(unsigned int angle);
	short angle_to_analog(short angle);

	unsigned int _i2cAddr;
	int _i2cDeviceHandle;
	std::string nodeName;
};


