
#include <stdint.h>
#include <wiringPiI2C.h>
#define TRACE_MOTOR_HAT 0
class PCA6895
{

public:

	typedef enum MOTOR_DRIVE
	{
		FORWARD = 1,
		BACKWARD = 2,
		BRAKE = 3,
		RELEASE = 4,
	} MOTOR_DRIVE_t;
	typedef enum STEP_COMMAND
	{
			SINGLE = 1,
			DOUBLE = 2,
			INTERLEAVE = 3,
			MICROSTEP = 4
	} STEP_COMMAND_t;
	const int MAX_MOTOR=3;
	const int MIN_MOTOR=0;

	PCA6895(unsigned int  addr = 0x6f);
	~PCA6895();


	bool initialize();
	bool setPwmAsAngle(unsigned short pinNumber, unsigned short angle);
private:

	void setAllPwm(unsigned short i2c, unsigned short on, unsigned short off);
	bool setPwm(unsigned short pinNumber, unsigned short onValue, unsigned short offValue);

	short range_map(short  x,short  in_min,short  in_max,short  out_min,short  out_max);
	short angle_to_analog(short angle);

	unsigned int _i2cAddr;
	int _i2cDeviceHandle;
};


