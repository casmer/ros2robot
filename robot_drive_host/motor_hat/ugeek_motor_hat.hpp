
#include <stdint.h>
#include <wiringPiI2C.h>

class UGeek_Motor_Hat
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

	UGeek_Motor_Hat(unsigned int  addr = 0x60);
	~UGeek_Motor_Hat();


	bool initialize();
	bool runMotor(int motorNum, MOTOR_DRIVE_t  command);
    bool setMotorSpeed(int motorNum, int speed);
private:
	const int _pwmPin[4] = {8,13,2,7};
	const int _in2Pin[4] = {9,12,3,6};
	const int _in1Pin[4] = {10,11,4,5};
	bool setPin(int pinNumber, int value);
	bool setPwm(int pinNumber, int onValue, int offValue);
	unsigned int _i2cAddr;
	int _i2cDeviceHandle;
};


