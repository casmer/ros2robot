// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <chrono>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "pca6895.hpp"
#include "mdds30.hpp"
#include "lalosoft_robot_msgs/msg/drive_message.hpp"
#include "lalosoft_robot_msgs/msg/pan_tilt.hpp"
#include "SimpleTimer.hpp"
#include "wiringPi.h"

using std::placeholders::_1;
using namespace std::chrono_literals;
#define RIGHT_MOTOR_PWM 4
#define LEFT_MOTOR_PWM 5

class RobotDriveHost : public rclcpp::Node
{
public:
	RobotDriveHost()
: Node("RobotDriveHost")
{
		subscription_ = this->create_subscription<lalosoft_robot_msgs::msg::DriveMessage>(
				"LalosoftDriveCommand", 10, std::bind(&RobotDriveHost::topic_callback, this, _1));

		RCLCPP_INFO(get_logger(), "Setting severity threshold to DEBUG");
		if (!_pca6895.initialize())
		{
			RCLCPP_ERROR(get_logger(), "PWM controller failed to Initialize!");
		}
//		_mdds30 = new MDDS30("/dev/ttyAMA0", 0, 115200);
//		if (!_mdds30->initialize())
//		{
//			RCLCPP_ERROR(get_logger(), "MDDS30 failed to Initialize!");
//		}
		wiringPiSetup();
		RCLCPP_INFO(get_logger(), "Setting up pin");
		pinMode(25, OUTPUT);
		pullUpDnControl(25, PUD_UP );
		digitalWrite(25, LOW);
		RCLCPP_INFO(get_logger(), "Setting up pin");
		driveCheck_ = this->create_wall_timer(
				500ms, std::bind(&RobotDriveHost::timer_callback, this));
		setMotor(RIGHT_MOTOR_PWM, 1500);
		setMotor(LEFT_MOTOR_PWM, 1500);

}
	~RobotDriveHost()
	{
		setMotor(RIGHT_MOTOR_PWM, 1500);
		setMotor(LEFT_MOTOR_PWM, 1500);
//		if (_mdds30)
//		{
//			delete _mdds30;
//			_mdds30= nullptr;
//		}
	}
	const int PAN_MIN_HORIZONTAL_ANGLE = 0;
	const int PAN_MAX_HORIZONTAL_ANGLE =  180;
	const int PAN_MIN_VERTICAL_ANGLE   = 0;
	const int PAN_MAX_VERTICAL_ANGLE   =  180;
private:

	rclcpp::Subscription<lalosoft_robot_msgs::msg::DriveMessage>::SharedPtr subscription_;

	std::mutex _motorSpeedLock;
	PCA6895 _pca6895;
//	MDDS30* _mdds30;
	SystemSimpleTimer _timer;
	rclcpp::TimerBase::SharedPtr driveCheck_;
	int _leftMotorSpeed;
	int _rightMotorSpeed;

	void setMotor(int number, int motor_speed)
	{
		static int lastmotorspeed=0;

		if (motor_speed!=lastmotorspeed)
		{
			RCLCPP_INFO(this->get_logger(), "motornum:%d, motorspeed:%d", number, motor_speed);
		}

		if (motor_speed==1500)
		{
			if (motor_speed!=lastmotorspeed)
			{
				RCLCPP_INFO(this->get_logger(), "write 0");
			}
			digitalWrite(25, 0);
		} else
		{
			if (motor_speed!=lastmotorspeed)
			{
				RCLCPP_INFO(this->get_logger(), "write 1");
			}
			digitalWrite(25, 1);
		}
		lastmotorspeed=motor_speed;

		_pca6895.setPwmAsSpeed(number, motor_speed);

	}

	void topic_callback(const lalosoft_robot_msgs::msg::DriveMessage::SharedPtr message)
	{

		std::lock_guard<std::mutex> lock (_motorSpeedLock);
		_timer.reset();

		int leftspeed = message->left_motor_speed == 0 ? 1500 : range_map(message->left_motor_speed+100, 0, 200, 600, 2400);
		int rightspeed =message->right_motor_speed == 0 ? 1500 : range_map(message->right_motor_speed+100, 0, 200, 600, 2400);
				setMotor(RIGHT_MOTOR_PWM,rightspeed );
				setMotor(LEFT_MOTOR_PWM, leftspeed);

		if ((_leftMotorSpeed != message->left_motor_speed) ||
				(_rightMotorSpeed != message->right_motor_speed))
		{



			RCLCPP_INFO(this->get_logger(), "Left: '%d (%d)', Right: '%d (%d)'",
					message->left_motor_speed,
					leftspeed,
					message->right_motor_speed,
					rightspeed);
		}



		_leftMotorSpeed = message->left_motor_speed;
				_rightMotorSpeed = message->right_motor_speed;

	}
	int range_map(int  x,int  in_min,int  in_max,
			int  out_min,int  out_max)
	{
	       return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}


	void timer_callback()
	{
		std::lock_guard<std::mutex> lock (_motorSpeedLock);
		unsigned int actual_wait_time = _timer.elapsed_time<unsigned int, std::chrono::milliseconds>();
		if (actual_wait_time > 500 && (_leftMotorSpeed!=0 && _rightMotorSpeed!=0))
		{
			RCLCPP_INFO(this->get_logger(), "No new command message, stopping motors.");
			setMotor(RIGHT_MOTOR_PWM, 1500);
		    setMotor(LEFT_MOTOR_PWM, 1500);
			_leftMotorSpeed=0;
			_rightMotorSpeed=0;
		}

	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotDriveHost>());
	rclcpp::shutdown();
	return 0;
}
