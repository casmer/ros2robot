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
#include "ugeek_motor_hat.hpp"

#include "lalosoft_robot_msgs/msg/drive_message.hpp"

#include "SimpleTimer.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RobotDriveHost : public rclcpp::Node
{
public:
	RobotDriveHost()
: Node("RobotDriveHost")
{
		subscription_ = this->create_subscription<lalosoft_robot_msgs::msg::DriveMessage>(
				"LalosoftDriveCommand", 10, std::bind(&RobotDriveHost::topic_callback, this, _1));


		if (!_motorHat.initialize())
		{
			RCLCPP_ERROR(get_logger(), "Motor HAT failed to Initialize!");
		}

		if (!_pca6895.initialize("/dev/i2c-1", 0x70))
		{
			RCLCPP_ERROR(get_logger(), "PW M Controller failed to Initialize!");
		}


		driveCheck_ = this->create_wall_timer(
				500ms, std::bind(&RobotDriveHost::timer_callback, this));
}
	~RobotDriveHost()
	{
		setMotor(0, 0);
		setMotor(1, 0);
	}

	const int PAN_MIN_HORIZONTAL_ANGLE = 0;
	const int PAN_MAX_HORIZONTAL_ANGLE =  180;
	const int PAN_MIN_VERTICAL_ANGLE   = 0;
	const int PAN_MAX_VERTICAL_ANGLE   =  180;
private:

	rclcpp::Subscription<lalosoft_robot_msgs::msg::DriveMessage>::SharedPtr subscription_;

	UGeek_Motor_Hat _motorHat;
	PCA6895 _pca6895;
	std::mutex _motorSpeedLock;
	SystemSimpleTimer _timer;
	rclcpp::TimerBase::SharedPtr driveCheck_;
	int _leftMotorSpeed;
	int _rightMotorSpeed;

	int _pan_horizontal_angle;
	int _pan_vertical_angle;

	void setMotor(int number, int motor_speed)
	{
		if (motor_speed==0)
		{
			if (!_motorHat.runMotor(number,UGeek_Motor_Hat::MOTOR_DRIVE::RELEASE))
			{
				RCLCPP_ERROR(get_logger(), "run motor failed!");
			}
			if (!_motorHat.setMotorSpeed(number, 0))
			{
				RCLCPP_ERROR(get_logger(), "set motor speed failed!");
			}
		} else if (motor_speed > 0)
		{
			if (!_motorHat.runMotor(number,UGeek_Motor_Hat::MOTOR_DRIVE::FORWARD))
			{
				RCLCPP_ERROR(get_logger(), "run motor failed!");
			}
			if (!_motorHat.setMotorSpeed(number, motor_speed*255/100))
			{
				RCLCPP_ERROR(get_logger(), "set motor speed failed!");
			}
		} else if (motor_speed<0)
		{
			if (!_motorHat.runMotor(number,UGeek_Motor_Hat::MOTOR_DRIVE::BACKWARD))
			{
				RCLCPP_ERROR(get_logger(), "run motor failed!");
			}
			if (!_motorHat.setMotorSpeed(number, abs(motor_speed)*255/100))
			{
				RCLCPP_ERROR(get_logger(), "set motor speed failed!");
			}
		}

	}

	void topic_callback(const lalosoft_robot_msgs::msg::DriveMessage::SharedPtr message)
	{
		RCLCPP_INFO(this->get_logger(), "Left: '%d', Right: '%d'",message->left_motor_speed, message->right_motor_speed);
		std::lock_guard<std::mutex> lock (_motorSpeedLock);
		_timer.reset();
		_leftMotorSpeed = message->left_motor_speed;
		_rightMotorSpeed = message->right_motor_speed;
		setMotor(0, message->right_motor_speed);
		setMotor(1, message->left_motor_speed);

	}




	void timer_callback()
	{
		std::lock_guard<std::mutex> lock (_motorSpeedLock);
		unsigned int actual_wait_time = _timer.elapsed_time<unsigned int, std::chrono::milliseconds>();
		if (actual_wait_time > 500 && (_leftMotorSpeed!=0 && _rightMotorSpeed!=0))
		{
			RCLCPP_INFO(this->get_logger(), "No new command message, stopping motors.");
			setMotor(0, 0);
			setMotor(1, 0);
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
