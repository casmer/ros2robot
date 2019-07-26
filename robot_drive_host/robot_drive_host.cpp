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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "motor_hat/ugeek_motor_hat.hpp"
#include "robot_msgs/msg/drive_message.hpp"

using std::placeholders::_1;

class RobotDriveHost : public rclcpp::Node
{
public:
	RobotDriveHost()
: Node("RobotDriveHost")
{
		subscription_ = this->create_subscription<robot_msgs::msg::DriveMessage>(
				"LalosoftDriveCommand", 10, std::bind(&RobotDriveHost::topic_callback, this, _1));


		RCLCPP_INFO(get_logger(), "Setting severity threshold to DEBUG");
		if (!_motorHat.initialize())
		{
			RCLCPP_ERROR(get_logger(), "Motor HAT failed to Initialize!");
		}
}

private:

	rclcpp::Subscription<robot_msgs::msg::DriveMessage>::SharedPtr subscription_;
	UGeek_Motor_Hat _motorHat;

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
	void topic_callback(const robot_msgs::msg::DriveMessage::SharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(), "Left: '%d', Right: '%d'",msg->left_motor_speed, msg->right_motor_speed);
		setMotor(0, msg->right_motor_speed);
		setMotor(1, msg->left_motor_speed);
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotDriveHost>());
	rclcpp::shutdown();
	return 0;
}
