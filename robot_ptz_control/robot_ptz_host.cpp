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
#include "wiringPi.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RobotDriveHost : public rclcpp::Node
{
public:
	RobotDriveHost()
: Node("RobotDriveHost")
{

		pantilt_subscription_ = this->create_subscription<lalosoft_robot_msgs::msg::PanTilt>(
				"LalosoftPanTilt", 10, std::bind(&RobotDriveHost::pantilt_callback, this, _1));


		RCLCPP_INFO(get_logger(), "Setting severity threshold to DEBUG");
		if (!_pca6895.initialize())
		{
			RCLCPP_ERROR(get_logger(), "PWM controller failed to Initialize!");
		}
		wiringPiSetup();
		RCLCPP_INFO(get_logger(), "Setting up pin");
		pinMode(25, OUTPUT);
		pullUpDnControl(25, PUD_UP );
		digitalWrite(25, LOW);
		RCLCPP_INFO(get_logger(), "Setting up pin");



}
	~RobotDriveHost()
	{

	}
	const int PAN_MIN_HORIZONTAL_ANGLE = 0;
	const int PAN_MAX_HORIZONTAL_ANGLE =  180;
	const int PAN_MIN_VERTICAL_ANGLE   = 0;
	const int PAN_MAX_VERTICAL_ANGLE   =  180;
private:

	rclcpp::Subscription<lalosoft_robot_msgs::msg::PanTilt>::SharedPtr pantilt_subscription_;
	PCA6895 _pca6895;


	int _pan_horizontal_angle;
	int _pan_vertical_angle;


	int range_map(int  x,int  in_min,int  in_max,
			int  out_min,int  out_max)
	{
	       return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}


	void pantilt_callback(const lalosoft_robot_msgs::msg::PanTilt::SharedPtr message)
	{
		if ((_pan_horizontal_angle != message->hortizontal_angle) ||
				(_pan_vertical_angle != message->vertical_angle))
		{
			RCLCPP_INFO(this->get_logger(), "hortizontal_angle: '%d', vertical_angle: '%d'",message->hortizontal_angle, message->vertical_angle);
		}

		_pan_horizontal_angle = message->hortizontal_angle;
		_pan_vertical_angle = message->vertical_angle;


		if (_pan_horizontal_angle<PAN_MIN_HORIZONTAL_ANGLE)
		{
			_pan_horizontal_angle=PAN_MIN_HORIZONTAL_ANGLE;
		} else if (_pan_horizontal_angle>PAN_MAX_HORIZONTAL_ANGLE)
		{
			_pan_horizontal_angle=PAN_MAX_HORIZONTAL_ANGLE;
		}
		if (_pan_vertical_angle<PAN_MIN_VERTICAL_ANGLE)
		{
			_pan_vertical_angle=PAN_MIN_VERTICAL_ANGLE;
		} else if (_pan_vertical_angle>PAN_MAX_VERTICAL_ANGLE)
		{
			_pan_vertical_angle=PAN_MAX_VERTICAL_ANGLE;
		}

		_pca6895.setPwmAsAngle(0, _pan_horizontal_angle);
		_pca6895.setPwmAsAngle(1, _pan_vertical_angle);
	}


};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotDriveHost>());
	rclcpp::shutdown();
	return 0;
}
