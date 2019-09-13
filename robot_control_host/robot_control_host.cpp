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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "robot_msgs/msg/drive_message.hpp"
#include "robot_msgs/msg/pan_tilt.hpp"
#include "xboxc_constants.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class RobotControlHost : public rclcpp::Node
{
public:
	RobotControlHost()
: Node("RobotControlHost"), count_(0)
{
		drive_publisher_ = this->create_publisher<robot_msgs::msg::DriveMessage>("LalosoftDriveCommand", 10);

		joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
				"/joy", 10, std::bind(&RobotControlHost::joy_callback, this, _1));

		pantilt_publisher_ = this->create_publisher<robot_msgs::msg::PanTilt>("LalosoftPanTilt", 10);
		timer_ = this->create_wall_timer(
				5000ms, std::bind(&RobotControlHost::timer_callback, this));
		pantilt_timer_ = this->create_wall_timer(
				20ms, std::bind(&RobotControlHost::pantilt_timer_callback, this));
		_pan_horizontal_angle =110;
		_pan_vertical_angle = 110;
}

private:

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr pantilt_timer_;
	rclcpp::Publisher<robot_msgs::msg::DriveMessage>::SharedPtr drive_publisher_;
	rclcpp::Publisher<robot_msgs::msg::PanTilt>::SharedPtr pantilt_publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
	size_t count_;
	std::mutex _panTiltSpeedLock;
	const float PAN_MIN_HORIZONTAL_ANGLE =  0;
	const float PAN_MAX_HORIZONTAL_ANGLE =  180;
	const float PAN_MIN_VERTICAL_ANGLE   =  19;
	const float PAN_MAX_VERTICAL_ANGLE   =  180;
	float _pan_horizontal_angle;
	float _pan_vertical_angle;
	float _pan_horizontal_speed;
	float _pan_vertical_speed;
	int last_send_horizontal_angle;
	int last_send_vertical_angle;
	float range_map(float  x,float  in_min,float  in_max,
			float  out_min,float  out_max)
	{
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	void pantilt_timer_callback()
	{
		std::lock_guard<std::mutex> lock (_panTiltSpeedLock);
		{

			_pan_horizontal_angle+=_pan_horizontal_speed;
			_pan_vertical_angle += _pan_vertical_speed;
		}
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
		int send_horizontal_angle = (int)_pan_horizontal_angle;
		int send_vertical_angle = (int)_pan_vertical_angle;
		auto message = robot_msgs::msg::PanTilt();
		message.hortizontal_angle = send_horizontal_angle;
		message.vertical_angle = send_vertical_angle;
		pantilt_publisher_->publish(message);
		if ((last_send_horizontal_angle != send_horizontal_angle)
				|| (last_send_vertical_angle != send_vertical_angle))
		{
			//log
			RCLCPP_INFO(this->get_logger(), "Vertical %d, Horizontal %d.",send_vertical_angle, send_horizontal_angle );
		}
		last_send_horizontal_angle = send_horizontal_angle;
		last_send_vertical_angle = send_vertical_angle;

	}

	void timer_callback()
	{
		RCLCPP_INFO(this->get_logger(), "Robot Control Host is Alive.");
	}

	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
	{
		float Xraw, Yraw, X, Y, R, L, V, W, pan_Xraw, pan_Yraw;
		pan_Xraw = msg->axes[XBOX_ANALOG_RIGHT_JOY_X];
		pan_Yraw = msg->axes[XBOX_ANALOG_RIGHT_JOY_Y];
		{
			if (abs(pan_Xraw)<=.3)
				pan_Xraw=0;
			if (abs(pan_Yraw)<=.3)
				pan_Yraw=0;
			//SPEED ADJUSTMENT WOULD BE HERE
			std::lock_guard<std::mutex> lock (_panTiltSpeedLock);
			_pan_horizontal_speed = pan_Xraw*4.0;
			_pan_vertical_speed = pan_Yraw*4.0;
			if (msg->buttons[XBOX_DIGITAL_BUTTON_RIGHT_JOY]==1)
			{
				_pan_horizontal_angle =110;
			    _pan_vertical_angle = 110;
			}
		}
		Xraw = msg->axes[XBOX_ANALOG_LEFT_JOY_X];
		Yraw = msg->axes[XBOX_ANALOG_LEFT_JOY_Y];

		//Smooth Load and high end.
		if (abs(Yraw)>=.999)
			Xraw=0;
		if (abs(Yraw)<=.2)
			Yraw=0;
		if (abs(Xraw)>=.999)
			Yraw=0;
		if (abs(Xraw)<=.2)
			Xraw=0;
		//Scale the value
		X = Xraw*100;
		Y = Yraw*100;

		//get the number
		V =(100-abs(X)) * (Y/100) + Y;
		W= (100-abs(Y)) * (X/100) + X;
		R = (V+W) /2;
		L= (V-W)/2;
		auto message = robot_msgs::msg::DriveMessage();
		message.right_motor_speed = R;
		message.left_motor_speed = L;
		drive_publisher_->publish(message);
		RCLCPP_DEBUG(this->get_logger(), "Xraw='%1.3f', Yraw='%1.3f',\n X='%1.3f', Y='%1.3f',\n V='%1.3f', W='%1.3f',\n R='%1.3f', L='%1.3f'",Xraw, Yraw, X, Y, V, W, R, L );
	}


};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotControlHost>());
	rclcpp::shutdown();
	return 0;
}
