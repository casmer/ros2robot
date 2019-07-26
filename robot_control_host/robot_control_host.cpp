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
    publisher_ = this->create_publisher<robot_msgs::msg::DriveMessage>("LalosoftDriveCommand", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
          "/joy", 10, std::bind(&RobotControlHost::joy_callback, this, _1));

    timer_ = this->create_wall_timer(
      5000ms, std::bind(&RobotControlHost::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // auto message = robot_msgs::msg::DriveMessage();

//    message.data = "Hello, world! " + std::to_string(count_++);
//    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//    publisher_->publish(message);
	  RCLCPP_INFO(this->get_logger(), "Robot Control Host is Alive.");
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
	{
	  float Xraw, Yraw, X, Y, R, L, V, W;
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
	  publisher_->publish(message);
	  RCLCPP_DEBUG(this->get_logger(), "Xraw='%1.3f', Yraw='%1.3f',\n X='%1.3f', Y='%1.3f',\n V='%1.3f', W='%1.3f',\n R='%1.3f', L='%1.3f'",Xraw, Yraw, X, Y, V, W, R, L );
	}

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<robot_msgs::msg::DriveMessage>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotControlHost>());
  rclcpp::shutdown();
  return 0;
}
