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
    publisher_ = this->create_publisher<std_msgs::msg::String>("LalosoftDriveCommand", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
          "/joy", 10, std::bind(&RobotControlHost::joy_callback, this, _1));


    timer_ = this->create_wall_timer(
      500ms, std::bind(&RobotControlHost::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
	{
	  RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->axes[0]);
	}

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
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
