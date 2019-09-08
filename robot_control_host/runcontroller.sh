#!/bin/bash
ros2 run joy joy_node &
ros2 run lalosoft_robot_control_host control_host &
