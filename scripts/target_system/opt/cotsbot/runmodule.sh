#!/bin/bash
source /opt/ros/dashing/setup.bash

export ROS_DOMAIN_ID=2
echo "ROS2 setup, domain: $ROS_DOMAIN_ID"

cd /home/robot/install
. setup.bash

cd ..
ros2 node list

ros2 run lalosoft_robot_drive_host  drive_host

