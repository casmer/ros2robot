#!/bin/bash
source /opt/ros/dashing/setup.bash

if [ -f /opt/cotsbot/cotsbot_env_vars_override.sh ] ; then
	. /opt/cotsbot/cotsbot_env_vars_override.sh
else
	. /opt/cotsbot/cotsbot_env_vars.sh
fi
echo "ROS2 setup, domain: $ROS_DOMAIN_ID"

cd /opt/cotsbot/ros_packages
. setup.bash

cd ..
ros2 node list

ros2 run lalosoft_robot_jazzy_drive_host drive_host

