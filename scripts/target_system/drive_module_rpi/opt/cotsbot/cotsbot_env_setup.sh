source /opt/ros/dashing/setup.bash

echo "loading /opt/cotsbot/cotsbot_env_vars.sh"
. /opt/cotsbot/cotsbot_env_vars.sh
	

echo "ROS2 setup, domain: $ROS_DOMAIN_ID"

cd /opt/cotsbot/ros_packages
. setup.bash

ros2 node list