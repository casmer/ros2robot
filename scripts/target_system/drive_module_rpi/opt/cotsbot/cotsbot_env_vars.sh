export ROS_DOMAIN_ID=2
export LD_LIBRARY_PATH="/opt/cotsbot/lib:$LD_LIBRARY_PATH"


#now load overrides
if [ -f /opt/cotsbot/cotsbot_env_vars_override.sh ] ; then
	echo "using /opt/cotsbot/cotsbot_env_vars_override.sh"
	. /opt/cotsbot/cotsbot_env_vars_override.sh
fi