#include "robot_ptz_host.h"


using std::placeholders::_1;
using namespace std::chrono_literals;

RobotArmHost ::RobotArmHost() : Node("RobotArmHost"),
		settingsFilename("/opt/cotsbot/settings.ini"),
		ptzSettings(settingsFilename),
		i2cDeviceName(""),
		i2cPwmDeviceAddress(0),
		_pan_horizontal_angle(0),
		_pan_vertical_angle(0)

{
	RCLCPP_INFO(get_logger(), "Starting PWM Host");

	loadSettings();
	wiringPiSetup();

	if (!_pca6895.initialize(i2cDeviceName.c_str(), i2cPwmDeviceAddress))
	{
		RCLCPP_ERROR(get_logger(), "PWM controller failed to Initialize!");
	}


	pantilt_subscription_ = this->create_subscription<lalosoft_robot_msgs::msg::PanTilt>(
			"LalosoftPanTilt", 10, std::bind(&RobotArmHost::pantilt_callback, this, _1));
	armtelemetry_subscription_ = this->create_subscription<lalosoft_robot_msgs::msg::ArmTelemetry>(
			"LalosoftArmTelemetry", 10, std::bind(&RobotArmHost::armtelemetry_callback, this, _1));
	robotHeartbeatPublisher = this->create_publisher<lalosoft_robot_msgs::msg::RobotHeartbeat>("LalosoftRobotHeartbeat", 10);
	hearbeatTimer = this->create_wall_timer(
			5000ms, std::bind(&RobotArmHost::heartbeatTimerCallback, this));


}

void RobotArmHost::heartbeatTimerCallback()
{
	auto message = lalosoft_robot_msgs::msg::RobotHeartbeat();
	message.service_name = "PTZ Host";
	message.robot_number = 1;
	robotHeartbeatPublisher->publish(message);
	RCLCPP_INFO(this->get_logger(), "Robot PTZ Host is Alive.");
}

void RobotArmHost::loadSettings()
{
	i2cDeviceName = ptzSettings["Pwm Board"]["i2c_device"].as<std::string>();
	i2cPwmDeviceAddress  = ptzSettings["Pwm Board"]["i2c_address"].as<int>();
	RCLCPP_INFO(get_logger(), "Pwm dev: %s, addr: %x", i2cDeviceName.c_str(), i2cPwmDeviceAddress);
	armPwmChannels.shoulder_hortizontal =	ptzSettings["Arm Pwm Channels"]["shoulder_hortizontal"].as<int>();
	armPwmChannels.shoulder_vertical =	ptzSettings["Arm Pwm Channels"]["shoulder_vertical"].as<int>();
	armPwmChannels.elbow =	ptzSettings["Arm Pwm Channels"]["elbow"].as<int>();
	armPwmChannels.wrist_bend =	ptzSettings["Arm Pwm Channels"]["wrist_bend"].as<int>();
	armPwmChannels.wrist_rotation =	ptzSettings["Arm Pwm Channels"]["wrist_rotation"].as<int>();
	armPwmChannels.end_effector =	ptzSettings["Arm Pwm Channels"]["end_effector"].as<int>();


	RCLCPP_INFO(get_logger(), "Pwm channels:");
	RCLCPP_INFO(get_logger(), "shoulder_hortizontal = %d:", armPwmChannels.shoulder_hortizontal);
	RCLCPP_INFO(get_logger(), "shoulder_vertical = %d:", armPwmChannels.shoulder_vertical);
	RCLCPP_INFO(get_logger(), "elbow = %d:", armPwmChannels.elbow);
	RCLCPP_INFO(get_logger(), "wrist_bend = %d:", armPwmChannels.wrist_bend);
	RCLCPP_INFO(get_logger(), "wrist_rotation = %d:", armPwmChannels.wrist_rotation);
	RCLCPP_INFO(get_logger(), "end_effector = %d:", armPwmChannels.end_effector);

	armAngles.shoulder_hortizontal_angle.min = ptzSettings["Arm Min Angles"]["shoulder_hortizontal"].as<int>();
	armAngles.shoulder_vertical_angle.min = ptzSettings["Arm Min Angles"]["shoulder_vertical"].as<int>();
	armAngles.elbow_angle.min = ptzSettings["Arm Min Angles"]["elbow"].as<int>();
	armAngles.wrist_bend_angle.min = ptzSettings["Arm Min Angles"]["wrist_bend"].as<int>();
	armAngles.wrist_rotation_angle.min = ptzSettings["Arm Min Angles"]["wrist_rotation"].as<int>();
	armAngles.end_effector_position.min = ptzSettings["Arm Min Angles"]["end_effector"].as<int>();

	armAngles.shoulder_hortizontal_angle.max = ptzSettings["Arm Max Angles"]["shoulder_hortizontal"].as<int>();
	armAngles.shoulder_vertical_angle.max = ptzSettings["Arm Max Angles"]["shoulder_vertical"].as<int>();
	armAngles.elbow_angle.max = ptzSettings["Arm Max Angles"]["elbow"].as<int>();
	armAngles.wrist_bend_angle.max = ptzSettings["Arm Max Angles"]["wrist_bend"].as<int>();
	armAngles.wrist_rotation_angle.max = ptzSettings["Arm Max Angles"]["wrist_rotation"].as<int>();
	armAngles.end_effector_position.max = ptzSettings["Arm Max Angles"]["end_effector"].as<int>();

	armAngles.shoulder_hortizontal_angle.def = ptzSettings["Arm Default Pose"]["shoulder_hortizontal"].as<int>();
	armAngles.shoulder_vertical_angle.def = ptzSettings["Arm Default Pose"]["shoulder_vertical"].as<int>();
	armAngles.elbow_angle.def = ptzSettings["Arm Default Pose"]["elbow"].as<int>();
	armAngles.wrist_bend_angle.def = ptzSettings["Arm Default Pose"]["wrist_bend"].as<int>();
	armAngles.wrist_rotation_angle.def = ptzSettings["Arm Default Pose"]["wrist_rotation"].as<int>();
	armAngles.end_effector_position.def = ptzSettings["Arm Default Pose"]["end_effector"].as<int>();

	armAngles.shoulder_hortizontal_angle.current = ptzSettings["Arm Default Pose"]["shoulder_hortizontal"].as<int>();
	armAngles.shoulder_vertical_angle.current = ptzSettings["Arm Default Pose"]["shoulder_vertical"].as<int>();
	armAngles.elbow_angle.current = ptzSettings["Arm Default Pose"]["elbow"].as<int>();
	armAngles.wrist_bend_angle.current = ptzSettings["Arm Default Pose"]["wrist_bend"].as<int>();
	armAngles.wrist_rotation_angle.current = ptzSettings["Arm Default Pose"]["wrist_rotation"].as<int>();
	armAngles.end_effector_position.current = ptzSettings["Arm Default Pose"]["end_effector"].as<int>();

	_pca6895.setPwmAsAngle(armPwmChannels.shoulder_hortizontal , armAngles.shoulder_hortizontal_angle.current);
	_pca6895.setPwmAsAngle(armPwmChannels.shoulder_vertical , armAngles.shoulder_vertical_angle.current);
	_pca6895.setPwmAsAngle(armPwmChannels.elbow , armAngles.elbow_angle.current);
	_pca6895.setPwmAsAngle(armPwmChannels.wrist_bend, armAngles.wrist_bend_angle.current);
	_pca6895.setPwmAsAngle(armPwmChannels.wrist_rotation, armAngles.wrist_rotation_angle.current);
	_pca6895.setPwmAsAngle(armPwmChannels.end_effector, armAngles.end_effector_position.current);
}
RobotArmHost ::~RobotArmHost()
{

}



void RobotArmHost ::processArmChange(int angle, ArmAngle& armAngle, int pwmChannel, const char* propertyName)
{
	int newAdjustedAngle = angle;


	if (newAdjustedAngle<armAngle.min)
	{
		newAdjustedAngle=armAngle.min;
	}
	else if (newAdjustedAngle>armAngle.max)
	{
		newAdjustedAngle=armAngle.max;
	}

	if (armAngle.current != newAdjustedAngle)
	{
		RCLCPP_INFO(this->get_logger(), "%s: '%d'->'%d'",
				propertyName, armAngle.current , newAdjustedAngle);
		armAngle.current = newAdjustedAngle;
		_pca6895.setPwmAsAngle(pwmChannel, newAdjustedAngle);
	}

}

void RobotArmHost ::armtelemetry_callback(const lalosoft_robot_msgs::msg::ArmTelemetry::SharedPtr message)
{

	processArmChange(message->shoulder_hortizontal_angle,armAngles.shoulder_hortizontal_angle,
			armPwmChannels.shoulder_hortizontal, "shoulder_hortizontal_angle");
	processArmChange(message->shoulder_vertical_angle,armAngles.shoulder_vertical_angle,
			armPwmChannels.shoulder_vertical, "shoulder_vertical_angle");
	processArmChange(message->elbow_angle,armAngles.elbow_angle,
			armPwmChannels.elbow, "elbow_angle");
	processArmChange(message->wrist_bend_angle,armAngles.wrist_bend_angle,
			armPwmChannels.wrist_bend, "wrist_bend_angle");
	processArmChange(message->write_rotation_angle,armAngles.wrist_rotation_angle,
			armPwmChannels.wrist_rotation, "wrist_rotation_angle");
	processArmChange(message->end_effector_position,armAngles.end_effector_position,
			armPwmChannels.end_effector, "end_effector_position");
}

void RobotArmHost ::pantilt_callback(const lalosoft_robot_msgs::msg::PanTilt::SharedPtr message)
{

	int newPanHorizontalAngle = message->hortizontal_angle;
	int newPanVerticalAngle = message->vertical_angle;

	if (newPanHorizontalAngle<PAN_MIN_HORIZONTAL_ANGLE)
	{
		newPanHorizontalAngle=PAN_MIN_HORIZONTAL_ANGLE;
	} else if (newPanHorizontalAngle>PAN_MAX_HORIZONTAL_ANGLE)
	{
		newPanHorizontalAngle=PAN_MAX_HORIZONTAL_ANGLE;
	}
	if (newPanVerticalAngle<PAN_MIN_VERTICAL_ANGLE)
	{
		newPanVerticalAngle=PAN_MIN_VERTICAL_ANGLE;
	} else if (newPanVerticalAngle>PAN_MAX_VERTICAL_ANGLE)
	{
		newPanVerticalAngle=PAN_MAX_VERTICAL_ANGLE;
	}

	if ((_pan_horizontal_angle != newPanHorizontalAngle) ||
			(_pan_vertical_angle != newPanVerticalAngle))
	{
		RCLCPP_INFO(this->get_logger(), "hortizontal_angle: '%d', vertical_angle: '%d'",message->hortizontal_angle, message->vertical_angle);

		_pan_horizontal_angle = newPanHorizontalAngle;
		_pan_vertical_angle=	newPanVerticalAngle;
		_pca6895.setPwmAsAngle(0, _pan_horizontal_angle);
		_pca6895.setPwmAsAngle(1, _pan_vertical_angle);
	}

}



int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotArmHost>());
	rclcpp::shutdown();
	return 0;
}
