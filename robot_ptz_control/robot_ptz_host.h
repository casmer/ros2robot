#include <memory>
#include <chrono>
#include <mutex>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "pca6895.hpp"
#include "mdds30.hpp"
#include "lalosoft_robot_msgs/msg/drive_message.hpp"
#include "lalosoft_robot_msgs/msg/pan_tilt.hpp"
#include "lalosoft_robot_msgs/msg/arm_telemetry.hpp"
#include "lalosoft_robot_msgs/msg/robot_heartbeat.hpp"
#include "wiringPi.h"
#include "inicpp.h"


using std::placeholders::_1;
using namespace std::chrono_literals;



class RobotArmHost : public rclcpp::Node
{
public:
	RobotArmHost();
	~RobotArmHost();
	const int PAN_MIN_HORIZONTAL_ANGLE = 0;
	const int PAN_MAX_HORIZONTAL_ANGLE =  180;
	const int PAN_MIN_VERTICAL_ANGLE   = 0;
	const int PAN_MAX_VERTICAL_ANGLE   =  180;



	typedef struct arm_angle_
	{
		int min;
		int max;
		int def;
		int current;
		arm_angle_()
		: min(0),
		  max(180),
		  def(90),
		  current(def)
		{

		}

	} ArmAngle;

	typedef struct arm_angles_
	{
		int arm_number;
		ArmAngle shoulder_hortizontal_angle;
		ArmAngle shoulder_vertical_angle;
		ArmAngle elbow_angle;
		ArmAngle wrist_bend_angle;
		ArmAngle wrist_rotation_angle;
		ArmAngle end_effector_position;

		arm_angles_() :
			arm_number(0),
			shoulder_hortizontal_angle(),
			shoulder_vertical_angle(),
			elbow_angle(),
			wrist_bend_angle(),
			wrist_rotation_angle(),
			end_effector_position()
		{

		}
	} ArmAngles;

	typedef struct arm_pwm_channels
	{
		int arm_number;
		int shoulder_hortizontal;
		int shoulder_vertical;
		int elbow;
		int wrist_bend;
		int wrist_rotation;
		int end_effector;

		arm_pwm_channels() :
			arm_number(0),
			shoulder_hortizontal(-1),
			shoulder_vertical(-1),
			elbow(-1),
			wrist_bend(-1),
			wrist_rotation(-1),
			end_effector(-1)
		{

		}
	} ArmPwmChannels;
private:
	ArmAngles armAngles;
	ArmPwmChannels armPwmChannels;

	rclcpp::Subscription<lalosoft_robot_msgs::msg::PanTilt>::SharedPtr pantilt_subscription_;
	rclcpp::Subscription<lalosoft_robot_msgs::msg::ArmTelemetry>::SharedPtr armtelemetry_subscription_;
	rclcpp::Publisher<lalosoft_robot_msgs::msg::RobotHeartbeat>::SharedPtr robotHeartbeatPublisher;
	rclcpp::TimerBase::SharedPtr hearbeatTimer;
	PCA6895 _pca6895;
	std::string settingsFilename;
	ini::IniFile ptzSettings;
	std::string i2cDeviceName;
	int i2cPwmDeviceAddress;

	int _pan_horizontal_angle;
	int _pan_vertical_angle;


	int range_map(int  x,int  in_min,int  in_max,
			int  out_min,int  out_max)
	{
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	void processArmChange(int angle, ArmAngle& armAngle, int pwmChannel, const char* propertyName);
	void loadSettings();
	void heartbeatTimerCallback();
	void armtelemetry_callback(const lalosoft_robot_msgs::msg::ArmTelemetry::SharedPtr message);

	void pantilt_callback(const lalosoft_robot_msgs::msg::PanTilt::SharedPtr message);



};
