
#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "lalosoft_robot_msgs/msg/drive_message.hpp"
#include "lalosoft_robot_msgs/msg/pan_tilt.hpp"
#include "lalosoft_robot_msgs/msg/arm_telemetry.hpp"
#include "lalosoft_robot_msgs/msg/robot_heartbeat.hpp"
#include "xboxc_constants.hpp"

using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
#define SHOULDER_HORIZONTAL 0
#define SHOULDER_VERTICAL 1
#define ELBOW 2
#define WRIST_BEND 3
#define WRIST_ROTATION 4
#define END_EFFECTOR 5

class RobotControlHost : public rclcpp::Node
{
public:
	RobotControlHost();

	typedef struct arm_angle_
	{
		float min;
		float max;
		float def;
		float current;
		float last;
		float speed;
		float lastSpeed;
		arm_angle_()
		: min(0),
		  max(180),
		  def(90),
		  current(def),
		  last(0),
		  speed(0),
		  lastSpeed(0)
		{

		}

	} ArmAngle;



private:

	std::vector<ArmAngle> armAngles;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr pantilt_timer_;
	rclcpp::Publisher<lalosoft_robot_msgs::msg::DriveMessage>::SharedPtr drivePublisher;
	rclcpp::Publisher<lalosoft_robot_msgs::msg::PanTilt>::SharedPtr pantiltPublisher;
	rclcpp::Publisher<lalosoft_robot_msgs::msg::ArmTelemetry>::SharedPtr armTelemetryPublisher;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
	rclcpp::Subscription<lalosoft_robot_msgs::msg::RobotHeartbeat>::SharedPtr robotHeartbeatSubscription;
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
	const float SPEED_FACTOR_MIN = .1;
	const float SPEED_FACTOR_MAX = 1;
	float _speedFactor;


	float range_map(float  x,float  in_min,float  in_max,
			float  out_min,float  out_max);
	void control_message_timer_callback();
	void control_heartbeat_timer_callback();
	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
	void zeroAllOtherSpeeds(size_t joint_num1, size_t joint_num2= 10,size_t joint_num3 = 10,size_t joint_num4 = 10);
	void zeroAllSpeeds();
	void heartbeatCallback(const lalosoft_robot_msgs::msg::RobotHeartbeat::SharedPtr msg);
	void applyAngleSpeed(float& currentSpeed, float speed, float min, float max);
	void applyAngleSpeed(int joint_num);
	void ProcessDriveMessaging(const sensor_msgs::msg::Joy::SharedPtr msg);
	void sendPanTiltMessage();
	void sendRobotArmMessage();
	void processSpeedChange(int joint_num, float rawJoyValue);

} ;
