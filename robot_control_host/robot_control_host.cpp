
#include "robot_control_host.hpp"

RobotControlHost::RobotControlHost() : Node("RobotControlHost"),
armAngles{{arm_angle_()},{arm_angle_()},{arm_angle_()},{arm_angle_()},{arm_angle_()},{arm_angle_()}},
	  count_(0),
	  _pan_horizontal_angle (0),
	  _pan_vertical_angle(0),
	  _pan_horizontal_speed(0),
	  _pan_vertical_speed(0),
	  last_send_horizontal_angle(0),
	  last_send_vertical_angle(0),
	  _speedFactor(0.0F)

{


		drivePublisher = this->create_publisher<lalosoft_robot_msgs::msg::DriveMessage>("LalosoftDriveCommand", 10);

		joySubscription = this->create_subscription<sensor_msgs::msg::Joy>(
				"/joy", 10, std::bind(&RobotControlHost::joy_callback, this, _1));

		pantiltPublisher = this->create_publisher<lalosoft_robot_msgs::msg::PanTilt>("LalosoftPanTilt", 10);
		robotHeartbeatSubscription =  this->create_subscription<lalosoft_robot_msgs::msg::RobotHeartbeat>(
				"LalosoftRobotHeartbeat", 10, std::bind(&RobotControlHost::heartbeatCallback, this, _1));
		armTelemetryPublisher = this->create_publisher<lalosoft_robot_msgs::msg::ArmTelemetry>("LalosoftArmTelemetry", 10);
		timer_ = this->create_wall_timer(
				5000ms, std::bind(&RobotControlHost::control_heartbeat_timer_callback, this));
		pantilt_timer_ = this->create_wall_timer(
				20ms, std::bind(&RobotControlHost::control_message_timer_callback, this));
		_pan_horizontal_angle =110;
		_pan_vertical_angle = 110;
		_speedFactor =1;
}

	float RobotControlHost::range_map(float  x,float  in_min,float  in_max,
			float  out_min,float  out_max)
	{
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	void RobotControlHost::applyAngleSpeed(float& currentSpeed, float speed, float min, float max)
	{
		currentSpeed+=speed;
		if (currentSpeed<min)
		{
			currentSpeed=min;
		} else if (currentSpeed>max)
		{
			currentSpeed=max;
		}
	}

	void RobotControlHost::applyAngleSpeed(int joint_num)
	{
		applyAngleSpeed(armAngles[joint_num].current, armAngles[joint_num].speed, armAngles[joint_num].min, armAngles[joint_num].max);
	}
	void RobotControlHost::sendPanTiltMessage()
	{

		{
			applyAngleSpeed(_pan_horizontal_angle, _pan_horizontal_speed, PAN_MIN_HORIZONTAL_ANGLE, PAN_MAX_HORIZONTAL_ANGLE);
			applyAngleSpeed(_pan_vertical_angle, _pan_vertical_speed, PAN_MIN_VERTICAL_ANGLE, PAN_MAX_VERTICAL_ANGLE);
			_pan_vertical_angle += _pan_vertical_speed;
		}

		int send_horizontal_angle = (int)_pan_horizontal_angle;
		int send_vertical_angle = (int)_pan_vertical_angle;
		auto message = lalosoft_robot_msgs::msg::PanTilt();
		message.hortizontal_angle = send_horizontal_angle;
		message.vertical_angle = send_vertical_angle;
		pantiltPublisher->publish(message);
		if ((last_send_horizontal_angle != send_horizontal_angle)
				|| (last_send_vertical_angle != send_vertical_angle))
		{
			//log
			RCLCPP_INFO(this->get_logger(), "Vertical %d, Horizontal %d.",send_vertical_angle, send_horizontal_angle );
		}
		last_send_horizontal_angle = send_horizontal_angle;
		last_send_vertical_angle = send_vertical_angle;
	}

	void RobotControlHost::sendRobotArmMessage()
	{


		for (size_t joint_num = 0; joint_num<6 ; joint_num++)
		{
			applyAngleSpeed(joint_num);
			if (armAngles[joint_num].current != armAngles[joint_num].last)
			{
			RCLCPP_INFO(this->get_logger(), "Joint %u, Angle %f",joint_num ,armAngles[joint_num].current);
			armAngles[joint_num].last = armAngles[joint_num].current;
			}
		}

		auto message = lalosoft_robot_msgs::msg::ArmTelemetry();
		message.arm_number = 0;
		message.shoulder_hortizontal_angle = armAngles[SHOULDER_HORIZONTAL].current;
		message.shoulder_vertical_angle = armAngles[SHOULDER_VERTICAL].current;
		message.elbow_angle = armAngles[ELBOW].current;
		message.wrist_bend_angle = armAngles[WRIST_BEND].current;
		message.write_rotation_angle = armAngles[WRIST_ROTATION].current;
		message.end_effector_position = armAngles[END_EFFECTOR].current;


		armTelemetryPublisher->publish(message);
	}
	void RobotControlHost::control_message_timer_callback()
	{
		std::lock_guard<std::mutex> lock (_panTiltSpeedLock);
		sendPanTiltMessage();
		sendRobotArmMessage();

	}

	void RobotControlHost::control_heartbeat_timer_callback()
	{
		RCLCPP_INFO(this->get_logger(), "Robot Control Host is Alive.");
	}

	void RobotControlHost::heartbeatCallback(const lalosoft_robot_msgs::msg::RobotHeartbeat::SharedPtr msg)
	{
		RCLCPP_INFO(this->get_logger(), "Recieved Heartbeat from %s", msg->service_name.c_str());
	}

	void RobotControlHost::ProcessDriveMessaging(const sensor_msgs::msg::Joy::SharedPtr msg)
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
		auto message = lalosoft_robot_msgs::msg::DriveMessage();
		static int32_t last_left_motor_speed=0;
		static int32_t last_right_motor_speed=0;
		message.right_motor_speed = R * _speedFactor;
		message.left_motor_speed = L * _speedFactor;

		if (last_left_motor_speed != message.left_motor_speed
				|| last_right_motor_speed != message.right_motor_speed )
		{
			RCLCPP_INFO(this->get_logger(), "Left Motor: %d \t Right Motor: %d", message.left_motor_speed, message.left_motor_speed);
		}
		last_left_motor_speed = message.left_motor_speed;
		last_right_motor_speed = message.right_motor_speed;
		drivePublisher->publish(message);
		RCLCPP_DEBUG(this->get_logger(), "Xraw='%1.3f', Yraw='%1.3f',\n X='%1.3f', Y='%1.3f',\n V='%1.3f', W='%1.3f',\n R='%1.3f', L='%1.3f'",Xraw, Yraw, X, Y, V, W, R, L );
	}

	void RobotControlHost::processSpeedChange(int joint_num, float rawJoyValue)
	{
		if (abs(rawJoyValue)<=.3)
			rawJoyValue=0;
		//SPEED ADJUSTMENT WOULD BE HERE

		armAngles[joint_num].speed = rawJoyValue; //*4.0;
		if (armAngles[joint_num].speed != armAngles[joint_num].lastSpeed )
		{
			RCLCPP_DEBUG(this->get_logger(), "joint_num %d, speed %f", joint_num, armAngles[joint_num].speed);

		}
		armAngles[joint_num].lastSpeed = armAngles[joint_num].speed ;

	}

	void RobotControlHost::zeroAllOtherSpeeds(size_t joint_num1,size_t joint_num2,size_t joint_num3,size_t joint_num4)
	{
		for (size_t joint_num=0; joint_num < armAngles.size(); joint_num++)
		{
			if (joint_num != joint_num1 && joint_num != joint_num2
					 && joint_num != joint_num3 && joint_num != joint_num4)
			{
				armAngles[joint_num].speed=0;
			}
		}
	}

	void RobotControlHost::zeroAllSpeeds()
		{
			for (size_t joint_num=0; joint_num < armAngles.size(); joint_num++)
			{
					armAngles[joint_num].speed=0;
			}
		}

	void RobotControlHost::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
	{
		float L_Xraw, L_Yraw, R_Xraw, R_Yraw;
		R_Xraw = msg->axes[XBOX_ANALOG_RIGHT_JOY_X];
		R_Yraw = msg->axes[XBOX_ANALOG_RIGHT_JOY_Y];
		L_Xraw = msg->axes[XBOX_ANALOG_LEFT_JOY_X];
		L_Yraw = msg->axes[XBOX_ANALOG_LEFT_JOY_Y];

		bool LTrigger =!(msg->axes[XBOX_ANALOG_LEFT_TRIGGER] > .3);
		bool LBumper = msg->buttons[XBOX_DIGITAL_BUTTON_LEFT_BUMPER]==1;
		bool RTrigger = !(msg->axes[XBOX_ANALOG_RIGHT_TRIGGER] > .3);
		bool RBumper = msg->buttons[XBOX_DIGITAL_BUTTON_RIGHT_BUMPER]==1;

		static bool lastLTrigger = false;
		static bool lastLBumper = false;
		static bool lastRTrigger = false;
		static bool lastRBumper = false;
		if (LTrigger !=lastLTrigger || LBumper!=lastLBumper || RTrigger!=lastRTrigger || RBumper!=lastRBumper)
		{
			RCLCPP_INFO(this->get_logger(), "LTrigger %d, LBumper %d, RTrigger %d, RBumper %d",
				LTrigger ? 1 : 0, LBumper ? 1 : 0,RTrigger ? 1 : 0, RBumper ? 1 : 0);
		}
		lastLTrigger = LTrigger;
		lastLBumper = LBumper;
		lastRTrigger =RTrigger;
		lastRBumper = RBumper;
		std::lock_guard<std::mutex> lock (_panTiltSpeedLock);
		zeroAllSpeeds();
		if (LTrigger && !(LBumper || RTrigger || RBumper))
		{  // Arm Movement Shoulder and Elbow and Wrist
			//zeroAllOtherSpeeds(SHOULDER_HORIZONTAL,SHOULDER_VERTICAL,ELBOW, WRIST_BEND );
			if (msg->buttons[XBOX_DIGITAL_BUTTON_LEFT_JOY]==1)
			{
				armAngles[SHOULDER_HORIZONTAL].current=armAngles[SHOULDER_HORIZONTAL].def;
				armAngles[SHOULDER_VERTICAL].current=armAngles[SHOULDER_VERTICAL].def;
			}
			else
			{
				processSpeedChange(SHOULDER_HORIZONTAL, L_Xraw);
				processSpeedChange(SHOULDER_VERTICAL, L_Yraw);
			}
			if (msg->buttons[XBOX_DIGITAL_BUTTON_RIGHT_JOY]==1)
			{
				armAngles[ELBOW].current=armAngles[ELBOW].def;
				armAngles[WRIST_BEND].current=armAngles[WRIST_BEND].def;
			}
			else
			{
				processSpeedChange(ELBOW, R_Xraw);
				processSpeedChange(WRIST_BEND, R_Yraw);
			}
		}
		else if (LBumper && !(LTrigger  || RTrigger || RBumper))
		{ // Arm Movement End Effector
			//zeroAllOtherSpeeds(WRIST_ROTATION,END_EFFECTOR);
			if (msg->buttons[XBOX_DIGITAL_BUTTON_LEFT_JOY]==1)
			{
				armAngles[WRIST_ROTATION].current=armAngles[WRIST_ROTATION].def;
			}
			else
			{
				processSpeedChange(WRIST_ROTATION, L_Xraw);
			}
			if (msg->buttons[XBOX_DIGITAL_BUTTON_RIGHT_JOY]==1)
			{
				armAngles[END_EFFECTOR].current=armAngles[END_EFFECTOR].def;
			}
			else
			{
				processSpeedChange(END_EFFECTOR, -R_Yraw);
			}
		}
		else if ( !(LBumper || LTrigger  || RTrigger || RBumper))
		{ //drive and pan tilt
			if (msg->buttons[XBOX_DIGITAL_BUTTON_RIGHT_JOY]==1)
			{
				_pan_horizontal_angle =110;
				_pan_vertical_angle = 110;
			}
			{
				if (abs(R_Xraw)<=.3)
					R_Xraw=0;
				if (abs(R_Yraw)<=.3)
					R_Yraw=0;
				{
					//SPEED ADJUSTMENT WOULD BE HERE
					_pan_horizontal_speed = R_Xraw*4.0;
					_pan_vertical_speed = R_Yraw*4.0;
				}
			}
			ProcessDriveMessaging(msg);
		}
		if (msg->buttons[XBOX_DIGITAL_BUTTON_DPAD_DOWN]==1)
		{
			if (_speedFactor > SPEED_FACTOR_MIN)
			{
				_speedFactor -=.2;
				RCLCPP_INFO(this->get_logger(), "Speed Factor='%1.2f'", _speedFactor);
			}
		}
		if (msg->buttons[XBOX_DIGITAL_BUTTON_DPAD_UP]==1)
		{
			if (_speedFactor < SPEED_FACTOR_MAX)
			{
				_speedFactor +=.2;
				RCLCPP_INFO(this->get_logger(), "Speed Factor='%1.2f'", _speedFactor);
			}
		}

	}


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotControlHost>());
	rclcpp::shutdown();
	return 0;
}
