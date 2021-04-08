#include <operations/navigation_server.h>

NavigationServer::NavigationServer(ros::NodeHandle& nh, std::string robot_name)
{
	robot_name_ = robot_name;
	std::string node_name = robot_name + "_navigation_action_server";

	// Initialise the publishers for steering and wheel velocites
	initPublishers(nh, robot_name);
	initSubscribers(nh, robot_name);

	printf("Starting navigation server...\n");

	// Action server
	server_ = new Server(nh, NAVIGATION_ACTIONLIB, boost::bind(&NavigationServer::execute, this, _1), false);
	server_->registerPreemptCallback(boost::bind(&NavigationServer::cancelGoal, this));
	server_->start();

	printf("Navigation server started.\n");

	listener_ = new tf2_ros::TransformListener(buffer_);

	// Initialize the rate limiter to 100 HZ
	update_rate_ = new ros::Rate(100);

	moveRobotWheels(0);
	steerRobot(0);
}

NavigationServer::~NavigationServer()
{
	// Cleanup the TransformListener
	delete listener_;

	// Cleanup the actionlib server
	delete server_;

	// Cleanup the rate limiter
	delete update_rate_;
}

/**
 * @brief Initialise the publishers for wheel velocities
 * 
 */
void NavigationServer::initVelocityPublisher(ros::NodeHandle& nh, const std::string& robot_name)
{
	// front_left_vel_pub_ = nh.advertise<std_msgs::Float64>(CAPRICORN_TOPIC + robot_name + WHEEL_PID + FRONT_LEFT_WHEEL + DESIRED_VELOCITY, 1000);
	// front_right_vel_pub_ = nh.advertise<std_msgs::Float64>(CAPRICORN_TOPIC + robot_name + WHEEL_PID + FRONT_RIGHT_WHEEL + DESIRED_VELOCITY, 1000);
	// back_left_vel_pub_ = nh.advertise<std_msgs::Float64>(CAPRICORN_TOPIC + robot_name + WHEEL_PID + BACK_LEFT_WHEEL + DESIRED_VELOCITY, 1000);
	// back_right_vel_pub_ = nh.advertise<std_msgs::Float64>(CAPRICORN_TOPIC + robot_name + WHEEL_PID + BACK_RIGHT_WHEEL + DESIRED_VELOCITY, 1000);

	front_left_vel_pub_ = nh.advertise<std_msgs::Float64>("/" + robot_name + FRONT_LEFT_WHEEL + VELOCITY_TOPIC, 1000);
	front_right_vel_pub_ = nh.advertise<std_msgs::Float64>("/" + robot_name + FRONT_RIGHT_WHEEL + VELOCITY_TOPIC, 1000);
	back_left_vel_pub_ = nh.advertise<std_msgs::Float64>("/" + robot_name + BACK_LEFT_WHEEL + VELOCITY_TOPIC, 1000);
	back_right_vel_pub_ = nh.advertise<std_msgs::Float64>("/" + robot_name + BACK_RIGHT_WHEEL + VELOCITY_TOPIC, 1000);
}

/**
 * @brief Initialise the publishers for wheel steering angles
 * 
 */
void NavigationServer::initSteerPublisher(ros::NodeHandle& nh, const std::string& robot_name)
{
	front_left_steer_pub_ = nh.advertise<std_msgs::Float64>("/" + robot_name + FRONT_LEFT_WHEEL + STEERING_TOPIC, 1000);
	front_right_steer_pub_ = nh.advertise<std_msgs::Float64>("/" + robot_name + FRONT_RIGHT_WHEEL + STEERING_TOPIC, 1000);
	back_left_steer_pub_ = nh.advertise<std_msgs::Float64>("/" + robot_name + BACK_LEFT_WHEEL + STEERING_TOPIC, 1000);
	back_right_steer_pub_ = nh.advertise<std_msgs::Float64>("/" + robot_name + BACK_RIGHT_WHEEL + STEERING_TOPIC, 1000);
}

/**
 * @brief Initialise publihers used to debug
 * 
 */
void NavigationServer::initDebugPublishers(ros::NodeHandle& nh, const std::string& robot_name)
{
	waypoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>(CAPRICORN_TOPIC + robot_name + "/current_waypoint", 1000);
}

/**
 * @brief Call Publisher initializers
 * 
 */
void NavigationServer::initPublishers(ros::NodeHandle& nh, const std::string& robot_name)
{
	initVelocityPublisher(nh, robot_name);
	initSteerPublisher(nh, robot_name);
	initDebugPublishers(nh, robot_name);
}

/**
 * @brief Subscribes to an odometry topic, and updates the global robot_pose
 * 
 * @param msg The odometry message to process
 */
void NavigationServer::updateRobotPose(const nav_msgs::Odometry::ConstPtr& msg)
{
	std::lock_guard<std::mutex> pose_lock(pose_mutex_);
	robot_pose_.header = msg->header;
	robot_pose_.pose = msg->pose.pose;
	return;
}

geometry_msgs::PoseStamped* NavigationServer::getRobotPose()
{
	std::lock_guard<std::mutex> pose_lock(pose_mutex_);
	return &robot_pose_;
}

/**
 * @brief Initialize the subscriber for robot position
 * 
 */
void NavigationServer::initSubscribers(ros::NodeHandle& nh, std::string& robot_name)
{
	// TODO: Swap this topic from CHEAT_ODOM_TOPIC to the real odom topic. Or, add a flag in the launch file and switch between the two
	update_current_robot_pose_ = nh.subscribe(CAPRICORN_TOPIC + robot_name + CHEAT_ODOM_TOPIC, 1000, &NavigationServer::updateRobotPose, this);
	brake_client_ = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/" + robot_name + BRAKE_ROVER);
}

/*********************************************************************/
/****************** P U B L I S H   M E S S A G E S ******************/
/*********************************************************************/

/**
 * @brief Publish the message over rostopic
 * 
 * @param publisher   publisher over which message will be published
 * @param data        data that will be published over the publisher
 */
void NavigationServer::publishMessage(ros::Publisher& publisher, float data)
{
	std_msgs::Float64 pub_data;
	pub_data.data = data;

	publisher.publish(pub_data);
}

/**
 * @brief Steers the robot wheels for the angles
 * 
 * @param angles Steering angles
	 *                  The vector will be in order:
	 *                  Clockwise from top, starting with FRONT_LEFT
	 * 
	 *          element 0: Front Left Wheel
	 *          element 1: Front Right Wheel
	 *          element 2: Back Right Wheel
	 *          element 3: Back Left Wheel
 */
void NavigationServer::steerRobot(const std::vector<double>& angles)
{
	publishMessage(front_left_steer_pub_, angles.at(0));
	publishMessage(front_right_steer_pub_, angles.at(1));
	publishMessage(back_right_steer_pub_, angles.at(2));
	publishMessage(back_left_steer_pub_, angles.at(3));

	ros::Duration(0.5).sleep();
}

/**
 * @brief Steers the robot wheels for the angles
 * 
 * @param angle Angles at which the robot wheels will be steered
 */
void NavigationServer::steerRobot(const double angle)
{
	publishMessage(front_left_steer_pub_, angle);
	publishMessage(front_right_steer_pub_, angle);
	publishMessage(back_right_steer_pub_, angle);
	publishMessage(back_left_steer_pub_, angle);
}

/**
 * @brief Move robot wheels with the given velocities
 * 
 * @param velocity Wheel Velocities
	 *                  The vector will be in order:
	 *                  Clockwise from top, starting with FRONT_LEFT
	 * 
	 *          element 0: Front Left Wheel
	 *          element 1: Front Right Wheel
	 *          element 2: Back Right Wheel
	 *          element 3: Back Left Wheel
 */
void NavigationServer::moveRobotWheels(const std::vector<double> velocity)
{
	std::vector<double> angular_vels;

	for(int i = 0; i < velocity.size(); i++)
	{
		angular_vels.push_back(NavigationAlgo::linearToAngularVelocity(velocity.at(i)));
	}

	publishMessage(front_left_vel_pub_, angular_vels.at(0));
	publishMessage(front_right_vel_pub_, angular_vels.at(1));
	publishMessage(back_right_vel_pub_, angular_vels.at(2));
	publishMessage(back_left_vel_pub_, angular_vels.at(3));
}

/**
 * @brief Move robot wheels with the given velocity 
 * 
 * @param velocity velocity for the wheels
 */
void NavigationServer::moveRobotWheels(const double velocity)
{
	double angular_velocity = NavigationAlgo::linearToAngularVelocity(velocity);

	publishMessage(front_left_vel_pub_, angular_velocity);
	publishMessage(front_right_vel_pub_, angular_velocity);
	publishMessage(back_left_vel_pub_, angular_velocity);
	publishMessage(back_right_vel_pub_, angular_velocity);
}

/*******************************************************************/
/****************** P U B L I S H E R   L O G I C ******************/
/*******************************************************************/

operations::TrajectoryWithVelocities* NavigationServer::sendGoalToPlanner(const operations::NavigationGoalConstPtr& goal)
{
	operations::TrajectoryWithVelocities *traj = new operations::TrajectoryWithVelocities();

	// Temporary, replace with service call once the planner is complete
	geometry_msgs::PoseStamped goal_pose = goal->pose;
	std_msgs::Float64 speed;
	speed.data = BASE_DRIVE_SPEED;

	traj->waypoints.push_back(goal_pose);
	traj->velocities.push_back(speed);

	return traj;
}

void NavigationServer::brakeRobot(bool brake)
{
	srcp2_msgs::BrakeRoverSrv srv;

	if(brake)
		srv.request.brake_force = 1000;
	else
		srv.request.brake_force = 0;

	brake_client_.call(srv);
}

bool NavigationServer::rotateWheels(const geometry_msgs::PoseStamped& target_robot_pose)
{
	brakeRobot(0);
	double delta_heading = NavigationAlgo::changeInHeading(*getRobotPose(), target_robot_pose, robot_name_, buffer_);
	printf("Steering to %frad\n", delta_heading);
	steerRobot(delta_heading);
	ros::Duration(0.5).sleep();
	return true;
}

bool NavigationServer::rotateRobot(const geometry_msgs::PoseStamped& target_robot_pose)
{
	brakeRobot(false);

	// For these function calls, a point at (0,0,0) represents the center of the robot. For a turn in place, this is what we want.
	geometry_msgs::Point center_of_robot;

	std::vector<double> wheel_angles = NavigationAlgo::getSteeringAnglesRadialTurn(center_of_robot);
	std::vector<double> wheel_speeds_right = NavigationAlgo::getDrivingVelocitiesRadialTurn(center_of_robot, -BASE_SPIN_SPEED);
	std::vector<double> wheel_speeds_left = NavigationAlgo::getDrivingVelocitiesRadialTurn(center_of_robot, BASE_SPIN_SPEED);

	// Save starting robot pose to track the change in heading
	geometry_msgs::PoseStamped starting_pose = *getRobotPose();

	double delta_heading = NavigationAlgo::changeInHeading(starting_pose, target_robot_pose, robot_name_, buffer_);
	
	if (abs(delta_heading) <= ANGLE_EPSILON)
	{
		return true;
	}

	printf("Turning %frad\n", delta_heading);
	steerRobot(wheel_angles);

	// While we have not turned the desired amount
	while (abs(NavigationAlgo::changeInHeading(starting_pose, target_robot_pose, robot_name_, buffer_)) > ANGLE_EPSILON && ros::ok())
	{
		printf("Heading error: %f\n", abs(NavigationAlgo::changeInHeading(starting_pose, target_robot_pose, robot_name_, buffer_)));

		// target_robot_pose in the robot's frame of reference
		geometry_msgs::PoseStamped target_in_robot_frame = target_robot_pose;
		NavigationAlgo::transformPose(target_in_robot_frame, robot_name_ + ROBOT_CHASSIS, buffer_, 0.1);
		
		waypoint_pub_.publish(target_in_robot_frame);

		if(manual_driving_)
		{
			return false;
		}

		//printf("Heading remaining: %frad\n", abs(NavigationAlgo::changeInHeading(&starting_pose, target_robot_pose, robot_name, &buffer)));

		if (delta_heading < 0)
		{
			// Turn clockwise	
			moveRobotWheels(wheel_speeds_right);
		}
		else if (delta_heading > 0)
		{
			// Turn counter-clockwise
			moveRobotWheels(wheel_speeds_left);
		}

		// Allow ROS to catch up and update our subscribers
		ros::spinOnce();

		// Slow this loop down a bit
		update_rate_->sleep();
	}

	printf("Done rotating\n");

	// Stop moving the robot after we are done moving
	moveRobotWheels(0);

	// Reset steering of wheels
	steerRobot(0);

	brakeRobot(true);

	return true;
}

bool NavigationServer::driveDistance(double delta_distance)
{
	brakeRobot(false);
	printf("Driving forwards %fm\n", delta_distance);

	// Save the starting robot pose so we can track delta distance
	geometry_msgs::PoseStamped starting_pose = *getRobotPose();

	// While we have not traveled the
	while (abs(NavigationAlgo::changeInPosition(starting_pose, *getRobotPose()) - delta_distance) > DIST_EPSILON && ros::ok())
	{
		if(manual_driving_)
		{
			return false;
		}

		// Move the wheels forward at a constant speed
		moveRobotWheels(BASE_DRIVE_SPEED);

		// Allow ROS to catch up and update our subscribers
		ros::spinOnce();

		// Slow this loop down a bit
		update_rate_->sleep();
	}

	printf("Done driving forwards\n");

	// Stop moving the robot after we are done moving
	moveRobotWheels(0);

	brakeRobot(true);

	return true;
}

void NavigationServer::automaticDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	printf("Auto drive\n");

	//Forward goal to local planner, and save the returned trajectory
	operations::TrajectoryWithVelocities *trajectory = sendGoalToPlanner(goal);

	//geometry_msgs::PoseStamped final_pose = buffer_.transform(goal->pose, MAP, ros::Duration(0.1));
	geometry_msgs::PoseStamped final_pose = goal->pose;

	NavigationAlgo::transformPose(final_pose, MAP, buffer_, 0.1);

	//Loop over trajectories
	for (int i = 0; i < trajectory->waypoints.size(); i++)
	{
		if(manual_driving_)
		{
			ROS_ERROR_STREAM("Overridden by manual driving! Exiting.\n");
			operations::NavigationResult res;
			res.result = NAV_RESULT::INTERRUPTED;
			action_server->setSucceeded(res);

			// Cleanup trajectory
			delete trajectory;

			return;
		}

		printf("Going to waypoint %d\n", i);

		//Extract the waypoint and desired velocity
		geometry_msgs::PoseStamped current_waypoint = trajectory->waypoints[i];
		float current_velocity = trajectory->velocities[i].data;

		current_waypoint.header.stamp = ros::Time(0);

		NavigationAlgo::transformPose(current_waypoint, MAP, buffer_, 0.1);

		// Needed, otherwise we get extrapolation into the past
		current_waypoint.header.stamp = ros::Time(0);

		//Turn wheels to heading
		printf("Rotating wheels\n");
		bool turned_successfully = rotateWheels(current_waypoint);

		if (!turned_successfully)
		{
			operations::NavigationResult res;

			if(manual_driving_)
			{
				ROS_ERROR_STREAM("Overridden by manual driving! Exiting.\n");
				res.result = NAV_RESULT::INTERRUPTED;
			}
			else
			{
				//AAAH ERROR
				ROS_ERROR_STREAM("Turn to waypoint " << i << " did not succeed. Exiting.\n");
				res.result = NAV_RESULT::FAILED;
				
			}
			action_server->setSucceeded(res);

			// Cleanup trajectory
			delete trajectory;

			return;
		}

		//Get current pose + position from odometry
		geometry_msgs::PoseStamped current_robot_pose = *getRobotPose();

		//Calculate delta distance
		float delta_distance = NavigationAlgo::changeInPosition(current_robot_pose, current_waypoint);

		//Drive to goal
		printf("Going the distance, going for speed\n");
		bool drove_successfully = driveDistance(delta_distance);

		if (!drove_successfully)
		{
			operations::NavigationResult res;
			if(manual_driving_)
			{
				ROS_ERROR_STREAM("Overridden by manual driving! Exiting.\n");
				res.result = NAV_RESULT::INTERRUPTED;
			} else 
			{
				//AAAH ERROR
				ROS_ERROR_STREAM("Drive to waypoint " << i << " did not succeed.\n");
				res.result = NAV_RESULT::FAILED;
			}
			action_server->setSucceeded(res);
			
			// Cleanup trajectory
			delete trajectory;

			return;
		}
	}

	geometry_msgs::PoseStamped current_robot_pose = *getRobotPose();

	// The final pose is on top of the robot, we only care about orientation
	final_pose.pose.position.x = current_robot_pose.pose.position.x;
	final_pose.pose.position.y = current_robot_pose.pose.position.y;

	final_pose.header.stamp = ros::Time(0);

	printf("Final rotate\n");

	//Turn to heading
	bool turned_successfully = rotateRobot(final_pose);

	if (!turned_successfully)
	{
		operations::NavigationResult res;

		if(manual_driving_)
		{
			ROS_ERROR_STREAM("Overridden by manual driving! Exiting.\n");
			res.result = NAV_RESULT::INTERRUPTED;
		}
		else
		{
			//AAAH ERROR
			ROS_ERROR_STREAM("Final turn did not succeed. Exiting.\n");
			res.result = NAV_RESULT::FAILED;
			
		}
		action_server->setSucceeded(res);
	}
	
	// Cleanup trajectory
	delete trajectory;

	printf("Finished automatic goal!\n");

	brakeRobot(true);

	operations::NavigationResult res;
	res.result = NAV_RESULT::SUCCESS;
	action_server->setSucceeded(res);

	printf("setSucceeded on server_\n");
}

void NavigationServer::linearDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	printf("Manual drive: Linear velocity\n");
	brakeRobot(false);
	steerRobot(goal->direction);
	moveRobotWheels(goal->forward_velocity);

	if(0 == goal->forward_velocity)
	{
		printf("0 linear, braking\n");
		brakeRobot(true);
	}
	
	operations::NavigationResult res;
	res.result = NAV_RESULT::SUCCESS;
	action_server->setSucceeded(res);
	return;
}

void NavigationServer::angularDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	printf("Manual drive: Angular velocity\n");
	brakeRobot(false);
	double angular_velocity = goal->angular_velocity;

	std::vector<double> wheel_angles = {-M_PI/4, M_PI/4, -M_PI/4, M_PI/4};
	std::vector<double> wheel_speeds = {-angular_velocity, angular_velocity, angular_velocity, -angular_velocity};

	steerRobot(wheel_angles);
	moveRobotWheels(wheel_speeds);
	
	operations::NavigationResult res;
	res.result = NAV_RESULT::SUCCESS;
	action_server->setSucceeded(res);
	return;
}

void NavigationServer::revolveDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	printf("Revolve drive\n");

	brakeRobot(false);

	geometry_msgs::PointStamped revolve_about = goal->point;

	NavigationAlgo::transformPoint(revolve_about, robot_name_ + ROBOT_CHASSIS, buffer_, 0.1);

	std::vector<double> angles = NavigationAlgo::getSteeringAnglesRadialTurn(revolve_about.point);
	std::vector<double> speeds = NavigationAlgo::getDrivingVelocitiesRadialTurn(revolve_about.point, goal->forward_velocity);

	steerRobot(angles);
	moveRobotWheels(speeds);

	operations::NavigationResult res;
	res.result = NAV_RESULT::SUCCESS;
	action_server->setSucceeded(res);
	return;
}

void NavigationServer::spiralDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	printf("Spiral drive: Spiral Away!\n");

	ros::Duration(5).sleep();

	//TODO: actually make it spiral

	operations::NavigationResult res;
	res.result = NAV_RESULT::SUCCESS;
	action_server->setSucceeded(res);
	return;
}

void NavigationServer::followDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	printf("Follow drive: Following!\n");

	ros::Duration(5).sleep();
	
	//TODO: actually make it follow the thing

	operations::NavigationResult res;
	res.result = NAV_RESULT::SUCCESS;
	action_server->setSucceeded(res);
	return;
}

void NavigationServer::execute(const operations::NavigationGoalConstPtr &goal)
{
    printf("Received NavigationGoal, dispatching\n");

	switch(goal->drive_mode)
	{
		case NAV_TYPE::MANUAL:

			manual_driving_ = true;
			if(goal->angular_velocity != 0)
			{
				angularDriving(goal, server_);
			}
			else
			{
				linearDriving(goal, server_);
			}		

			break;
		
		case NAV_TYPE::GOAL:

			manual_driving_ = false;
			automaticDriving(goal, server_);

			break;

		case NAV_TYPE::REVOLVE:
			manual_driving_ = false;
			revolveDriving(goal, server_);

			break;
		
		case NAV_TYPE::SPIRAL:

			manual_driving_ = false;
			spiralDriving(goal, server_);

			break;
		
		case NAV_TYPE::FOLLOW:

			manual_driving_ = false;
			followDriving(goal, server_);

			break;

		default:
            ROS_ERROR_STREAM(robot_name_ + " encountered an unknown driving mode!");
            break;
	}
}

void NavigationServer::cancelGoal()
{
	manual_driving_ = true;
	printf("Clearing current goal, got a new one\n");
}
