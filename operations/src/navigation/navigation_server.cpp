#include <operations/navigation_server.h>

NavigationServer::NavigationServer(ros::NodeHandle& nh, std::string robot_name)
{
	robot_name_ = robot_name;
	std::string node_name = robot_name + "_navigation_action_server";

	// Initialise the publishers for steering and wheel velocites
	initPublishers(nh, robot_name);
	initSubscribers(nh, robot_name);

	nh.param("crab_drive", CRAB_DRIVE_, false);

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
	bool odom_flag = true;
	
	nh.getParam("cheat_odom", odom_flag);

	update_current_robot_pose_ = nh.subscribe("/" + robot_name + RTAB_ODOM_TOPIC, 1000, &NavigationServer::updateRobotPose, this);

	if (odom_flag)
	{
		ROS_INFO("Currently using cheat odom from Gazebo\n");
	}
	else
	{	
		ROS_INFO("Currently using odom from rtabmap\n");
	}
	
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

operations::TrajectoryWithVelocities NavigationServer::sendGoalToPlanner(const geometry_msgs::PoseStamped& goal)
{
	// Declare a trajectory message
	operations::TrajectoryWithVelocities traj;

	// Temporary, replace with service call once the planner is complete
	std_msgs::Float64 speed;
	speed.data = BASE_DRIVE_SPEED;

	traj.waypoints.push_back(goal);
	traj.velocities.push_back(speed);

	// Make sure that all trajectory waypoints are in the map frame before returning it
	return getTrajInMapFrame(traj);
}

operations::TrajectoryWithVelocities NavigationServer::getTrajInMapFrame(const operations::TrajectoryWithVelocities& traj)
{
	operations::TrajectoryWithVelocities in_map_frame;

	// For each waypoint in the trajectories message
	for(int pt = 0; pt < traj.waypoints.size(); pt++)
	{
		geometry_msgs::PoseStamped map_pose = traj.waypoints[pt];

		// Transform that waypoint into the map frame
		NavigationAlgo::transformPose(map_pose, MAP, buffer_);

		// Push this waypoint and its velocity to the trajectory message to return
		in_map_frame.waypoints.push_back(map_pose);
		in_map_frame.velocities.push_back(traj.velocities[pt]);
	}

	return in_map_frame;
}

void NavigationServer::brakeRobot(bool brake)
{
	srcp2_msgs::BrakeRoverSrv srv;
	moveRobotWheels(0); // Its better to stop wheels from rotating if we are braking

	if(brake)
		srv.request.brake_force = 1000;
	else
		srv.request.brake_force = 0;

	brake_client_.call(srv);
}

bool NavigationServer::rotateWheels(const geometry_msgs::PoseStamped& target_robot_pose)
{
	brakeRobot(false);

	// Calculate the change in heading between the current and target pose
	double delta_heading = NavigationAlgo::changeInHeading(*getRobotPose(), target_robot_pose, robot_name_, buffer_);
	
	ROS_INFO("Steering wheels to %frad\n", delta_heading);
	steerRobot(delta_heading);

	// Currently, we never expect rotating wheels to fail.
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
	ROS_INFO("Driving forwards %fm\n", delta_distance);

	// Save the starting robot pose so we can track delta distance
	geometry_msgs::PoseStamped starting_pose = *getRobotPose();

	// Initialize the current traveled distance to 0. Used to terminate the loop, and to request a new trajectory.
	double distance_traveled = 0;

	// While we have not traveled the desired distance, keep driving.
	while (abs(distance_traveled - delta_distance) > DIST_EPSILON && ros::ok())
	{
		if(manual_driving_)
		{
			// Stop moving the robot, as we were interrupted.
			moveRobotWheels(0);
			brakeRobot(true);

			return false;
		}

		distance_traveled = abs(NavigationAlgo::changeInPosition(starting_pose, *getRobotPose()));

		// If the current distance we've traveled plus the distance since the last reset is greater than the set constant, then
		// we want to get a new trajectory from the planner.
		if(distance_traveled + total_distance_traveled_ > TRAJECTORY_RESET_DIST)
		{
			ROS_INFO("driveDistance detected total distance > trajectory reset, setting trajectory flag.\n");

			moveRobotWheels(0);
			brakeRobot(true);

			// Reset the distance traveled
			total_distance_traveled_ = 0;

			get_new_trajectory_ = true;
			return true;
		}

		// Move the wheels forward at a constant speed
		moveRobotWheels(BASE_DRIVE_SPEED);

		// Allow ROS to catch up and update our subscribers
		ros::spinOnce();

		// Slow this loop down a bit
		update_rate_->sleep();
	}

	// Update the total traveled distance with the total distance we just traveled.
	total_distance_traveled_ += distance_traveled;

	printf("Done driving forwards\n");

	// Stop moving the robot after we are done moving
	moveRobotWheels(0);
	brakeRobot(true);

	return true;
}

void NavigationServer::automaticDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	ROS_INFO("Beginning auto drive\n");

	// Initialize to true, so that we don't immediately end the loop.
	get_new_trajectory_ = true;

	// Save the goal pose in the MAP frame, so that trajectory updates will use a goal relative to the map.
	geometry_msgs::PoseStamped final_pose = goal->pose;
	NavigationAlgo::transformPose(final_pose, MAP, buffer_, 0.1);

	// While we have a new trajectory. If driveDistance does not reset this, then this loop only runs once.
	while(get_new_trajectory_)
	{
		// Forward goal to local planner, and save the returned trajectory
		operations::TrajectoryWithVelocities trajectory = sendGoalToPlanner(goal->pose);

		// We got the new trajectory, so we should reset the new trajectory flag.
		get_new_trajectory_ = false;

		// Loop over trajectory waypoints
		for (int i = 0; i < trajectory.waypoints.size(); i++)
		{
			if(manual_driving_)
			{
				ROS_ERROR_STREAM("Overridden by manual driving! Exiting.\n");
				operations::NavigationResult res;
				res.result = COMMON_RESULT::INTERRUPTED;
				action_server->setSucceeded(res);

				return;
			}

			ROS_INFO("Going to waypoint %d\n", i);

			// Extract the waypoint and desired velocity
			geometry_msgs::PoseStamped current_waypoint = trajectory.waypoints[i];
			float current_velocity = trajectory.velocities[i].data;

			waypoint_pub_.publish(current_waypoint);

			// Get the current waypoint in the map frame, based on the most recent transforms.
			current_waypoint.header.stamp = ros::Time(0);
			NavigationAlgo::transformPose(current_waypoint, MAP, buffer_, 0.1);

			// Needed, otherwise we get extrapolation into the past
			// current_waypoint.header.stamp = ros::Time(0);

			bool turned_successfully;

			// Based on the parameter in this server's constructor, either do crab drive or rotate in place drive.
			if(CRAB_DRIVE_)
			{
				// Turn wheels to heading
				ROS_INFO("Rotating wheels\n");
				turned_successfully = rotateWheels(current_waypoint);
			}
			else
			{
				ROS_INFO("Rotating robot\n");
				turned_successfully = rotateRobot(current_waypoint);
			}

			ros::Duration(1.0).sleep();

			if (!turned_successfully)
			{
				operations::NavigationResult res;

				if(manual_driving_)
				{
					ROS_ERROR_STREAM("Overridden by manual driving! Exiting.\n");
					res.result = COMMON_RESULT::INTERRUPTED;
				}
				else
				{
					//AAAH ERROR
					ROS_ERROR_STREAM("Turn to waypoint " << i << " did not succeed. Exiting.\n");
					res.result = COMMON_RESULT::FAILED;
					
				}
				action_server->setSucceeded(res);

				return;
			}

			//Get current pose + position from odometry
			geometry_msgs::PoseStamped current_robot_pose = *getRobotPose();

			//Calculate delta distance
			float delta_distance = NavigationAlgo::changeInPosition(current_robot_pose, current_waypoint);

			//Drive to goal
			ROS_INFO("Going the distance, going for speed\n");
			bool drove_successfully = driveDistance(delta_distance);

			// If driveDistance set the get_new_trajectory_ flag, we should quit out of the for loop, which will get a new trajectory.
			if(get_new_trajectory_)
			{
				ROS_INFO("Distance planner interrupt. Getting new trajectory.\n");

				// Setting i to the length of the trajectory will terminate the for loop.
				i = trajectory.waypoints.size();
			}

			if (!drove_successfully)
			{
				operations::NavigationResult res;
				if(manual_driving_)
				{
					ROS_ERROR_STREAM("Overridden by manual driving! Exiting.\n");
					res.result = COMMON_RESULT::INTERRUPTED;
				} else 
				{
					//AAAH ERROR
					ROS_ERROR_STREAM("Drive to waypoint " << i << " did not succeed.\n");
					res.result = COMMON_RESULT::FAILED;
				}
				action_server->setSucceeded(res);

				return;
			}
		}
	}

	// This final logic shouldn't be run more than once, so it is outside of the get_new_trajectory_ loop.

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
			res.result = COMMON_RESULT::INTERRUPTED;
		}
		else
		{
			//AAAH ERROR
			ROS_ERROR_STREAM("Final turn did not succeed. Exiting.\n");
			res.result = COMMON_RESULT::FAILED;
			
		}
		action_server->setSucceeded(res);

		return;
	}

	printf("Finished automatic goal!\n");

	brakeRobot(true);

	operations::NavigationResult res;
	res.result = COMMON_RESULT::SUCCESS;
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
	res.result = COMMON_RESULT::SUCCESS;
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
	res.result = COMMON_RESULT::SUCCESS;
	action_server->setSucceeded(res);
	return;
}

void NavigationServer::revolveRobot(geometry_msgs::PointStamped &revolve_about, double forward_velocity)
{
	// NavigationAlgo::transformPoint(revolve_about, robot_name_ + ROBOT_CHASSIS, buffer_, 0.1);

	std::vector<double> angles = NavigationAlgo::getSteeringAnglesRadialTurn(revolve_about.point);
	std::vector<double> speeds = NavigationAlgo::getDrivingVelocitiesRadialTurn(revolve_about.point, forward_velocity);

	steerRobot(angles);
	moveRobotWheels(speeds);
}

void NavigationServer::revolveDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	printf("Revolve drive\n");

	brakeRobot(false);

	geometry_msgs::PointStamped revolve_about = goal->point;
	double forward_velocity = goal->forward_velocity;

	revolveRobot(revolve_about, forward_velocity);

	operations::NavigationResult res;
	res.result = COMMON_RESULT::SUCCESS;
	action_server->setSucceeded(res);
	return;
}

double NavigationServer::getCumulativeTheta(double yaw, int &rotation_counter)
{
  // static variable, hence will retain its value
  static double last_yaw = 0;

  // If yaw if positive, then consider actual value of yaw (first two quads)
  // Else consider 2PI plus yaw, becaues its the third and fourth quadrants
  yaw = yaw >= 0 ? yaw : 2 * M_PI + yaw;
  yaw /= 2; // Not sure why, but this is needed for theta calculation.

  if ((last_yaw > M_PI_2 && yaw < M_PI_2))
  {
    ROS_INFO_STREAM("Rotation Complete");
    rotation_counter++;
  }
  last_yaw = yaw;
  return (rotation_counter * M_PI + yaw);
}

void NavigationServer::spiralDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	ROS_INFO("Starting spiral motion");
	brakeRobot(false);
  
	geometry_msgs::PoseStamped robot_start_pose = *getRobotPose();

	// Stamp must be set to 0 for the latest transform
	robot_start_pose.header.stamp = ros::Time(0);

	// Counts the number of rotations complited in a spiral
	int rotation_counter = 0;

	double current_yaw;

	geometry_msgs::PointStamped rotation_point;
	rotation_point.header = getRobotPose()->header;

	/**
	 * @brief Continue loop until interrupted externally (through cancel goal)
	 * 				This depends on the concept of osculating circles for a spiral
	 * 				Depending on how much the robot has already travelled, get and 
	 * 					execute the instantaneous radial turn to produce spiral motion
	 * 					motion as a whole
	 *				Source: https://en.wikipedia.org/wiki/Osculating_circle
	 */
	while (spiral_motion_continue_)
	{		
		current_yaw = NavigationAlgo::changeInOrientation(robot_start_pose, robot_name_, buffer_);

		double spiral_t = getCumulativeTheta(current_yaw, rotation_counter);
		double inst_radius = NavigationAlgo::getRadiusInArchimedeanSpiral(spiral_t);

		rotation_point.point.x = 0;
		rotation_point.point.y = inst_radius;
		rotation_point.point.z = 0;

		revolveRobot(rotation_point, SPIRAL_SPEED);
		ros::Duration(0.1).sleep();
	}

	ROS_INFO_STREAM("Spiraling Complete");
	steerRobot(0);
	brakeRobot(true);

  operations::NavigationResult res;
	res.result = COMMON_RESULT::SUCCESS;
	action_server->setSucceeded(res);
	return;
}

void NavigationServer::followDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	printf("Follow drive: Following!\n");

	ros::Duration(5).sleep();
	
	//TODO: actually make it follow the thing

	operations::NavigationResult res;
	res.result = COMMON_RESULT::SUCCESS;
	action_server->setSucceeded(res);
	return;
}

void NavigationServer::execute(const operations::NavigationGoalConstPtr &goal)
{
    printf("Received NavigationGoal, dispatching\n");

	// Zero out the total distance traveled when we receive a new goal
	total_distance_traveled_ = 0;

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
			manual_driving_ = true;
			revolveDriving(goal, server_);

			break;
		
		case NAV_TYPE::SPIRAL:

			manual_driving_ = true;
			spiralDriving(goal, server_);

			break;
		
		case NAV_TYPE::FOLLOW:

			manual_driving_ = true;
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
	spiral_motion_continue_ = false;
	steerRobot(0);
	brakeRobot(true);
	printf("Clearing current goal, got a new one\n");
}