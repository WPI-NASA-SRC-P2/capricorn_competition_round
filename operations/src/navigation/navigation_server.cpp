#include <operations/navigation_server.h>

NavigationServer::NavigationServer(ros::NodeHandle& nh, std::string robot_name)
{
	robot_name_ = robot_name;
	std::string node_name = robot_name + "_navigation_action_server";

	// Initialise the publishers for steering and wheel velocites
	initPublishers(nh, robot_name);
	initSubscribers(nh, robot_name);

	nh.param("crab_drive", CRAB_DRIVE_, false);

	ROS_INFO("[operations | nav_server | %s]: Starting navigation server...\n", robot_name_.c_str());

	// Action server
	server_ = new Server(nh, NAVIGATION_ACTIONLIB, boost::bind(&NavigationServer::execute, this, _1), false);
	server_->registerPreemptCallback(boost::bind(&NavigationServer::cancelGoal, this));
	server_->start();

	ROS_INFO("[operations | nav_server | %s]: Navigation server started.\n", robot_name_.c_str());

	listener_ = new tf2_ros::TransformListener(buffer_);

	// Initialize the rate limiter to 10 HZ
	update_rate_ = new ros::Rate(100);

	moveRobotWheels(0);
	steerRobot(0);
}

NavigationServer::~NavigationServer()
{
	// Cleanup all pointers
	delete listener_;
	delete server_;
	delete update_rate_;
}

/**
 * @brief Initialise the publisher for wheel ramping
 * 
 */
void NavigationServer::initVelocityPublisherNew(ros::NodeHandle& nh, const std::string& robot_name)
{
	wheel_ramp_pub_ = nh.advertise<operations::WheelVelocities>(CAPRICORN_TOPIC + robot_name + "/desired_wheel_velocities", 1000);
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
	waypoint_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/vis_poses", 1000);
}

/**
 * @brief Call Publisher initializers
 * 
 */
void NavigationServer::initPublishers(ros::NodeHandle& nh, const std::string& robot_name)
{
	initVelocityPublisher(nh, robot_name);
	initVelocityPublisherNew(nh, robot_name);
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

/**
 * @brief Subscribes to the ramping topic and updates whether the robot is still ramping
 * No mutex because of how inconsequential this is
 * 
 * @param msg bool indicating if ramp has finished
 */
void NavigationServer::updateRobotRamping(const std_msgs::Bool::ConstPtr& msg)
{
	ramp_finished_ = msg->data;
	return;
}

geometry_msgs::PoseStamped* NavigationServer::getRobotPose()
{
	std::lock_guard<std::mutex> pose_lock(pose_mutex_);
	return &robot_pose_;
}

// TODO: Move to not here
void NavigationServer::publishWaypoints(std::vector<geometry_msgs::PoseStamped> waypoints)
{
	visualization_msgs::MarkerArray markers;

	// Empty vector before adding new markers
	markers.markers.resize(0);

	for(auto pose : waypoints)
	{
		visualization_msgs::Marker marker;
		marker.pose = pose.pose;

		markers.markers.push_back(marker);
	}

	ROS_INFO("[operations | nav_server | %s]: About to publish markers", robot_name_.c_str());
	waypoint_pub_.publish(markers);
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

	ramp_finished_client_ = nh.subscribe("/" + robot_name + RAMP_DONE_TOPIC, 50, &NavigationServer::updateRobotRamping, this);

	if (odom_flag)
	{
		ROS_INFO("[operations | nav_server | %s]: Currently using cheat odom from Gazebo", robot_name_.c_str());
	}
	else
	{	
		ROS_INFO("[operations | nav_server | %s]: Currently using odom from rtabmap", robot_name_.c_str());
	}
	
	brake_client_ = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/" + robot_name + BRAKE_ROVER);

	brake_client_.waitForExistence();
	
	// Integrating in the planner through a service call
	trajectory_client_ = nh.serviceClient<planning::trajectory>("/capricorn/" + robot_name + "/trajectoryGenerator");
	
	// Make sure things get launched in the right order
	trajectory_client_.waitForExistence();
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
void NavigationServer::driveRobot(const std::vector<double>& speeds)
{
	operations::WheelVelocities vels;

	vels.velocities = speeds;
	vels.drive_mode = current_mode_;

	wheel_ramp_pub_.publish(vels);
}

// Steers the robot wheels for the angles. Calls the other version
void NavigationServer::steerRobot(const double angle)
{
	std::vector<double> angles = {angle, angle, angle, angle};

	steerRobot(angles);
}

// Steers the robot wheels for the angles
void NavigationServer::steerRobot(const std::vector<double>& angles)
{
	publishMessage(front_left_steer_pub_,  angles.at(0));
	publishMessage(front_right_steer_pub_, angles.at(1));
	publishMessage(back_right_steer_pub_,  angles.at(2));
	publishMessage(back_left_steer_pub_,   angles.at(3));
}

/*********************************************************************/
/******************       ROBOT VELOCITY LOGIC      ******************/
/*********************************************************************/

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
	moveRobotWheelsNew(velocity);
	// std::vector<double> angular_vels;

	// for(int i = 0; i < velocity.size(); i++)
	// {
	// 	angular_vels.push_back(NavigationAlgo::linearToAngularVelocity(velocity.at(i)));
	// }

	// publishMessage(front_left_vel_pub_, angular_vels.at(0));
	// publishMessage(front_right_vel_pub_, angular_vels.at(1));
	// publishMessage(back_right_vel_pub_, angular_vels.at(2));
	// publishMessage(back_left_vel_pub_, angular_vels.at(3));
}

/**
 * @brief Move robot wheels with the given velocity 
 * 
 * @param velocity velocity for the wheels
 */
void NavigationServer::moveRobotWheels(const double velocity)
{
	moveRobotWheelsNew(velocity);
	// double angular_velocity = NavigationAlgo::linearToAngularVelocity(velocity);

	// publishMessage(front_left_vel_pub_, angular_velocity);
	// publishMessage(front_right_vel_pub_, angular_velocity);
	// publishMessage(back_left_vel_pub_, angular_velocity);
	// publishMessage(back_right_vel_pub_, angular_velocity);
}

// Ramp robot wheels at a given velocity. Calls the other version
void NavigationServer::moveRobotWheelsNew(const double velocity)
{
	std::vector<double> velocities = {velocity, velocity, velocity, velocity};

	moveRobotWheelsNew(velocities);
}

// Ramp robot wheels at the velocity given in the vector. Will send velocities to the
// ramp node to properly interpolate velociies
void NavigationServer::moveRobotWheelsNew(const std::vector<double> velocity)
{
	std::vector<double> angular_vels;

	for(int i = 0; i < velocity.size(); i++)
	{
		angular_vels.push_back(NavigationAlgo::linearToAngularVelocity(velocity.at(i)));
	}

	changeWheelSpeeds(angular_vels);
}

void NavigationServer::changeWheelSpeeds(std::vector<double> desired_velocities)
{
	// If we are changing drive modes, ramp to 0, then ramp to our new velocity
	if(last_mode_ != current_mode_)
	{
		std::vector<double> zeros {0, 0, 0, 0};
		driveRobot(zeros);
	}

	// Ramp to desired_velocity
	driveRobot(desired_velocities);

	// Update mode after checking it's state
	last_mode_ = current_mode_;
}


/*******************************************************************/
/****************** P U B L I S H E R   L O G I C ******************/
/*******************************************************************/

planning::TrajectoryWithVelocities NavigationServer::sendGoalToPlanner(const geometry_msgs::PoseStamped& goal)
{
	// Declare a trajectory message
	planning::TrajectoryWithVelocities traj;

	// Use the service call to get a trajectory
	planning::trajectory srv;
	srv.request.targetPose = goal;

	//TODO: Waiting for galaga's fix on 0 wpts at beginning of trajectory
	if (trajectory_client_.call(srv))
	{
		ROS_INFO("[operations | nav_server | %s]: Trajectory client call succeeded", robot_name_.c_str());
		traj = srv.response.trajectory;
		//TODO: Delete hotfix once planner issue with extra waypoints has been solved
		int trajLength = traj.waypoints.size();
		if(trajLength >= 2)
		{
			traj.waypoints = std::vector<geometry_msgs::PoseStamped>(traj.waypoints.begin(), traj.waypoints.end() - 2);
		} 
		else 
		{
			//Error catching- if trajectory doesn't have 2 items, the planner messed up. Delete the trajectory.
			ROS_ERROR("[operations | nav_server | %s]: Trajectory less than 2 items long- if we've fixed the extra traj points, this should be removed", robot_name_.c_str());
			traj.waypoints.resize(0);
			return traj;
		}
		
	}
	else
	{
		ROS_ERROR("[operations | nav_server | %s]: Failed to call service trajectory generator", robot_name_.c_str());
	}

	// Make sure that all trajectory waypoints are in the map frame before returning it
	return getTrajInMapFrame(traj);
}

planning::TrajectoryWithVelocities NavigationServer::getTrajInMapFrame(const planning::TrajectoryWithVelocities& traj)
{
	planning::TrajectoryWithVelocities in_map_frame;

	ROS_INFO("[operations | nav_server | %s]: Getting traj in map frame", robot_name_.c_str());
	ROS_INFO("[operations | nav_server | %s]: Traj length: %d", robot_name_.c_str(), traj.waypoints.size());

	// For each waypoint in the trajectories message
	for(int pt = 0; pt < traj.waypoints.size(); pt++)
	{
		geometry_msgs::PoseStamped map_pose = traj.waypoints[pt];

		// Transform that waypoint into the map frame
		NavigationAlgo::transformPose(map_pose, MAP, buffer_);

		// Push this waypoint and its velocity to the trajectory message to return
		in_map_frame.waypoints.push_back(map_pose);
		std_msgs::Float64 temp_vel;
		temp_vel.data = 0;
		in_map_frame.velocities.push_back(temp_vel);

	}

	//Reverse trajectory waypoints from planner
	std::reverse(in_map_frame.waypoints.begin(), in_map_frame.waypoints.end());
	return in_map_frame;
}

void NavigationServer::brakeRobot(bool brake)
{
	srcp2_msgs::BrakeRoverSrv srv;

	// Its better to stop wheels from rotating if we are braking
	moveRobotWheels(0);

	if(brake)
		srv.request.brake_force = 1000;
	else
		srv.request.brake_force = 0;

	brake_client_.call(srv);
}

void NavigationServer::brakeRobotNew(bool brake)
{
	srcp2_msgs::BrakeRoverSrv srv;

	// Its better to stop wheels from rotating if we are braking
	moveRobotWheelsNew(0);
	ramp_finished_ = false;

	while(!ramp_finished_)
	{
		ros::spinOnce();
		update_rate_->sleep();
	}

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
	
	ROS_INFO("[operations | nav_server | %s]: Steering wheels to %frad\n", robot_name_.c_str(), delta_heading);
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
		ROS_INFO("[operations | nav_server | %s]: Delta heading not greater than epsilon threshold, done rotating...", robot_name_.c_str());
		return true;
	}

	ROS_INFO("[operations | nav_server | %s]: Turning %frad", robot_name_.c_str(), delta_heading);
	steerRobot(wheel_angles);

	// While we have not turned the desired amount
	while (abs(delta_heading) > ANGLE_EPSILON && ros::ok())
	{
		delta_heading = NavigationAlgo::changeInHeading(*getRobotPose(), target_robot_pose, robot_name_, buffer_);
		// printf("Current delta heading: %frad\n", delta_heading);

		// target_robot_pose in the robot's frame of reference
		geometry_msgs::PoseStamped target_in_robot_frame = target_robot_pose;
		NavigationAlgo::transformPose(target_in_robot_frame, robot_name_ + ROBOT_CHASSIS, buffer_, 0.1);
		
		//waypoint_pub_.publish(target_in_robot_frame);

		if(manual_driving_)
		{
			return false;
		}

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

	// Stop moving the robot after we are done moving
	moveRobotWheels(0);

	// Reset steering of wheels
	steerRobot(0);

	brakeRobot(true);

	return true;
}

bool NavigationServer::driveDistance(double delta_distance)
{
	brakeRobotNew(false);
	ROS_INFO("[operations | nav_server | %s]: Driving forwards %fm\n", robot_name_.c_str(), delta_distance);

	// Save the starting robot pose so we can track delta distance
	geometry_msgs::PoseStamped starting_pose = *getRobotPose();

	// Initialize the current traveled distance to 0. Used to terminate the loop, and to request a new trajectory.
	double distance_traveled = 0;

	// While we have not traveled the desired distance, keep driving.
	while (abs(distance_traveled - delta_distance) > c_dist_epsilon_ && ros::ok())
	{
		if(manual_driving_)
		{
			// Stop moving the robot, as we were interrupted.
			// moveRobotWheels(0);
			brakeRobotNew(true);

			return false;
		}

		distance_traveled = abs(NavigationAlgo::changeInPosition(starting_pose, *getRobotPose()));

		// If the current distance we've traveled plus the distance since the last reset is greater than the set constant, then
		// we want to get a new trajectory from the planner.
		if(distance_traveled + total_distance_traveled_ > trajectory_reset_dist_)
		{
			ROS_INFO("[operations | nav_server | %s]: driveDistance detected total distance > trajectory reset, setting trajectory flag.\n", robot_name_.c_str());

			// moveRobotWheels(0);
			brakeRobot(true);

			// Reset the distance traveled
			total_distance_traveled_ = 0;

			get_new_trajectory_ = true;
			return true;
		}

		// Move the wheels forward at a constant speed
		moveRobotWheelsNew(BASE_DRIVE_SPEED);

		// Allow ROS to catch up and update our subscribers
		ros::spinOnce();

		// Slow this loop down a bit
		update_rate_->sleep();
	}

	// Update the total traveled distance with the total distance we just traveled.
	total_distance_traveled_ += distance_traveled;

	printf("Done driving forwards\n");

	// Stop moving the robot after we are done moving
	// moveRobotWheels(0);
	brakeRobotNew(true);

	return true;
}

bool NavigationServer::smoothDriving(const geometry_msgs::PoseStamped waypoint, const geometry_msgs::PoseStamped future_waypoint)
{
	brakeRobotNew(false);

	// Save the starting robot pose so we can track delta distance
	geometry_msgs::PoseStamped starting_pose = *getRobotPose();

	double distance_to_waypoint = NavigationAlgo::changeInPosition(*getRobotPose(), waypoint);
		// Initialize the current traveled distance to 0. Used to terminate the loop, and to request a new trajectory.
	double distance_traveled = 0;

	// While we're not at the waypoint
	while((distance_to_waypoint > c_dist_epsilon_) && ros::ok())
	{
		// If we are interrupted, stop.
		if(manual_driving_)
		{	
			// Stop moving the robot, as we were interrupted.
			// moveRobotWheelsNew(0);
			brakeRobotNew(true);

			return false;
		}

		distance_traveled = abs(NavigationAlgo::changeInPosition(starting_pose, *getRobotPose()));

		// If the current distance we've traveled plus the distance since the last reset is greater than the set constant, then
		// we want to get a new trajectory from the planner.
		if(distance_traveled + total_distance_traveled_ > trajectory_reset_dist_)
		{
			ROS_INFO("[operations | nav_server | %s]: smoothDriving detected total distance > trajectory reset, setting trajectory flag.\n", robot_name_.c_str());
			ROS_INFO_STREAM("[operations | nav_server | "<<robot_name_<<"] "<<"Current distance traveled" << distance_traveled);

			// moveRobotWheelsNew(0);
			brakeRobotNew(true);

			// Reset the distance traveled
			total_distance_traveled_ = 0;

			get_new_trajectory_ = true;
			return true;
		}

		// Calculate the current dist to waypoint
		double current_distance_to_waypoint = NavigationAlgo::changeInPosition(*getRobotPose(), waypoint);

		// Calculate the delta heading
		double delta_heading, center_radius;

		// Start steering towards the next waypoint's heading when the robot is half its length away from it
		// NOTE: Add c_dist_epsilon_ to calculation if we want to start turning torwards the next heading sooner
		if (current_distance_to_waypoint < (current_distance_to_waypoint - NavigationAlgo::wheel_sep_length_/2))
			delta_heading = NavigationAlgo::changeInHeading(*getRobotPose(), future_waypoint, robot_name_, buffer_);
		else
			delta_heading = NavigationAlgo::changeInHeading(*getRobotPose(), waypoint, robot_name_, buffer_);

		//ROS_WARN("[operations | nav_server | %s]: Delta heading %f", robot_name_.c_str(), delta_heading);

		// Calculate center of radius of turn for Ackermann steering
		// If delta heading is 0 (aka no change in heading) then we rotate about an infite center radius (aka we drive straight)
		if (delta_heading == 0.0)
			center_radius = DBL_MAX;
		else
			center_radius = NavigationAlgo::wheel_sep_length_/tan(delta_heading);

		geometry_msgs::Point center_of_rotation;

		// Center pt of rotation is about the back wheels
		center_of_rotation.x = -NavigationAlgo::wheel_sep_length_/2;

		// Steering of the front wheels depends on the center radius previously calculated
		//ROS_ERROR("[operations | nav_server | %s]: Center radius %f", robot_name_.c_str(), center_radius);

		center_of_rotation.y = center_radius;

		std::vector<double> wheel_angles = NavigationAlgo::getSteeringAnglesRadialTurn(center_of_rotation);
		std::vector<double> wheel_velocities = NavigationAlgo::getDrivingVelocitiesRadialTurn(center_of_rotation, BASE_DRIVE_SPEED);

		// Set wheels to that angle
		steerRobot(wheel_angles);

		// Set wheels at speed
		moveRobotWheelsNew(wheel_velocities);

		// Update distance to waypoint
		distance_to_waypoint = NavigationAlgo::changeInPosition(waypoint, *getRobotPose());

		update_rate_->sleep();
		ros::spinOnce();
	}

	// Update the total traveled distance with the total distance we just traveled.
	total_distance_traveled_ += distance_traveled;

	// Stop moving the robot after we are done moving
	// moveRobotWheelsNew(0);
	brakeRobotNew(true);

	return true;
}

void NavigationServer::requestNewTrajectory(void)
{
	ROS_INFO("[operations | nav_server | %s]: Resetting trajectory flag after inital turn of new goal.\n", robot_name_.c_str());

	// moveRobotWheels(0);
	brakeRobot(true);

	// Reset the distance traveled
	total_distance_traveled_ = 0;

	get_new_trajectory_ = true;
}

void NavigationServer::automaticDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server, bool smooth)
{
	ROS_INFO("[operations | nav_server | %s]: Beginning auto drive", robot_name_.c_str());

	// Initialize to true, so that we don't immediately end the loop.
	get_new_trajectory_ = true;

	// Save the goal pose in the MAP frame, so that trajectory updates will use a goal relative to the map.

	geometry_msgs::PoseStamped final_pose = goal->pose;
	NavigationAlgo::transformPose(final_pose, MAP, buffer_, 0.1);
	
	// While we have a new trajectory. If driveDistance does not reset this, then this loop only runs once.
	while(get_new_trajectory_)
	{
		geometry_msgs::PoseStamped pose_wrt_robot = final_pose;

		// Forward goal to local planner, and save the returned trajectory
		ROS_INFO("[operations | nav_server | %s]: Requesting new trajectory...", robot_name_.c_str());
		bool proceed = NavigationAlgo::transformPose(pose_wrt_robot, robot_name_ + ROBOT_CHASSIS, buffer_, 0.1);
		if(!proceed)
		{
			ROS_ERROR("[operations | nav_server | %s]: Failed to transform to robot frame! Exiting.", robot_name_.c_str());
			operations::NavigationResult res;
			res.result = COMMON_RESULT::FAILED;
			action_server->setAborted(res);
			return;
		}

		planning::TrajectoryWithVelocities trajectory = sendGoalToPlanner(pose_wrt_robot);

		//Catch malformed trajectories here
		if(trajectory.waypoints.size() <= 0)
		{
			ROS_ERROR("[operations | nav_server | %s]: Got 0 length trajectory! Exiting.", robot_name_.c_str());
			operations::NavigationResult res;
			res.result = COMMON_RESULT::FAILED;
			action_server->setAborted(res);
			return;
		}

		// We got the new trajectory, so we should reset the new trajectory flag.
		get_new_trajectory_ = false;
		ROS_INFO("[operations | nav_server | %s]: Got new trajectory!", robot_name_.c_str());

		// Visualize waypoints in Gazebo
		// NavigationServer::publishWaypoints(trajectory.waypoints);

		// Loop over trajectory waypoints
		for (int i = 0; i < trajectory.waypoints.size(); i++)
		{
			if(manual_driving_)
			{
				ROS_ERROR("[operations | nav_server | %s]: Overridden by manual driving! Exiting.", robot_name_.c_str());
				operations::NavigationResult res;
				res.result = COMMON_RESULT::INTERRUPTED;
				action_server->setSucceeded(res);

				return;
			}

			ROS_INFO("[operations | nav_server | %s]: Going to waypoint %d", robot_name_.c_str(), i);

			// Extract the waypoint and desired velocity
			geometry_msgs::PoseStamped current_waypoint = trajectory.waypoints[i];
			float current_velocity = trajectory.velocities[i].data;

			//waypoint_pub_.publish(current_waypoint);

			// Get the current waypoint in the map frame, based on the most recent transforms.
			current_waypoint.header.stamp = ros::Time(0);
			NavigationAlgo::transformPose(current_waypoint, MAP, buffer_, 0.1);

			// Extract the future waypoint in trajectory
			geometry_msgs::PoseStamped future_waypoint;

			// Error checking to prevent out of bounds indexing
			if(i + 1 < trajectory.waypoints.size())
			{
				future_waypoint = trajectory.waypoints[i + 1];
			}
			else
			{
				// If at the end of vector then we want the final pose of the current trajectory
				future_waypoint = final_pose;
			}
				

			future_waypoint.header.stamp = ros::Time(0);
			NavigationAlgo::transformPose(future_waypoint, MAP, buffer_, 0.1);

			trajectory_reset_dist_ = LARGE_TRAJECTORY_RESET_DIST;

			if(smooth)
			{
				// Initial check for delta heading in case it is above the max turning limit for ackermann steering
				double delta_heading = NavigationAlgo::changeInHeading(*getRobotPose(), current_waypoint, robot_name_, buffer_);

				//Kludge for reducing reset distance after hard turns. Gets reset
				// on receiving a new trajectory.
				if(abs(delta_heading) > HALF_VIEWING)
				{
					trajectory_reset_dist_ = SMALL_TRAJECTORY_RESET_DIST;
					ROS_INFO("[operations | nav_server | %s]: Sharp turn detected, using small reset distance", robot_name_.c_str());
				}

				if (delta_heading >= MAX_TURNING_RAD || delta_heading <= MIN_TURNING_RAD)
				{
					bool turned_successfully;

					// Based on the parameter in this server's constructor, either do crab drive or rotate in place drive.
					if(CRAB_DRIVE_)
					{
						// Turn wheels to heading
						ROS_INFO("[operations | nav_server | %s]: Rotating wheels", robot_name_.c_str());
						turned_successfully = rotateWheels(current_waypoint);
					}
					else
					{
						ROS_INFO("[operations | nav_server | %s]: Rotating robot", robot_name_.c_str());
						turned_successfully = rotateRobot(current_waypoint);
					}

					ros::Duration(1.0).sleep();

					if(initial_turn_completed == false)
					{
						//Request new trajectory after inital turn in case new obstacles have appeared
						requestNewTrajectory();
						initial_turn_completed = true;
						break;
						
					}

					if (!turned_successfully)
					{
						operations::NavigationResult res;

						if(manual_driving_)
						{
							ROS_ERROR("[operations | nav_server | %s]: Overridden by manual driving! Exiting", robot_name_.c_str());
							res.result = COMMON_RESULT::INTERRUPTED;
						}
						else
						{
							//AAAH ERROR
							ROS_ERROR("[operations | nav_server | %s]: Turn to waypoint %d did not succeed. Exiting", robot_name_.c_str(), i);
							res.result = COMMON_RESULT::FAILED;
							
						}
						action_server->setSucceeded(res);

						return;
					}
				}

				bool drove_successfully = smoothDriving(current_waypoint, future_waypoint);

				// Reset initial turn flag
				initial_turn_completed = false;

				if (!drove_successfully)
				{
					operations::NavigationResult res;
					if(manual_driving_)
					{
						ROS_ERROR("[operations | nav_server | %s]: Overridden by manual driving! Exiting", robot_name_.c_str());
						res.result = COMMON_RESULT::INTERRUPTED;
					}
					else 
					{
						//AAAH ERROR
						ROS_ERROR("[operations | nav_server | %s]: Turn to waypoint %d did not succeed. Exiting", robot_name_.c_str(), i);
						res.result = COMMON_RESULT::FAILED;
					}
					action_server->setSucceeded(res);

					return;
				}
				
			}
			else
			{
				bool turned_successfully;

				// Based on the parameter in this server's constructor, either do crab drive or rotate in place drive.
				if(CRAB_DRIVE_)
				{
					// Turn wheels to heading
					ROS_INFO("[operations | nav_server | %s]: Rotating wheels", robot_name_.c_str());
					turned_successfully = rotateWheels(current_waypoint);
				}
				else
				{
					ROS_INFO("[operations | nav_server | %s]: Rotating robot", robot_name_.c_str());
					turned_successfully = rotateRobot(current_waypoint);
				}

				ros::Duration(1.0).sleep();

				if (!turned_successfully)
				{
					operations::NavigationResult res;

					if(manual_driving_)
					{
						ROS_ERROR("[operations | nav_server | %s]: Overridden by manual driving! Exiting", robot_name_.c_str());
						res.result = COMMON_RESULT::INTERRUPTED;
					}
					else
					{
						//AAAH ERROR
						ROS_ERROR("[operations | nav_server | %s]: Turn to waypoint %d did not succeed. Exiting", robot_name_.c_str(), i);
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
				ROS_INFO("[operations | nav_server | %s]: Going the distance, going for speed", robot_name_.c_str());
				bool drove_successfully = driveDistance(delta_distance);

				// If driveDistance set the get_new_trajectory_ flag, we should quit out of the for loop, which will get a new trajectory.
				if(get_new_trajectory_)
				{
					ROS_INFO("[operations | nav_server | %s]: Distance planner interrupt. Getting new trajectory.", robot_name_.c_str());

					// Setting i to the length of the trajectory will terminate the for loop.
					i = trajectory.waypoints.size();
					continue;
				}

				if (!drove_successfully)
				{
					operations::NavigationResult res;
					if(manual_driving_)
					{
						ROS_ERROR("[operations | nav_server | %s]: Overridden by manual driving! Exiting", robot_name_.c_str());;
						res.result = COMMON_RESULT::INTERRUPTED;
					} else 
					{
						//AAAH ERROR
						ROS_ERROR("[operations | nav_server | %s]: Turn to waypoint %d did not succeed. Exiting", robot_name_.c_str(), i);
						res.result = COMMON_RESULT::FAILED;
					}
					action_server->setSucceeded(res);

					return;
				}
			}
		}
	}

	// This final logic shouldn't be run more than once, so it is outside of the get_new_trajectory_ loop.

	if(goal->final_rotate)
	{
		geometry_msgs::PoseStamped current_robot_pose = *getRobotPose();

		// The final pose is on top of the robot, we only care about orientation
		final_pose.pose.position.x = current_robot_pose.pose.position.x;
		final_pose.pose.position.y = current_robot_pose.pose.position.y;

		final_pose.header.stamp = ros::Time(0);

		ROS_INFO("[operations | nav_server | %s]: Final rotate", robot_name_.c_str());

		//Turn to heading
		bool turned_successfully = rotateRobot(final_pose);

		if (!turned_successfully)
		{
			operations::NavigationResult res;

			if(manual_driving_)
			{
				ROS_ERROR("[operations | nav_server | %s]: Overridden by manual driving! Exiting.", robot_name_.c_str());
				res.result = COMMON_RESULT::INTERRUPTED;
			}
			else
			{
				//AAAH ERROR
				ROS_ERROR("[operations | nav_server | %s]: Final turn did not succeed. Exiting.", robot_name_.c_str());
				res.result = COMMON_RESULT::FAILED;
				
			}
			action_server->setSucceeded(res);

			return;
		}
	}

	

	ROS_INFO("[operations | nav_server | %s]: Finished automatic goal!", robot_name_.c_str());

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

void NavigationServer::followDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	ROS_INFO("[operations | nav_server | %s]: Follow drive: Following!", robot_name_.c_str());

	ros::Duration(5).sleep();
	
	//TODO: actually make it follow the thing

	operations::NavigationResult res;
	res.result = COMMON_RESULT::SUCCESS;
	action_server->setSucceeded(res);
	return;
}

void NavigationServer::execute(const operations::NavigationGoalConstPtr &goal)
{
	if(goal->pose.header.frame_id.empty() && goal->drive_mode == NAV_TYPE::GOAL)
	{
		ROS_ERROR("[operations | nav_server | %s]: Empty frame_id!", robot_name_.c_str());
		operations::NavigationResult res;
		res.result = COMMON_RESULT::FAILED;
		server_->setSucceeded(res);
		return;
	}

	ROS_INFO("[operations | nav_server | %s]: Bool for final rotate: %d", robot_name_.c_str(), goal->final_rotate);

  	ROS_INFO("[operations | nav_server | %s]: Received NavigationGoal, dispatching", robot_name_.c_str());
	if(goal->epsilon == 0.0)
	{
		c_dist_epsilon_ = DIST_EPSILON;
		ROS_INFO("[operations | nav_server | %s]: Default epsilon", robot_name_.c_str());
	}
	else
	{
		c_dist_epsilon_ = goal->epsilon;
		ROS_INFO("[operations | nav_server | %s]: Got epsilon of %f", robot_name_.c_str(), c_dist_epsilon_);
	}

	// Zero out the total distance traveled when we receive a new goal
	total_distance_traveled_ = 0;

	// Update the new drive mode. Used for wheel velocity ramping
	current_mode_ = NAV_TYPE(goal->drive_mode);

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
		
		case NAV_TYPE::GOAL_OLD:

			manual_driving_ = false;
			automaticDriving(goal, server_, false);

			break;

		case NAV_TYPE::GOAL:

			manual_driving_ = false;
			automaticDriving(goal, server_, true);

			break;

		case NAV_TYPE::REVOLVE:
			manual_driving_ = true;
			revolveDriving(goal, server_);

			break;
		
		case NAV_TYPE::FOLLOW:

			manual_driving_ = true;
			followDriving(goal, server_);

			break;

		default:
            ROS_ERROR("[operations | nav_server | %s]: Encountered an unknown driving mode!", robot_name_.c_str());
            break;
	}
}

void NavigationServer::cancelGoal()
{
	manual_driving_ = true;
	steerRobot(0);
	brakeRobot(true);
	ROS_WARN("[operations | nav_server | %s]: Clearing current goal, got a new one", robot_name_.c_str());
}
