#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <operations/TrajectoryWithVelocities.h>
#include <nav_msgs/Odometry.h>
#include <srcp2_msgs/BrakeRoverSrv.h>

#include <operations/navigation_algorithm.h>
#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>

#include <math.h>
#include <string.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mutex>

// Allow us to omit the COMMON_NAMES:: prefix to common strings
using namespace COMMON_NAMES;

// Base speed for rotating and driving the robot. TODO: PID controller does not properly match this value
#define BASE_SPEED 0.6

// Tolerances for drives and turns
#define DIST_EPSILON 0.05
#define ANGLE_EPSILON 0.02

// Simplify creation of action servers
typedef actionlib::SimpleActionServer<operations::NavigationAction> Server;

// Publishers for each wheel velocity and steering controller
ros::Publisher front_left_vel_pub_, front_right_vel_pub_, back_left_vel_pub_, back_right_vel_pub_;
ros::Publisher front_left_steer_pub_, front_right_steer_pub_, back_left_steer_pub_, back_right_steer_pub_;

// Used to get the current robot pose
ros::Subscriber update_current_robot_pose;

ros::ServiceClient brake_client_;

// Declared globally so that execute() can access. TODO: probably temporary
std::string robot_name;

// Declare robot pose to be used globally
geometry_msgs::PoseStamped robot_pose;
std::mutex pose_mutex;

// Used to perform transforms between the robot and map reference frames
tf2_ros::Buffer buffer;
tf2_ros::TransformListener* listener;

// Whether we are currently manually driving, or automatically following a trajectory
bool manual_driving = false;

/*******************************************************************************/
/****************** I N I T I A L I Z E   P U B L I S H E R S ******************/
/*******************************************************************************/
/**
 * @brief Initialise the publishers for wheel velocities
 * 
 */
void initVelocityPublisher(ros::NodeHandle &nh, const std::string &robot_name)
{
	front_left_vel_pub_ = nh.advertise<std_msgs::Float64>(CAPRICORN_TOPIC + robot_name + WHEEL_PID + FRONT_LEFT_WHEEL + DESIRED_VELOCITY, 1000);
	front_right_vel_pub_ = nh.advertise<std_msgs::Float64>(CAPRICORN_TOPIC + robot_name + WHEEL_PID + FRONT_RIGHT_WHEEL + DESIRED_VELOCITY, 1000);
	back_left_vel_pub_ = nh.advertise<std_msgs::Float64>(CAPRICORN_TOPIC + robot_name + WHEEL_PID + BACK_LEFT_WHEEL + DESIRED_VELOCITY, 1000);
	back_right_vel_pub_ = nh.advertise<std_msgs::Float64>(CAPRICORN_TOPIC + robot_name + WHEEL_PID + BACK_RIGHT_WHEEL + DESIRED_VELOCITY, 1000);
}

/**
 * @brief Initialise the publishers for wheel steering angles
 * 
 */
void initSteerPublisher(ros::NodeHandle &nh, const std::string &robot_name)
{
	front_left_steer_pub_ = nh.advertise<std_msgs::Float64>(robot_name + FRONT_LEFT_WHEEL + STEERING_TOPIC, 1000);
	front_right_steer_pub_ = nh.advertise<std_msgs::Float64>(robot_name + FRONT_RIGHT_WHEEL + STEERING_TOPIC, 1000);
	back_left_steer_pub_ = nh.advertise<std_msgs::Float64>(robot_name + BACK_LEFT_WHEEL + STEERING_TOPIC, 1000);
	back_right_steer_pub_ = nh.advertise<std_msgs::Float64>(robot_name + BACK_RIGHT_WHEEL + STEERING_TOPIC, 1000);
}

/**
 * @brief Initialise publihers used to debug
 * 
 */
void initDebugPublisher(ros::NodeHandle &nh, const std::string &robot_name)
{
	//waypoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>(CAPRICORN_TOPIC + robot_name + "/current_waypoint", 1000);
}

/**
 * @brief Call Publisher initializers
 * 
 */
void initPublishers(ros::NodeHandle &nh, const std::string &robot_name)
{
	initVelocityPublisher(nh, robot_name);
	initSteerPublisher(nh, robot_name);
	initDebugPublisher(nh, robot_name);
}

/**
 * @brief Subscribes to an odometry topic, and updates the global robot_pose
 * 
 * @param msg The odometry message to process
 */
void updateRobotPose(const nav_msgs::Odometry::ConstPtr &msg)
{
	std::lock_guard<std::mutex> pose_lock(pose_mutex);
	robot_pose.header = msg->header;
	robot_pose.pose = msg->pose.pose;
	return;
}

geometry_msgs::PoseStamped* getRobotPose()
{
	std::lock_guard<std::mutex> pose_lock(pose_mutex);
	return &robot_pose;
}

/**
 * @brief Initialize the subscriber for robot position
 * 
 */
void initSubscribers(ros::NodeHandle &nh, std::string &robot_name)
{
	// TODO: Swap this topic from CHEAT_ODOM_TOPIC to the real odom topic. Or, add a flag in the launch file and switch between the two
	update_current_robot_pose = nh.subscribe(CAPRICORN_TOPIC + robot_name + CHEAT_ODOM_TOPIC, 1000, updateRobotPose);
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
void publishMessage(ros::Publisher &publisher, float data)
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
void steerRobot(const std::vector<double> &angles)
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
void steerRobot(const double angle)
{
	publishMessage(front_left_steer_pub_, angle);
	publishMessage(front_right_steer_pub_, angle);
	publishMessage(back_right_steer_pub_, angle);
	publishMessage(back_left_steer_pub_, angle);

	ros::Duration(0.5).sleep();
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
void moveRobotWheels(const std::vector<double> velocity)
{
	publishMessage(front_left_vel_pub_, velocity.at(0));
	publishMessage(front_right_vel_pub_, velocity.at(1));
	publishMessage(back_right_vel_pub_, velocity.at(2));
	publishMessage(back_left_vel_pub_, velocity.at(3));
}

/**
 * @brief Move robot wheels with the given velocity 
 * 
 * @param velocity velocity for the wheels
 */
void moveRobotWheels(const double velocity)
{
	publishMessage(front_left_vel_pub_, velocity);
	publishMessage(front_right_vel_pub_, velocity);
	publishMessage(back_left_vel_pub_, velocity);
	publishMessage(back_right_vel_pub_, velocity);
}

/*******************************************************************/
/****************** P U B L I S H E R   L O G I C ******************/
/*******************************************************************/

operations::TrajectoryWithVelocities* sendGoalToPlanner(const operations::NavigationGoalConstPtr &goal)
{
	operations::TrajectoryWithVelocities *traj = new operations::TrajectoryWithVelocities();

	// Temporary, replace with service call once the planner is complete
	// TODO: Stop assuming we'll always get a PoseStamped. Adapt this to allow for PointStamped as well
	geometry_msgs::PoseStamped goal_pose = goal->pose;
	std_msgs::Float64 speed;
	speed.data = BASE_SPEED;

	traj->waypoints.push_back(goal_pose);
	traj->velocities.push_back(speed);

	return traj;
}

void brakeRobot(double brake_force)
{
	srcp2_msgs::BrakeRoverSrv srv;
	srv.request.brake_force = brake_force;
	brake_client_.call(srv);
}

bool rotateRobot(const geometry_msgs::PoseStamped& target_robot_pose)
{
	brakeRobot(0);
	ros::Duration(.5).sleep();
	std::vector<double> wheel_angles = {-M_PI/4, M_PI/4, -M_PI/4, M_PI/4};
	std::vector<double> wheel_speeds_right = {BASE_SPEED, -BASE_SPEED, -BASE_SPEED, BASE_SPEED};
	std::vector<double> wheel_speeds_left = {-BASE_SPEED, BASE_SPEED, BASE_SPEED, -BASE_SPEED};

	// Save starting robot pose to track the change in heading
	geometry_msgs::PoseStamped starting_pose = *getRobotPose();

	double delta_heading = NavigationAlgo::changeInHeading(starting_pose, target_robot_pose, robot_name, buffer);
	
	if (abs(delta_heading) <= ANGLE_EPSILON)
	{
		return true;
	}

	printf("Turning %frad\n", delta_heading);
	steerRobot(wheel_angles);

	// While we have not turned the desired amount
	while (abs(NavigationAlgo::changeInHeading(starting_pose, target_robot_pose, robot_name, buffer)) > ANGLE_EPSILON && ros::ok())
	{
		if(manual_driving)
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
	}

	printf("Done rotating\n");

	// Stop moving the robot after we are done moving
	moveRobotWheels(0);

	// Reset steering of wheels
	steerRobot(0);

	brakeRobot(1000);
	ros::Duration(1.5).sleep();

	return true;
}

bool driveDistance(double delta_distance)
{
	brakeRobot(0);
	ros::Duration(.5).sleep();
	printf("Driving forwards %fm\n", delta_distance);

	// Point the wheels forward
	steerRobot(0);

	// Save the starting robot pose so we can track delta distance
	geometry_msgs::PoseStamped starting_pose = *getRobotPose();

	// While we have not traveled the
	while (abs(NavigationAlgo::changeInPosition(starting_pose, *getRobotPose()) - delta_distance) > DIST_EPSILON && ros::ok())
	{
		if(manual_driving)
		{
			return false;
		}

		// Move the wheels forward at a constant speed
		moveRobotWheels(BASE_SPEED);

		// Allow ROS to catch up and update our subscribers
		ros::spinOnce();
	}

	printf("Done driving forwards\n");

	// Stop moving the robot after we are done moving
	moveRobotWheels(0);

	brakeRobot(1000);
	ros::Duration(1).sleep();

	return true;
}

void automaticDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	printf("Auto drive\n");

	//Forward goal to local planner, and save the returned trajectory
	operations::TrajectoryWithVelocities *trajectory = sendGoalToPlanner(goal);

	geometry_msgs::PoseStamped final_pose = buffer.transform(goal->pose, MAP, ros::Duration(0.1));

	//Loop over trajectories
	for (int i = 0; i < trajectory->waypoints.size(); i++)
	{
		if(manual_driving)
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

		current_waypoint = buffer.transform(current_waypoint, MAP, ros::Duration(0.1));

		// Needed, otherwise we get extrapolation into the past
		current_waypoint.header.stamp = ros::Time(0);

		//Turn to heading
		bool turned_successfully = rotateRobot(current_waypoint);

		if (!turned_successfully)
		{
			operations::NavigationResult res;

			if(manual_driving)
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
		bool drove_successfully = driveDistance(delta_distance);

		if (!drove_successfully)
		{
			operations::NavigationResult res;
			if(manual_driving)
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

		if(manual_driving)
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

	brakeRobot(1000);

	operations::NavigationResult res;
	res.result = NAV_RESULT::SUCCESS;
	action_server->setSucceeded(res);
}

void linearDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	printf("Manual drive: Linear velocity\n");
	brakeRobot(0);
	steerRobot(0);
	moveRobotWheels(goal->forward_velocity);

	if(0 == goal->forward_velocity)
	{
		brakeRobot(1000);
		ros::Duration(1).sleep();
	}
	
	operations::NavigationResult res;
	res.result = NAV_RESULT::SUCCESS;
	action_server->setSucceeded(res);
	return;
}

void angularDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	printf("Manual drive: Angular velocity\n");
	brakeRobot(0);
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

void spiralDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	printf("Spiral drive: Spiral Away!\n");

	ros::Duration(5).sleep();

	//TODO: actually make it spiral

	operations::NavigationResult res;
	res.result = NAV_RESULT::SUCCESS;
	action_server->setSucceeded(res);
	return;
}

void followDriving(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	printf("Follow drive: Following!\n");

	ros::Duration(5).sleep();
	
	//TODO: actually make it follow the thing

	operations::NavigationResult res;
	res.result = NAV_RESULT::SUCCESS;
	action_server->setSucceeded(res);
	return;
}

void execute(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
    printf("Received NavigationGoal, dispatching\n");

	switch(goal->drive_mode)
	{
		case NAV_TYPE::MANUAL:

			manual_driving = true;
			if(goal->angular_velocity != 0)
			{
				angularDriving(goal, action_server);
			}
			else
			{
				linearDriving(goal, action_server);
			}		

			break;
		
		case NAV_TYPE::GOAL:

			manual_driving = false;
			automaticDriving(goal, action_server);

			break;
		
		case NAV_TYPE::SPIRAL:

			manual_driving = false;
			spiralDriving(goal, action_server);

			break;
		
		case NAV_TYPE::FOLLOW:

			manual_driving = false;
			followDriving(goal, action_server);

			break;

		default:
            ROS_ERROR_STREAM(robot_name + " encountered an unknown driving mode!");
            break;
	}
}

void cancelGoal()
{
	manual_driving = true;
	printf("Clearing current goal, got a new one\n");
}
/*********************************************/
/****************** M A I N ******************/
/*********************************************/

int main(int argc, char **argv)
{
	// Check if the node is being run through roslauch, and have one parameter of RobotName_Number
	if (argc != 2 && argc != 4)
	{
		// Displaying an error message for correct usage of the script, and returning error.
		ROS_ERROR_STREAM("Not enough arguments! Please pass in robot name with number.");
		return -1;
	}
	else
	{
		robot_name = (std::string) argv[1];
		std::string node_name = robot_name + "_navigation_action_server";

		ros::init(argc, argv, node_name);
		ros::NodeHandle nh;

		// Initialise the publishers for steering and wheel velocites
		initPublishers(nh, robot_name);
		initSubscribers(nh, robot_name);

		printf("Starting navigation server...\n");

		// Action server
		Server server(nh, NAVIGATION_ACTIONLIB, boost::bind(&execute, _1, &server), false);
		server.registerPreemptCallback(cancelGoal);
		server.start();

		printf("Navigation server started.\n");

		listener = new tf2_ros::TransformListener(buffer);

		moveRobotWheels(0);
		steerRobot(0);
		
		ros::spin();

		// Cleanup the TransformListener
		delete listener;

		return 0;
	}
}
