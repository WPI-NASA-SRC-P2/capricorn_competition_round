#include "ros/ros.h"

#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <string.h>
#include <sensor_msgs/Imu.h>
#include <actionlib/server/simple_action_server.h>

#include <operations/TrajectoryWithVelocities.h>

#include <utils/common_names.h>
#include <operations/navigation_algorithm.h>
#include <operations/NavigationAction.h>  // Note: "Action" is appended


typedef actionlib::SimpleActionServer<operations::NavigationAction> Server;
using namespace COMMON_NAMES;

ros::Publisher front_left_vel_pub_, front_right_vel_pub_, back_left_vel_pub_, back_right_vel_pub_;
ros::Publisher front_left_steer_pub_, front_right_steer_pub_, back_left_steer_pub_, back_right_steer_pub_;

// Declared globally so that execute() can access. TODO: probably temporary
std::string robot_name;

/*******************************************************************************/
/****************** I N I T I A L I Z E   P U B L I S H E R S ******************/
/*******************************************************************************/
/**
 * @brief Initialise the publishers for wheel velocities
 * 
 */
void initVelocityPublisher(ros::NodeHandle &nh, const std::string &robot_name)
{
	front_left_vel_pub_ = nh.advertise<std_msgs::Float64>(robot_name + FRONT_LEFT_WHEEL + VELOCITY_TOPIC, 1000);  
	front_right_vel_pub_ = nh.advertise<std_msgs::Float64>(robot_name + FRONT_RIGHT_WHEEL + VELOCITY_TOPIC, 1000);  
	back_left_vel_pub_ = nh.advertise<std_msgs::Float64>(robot_name + BACK_LEFT_WHEEL + VELOCITY_TOPIC, 1000);  
	back_right_vel_pub_ = nh.advertise<std_msgs::Float64>(robot_name + BACK_RIGHT_WHEEL + VELOCITY_TOPIC, 1000);
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
 * @brief Call Publisher initializers
 * 
 */
void initPublishers(ros::NodeHandle &nh, const std::string &robot_name)
{
	initVelocityPublisher(nh, robot_name);
	initSteerPublisher(nh, robot_name);
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
void steerRobot(const std::vector<float>& angles)
{
	publishMessage(front_left_steer_pub_, angles.at(0));
	publishMessage(front_right_steer_pub_, angles.at(1));
	publishMessage(back_right_steer_pub_, angles.at(2));
	publishMessage(back_left_steer_pub_, angles.at(3));
}

/**
 * @brief Steers the robot wheels for the angles
 * 
 * @param angle Angles at which the robot wheels will be steeered
 */
void steerRobot(const float angle)
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
void moveRobotWheels(const std::vector<float> velocity)
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
void moveRobotWheels(const float velocity)
{
	publishMessage(front_left_vel_pub_, velocity);
	publishMessage(front_right_vel_pub_, velocity);
	publishMessage(back_left_vel_pub_, velocity);
	publishMessage(back_right_vel_pub_, velocity);
}

void followTrajectory(operations::TrajectoryWithVelocities* traj)
{
	std::cout << "Following custom trajectory message" << std::endl;

	std::cout << "First x coordinate: " << traj->waypoints[0].pose.position.x << std::endl;
  std::cout << "Frame of reference: " << traj->waypoints[0].header.frame_id << std::endl;
}

void followManually(float forward_velocity, float angular_velocity)
{
	float radius = 2.5, velocity;
	
	// If the forward velocity is 0, then the robot will rotate about its center,
	// Hence the radius of rotation will be 0 and rotational velocity is
	if(forward_velocity==0)
	{
		velocity = -angular_velocity;
		radius = 0;
	} 

	// If the goal has forward velocity, then the robot should take radial turn
	// Velocity will be the same as forward_velocity
	else
	{
		velocity = forward_velocity;
		radius = std::copysign(radius, angular_velocity);
	}

	// This will give us the steering angles and the velocities needed for taking the radial turn
	std::vector<float> steering_angles;
	std::vector<float> velocities;
	steering_angles = NavigationAlgo::getSteeringAnglesRadialTurn(radius);
	velocities = NavigationAlgo::getDrivingVelocitiessRadialTurn(radius, velocity);

	// Steer and move the robot.
	steerRobot(steering_angles);
	moveRobotWheels(velocities);
}

/*******************************************************************/
/****************** P U B L I S H E R   L O G I C ******************/
/*******************************************************************/

/**
 * @brief executes the actionlib goal request
 * 
 * @param goal Goal requestedfor the action:
 *             Goal components:
 *                float32 forward_velocity  
 *                float32 sideways_velocity  
 *                geometry_msgs/Point point
 *                geometry_msgs/Pose pose
 * @param action_server Action Server object 
 */
void execute(const operations::NavigationGoalConstPtr& goal, Server* action_server)  
{
	// Velocities 
	float forward_velocity, angular_velocity;
	forward_velocity = goal->forward_velocity;
	angular_velocity = goal->angular_velocity;

	std::cout << "Execute called" << std::endl;


	// If the goal has sideways velocity, then the robots should take radial turn
	if (angular_velocity != 0)
	{
		followManually(forward_velocity, angular_velocity);
	}
	else
	{
		// Make service call to planner to get the trajectory
		// TODO: Replace this temp code with real code
		operations::TrajectoryWithVelocities* traj = new operations::TrajectoryWithVelocities();

		geometry_msgs::PoseStamped* pose1 = new geometry_msgs::PoseStamped();

		std_msgs::Header chassis;
		chassis.frame_id = robot_name + "_small_chassis";

    pose1->header = chassis;
		pose1->pose.position.x = 2;

		geometry_msgs::PoseStamped* pose2 = pose1;
		pose2->pose.position.x = 4;

		traj->waypoints.push_back(*pose1);
		traj->waypoints.push_back(*pose2);

		std_msgs::Float64 speed1;
		std_msgs::Float64 speed2;

		speed1.data = 0.5;
		speed2.data = 0.7;

		traj->velocities.push_back(speed1);
		traj->velocities.push_back(speed2);

		// Follow the trajectory generated by the local planner
		followTrajectory(traj);
	}

	action_server->setSucceeded();
}

/*********************************************/
/****************** M A I N ******************/
/*********************************************/

int main(int argc, char** argv)
{
	// Check if the node is being run through roslauch, and have one parameter of RobotName_Number
	if (argc != 2)
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
		
		// Action server 
		Server server(nh, NAVIGATION_ACTIONLIB, boost::bind(&execute, _1, &server), false);
		server.start();
		ros::spin();

		return 0;
	}
}
