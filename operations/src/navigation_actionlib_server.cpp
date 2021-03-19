#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <string.h>
#include <sensor_msgs/Imu.h>
#include <actionlib/server/simple_action_server.h>

#include <operations/TrajectoryWithVelocities.h>

#include <utils/common_names.h>
#include <operations/navigation_algorithm.h>
#include <operations/NavigationAction.h> // Note: "Action" is appended
#include <math.h>


#include <gazebo_msgs/ModelStates.h>

typedef actionlib::SimpleActionServer<operations::NavigationAction> Server;
using namespace COMMON_NAMES;

ros::Publisher front_left_vel_pub_, front_right_vel_pub_, back_left_vel_pub_, back_right_vel_pub_;
ros::Publisher front_left_steer_pub_, front_right_steer_pub_, back_left_steer_pub_, back_right_steer_pub_;

ros::Subscriber update_current_robot_pose;

// Declared globally so that execute() can access. TODO: probably temporary
std::string robot_name;

// Declare robot pose to be used globally
geometry_msgs::PoseStamped robot_pose;

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

// TODO: Should be factored out, and subscribing to cheat odom/normal odom should be implemented instead
// (see maploc/publish_cheat_odom)
void updateRobotPose(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
	for (int i = 0; i < msg->name.size(); i++)
	{
		if (strcmp(msg->name[i].c_str(), robot_name.c_str()) == 0)
		{
			std_msgs::Header *robot_header = new std_msgs::Header();
			robot_header->frame_id = robot_name + "_small_chassis";

			robot_pose.header = *robot_header;
			robot_pose.pose = msg->pose[i];
		}
	}
}

/**
 * @brief Initialize the subscriber for robot position
 * 
 */
void initSubscribers(ros::NodeHandle &nh, std::string &robot_name)
{
	/*
	 * WARNING WARNING WARNING
	 * If we use this in submission-ready code, we will get disqualified
	 */
	update_current_robot_pose = nh.subscribe("/gazebo/model_states", 1000, updateRobotPose);
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
void steerRobot(const std::vector<float> &angles)
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

/*******************************************************************/
/****************** P U B L I S H E R   L O G I C ******************/
/*******************************************************************/

operations::TrajectoryWithVelocities *sendGoalToPlanner(const operations::NavigationGoalConstPtr &goal)
{
	operations::TrajectoryWithVelocities *traj = new operations::TrajectoryWithVelocities();

	// Temporary, replace with service call once the planner is complete
	// TODO: Stop assuming we'll always get a PoseStamped. Adapt this to allow for PointStamped as well
	geometry_msgs::PoseStampted goal_pose = goal->

	return traj;
}

geometry_msgs::PoseStamped *getRobotPose()
{
	return &robot_pose;
}

std::vector<double> quaternionToEulerAngles(geometry_msgs::PoseStamped *pose)
{
	geometry_msgs::Quaternion q = pose.pose.orientation;

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);

	return std::atan2(siny_cosp, cosy_cosp);
}

float changeInHeading(geometry_msgs::PoseStamped *current_robot_pose, geometry_msgs::PoseStamped *current_waypoint)
{
	/*take the current robot's pose and the current waypoint we want to get to
	convert from quaternion to euler and create an array of them
	calculate the difference in angle between the two for the yaw
	return that value */

	std::vector<double> current_robot_pose_angles = quaternionToEulerAngles(current_robot_pose);
	std::vector<double> current_waypoint_angles = quaternionToEulerAngles(current_waypoint);
	
	double change_in_yaw = current_robot_pose_angles[0] - current_waypoint_angles[0];
	
	// we want the robot to turn in the direction of the smallest angle change
	if(change_in_yaw >= M_PI/2)
	{
		change_in_yaw  -= 2*M_PI;
	}
	
	if (change_in_yaw <= -M_PI/2) 
	{
		change_in_yaw += 2*M_PI;
	}
	
	return change_in_yaw;
}

bool rotateRobot(double delta_heading)
{
	/*
	use the current delta_heading as an input 
	if there is a need to rotate the robot then return true 
	else return false
	*/

	// Speed at which the robot should be driving
	const float BASE_SPEED = 0.25;

	// Delta position that we allow. With better nav methods, we can decrease this value
	const float EPSILON = 0.05;

	std::vector<int> wheel_angles = {M_PI/4, -M_PI/4, -M_PI/4, M_PI/4};
	std::vector<int> wheel_speeds = {BASE_SPEED, -BASE_SPEED, -BASE_SPEED, BASE_SPEED};

	transform(wheel_speeds.begin(), wheel_speeds.end(), wheel_speeds.begin(), _1 * 3);

	// Save starting robot pose to track the change in heading
	geometry_msgs::PoseStamped starting_pose = robot_pose;


	if (delta_heading == 0)
	{
		return true;
	}

	// While we have not turned the desired amount
	while (abs(changeInHeading(starting_pose, robot_pose) - delta_heading) > EPSILON)
	{
		steerRobot(wheel_angles);

		if (delta_heading < 0)
		{
			// Turn clockwise	
			moveRobotWheels(wheel_speeds);
		}
		else if (delta_heading > 0)
		{
			// Turn counter-clockwise
			moveRobotWheels(-wheel_speeds);
		}

		// Allow ROS to catch up and update our subscribers
		ros::spinOnce();
	}

	// Stop moving the robot after we are done moving
	moveRobotWheels(0);

	// Reset steering of wheels
	steerRobot(0);

	return true;
}

float changeInPosition(geometry_msgs::PoseStamped *current_robot_pose, geometry_msgs::PoseStamped *target_robot_pose)
{
	/*
	Take the current and target pose, return the euclidean distance between them.
	*/
	float delta_x = current_robot_pose->pose.position.x - target_robot_pose->pose.position.x;
	float delta_y = current_robot_pose->pose.position.y - target_robot_pose->pose.position.y;
	return pow(pow(delta_x, 2) + pow(delta_y, 2), 0.5);
}

bool driveToGoal(double delta_distance)
{
	// Speed at which the robot should be driving
	const float BASE_SPEED = 0.25;

	// Delta position that we allow. With better nav methods, we can decrease this value
	const float EPSILON = 0.2;

	// Point the wheels forward
	steerRobot(0);

	// Save the starting robot pose so we can track delta distance
	geometry_msgs::PoseStamped starting_pose = robot_pose;

	// While we have not traveled the
	while (abs(changeInPosition(starting_pose, robot_pose) - delta_distance) > EPSILON)
	{
		// Move the wheels forward at a constant speed
		moveRobotWheels(BASE_SPEED);

		// Allow ROS to catch up and update our subscribers
		ros::spinOnce();
	}

	// Stop moving the robot after we are done moving
	moveRobotWheels(0);

	return true;
}

void execute(const operations::NavigationGoalConstPtr &goal, Server *action_server)
{
	//Forward goal to local planner
	//Read trajectory list
	operations::TrajectoryWithVelocities *trajectory = sendGoalToPlanner(goal);

	//Loop over trajectories
	for (int i = 0; i < trajectory->waypoints.size(); i++)
	{
		//Current waypoint comprehension
		geometry_msgs::PoseStamped *current_waypoint = &trajectory->waypoints[i];
		float current_velocity = trajectory->velocities[i].data;

		//Get current pose + position from odometry
		geometry_msgs::PoseStamped *current_robot_pose = getRobotPose();

		//Calculate delta heading
		float delta_heading = changeInHeading(current_robot_pose, current_waypoint);

		//Turn to heading
		bool turned_successfully = rotateRobot(delta_heading);

		if (!turned_successfully)
		{
			//AAAH ERROR
			std::cout << "Turn to waypoint " << i << " did not succeed." << std::endl;

			//TODO: We may need to add more logic here
			return;
		}

		//Get current pose + position from odometry
		current_robot_pose = getRobotPose();

		//Calculate delta distance
		float delta_distance = changeInPosition(current_robot_pose, current_waypoint);

		//Drive to goal
		bool drove_successfully = driveToGoal(delta_distance);

		if (!drove_successfully)
		{
			//AAAH ERROR
			std::cout << "Drive to waypoint " << i << " did not succeed." << std::endl;

			//TODO: We may need to add more logic here
			return;
		}
	}

	if (false /* goal came with orientation information */)
	{
		//Get current pose + position from odometry
		geometry_msgs::PoseStamped *current_robot_pose = getRobotPose();

		//Calculate delta heading
		geometry_msgs::PoseStamped final_pose = goal->pose;
		float delta_heading = changeInHeading(current_robot_pose, &final_pose);

		//Turn to heading
		bool turned_successfully = rotateRobot(delta_heading);

		if (!turned_successfully)
		{
			//AAAH ERROR
			std::cout << "Turn to goal did not succeed." << std::endl;

			//TODO: We may need to add more logic here
			return;
		}
	}

	//Message the Robot State Machine with success on goal follow
}

/*********************************************/
/****************** M A I N ******************/
/*********************************************/

int main(int argc, char **argv)
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
		robot_name = (std::string)argv[1];
		std::string node_name = robot_name + "_navigation_action_server";

		ros::init(argc, argv, node_name);
		ros::NodeHandle nh;

		// Initialise the publishers for steering and wheel velocites
		initPublishers(nh, robot_name);
		initSubscribers(nh, robot_name);

		// Action server
		Server server(nh, NAVIGATION_ACTIONLIB, boost::bind(&execute, _1, &server), false);
		server.start();
		ros::spin();

		return 0;
	}
}
