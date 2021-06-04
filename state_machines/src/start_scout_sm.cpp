/**
 * @file start_scout_sm.cpp
 * @author Ashay Aswale (asaswale.wpi.edu)
 * @brief This ros node creates an action library for the scout, handles every task present in 
 * COMMON_NAMES::STATE_MACHINE_TASK for scout
 * 
 * @version 0.1
 * @date 2021-05-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <state_machines/scout_state_machine.h>

int main(int argc, char *argv[])
{
	// if (argc != 2 && argc != 4)
	// {
	// 	ROS_ERROR_STREAM("This node must be launched with the robotname passed as a command line argument!");
	// 	return -1;
	// }

	std::string g_robot_name = argv[1];

	ros::init(argc, argv, g_robot_name + "_sm");
	ros::NodeHandle nh;

	// ScoutBaseState base_state(nh, g_robot_name);
	// Undock undock(nh, g_robot_name);
	// bool undock_entry = undock.entryPoint();
	// if (undock_entry)
	// {
	// 	ROS_INFO("Undocking!");
	// 	undock.exec();
	// 	ROS_INFO_STREAM("exitPoint: "<<undock.exitPoint());
	// }
	// else
	// {
	// 	ROS_INFO("Not Undocking");
	// }
	Locate locate(nh, g_robot_name);
	if(locate.entryPoint())
	{
		ROS_INFO("Lcoating");
		locate.exec();
		ros::Duration(10).sleep();
		locate.exitPoint();
	}
	else
	{
		ROS_INFO("Not Near Volatile, not locating");
	}
	return 0;
}