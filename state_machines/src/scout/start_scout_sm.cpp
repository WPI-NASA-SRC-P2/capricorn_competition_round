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

std::string g_robot_name;

int main(int argc, char *argv[])
{
	if (argc != 2 && argc != 4)
	{
		ROS_ERROR_STREAM("This node must be launched with the robotname passed as a command line argument!");
		return -1;
	}

	g_robot_name = argv[1];
	ROS_INFO_STREAM("Robot Name = " + g_robot_name);
	ros::init(argc, argv, g_robot_name + "_sm");
	ros::NodeHandle nh;

	ROS_INFO("Started Scout State Machine Actionlib Server");

	try {
		RobotScheduler cSchd(nh, SCOUT_1_NAME);
		// cSchd.initROS(nh, SCOUT_1_NAME);
		cSchd.addState(new Search(nh, SCOUT_1_NAME));
		cSchd.addState(new Undock(nh, SCOUT_1_NAME));
		cSchd.addState(new Locate(nh, SCOUT_1_NAME));
		cSchd.setInitialState(SCOUT_SEARCH_VOLATILE);
		// cSchd.setInitialState(SCOUT_UNDOCK);
		cSchd.exec();
		return 0;
	}

	catch(StateMachineException& ex) {
		std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
	}
	// ros::spin();

	ROS_WARN("Scout state machine died!\n");

	return 0;
}