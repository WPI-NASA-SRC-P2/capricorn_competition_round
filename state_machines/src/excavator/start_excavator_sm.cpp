/**
 * @file start_excavator_sm.cpp
 * @author team bebop (mmuqeetjibran.wpi.edu)
 * @brief This ros node creates a tester for the excavator, handles every task present in 
 * COMMON_NAMES::STATE_MACHINE_TASK for excavator
 * 
 * @version 0.1
 * @date 2021-06-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <state_machines/excavator_state_machine.h>
#include <state_machines/RobotStateMachineTaskAction.h>
#include <actionlib/server/simple_action_server.h>

std::string g_robot_name;

int main(int argc, char *argv[])
{
	if (argc != 2 && argc != 4)
	{
		ROS_ERROR_STREAM("[STATE_MACHINES] | start_excavator_sm.cpp | This node must be launched with the robotname passed as a command line argument!");
		return -1;
	}

	g_robot_name = argv[1];
	// ROS_INFO_STREAM("Robot Name = " + g_robot_name);
	ros::init(argc, argv, g_robot_name + "_sm");
	ros::NodeHandle nh;

	ROS_INFO_STREAM("[STATE_MACHINES | start_excavator_sm.cpp | " << g_robot_name << "]: Started Excavator State Machine Actionlib Server");

	try {
		RobotScheduler cSchd(nh, g_robot_name);
		// cSchd.initROS(nh, SCOUT_1_NAME);
		cSchd.addState(new GoToDefaultArmPosition(nh, g_robot_name));
		cSchd.addState(new GoToScout(nh, g_robot_name));
		cSchd.addState(new ParkAndPub(nh, g_robot_name));
		cSchd.addState(new DigAndDump(nh, g_robot_name));
		cSchd.addState(new IdleState(nh, g_robot_name));
		cSchd.addState(new PreParkHauler(nh, g_robot_name));
		cSchd.addState(new ExcavatorGoToLoc(nh, g_robot_name));
		cSchd.addState(new ExcavatorResetOdomAtHopper(nh, g_robot_name));
		cSchd.addState(new ExcavatorGoToRepairStation(nh, g_robot_name));
		cSchd.setInitialState(ROBOT_IDLE_STATE);
		cSchd.exec();
		return 0;
	}

	catch(StateMachineException& ex) {
		// std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
		ROS_ERROR_STREAM("[STATE_MACHINES | start_excavator_sm.cpp | " << g_robot_name << "]: [ERROR] " << ex.getMessage());
	}
	// ros::spin();

	ROS_WARN_STREAM("[STATE_MACHINES | start_hauler_sm.cpp | " << g_robot_name << "]: Excavator state machine died!\n");

	return 0;
}