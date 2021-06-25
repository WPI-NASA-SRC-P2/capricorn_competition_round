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
#include <state_machines/hauler_state_machine.h>
#include <state_machines/RobotStateMachineTaskAction.h>
#include <actionlib/server/simple_action_server.h>

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

	ROS_INFO("Started Excavator State Machine Actionlib Server");

	try {
		RobotScheduler cSchd(nh, EXCAVATOR_1_NAME);
      
      cSchd.addState(new GoToProcPlant(nh, HAULER_1_NAME));
      cSchd.addState(new ParkAtHopper(nh, HAULER_1_NAME));
      cSchd.addState(new ResetOdom(nh, HAULER_1_NAME));
      cSchd.addState(new UndockHopper(nh, HAULER_1_NAME));
      cSchd.addState(new ResetOdomMacro(nh, HAULER_1_NAME));
      cSchd.addState(new GoToExcavator(nh, HAULER_1_NAME));
      cSchd.addState(new ParkAtExcavator(nh, HAULER_1_NAME));
      cSchd.addState(new UndockExcavator(nh, HAULER_1_NAME));
      cSchd.addState(new DumpVolatile(nh, HAULER_1_NAME));
      cSchd.setInitialState(HAULER_PARK_AT_HOPPER);
      cSchd.exec();
      return 0;
	}

	catch(StateMachineException& ex) {
		std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
	}
	// ros::spin();

	ROS_WARN("Excavator state machine died!\n");

	return 0;
}