/**
 * @file start_hauler_sm.cpp
 * @author team bebop (mmuqeetjibran.wpi.edu)
 * @brief This ros node creates a tester for the hauler, handles every task present in 
 * COMMON_NAMES::STATE_MACHINE_TASK for hauler
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

	ROS_INFO("Started Hauler State Machine Actionlib Server");

	try {
		RobotScheduler cSchd(nh, g_robot_name);
      
      cSchd.addState(new GoToProcPlant(nh, g_robot_name));
      cSchd.addState(new ParkAtHopper(nh, g_robot_name));
      // cSchd.addState(new ResetOdom(nh, g_robot_name));
      cSchd.addState(new UndockHopper(nh, g_robot_name));
      cSchd.addState(new ResetOdomMacro(nh, g_robot_name));
      cSchd.addState(new GoToExcavator(nh, g_robot_name));
      cSchd.addState(new ParkAtExcavator(nh, g_robot_name));
      cSchd.addState(new UndockExcavator(nh, g_robot_name));
      cSchd.addState(new DumpVolatile(nh, g_robot_name));
			cSchd.addState(new HaulerGoToScout(nh, g_robot_name));
			cSchd.addState(new HaulerGoToLoc(nh, g_robot_name));
			cSchd.addState(new IdleState(nh, g_robot_name));
      cSchd.setInitialState(ROBOT_IDLE_STATE);
      cSchd.exec();
      return 0;


	}

	catch(StateMachineException& ex) {
		std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
	}
	// ros::spin();

	ROS_WARN("Hauler state machine died!\n");

	return 0;
}