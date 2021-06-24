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
#include <state_machines/RobotStateMachineTaskAction.h>
#include <actionlib/server/simple_action_server.h>

std::string g_robot_name;

typedef actionlib::SimpleActionServer<state_machines::RobotStateMachineTaskAction> SM_SERVER;

ScoutScheduler cSchd;

/**
 * @brief Function which gets executed when any goal is received to actionlib
 * 
 * @param goal for action lib
 * @param as variable to send feedback
 * @param sm ScoutStateMachine object
 */
void execute(const state_machines::RobotStateMachineTaskGoalConstPtr &goal, SM_SERVER *as)
{
	ROS_INFO_STREAM("Received " << g_robot_name << "  State Machine Goal: " << goal->task);

	state_machines::RobotStateMachineTaskResult result;
	STATE_MACHINE_TASK robot_state = (STATE_MACHINE_TASK)goal->task;

	bool output = false;

	cSchd.setState(robot_state);

	result.result = output ? COMMON_NAMES::COMMON_RESULT::SUCCESS : COMMON_NAMES::COMMON_RESULT::FAILED;
	as->setSucceeded(result);

	ROS_INFO_STREAM(g_robot_name << "Goal Finished "<< output);

	return;
}

int main(int argc, char *argv[])
{
	if (argc != 2 && argc != 4)
	{
		ROS_ERROR_STREAM("This node must be launched with the robotname passed as a command line argument!");
		return -1;
	}

	g_robot_name = argv[1];

	ros::init(argc, argv, g_robot_name + "_sm");
	ros::NodeHandle nh;


	SM_SERVER server(nh, g_robot_name + COMMON_NAMES::STATE_MACHINE_ACTIONLIB, boost::bind(&execute, _1, &server), false);
	// server.registerPreemptCallback(boost::bind(&cancelGoal, sm));
	server.start();

	ROS_INFO("Started Scout State Machine Actionlib Server");

	try {
      cSchd.addState(new Search());
      cSchd.addState(new Undock());
      cSchd.addState(new Locate());
      // cSchd.addState(new IdleState());
      // cSchd.setInitialState(ROBOT_IDLE_STATE);
      cSchd.setInitialState(SCOUT_SEARCH_VOLATILE);
			cSchd.exec();
	  ros::spin();
	}
	catch(StateMachineException& ex) {
		std::cerr << "[ERROR] " << ex.getMessage() << std::endl;
	}
	// ros::spin();

	ROS_WARN("Scout state machine died!\n");

	return 0;
}