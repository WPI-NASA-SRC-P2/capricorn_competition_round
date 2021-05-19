/**
 * @file start_excavator_sm.cpp
 * @author Ashay Aswale (asaswale.wpi.edu), Mahimana Bhatt (mbhatt@wpi.edu)
 * @brief This ros node creates an action library for the excavator, handles every task present in 
 * COMMON_NAMES::STATAE_MACHINE_TASK for excavator
 * 
 * @version 0.1
 * @date 2021-05-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <state_machines/excavator_state_machine.h>

std::string g_robot_name;

/**
 * @brief Checking if the input task is in the task list of the excavator,
 * if any new task is added, it should be added in common names and then to EXCAVATOR_TASKS
 * in operation/excavator_state_machine.h
 * 
 * @param task COMMON_NAMES::STATE_MACHINE_TASK
 * @return true 
 * @return false 
 */
bool checkTask(STATE_MACHINE_TASK task)
{
	return EXCAVATOR_TASKS.find(task) != EXCAVATOR_TASKS.end();
}

/**
 * @brief Function which gets executed when any goal is received to actionlib
 * 
 * @param goal for action lib
 * @param as variable to send feedback
 * @param sm ExcavatorStateMachine object
 */
void execute(const state_machines::RobotStateMachineTaskGoalConstPtr &goal, SM_SERVER *as, ExcavatorStateMachine *sm)
{
	ROS_INFO_STREAM(g_robot_name << " State Machine: Received Goal: " << goal->task);

	state_machines::RobotStateMachineTaskResult result;

	geometry_msgs::Pose testPose;
	testPose.position.x = 4;                       //Answer to life, universe and everything. 
	testPose.position.y = 2;
	testPose.position.z = 0;
	testPose.orientation.w = 1;

	// Waiting for the servers to start
	sm->navigation_client_->waitForServer();
	sm->excavator_arm_client_->waitForServer();
	sm->navigation_vision_client_->waitForServer(); //Not being used currently
	ROS_INFO_STREAM(g_robot_name << " State Machine : Servers Connected, Executing Goal");

	STATE_MACHINE_TASK robot_state = (STATE_MACHINE_TASK)goal->task;

	if (!checkTask(robot_state))
	{
		// the class is not valid, send the appropriate result
		result.result = COMMON_RESULT::INVALID_GOAL;
		as->setAborted(result, "Invalid Task");
		ROS_ERROR_STREAM(g_robot_name << " State Machine : Invalid Task");
		return;
	}

	bool output = false;

	switch (robot_state)
	{
	case STATE_MACHINE_TASK::EXCAVATOR_GO_TO_LOC:
		output = sm->goToLoc(goal->goal_loc);
		break;
	case STATE_MACHINE_TASK::EXCAVATOR_GO_TO_SCOUT:
		output = sm->goToScout();
		break;
	case STATE_MACHINE_TASK::EXCAVATOR_PARK_AND_PUB:
		output = sm->parkExcavator();
		break;
	case STATE_MACHINE_TASK::EXCAVATOR_DIG_AND_DUMP_VOLATILE:
		output = sm->digAndDumpVolatile();
		break;
	case STATE_MACHINE_TASK::EXCAVATOR_GOTO_DEFAULT_ARM_POSE:
		output = sm->goToDefaultArmPosition();
		break;
	case STATE_MACHINE_TASK::EXCAVATOR_RESET_ODOM_GROUND_TRUTH:
		output = sm->resetOdometry();
		break;
	case STATE_MACHINE_TASK::EXCAVATOR_RESET_ODOM:
		output = sm->resetOdometry(testPose);
		break;
	case STATE_MACHINE_TASK::EXCAVATOR_SYNC_ODOM :
		output = sm->resetOdometry(testPose);
		break;
	default:
		ROS_ERROR_STREAM(sm->robot_name_ + " state machine encountered unhandled state!");
		break;
	}

	result.result = output ? COMMON_RESULT::SUCCESS : COMMON_RESULT::FAILED;
	as->setSucceeded(result);

	ROS_INFO_STREAM(g_robot_name << " State Machine: Goal Finished");

	return;
}

/**
 * @brief Function called when the goal is cancelled
 * 
 */
void cancelGoal(ExcavatorStateMachine *sm)
{
	ROS_INFO_STREAM(g_robot_name << " State Machine : Cancelling Goal");

	sm->navigation_vision_client_->cancelGoal();
	sm->navigation_client_->cancelGoal();
	sm->excavator_arm_client_->cancelGoal();
}

int main(int argc, char *argv[])
{
	if (argc != 2 && argc != 4)
	{
		ROS_ERROR_STREAM("This node must be launched with the robotname passed as a command line argument!");
		return -1;
	}

	g_robot_name = argv[1];

	ros::init(argc, argv, g_robot_name + STATE_MACHINE_SERVER_NODE_NAME);
	ros::NodeHandle nh;

	ExcavatorStateMachine *sm = new ExcavatorStateMachine(nh, g_robot_name);

	SM_SERVER server(nh, g_robot_name + STATE_MACHINE_ACTIONLIB, boost::bind(&execute, _1, &server, sm), false);
	server.registerPreemptCallback(boost::bind(&cancelGoal, sm));
	server.start();

	ROS_INFO_STREAM(g_robot_name << " State Machine: Started");
	ros::spin();

	ROS_WARN_STREAM(g_robot_name << " State Machine: Died!\n");

	return 0;
}