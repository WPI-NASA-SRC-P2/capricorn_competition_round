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
#include <geometry_msgs/Pose.h>
#include <state_machines/scout_state_machine.h>


std::string g_robot_name;


/**
 * @brief Checking if the input task is in the task list of the scout,
 * if any new task is added, it should be added in common names and then to SCOUT_TASKS
 * in operation/scout_state_machine.h
 * 
 * @param task COMMON_NAMES::STATE_MACHINE_TASK
 * @return true 
 * @return false 
 */
bool checkTask(STATE_MACHINE_TASK task)
{
	return SCOUT_TASKS.find(task) != SCOUT_TASKS.end();
}

/**
 * @brief Function which gets executed when any goal is received to actionlib
 * 
 * @param goal for action lib
 * @param as variable to send feedback
 * @param sm ScoutStateMachine object
 */
void execute(const state_machines::RobotStateMachineTaskGoalConstPtr &goal, SM_SERVER *as, ScoutStateMachine *sm)
{
	ROS_INFO_STREAM("Received " << g_robot_name << "  State Machine Goal: " << goal->task);
	geometry_msgs::Pose testPose;
	testPose.position.x = 4;                       //Answer to life, universe and everything. 
	testPose.position.y = 2;
	testPose.position.z = 0;
	testPose.orientation.w = 1;

	state_machines::RobotStateMachineTaskResult result;
  state_machines::RobotStateMachineTaskFeedback feedback;
  feedback.volatile_found = false;
  as->publishFeedback(feedback);

	// Waiting for the servers to start
  sm->resource_localiser_client_->waitForServer(); 
  
	ROS_INFO_STREAM(g_robot_name << ": Servers Connected, Executing Goal");

	STATE_MACHINE_TASK robot_state = (STATE_MACHINE_TASK)goal->task;

	if (!checkTask(robot_state))
	{
		// the class is not valid, send the appropriate result
		result.result = COMMON_NAMES::COMMON_RESULT::INVALID_GOAL;
		as->setAborted(result, "Invalid Task");
		ROS_ERROR_STREAM("Invalid Task - " << g_robot_name << " State Machine");
		return;
	}

	bool output = false;

	switch (robot_state)
	{
	case STATE_MACHINE_TASK::SCOUT_SEARCH_VOLATILE:
		output = sm->startSearchingVolatile();
    // returns true if volatile found
    // if volatile found, then announce that it found vol
    // And start finding volatile on its own without supervisor
    // telling what to do (cause its obvious)
    if(output)
    {
      feedback.volatile_found = true;
      as->publishFeedback(feedback);
      output = sm->locateVolatile();
    }
		break;
	case STATE_MACHINE_TASK::SCOUT_STOP_SEARCH:
		output = sm->stopSearchingVolatile();
		break;
	case STATE_MACHINE_TASK::SCOUT_LOCATE_VOLATILE:
		output = sm->locateVolatile();
		break;
	case STATE_MACHINE_TASK::SCOUT_UNDOCK:
		output = sm->undockRobot();
		break;
	case STATE_MACHINE_TASK::SCOUT_RESET_ODOM_GROUND_TRUTH:
		output = sm->resetOdometry();
		break;
	case STATE_MACHINE_TASK::SCOUT_RESET_ODOM:
		output = sm->resetOdometry(testPose);
		break;

	default:
		ROS_ERROR_STREAM(sm->robot_name_ + " state machine encountered unhandled state!");
		break;
	}
	result.result = output ? COMMON_NAMES::COMMON_RESULT::SUCCESS : COMMON_NAMES::COMMON_RESULT::FAILED;
	as->setSucceeded(result);

	ROS_INFO_STREAM(g_robot_name << "Goal Finished "<< output);

	return;
}

/**
 * @brief Function called when the goal is cancelled
 * 
 */
void cancelGoal(ScoutStateMachine *sm)
{
	ROS_INFO_STREAM("Cancelling " << g_robot_name << "  State Machine Goal");
  sm->stopSearchingVolatile();
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

	ScoutStateMachine *sm = new ScoutStateMachine(nh, g_robot_name);

	SM_SERVER server(nh, g_robot_name + COMMON_NAMES::STATE_MACHINE_ACTIONLIB, boost::bind(&execute, _1, &server, sm), false);
	server.registerPreemptCallback(boost::bind(&cancelGoal, sm));
	server.start();

	ROS_INFO("Started Scout State Machine Actionlib Server");
	ros::spin();

	ROS_WARN("Scout state machine died!\n");

	return 0;
}