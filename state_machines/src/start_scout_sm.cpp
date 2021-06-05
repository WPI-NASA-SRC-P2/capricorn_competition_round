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

template<class T>
std::map<STATE_MACHINE_TASK, T> states_map_;

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

template<class T>
bool execState(T state_class)
{
	bool output = false;
	bool entry_point = state_class->entryPoint();
	ROS_INFO("Okay....");
	if(entry_point)
		output = state_class->exec();
	else 
		ROS_ERROR("Searching failed in entryPoint");
	return output;
}

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

	if (!checkTask(robot_state))
	{
		// the class is not valid, send the appropriate result
		result.result = COMMON_NAMES::COMMON_RESULT::INVALID_GOAL;
		as->setAborted(result, "Invalid Task");
		ROS_ERROR_STREAM("Invalid Task - " << g_robot_name << " State Machine");
		return;
	}

	static STATE_MACHINE_TASK curr_state;
	curr_state = robot_state;
	bool output = false;
	output = execState<ScoutBaseState*>(states_map_<ScoutBaseState*>[robot_state]);
	ROS_INFO_STREAM("Output:"<<output);
	
	// bool entry_point = states_map_[req_task]->entryPoint();
	// if(entry_point)
	// 	output = states_map_[req_task]->exec();
	// else 
	// 	ROS_ERROR("Searching failed in entryPoint");
	

	result.result = output ? COMMON_NAMES::COMMON_RESULT::SUCCESS : COMMON_NAMES::COMMON_RESULT::FAILED;
	as->setSucceeded(result);

	ROS_INFO_STREAM(g_robot_name << "Goal Finished "<< output);

	return;
}

void initStates(const ros::NodeHandle& nh, const std::string& robot_name)
{
	states_map_<ScoutBaseState*>.insert(std::make_pair<STATE_MACHINE_TASK, Undock*>(SCOUT_UNDOCK, new Undock(nh, robot_name)));
	states_map_<ScoutBaseState*>.insert(std::make_pair<STATE_MACHINE_TASK, Search*>(SCOUT_SEARCH_VOLATILE, new Search(nh, robot_name)));
	states_map_<ScoutBaseState*>.insert(std::make_pair<STATE_MACHINE_TASK, Locate*>(SCOUT_LOCATE_VOLATILE, new Locate(nh, robot_name)));
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

	// ScoutStateMachine *sm = new ScoutStateMachine(nh, g_robot_name);
	initStates(nh, g_robot_name);

	SM_SERVER server(nh, g_robot_name + COMMON_NAMES::STATE_MACHINE_ACTIONLIB, boost::bind(&execute, _1, &server), false);
	// server.registerPreemptCallback(boost::bind(&cancelGoal, sm));
	server.start();

	ROS_INFO("Started Scout State Machine Actionlib Server");
	ros::spin();

	ROS_WARN("Scout state machine died!\n");

	return 0;
}