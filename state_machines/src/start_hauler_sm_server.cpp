/**
 * @file start_hauler_sm_server.cpp
 * @author Mahimana Bhatt (mbhatt@wpi.edu)
 * @brief This ros node creates an action library for the hauler, handles every task present in 
 * COMMON_NAMES::STATAE_MACHINE_TASK for hauler
 * @version 0.1
 * @date 2021-05-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <state_machines/hauler_state_machine.h>

std::string g_robot_name;

/**
 * @brief Checking if the input task is in the task list of the hauler,
 * if any new task is added, it should be added in common names and then to HAULER_TASKS
 * in operation/hauler_state_machine.h
 * 
 * @param task COMMON_NAMES::STATE_MACHINE_TASK
 * @return true : if found
 * @return false : otherwise
 */
bool checkTask(STATE_MACHINE_TASK task)
{
    return HAULER_TASKS.find(task) != HAULER_TASKS.end();
}

/**
 * @brief Function called when the goal is cancelled
 * 
 */
void cancelGoal(HaulerStateMachine *sm)
{
    ROS_INFO_STREAM(g_robot_name << " State Machine : Cancelling Goal");

    sm->navigation_client_->cancelGoal();
    sm->hauler_client_->cancelGoal();
    sm->navigation_vision_client_->cancelGoal();
    sm->park_robot_client_->cancelGoal();
}

/**
 * @brief Function which gets executed when any goal is received to actionlib
 * 
 * @param goal for action lib
 * @param as variable to send feedback
 * @param sm HaulerStateMachine object
 */
void execute(const state_machines::RobotStateMachineTaskGoalConstPtr &goal, SM_SERVER *as, HaulerStateMachine *sm)
{
    ROS_INFO_STREAM(g_robot_name << " State Machine: Received Goal: " << goal->task);

    state_machines::RobotStateMachineTaskResult result;

    // Waiting for the servers to start
    sm->navigation_client_->waitForServer();
    sm->hauler_client_->waitForServer();
    sm->navigation_vision_client_->waitForServer(); //Not being used currently
    sm->park_robot_client_->waitForServer();        //Not being used currently

    cancelGoal(sm);

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
    case STATE_MACHINE_TASK::HAULER_GO_TO_LOC:
        output = sm->goToLoc(goal->goal_loc);
        break;
    case STATE_MACHINE_TASK::HAULER_FOLLOW_EXCAVATOR:
        output = sm->followExcavator();
        break;
    case STATE_MACHINE_TASK::HAULER_PARK_AT_EXCAVATOR:
        output = sm->parkAtExcavator();
        break;
    case STATE_MACHINE_TASK::HAULER_GO_TO_PROC_PLANT:
        output = sm->goToProcPlant();
        break;
    case STATE_MACHINE_TASK::HAULER_PARK_AT_HOPPER:
        output = sm->parkAtHopper();
        break;
    case STATE_MACHINE_TASK::HAULER_DUMP_VOLATILE:
        output = sm->dumpVolatile();
        break;
    case STATE_MACHINE_TASK::HAULER_UNDOCK_EXCAVATOR:
        output = sm->undockExcavator();
        break;
    case STATE_MACHINE_TASK::HAULER_UNDOCK_HOPPER:
        output = sm->undockHopper();
        break;
    case STATE_MACHINE_TASK::HAULER_DUMP_VOLATILE_TO_PROC_PLANT:
        output = sm->dumpVolatileToProcPlant();
        break;
    case STATE_MACHINE_TASK::HAULER_GO_BACK_TO_EXCAVATOR:
        output = sm->goBackToExcavator(goal->goal_loc);
        break;
    case STATE_MACHINE_TASK::HAULER_RESET_ODOM: //Reset odometry when it goes to processing plant
        output = sm->resetOdometry();
        break;
    case STATE_MACHINE_TASK::HAULER_RESET_ODOM_AT_HOPPER: //Reset odometry when it goes to processing plant
        output = sm->resetOdometryAtHopper();
        break;
    case STATE_MACHINE_TASK::HAULER_FACE_PROCESSING_PLANT:
        output = sm->faceProcessingPlant();
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

    HaulerStateMachine *sm = new HaulerStateMachine(nh, g_robot_name);

    SM_SERVER server(nh, g_robot_name + STATE_MACHINE_ACTIONLIB, boost::bind(&execute, _1, &server, sm), false);
    server.registerPreemptCallback(boost::bind(&cancelGoal, sm));
    server.start();

    ROS_INFO_STREAM(g_robot_name << " State Machine: Started");
    ros::spin();

    ROS_WARN_STREAM(g_robot_name << " State Machine: Died!\n");

    return 0;
}