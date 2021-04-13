/*
Author BY: Mahimana Bhatt
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE

This ros node calls an actionlib to park the specific robot with respect to hopper or excavator. This code does not wait for getting the actionlib
finished with the task, just sends the goal (waits if server is not available) to actionlib and exits.

Command Line Arguments Required:
1. robot_name: eg. small_scout_1, small_excavator_2
2. target object: it can be "hopper" or "excavator"
*/

#include <operations/ParkRobotAction.h>
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>

// defining client for ParkRobot ActionLib
typedef actionlib::SimpleActionClient<operations::ParkRobotAction> g_client;

int main(int argc, char** argv)
{
    if(argc != 3)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname passed as a command line argument!");
        return -1;
    }

    std::string hopper_or_excavator(argv[2]);

    if(hopper_or_excavator != COMMON_NAMES::OBJECT_DETECTION_HOPPER_CLASS && hopper_or_excavator != COMMON_NAMES::OBJECT_DETECTION_EXCAVATOR_CLASS)
    {
        ROS_ERROR_STREAM("Wrong Target Location Given. Please give hopper or excavator as second argument");
        return 0;
    }

    std::string robot_name(argv[1]);
    ros::init(argc, argv, robot_name + COMMON_NAMES::PARK_HAULER_HOPPER_CLIENT_NODE_NAME);

    // initializing the client
    g_client client(robot_name + COMMON_NAMES::PARK_HAULER_ACTIONLIB, true); 

    // wait for the server to run
    client.waitForServer();

    // defining goal
    operations::ParkRobotGoal goal; 
    
    // the second argument given would be the target object for parking, options: hopper or excavator
    goal.hopper_or_excavator = hopper_or_excavator; 

    client.sendGoal(goal);
    
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Reached goal");

    ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
    return 0;
}
