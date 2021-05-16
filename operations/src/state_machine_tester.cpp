/**
 * @file state_machine_tester.cpp
 * @author Mahimana Bhatt (mbhatt@wpi.edu)
 * @brief This file gives you an interface to directly interact with the state machine of every robot in the simulation
 * @version 0.1
 * @date 2021-05-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <operations/ExcavatorStateMachineTaskAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <utils/common_names.h>

typedef actionlib::SimpleActionClient<operations::ExcavatorStateMachineTaskAction> g_client;

int main(int argc, char** argv)
{
    if(argc < 2)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname and the target object detection class that you want to go passed asfirst and second command line arguments!");
        return -1;
    }

    std::string robot_name(argv[1]);
    ros::init(argc, argv, robot_name + COMMON_NAMES::NAVIGATION_VISION_CLIENT_NODE_NAME);  // please launch from capricorn namespace

    g_client client(robot_name + COMMON_NAMES::STATE_MACHINE_ACTIONLIB, true);
    client.waitForServer();

    operations::ExcavatorStateMachineTaskGoal goal; 

    // desired target object, any object detection class
    goal.task = std::stoi(argv[2]);

    geometry_msgs::PoseStamped zero_point;
    zero_point.header.frame_id = "map";
    goal.goal_loc = zero_point;
    client.sendGoal(goal);    
    bool finished_before_timeout = client.waitForResult();
    // client.cancelGoal();

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = client.getState();
        operations::ExcavatorStateMachineTaskResultConstPtr result = client.getResult();
        if(result->result == COMMON_NAMES::COMMON_RESULT::INVALID_GOAL)
        {
            ROS_INFO("Invalid Task");
        }
        ROS_INFO("Action finished: %s",state.toString().c_str());
        return 0;
    }

    ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
    return 0;
}