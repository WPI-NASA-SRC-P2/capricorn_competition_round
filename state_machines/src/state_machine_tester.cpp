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

#include <state_machines/RobotStateMachineTaskAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <utils/common_names.h>
#include <gazebo_msgs/GetModelState.h> 


typedef actionlib::SimpleActionClient<state_machines::RobotStateMachineTaskAction> g_client;

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname and the target object detection class that you want to go passed asfirst and second command line arguments!");
        return -1;
    }
    bool has_pose = false;
    if (std::string(argv[3]) != "0")//check here if you have pose arguments as well
    {
        ROS_INFO("has_pose = true, argc = ");
        ROS_INFO_STREAM(argc);
        // if yes, save them to the reseting odom pose values
        has_pose = true;
    }

    std::string robot_name(argv[1]);
    ros::init(argc, argv, robot_name + COMMON_NAMES::NAVIGATION_VISION_CLIENT_NODE_NAME); // please launch from capricorn namespace
    ros::NodeHandle n;

    g_client client(robot_name + COMMON_NAMES::STATE_MACHINE_ACTIONLIB, true);
    client.waitForServer();

    state_machines::RobotStateMachineTaskGoal goal;

    // desired target object, any object detection class
    goal.task = std::stoi(argv[2]);

    //If the task needs a pose, then we provide the pose here, currently using the true pose of the object
    if(has_pose)
    {
        std::string target_name(argv[3]);
        ROS_INFO("GOING TO POSE:");
        
        ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"); 
        gazebo_msgs::GetModelState srv;
        srv.request.model_name = target_name;
        srv.request.relative_entity_name = "map";
        // excavator going to scout = current setup
        if (client.call(srv))
        {
            goal.goal_loc.header = srv.response.header;
            goal.goal_loc.pose = srv.response.pose;
            goal.goal_loc.pose.position.x;
            goal.goal_loc.pose.position.y;
        }
    }

    geometry_msgs::PoseStamped zero_point;
    zero_point.header.frame_id = "map";
    // goal.goal_loc = zero_point;
    client.sendGoal(goal);
    ROS_INFO_STREAM("Goal received " << goal);
    bool finished_before_timeout = client.waitForResult();
    // client.cancelGoal();

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = client.getState();
        state_machines::RobotStateMachineTaskResultConstPtr result = client.getResult();
        if (result->result == COMMON_NAMES::COMMON_RESULT::INVALID_GOAL)
        {
            ROS_INFO("Invalid Task");
        }
        ROS_INFO("Action finished: %s", state.toString().c_str());
        return 0;
    }

    ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
    return 0;
}