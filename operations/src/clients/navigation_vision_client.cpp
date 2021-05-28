/*
Author BY: Mahimana Bhatt
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE

This ros node calls an actionlib to take the specified robot near to the given target. This code does not wait for getting the actionlib
finished with the task, just sends the goal (waits if server is not available) to actionlib and exits.

Command Line Arguments Required:
1. robot_name: eg. small_scout_1, small_excavator_2
2. target object: it can be any object detection class eg. hauler, excavator, scout, processingPlant, repairStation, furnace, hopper, all 
object detection classes are given in COMMON_NAMES
*/

#include <operations/NavigationVisionAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>

typedef actionlib::SimpleActionClient<operations::NavigationVisionAction> g_client;

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname and the target object detection class that you want to go passed asfirst and second command line arguments!");
        return -1;
    }

    std::string robot_name(argv[1]);
    ros::init(argc, argv, robot_name + COMMON_NAMES::NAVIGATION_VISION_CLIENT_NODE_NAME); // please launch from capricorn namespace

    g_client client(robot_name + COMMON_NAMES::NAVIGATION_VISION_ACTIONLIB, true);
    client.waitForServer();

    operations::NavigationVisionGoal goal;

    // desired target object, any object detection class
    goal.desired_object_label = std::string(argv[2]);
    goal.mode = std::stoi(argv[3]);
    geometry_msgs::PoseStamped zero_point;
    zero_point.header.frame_id = "map";
    goal.goal_loc = zero_point;
    client.sendGoal(goal);
    bool finished_before_timeout = client.waitForResult(ros::Duration(1.0));
    // client.cancelGoal();

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = client.getState();
        operations::NavigationVisionResultConstPtr result = client.getResult();
        if (result->result == COMMON_NAMES::COMMON_RESULT::INVALID_GOAL)
        {
            ROS_INFO("Invalid Object Detection Class or Cannot go to the class");
        }
        ROS_INFO("Action finished: %s", state.toString().c_str());
        return 0;
    }

    ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
    return 0;
}
