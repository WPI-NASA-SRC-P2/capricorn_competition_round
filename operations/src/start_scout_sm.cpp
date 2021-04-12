#include <string>
#include <ros/ros.h>
#include <operations/scout_state_machine.h>

int main(int argc, char* argv[])
{
    if(argc != 2 && argc != 4)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname passed as a command line argument!");

        return -1;
    }

    std::string robot_name(argv[1]);

    ros::init(argc, argv, robot_name + "_sm");
    ros::NodeHandle nh;

    ScoutStateMachine* sm = new ScoutStateMachine(nh, robot_name);

    sm->startStateMachine();

    ROS_INFO("Scout state machine died!\n");
    return 0;
}