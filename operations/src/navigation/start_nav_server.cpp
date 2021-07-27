#include <string>
#include <ros/ros.h>
#include <operations/navigation_server.h>

typedef actionlib::SimpleActionServer<operations::NavigationAction> Server;

int main(int argc, char* argv[])
{
    if(argc != 2 && argc != 4)
    {
        ROS_ERROR_STREAM("[operations | start_nav_server]: This node must be launched with the robotname passed as a command line argument!");

        return -1;
    }

    std::string robot_name(argv[1]);

    ros::init(argc, argv, robot_name + "_nav_server");
    ros::NodeHandle nh;

    ROS_INFO("[operations | start_nav_server | %s]: Constructing nav server", robot_name.c_str());

    NavigationServer* ns = new NavigationServer(nh, robot_name);

    ROS_INFO("[operations | start_nav_server | %s]: Done constructing nav server", robot_name.c_str());

    ros::spin();

    delete ns;

    ROS_WARN("[operations | start_nav_server | %s]: NavigationServer died!", robot_name.c_str());
}