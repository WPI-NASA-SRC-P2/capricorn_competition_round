#include <string>
#include <ros/ros.h>
#include <operations/navigation_server.h>

int main(int argc, char* argv[])
{
    if(argc != 2 && argc != 4)
    {
        ROS_ERROR_STREAM("This node must be launched with the robotname passed as a command line argument!");

        return -1;
    }

    std::string robot_name(argv[1]);

    ros::init(argc, argv, robot_name + "_nav_server");
    ros::NodeHandle nh("~");

    printf("Constructing nav server\n");

    NavigationServer* ns = new NavigationServer(nh, robot_name);

    printf("Done constructing nav server\n");

    ros::spin();

    printf("NavigationServer died!\n");
}