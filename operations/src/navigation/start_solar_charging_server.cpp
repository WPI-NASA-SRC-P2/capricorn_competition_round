#include <string>
#include <ros/ros.h>
#include <operations/solar_charging_server.h>

typedef actionlib::SimpleActionServer<operations::SolarModeAction> SolarServer_;

int main(int argc, char* argv[])
{
    if(argc != 2 && argc != 4)
    {
        ROS_ERROR_STREAM("[operations | start_solar_charging_server]: This node must be launched with the robotname passed as a command line argument!");

        return -1;
    }

    std::string robot_name(argv[1]);

    ros::init(argc, argv, robot_name + "_solar_mode_server");
    ros::NodeHandle nh;

    ROS_INFO("[operations | start_solar_charging_server]: Constructing solar charging server", robot_name.c_str());

    SolarModeServer* ns = new SolarModeServer(nh, robot_name);

    ROS_INFO("[operations | start_solar_charging_server]: Done constructing solar charging server", robot_name.c_str());

    ros::spin();

    delete ns;

    ROS_WARN("[operations | start_solar_charging_server]:  Solar charging server died!", robot_name.c_str());
}