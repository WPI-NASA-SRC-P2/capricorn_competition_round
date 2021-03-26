#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include "path_planner.h"

//How fast the main loop of the node will run.
#define UPDATE_HZ 10

void callBack(const nav_msgs::OccupancyGrid& gridValues);

int main(int argc, char *argv[])
{
    // ROS initialization
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;

    ROS_INFO("Started Path Planner node");

    //Subscribe to the node publishing map values 
    ros::Subscriber indexValues = nh.subscribe("/capricorn/Ground_Truth_Map", 1000, callBack);

    ros::spin();

    return 0;
}

void callBack(const nav_msgs::OccupancyGrid& gridValues)
{
    PathPlanner::test(gridValues);
}