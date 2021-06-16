#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "utils/battery_deadlines.h"

class  BatteryLevelClient{

private:
geometry_msgs::PoseStamped current_location_;

public:
    ros::Subscriber pose_subscriber;
    static void poseCallback(nav_msgs::Odometry odom);
};
