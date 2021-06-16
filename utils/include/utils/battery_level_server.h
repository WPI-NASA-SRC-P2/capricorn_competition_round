#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "utils/battery_deadlines.h"

class  BatteryLevelServer{
private:
    float distance_;
    float soft_deadline_;
    float hard_deadline_;
    geometry_msgs::PoseStamped target_location_;
    geometry_msgs::PoseStamped current_location_;
    


public:
    ros::Subscriber pose_subscriber;
    //void poseCallback(nav_msgs::Odometry odom);
    bool deadlinesCallback(utils::battery_deadlines::Request &req, utils::battery_deadlines::Response &res);
};