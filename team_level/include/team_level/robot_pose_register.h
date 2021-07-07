#pragma once
#include <utils/common_names.h>
#include <map>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

using namespace COMMON_NAMES;

class RobotPoseRegister
{
public:
    RobotPoseRegister(ros::NodeHandle nh);
    geometry_msgs::PoseStamped getRobotPose(ROBOTS_ENUM robot);

private:
    std::map<ROBOTS_ENUM, geometry_msgs::PoseStamped> ROBOT_ENUM_POSE_MAP;
    ros::Subscriber scout_1_odom_sub, scout_2_odom_sub, scout_3_odom_sub, 
                    excavator_1_odom_sub, excavator_2_odom_sub, excavator_3_odom_sub, 
                    hauler_1_odom_sub, hauler_2_odom_sub, hauler_3_odom_sub;
    
    void odomCBScout1(const nav_msgs::Odometry::ConstPtr &msg);
    void odomCBScout2(const nav_msgs::Odometry::ConstPtr &msg);
    void odomCBScout3(const nav_msgs::Odometry::ConstPtr &msg);
    void odomCBExcavator1(const nav_msgs::Odometry::ConstPtr &msg);
    void odomCBExcavator2(const nav_msgs::Odometry::ConstPtr &msg);
    void odomCBExcavator3(const nav_msgs::Odometry::ConstPtr &msg);
    void odomCBHauler1(const nav_msgs::Odometry::ConstPtr &msg);
    void odomCBHauler2(const nav_msgs::Odometry::ConstPtr &msg);
    void odomCBHauler3(const nav_msgs::Odometry::ConstPtr &msg);
};