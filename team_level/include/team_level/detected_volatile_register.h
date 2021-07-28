#pragma once
#include <vector>
#include <utils/common_names.h>
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <srcp2_msgs/VolSensorMsg.h>
#include <geometry_msgs/PoseStamped.h>

using namespace COMMON_NAMES;

class DetectedVolatileRegister
{
public:
    DetectedVolatileRegister(ros::NodeHandle nh);
    ~DetectedVolatileRegister(){};

    bool isNewVolatile(ROBOTS_ENUM robot, const geometry_msgs::PoseStamped& robot_pose);
    void registerNewVolatile(ROBOTS_ENUM robot, geometry_msgs::PoseStamped robot_pose);

private:
    const float DISTANCE_THRESHOLD = 10.0;

    ros::Subscriber scout_1_subscriber, scout_2_subscriber;

    inline static std::vector<std::pair<std::string, geometry_msgs::Point>> detected_volatile_list;

    bool isCloseEnough(const geometry_msgs::Point& pose_1, const geometry_msgs::Point& pose_2);

    void scout1VolatileCB(const srcp2_msgs::VolSensorMsg::ConstPtr &msg);
    void scout2VolatileCB(const srcp2_msgs::VolSensorMsg::ConstPtr &msg);

    std::string scout_1_current_volatile, scout_2_current_volatile;
};
