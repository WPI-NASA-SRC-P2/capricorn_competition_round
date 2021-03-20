/*
Copied and modified from qualification phase: https://github.com/WPI-NASA-SRC-P2/TeamCapricorn/blob/master/capricorn_examples/src/publish_cheat_odom.cpp
MODIFIED BY: Mahimana Bhatt
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetLinkState.h>
#include <maploc/PosePR.h>
#include <utils/common_names.h>
#include <math.h>

#define UPDATE_HZ 10

ros::ServiceClient client;
gazebo_msgs::GetLinkState req;
std::string robot_name;

geometry_msgs::Pose ground_truth_3d(std::string model_name) {
    req.request.link_name = model_name + COMMON_NAMES::SENSOR_BAR_GAZEBO;
    req.request.reference_frame = COMMON_NAMES::PROCESSING_PLANT_LINK_GAZEBO;
    if (client.call(req))
    {
        return req.response.link_state.pose;
    }
    else
    {
        ROS_ERROR("Something went wrong in getting robot pose from gazebo.");
    }
}

maploc::PosePR ground_truth_2d(std::string model_name, geometry_msgs::Pose pose_robot) {
    req.request.link_name = COMMON_NAMES::REPAIR_STATION_LINK_GAZEBO;
    req.request.reference_frame = COMMON_NAMES::PROCESSING_PLANT_LINK_GAZEBO;
    if (client.call(req))
    {
        geometry_msgs::Pose pose_rs = req.response.link_state.pose;

        float radius = sqrt(pow(pose_robot.position.x, 2) +  pow(pose_robot.position.y, 2) * 1.0);

        float dot = pose_rs.position.x * pose_robot.position.x + pose_rs.position.y * pose_robot.position.y;
        float det = pose_rs.position.x * pose_robot.position.y - pose_rs.position.y * pose_robot.position.x;
        float angle = atan2(det, dot);

        if(angle < 0) {
            angle = 2 * M_PI + angle;
        }
        
        maploc::PosePR pr_msg;
        pr_msg.angle = angle;
        pr_msg.radius = radius;
        pr_msg.robot_name = model_name;
        return pr_msg;
    }
    else
    {
        ROS_ERROR("Something went wrong in getting repair station from gazebo.");
    }
}