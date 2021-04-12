/*
Copied and modified from qualification phase: https://github.com/WPI-NASA-SRC-P2/TeamCapricorn/blob/master/capricorn_odom/src/capricorn_odom_initialize.cpp
MODIFIED BY: Mahimana Bhatt
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <rtabmap_ros/ResetPose.h>
#include <utils/common_names.h>

/**
 * @brief This script resets the pose of RTabMap to the current ground truth pose fetched from gazebo model state.
 * DISCLAIMER: This script sets the inital pose of RTabMap in "robot_base_footprint" frame, make sure the output frame id
 * parameter in launch file for RTabMap is "robot_base_footprint"
 * 
 * DISCLAIMER: "SHOULD NOT BE USED IN SUBMISSION, just for testing and debugging"
 * 
 * @param argv : REQUIRED paramter is the robot name eg. small_scout_1
 * @return int 
 */
int main(int argc, char *argv[])
{
    //Setup robot_name from passed in arguments
    std::string robot_name(argv[1]);

    //Startup ROS
    ros::init(argc, argv, robot_name + COMMON_NAMES::INITALIZE_ODOM_NODE_NAME);
    ros::NodeHandle nh;

    // Variable for getting ground truth pose from gazebo
    geometry_msgs::PoseWithCovarianceStamped true_pose;

    ros::ServiceClient gazebo_client;
    ros::ServiceClient nasa_client;
    
    ROS_INFO("Initializing odom with gazebo");

    gazebo_client = nh.serviceClient<gazebo_msgs::GetModelState>(COMMON_NAMES::MODEL_STATE_QUERY);

    gazebo_msgs::GetModelState state;

    // It is assumed that the model name same as the robot name or whatever argument is supplied 
    state.request.model_name = robot_name;

    // HEIGHTMAP is considered as origin of the simulation (as per the observation it is)
    state.request.relative_entity_name = COMMON_NAMES::HEIGHTMAP;

    if(gazebo_client.call(state))
    {
        true_pose.pose.pose = state.response.pose;
        true_pose.header    = state.response.header;
    }
    else
    {
        ROS_ERROR("Gazebo client call on odom initialization failed.");
        return -1;
    }

    geometry_msgs::TransformStamped transformStamped;

    geometry_msgs::PoseStamped stampedPose;
    stampedPose.pose = true_pose.pose.pose;

    tf2::Quaternion q(stampedPose.pose.orientation.x,
                        stampedPose.pose.orientation.y,
                        stampedPose.pose.orientation.z,
                        stampedPose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);

    ros::ServiceClient rtabmap_client = nh.serviceClient<rtabmap_ros::ResetPose>(robot_name + COMMON_NAMES::RESET_POSE_CLIENT);
    rtabmap_ros::ResetPose pose;
    pose.request.x = stampedPose.pose.position.x;
    pose.request.y = stampedPose.pose.position.y;
    pose.request.z = stampedPose.pose.position.z;

    pose.request.roll = r;
    pose.request.pitch = p;
    pose.request.yaw = y;

    ROS_INFO("Waiting for rtabmap client");
    
    rtabmap_client.waitForExistence();

    ROS_INFO("Rtabmap client loaded");

    if(rtabmap_client.call(pose))
    {
        ROS_INFO("Pose initialized for rtabmap");
    }
    else
    {
        ROS_INFO("RTabMap initialize pose failed.");
    }
}