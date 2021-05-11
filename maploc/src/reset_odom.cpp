/*
CREATED BY: Team Bebop
Slack: #team_bebop

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

// TODO: REMOVE UNNECESSARY LIBRARIES
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <string>
#include <sstream>

#include <rtabmap_ros/ResetPose.h>
#include <utils/common_names.h>

#include <srcp2_msgs/LocalizationSrv.h>

/* Goal of Node: 
    - Subscribe to topic or service for resetting odometry
    - input from subscriber: robot name, pose to reset w.r.t.
    - end result: robot's odometry is reset w.r.t. the input pose
*/

// TODO: decide whether to use a rossrv or a rosmsg
// TODO: if using rosmsg, have to define custom message and then update cmake

/**
 * @brief:
 * DISCLAIMER: 
 * 
 * @param argv : REQUIRED paramter is the robot name eg. small_scout_1
 * @return int 
 */

void reset_odom(rtabmap_ros::ResetPose::Request &req, rtabmap_ros::ResetPose::Response &res) {
    req.request.x = 0;
    req.request.y = 0;
    req.request.z = 0;
}

int main(int argc, char *argv[])
{
    //Startup ROS
    ros::init(argc, argv, "odom_reset_service_node");
    ros::NodeHandle nh;

    // set up service for obtaining rover name and resets pose
    ros::ServiceServer service = nh.advertiseService("/capricorn/reset_rover_odom_srv", reset_odom);
    // set up reset for odometry

    // return an indicator that reset is complete
    rospy.loginfo("odometry has been resetted")
}