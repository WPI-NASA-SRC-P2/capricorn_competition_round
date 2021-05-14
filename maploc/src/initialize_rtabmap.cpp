/*
Copied and modified from qualification phase: https://github.com/WPI-NASA-SRC-P2/TeamCapricorn/blob/master/capricorn_odom/src/capricorn_odom_initialize.cpp
MODIFIED BY: Albert Enyedy
Email: ajenyedy@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

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
    bool get_true_pose(argv[2]);

    //Startup ROS
    ros::init(argc, argv, robot_name + COMMON_NAMES::INITALIZE_ODOM_NODE_NAME);
    ros::NodeHandle nh;

    ros::ServiceClient nasa_client;

    // initialize the rtabmap with the true pose to have unified coordinates
    if(get_true_pose){
        ROS_INFO("Initializing odom with get_true_pose service");
        // service client for calling the LocalizationSrv service for each rover
        ros::ServiceClient get_true_pose_client;

        // initialize the LocalizationSrv client on proper rosservice call name: /robot_name/get_true_pose
        get_true_pose_client = nh.serviceClient<srcp2_msgs::LocalizationSrv>("/" + robot_name + COMMON_NAMES::TRUE_POSE_SRV);

        // set up the request/response message for LocalizationSrv
        srcp2_msgs::LocalizationSrv loc_pose;
        loc_pose.request.call = true; // I have no idea what to do here 
        
        // variable for storing the service pose
        geometry_msgs::Pose pose_wrt_heightmap;

        if(get_true_pose_client.call(loc_pose))
        {
            pose_wrt_heightmap = loc_pose.response.pose;
            ROS_INFO("True Pose Obtained");
        }

        // end get_true_pose

        // remainder of the node is just copied from the original initialize_rtabmap (which is now cheat_initialize_rtabmap.cpp)
        // -> as the input is of the same type, but w.r.t. the desired unified heightmap coordinate frame as opposed to w.r.t. the robot's local baseframe
        geometry_msgs::TransformStamped transformStamped;

        geometry_msgs::PoseStamped stampedPose;
        stampedPose.pose = pose_wrt_heightmap;

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
    // initialize the rtabmap without get true pose, such that true pose can be obtained with reset pose service after reaching a location instead
    else{
        ROS_INFO("Initializing odom wihout get_true_pose service");
    }
}