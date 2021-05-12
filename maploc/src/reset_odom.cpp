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
#include "maploc/ResetOdom.h"

#include <map> // for map operations

/* Goal of Node: 
    - Subscribe to topic or service for resetting odometry
    - input from subscriber: robot name, pose to reset w.r.t.
    - end result: robot's odometry is reset w.r.t. the input pose
*/

// bool gt_first_hauler_1 = true;
// bool gt_first_excavator_1 = true;
// bool gt_first_scout_1 = true;

// stored ground truths
geometry_msgs::PoseStamped gt_pose_hauler_1;
std::map<std::string, geometry_msgs::PoseStamped> gt_pose_values;


//Startup ROS

// robot_first["gt_first_hauler_1"] = true;
// robot_first["gt_first_excavator_1"] = true;
// robot_first["gt_first_scout_1"] = true;

// stored whether first ground truth call made
std::map<std::string, bool> robot_first;

/**
 * @brief:
 * DISCLAIMER: 
 * 
 * @param argv : REQUIRED paramter is the robot name eg. small_scout_1
 * @return int 
 */

// obtains the ground truth pose of the rover using the 1x per simulation ground truth service call
geometry_msgs::PoseStamped getTruePose(std::string robot_name){
    ros::NodeHandle nh;
    // service client for calling the LocalizationSrv service for each rover
    ros::ServiceClient get_true_pose_client;
    // set up the ground truth service call client
    get_true_pose_client = nh.serviceClient<srcp2_msgs::LocalizationSrv>("/" + robot_name + COMMON_NAMES::TRUE_POSE_SRV);
    
    // set up the request/response message for LocalizationSrv
    srcp2_msgs::LocalizationSrv loc_pose;
    loc_pose.request.call = true; 
    
    // variable for storing the service pose
    geometry_msgs::Pose pose_wrt_heightmap;

    if(get_true_pose_client.call(loc_pose)){
        pose_wrt_heightmap = loc_pose.response.pose;
        ROS_INFO("True Pose Obtained");
    }
    else {
        ROS_INFO("True Pose Not Obtained, we'll get em next time");
        // return an empty pose (TODO: proper exception thrown)
    }        
    
    // convert true pose into a PoseStamped
    geometry_msgs::PoseStamped stamped_pose;
    stamped_pose.pose = pose_wrt_heightmap;

    // return PoseStamped true pose (or empty pose if true pose not obtained)
    return stamped_pose;  
}
// irst robot 
// resets the rtabmap odometry based on the input PoseStampedgeometry_msgs::PoseStamped
bool resetOdomPose(std::string robot_name, geometry_msgs::PoseStamped stamped_pose) {
    ros::NodeHandle nh;
    tf2::Quaternion q(stamped_pose.pose.orientation.x,
                        stamped_pose.pose.orientation.y,
                        stamped_pose.pose.orientation.z,
                        stamped_pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);

    // set up the odometry reset service client and convert the PoseStamped into the required rtabmap ResetPose
    ros::ServiceClient rtabmap_client = nh.serviceClient<rtabmap_ros::ResetPose>(robot_name + COMMON_NAMES::RESET_POSE_CLIENT);
    rtabmap_ros::ResetPose pose;
    pose.request.x = stamped_pose.pose.position.x;
    pose.request.y = stamped_pose.pose.position.y;
    pose.request.z = stamped_pose.pose.position.z;
    pose.request.roll = r;
    pose.request.pitch = p;
    pose.request.yaw = y;

    // ensure that the rtabmap odom client is ready for reset
    ROS_INFO("Waiting for rtabmap client");
    
    rtabmap_client.waitForExistence();

    ROS_INFO("Rtabmap client loaded");

    // reset the odometry w.r.t. the input pose
    if(rtabmap_client.call(pose)){
        ROS_INFO("Pose initialized for rtabmap");
        return true;
    }
    else{
        ROS_INFO("RTabMap initialize pose failed.");
        return false;
    }
    
}

// service callback function that resets the odometry of the specified rover w.r.t. the specified pose
// if not_gt
// method returns a boolean due to services needing to have a boolean return statement
//bool resetOdom(geometry_msgs::PoseStamped &ref_pose, std::string &target_robot_name, bool& not_gt) {
//bool resetOdom()
bool resetOdom(maploc::ResetOdom::Request &req, maploc::ResetOdom::Response &res) {
    /*
    geometry_msgs/PoseStamped ref_pose
    string target_robot_name
    bool not_gt
    ---
    bool success
    */
    geometry_msgs::PoseStamped ref_pose = req.ref_pose;
    std::string target_robot_name = req.target_robot_name;
    bool not_gt = req.not_gt;
    if(!not_gt) {
        // map<string, bool>::iterator it;
        // calls the desired robot within the robot_first map
        std::map<std::string, bool>::iterator it = robot_first.find(target_robot_name);
        // if the robot called does not exist, throw an exception
        // TODO: make sure this throws an exception 
        if (it == robot_first.end()) {
            ROS_INFO("No robot was found with input target robot name!");
            res.success = false;
            return false;
        }
        // if gt_first_robot call is true
        if (it->second) {
            ref_pose = getTruePose(target_robot_name);
            // store the ground truth pose of the rover for subsequent calls
            gt_pose_values[target_robot_name] = ref_pose;
            // set that used robot to now false
            robot_first[target_robot_name] = false;
        } // use the stored ground truth value for the reset 
        else{
            ref_pose = gt_pose_values[target_robot_name];
        }
        // resets the rtabmap odom based on the ground truth position from get_true_pose
        resetOdomPose(target_robot_name, ref_pose);
        res.success = true;
        return true;
    } else {
        // resets the rtabmap odom based on the specified pose
        resetOdomPose(target_robot_name, ref_pose);
        res.success = true;
        return true;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_reset_service_node");
    ros::NodeHandle nh;


    robot_first["gt_first_hauler_1"] = true;
    robot_first["gt_first_excavator_1"] = true;
    robot_first["gt_first_scout_1"] = true;

    // gt_first_hauler_1 = true;
    // gt_first_excavator_1 = true;
    // gt_first_scout_1 = true;

    // gt_pose_hauler_1 = 
    // gt_pose_values
    // set up service for obtaining rover name and resets pose
    ros::ServiceServer service = nh.advertiseService("/capricorn/reset_rover_odom_srv", resetOdom);
    // set up reset for odometry

    // return an indicator that reset is complete
    ROS_INFO("odometry has been reset");

    ros::spin();
    return 0;
}