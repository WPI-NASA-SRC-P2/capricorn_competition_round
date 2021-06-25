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

// stored ground truths for each robot
std::map<std::string, geometry_msgs::PoseStamped> gt_pose_values;

// stored whether first ground truth call made for each robot
std::map<std::string, bool> robot_first;

// obtains the ground truth pose of the rover using the one time per simulation ground truth service call
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

    // obtain the true pose of the rover using the service if possible
    if(get_true_pose_client.call(loc_pose)){
        pose_wrt_heightmap = loc_pose.response.pose;
        ROS_INFO_STREAM("[MAPLOC | reset_odom.cpp | " + robot_name + "]: " + "True Pose Obtained");
    }
    else {
        ROS_ERROR_STREAM("[MAPLOC | reset_odom.cpp | " + robot_name + "]: " + "True Pose Not Obtained, we'll get em next time");
        // return an empty pose (TODO: proper exception thrown)
    }        

    // convert true pose into a PoseStamped
    geometry_msgs::PoseStamped stamped_pose;
    stamped_pose.pose = pose_wrt_heightmap;

    // return PoseStamped true pose (or empty pose if true pose not obtained)
    return stamped_pose;  
}

// resets the rtabmap odometry based on the input PoseStamped
bool resetOdomPose(std::string robot_name, geometry_msgs::PoseStamped stamped_pose) {
    ros::NodeHandle nh;
    
    // convert the poseStamped's quaternion into roll,pitch,yaw for the rtabmap reset odom service call
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
    ROS_INFO_STREAM("[MAPLOC | reset_odom.cpp | " + robot_name + "]: " + "Waiting for rtabmap client");
    
    rtabmap_client.waitForExistence();

    ROS_INFO_STREAM("[MAPLOC | reset_odom.cpp | " + robot_name + "]: " + "Rtabmap client loaded");

    // debugging info to double check if the pose is properly being converted
    // ROS_INFO("POSE = ");
    // ROS_INFO_STREAM(pose.request);

    // reset the odometry w.r.t. the input pose
    if(rtabmap_client.call(pose)){
        ROS_INFO_STREAM("[MAPLOC | reset_odom.cpp | " + robot_name + "]: " + "Pose initialized for rtabmap");
        return true;
    }
    else{
        ROS_ERROR_STREAM("[MAPLOC | reset_odom.cpp | " + robot_name + "]: " + "RTabMap initialize pose failed.");
        return false;
    }
    
}

// service callback function that resets the odometry of the specified rover w.r.t. the specified pose
// if use_ground_truth
// method returns a boolean due to services needing to have a boolean return statement and to provide feedback on success of service call
bool resetOdom(maploc::ResetOdom::Request &req, maploc::ResetOdom::Response &res) {

    // obtain the service call request data
    geometry_msgs::PoseStamped ref_pose = req.ref_pose;
    std::string target_robot_name = req.target_robot_name;
    bool use_ground_truth = req.use_ground_truth;
    
    // calls the desired robot within the robot_first map
    std::map<std::string, bool>::iterator it = robot_first.find(target_robot_name);

    // if the robot called does not exist, throw an exception
    // TODO: make sure this throws an exception 
    if (it == robot_first.end()) {
        ROS_ERROR_STREAM("[MAPLOC | reset_odom.cpp | " + target_robot_name + "]: " + "No robot was found with input target robot name!");
        // response for service returns false
        res.success = false;
        return false;
    }

    // if using ground truth, get true pose if first call, else use the stored in memory ground truth pose
    if(use_ground_truth) {
        // if gt_first_robot call is true
        if (it->second) {
            // obtain the ground truth pose of the rover using the provided service call
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
        // resets the rtabmap odom based on the specified pose from the service call (i.e., the hauler's pose if service called by excavator)
        resetOdomPose(target_robot_name, ref_pose);
        res.success = true;
        return true;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_reset_service_node");
    ros::NodeHandle nh;

    // ensure that the memory of which rovers already used their ground truth call is reset to true upon starting the node
    robot_first[COMMON_NAMES::HAULER_1] = true;
    robot_first[COMMON_NAMES::SCOUT_1] = true;
    robot_first[COMMON_NAMES::EXCAVATOR_1] = true;

    // set up service for reseting the odometry of the rover 
    ros::ServiceServer service = nh.advertiseService(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::RESET_ODOMETRY, resetOdom);

    // indicate that reset is complete
    ROS_INFO("[MAPLOC | reset_odom.cpp | SERVICE]: reset rover odometry service, online");

    ros::spin();
    return 0;
}