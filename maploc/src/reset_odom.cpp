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

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

#include <string>
#include <sstream>
#include <unordered_set>

#include <rtabmap_ros/ResetPose.h>
#include <utils/common_names.h>

#include <srcp2_msgs/LocalizationSrv.h>
#include "maploc/ResetOdom.h"
// #include <operations/navigation_algorithm.h>

#include <map> // for map operations

using namespace COMMON_NAMES;

/* Goal of Node: 
    - Subscribe to topic or service for resetting odometry
    - input from subscriber: robot name, pose to reset w.r.t.
    - end result: robot's odometry is reset w.r.t. the input pose
*/

// stored ground truth at hopper
geometry_msgs::PoseStamped common_gt_pose_value;
bool common_gt_pose_saved = false;

// stored whether first ground truth call made for each robot
std::unordered_set<std::string> used_groundtruth, expected_robot_names{SCOUT_1_NAME, SCOUT_2_NAME, SCOUT_3_NAME, EXCAVATOR_1_NAME, EXCAVATOR_2_NAME, EXCAVATOR_3_NAME, HAULER_1_NAME, HAULER_2_NAME, HAULER_3_NAME};

//Used for the transformation made in the transformPose function. 
tf2_ros::Buffer buffer_;
tf2_ros::TransformListener *listener_;

 static bool transformPose(geometry_msgs::PoseStamped& pose, const std::string& frame, const tf2_ros::Buffer& tf_buffer, float duration = 0.1, int tries = 10);

/** #TODO: Copied from operations/navigation_algorithm.h as I couldnt include it in cmake or package.xml due to build issues, find a way to fix this.*/

bool transformPose(geometry_msgs::PoseStamped& pose, const std::string& frame, const tf2_ros::Buffer& tf_buffer, float duration, int tries)
{
  int count = 0;
  pose.header.stamp = ros::Time(0);
  while(count++ < tries)
  {
    try
    {
      pose = tf_buffer.transform(pose, frame, ros::Duration(duration));
      return true;
    }
    catch(tf2::ExtrapolationException e)
    {
      // do nothing, this is fine if count < tries
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
  }

  ROS_ERROR("[MAPLOC | reset_odom.cpp]: tf2::ExtrapolationException too many times in a row! Failed while transforming from %s to %s at time %d.%d", pose.header.frame_id.c_str(), frame.c_str(), pose.header.stamp.sec, pose.header.stamp.nsec);

	return false;
}

// obtains the ground truth pose of the rover using the one time per simulation ground truth service call
geometry_msgs::PoseStamped getTruePose(std::string robot_name){
    ros::NodeHandle nh;
    
    // service client for calling the LocalizationSrv service for each rover
    ros::ServiceClient get_true_pose_client;
    
    // set up the ground truth service call client
    get_true_pose_client = nh.serviceClient<srcp2_msgs::LocalizationSrv>("/" + robot_name + TRUE_POSE_SRV);
    
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


    // ORIENTATION LEFT OUT OF RESET ODOM
    // IF SOMETHING BREAKS LOOK OVER HERE

    // // convert the poseStamped's quaternion into roll,pitch,yaw for the rtabmap reset odom service call
    // tf2::Quaternion q(stamped_pose.pose.orientation.x,
    //                     stamped_pose.pose.orientation.y,
    //                     stamped_pose.pose.orientation.z,
    //                     stamped_pose.pose.orientation.w);

    // tf2::Matrix3x3 m(q);
    // double r, p, y;
    // m.getRPY(r, p, y);

    // set up the odometry reset service client and convert the PoseStamped into the required rtabmap ResetPose
    ros::ServiceClient rtabmap_client = nh.serviceClient<rtabmap_ros::ResetPose>(robot_name + RESET_POSE_CLIENT);
    rtabmap_ros::ResetPose pose;
    pose.request.x = stamped_pose.pose.position.x;
    pose.request.y = stamped_pose.pose.position.y;
    pose.request.z = stamped_pose.pose.position.z;
    // pose.request.roll = r;
    // pose.request.pitch = p;
    // pose.request.yaw = y;

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

//Reset odom by using the name of both rovers after parking. 
bool resetOdomPose(const std::string& ref_robot, const std::string& target_name) {
    float antenna_distance = 2.0;   /**@HARCODED: Distance b/w rovers when they park wrt each other */
    geometry_msgs::PoseStamped relative_pose;
    relative_pose.header.frame_id = ref_robot + "_base_footprint";
    relative_pose.pose.position.x = antenna_distance;
    relative_pose.pose.orientation.x = 0.0;
    relative_pose.pose.orientation.y = 0.0;
    relative_pose.pose.orientation.z = 1.0;
    relative_pose.pose.orientation.w = 0.0;
    transformPose(relative_pose, MAP, buffer_, 0.1);

    // set up nodehandle again to be able to call the rtabmap reset
    ros::NodeHandle nh;
    
    // convert the poseStamped's quaternion into roll,pitch,yaw for the rtabmap reset odom service call
    tf2::Quaternion q(relative_pose.pose.orientation.x,
                        relative_pose.pose.orientation.y,
                        relative_pose.pose.orientation.z,
                        relative_pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);

    // set up the odometry reset service client and convert the PoseStamped into the required rtabmap ResetPose
    ros::ServiceClient rtabmap_client = nh.serviceClient<rtabmap_ros::ResetPose>(target_name + RESET_POSE_CLIENT);
    rtabmap_ros::ResetPose pose;
    pose.request.x = relative_pose.pose.position.x;
    pose.request.y = relative_pose.pose.position.y;
    pose.request.z = relative_pose.pose.position.z;
    pose.request.roll = r;
    pose.request.pitch = p;
    pose.request.yaw = y;

    // ensure that the rtabmap odom client is ready for reset
    ROS_INFO_STREAM("[MAPLOC | reset_odom.cpp | " + target_name + "]: " + "Waiting for rtabmap client");
    
    rtabmap_client.waitForExistence();

    ROS_INFO_STREAM("[MAPLOC | reset_odom.cpp | " + target_name + "]: " + "Rtabmap client loaded");

    // reset the odometry w.r.t. the input pose
    if(rtabmap_client.call(pose)){
        ROS_INFO_STREAM("[MAPLOC | reset_odom.cpp | " + target_name + "]: " + "Pose initialized for rtabmap");
        return true;
    }
    else{
        ROS_ERROR_STREAM("[MAPLOC | reset_odom.cpp | " + target_name + "]: " + "RTabMap initialize pose failed.");
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
    std::string ref_robot_name = req.ref_robot_name;
    bool use_ground_truth = req.use_ground_truth;
    bool at_hopper = req.at_hopper;

    // if the robot called does not exist, throw an exception
    // TODO: make sure this throws an exception 
    if (expected_robot_names.find(target_robot_name) == expected_robot_names.end()) {
        ROS_ERROR_STREAM("[MAPLOC | reset_odom.cpp | " + target_robot_name + "]: " + "No robot was found with input target robot name!");
        // response for service returns false
        res.success = false;
        return false;
    }

    if(at_hopper)
    {
        if(!common_gt_pose_saved && used_groundtruth.find(target_robot_name) != used_groundtruth.end())
        {
            ROS_ERROR_STREAM("[MAPLOC | reset_odom.cpp | " + target_robot_name + "]: " + " Ground truth already called, sorry!!!");
            // response for service returns false
            res.success = false;
            return false;
        }

        if(!common_gt_pose_saved)
        {
            ROS_INFO_STREAM("[MAPLOC | reset_odom.cpp | " + target_robot_name + "]: " + " Calling ground truth");
            try
            {
                common_gt_pose_value = getTruePose(target_robot_name);
                common_gt_pose_value.header.frame_id = ODOM;
                common_gt_pose_saved = true;
                used_groundtruth.insert(target_robot_name);
            }
            catch (int num)
            {
                res.success = false;
                return false;
            }
        }

        ROS_INFO_STREAM(common_gt_pose_value);
        res.success = resetOdomPose(target_robot_name, common_gt_pose_value);
        if (res.success)
        {        
            ROS_INFO_STREAM("[MAPLOC | reset_odom.cpp | " + target_robot_name + "]: " + " At Hopper Reset Successful");
        }
        return res.success;
    }

    // if using ground truth, get true pose if first call, else use the stored in memory ground truth pose
    if(use_ground_truth) 
    {
        // if gt_first_robot call is true
        if (used_groundtruth.find(target_robot_name) == used_groundtruth.end()) 
        {
            // obtain the ground truth pose of the rover using the provided service call
            ref_pose = getTruePose(target_robot_name);
            used_groundtruth.insert(target_robot_name);
            // store the ground truth pose of the rover for subsequent calls
            // set that used robot to now false

            // resets the rtabmap odom based on the ground truth position from get_true_pose
            res.success = resetOdomPose(target_robot_name, ref_pose);
            return res.success;
        } // use the stored ground truth value for the reset 
        else
        {
            ROS_ERROR_STREAM("[MAPLOC | reset_odom.cpp | " + target_robot_name + "]: " + " Ground truth already called, sorry!!!");
            // response for service returns false
            res.success = false;
            return false;
        }
    } 
    else 
    {
        // resets the rtabmap odom based on the specified pose from the service call (i.e., the hauler's pose if service called by excavator)
        if(ref_robot_name != "")
        {
            res.success = resetOdomPose(ref_robot_name, target_robot_name);
        }
        else
        {
            res.success = resetOdomPose(target_robot_name, ref_pose);
        }    
        return res.success;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_reset_service_node");
    ros::NodeHandle nh;

    listener_ = new tf2_ros::TransformListener(buffer_);

    // set up service for reseting the odometry of the rover 
    ros::ServiceServer service = nh.advertiseService(CAPRICORN_TOPIC + RESET_ODOMETRY, resetOdom);

    // indicate that reset is complete
    ROS_INFO("[MAPLOC | reset_odom.cpp | SERVICE]: reset rover odometry service, online");

    ros::spin();
    return 0;
}