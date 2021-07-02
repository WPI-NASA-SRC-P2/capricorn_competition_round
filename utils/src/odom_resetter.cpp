/*
Author: Chris DeMaio
Email: cjdemaio@wpi.edu
TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

#include <ros/ros.h>
#include <utils/common_names.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <rtabmap_ros/ResetPose.h>
#include <tf2/LinearMath/Matrix3x3.h>

bool imu_message_received = false;
bool odom_message_received = false;

nav_msgs::Odometry global_odometry;
sensor_msgs::Imu global_imu;

void odom_callback(nav_msgs::Odometry odom_data) 
{   
    global_odometry = odom_data;
    odom_message_received = true;

}

void imu_callback(sensor_msgs::Imu imu_data) 
{   
    global_imu = imu_data;
    imu_message_received = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_corrector");
    ros::NodeHandle nh;

    std::string robot_name = argv[1];
    
    ros::Subscriber camera_odom_sub = nh.subscribe("/"+ robot_name + "/camera/odom", 223, odom_callback);
    ros::Subscriber imu_odom_sub = nh.subscribe("/"+ robot_name + "/imu", 223, imu_callback);

    ros::ServiceClient reset_odom_to_pose_client = nh.serviceClient<rtabmap_ros::ResetPose>(robot_name + COMMON_NAMES::RESET_POSE_CLIENT);
    // ros::ServiceClient rtabmap_client = nh.serviceClient<rtabmap_ros::ResetPose>(target_name + RESET_POSE_CLIENT);

    while(ros::ok() && (!imu_message_received && !odom_message_received))
    {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    geometry_msgs::Quaternion init_quat = global_odometry.pose.pose.orientation;//pose.pose.orientation from odom
    tf2::Quaternion init_tf_quat(init_quat.x,
                                 init_quat.y,
                                 init_quat.z,
                                 init_quat.w);
    
    tf2::Matrix3x3 init_m(init_tf_quat);
    double init_r, init_p, init_y;
    init_m.getRPY(init_r, init_p, init_y);

    while(ros::ok())
    {
        //get position from odom
        geometry_msgs::Point position = global_odometry.pose.pose.position;
        
        //get orientation from imu as a quat
        geometry_msgs::Quaternion quat = global_imu.orientation;
        tf2::Quaternion tf_quat(quat.x,
                                quat.y,
                                quat.z,
                                quat.w);

        // convert current orientation to rpy
        tf2::Matrix3x3 m(tf_quat);
        double r, p, y;
        m.getRPY(r, p, y);

        // add init + current rpy
        double new_r, new_p, new_y;
        new_r = init_r + r;
        new_p = init_p + p;
        new_y = init_y + y;

        // Assemble data into pose for Rosservice call 
        rtabmap_ros::ResetPose pose;
        pose.request.x = position.x;
        pose.request.y = position.y;
        pose.request.z = position.z;
        pose.request.roll = new_r;
        pose.request.pitch = new_p;
        pose.request.yaw = new_y;

        // call the 'reset pose' rosservice
        if(reset_odom_to_pose_client.call(pose)){
            ROS_INFO_STREAM("[UTILS | odom_resetter.cpp | " + robot_name + "]: " + "Pose initialized for rtabmap");
        }
        else{
            ROS_ERROR_STREAM("[UTILS | odom_resetter.cpp | " + robot_name + "]: " + "RTabMap initialize pose failed.");
        }

        ros::Duration(1).sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}