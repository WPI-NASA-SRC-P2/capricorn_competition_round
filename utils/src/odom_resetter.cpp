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

double roll_avg;
double pitch_avg;
double yaw_avg;
int reading_count = 0;

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

void getInitRPY(double &init_r, double &init_p, double &init_y)
{
    geometry_msgs::Quaternion init_quat = global_odometry.pose.pose.orientation;//pose.pose.orientation from odom
    tf2::Quaternion init_tf_quat(init_quat.x,
                                 init_quat.y,
                                 init_quat.z,
                                 init_quat.w);
    
    tf2::Matrix3x3 init_m(init_tf_quat);
    init_m.getRPY(init_r, init_p, init_y);

    // Set the initial values of rpy data
    roll_avg = init_r;
    pitch_avg = init_p;
    yaw_avg = init_y;
    reading_count++;
}

void updateAverages(double r, double p, double y){

    roll_avg = (roll_avg * (reading_count - 1) + r) / reading_count;
    pitch_avg = (pitch_avg * (reading_count - 1) + p) / reading_count;
    yaw_avg = (yaw_avg * (reading_count - 1) + y) / reading_count;
}

double lowPassExponential(double input, double average, double factor) 
{  
    double retVal = input*factor + (1-factor)*average;  // ensure factor belongs to  [0,1]
} 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_corrector");
    ros::NodeHandle nh;

    std::string robot_name = argv[1];
    
    ros::Subscriber camera_odom_sub = nh.subscribe("/"+ robot_name + "/camera/odom", 223, odom_callback);
    ros::Subscriber imu_odom_sub = nh.subscribe("/"+ robot_name + "/imu", 223, imu_callback);

    ros::ServiceClient reset_odom_to_pose_client = nh.serviceClient<rtabmap_ros::ResetPose>(robot_name + COMMON_NAMES::RESET_POSE_CLIENT);

    // Wait until we have received a message from both odom and imu
    while(ros::ok() && !imu_message_received && !odom_message_received)
    {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    // Get the initial rpy values from the odom
    double init_r, init_p, init_y;
    do
    {
        getInitRPY(init_r, init_p, init_y);
        ros::spinOnce();
    } while ((init_r != init_r || init_p != init_p || init_y != init_y) && ros::ok());
    
    // Inf loop
    while(ros::ok())
    {
        //get current position from odom
        geometry_msgs::Point curr_odom_position = global_odometry.pose.pose.position;
        
        //get current orientation from imu as a quat
        geometry_msgs::Quaternion quat = global_imu.orientation;
        tf2::Quaternion tf_quat(quat.x,
                                quat.y,
                                quat.z,
                                quat.w);

        // convert current orientation from quat to rpy
        tf2::Matrix3x3 m(tf_quat);
        double r, p, y;
        m.getRPY(r, p, y);

        reading_count++;
        
        // Use low pass filter to get new filtered values of rpy
        double new_r, new_p, new_y;
        new_r = lowPassExponential(r, roll_avg, 0.2);
        new_p = lowPassExponential(p, pitch_avg, 0.2);
        new_y = lowPassExponential(y, yaw_avg, 0.2);

        // Update averages for future filtering
        updateAverages(r, p, y);   

        // Assemble data into pose for Rosservice call 
        rtabmap_ros::ResetPose pose;
        pose.request.x = curr_odom_position.x;
        pose.request.y = curr_odom_position.y;
        pose.request.z = curr_odom_position.z;
        pose.request.roll = new_r;
        pose.request.pitch = new_p;
        pose.request.yaw = new_y;

        // call the 'reset pose' rosservice to send updated data
        if(reset_odom_to_pose_client.call(pose)){
            ROS_INFO_STREAM("[UTILS | odom_resetter.cpp | " + robot_name + "]: " + "Pose initialized for rtabmap");
        }
        else{
            ROS_ERROR_STREAM("[UTILS | odom_resetter.cpp | " + robot_name + "]: " + "RTabMap initialize pose failed.");
        }

        // Wait 10 seconds and then do it all again
        ros::Duration(10).sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}