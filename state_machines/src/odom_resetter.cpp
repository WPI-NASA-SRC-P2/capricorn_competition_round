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
#include <state_machines/robot_state_status.h>

bool imu_message_received = false;
bool odom_message_received = false;
bool wait_till_reset = false;

nav_msgs::Odometry global_odometry;
sensor_msgs::Imu global_imu;

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

void robot_state_callback(state_machines::robot_state_status robot_state_info) 
{   
    std::string curr_bot = robot_state_info.robot_name;
    int curr_state = robot_state_info.robot_current_state;
    bool curr_state_done = robot_state_info.current_state_done;
    bool last_state_succeeded = robot_state_info.last_state_succeeded;

    wait_till_reset = (curr_bot == COMMON_NAMES::HAULER_1_NAME && 
                                                 curr_state == COMMON_NAMES::HAULER_DUMP_VOLATILE_TO_PROC_PLANT && 
                                                 curr_state_done == true && 
                                                 last_state_succeeded == true);

}

void getOdomRPY(double &r, double &p, double &y)
{
    geometry_msgs::Quaternion init_quat = global_odometry.pose.pose.orientation;//pose.pose.orientation from odom
    tf2::Quaternion init_tf_quat(init_quat.x,
                                 init_quat.y,
                                 init_quat.z,
                                 init_quat.w);
    
    tf2::Matrix3x3 init_m(init_tf_quat);
    init_m.getRPY(r, p, y);
}

void getImuRPY(double &r, double &p, double &y)
{
    geometry_msgs::Quaternion init_quat = global_imu.orientation;
    tf2::Quaternion init_tf_quat(init_quat.x,
                                 init_quat.y,
                                 init_quat.z,
                                 init_quat.w);
    
    tf2::Matrix3x3 init_m(init_tf_quat);
    init_m.getRPY(r, p, y);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_corrector");
    ros::NodeHandle nh;

    std::string robot_name = argv[1];
    
    ros::Subscriber camera_odom_sub = nh.subscribe("/"+ robot_name + COMMON_NAMES::RTAB_ODOM_TOPIC, 223, odom_callback);
    ros::Subscriber imu_odom_sub = nh.subscribe("/"+ robot_name + "/imu", 223, imu_callback);
    ros::Subscriber robot_state_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::ROBOTS_CURRENT_STATE_TOPIC, 223, robot_state_callback);

    ros::ServiceClient reset_odom_to_pose_client = nh.serviceClient<rtabmap_ros::ResetPose>(robot_name + COMMON_NAMES::RESET_POSE_CLIENT);

    // Wait until we have received a message from both odom and imu
    bool name_is_not_hauler_1 = robot_name != COMMON_NAMES::HAULER_1_NAME;
    
    while(ros::ok() && !((imu_message_received && odom_message_received) && (name_is_not_hauler_1 || wait_till_reset)))
    {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    ROS_WARN("Hauler Resets Started");

    double init_odom_r, init_odom_p, init_odom_y;
    double init_imu_r, init_imu_p, init_imu_y;
    // do
    // {
    getOdomRPY(init_odom_r, init_odom_p, init_odom_y);
    getImuRPY(init_imu_r, init_imu_p, init_imu_y);
    ros::Duration(0.1).sleep();

    ROS_ERROR_STREAM(  "init_imu_r: " << init_imu_r<< "  init_imu_p: " << init_imu_p<< "  init_imu_y: " << init_imu_y);
    //     ros::spinOnce();
    // } while ((init_odom_r != init_odom_r || init_odom_p != init_odom_p || init_odom_y != init_odom_y) && ros::ok());
    
    // Inf loop
    while(ros::ok())
    {
        double new_r, new_p, new_y;
        double curr_imu_r, curr_imu_p, curr_imu_y;
        getImuRPY(curr_imu_r, curr_imu_p, curr_imu_y);
        
        // Use low pass filter to get new filtered values of rpy
        new_r = init_odom_r + curr_imu_r - init_imu_r;
        new_p = init_odom_p + curr_imu_p - init_imu_p;
        new_y = init_odom_y + curr_imu_y - init_imu_y;  

        // Assemble data into pose for Rosservice call 
        rtabmap_ros::ResetPose pose;
        pose.request.x = global_odometry.pose.pose.position.x;
        pose.request.y = global_odometry.pose.pose.position.y;
        pose.request.z = global_odometry.pose.pose.position.z;
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