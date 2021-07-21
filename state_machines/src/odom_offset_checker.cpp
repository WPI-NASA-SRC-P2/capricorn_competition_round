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

#define UPDATE_HZ 10

bool imu_message_received = false;
bool odom_message_received = false;
bool wait_till_reset = false;

nav_msgs::Odometry global_odometry;
sensor_msgs::Imu global_imu;

double prev_odom_x, prev_odom_y, prev_odom_z;

void odom_callback(nav_msgs::Odometry odom_data) 
{   
    global_odometry = odom_data;
    odom_message_received = true;
}

void robot_state_callback(state_machines::robot_state_status robot_state_info) 
{   
    std::string curr_bot = robot_state_info.robot_name;
    int curr_state = robot_state_info.robot_current_state;
    bool curr_state_done = robot_state_info.current_state_done;
    bool last_state_succeeded = robot_state_info.last_state_succeeded;

    wait_till_reset = (curr_bot == COMMON_NAMES::HAULER_2_NAME && 
                                                 curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_DUMP_VOLATILE_TO_PROC_PLANT && 
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

int main(int argc, char** argv)
{
    // ****** Initialization stuff ******
    ros::init(argc, argv, "odom_offset_checker");
    ros::NodeHandle nh;
    ros::Rate update_rate(UPDATE_HZ);

    std::string robot_name = argv[1];
    
    ros::Subscriber camera_odom_sub = nh.subscribe("/"+ robot_name + COMMON_NAMES::RTAB_ODOM_TOPIC, 223, odom_callback);
    ros::Subscriber robot_state_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::ROBOTS_CURRENT_STATE_TOPIC, 223, robot_state_callback);

    ros::ServiceClient reset_odom_to_pose_client = nh.serviceClient<rtabmap_ros::ResetPose>(robot_name + COMMON_NAMES::RESET_POSE_CLIENT);

    bool name_is_not_hauler_2 = robot_name != COMMON_NAMES::HAULER_2_NAME;
    // **********************************

    // Wait until we have received a message from both the imu and odom
    while(ros::ok() && !(name_is_not_hauler_2 || wait_till_reset))
    {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    ros::Duration(10).sleep();
    ros::spinOnce();
    double new_odom_x = global_odometry.pose.pose.position.x;
    double new_odom_y = global_odometry.pose.pose.position.y;
    double new_odom_z = global_odometry.pose.pose.position.z;
    double prev_odom_x = new_odom_x, prev_odom_y = new_odom_y, prev_odom_z = new_odom_z;

    double new_r, new_p, new_y;
    getOdomRPY(new_r, new_p, new_y);
    double prev_r = new_r, prev_p = new_p, prev_y = new_y;

    // Inf loop
    while(ros::ok())
    {
        // Run the loop only once we've received another message
        if (odom_message_received)    
        {
            // Initialize variables

            // Store the latest odometry position readings
            
            // Calculate distance between previous odometry position and new odometry position
            double distance = sqrt(pow(new_odom_x - prev_odom_x, 2) + pow(new_odom_y - prev_odom_y, 2));

            // If the distance > 1, then reset odom to the previous value
            if (distance > 1.0)
            {
                // Assemble data into pose for Rosservice call 
                //      Use previous positions and new orientation
                rtabmap_ros::ResetPose pose;
                pose.request.x = prev_odom_x;
                pose.request.y = prev_odom_y;
                pose.request.z = prev_odom_z;
                pose.request.roll = prev_r;
                pose.request.pitch = prev_p;
                pose.request.yaw = prev_y;

                // call the 'reset pose' rosservice to send updated data
                if(reset_odom_to_pose_client.call(pose)){
                    ROS_WARN_STREAM("[STATE_MACHINES | odom_offset_checker.cpp | " + robot_name + "]: " + "Odometry has been reset");
                }
                else{
                    ROS_ERROR_STREAM("[STATE_MACHINES | odom_offset_checker.cpp | " + robot_name + "]: " + "Odometry reset has failed.");
                }
            }
            else
            {
                // Since odom is ok, update the "previous" position values
                prev_odom_x = new_odom_x;
                prev_odom_y = new_odom_y;
                prev_odom_z = new_odom_z;
                prev_r = new_r;
                prev_p = new_p;
                prev_y = new_y;

                // ROS_INFO_STREAM("[STATE_MACHINES | odom_offset_checker.cpp | " + robot_name + "]: " + "Odometry is ok");
            }

            // Set this false so we can check to see when we get the next odom message
            odom_message_received = false;

        }

        ros::spinOnce();
        update_rate.sleep();
    }

    ros::spin();
    return 0;
}