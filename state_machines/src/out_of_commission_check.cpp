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

bool scout_1_odom_msg_received = false;
bool scout_2_odom_msg_received = false;
bool hauler_1_odom_msg_received = false;
bool hauler_2_odom_msg_received = false;
bool excavator_1_odom_msg_received = false;
bool excavator_2_odom_msg_received = false;
bool state_msg_received = false;
bool wait_till_reset = false;

double scout_1_x;
double scout_1_y;
double scout_1_z;
double scout_2_x;
double scout_2_y;
double scout_2_z;
double hauler_1_x;
double hauler_1_y;
double hauler_1_z;
double hauler_2_x;
double hauler_2_y;
double hauler_2_z;
double excavator_1_x;
double excavator_1_y;
double excavator_1_z;
double excavator_2_x;
double excavator_2_y;
double excavator_2_z;

bool scout_1_should_be_moving = false;
bool scout_2_should_be_moving = false;
bool hauler_1_should_be_moving = false;
bool hauler_2_should_be_moving = false;
bool excavator_1_should_be_moving = false;
bool excavator_2_should_be_moving = false;

int scout_1_count = 0;
int scout_2_count = 0;
int hauler_1_count = 0;
int hauler_2_count = 0;
int excavator_1_count = 0;
int excavator_2_count = 0;

ros::Time scout_1_start_time;
ros::Time scout_2_start_time;
ros::Time hauler_1_start_time;
ros::Time hauler_2_start_time;
ros::Time excavator_1_start_time;
ros::Time excavator_2_start_time;

std::string curr_bot;
int curr_state;
bool curr_state_done;
bool last_state_succeeded;

void scout_1_odom_callback(nav_msgs::Odometry odom_data) 
{   
    scout_1_x = odom_data.pose.pose.position.x;
    scout_1_y = odom_data.pose.pose.position.y;
    scout_1_z = odom_data.pose.pose.position.z;
    scout_1_odom_msg_received = true;
}

void scout_2_odom_callback(nav_msgs::Odometry odom_data) 
{   
    scout_2_x = odom_data.pose.pose.position.x;
    scout_2_y = odom_data.pose.pose.position.y;
    scout_2_z = odom_data.pose.pose.position.z;
    scout_2_odom_msg_received = true;
}

void hauler_1_odom_callback(nav_msgs::Odometry odom_data) 
{   
    hauler_1_x = odom_data.pose.pose.position.x;
    hauler_1_y = odom_data.pose.pose.position.y;
    hauler_1_z = odom_data.pose.pose.position.z;
    hauler_1_odom_msg_received = true;
}

void hauler_2_odom_callback(nav_msgs::Odometry odom_data) 
{   
    hauler_2_x = odom_data.pose.pose.position.x;
    hauler_2_y = odom_data.pose.pose.position.y;
    hauler_2_z = odom_data.pose.pose.position.z;
    hauler_2_odom_msg_received = true;
}

void excavator_1_odom_callback(nav_msgs::Odometry odom_data) 
{   
    excavator_1_x = odom_data.pose.pose.position.x;
    excavator_1_y = odom_data.pose.pose.position.y;
    excavator_1_z = odom_data.pose.pose.position.z;
    excavator_1_odom_msg_received = true;
}

void excavator_2_odom_callback(nav_msgs::Odometry odom_data) 
{   
    excavator_2_x = odom_data.pose.pose.position.x;
    excavator_2_y = odom_data.pose.pose.position.y;
    excavator_2_z = odom_data.pose.pose.position.z;
    excavator_2_odom_msg_received = true;
}

void robot_state_callback(state_machines::robot_state_status robot_state_info) 
{   
    curr_bot = robot_state_info.robot_name;
    curr_state = robot_state_info.robot_current_state;

    bool was_moving;

    if (curr_bot == "small_scout_1")
    {
        was_moving = scout_1_should_be_moving;
        scout_1_should_be_moving = (curr_state == COMMON_NAMES::STATE_MACHINE_TASK::SCOUT_SEARCH_VOLATILE || 
                                    curr_state == COMMON_NAMES::STATE_MACHINE_TASK::SCOUT_GO_TO_LOC || 
                                    curr_state == COMMON_NAMES::STATE_MACHINE_TASK::SCOUT_UNDOCK || 
                                    curr_state == COMMON_NAMES::STATE_MACHINE_TASK::SCOUT_RESET_ODOM || 
                                    curr_state == COMMON_NAMES::STATE_MACHINE_TASK::SCOUT_GOTO_REPAIR_STATION);
        if (!was_moving && scout_1_should_be_moving)
        {
            scout_1_start_time = ros::Time::now();
        }
    }
    else if (curr_bot == "small_scout_2")
    {
        was_moving = scout_2_should_be_moving;
        scout_2_should_be_moving = scout_1_should_be_moving = (curr_state == COMMON_NAMES::STATE_MACHINE_TASK::SCOUT_SEARCH_VOLATILE || 
                                    curr_state == COMMON_NAMES::STATE_MACHINE_TASK::SCOUT_GO_TO_LOC || 
                                    curr_state == COMMON_NAMES::STATE_MACHINE_TASK::SCOUT_UNDOCK || 
                                    curr_state == COMMON_NAMES::STATE_MACHINE_TASK::SCOUT_RESET_ODOM || 
                                    curr_state == COMMON_NAMES::STATE_MACHINE_TASK::SCOUT_GOTO_REPAIR_STATION);
        if (!was_moving && scout_2_should_be_moving)
        {
            scout_2_start_time = ros::Time::now();
        }
    } 
    else if (curr_bot == "small_hauler_1")
    {
        was_moving = hauler_1_should_be_moving;
        hauler_1_should_be_moving = (curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_GO_TO_LOC || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_DUMP_VOLATILE_TO_PROC_PLANT || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_GO_BACK_TO_EXCAVATOR || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_FOLLOW_EXCAVATOR || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_UNDOCK_EXCAVATOR || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_GOTO_REPAIR_STATION || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HUALER_GO_TO_INIT_LOC || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_GO_TO_EXCAVATOR_RECOVERY || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_GO_TO_LOOKOUT_LOCATION);
        if (!was_moving && hauler_1_should_be_moving)
        {
            hauler_1_start_time = ros::Time::now();
        }
    } 
    else if (curr_bot == "small_hauler_2")
    {
        was_moving = hauler_2_should_be_moving;
        hauler_2_should_be_moving = (curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_GO_TO_LOC || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_DUMP_VOLATILE_TO_PROC_PLANT || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_GO_BACK_TO_EXCAVATOR || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_FOLLOW_EXCAVATOR || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_UNDOCK_EXCAVATOR || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_GOTO_REPAIR_STATION || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HUALER_GO_TO_INIT_LOC || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_GO_TO_EXCAVATOR_RECOVERY || 
                                     curr_state == COMMON_NAMES::STATE_MACHINE_TASK::HAULER_GO_TO_LOOKOUT_LOCATION);
        if (!was_moving && hauler_2_should_be_moving)
        {
            hauler_2_start_time = ros::Time::now();
        }
    } 
    else if (curr_bot == "small_excavator_1")
    {
        was_moving = excavator_1_should_be_moving;
        excavator_1_should_be_moving = (curr_state == COMMON_NAMES::STATE_MACHINE_TASK::EXCAVATOR_GO_TO_LOC || 
                                        curr_state == COMMON_NAMES::STATE_MACHINE_TASK::EXCAVATOR_GO_TO_SCOUT || 
                                        curr_state == COMMON_NAMES::STATE_MACHINE_TASK::EXCAVATOR_GO_TO_REPAIR || 
                                        curr_state == COMMON_NAMES::STATE_MACHINE_TASK::EXCAVATOR_RESET_ODOM_AT_HOPPER || 
                                        curr_state == COMMON_NAMES::STATE_MACHINE_TASK::EXCAVATOR_GO_TO_SCOUT_RECOVERY || 
                                        curr_state == COMMON_NAMES::STATE_MACHINE_TASK::EXCAVATOR_GO_TO_INIT_LOCATION || 
                                        curr_state == COMMON_NAMES::STATE_MACHINE_TASK::EXCAVATOR_GO_TO_LOOKOUT_LOCATION);
        if (!was_moving && excavator_1_should_be_moving)
        {
            excavator_1_start_time = ros::Time::now();
        }
    } 
    else if (curr_bot == "small_excavator_2")
    {
        was_moving = excavator_2_should_be_moving;
        excavator_2_should_be_moving = (curr_state == COMMON_NAMES::STATE_MACHINE_TASK::EXCAVATOR_GO_TO_LOC || 
                                        curr_state == COMMON_NAMES::STATE_MACHINE_TASK::EXCAVATOR_GO_TO_SCOUT || 
                                        curr_state == COMMON_NAMES::STATE_MACHINE_TASK::EXCAVATOR_GO_TO_REPAIR || 
                                        curr_state == COMMON_NAMES::STATE_MACHINE_TASK::EXCAVATOR_RESET_ODOM_AT_HOPPER || 
                                        curr_state == COMMON_NAMES::STATE_MACHINE_TASK::EXCAVATOR_GO_TO_SCOUT_RECOVERY || 
                                        curr_state == COMMON_NAMES::STATE_MACHINE_TASK::EXCAVATOR_GO_TO_INIT_LOCATION || 
                                        curr_state == COMMON_NAMES::STATE_MACHINE_TASK::EXCAVATOR_GO_TO_LOOKOUT_LOCATION);
        if (!was_moving && excavator_2_should_be_moving)
        {
            excavator_2_start_time = ros::Time::now();
        }
    }

    state_msg_received = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "out_of_commission_checker");
    ros::NodeHandle nh;
    
    ros::Subscriber camera_odom_sub_scout_1 = nh.subscribe("/small_scout_1" + COMMON_NAMES::RTAB_ODOM_TOPIC, 223, scout_1_odom_callback);
    ros::Subscriber camera_odom_sub_scout_2 = nh.subscribe("/small_scout_2" + COMMON_NAMES::RTAB_ODOM_TOPIC, 223, scout_2_odom_callback);
    ros::Subscriber camera_odom_sub_hauler_1 = nh.subscribe("/small_hauler_1" + COMMON_NAMES::RTAB_ODOM_TOPIC, 223, hauler_1_odom_callback);
    ros::Subscriber camera_odom_sub_hauler_2 = nh.subscribe("/small_hauler_2" + COMMON_NAMES::RTAB_ODOM_TOPIC, 223, hauler_2_odom_callback);
    ros::Subscriber camera_odom_sub_excavator_1 = nh.subscribe("/small_excavator_1" + COMMON_NAMES::RTAB_ODOM_TOPIC, 223, excavator_1_odom_callback);
    ros::Subscriber camera_odom_sub_excavator_2 = nh.subscribe("/small_excavator_2"+ COMMON_NAMES::RTAB_ODOM_TOPIC, 223, excavator_2_odom_callback);

    ros::Subscriber robot_state_sub = nh.subscribe(COMMON_NAMES::CAPRICORN_TOPIC + COMMON_NAMES::ROBOTS_CURRENT_STATE_TOPIC, 223, robot_state_callback);

    ros::Publisher out_of_commission_pub = nh.advertise<std_msgs::String>("/capricorn/" + COMMON_NAMES::ROBOTS_OUT_OF_COMMISSION_TOPIC, 223);

    // Wait until we have received a message from all odoms
    bool all_odoms_received = scout_1_odom_msg_received/* && scout_2_odom_msg_received && hauler_1_odom_msg_received && hauler_2_odom_msg_received && excavator_1_odom_msg_received && excavator_2_odom_msg_received*/;
    
    while(ros::ok() && !(state_msg_received && all_odoms_received))
    {
        all_odoms_received = scout_1_odom_msg_received/* && scout_2_odom_msg_received && hauler_1_odom_msg_received && hauler_2_odom_msg_received && excavator_1_odom_msg_received && excavator_2_odom_msg_received*/;
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    ROS_WARN_STREAM("MADE IT HERE");

    ros::Time curr_time = ros::Time::now();

    double scout_1_start_x = scout_1_x;
    double scout_1_start_y = scout_1_y;
    double scout_1_start_z = scout_1_z;
    scout_1_start_time = curr_time;

    double scout_2_start_x = scout_2_x;
    double scout_2_start_y = scout_2_y;
    double scout_2_start_z = scout_2_z;
    scout_2_start_time = curr_time;

    double hauler_1_start_x = hauler_1_x;
    double hauler_1_start_y = hauler_1_y;
    double hauler_1_start_z = hauler_1_z;
    hauler_1_start_time = curr_time;

    double hauler_2_start_x = hauler_2_x;
    double hauler_2_start_y = hauler_2_y;
    double hauler_2_start_z = hauler_2_z;
    hauler_2_start_time = curr_time;

    double excavator_1_start_x = excavator_1_x;
    double excavator_1_start_y = excavator_1_y;
    double excavator_1_start_z = excavator_1_z;
    excavator_1_start_time = curr_time;

    double excavator_2_start_x = excavator_2_x;
    double excavator_2_start_y = excavator_2_y;
    double excavator_2_start_z = excavator_2_z;
    excavator_2_start_time = curr_time;

    ROS_WARN_STREAM("[STATE_MACHINES | out_of_commission_check.cpp | Checking if Robot is Out of Commission");
    
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    
    // Inf loop
    while(ros::ok())
    {

        ros::Time curr_time = ros::Time::now();

        // If the robot has moved at least 1 meter, then it is still good
        // Else, if it should be moving, that is a problem. Increment the counter to move the robot closer to "out of commission" status
        if (sqrt(pow(scout_1_x - scout_1_start_x, 2) + pow(scout_1_y - scout_1_start_y, 2) + pow(scout_1_z - scout_1_start_z, 2)) > 1)
        {
            
            ROS_WARN_STREAM("SCOUT 1 DISTANCE OK");
            
            scout_1_start_x = scout_1_x;
            scout_1_start_y = scout_1_y;
            scout_1_start_z = scout_1_z;

            scout_1_start_time = curr_time;
            scout_1_count = 0;
        }
        else if (scout_1_should_be_moving)
        {
            scout_1_count++;
            ROS_WARN_STREAM("small scout 1 bad");
        }

        if (sqrt(pow(scout_2_x - scout_2_start_x, 2) + pow(scout_2_y - scout_2_start_y, 2) + pow(scout_2_z - scout_2_start_z, 2)) > 1)
        {
            
            ROS_WARN_STREAM("SCOUT 2 DISTANCE OK");
            
            scout_2_start_x = scout_2_x;
            scout_2_start_y = scout_2_y;
            scout_2_start_z = scout_2_z;

            scout_2_start_time = curr_time;
            scout_2_count = 0;
        }
        else if (scout_2_should_be_moving)
        {
            scout_2_count++;
            ROS_WARN_STREAM("small scout 2 bad");
        }

        if (sqrt(pow(hauler_1_x - hauler_1_start_x, 2) + pow(hauler_1_y - hauler_1_start_y, 2) + pow(hauler_1_z - hauler_1_start_z, 2)) > 1)
        {
            
            ROS_WARN_STREAM("HAULER 1 DISTANCE OK");
            
            hauler_1_start_x = hauler_1_x;
            hauler_1_start_y = hauler_1_y;
            hauler_1_start_z = hauler_1_z;

            hauler_1_start_time = curr_time;
            hauler_1_count = 0;
        }
        else if (hauler_1_should_be_moving)
        {
            hauler_1_count++;
            ROS_WARN_STREAM("small hauler 1 bad");
        }

        if (sqrt(pow(hauler_2_x - hauler_2_start_x, 2) + pow(hauler_2_y - hauler_2_start_y, 2) + pow(hauler_2_z - hauler_2_start_z, 2)) > 1)
        {
            
            ROS_WARN_STREAM("HAULER 2 DISTANCE OK");
            
            hauler_2_start_x = hauler_2_x;
            hauler_2_start_y = hauler_2_y;
            hauler_2_start_z = hauler_2_z;

            hauler_2_start_time = curr_time;
            hauler_2_count = 0;
        }
        else if (hauler_2_should_be_moving)
        {
            hauler_2_count++;
            ROS_WARN_STREAM("small hauler 2 bad");
        }

        if (sqrt(pow(excavator_1_x - excavator_1_start_x, 2) + pow(excavator_1_y - excavator_1_start_y, 2) + pow(excavator_1_z - excavator_1_start_z, 2)) > 1)
        {
            
            ROS_WARN_STREAM("EXCAVATOR 1 DISTANCE OK");
            
            excavator_1_start_x = excavator_1_x;
            excavator_1_start_y = excavator_1_y;
            excavator_1_start_z = excavator_1_z;

            excavator_1_start_time = curr_time;
            excavator_1_count = 0;
        }
        else if (excavator_1_should_be_moving)
        {
            excavator_1_count++;
            ROS_WARN_STREAM("small excavator 1 bad");
        }

        if (sqrt(pow(excavator_2_x - excavator_2_start_x, 2) + pow(excavator_2_y - excavator_2_start_y, 2) + pow(excavator_2_z - excavator_2_start_z, 2)) > 1)
        {
            
            ROS_WARN_STREAM("EXCAVATOR 2 DISTANCE OK");
            
            excavator_2_start_x = excavator_2_x;
            excavator_2_start_y = excavator_2_y;
            excavator_2_start_z = excavator_2_z;

            excavator_2_start_time = curr_time;
            excavator_2_count = 0;
        }
        else if (excavator_2_should_be_moving)
        {
            excavator_2_count++;
            ROS_WARN_STREAM("small excavator 2 bad");
        }

        // If the robot has been stopped for at least 2 minutes, it is out of commission
        if (scout_1_count > 5)
        {
            std_msgs::String pub_data;
            pub_data.data = "small_scout_1";
            out_of_commission_pub.publish(pub_data);
        }
        if (scout_2_count > 5)
        {
            std_msgs::String pub_data;
            pub_data.data = "small_scout_2";
            out_of_commission_pub.publish(pub_data);
        }
        if (hauler_1_count > 5)
        {
            std_msgs::String pub_data;
            pub_data.data = "small_hauler_1";
            out_of_commission_pub.publish(pub_data);
        }
        if (hauler_2_count > 5)
        {
            std_msgs::String pub_data;
            pub_data.data = "small_hauler_2";
            out_of_commission_pub.publish(pub_data);
        }
        if (excavator_1_count > 5)
        {
            std_msgs::String pub_data;
            pub_data.data = "small_excavator_1";
            out_of_commission_pub.publish(pub_data);
        }
        if (excavator_2_count > 5)
        {
            std_msgs::String pub_data;
            pub_data.data = "small_excavator_2";
            out_of_commission_pub.publish(pub_data);
        }

        ros::Duration(20).sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}