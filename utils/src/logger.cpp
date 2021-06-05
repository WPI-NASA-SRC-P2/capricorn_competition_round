/*
Author: Mahimana Bhatt
Email: mbhatt@wpi.edu
TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

#include <ros/ros.h>
#include <srcp2_msgs/ScoreMsg.h>
#include <utils/common_names.h>

void score_callback(const srcp2_msgs::ScoreMsg& score) 
{   
    ROS_INFO_STREAM("Hauler Score: "<<score.hauler_volatile_score);
    ROS_INFO_STREAM("Score: "<<score.score);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, COMMON_NAMES::LOGGER_NODE_NAME);
    ros::NodeHandle nh;
    
    ros::Subscriber objects_sub = nh.subscribe(COMMON_NAMES::SRC_SCORE_TOPIC, 1, &score_callback);
    
    ros::spin();
    return 0;
}