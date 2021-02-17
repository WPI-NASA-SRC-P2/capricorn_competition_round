/*
Author: Mahimana Bhatt
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

#include<iostream>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <utils/common_names.h>



std::string robot_name;

//initializing variables
int num_img = 15000;
int cv_image[512][512][3];

void callback() {
}

int main(int argc, char *argv[]) {

    std::string name(argv[1]);
    robot_name = name;

    ros::init(argc, argv, robot_name + COMMON_NAMES::NOISY_IMAGE_NODE_NAME); 
    ros::NodeHandle nh("");

    message_filters::Subscriber<sensor_msgs::Image> img_sub(nhm, '/' + robot_name + COMMON_NAMES::LEFT_IMAGE_RAW_TOPIC, 1);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), img_sub);
    sync.registerCallback(boost::bind(&callback,_1));

    ros::spin();
}