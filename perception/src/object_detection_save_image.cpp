/*
Author: Mahimana Bhatt
Email: mbhatt@wpi.edu

TEAM CAPRICORN
NASA SPACE ROBOTICS CHALLENGE
*/

#include<iostream>
#include <limits>
#include <string>

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
int num_img = 0;
std::string dataset_path = "~/catkin_ws/dataset/images/";
cv::Mat cv_image; 

void img_callback(const sensor_msgs::ImageConstPtr& img) 
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv_image = cv_ptr->image;
    cv::imshow("Image Window", cv_image);
    cv::waitKey(1);
}

int main(int argc, char *argv[]) 
{

    std::string name(argv[1]);
    robot_name = name;

    ros::init(argc, argv, robot_name + COMMON_NAMES::NOISY_IMAGE_NODE_NAME); 
    ros::NodeHandle nh("");

    
    ros::Subscriber img_sub = nh.subscribe('/' + robot_name + COMMON_NAMES::LEFT_IMAGE_RAW_TOPIC, 1, img_callback); 
    
    // while(ros::ok()) 
    // {
    //     ros::spinOnce();
    //     // std::cout << "Press ENTER to continue...";
    //     // std::cin.ignore( std::numeric_limits <std::streamsize> ::max(), '\n' );

    //     // cv::imwrite(dataset_path + std::to_string(num_img), cv_image);
    //     // num_img += 1;
    // }

    ros::spin();
}