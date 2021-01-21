#include<iostream>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

std::string robot_name;

// Initializing noisy image check parameters
float right_last_sum = 0;
float left_last_sum = 0 ;
float right_actual_sum = 0; 
float left_actual_sum = 0;
float threshold = 4000;
float upper_threshold = 1.3;
float lower_threshold = 0.7;

// Initializing global publishers
ros::Publisher right_image_pub, left_image_pub, right_info_pub, left_info_pub;

/**
 * @brief Checks whether an image is noise free or not
 * 
 * @param img ROS Image Message : Image to be check, should not be null
 * @param left Bool : Whether the image is a left image of a stereo pair
 * @return true if image does not have noise
 * @return false if image has noise
 */
bool check_image(const sensor_msgs::ImageConstPtr& img, bool left) {
    
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv::Mat edge;
    cv::Canny(cv_ptr->image, edge, 250, 255);
    
    int s = cv::sum(edge)[0];

    if(left) {
        int diff = s - left_last_sum;
        if(s >= left_actual_sum * lower_threshold && s <= left_actual_sum * upper_threshold) {
            left_last_sum = s;
        }
        
        left_actual_sum = s;
        
        if(diff < threshold) {
            left_last_sum = s;
            return true;
        }
        else {
            return false;
        }
    }

    else {
        int diff = s - right_last_sum; 
        if(s >= right_actual_sum * lower_threshold && s <= right_actual_sum * upper_threshold) {
            right_last_sum = s;
        }

        right_actual_sum = s;

        if(diff < threshold) {
            right_last_sum = s;
            return true;
        }
        else {
            return false;
        }
    }
}

/**
 * @brief : Callback function for getting messages from stereo camera, publishes images and camera info
 * on new capricorn topics if the images does not have noise
 * 
 * @param right : Right Image (ROS Image msg)
 * @param left  : Left Image (ROS Image msg)
 * @param right_info : Right Camera Info (ROS CameraInfo msg)
 * @param left_info  : Left Camera Info (ROS CameraInfo msg)
 */
void img_callback(const sensor_msgs::ImageConstPtr& right, const sensor_msgs::ImageConstPtr& left, const sensor_msgs::CameraInfoConstPtr& right_info, const sensor_msgs::CameraInfoConstPtr& left_info) {
    if(check_image(right, false)) {
        right_image_pub.publish(right);
        right_info_pub.publish(right_info);
    }
    
    if(check_image(left, true)) {
        left_image_pub.publish(left);
        left_info_pub.publish(left_info);
    }
}

int main(int argc, char *argv[]) {
    std::string name(argv[1]);
    robot_name = name;

    ros::init(argc, argv, "noisy_image_eliminate");
    ros::NodeHandle nh("~");
    
    message_filters::Subscriber<sensor_msgs::Image> img_sub_r(nh, '/'+robot_name+"/camera/right/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> img_sub_l(nh, '/'+robot_name+"/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_r(nh, '/'+robot_name+"/camera/right/camera_info", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_l(nh, '/'+robot_name+"/camera/left/camera_info", 1);
    
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> sync(img_sub_r, img_sub_l, info_sub_r, info_sub_l, 10);
    sync.registerCallback(boost::bind(&img_callback, _1, _2, _3, _4));
    
    right_image_pub = nh.advertise<sensor_msgs::Image>("/capricorn/" + robot_name + "/camera/right/image_raw", 10);
    left_image_pub = nh.advertise<sensor_msgs::Image>("/capricorn/" + robot_name + "/camera/left/image_raw", 10);
    right_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/capricorn/" + robot_name + "/camera/right/camera_info", 10);
    left_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/capricorn/" + robot_name + "/camera/left/camera_info", 10);
    
    ros::spin();
}