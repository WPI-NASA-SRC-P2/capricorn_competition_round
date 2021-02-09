#include<iostream>

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

std::string robot_name;
std_msgs::Float64 pitch;
float location_of_horizon = 0.5;

// Initializing global publisher
ros::Publisher test_image_pub, pitch_camera_pub;

/**
 * @brief 
 *   
 *    Up Down topic: /small_scout_1/sensor/pitch/command/position Type: std_msgs/Float64
 *
 *   Left right Topic: /small_scout_1/sensor/yaw/command/position Type: std_msgs/Float64 
 * @param left 
 *
 */
void img_callback(const sensor_msgs::ImageConstPtr& img) 
{
    cv_bridge::CvImagePtr cv_ptr;

    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv::Mat edge, color_zero, gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    cv::imwrite("this.jpg", gray);
    cv::Canny(cv_ptr->image, edge, 150, 200);

    // cv::imshow("Test Image", edge);

    // std::cout<<cv_ptr->image.channels()<<"\n";
    // for(int j = 0; j < gray.rows; j++) {
    //     for(int i= 0; i < 50; i++) {
    //         std::cout<<int(gray.at<uchar>(j, i))<<" ";
    //     }
    //     std::cout<<"\n";
    // }

    cv::Mat zero = cv::Mat::zeros(cv::Size(gray.size[1], gray.size[0]), CV_8UC1);
    // int m = 0;
    // long 
    for(int j = 10; j < gray.cols; j++) {
        for(int i = 0; i < gray.rows; i++) {
            if(gray.at<uchar>(i, j) > 10) {
                zero.at<uchar>(i, j) = 255;
                break;
            }
            // m = std::max(m, i);
        }
    }

    // std_msgs::Float64 pitch;

    // pitch.data = -float(((float(gray.rows) / 3) - m) / 800 );
    // std::cout<<pitch.data<<"\n";

    // pitch_camera_pub.publish(pitch);

    std::vector<cv::Vec4i> lines;
    int thresh = 50;

    int morph_elem = 0;
    int morph_size = 0;
    int morph_operator = 0;
    int const max_operator = 4;
    int const max_elem = 2;
    int const max_kernel_size = 21;

    int operation = morph_operator + 2;
    cv::Mat element = getStructuringElement( morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
    cv::morphologyEx( zero, zero, operation, element );
    cv::dilate(zero, zero, cv::Mat(), cv::Point(-1, -1), 1, 1, 1);
    cv::HoughLinesP(zero, lines, 1, 2*CV_PI / 180, thresh, 300, 500);
    cv::cvtColor(zero, color_zero, cv::COLOR_GRAY2BGR);

    if(lines.size() > 0) {
        cv::Vec4i l = lines[0];
        cv::line(color_zero, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
        int x1 = l[0], y1 = l[1], x2 = l[2], y2 = l[3];
        int mid_point[2] = {int(gray.cols * 0.5), int(gray.rows * location_of_horizon)};
        float dist = float((((y1 - y2) * mid_point[0]) + ((x2 - x1) * mid_point[1]) + ((x1 * y2) - (x2 * y1))) / (sqrt(((y1 - y2) * (y1 - y2)) + ((x2 - x1) * (x2 - x1))) * 800));
        pitch.data = pitch.data - dist;
        std::cout<<pitch.data<<"  "<<dist<<"\n";
        pitch_camera_pub.publish(pitch);
    }

    // for (size_t i=0; i<lines.size(); i++) {
    //     cv::Vec4i l = lines[i];
    //     cv::line(color_zero, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
    // }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_zero).toImageMsg();
    msg->height = color_zero.size[0];
    msg->width = color_zero.size[1];
    test_image_pub.publish(msg);
}

int main(int argc, char *argv[]) 
{
    std::string name(argv[1]);
    robot_name = name;

    ros::init(argc, argv, "horzion_tracking_"+robot_name);
    ros::NodeHandle nh("~");
    
    ros::Subscriber img_sub_l = nh.subscribe("/capricorn/"+robot_name+"/camera/left/image_raw", 1, &img_callback);
    
    test_image_pub = nh.advertise<sensor_msgs::Image>("/capricorn/" + robot_name + "/horizon_test_image", 10);
    pitch_camera_pub = nh.advertise<std_msgs::Float64>("/" + robot_name + "/sensor/pitch/command/position", 10);
    pitch.data = 0;
    pitch_camera_pub.publish(pitch);
    
    ros::spin();
}