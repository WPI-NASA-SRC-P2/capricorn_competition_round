#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class  BatteryLevelServer{
private:
    float distance_;
    float soft_deadline_;
    float hard_deadline_;

public:
    ros::Subscriber pose_subscriber;


    void poseCallback(nav_msgs::Odometry odom);
    void deadlinesCallback();



};
