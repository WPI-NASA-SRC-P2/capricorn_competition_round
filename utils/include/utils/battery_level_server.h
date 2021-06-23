#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "utils/battery_deadlines.h"

class  BatteryLevelServer{
private:
    float distance_;
    float soft_deadline_;
    float hard_deadline_;
    geometry_msgs::PoseStamped target_location_;
    geometry_msgs::PoseStamped current_location_;
    


public:
    ros::Subscriber pose_subscriber;

    /**
     * @brief Calculates the deadlines and assigns it as a response for a ROS Service call
     * 
     * @param req Client's request to the server - in this case a current location of the robot in the simulation
     * @param res Server's response to the client's request - in this case battery level deadlines
     */
    void deadlinesCallback(utils::battery_deadlines::Request &req, utils::battery_deadlines::Response &res);
};
