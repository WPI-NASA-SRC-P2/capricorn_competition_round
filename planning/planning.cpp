#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "math.h"
#include "string.h"

float heurstics_dist(float x,float y, float goal_x, float goal_y,  float D){
    // Euclidean Distance Estimation Cost Function
    float dx = abs(x - goal_x);
    float dy = abs(y - goal_y); 
    // D is the passability of the node

    return D*sqrt(dx*dx + dy*dy)
}

float current_location(){

    return x, y
}

float goal_location(){

    return goal_x, goal_y
}

void pathingCallback(const nav_msgs::OccupancyGrid oGrid) {

}

int main(int argc, char** argv) {
    if (argc != 4) {
        ROS_ERROR_STREAM("This Node must be launched via 'roslaunch' and needs an argument as <RobotName_Number>";);
        return -1;
    }

    std::string robot_name(argv[1]);
    std::string node_name = robot_name + "_planning_pathing_client";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Subscribe to small scout for testing
    ros::Subscriber sub = nh.subscribe("/small_scout_1/camera/grid_map", 1000, pathingCallback);

    ros::spin();
    return 0;
}