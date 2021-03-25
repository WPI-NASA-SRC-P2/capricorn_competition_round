#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <path_planner.h>

//How fast the main loop of the node will run.
#define UPDATE_HZ 10

int main(int argc, char *argv[])
{
    // ROS initialization
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;

    // Creted an object of the class
    PathPlanner planner;

    //Subscribe to the node publishing map values 
    ros::Subscriber indexValues = nh.subscribe("/small_scout1/camera/grid_map", 1000, PrintMessage);

    // std::vector<nav_msgs::GridCell> gridCells = planner.indexToGrid(indexValues);

    // std::vector<nav_msgs::GridCell> neighbors4 = planner.neightborsOf4(gridCells);

    // std::vector<nav_msgs::GridCell> neighbors4 = planner.neightborsOf8(gridCells);
    
    ros::spin();

    return 0;
}