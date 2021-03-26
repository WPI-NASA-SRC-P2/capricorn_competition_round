#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>


#include <astar.h>
#include <cspace.h>

//How fast the main loop of the node will run.
#define UPDATE_HZ 10

ros::Publisher publisher;

using geometry_msgs::Point;

void callback(const nav_msgs::OccupancyGrid oGrid) {
    Point origin;
    origin.x = 180;
    origin.y = 0;

    Point target;
    target.x = 180;
    target.y = 300;

    auto CSpace = CSpace::GetCSpace(oGrid, 50, 1);
    auto path = AStar::FindPathOccGrid(CSpace, target, origin);

    publisher.publish(path);
}

int main(int argc, char *argv[])
{
    // ROS initialization
    ros::init(argc, argv, "path_planner");

    ros::NodeHandle nh;

    publisher = nh.advertise<nav_msgs::Path>("/capricorn/navigation_path", 1000);

    //Subscribe to the node publishing map values 
    ros::Subscriber indexValues = nh.subscribe("/capricorn/Ground_Truth_Map", 1000, callback);
    
    ros::spin();

    return 0;
}