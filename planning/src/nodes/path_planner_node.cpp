#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>


#include <astar.h>

//How fast the main loop of the node will run.
#define UPDATE_HZ 10

ros::Publisher publisher;

void callback(const nav_msgs::OccupancyGrid oGrid) {
    geometry_msgs::Point pt1;
    pt1.x = 0;
    pt1.y = 0;

    geometry_msgs::Point pt2;
    pt2.x = 30;
    pt2.y = 30;

    auto astar = AStar::FindPathOccGrid(oGrid, pt1, pt2);

    astar.header.frame_id = oGrid.header.frame_id;

    publisher.publish(astar);
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