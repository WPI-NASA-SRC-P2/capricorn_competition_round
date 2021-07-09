#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

class DynamicPlanning
{
private:
    
public:

    static float lengthOfPath(nav_msgs::Path path);
    static double distance(geometry_msgs::Point firstpoint, geometry_msgs::Point secondpoint);
    static std::pair <geometry_msgs::Point, geometry_msgs::Point> waypointOnPath(geometry_msgs::Point upcoming_point, geometry_msgs::PoseStamped robot_pose, nav_msgs::Path &path, float distance);


    static bool checkForObstacles(nav_msgs::Path& path, nav_msgs::OccupancyGrid& oGrid, geometry_msgs::PoseStamped robot_pose);// path is giving the grid coordinates

};
