#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

class DynamicPlanning
{
private:
    
public:
    static bool checkForObstacles(nav_msgs::Path& path, nav_msgs::OccupancyGrid& oGrid);
};
