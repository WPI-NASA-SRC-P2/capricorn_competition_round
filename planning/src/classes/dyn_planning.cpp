#include "dyn_planning.h"    
#include "astar.h"

    
bool DynamicPlanning::checkForObstacles(nav_msgs::Path& path, nav_msgs::OccupancyGrid& oGrid)
{
 
    for (int i = 0; i < path.poses.size(); i++)
    {
        if(oGrid.data[AStar::indexFromPoseStamped(path.poses[i].pose.position, oGrid)] > 50)
        {
            ROS_INFO("Obstacle on the current path");
            return true;
        }
    }

    return false;
}
