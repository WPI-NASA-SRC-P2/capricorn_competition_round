#include "dyn_planning.h"    
#include "astar.h"
using namespace std;


float lengthOfPath(int path[])
{
    start_point = path[0];
    end_point = path[path.size() - 1];
    return std::sqrt(pow(start_point[0] - end_point[0], 2) + pow(start_point[1] - end_point[1], 2));
}

bool DynamicPlanning::checkForObstacles(nav_msgs::Path& path, nav_msgs::OccupancyGrid& oGrid) // path is giving the grid coordinates
{
 /* give out the number of points till 6 m in the path 
    to be checked for their threshold values, 
    not just the wavepoints that the path is returning 
 */

    std::vector<int>Path_index;
    if(lengthOfPath(nav::msgs Path& path) > 6)    // it is needed because we cant decide from the number of waypoints.   
        for (int i = 0; i < path.poses.size(); i++)
        {   
            Path_index.push_back(oGrid.data[AStar::indexFromPoseStamped(path.poses[i].pose.position, oGrid)]);
            std::reverse_iterator(Path_index.rbegin(), Path_index.rend()); // saved all the index points from start point to end point.
            
        


            if(oGrid.data[AStar::indexFromPoseStamped(path.poses[i].pose.position, oGrid)] > 50) // first converting the grid coordinate to the index  
            {                                                                                  // and then checking the threshold value of that index 
                ROS_INFO("Obstacle on the current path");
                return true;
            }
        }

        return false;
    }
}
