#pragma once

#include <vector>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

class AStar
{
public:
    /**
     * @brief Calculated the shortest path using an occupancy grid based approach.
     * 
     * @param oGrid The occupancy grid, ranging 0 (unoccupied) to 100 (completely blocked)
     * @param target The target Point for the algorithm.
     * @param start The start point of the algorithm (usually the robot position)
     *
     * @return A ROS Path message containing the points in the shortest path.
    **/

    static nav_msgs::Path FindPathOccGrid(nav_msgs::OccupancyGrid oGrid, geometry_msgs::Point target, geometry_msgs::Point start);
};