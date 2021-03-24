#pragma once

#include <vector>
#include <geometry_msgs/Point.h>

class AStar
{
public:
    /**
    * @brief Calculates the best distance to target using the A Star Algorithm.
    * 
    * @param frontiers The mapped frontiers of the occupancy grid.
    * @param target The destination point for the algorithm.
    * @param start The starting location for the algorithm. (Current location of robot, usually.)
    * 
    * @returns A list of waypoints where each waypoint is guarenteed not to be collinear with its neighbors.
    */
    
    static std::vector<geometry_msgs::Point> FindPath(std::vector<geometry_msgs::Point> &frontiers, geometry_msgs::Point target, geometry_msgs::Point start);
};