#pragma once

#include <vector>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

class AStar
{
public:
    /**
    * @brief Calculates the best distance to target using the A Star Algorithm.
    * 
    * @param frontiers The mapped frontiers of the occupancy grid.
    * @param obstacles The mapped obstacles of the occupancy grid.
    * @param target The destination point for the algorithm.
    * @param start The starting location for the algorithm. (Current location of robot, usually.)
    * 
    * @returns A list of waypoints where each waypoint is guarenteed not to be collinear with its neighbors.
    */
    
    static nav_msgs::Path FindPathFrontier(std::vector<geometry_msgs::Point> &frontiers, std::vector<geometry_msgs::Point> &obstacles, geometry_msgs::Point target, geometry_msgs::Point start, bool DirectToDest);
    
    static nav_msgs::Path FindPathOccGrid(nav_msgs::OccupancyGrid oGrid, geometry_msgs::Point target, geometry_msgs::Point start);
};