#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <ros/ros.h>
#include <iostream>

class PathPlanner
{
private:

public:
  PathPlanner();
  ~PathPlanner();
  
  std::vector<geometry_msgs::Point> PathPlanner::indexToGrid(const int8[] data, uint32 width, uint32 height);

  /**
 * @brief converts grid cell into list of walkable neightbors (full adjacent neightbors only)
 * 
 */
void PathPlanner::neightborsOf4(nav_msgs::GridCell> coordinate, std::vector<nav_msgs::GridCell> listOfCoordinates);

/**
 * @brief converts grid cell into list of walkable neightbors (adjacent and diagonal)
 * 
 */
void PathPlanner::neightborsOf8(nav_msgs::GridCell coordinate, std::vector<nav_msgs::GridCell> listOfCoordinates);

/**
 * @brief derives a path from a list of coordinates 
 * 
 */
void wavefront(void);

};

#endif // TEMPLATE_CLASS_H