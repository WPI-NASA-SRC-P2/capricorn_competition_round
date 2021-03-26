#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>

class PathPlanner
{
private:

public:
  
static void test(const nav_msgs::OccupancyGrid& gridValues);

static geometry_msgs::Point indexToGrid(nav_msgs::OccupancyGrid gridValues, int index);

static int gridToIndex(nav_msgs::OccupancyGrid gridValues, geometry_msgs::Point coordinate);

static void PrintMessage(nav_msgs::OccupancyGrid OccGrid);

  /**
 * @brief converts grid cell into list of walkable neightbors (full adjacent neightbors only)
 * 
 */
static std::vector<geometry_msgs::Point> neighborsOf4(geometry_msgs::Point coordinate, nav_msgs::OccupancyGrid gridValues);

/**
 * @brief converts grid cell into list of walkable neightbors (adjacent and diagonal)
 * 
 */
static std::vector<geometry_msgs::Point> neighborsOf8(geometry_msgs::Point coordinate, nav_msgs::OccupancyGrid gridValues);

/**
 * @brief derives a path from a list of coordinates 
 * 
 */
static void wavefront(void);

};

#endif // TEMPLATE_CLASS_H