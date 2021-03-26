#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include "path_planner.h"

/**
 * @brief prints data from the occupancy grid 
 */
void PathPlanner::PrintMessage(nav_msgs::OccupancyGrid OccGrid)
{
   // printf(OccGrid.data);
}

/**
 * @brief test function
 */
void PathPlanner::test(const nav_msgs::OccupancyGrid& gridValues)
{
    //geometry_msgs::Point coordinate = indexToGrid(gridValues, 408);
    //int index = gridToIndex(gridValues, coordinate);

    neighborsOf8(indexToGrid(gridValues, 159999), gridValues);

    //ROS_INFO("%i", index);
}

/**
 * @brief converts index of a grid into x and y coordinates
 * 
 */
geometry_msgs::Point PathPlanner::indexToGrid(nav_msgs::OccupancyGrid gridValues, int index)
{
    //declare a return array
    geometry_msgs::Point retCoordinate; 

    //get the width and height of the map from the OccupancyGrid message 
    int map_width = gridValues.info.width;
    int map_height = gridValues.info.height;

    //iterate thru the array and convert each occupancy grid index to a grid cell message
    retCoordinate.x = (index % map_width) * gridValues.info.resolution;
    retCoordinate.y = (float)(index / map_width) * gridValues.info.resolution;

    //return the array of grid cells
    ROS_INFO("x: %f", retCoordinate.x);
    ROS_INFO("y: %f", retCoordinate.y);
    return retCoordinate;
}

/**
 * @brief converts x and y coordinates of a grid cell to its index
 * 
 */
int PathPlanner::gridToIndex(nav_msgs::OccupancyGrid gridValues, geometry_msgs::Point coordinate)
{
    int retInt = coordinate.x / gridValues.info.resolution + (coordinate.y / gridValues.info.resolution) * gridValues.info.width; 
    
    ROS_INFO("index: %d", retInt);

    return retInt;
}

/**
 * @brief converts grid cell into list of walkable neightbors (full adjacent neightbors only)
 * 
 */
std::vector<geometry_msgs::Point> PathPlanner::neighborsOf4(geometry_msgs::Point coordinate, nav_msgs::OccupancyGrid gridValues) 
{
    //declare a return array 
    std::vector<geometry_msgs::Point> retPoints;

    //find the index from the given coordinate and grid
    int index = gridToIndex(gridValues, coordinate);

    int map_width = gridValues.info.width;
    int map_height = gridValues.info.height;
    int map_area = gridValues.info.width * gridValues.info.height;

    //left
    if(index % map_width != 0) retPoints.push_back(indexToGrid(gridValues, (index - 1)));

    //right
    if(index % map_width != (map_width - 1)) retPoints.push_back(indexToGrid(gridValues, (index + 1)));

    //bottom
    if(index >= map_width) retPoints.push_back(indexToGrid(gridValues, (index - map_width)));

    //top
    if(index < (map_area - map_width)) retPoints.push_back(indexToGrid(gridValues, (index + map_width)));

    //return the return array
    return retPoints;
}

/**
 * @brief converts grid cell into list of walkable neightbors (adjacent and diagonal)
 * 
 */
std::vector<geometry_msgs::Point> PathPlanner::neighborsOf8(geometry_msgs::Point coordinate, nav_msgs::OccupancyGrid gridValues)
{
    //declare a return array 
    std::vector<geometry_msgs::Point> retPoints;

    //find the index from the given coordinate and grid
    int index = gridToIndex(gridValues, coordinate);

    int map_width = gridValues.info.width;
    int map_height = gridValues.info.height;
    int map_area = gridValues.info.width * gridValues.info.height;
    
    //left
    if(index % map_width != 0) retPoints.push_back(indexToGrid(gridValues, (index - 1)));

    //right
    if(index % map_width != (map_width - 1)) retPoints.push_back(indexToGrid(gridValues, (index + 1)));

    //bottom
    if(index >= map_width) retPoints.push_back(indexToGrid(gridValues, (index - map_width)));

    //top
    if(index < (map_area - map_width)) retPoints.push_back(indexToGrid(gridValues, (index + map_width)));

    //top left
    if(index > map_width + 1 && index % map_width != 0) retPoints.push_back(indexToGrid(gridValues, (index - (map_width + 1))));

    //top right
    if(index > map_width && index % map_width != (map_width -1)) retPoints.push_back(indexToGrid(gridValues, (index - (map_width - 1))));

    //bottom left
    if(index < (map_area - map_width) && index % map_width != 0) retPoints.push_back(indexToGrid(gridValues, (index + (map_width -1))));

    //bottom right 
    if(index < ((map_area) - (map_width + 1)) && index % map_width != (map_width -1)) retPoints.push_back(indexToGrid(gridValues, (index + (map_width + 1))));
    
    //return the return array
    return retPoints;
}

/**
 * @brief derives a path from a list of coordinates 
 * 
 */
void wavefront(void)
{

}