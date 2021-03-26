#include <ros/ros.h>
#include <templates/template_class.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <math.h>


PathPlanner::PathPlanner()
{

}

PathPlanner::~PathPlanner()
{

}

/**
 * @brief prints data from the occupancy grid 
 */
void PathPlanner::PrintMessage(nav_msgs::OccupancyGrid OccGrid)
{
    printf(OccGrid.data);
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
    
    retCoordinate.x = (index % map_width + 1) * gridValues.info.resolution;
    retCoordinate.y = ceil((float)index / map_width) * gridValues.info.resolution;

    //return the array of grid cells
    return retCoordinate;
}

/**
 * @brief converts x and y coordinates of a grid cell to its index
 * 
 */
int PathPlanner::gridToIndex(nav_msgs::OccupancyGrid gridValues, geometry_msgs::Point coordinate)
{
    return coordinate.x / gridValues.info.resolution + coordinate.y * gridValues.info.resolution * gridValues.info.width;
}

/**
 * @brief converts grid cell into list of walkable neightbors (full adjacent neightbors only)
 * 
 */
std::vector<geometry_msgs::Point> PathPlanner::neightborsOf4(geometry_msgs::Point coordinate, nav_msgs::OccupancyGrid gridValues) 
{
    //just add and subract x and ys
    //use gridToIndex
    //add one, subtract one, add width, subtract width to get neighbors
    //check bounds
    //declare a return array 
    std::vector<geometry_msgs::Point> retPoints;

    int index = gridToIndex(gridValues, coordinate);

    if(index % gridValues.info.width != 1)
    {
        retPoints.pushback(index - 1);
    }
    if(index % gridValues.info.width != 0)
    {
        retPoints.pushback(index + 1);
    }
    if(index > gridValues.info.width)
    {
        retPoints.pushback(index - gridValues.info.width);
    }
    if(index < ((gridValues.info.width * gridValues.info.height) - gridValues.info.width))
    {
        retPoints.pushback(index + gridValues.info.width);
    }

    //return the return array
    return retList;
}

/**
 * @brief converts grid cell into list of walkable neightbors (adjacent and diagonal)
 * 
 */
std::vector<geometry_msgs::Point> PathPlanner::neightborsOf8(nav_msgs::GridCell coordinate, nav_msgs::OccupancyGrid gridValues)
{
    //declare a return array 
    std::vector<geometry_msgs::Points> retPoints;

    //add the neightbors of 4 to the return array
    std::vector<geometry_msgs::Points> existingNeighbors = neighborsOf4(coordinatem, gridValues);

    for(int i  = 0; i < existingNeighbors.size(); i++)
    {
        retPoints.pushback(existingNeightbors[i]);
    }

    if(index > gridValues.info.width + 1 && index % gridValues.info.width != 1)
    {
        retPoints.pushback(index - (gridValues.info.width + 1));
    }
    if(index > gridValues.info.width && index % gridValues.info.width != 0)
    {
        retPoints.pushback(index - (gridValues.info.width + 1));
    }
    if(index < ((gridValues.info.width * gridVaues.info.height) - gridValues.info.width) && index % gridValues.info.width != 1)
    {
        retPoints.pushback(index + (gridValues.info.width -1));
    }
    if(index < ((gridValues.info.width * gridVaues.info.height) - (gridValues.info.width + 1)) && index % gridValues.info.width != 0)
    {
        retPoints.pushback(index + gridValues.info.width + 1);
    }
    
    //return the return array
    return retList;
}

/**
 * @brief derives a path from a list of coordinates 
 * 
 */
void wavefront(void)
{

}