#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <path_planner.h>


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
   // printf(OccGrid.data);
}

/**
 * @brief test function
 */
void PathPlanner::test(const nav_msgs::OccupancyGrid& gridValues)
{
    geometry_msgs::Point coordinate = indexToGrid(gridValues, 4);
    int index = gridToIndex(gridValues, coordinate);

    ROS_INFO("%i", index);
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
    //declare a return array 
    std::vector<geometry_msgs::Point> retPoints;

    //find the index from the given coordinate and grid
    int index = gridToIndex(gridValues, coordinate);

    int map_width = gridValues.info.width;
    int map_height = gridValues.info.height;
    int map_area = gridValues.info.width * gridValues.info.height;

    if(index % map_width != 1)
    {
        retPoints.push_back(indexToGrid(gridValues, (index - 1)));
    }
    if(index % map_width != 0)
    {
        retPoints.push_back(indexToGrid(gridValues, (index + 1)));
    }
    if(index > map_width)
    {
        retPoints.push_back(indexToGrid(gridValues, (index - map_width)));
    }
    if(index < (map_area - map_width))
    {
        retPoints.push_back(indexToGrid(gridValues, (index + map_width)));
    }

    //return the return array
    return retPoints;
}

/**
 * @brief converts grid cell into list of walkable neightbors (adjacent and diagonal)
 * 
 */
std::vector<geometry_msgs::Point> PathPlanner::neightborsOf8(geometry_msgs::Point coordinate, nav_msgs::OccupancyGrid gridValues)
{
    //declare a return array 
    std::vector<geometry_msgs::Point> retPoints;

    //add the neightbors of 4 to the return array
    std::vector<geometry_msgs::Point> existingNeighbors = neightborsOf4(coordinate, gridValues);

    for(int i  = 0; i < existingNeighbors.size(); i++)
    {
        retPoints.push_back(existingNeighbors[i]);
    }

    //find the index from the given coordinate and grid
    int index = gridToIndex(gridValues, coordinate);

    int map_width = gridValues.info.width;
    int map_height = gridValues.info.height;
    int map_area = gridValues.info.width * gridValues.info.height;
    

    if(index > map_width + 1 && index % map_width != 1)
    {
        retPoints.push_back(indexToGrid(gridValues, (index - (map_width + 1))));
    }
    if(index > map_width && index % map_width != 0)
    {
        retPoints.push_back(indexToGrid(gridValues, (index - (map_width + 1))));
    }
    if(index < (map_area - map_width) && index % map_width != 1)
    {
        retPoints.push_back(indexToGrid(gridValues, (index + (map_width -1))));
    }
    if(index < ((map_area) - (map_width + 1)) && index % map_width != 0)
    {
        retPoints.push_back(indexToGrid(gridValues, (index + (map_width + 1))));
    }
    
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