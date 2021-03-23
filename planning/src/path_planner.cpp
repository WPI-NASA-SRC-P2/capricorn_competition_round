#include <ros/ros.h>
#include <templates/template_class.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <math.h>


PathPlanner::PathPlanner())
{

}

PathPlanner::~PathPlanner()
{

}

/**
 * @brief converts index of a grid into x and y coordinates
 * 
 */
std::vector<nav_msgs::GridCells> PathPlanner::indexToGrid(std::vector<nav_msgs::OccupancyGrid> indexValues)   //get rid of list here; included in message
{
    //create a list of arrays of x and y 
    //declare a return array
    std::vector<nav_msgs::GridCells> gridCells;

    //get the width and height of the map from the OccupancyGrid message 
    int map_width = indexValues.info.width;
    int map_height = indexValues.info.height;

    //iterate thru the array and convert each occupancy grid index to a grid cell message
    for(int i = 0; i < indexValues.size(); i++){
        //create a new grid cell message to add data to
        nav_msgs::GridCells newCell;


        //indexValues.data[0] gives list of arrays     
        //populate the fields
        newCell[i].cells.x= i % map_width + 1;
        newCell[i].cells.y = ceil(i/map_width);
        newCell[i].cells.z = data[i];

        //add the new grid cell to the return array
        gridCells.pushback(newCell);
    }

    //return the array of grid cells
    return gridCells;

}

/**
 * @brief converts x and y coordinates of a grid cell to its index
 * 
 */
void PathPlanner::gridToIndex(nav_msg msg, uint width, uint height)
{
    
    //nav_msgs/OccupancyGrip.msg.info.width;
    //int32 height = msgs.OccupancyGrip.msg.info.height;
    //int32 width =  msg.OccupancyGrip.msg.info.width;
    //int8[] data = msg.OccupancyGrip.msg.data;
    //return (x,y);
}

/**
 * @brief converts grid cell into list of walkable neightbors (full adjacent neightbors only)
 * 
 */
void PathPlanner::neightborsOf4(nav_msgs::GridCell> coordinate, std::vector<nav_msgs::GridCell> listOfCoordinates)   //pass in occupancy grid
{

    //just add and subract x and ys
    //declare a return array 
    std::vector<nav_msgs::GridCell> retList;

    //iterate thru the passed in array and check for neighbors; add each neightbor ro the return array 
    for(int i =0; i < listOfCoordinates.size; i++){
        if(abs(coordinate.cell_width - listOfCoordinates[i].cell_width) == 1 && coordinate.cell_height == i.cell_height) retList.pushback(listOfCoordinates[i]);
        else if(abs(coordinate.cell_height - i.cell_height) == 1 && coordinate.cell_width == i.cell_width) retList.pushback(listOfCoordinates[i]);
    }    

    //return the return array
    return retList;
}

/**
 * @brief converts grid cell into list of walkable neightbors (adjacent and diagonal)
 * 
 */
void PathPlanner::neightborsOf8(nav_msgs::GridCell coordinate, std::vector<nav_msgs::GridCell> listOfCoordinates)
{
    //declare a return array 
    std::vector<nav_msgs::GridCell> retList;

    
    //add the neightbors of 4 to the return array
    neighborsOf4(coordinate, listOfCoordinates);

    //check for diagonal neightbors; add to the list
    for(int i =0; i < listOfCoodinates.size; i++){
        if(abs(coordinate.cell_width - i.cell_width) == 1 && abs(coordinate.cell_height - i.cell_height) == 1) retList.pushback(i);
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