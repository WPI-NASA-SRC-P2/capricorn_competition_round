#pragma once

#include <nav_msgs/OccupancyGrid.h>

class CSpace
{
private:
	/**
	 * @brief Get the neighbors indicies of a point in the occupancy Grid
	 * 
	 * @param pt the index that you want to find the neighbors of
	 * @param widthOfGrid width of the map 
	 * @param sizeOfGrid total number of elements in the map
	 * @return std::vector<int> the indexes of the neighbors of pt
	 */
	static std::array<int, 8> getNeighborsIndiciesArray(int pt, int widthOfGrid, int sizeOfGrid);

	/**
	 * @brief recursivly searches for obstacles and edits its neighbors to also be obstacles
	 * 
	 * @param pt the inital point for which the neighbor are around 
	 * @param radius the radius of the search area
	 * @param threshold the value at which a cell is considered an obstacle
	 * @param editGrid the edited searchGrid
	 * @param searchGrid the original grid passed to the funciton
	 */
	static void recursiveSearchNeighbors(int pt, int radius, int threshold, nav_msgs::OccupancyGrid &editGrid, nav_msgs::OccupancyGrid &searchGrid);

public:
	/**
	 * @brief This will return a modified oGrid to include the CSpace
	 * 
	 * @param oGrid The Occupancy Grid that you want to modify
	 * @param threshold The threshold to classify a point as an obstacle 
	 * @param radius the radius in cell units of the CSpace
	 * @return OccupancyGrid The Occupancy Grid with the CSpace included
	 */
	static nav_msgs::OccupancyGrid getCSpace(nav_msgs::OccupancyGrid &grid, int threshold, int paddingRadius);
};
