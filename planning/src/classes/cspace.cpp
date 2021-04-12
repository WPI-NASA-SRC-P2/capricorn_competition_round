#include <ros/ros.h>
#include <cspace.h>
#include <geometry_msgs/Point.h>

using geometry_msgs::Point;
using nav_msgs::OccupancyGrid;

/**
 * @brief Get the neighbors indicies of a point in the occupancy Grid
 * 
 * @param pt the index that you want to find the neighbors of
 * @param widthOfGrid width of the map 
 * @param sizeOfGrid total number of elements in the map
 * @return std::vector<int> the indexes of the neighbors of pt
 */

std::vector<int> CSpace::getNeighborsIndicies(int pt, int widthOfGrid, int sizeOfGrid)
{ // maybe use static array instead

	std::vector<int> neighbors;

	if (!((pt + 1) < 0) && !((pt + 1) > sizeOfGrid))
	{
		neighbors.push_back(pt + 1);
	}
	if (!((pt - 1) < 0) && !((pt - 1) > sizeOfGrid))
	{
		neighbors.push_back(pt - 1);
	}
	if (!((pt + widthOfGrid) < 0) && !((pt + widthOfGrid) > sizeOfGrid))
	{
		neighbors.push_back(pt + widthOfGrid);
	}
	if (!((pt + widthOfGrid + 1) < 0) && !((pt + widthOfGrid + 1) > sizeOfGrid))
	{
		neighbors.push_back(pt + widthOfGrid + 1);
	}
	if (!((pt + widthOfGrid - 1) < 0) && !((pt + widthOfGrid - 1) > sizeOfGrid))
	{
		neighbors.push_back(pt + widthOfGrid - 1);
	}
	if (!((pt - widthOfGrid) < 0) && !((pt - widthOfGrid) > sizeOfGrid))
	{
		neighbors.push_back(pt - widthOfGrid);
	}
	if (!((pt - widthOfGrid + 1) < 0) && !((pt - widthOfGrid + 1) > sizeOfGrid))
	{
		neighbors.push_back(pt - widthOfGrid + 1);
	}
	if (!((pt - widthOfGrid - 1) < 0) && !((pt - widthOfGrid - 1) > sizeOfGrid))
	{
		neighbors.push_back(pt - widthOfGrid - 1);
	}

	return neighbors;
}

/**
 * @brief recursivly searches for obstacles and edits its neighbors to also be obstacles
 * 
 * @param pt the inital point for which the neighbor are around 
 * @param radius the radius of the search area
 * @param threshold the value at which a cell is considered an obstacle
 * @param editGrid the edited searchGrid
 * @param searchGrid the original grid passed to the funciton
 */
void CSpace::recursiveSearchNeighbors(int pt, int radius, int threshold, OccupancyGrid *editGrid, const OccupancyGrid *searchGrid)
{
	if (radius > 0)
	{
		auto neighbors = getNeighborsIndicies(pt, editGrid->info.width, editGrid->data.size());
		for (int i = 0; i < neighbors.size(); ++i)
		{
			editGrid->data[neighbors[i]] = 100;
			recursiveSearchNeighbors(neighbors[i], radius - 1, threshold, editGrid, searchGrid);
		}
	}
}

/**
 * @brief This will return a modified oGrid to include the CSpace
 * 
 * @param oGrid The Occupancy Grid that you want to modify
 * @param threshold The threshold to classify a point as an obstacle 
 * @param radius the radius in cell units of the CSpace
 * @return OccupancyGrid The Occupancy Grid with the CSpace included
 */
OccupancyGrid CSpace::getCSpace(const nav_msgs::OccupancyGrid &oGrid, int threshold, int radius)
{

	OccupancyGrid paddedGrid = oGrid;

	for (int i = 0; i < oGrid.data.size(); ++i)
	{
		if (oGrid.data[i] > threshold)
		{
			recursiveSearchNeighbors(i, radius, threshold, &paddedGrid, &oGrid);
		}
	}

	return paddedGrid;
}