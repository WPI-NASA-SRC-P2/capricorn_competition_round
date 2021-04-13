#include <ros/ros.h>
#include <cspace.h>
#include <geometry_msgs/Point.h>

using geometry_msgs::Point;
using nav_msgs::OccupancyGrid;



std::vector<int> CSpace::getNeighborsIndicies(int pt, int widthOfGrid, int sizeOfGrid)
{ 

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