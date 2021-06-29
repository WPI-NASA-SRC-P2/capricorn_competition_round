#include <ros/ros.h>
#include <cspace.h>
#include <geometry_msgs/Point.h>

using geometry_msgs::Point;
using nav_msgs::OccupancyGrid;

std::array<int, 8> CSpace::getNeighborsIndiciesArray(int pt, int widthOfGrid, int sizeOfGrid)
{
  // Get the neighbors around any given index (ternary operators to ensure the points remain within bounds)
  std::array<int, 8> neighbors;

  neighbors[0] = ((pt + 1) < 0)               || ((pt + 1) > sizeOfGrid) 								? -1 : pt + 1;
  neighbors[1] = ((pt - 1) < 0)               || ((pt - 1) > sizeOfGrid) 								? -1 : pt - 1;
  neighbors[2] = ((pt + widthOfGrid    ) < 0) || ((pt + widthOfGrid) > sizeOfGrid) 			? -1 : pt + widthOfGrid;
  neighbors[3] = ((pt + widthOfGrid + 1) < 0) || !((pt + widthOfGrid + 1) > sizeOfGrid) ? -1 : pt + widthOfGrid + 1;
  neighbors[4] = ((pt + widthOfGrid - 1) < 0) || !((pt + widthOfGrid - 1) > sizeOfGrid) ? -1 : pt + widthOfGrid - 1;
  neighbors[5] = ((pt - widthOfGrid) < 0)     || !((pt - widthOfGrid) > sizeOfGrid) 		? -1 : pt - widthOfGrid;
  neighbors[6] = ((pt - widthOfGrid + 1) < 0) || !((pt - widthOfGrid + 1) > sizeOfGrid) ? -1 : pt - widthOfGrid + 1;
  neighbors[7] = ((pt - widthOfGrid - 1) < 0) || !((pt - widthOfGrid - 1) > sizeOfGrid) ? -1 : pt - widthOfGrid - 1;

  return neighbors;
}

void CSpace::recursiveSearchNeighbors(const int pt, const int radius, const int threshold, OccupancyGrid &editGrid, OccupancyGrid &searchGrid)
{
	if (radius > 0)
	{
		auto neighbors = getNeighborsIndiciesArray(pt, editGrid.info.width, editGrid.data.size());
		for (int i = 0; i < neighbors.size(); ++i)
		{
			if(neighbors[i] == -1)
				continue;

			editGrid.data[neighbors[i]] = 100;
			recursiveSearchNeighbors(neighbors[i], radius - 1, threshold, editGrid, searchGrid);
		}
	}
}

nav_msgs::OccupancyGrid CSpace::getCSpace(nav_msgs::OccupancyGrid &oGrid, const int threshold, const int radius)
{

	OccupancyGrid paddedGrid = oGrid;

	for (int i = 0; i < oGrid.data.size(); ++i)
	{
		if (oGrid.data[i] > threshold)
		{
			recursiveSearchNeighbors(i, radius, threshold, paddedGrid, oGrid);
		}
		if (oGrid.data[i] == -1)
			paddedGrid.data[i] = 0;

	std::cout << ("end of for loop - 3\n");
		
	}
	//std::cout << ("out of for loop - 4\n");
	return paddedGrid;
}