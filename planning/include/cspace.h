#pragma once

#include <nav_msgs/OccupancyGrid.h>

class CSpace
{
private:
	static std::vector<int> getNeighborsIndicies(int pt, int widthOfGrid, int sizeOfGrid);
	static void recursiveSearchNeighbors(int pt, int radius, int threshold, nav_msgs::OccupancyGrid *editGrid, const nav_msgs::OccupancyGrid *searchGrid);

public:
	static nav_msgs::OccupancyGrid getCSpace(const nav_msgs::OccupancyGrid &grid, int threshold, int paddingRadius);
};
