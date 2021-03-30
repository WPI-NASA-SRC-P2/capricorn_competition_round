#pragma once

#include <nav_msgs/OccupancyGrid.h>

class CSpace {
public:
	static std::vector<int> get_neighbors_indicies(int pt, int widthOfGrid, int sizeOfGrid);
	static nav_msgs::OccupancyGrid GetCSpace(const nav_msgs::OccupancyGrid& grid, int threshold, int paddingRadius);
};
