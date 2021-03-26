#pragma once

#include <nav_msgs/OccupancyGrid.h>

class CSpace {
public:
	static nav_msgs::OccupancyGrid GetCSpace(const nav_msgs::OccupancyGrid& grid, int threshold, int paddingRadius);
};
