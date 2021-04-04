#include <ros/ros.h>
#include <cspace.h>
#include <geometry_msgs/Point.h>

using nav_msgs::OccupancyGrid;
using geometry_msgs::Point;

std::vector<int> get_neighbors_indicies(int pt, int widthOfGrid, int sizeOfGrid) { // maybe use static array instead
    std::vector<int> neighbors;

    if(!((pt + 1) < 0) 				 && !((pt + 1) > sizeOfGrid)) {       		
		neighbors.push_back(pt + 1);
		// ROS_INFO("GOT N 1");	
	}
	if(!((pt - 1) < 0) 				 && !((pt - 1) > sizeOfGrid)) {
		neighbors.push_back(pt - 1);
		// ROS_INFO ("GOT N 2");
		}
	if(!((pt + widthOfGrid) < 0)     && !((pt + widthOfGrid) > sizeOfGrid)) 	{
		neighbors.push_back(pt + widthOfGrid);
		// ROS_INFO ("GOT N 3");
		}
	if(!((pt + widthOfGrid + 1) < 0) && !((pt + widthOfGrid + 1) > sizeOfGrid)) {
		neighbors.push_back(pt + widthOfGrid + 1);
		// ROS_INFO ("GOT N 4");
		}
	if(!((pt + widthOfGrid - 1) < 0) && !((pt + widthOfGrid - 1) > sizeOfGrid)) {
		neighbors.push_back(pt + widthOfGrid - 1);
		// ROS_INFO ("GOT N 5");
		}
	if(!((pt - widthOfGrid) < 0)     && !((pt - widthOfGrid) > sizeOfGrid)) 	{
		neighbors.push_back(pt - widthOfGrid);
		// ROS_INFO ("GOT N 6");
	} else {
		// ROS_INFO("POINT %i, WIDTH %i, SIZE %i", pt, widthOfGrid, sizeOfGrid);
	}
	if(!((pt - widthOfGrid + 1) < 0) && !((pt - widthOfGrid + 1) > sizeOfGrid)) {
		neighbors.push_back(pt - widthOfGrid + 1);
		// ROS_INFO ("GOT N 7");
		}
	if(!((pt - widthOfGrid - 1) < 0) && !((pt - widthOfGrid - 1) > sizeOfGrid)) {
		neighbors.push_back(pt - widthOfGrid - 1);
		// ROS_INFO ("GOT N 8");
		}

    return neighbors;
}

void recursive_search_neighbors(int pt, int radius, int threshold, OccupancyGrid* editGrid, const OccupancyGrid* searchGrid) {
	if(radius > 0) {
		// ROS_INFO("Searching at radius of %i", radius );
		auto neighbors = get_neighbors_indicies(pt, editGrid->info.width, editGrid->data.size());
		// ROS_INFO("got the neighbors, length is %li", neighbors.size());
		// ROS_INFO("data width %i", editGrid->info.width);
		// ROS_INFO("data size %li", editGrid->data.size());
		for(int i = 0; i < neighbors.size(); ++i) {
			editGrid->data[neighbors[i]] = 100;
			// ROS_INFO("edited data at %i", neighbors[i]);
			recursive_search_neighbors(neighbors[i], radius-1, threshold, editGrid, searchGrid);
		}
	}
	
}

OccupancyGrid CSpace::GetCSpace(const nav_msgs::OccupancyGrid& oGrid, int threshold, int radius) {

	OccupancyGrid paddedGrid = oGrid;

	for(int i = 0; i < oGrid.data.size(); ++i) {
		if(oGrid.data[i] > threshold) {
			//auto neighbors = get_neighbors_indicies(i, oGrid.info.width, oGrid.data.size());
			// ROS_INFO("found at index %i", i);
			recursive_search_neighbors(i, radius, threshold, &paddedGrid, &oGrid);
		}
	}

	return paddedGrid;
}