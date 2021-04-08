#include <cspace.h>
#include <geometry_msgs/Point.h>

using geometry_msgs::Point;
using nav_msgs::OccupancyGrid;

std::vector<int> get_neighbors_indicies(int pt, int widthOfGrid, int sizeOfGrid)
{ // maybe use static array instead
	std::vector<int> neighbors;

	if (!((pt + 1) < 0) || !((pt + 1) > sizeOfGrid))
		neighbors.push_back(pt + 1);
	if (!((pt - 1) < 0) || !((pt - 1) > sizeOfGrid))
		neighbors.push_back(pt - 1);
	if (!((pt + widthOfGrid) < 0) || !((pt + widthOfGrid) > sizeOfGrid))
		neighbors.push_back(pt + widthOfGrid);
	if (!((pt + widthOfGrid + 1) < 0) || !((pt + widthOfGrid + 1) > sizeOfGrid))
		neighbors.push_back(pt + widthOfGrid + 1);
	if (!((pt + widthOfGrid - 1) < 0) || !((pt + widthOfGrid - 1) > sizeOfGrid))
		neighbors.push_back(pt + widthOfGrid - 1);
	if (!((pt - widthOfGrid) < 0) || !((pt - widthOfGrid) > sizeOfGrid))
		neighbors.push_back(pt - widthOfGrid);
	if (!((pt - widthOfGrid + 1) < 0) || !((pt - widthOfGrid + 1) > sizeOfGrid))
		neighbors.push_back(pt - widthOfGrid + 1);
	if (!((pt - widthOfGrid - 1) < 0) || !((pt - widthOfGrid - 1) > sizeOfGrid))
		neighbors.push_back(pt - widthOfGrid - 1);

	return neighbors;
}

// void recursive_search_neighbors(int pt, int depth, int threshold, OccupancyGrid* editGrid, const OccupancyGrid* searchGrid) {
// 	if(depth == 0) return;
// 	auto neighbors = get_neighbors_indicies(searchGrid->data[pt], editGrid->info.width, editGrid->data.size());
// 	for(int i = 0; i < 8; ++i) {
// 		editGrid->data[neighbors[i]] = 100;
// 		recursive_search_neighbors(neighbors[i], depth-1, threshold, editGrid, searchGrid);
// 	}
// }

OccupancyGrid CSpace::GetCSpace(const nav_msgs::OccupancyGrid &oGrid, int threshold, int radius)
{

	OccupancyGrid paddedGrid = oGrid;

	for (int i = 0; i < oGrid.data.size(); ++i)
	{
		if (oGrid.data[i] > threshold)
		{
			auto neighbors = get_neighbors_indicies(i, oGrid.info.width, oGrid.data.size());
			for (int n : neighbors)
			{
				paddedGrid.data[n] = 100;
			}
		}
	}

	return paddedGrid;
}