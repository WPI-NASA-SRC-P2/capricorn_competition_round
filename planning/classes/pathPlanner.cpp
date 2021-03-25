#include <ros/ros.h>

#include "../include/pathPlanner.h"







static std::vector <int8> pathPlanner::cSpace(std::vector <geometry_msgs::Point>& grid, int width, std::vector <int8>& probability, int threshold, int radius) {
	std::vector <int8> newProbability = probability;
	
	for(int i = 0; i < probability.size(); i++) {
		
		if (probability.at(i) > threshold) {
			std::vector <geometry_msgs::Point> origin;
			origin.insert(origin.begin(), grid.at(i));
			std::vector <geometry_msgs::Point> neighbors = findAllNeighbors(grid, width, origin, radius);
			
			int index = 0;
			for(int j  = 0; j < neighbors.size(); j++) {
				for (int i = 0; i < grid.size(); i++) {
					if (grid.at(i) == neighbors.at(j)) {
						index = i;
					}
				}
				newProbability.at(index) = 100; // there was a a grid_to_index here
			}
		}
		


	}
	return newProbability;

}

static std::vector <geometry_msgs::Point> pathPlanner::findAllNeighbors(std::vector <geometry_msgs::Point>& grid, int width, std::vector <geometry_msgs::Point>& toSearch, int r)
{
	
	std::vector <geometry_msgs::Point> toAdd;
	for(int i = 0; i < toSearch.size(); i++){
		std::vector <geometry_msgs::Point> neighbors;// =  eightNeighbors(grid, toSearch.at(i),width);
		
		// there was an eightNeighbors here
		int cordIndex  = 0;
		// there was a grid_to_index here
		for (int j = 0; j < grid.size(); j++) {
			// printf("%i, %i", grid.at(i).x, grid.at(i).y);
			if (grid.at(j) == toSearch.at(i)) {
				cordIndex = j;
			}
		}
		
		neighbors.push_back(grid.at(cordIndex + 1));
		neighbors.push_back(grid.at(cordIndex - 1));
		neighbors.push_back(grid.at(cordIndex - width + 1));
		neighbors.push_back(grid.at(cordIndex - width));
		neighbors.push_back(grid.at(cordIndex - width - 1));
		neighbors.push_back(grid.at(cordIndex + width + 1));
		neighbors.push_back(grid.at(cordIndex + width - 1));
		neighbors.push_back(grid.at(cordIndex + width)); 
		

		toAdd.insert(toAdd.end(), neighbors.begin(), neighbors.end());
	}
	// sort(toAdd.begin(),toAdd.end());
	// toAdd.erase(std::unique(toAdd.begin(), toAdd.end()), toAdd.end());
	

	// sort(toAdd.begin(),toAdd.end());
	// toAdd.erase(unique(toSearch.begin(), toSearch.end()), toAdd.end());
	r--;
	if (r > 0) 
	{
		
		std::vector <geometry_msgs::Point> recusiveAdd = findAllNeighbors(grid, width, toAdd, r);

		toAdd.insert(toAdd.end(), recusiveAdd.begin(), recusiveAdd.end());
	}
	return toAdd;
  }


/**
 * @brief main function for quick-testing of the class
 * 
 */
// int main(int argc, char *argv[])
// {
	// ROS initialization
	// ros::init(argc, argv, "template_class_tester");
	// ros::NodeHandle nh;

	// // Creted an object of the class
	// pathPlanner template_class;

	// // Setting the team name
	// std::string temp_string = "Team Capricorn";
	// template_class.setTeamName(temp_string);
	
	// // Getting the team name
	// std::string team_name = template_class.getTeamName();
	
	// // Printing the team name
	// ROS_INFO_STREAM("Team name from class object is: "<<team_name);


	/*
		5x5 gird
		(3, 3) at 100
		run cspace(r = 1, width = 5, threashold = 20)
	*/
//    std::vector <geometry_msgs::Point> grid;
//    std::vector <int> probability;
//    for (int i = 1; i <= 5; i++) 
//    {
//     for (int j = 1; j <= 5; j++) 
//     {
//         geometry_msgs::Point currentCord;
//         currentCord.x = i;
//         currentCord.y = j;
//         grid.insert(grid.end(), currentCord);

//         if (i == 3 && j == 3) {
//             probability.push_back(100);
//         } else {
//             probability.push_back(0);
//         }

//     }
	
//    }

	// for(int i = 0; i < 5; i++){
	//     for(int j = 0; j < 5; j++) 
	//     {
	//         printf("(%i, %i)\t", grid.at((i *5) + j).x, grid.at((i*5) + j).y);
	//     }
	//     printf("\n");
	// }
	// printf("\n");

	// for(int i = 0; i < 5; i++)
	// {
	//     for(int j = 0; j < 5; j++) 
	//     {
	//         printf("(%i)\t", probability.at((i * 5) + j));
	//     }
	//     printf("\n");
	// }
	// pathPlanner* testpathPlanner = new pathPlanner();
	// printf("\n");
	// std::vector<int> cSpaceOut = testpathPlanner->cSpace(grid, 5, probability, 20, 1);

	// for(int i = 0; i < 5; i++)
	// {
	//     for(int j = 0; j < 5; j++) 
	//     {
	//         printf("(%i)\t", cSpaceOut.at((i *5) + j));
	//     }
	//     printf("\n");
	// }



// 	return 0;
// }
