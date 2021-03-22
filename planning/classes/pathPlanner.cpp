#include <ros/ros.h>

#include "../include/pathPlanner.h"


pathPlanner::pathPlanner()
{
}

pathPlanner::~pathPlanner()
{
}

std::string pathPlanner::getTeamName()
{
    // returns the private variable 
    return team_name;
}

void pathPlanner::setTeamName(const std::string& input_string)
{
    // team_name is assigned the value of Input string
    // Doing this, you keep the private variable safe 
    team_name = input_string;
}

// int grid_to_index(std::vector <pathPlanner::cord> gridArray, pathPlanner::cord cordinate) {
//     for (int i = 0; gridArray.size(); i++) {
//         if (gridArray.at(i) == cordinate) {
//             return i;
//         }
//     }

//     return -1;
// }

// pathPlanner::cord index_to_grid(std::vector <pathPlanner::cord> grid, int index){
//     return grid.at(index);
// }



// std::vector <pathPlanner::cord> eightNeighbors(std::vector <pathPlanner::cord> grid, pathPlanner::cord cord, int width){
//     int cordIndex = grid_to_index(grid, cord);
//     std::vector <pathPlanner::cord> neighbors;
//     neighbors.push_back(index_to_grid(grid, cordIndex + 1));
//     neighbors.push_back(index_to_grid(grid, cordIndex - 1));
//     neighbors.push_back(index_to_grid(grid, cordIndex - width + 1));
//     neighbors.push_back(index_to_grid(grid, cordIndex - width));
//     neighbors.push_back(index_to_grid(grid, cordIndex - width - 1));
//     neighbors.push_back(index_to_grid(grid, cordIndex + width + 1));
//     neighbors.push_back(index_to_grid(grid, cordIndex + width - 1));
//     neighbors.push_back(index_to_grid(grid, cordIndex + width));
//     return neighbors;
// }

std::vector <int> pathPlanner::cSpace(std::vector <pathPlanner::cord> grid, int width, std::vector <int> probibility, int threshold, int radius) 
{
    std::vector <int> newProbibility = probibility;
    
    for(int i = 0; i < probibility.size(); i++) {
        
        if (probibility.at(i) > threshold) 
        {
            std::vector <pathPlanner::cord> origin;
            origin.insert(origin.begin(), grid.at(i));
            std::vector <pathPlanner::cord> neighbors = findAllNeighbors(grid, width, origin, radius);
            
            int index = 0;
            for(int j  = 0; j < neighbors.size(); j++) 
            {
                for (int i = 0; i < grid.size(); i++) 
                {
                    if (grid.at(i) == neighbors.at(j)) 
                    {
                        index = i;
                    }
                }
                newProbibility.at(index) = 100; // there wasa a grid_to_index here
            }
        }
        


    }
    return newProbibility;

}

std::vector <pathPlanner::cord> pathPlanner::findAllNeighbors(std::vector <pathPlanner::cord> grid, int width, std::vector <pathPlanner::cord> toSearch, int r)
{
    
    std::vector <pathPlanner::cord> toAdd;
    for(int i = 0; i < toSearch.size(); i++)
    {
        std::vector <pathPlanner::cord> neighbors;// =  eightNeighbors(grid, toSearch.at(i),width);
        
        // there was an eightNeighbors here
        int cordIndex  = 0;
        // there was a grid_to_index here
        for (int j = 0; j < grid.size(); j++) 
        {
            // printf("%i, %i", grid.at(i).x, grid.at(i).y);
            if (grid.at(j) == toSearch.at(i)) 
            {
                cordIndex = j;
            }
        }
        // printf("finding Neighbors at (%i, %i), cordIndex of %i\n", toSearch.at(i).x, toSearch.at(i).y, cordIndex);
        // changed index_to_grid to grid.at(cordIndex + ...)
        //printf("adding %i, %i ,%i, %i ,%i, %i ,%i, %i \n",cordIndex + 1, cordIndex - 1, cordIndex - width + 1, cordIndex - width, cordIndex - width - 1, cordIndex + width + 1 , cordIndex + width - 1, cordIndex + width );
        neighbors.push_back(grid.at(cordIndex + 1));
        neighbors.push_back(grid.at(cordIndex - 1));
        neighbors.push_back(grid.at(cordIndex - width + 1));
        // printf("(%i, %i)\n", grid.at(cordIndex - width).x, grid.at(cordIndex - width).y);
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
        
        std::vector <pathPlanner::cord> recusiveAdd = findAllNeighbors(grid, width, toAdd, r);

        toAdd.insert(toAdd.end(), recusiveAdd.begin(), recusiveAdd.end());
    }
    return toAdd;
  }


/**
 * @brief main function for quick-testing of the class
 * 
 */
int main(int argc, char *argv[])
{
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
   std::vector <pathPlanner::cord> grid;
   std::vector <int> probibility;
   for (int i = 1; i <= 5; i++) 
   {
    for (int j = 1; j <= 5; j++) 
    {
        pathPlanner::cord currentCord;
        currentCord.x = i;
        currentCord.y = j;
        grid.insert(grid.end(), currentCord);

        if (i == 3 && j == 3) 
        {
            probibility.push_back(100);
        } else 
        {
            probibility.push_back(0);
        }

    }
    
   }

    for(int i = 0; i < 5; i++){
        for(int j = 0; j < 5; j++) 
        {
            printf("(%i, %i)\t", grid.at((i *5) + j).x, grid.at((i*5) + j).y);
        }
        printf("\n");
    }
    printf("\n");

    for(int i = 0; i < 5; i++)
    {
        for(int j = 0; j < 5; j++) 
        {
            printf("(%i)\t", probibility.at((i * 5) + j));
        }
        printf("\n");
    }
    pathPlanner* testpathPlanner = new pathPlanner();
    printf("\n");
    std::vector<int> cSpaceOut = testpathPlanner->cSpace(grid, 5, probibility, 20, 1);

    for(int i = 0; i < 5; i++)
    {
        for(int j = 0; j < 5; j++) 
        {
            printf("(%i)\t", cSpaceOut.at((i *5) + j));
        }
        printf("\n");
    }



    return 0;
}
