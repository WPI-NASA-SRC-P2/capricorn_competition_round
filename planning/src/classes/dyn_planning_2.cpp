#include "dyn_planning_2.h"



std::unordered_map<std::string, float> DynamicPlanning2::LineEquation(geometry_msgs::Point wp1, geometry_msgs::Point wp2)
{
    std::unordered_map<std::string, float> parameters;
    float deltaX = wp2.x - wp1.x;
    float deltaY = wp2.y - wp1.y;
    float m = deltaY/deltaX;
    parameters["a"]= -deltaY/deltaX;;
    parameters["b"] = 1;
    parameters["c"] = wp2.y - m*wp2.x;
    parameters["m"] = deltaY/deltaX;;
    
    return parameters;
}

float DynamicPlanning2::ObstacleRadius(float width)
{
    float radius = (width +2)/2;

    return radius;

}

float DynamicPlanning2::PerpendicularDistance(std::unordered_map<std::string, float> parameters, geometry_msgs::Point centroidOfObstacle)
{   
  float denom = sqrt(pow(parameters["m"], 2) + 1);  
  float distance = abs(-parameters["m"]*centroidOfObstacle.x + centroidOfObstacle.y - parameters["c"])/denom;

  return distance;
    
}

 

bool DynamicPlanning2::checkAllObstacles(perception::ObjectArray obstacles, nav_msgs::Path path)
{
    float min_dist = INFINITY;
    float dist;
    float radius;
    std::vector<std::unordered_map<std::string, float>> CompletePathParameters;
    for (int i = 0; i < path.poses.size() - 1; i++)
    {
        std::unordered_map<std::string, float> parameters = LineEquation(path.poses[i].pose.position, path.poses[i + 1].pose.position );
        CompletePathParameters.push_back(parameters); // should save the parameters of all the segments in path
    }

    for (int l = 0; l < CompletePathParameters.size(); l++ )
    {
        for (int j = 0; j < obstacles.number_of_objects; j++)
        {
            geometry_msgs::Point centroid;
            centroid.x = obstacles.obj[j].center.x;
            centroid.y = obstacles.obj[j].center.y;

            dist = PerpendicularDistance( CompletePathParameters[l], centroid);
            radius = ObstacleRadius(obstacles.obj[j].width);
            if(dist < radius )
            {
                return true;
            }
            else 
            {
                return false;
            }
        }
        
    }
    return false;
}


 
 