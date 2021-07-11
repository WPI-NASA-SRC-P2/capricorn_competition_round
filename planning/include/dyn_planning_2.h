#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <perception/ObjectArray.h>
#include <perception/Object.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <unordered_map>
#include <math.h>

class DynamicPlanning2
{
private:
    
public:

/**
 * @brief 
 * 
 * @param wp1 
 * @param wp2 
 * @return unordered_map<char, float> 
 */
 static std::unordered_map<std::string, float> LineEquation(geometry_msgs::Point wp1, geometry_msgs::Point wp2);
 
 
 /**
  * @brief 
  * 
  * @param obstacleCentroids 
  * @return float 
  */
 static float ObstacleRadius(float width);
 
 /**
  * @brief 
  * 
  * @param parameters 
  * @param centroidOfObstacle 
  * @return float 
  */
 static float PerpendicularDistance(std::unordered_map<std::string, float> parameters, geometry_msgs::Point centroidOfObstacle);


/**
 * @brief 
 * 
 * @param obstacles 
 * @param path 
 * @return true 
 * @return false 
 */
bool checkAllObstacles(perception::ObjectArray obstacles, nav_msgs::Path path);


};
